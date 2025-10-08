/// @file robot_parser.cpp
/// @brief Parses the robot's URDF to publish PF obstacles (real robot and planning robot)
/// @author Sharwin Patil
/// @date   October 6, 2025
///
/// PARAMETERS:
///   robot_geometry_update_frequency (float64) : Timer frequency for regenerating obstacle arrays
///   robot_description (string): URDF XML string (or already-resolved value) for the live robot
///   fixed_frame (string): Global/world frame used for TF queries and obstacle frames
///   planning_tf_prefix (string): Prefix applied to planning TF frames / generated planning URDF
///   use_jsp (bool): Flag indicating if a Joint State Publisher GUI is expected

/// SUBSCRIBERS:
///   (none) – This node is passive with respect to subscriptions; it relies on TF and parameters.
///
/// PUBLISHERS:
///   ~/pfield/obstacles (potential_fields_interfaces::msg::ObstacleArray) Collision geometry obstacles derived
///                                                                        from the live robot's URDF & current TF transforms.
///   ~/pfield/planning_obstacles (potential_fields_interfaces::msg::ObstacleArray) Collision geometry obstacles derived from the
///                                                                                 planning duplicate robot.
///   ~/pfield/planning_robot_description (std_msgs::msg::String) The planning-modified URDF for downstream consumers

#include "ros/robot_parser.hpp"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <regex>


RobotParser::RobotParser() : Node("robot_parser") {
  RCLCPP_INFO(this->get_logger(), "RobotParser Initialized");

  // Declare parameters
  this->robotUpdateFrequency = this->declare_parameter("robot_geometry_update_frequency", 50.0f); // [Hz]
  this->robotDescription = this->declare_parameter("robot_description", "urdf/robot.urdf");
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->planningTFPrefix = this->declare_parameter("planning_tf_prefix", std::string("planning"));
  this->useJSPGui = this->declare_parameter("use_jsp", true);
  // Get parameters from yaml file
  this->robotUpdateFrequency = this->get_parameter("robot_geometry_update_frequency").as_double();
  this->robotDescription = this->get_parameter("robot_description").as_string();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->planningTFPrefix = this->get_parameter("planning_tf_prefix").as_string();
  this->useJSPGui = this->get_parameter("use_jsp").as_bool();

  // Setup obstacle publisher (smaller queue reduces latency/buffering)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();
  this->obstaclePub = this->create_publisher<ObstacleArray>("pfield/obstacles", qos);
  this->planningObstaclePub = this->create_publisher<ObstacleArray>("pfield/planning_obstacles", qos);
  this->planningRobotDescriptionPub = this->create_publisher<std_msgs::msg::String>("pfield/planning_robot_description", 1);

  // Setup TF2 broadcaster, buffer, and listener
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);
  RCLCPP_INFO(this->get_logger(), "TF2 broadcaster, buffer, and listener initialized");

  // Parse URDF once to build the collision catalog
  if (!this->robotModel.initString(this->robotDescription)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model");
    this->modelLoaded = false;
  }
  else {
    this->modelLoaded = true;
    // Publish the planning robot description (with modified collision, link, and joint names)
    const std::string planningRobotDesc = this->createPlanningRobotDescription(this->robotDescription);
    std_msgs::msg::String planningRobotDescMsg;
    planningRobotDescMsg.data = planningRobotDesc;
    this->planningRobotDescriptionPub->publish(planningRobotDescMsg);
    // Dump descriptions for inspection
    this->dumpRobotDescriptions(this->robotDescription, planningRobotDesc,
      "/tmp/original_robot_description.xml", "/tmp/planning_robot_description.xml");
    // After publishing Planning Robot Description, set the parameter for the planning_RSP
    this->planningRobotModel.initString(planningRobotDesc);
    this->collisionCatalog = this->buildCollisionCatalog(this->robotModel, false);
    this->planningCollisionCatalog = this->buildCollisionCatalog(this->planningRobotModel, true);
    RCLCPP_INFO(this->get_logger(), "URDF model loaded: %zu collision elements cached", this->collisionCatalog.size());
    // Emit a one-time detailed listing of expected collision-derived obstacles
    for (const auto& entry : this->collisionCatalog) {
      if (!entry.col || !entry.col->geometry) {
        RCLCPP_WARN(this->get_logger(), "Collision entry '%s' has no geometry", entry.id.c_str());
        continue;
      }
      std::string gType = "Unknown";
      if (dynamic_cast<urdf::Box*>(entry.col->geometry.get())) gType = "Box";
      else if (dynamic_cast<urdf::Sphere*>(entry.col->geometry.get())) gType = "Sphere";
      else if (dynamic_cast<urdf::Cylinder*>(entry.col->geometry.get())) gType = "Cylinder";
      else if (dynamic_cast<urdf::Mesh*>(entry.col->geometry.get())) gType = "Mesh";
      RCLCPP_INFO(this->get_logger(), "Catalog: id=%s link=%s type=%s", entry.id.c_str(), entry.linkName.c_str(), gType.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Robot has %zu links", this->robotModel.links_.size());
  }

  if (!this->modelLoaded) {
    RCLCPP_ERROR(this->get_logger(), "RobotParser failed to load URDF Model, shutting down node...");
    rclcpp::shutdown();
  }
  else {
    // Publish a static transform from robot's base frame to planning base frame as identity
    geometry_msgs::msg::TransformStamped staticTransform;
    staticTransform.header.stamp = this->get_clock()->now();
    staticTransform.header.frame_id = this->fixedFrame;
    staticTransform.child_frame_id = this->planningTFPrefix + "_" + this->robotModel.getRoot()->name;
    staticTransform.transform.translation.x = 0.0;
    staticTransform.transform.translation.y = 0.0;
    staticTransform.transform.translation.z = 0.0;
    staticTransform.transform.rotation.x = 0.0;
    staticTransform.transform.rotation.y = 0.0;
    staticTransform.transform.rotation.z = 0.0;
    staticTransform.transform.rotation.w = 1.0;
    this->staticTfBroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->staticTfBroadcaster->sendTransform(staticTransform);
    RCLCPP_INFO(this->get_logger(), "Published static transform from %s to %s",
      this->fixedFrame.c_str(), staticTransform.child_frame_id.c_str());

    // Before we start the timer, ensure we can use TF
    while (!this->tfBuffer->canTransform(this->fixedFrame, this->fixedFrame, tf2::TimePointZero, tf2::durationFromSec(0.1))) {}

    // Run the timer to periodically update the robot state
    this->timer = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / this->robotUpdateFrequency),
      std::bind(&RobotParser::timerCallback, this));
  }
}

void RobotParser::timerCallback() {
  if (!this->modelLoaded) { return; }

  // Publish Obstacles from Robot Collision Geometry (Using links in the Catalog we already built)
  std::vector<Obstacle> obstacles = this->extractObstaclesFromCatalog(this->collisionCatalog);
  ObstacleArray obstacleArray;
  obstacleArray.header.frame_id = this->fixedFrame;
  obstacleArray.header.stamp = this->get_clock()->now();
  for (const auto& obstacle : obstacles) {
    obstacleArray.obstacles.push_back(obstacle);
  }
  this->obstaclePub->publish(obstacleArray);

  // Publish Obstacles for Planning (with "planning::" prefix in the id)
  std::vector<Obstacle> planningObstacles = this->extractObstaclesFromCatalog(this->planningCollisionCatalog);
  ObstacleArray planningObstacleArray;
  planningObstacleArray.header.frame_id = this->fixedFrame;
  planningObstacleArray.header.stamp = this->get_clock()->now();
  for (const auto& obstacle : planningObstacles) {
    planningObstacleArray.obstacles.push_back(obstacle);
  }
  this->planningObstaclePub->publish(planningObstacleArray);
}

std::vector<Obstacle> RobotParser::extractObstaclesFromCatalog(const std::vector<CollisionCatalogEntry>& catalog) {
  std::vector<Obstacle> collisionObjects;
  collisionObjects.reserve(catalog.size());
  // Collect unique link names first
  std::unordered_map<std::string, Eigen::Affine3d> linkPoses;
  linkPoses.reserve(catalog.size());
  size_t tfSuccess = 0, tfFail = 0;
  rclcpp::Time queryTime(0, 0, this->get_clock()->get_clock_type());
  for (const auto& entry : catalog) {
    if (linkPoses.find(entry.linkName) != linkPoses.end()) continue; // already looked up
    try {
      auto tf = this->tfBuffer->lookupTransform(this->fixedFrame, entry.linkName, tf2::TimePointZero);
      linkPoses.emplace(entry.linkName, tf2::transformToEigen(tf.transform));
      tfSuccess++;
    }
    catch (const tf2::TransformException& ex) {
      tfFail++;
      // Only warn occasionally to avoid spam but still surface systemic issues
      RCLCPP_DEBUG(this->get_logger(),
        "Failed to find TF (%s -> %s): %s", this->fixedFrame.c_str(), entry.linkName.c_str(), ex.what());
    }
  }
  // Build obstacles for links whose transforms we obtained
  for (const auto& entry : catalog) {
    auto it = linkPoses.find(entry.linkName);
    if (it == linkPoses.end()) continue;
    const urdf::Pose& origin = entry.col->origin;
    Eigen::Affine3d link_T_col =
      Eigen::Translation3d(origin.position.x, origin.position.y, origin.position.z) *
      Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    Eigen::Affine3d world_T_col = it->second * link_T_col;
    Eigen::Vector3d obstCenter = world_T_col.translation();
    Eigen::Quaterniond obstOrientation(world_T_col.rotation());
    collisionObjects.push_back(this->obstacleFromCollisionObject(entry.id, *entry.col, obstCenter, obstOrientation));
  }
  RCLCPP_DEBUG(this->get_logger(),
    "extractObstaclesFromCatalog: links_ok=%zu links_fail=%zu collision_objects=%zu", tfSuccess, tfFail, collisionObjects.size());
  return collisionObjects;
}

std::vector<CollisionCatalogEntry> RobotParser::buildCollisionCatalog(urdf::Model& model, bool forPlanning) {
  std::vector<CollisionCatalogEntry> catalog;
  for (const auto& [link_name, link] : model.links_) {
    if (!link) { continue; }

    auto add_one = [&](const urdf::CollisionSharedPtr& col_ptr, size_t index) {
      if (!col_ptr || !col_ptr->geometry) { return; }
      CollisionCatalogEntry e;
      e.linkName = link_name;
      if (!col_ptr->name.empty()) {
        if (forPlanning) {
          e.id = "planning::" + link_name + "::" + col_ptr->name;
        }
        else {
          e.id = link_name + "::" + col_ptr->name;
        }
      }
      else {
        if (forPlanning) {
          e.id = "planning::" + link_name + "::col" + std::to_string(index);
        }
        else {
          e.id = link_name + "::col" + std::to_string(index);
        }
      }
      e.col = col_ptr;
      catalog.push_back(std::move(e));
    };

    if (!link->collision_array.empty()) {
      for (size_t i = 0; i < link->collision_array.size(); ++i) {
        add_one(link->collision_array[i], i);
      }
    }
    else if (link->collision) {
      add_one(link->collision, 0);
    }
  }
  return catalog;
}

Obstacle RobotParser::obstacleFromCollisionObject(
  const std::string& frameID, const urdf::Collision& collisionObject,
  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  std::string obstacleType;
  double radius = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;

  auto* geometry = collisionObject.geometry.get();
  if (urdf::Box* b = dynamic_cast<urdf::Box*>(geometry)) {
    obstacleType = "Box";
    length = b->dim.x;
    width = b->dim.y;
    height = b->dim.z;
  }
  else if (urdf::Sphere* s = dynamic_cast<urdf::Sphere*>(geometry)) {
    obstacleType = "Sphere";
    radius = s->radius;
  }
  else if (urdf::Cylinder* c = dynamic_cast<urdf::Cylinder*>(geometry)) {
    obstacleType = "Cylinder";
    radius = c->radius;
    height = c->length;
  }
  else if (urdf::Mesh* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    obstacleType = "Mesh";
    width = m->scale.y;
    height = m->scale.z;
  }
  else {
    RCLCPP_ERROR(this->get_logger(),
      "Unhandled URDF geometry type for collision object with id: %s", frameID.c_str());
  }
  Obstacle obstacle;
  obstacle.frame_id = frameID;
  obstacle.type = obstacleType;
  obstacle.group = "Robot";
  obstacle.pose.position.x = position.x();
  obstacle.pose.position.y = position.y();
  obstacle.pose.position.z = position.z();
  obstacle.pose.orientation.x = orientation.x();
  obstacle.pose.orientation.y = orientation.y();
  obstacle.pose.orientation.z = orientation.z();
  obstacle.pose.orientation.w = orientation.w();
  obstacle.radius = radius;
  obstacle.length = length;
  obstacle.width = width;
  obstacle.height = height;
  if (auto* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    obstacle.mesh_resource = m->filename;
    obstacle.scale_x = static_cast<float>(m->scale.x);
    obstacle.scale_y = static_cast<float>(m->scale.y);
    obstacle.scale_z = static_cast<float>(m->scale.z);
  }
  else {
    obstacle.mesh_resource = "";
    obstacle.scale_x = 1.0f;
    obstacle.scale_y = 1.0f;
    obstacle.scale_z = 1.0f;
  }
  return obstacle;
}

std::string RobotParser::createPlanningRobotDescription(const std::string& originalRobotDescription) {
  // From the original RD, create a copy robot that we can use for planning
  // We need a copy of the joints for a planning-JSP to publish joints to
  // We need a copy of the TF frames (links) for the RobotParser to listen to and update the obstacles for
  // This function will simply create the planning RD that we can publish to a planning/robot_description topic
  // RobotParser will need to listen to the planning TF Tree to pfield/planning/obstacles
  // So we will need a robot catalog (real robot) and a planning catalog (planning copy robot)
  //
  // Strategy:
  //  - Duplicate the URDF string and prefix all link & joint names with "planning::".
  //  - Update every reference to those names inside <parent link="...">, <child link="...">,
  //    <mimic joint="..."> and any <joint name="..."> occurrences (including inside transmissions).
  //  - Avoid double-prefixing if the function is ever called on an already-prefixed description.
  //  - Keep other tags (materials, gazebo, inertials, visuals) unchanged.
  //
  // NOTE: We intentionally do a lightweight regex / string rewrite instead of parsing the XML into
  //       a DOM to keep dependencies minimal. This assumes reasonably well‑formed URDF XML.

  static const std::string kPrefix = this->planningTFPrefix + "_";
  std::string result = originalRobotDescription; // working copy

  // Helper that prefixes the captured name if not already prefixed.
  auto applyPrefix = [&](const std::string& input, const std::regex& pattern) -> std::string {
    std::string output;
    output.reserve(input.size());
    std::sregex_iterator it(input.begin(), input.end(), pattern);
    std::sregex_iterator end;
    std::size_t lastPos = 0;
    for (; it != end; ++it) {
      const auto& m = *it;
      // m[0]: full match
      // m[1]: prefix before the name (e.g., <link ... name=")
      // m[2]: the actual name
      // m[3]: the trailing quote
      output.append(input, lastPos, m.position() - lastPos); // text before match
      std::string name = m[2].str();
      if (name.rfind(kPrefix, 0) != 0) { // not already prefixed
        name = kPrefix + name;
      }
      output += m[1].str();
      output += name;
      output += m[3].str();
      lastPos = m.position() + m.length();
    }
    output.append(input, lastPos, std::string::npos);
    return output;
  };

  // Patterns:
  //  (<link ... name=") (NAME) (" )
  //  (<joint ... name=") (NAME) (" )
  //  (<parent ... link=") (NAME) (" )
  //  (<child ... link=") (NAME) (" )
  //  (<mimic ... joint=") (NAME) (" )
  // Use \b to ensure we match the whole attribute name, allow any other attributes before/after.
  const std::regex linkName(R"((<link\b[^>]*?\bname=")([^"]+)("))");
  const std::regex jointName(R"((<joint\b[^>]*?\bname=")([^"]+)("))");
  const std::regex parentLink(R"((<parent\b[^>]*?\blink=")([^"]+)("))");
  const std::regex childLink(R"((<child\b[^>]*?\blink=")([^"]+)("))");
  const std::regex mimicJoint(R"((<mimic\b[^>]*?\bjoint=")([^"]+)("))");

  result = applyPrefix(result, linkName);
  result = applyPrefix(result, jointName);
  result = applyPrefix(result, parentLink);
  result = applyPrefix(result, childLink);
  result = applyPrefix(result, mimicJoint);

  return result;
}

void RobotParser::writeTextFile(const std::string& path, const std::string& contents, const std::string& label) {
  try {
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Failed to open %s for writing %s", path.c_str(), label.c_str());
      return;
    }
    ofs << contents;
    ofs.close();
    RCLCPP_INFO(this->get_logger(), "Wrote %s to %s (%zu chars)", label.c_str(), path.c_str(), contents.size());
  }
  catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Exception writing %s to %s: %s", label.c_str(), path.c_str(), e.what());
  }
}

void RobotParser::dumpRobotDescriptions(const std::string& original, const std::string& planning,
  const std::string& originalPath, const std::string& planningPath) {
  this->writeTextFile(originalPath, original, "original robot_description");
  this->writeTextFile(planningPath, planning, "planning robot_description");

  // Provide guidance about potential frame prefix mismatch
  // createPlanningRobotDescription currently uses 'prefix::name' pattern.
  if (planning.find(this->planningTFPrefix + "::") != std::string::npos) {
    RCLCPP_WARN(this->get_logger(),
      "Planning URDF uses '::' separators (e.g., %s::link). "
      "If the planning robot_state_publisher is configured with frame_prefix '%s', "
      "TF frames will appear as '%s<link>', not with '::'. This will cause TF lookup failures. "
      "Consider switching to underscores or aligning naming.",
      this->planningTFPrefix.c_str(), this->planningTFPrefix.c_str(), this->planningTFPrefix.c_str());
  }
}


#ifndef COMPILE_ROBOT_PARSER_NO_MAIN
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotParser>());
  rclcpp::shutdown();
  return 0;
}
#endif
