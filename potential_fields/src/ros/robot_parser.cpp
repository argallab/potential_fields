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


RobotParser::RobotParser() : Node("robot_parser") {
  RCLCPP_INFO(this->get_logger(), "RobotParser Initialized");

  // Declare parameters
  this->robotUpdateFrequency = this->declare_parameter("robot_geometry_update_frequency", 50.0f); // [Hz]
  this->urdfFileName = this->declare_parameter("urdf_file_path", "urdf/robot.urdf");
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->robotUpdateFrequency = this->get_parameter("robot_geometry_update_frequency").as_double();
  this->urdfFileName = this->get_parameter("urdf_file_path").as_string();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  RCLCPP_INFO(this->get_logger(), "Using URDF File: %s", this->urdfFileName.c_str());

  // Setup obstacle publisher (smaller queue reduces latency/buffering)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();
  this->obstaclePub = this->create_publisher<ObstacleArray>("pfield/obstacles", qos);

  // Setup Pinocchio kinematic model for converting joint angles to PF Obstacles
  this->kinematicModel = PFKinematics(this->urdfFileName);
  // Log some information about the loaded model
  auto pinModel = this->kinematicModel.getModel();
  RCLCPP_INFO(this->get_logger(), "Loaded URDF model with %u joints and %u frames",
    pinModel.njoints, pinModel.nframes);
  // Print the name of each frame
  for (const auto& frame : pinModel.frames) {
    RCLCPP_INFO(this->get_logger(), "  Frame: %s", frame.name.c_str());
  }

  // Parse URDF once to build the collision catalog
  if (this->robotModel.initFile(this->urdfFileName)) {
    this->collisionCatalog = this->buildCollisionCatalog(this->robotModel);
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
    RCLCPP_INFO(this->get_logger(), "Robot has %zu collision objects in catalog", this->collisionCatalog.size());
    // Setup joint state subscriber
    this->jointStateSub = this->create_subscription<JointState>(
      "/pfield/joint_states", 10,
      std::bind(&RobotParser::jointStateCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "PF Obstacles from JointStates initialized");
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model");
  }
}

void RobotParser::jointStateCallback(const JointState::SharedPtr msg) {
  std::unordered_map<std::string, double> jointAngleMap;
  for (size_t i = 0; i < msg->name.size(); ++i) { jointAngleMap[msg->name[i]] = msg->position[i]; }
  std::unordered_map<std::string, Eigen::Affine3d> linkTransforms = this->kinematicModel.jointAnglesToLinkTransforms(
    jointAngleMap, this->collisionLinkNames
  );
  ObstacleArray obsArray = this->extractObstaclesFromCatalog(this->collisionCatalog, linkTransforms);
  this->obstaclePub->publish(obsArray);
}

ObstacleArray RobotParser::extractObstaclesFromCatalog(
  const std::vector<CollisionCatalogEntry>& catalog,
  const std::unordered_map<std::string, Eigen::Affine3d>& linkToTransformMap) {
  ObstacleArray collisionObstacles;
  collisionObstacles.header.frame_id = this->fixedFrame;
  collisionObstacles.header.stamp = this->now();
  collisionObstacles.obstacles.reserve(catalog.size());
  // Build obstacles for links whose transforms we obtained
  for (const auto& entry : catalog) {
    auto it = linkToTransformMap.find(entry.linkName);
    if (it == linkToTransformMap.end()) {
      RCLCPP_WARN(this->get_logger(),
        "No transform found for link '%s' (collision id '%s')",
        entry.linkName.c_str(), entry.id.c_str()
      );
      continue;
    }
    const urdf::Pose& origin = entry.col->origin;
    Eigen::Affine3d link_T_col =
      Eigen::Translation3d(origin.position.x, origin.position.y, origin.position.z) *
      Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    Eigen::Affine3d world_T_col = it->second * link_T_col;
    Eigen::Vector3d obstCenter = world_T_col.translation();
    Eigen::Quaterniond obstOrientation(world_T_col.rotation());
    collisionObstacles.obstacles.push_back(this->obstacleFromCollisionObject(entry.id, *entry.col, obstCenter, obstOrientation));
  }
  return collisionObstacles;
}

std::vector<CollisionCatalogEntry> RobotParser::buildCollisionCatalog(urdf::Model& model) {
  std::vector<CollisionCatalogEntry> catalog;
  for (const auto& [link_name, link] : model.links_) {
    if (!link) { continue; }

    auto add_one = [&](const urdf::CollisionSharedPtr& col_ptr, size_t index) {
      if (!col_ptr || !col_ptr->geometry) { return; }
      CollisionCatalogEntry e;
      e.linkName = link_name;
      if (!col_ptr->name.empty()) {
        e.id = link_name + "::" + col_ptr->name;
      }
      else {
        e.id = link_name + "::col" + std::to_string(index);
      }
      e.col = col_ptr;
      catalog.push_back(std::move(e));
      this->collisionLinkNames.push_back(link_name);
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


#ifndef COMPILE_ROBOT_PARSER_NO_MAIN
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotParser>());
  rclcpp::shutdown();
  return 0;
}
#endif
