#include "rclcpp/rclcpp.hpp"
#include "pfield_manager.hpp"
#include "pfield.hpp"
#include <fstream>
#include "tf2_ros/static_transform_broadcaster.h"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");

  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 100.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [N]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [N]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [N]
  this->maxForce = this->declare_parameter("max_force", 10.0f); // [N]
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxForce = this->get_parameter("max_force").as_double();

  // Setup marker publisher
  this->markerPub = this->create_publisher<MarkerArray>("visualization_marker_array", 10);
  RCLCPP_INFO(this->get_logger(), "Markers publishing on: %s", this->markerPub->get_topic_name());

  // Setup path publisher
  this->pathPub = this->create_publisher<Path>("nav_msgs/msg/Path", 10);
  RCLCPP_INFO(this->get_logger(), "Query point path publishing on: %s", this->pathPub->get_topic_name());

  // Setup TF2 broadcaster
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Setup goal pose subscriber
  this->goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received goal pose");
    // Update the goal pose in the potential field
    this->pField.updateGoalPosition(Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
    ));
  }
  );

  // Publish a static transform at the origin of the world frame
  static tf2_ros::StaticTransformBroadcaster staticBroadcaster = tf2_ros::StaticTransformBroadcaster(this);
  geometry_msgs::msg::TransformStamped worldTransform;
  worldTransform.header.stamp = this->now();
  worldTransform.header.frame_id = "world";
  worldTransform.child_frame_id = "map";
  worldTransform.transform.translation.x = 0.0;
  worldTransform.transform.translation.y = 0.0;
  worldTransform.transform.translation.z = 0.0;
  worldTransform.transform.rotation.x = 0.0;
  worldTransform.transform.rotation.y = 0.0;
  worldTransform.transform.rotation.z = 0.0;
  worldTransform.transform.rotation.w = 1.0;
  staticBroadcaster.sendTransform(worldTransform);

  Eigen::Quaterniond yaw45Quat = Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI / 4.0, Eigen::Vector3d::UnitZ())
  );

  // Initialize the potential field
  this->pField = PotentialField(SpatialVector{Eigen::Vector3d::Zero()}, this->attractiveGain, this->rotationalAttractiveGain);
  this->pField.addObstacle(
    PotentialFieldObstacle(0, Eigen::Vector3d(3, 3, 0), Eigen::Quaterniond::Identity(), ObstacleType::SPHERE, ObstacleGeometry{1.0, 0.0, 0.0, 0.0}, 2.0, this->repulsiveGain)
  );
  this->pField.addObstacle(
    PotentialFieldObstacle(1, Eigen::Vector3d(-1.5, -1, 0), Eigen::Quaterniond::Identity(), ObstacleType::SPHERE, ObstacleGeometry{1.5, 0.0, 0.0, 0.0}, 2.0, this->repulsiveGain)
  );
  this->pField.addObstacle(
    PotentialFieldObstacle(2, Eigen::Vector3d(-2, 3, 1.0 / 2.0), Eigen::Quaterniond::Identity(), ObstacleType::BOX, ObstacleGeometry{0.0, 1.0, 1.0, 1.0}, 2.0, this->repulsiveGain)
  );
  this->pField.addObstacle(
    PotentialFieldObstacle(3, Eigen::Vector3d(2, -3.5, 1.5 / 2.0), Eigen::Quaterniond::Identity(), ObstacleType::CYLINDER, ObstacleGeometry{1.0, 0.0, 0.0, 1.5}, 2.0, this->repulsiveGain)
  );
  this->pField.addObstacle(
    PotentialFieldObstacle(4, Eigen::Vector3d(4, -0.5, 1.5 / 2.0), yaw45Quat, ObstacleType::BOX, ObstacleGeometry{0.0, 2.0, 1.0, 1.5}, 2.0, this->repulsiveGain)
  );

  // Create a CSV file to store the potential field data for python to plot
  std::string filename = "pfield_data";
  this->createCSV(filename);

  this->queryPoint = SpatialVector(Eigen::Vector3d(8, 4, 0));

  // Run the timer for visualizing the potential field
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->timerFreq), // Timer period based on frequency
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  this->visualizePF();
  this->updateQueryPoint();
  this->updateTransforms();
}

void PotentialFieldManager::updateTransforms() {
  for (const auto& obst : this->pField.getObstacles()) {
    // Publish a TF from world to obstacle frame
    geometry_msgs::msg::TransformStamped obstacleTransform;
    obstacleTransform.header.stamp = this->now();
    obstacleTransform.header.frame_id = "world";
    obstacleTransform.child_frame_id = "obstacle_" + std::to_string(obst.getID());
    obstacleTransform.transform.translation.x = obst.getPosition().x();
    obstacleTransform.transform.translation.y = obst.getPosition().y();
    obstacleTransform.transform.translation.z = obst.getPosition().z();
    obstacleTransform.transform.rotation.x = obst.getOrientation().x();
    obstacleTransform.transform.rotation.y = obst.getOrientation().y();
    obstacleTransform.transform.rotation.z = obst.getOrientation().z();
    obstacleTransform.transform.rotation.w = obst.getOrientation().w();
    this->dynamicTfBroadcaster->sendTransform(obstacleTransform);
  }
}

void PotentialFieldManager::updateQueryPoint() {
  // If the query point is near the goal, reset it to a random position
  double distanceToGoal = this->queryPoint.euclideanDistance(this->pField.getGoalPose());
  if (distanceToGoal < 0.5) {
    // Reset the query point to a random position
    this->queryPath.poses.clear();
    double x = static_cast<double>(rand()) / RAND_MAX * 10.0 - 5.0; // Random x between -5 and 5
    double y = static_cast<double>(rand()) / RAND_MAX * 10.0 - 5.0; // Random y between -5 and 5
    this->queryPoint.setPosition(Eigen::Vector3d(x, y, 0));
  }
  const double period = (1.0f / this->timerFreq); // Period of the timer
  SpatialVector velocity = this->pField.evaluateVelocityAtPose(queryPoint);
  // Determine the new position of the query point depending on the velocity and the period
  Eigen::Vector3d newPosition = queryPoint.getPosition() + (velocity.getPosition() * period);
  this->queryPoint.setPosition(newPosition);
  PoseStamped pstamped;
  pstamped.header.frame_id = "world";
  pstamped.header.stamp = this->now();
  pstamped.pose.position.x = this->queryPoint.getPosition().x();
  pstamped.pose.position.y = this->queryPoint.getPosition().y();
  pstamped.pose.position.z = this->queryPoint.getPosition().z();
  pstamped.pose.orientation.x = this->queryPoint.getOrientation().x();
  pstamped.pose.orientation.y = this->queryPoint.getOrientation().y();
  pstamped.pose.orientation.z = this->queryPoint.getOrientation().z();
  pstamped.pose.orientation.w = this->queryPoint.getOrientation().w();
  this->queryPath.header.frame_id = "world";
  this->queryPath.header.stamp = this->now();
  this->queryPath.poses.push_back(pstamped);
  this->pathPub->publish(this->queryPath);
}

void PotentialFieldManager::visualizePF() {
  MarkerArray markerArray;
  // markerArray.markers.push_back(this->createGoalMarker());
  MarkerArray goalMarkerArray = this->createGoalMarker();
  markerArray.markers.insert(markerArray.markers.cend(), goalMarkerArray.markers.cbegin(), goalMarkerArray.markers.cend());
  auto obstacleMarkers = this->createObstacleMarkers();
  markerArray.markers.insert(markerArray.markers.cend(), obstacleMarkers.markers.cbegin(),
    obstacleMarkers.markers.cend());
  auto potentialVectorMarkers = this->createPotentialVectorMarkers();
  markerArray.markers.insert(markerArray.markers.cend(), potentialVectorMarkers.markers.cbegin(),
    potentialVectorMarkers.markers.cend());
  auto queryPointMarker = this->createQueryPointMarker();
  markerArray.markers.insert(markerArray.markers.cend(), queryPointMarker.markers.cbegin(),
    queryPointMarker.markers.cend());
  // Publish the marker array
  this->markerPub->publish(markerArray);
}

MarkerArray PotentialFieldManager::createObstacleMarkers() {
  MarkerArray markerArray;
  int id = 0;
  for (const auto& obstacle : this->pField.getObstacles()) {
    Marker obstacleMarker;
    obstacleMarker.header.frame_id = "world";
    obstacleMarker.header.stamp = this->now();
    obstacleMarker.ns = "obstacle";
    obstacleMarker.id = id;
    obstacleMarker.action = Marker::ADD;
    auto position = obstacle.getPosition();
    auto orientation = obstacle.getOrientation();
    obstacleMarker.pose.position.x = position.x();
    obstacleMarker.pose.position.y = position.y();
    obstacleMarker.pose.position.z = position.z();
    obstacleMarker.pose.orientation.x = orientation.x();
    obstacleMarker.pose.orientation.y = orientation.y();
    obstacleMarker.pose.orientation.z = orientation.z();
    obstacleMarker.pose.orientation.w = orientation.w();
    switch (obstacle.getType()) {
    case ObstacleType::SPHERE: {
      // Scale is the Diameter of the Sphere
      obstacleMarker.type = Marker::SPHERE;
      obstacleMarker.scale.x = obstacle.getGeometry().radius * 2.0f;
      obstacleMarker.scale.y = obstacle.getGeometry().radius * 2.0f;
      obstacleMarker.scale.z = obstacle.getGeometry().radius * 2.0f;
      break;
    }
    case ObstacleType::BOX: {
      obstacleMarker.type = Marker::CUBE;
      obstacleMarker.scale.x = obstacle.getGeometry().length;
      obstacleMarker.scale.y = obstacle.getGeometry().width;
      obstacleMarker.scale.z = obstacle.getGeometry().height;
      break;
    }
    case ObstacleType::CYLINDER: {
      obstacleMarker.type = Marker::CYLINDER;
      obstacleMarker.scale.x = obstacle.getGeometry().radius * 2.0f; // Diameter
      obstacleMarker.scale.y = obstacle.getGeometry().radius * 2.0f; // Diameter
      obstacleMarker.scale.z = obstacle.getGeometry().height; // Height
      break;
    }
    }
    obstacleMarker.color.r = 1.0f;
    obstacleMarker.color.g = 0.0f;
    obstacleMarker.color.b = 0.0f;
    obstacleMarker.color.a = 1.0f; // Opaque
    obstacleMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(obstacleMarker);
    // Create a transparent volume representing the influence zone
    Marker influenceMarker;
    influenceMarker.header.frame_id = "world";
    influenceMarker.header.stamp = this->now();
    influenceMarker.ns = "obstacle_influence";
    influenceMarker.id = id;
    influenceMarker.action = Marker::ADD;
    influenceMarker.pose.position.x = position.x();
    influenceMarker.pose.position.y = position.y();
    influenceMarker.pose.position.z = position.z();
    influenceMarker.pose.orientation.x = orientation.x();
    influenceMarker.pose.orientation.y = orientation.y();
    influenceMarker.pose.orientation.z = orientation.z();
    influenceMarker.pose.orientation.w = orientation.w();
    switch (obstacle.getType()) {
    case ObstacleType::SPHERE: {
      influenceMarker.type = Marker::SPHERE;
      influenceMarker.scale.x = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().radius * 2.0f; // Diameter
      influenceMarker.scale.y = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().radius * 2.0f; // Diameter
      influenceMarker.scale.z = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().radius * 2.0f; // Diameter
      break;
    }
    case ObstacleType::BOX: {
      influenceMarker.type = Marker::CUBE;
      influenceMarker.scale.x = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().length;
      influenceMarker.scale.y = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().width;
      influenceMarker.scale.z = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().height;
      break;
    }
    case ObstacleType::CYLINDER: {
      influenceMarker.type = Marker::CYLINDER;
      influenceMarker.scale.x = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().radius * 2.0f; // Diameter
      influenceMarker.scale.y = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().radius * 2.0f; // Diameter
      influenceMarker.scale.z = obstacle.getInfluenceZoneScale() * obstacle.getGeometry().height; // Height
      break;
    }
    }
    influenceMarker.color.r = 1.0f;
    influenceMarker.color.g = 1.0f;
    influenceMarker.color.b = 0.0f;
    influenceMarker.color.a = 0.5f; // Semi-transparent
    influenceMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(influenceMarker);
    id++;
  }
  return markerArray;
}

MarkerArray PotentialFieldManager::createGoalMarker() {
  // Create a green sphere marker
  Marker goalMarker;
  goalMarker.header.frame_id = "world";
  goalMarker.header.stamp = this->now();
  goalMarker.ns = "goal";
  goalMarker.id = 0;
  goalMarker.type = Marker::SPHERE;
  goalMarker.action = Marker::ADD;
  SpatialVector goalPose = this->pField.getGoalPose();
  goalMarker.pose.position.x = goalPose.getPosition().x();
  goalMarker.pose.position.y = goalPose.getPosition().y();
  goalMarker.pose.position.z = goalPose.getPosition().z();
  goalMarker.pose.orientation.x = goalPose.getOrientation().x();
  goalMarker.pose.orientation.y = goalPose.getOrientation().y();
  goalMarker.pose.orientation.z = goalPose.getOrientation().z();
  goalMarker.pose.orientation.w = goalPose.getOrientation().w();
  goalMarker.scale.x = 0.5;
  goalMarker.scale.y = 0.5;
  goalMarker.scale.z = 0.5;
  goalMarker.color.r = 0.0f;
  goalMarker.color.g = 1.0f;
  goalMarker.color.b = 0.0f;
  goalMarker.color.a = 1.0f; // Opaque
  goalMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
  std::vector<Marker> goalAxes;
  goalAxes.reserve(3);
  for (int i = 0; i < 3; i++) {
    Marker axis;
    axis.header.frame_id = "world";
    axis.header.stamp = this->now();
    axis.ns = "goal";
    axis.id = i + 1;
    axis.type = Marker::CYLINDER;
    axis.action = Marker::ADD;
    axis.pose.position.x = goalPose.getPosition().x();
    axis.pose.position.y = goalPose.getPosition().y();
    axis.pose.position.z = goalPose.getPosition().z();
    Eigen::AngleAxisd axisRotation;
    if (i == 0) { // X-axis (red cylinder)
      axis.color.r = 1.0f;
      axisRotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
    } else if (i == 1) { // Y-axis (green cylinder)
      axis.color.g = 1.0f;
      axisRotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
    } else if (i == 2) { // Z-axis (blue cylinder)
      axis.color.b = 1.0f;
      axisRotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    }
    Eigen::Quaterniond Q = goalPose.getOrientation() * axisRotation;
    axis.pose.orientation.x = Q.x();
    axis.pose.orientation.y = Q.y();
    axis.pose.orientation.z = Q.z();
    axis.pose.orientation.w = Q.w();
    axis.scale.x = 0.075f; // Diameter
    axis.scale.y = 0.075f; // Diameter
    axis.scale.z = 1.0f;  // Length of the cylinder
    axis.color.a = 0.75f; // Semi-transparent
    axis.lifetime = rclcpp::Duration(0, 0); // No lifetime
    goalAxes.push_back(axis);
  }
  MarkerArray goalMarkerArray;
  goalMarkerArray.markers.push_back(goalMarker);
  goalMarkerArray.markers.insert(
    goalMarkerArray.markers.cend(), goalAxes.cbegin(), goalAxes.cend());
  return goalMarkerArray;
}

MarkerArray PotentialFieldManager::createPotentialVectorMarkers() {
  MarkerArray markerArray;
  int id = 0;
  // Discretize the space around the goal till the farthest obstacle
  // For now, let's hard-code 10x10 grid at a single Z-height
  const double Z = this->pField.getGoalPose().getPosition().z();
  const double res = 0.5f; // Resolution of the grid
  for (double x = -5.0f; x <= 5.0f; x += res) {
    for (double y = -5.0f; y <= 5.0f; y += res) {
      for (double z = -5.0f; z <= 5.0f; z += res) {
        // Skip the Z-axis since we are only interested in the XY plane for now
        if (z != Z) { continue; }
        // Skip any points that are inside obstacle radius
        Eigen::Vector3d point(x, y, z);
        if (this->pField.isPointInsideObstacle(point)) {
          continue;
        }
        Marker vectorMarker;
        SpatialVector position{point};
        SpatialVector velocity = this->pField.evaluateVelocityAtPose(position);
        double magnitude = velocity.getPosition().norm();
        // Normalize the velocity vector since we want to show direction
        velocity.normalizePosition();
        vectorMarker.header.frame_id = "world";
        vectorMarker.header.stamp = this->now();
        vectorMarker.ns = "potential_vectors";
        vectorMarker.id = id++;
        vectorMarker.type = Marker::ARROW;
        vectorMarker.action = Marker::ADD;
        vectorMarker.pose.position.x = position.getPosition().x();
        vectorMarker.pose.position.y = position.getPosition().y();
        vectorMarker.pose.position.z = position.getPosition().z();
        // Set the orientation of the arrow to point in the direction of the velocity vector
        double yaw = std::atan2(velocity.getPosition().y(), velocity.getPosition().x());
        vectorMarker.pose.orientation = PotentialFieldManager::getQuaternionFromYaw(yaw);
        vectorMarker.scale.x = 0.2f; // Length of the arrow
        vectorMarker.scale.y = 0.1f; // Shaft diameter
        vectorMarker.scale.z = 0.15f;// Head diameter
        // Color the arrows using a gradient depending on
        // the magnitude of the velocity vector
        // maxForce is red and 0 is blue
        double colorScale = std::min<double>(magnitude / this->maxForce, 1.0f);
        vectorMarker.color.r = 1.0f - colorScale;
        vectorMarker.color.g = 0.0f;
        vectorMarker.color.b = colorScale;
        vectorMarker.color.a = 0.75f; // Semi-transparent
        vectorMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
        markerArray.markers.push_back(vectorMarker);
      }
    }
  }
  return markerArray;
}

MarkerArray PotentialFieldManager::createQueryPointMarker() {
  MarkerArray markerArray;
  Marker queryPointMarker;
  queryPointMarker.header.frame_id = "world";
  queryPointMarker.header.stamp = this->now();
  queryPointMarker.ns = "query_point";
  queryPointMarker.id = 0;
  queryPointMarker.type = Marker::SPHERE;
  queryPointMarker.action = Marker::ADD;
  queryPointMarker.pose.position.x = this->queryPoint.getPosition().x();
  queryPointMarker.pose.position.y = this->queryPoint.getPosition().y();
  queryPointMarker.pose.position.z = this->queryPoint.getPosition().z();
  queryPointMarker.scale.x = 0.3f; // Diameter
  queryPointMarker.scale.y = 0.3f; // Diameter
  queryPointMarker.scale.z = 0.3f; // Diameter
  bool isInfluenced = this->pField.isPointWithinInfluenceZone(this->queryPoint.getPosition());
  if (isInfluenced) {
    queryPointMarker.color.r = 1.0f; // Red if inside an obstacle
    queryPointMarker.color.g = 0.0f;
    queryPointMarker.color.b = 0.0f;
  } else {
    queryPointMarker.color.r = 0.0f; // Blue if outside obstacles
    queryPointMarker.color.g = 0.0f;
    queryPointMarker.color.b = 1.0f;
  }
  queryPointMarker.color.a = 1.0f; // Opaque
  queryPointMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
  markerArray.markers.push_back(queryPointMarker);
  return markerArray;
}


void PotentialFieldManager::createCSV(const std::string& base_filename) {
  // Write obstacle positions to a CSV file
  std::string obstacles_filename = base_filename + "_obstacles.csv";
  std::ofstream obstacles_file(obstacles_filename);
  if (obstacles_file.is_open()) {
    obstacles_file << "obstacle_x,obstacle_y,obstacle_z,type,influence,repulsive_gain,radius,length,width,height\n";
    for (const auto& obstacle : this->pField.getObstacles()) {
      const Eigen::Vector3d& position = obstacle.getPosition();
      std::vector<double> obstGeomVector = obstacle.getGeometry().asVector(obstacle.getType());
      obstacles_file << position.x() << "," << position.y() << "," << position.z() << ","
        << obstacleTypeToString(obstacle.getType()) << "," << obstacle.getInfluenceZoneScale() << ","
        << obstacle.getRepulsiveGain() << ","
        << obstGeomVector[0] << "," << obstGeomVector[1] << "," << obstGeomVector[2] << obstGeomVector[3]
        << "\n";
    }
    obstacles_file.close();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", obstacles_filename.c_str());
  }

  // Write velocity vectors in a 10x10 grid at Z = 0 to a separate CSV file
  std::string vectors_filename = base_filename + "_vectors.csv";
  std::ofstream vectors_file(vectors_filename);
  if (vectors_file.is_open()) {
    vectors_file << "grid_x,grid_y,grid_z,vel_x,vel_y,vel_z\n";
    const double z = 0.0;
    const double res = 1.0; // 10x10 grid from -5 to 5
    for (double x = -5.0; x <= 5.0; x += res) {
      for (double y = -5.0; y <= 5.0; y += res) {
        Eigen::Vector3d point(x, y, z);
        if (this->pField.isPointInsideObstacle(point)) {
          continue;
        }
        SpatialVector position{point};
        SpatialVector velocity = this->pField.evaluateVelocityAtPose(position);
        vectors_file << x << "," << y << "," << z << ","
          << velocity.getPosition().x() << ","
          << velocity.getPosition().y() << ","
          << velocity.getPosition().z() << "\n";
      }
    }
    vectors_file.close();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", vectors_filename.c_str());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
