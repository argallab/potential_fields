/// @file pfield_manager.cpp
/// @brief Manages a PotentialField instance and visualizes obstacles, the goal, and planned paths.
///
/// PARAMETERS:
///   timer_frequency (float64): The frequency for the timer updating the potential field visualization [Hz]
///   attractive_gain (float64): Gain for the attractive force towards the goal [Ns/m]
///   rotational_attractive_gain (float64): Gain for the rotational attractive force [Ns/m]
///   repulsive_gain (float64): Gain for the repulsive force from obstacles [Ns/m]
///   max_force (float64): Maximum force allowed by the potential field [N]
///   fixed_frame (string): The fixed frame for RViz visualization and potential field computation
///
/// SERVICES:
///   ~/pfield/plan_path (potential_fields_interfaces::srv::PlanPath): Interpolates a path from a start pose to the goal pose
///
/// SUBSCRIBERS:
///   ~/goal_pose (geometry_msgs::msg::PoseStamped): Updates the goal pose in the potential field
///   ~/pfield/obstacles (potential_fields_interfaces::msg::Obstacle): Obtains the obstacles to be added to the potential field
///
/// PUBLISHERS:
///   ~/visualization_marker_array (visualization_msgs::msg::MarkerArray): Publishes markers for visualization in RViz
///   ~/nav_msgs/msg/Path (nav_msgs::msg::Path): Publishes the path of the query point to visualize its trajectory

#include "pfield_manager.hpp"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 100.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [Ns/m]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [Ns/m]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [Ns/m]
  this->maxForce = this->declare_parameter("max_force", 10.0f); // [N]
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxForce = this->get_parameter("max_force").as_double();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  // Initialize the potential field
  this->pField = PotentialField(SpatialVector{Eigen::Vector3d::Zero()}, this->attractiveGain, this->rotationalAttractiveGain);

  // Setup TF2 broadcaster, buffer, and listener
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);
  RCLCPP_INFO(this->get_logger(), "TF2 broadcaster, buffer, and listener initialized");

  // Setup marker publisher
  this->markerPub = this->create_publisher<MarkerArray>("visualization_marker_array", 10);
  RCLCPP_INFO(this->get_logger(), "Markers publishing on: %s", this->markerPub->get_topic_name());

  // Setup path publisher
  this->pathPub = this->create_publisher<Path>("nav_msgs/msg/Path", 10);
  RCLCPP_INFO(this->get_logger(), "Query point path publishing on: %s", this->pathPub->get_topic_name());

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

  // Setup obstacle subscriber
  this->obstacleSub = this->create_subscription<Obstacle>("pfield/obstacles", 10,
    [this](const Obstacle::SharedPtr msg) {
    PotentialFieldObstacle obstacle(
      msg->id,
      Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
      stringToObstacleType(msg->type),
      ObstacleGeometry{msg->radius, msg->length, msg->width, msg->height},
      2.0, // influence zone scale
      this->repulsiveGain
    );
    this->pField.addObstacle(obstacle);
  }
  );

  // Create service to get planned path from a query pose to the goal pose
  this->pathPlanningService = this->create_service<PlanPath>("pfield/plan_path",
    [this](const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received path request");
    // Interpolate the path from the start pose to the goal pose
    SpatialVector startPose(
      Eigen::Vector3d(
        request->start.pose.position.x, request->start.pose.position.y,
        request->start.pose.position.z
      ),
      Eigen::Quaterniond(
        request->start.pose.orientation.w, request->start.pose.orientation.x,
        request->start.pose.orientation.y, request->start.pose.orientation.z
      )
    );
    Path path = this->interpolatePath(startPose, request->delta_time, request->goal_tolerance);
    response->plan.header.frame_id = this->fixedFrame;
    response->plan.header.stamp = this->now();
    response->plan = path;
    RCLCPP_INFO(this->get_logger(), "Path generated with %zu poses", path.poses.size());
  }
  );

  // Create a CSV file to store the potential field data for python to plot
  // std::string filename = "pfield_data";
  // this->exportFieldDataToCSV(filename);

  // Run the timer for visualizing the potential field
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->timerFreq), // Timer period based on frequency
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  this->visualizePF();
}

Path PotentialFieldManager::interpolatePath(const SpatialVector& start, double deltaTime, double goalTolerance) {
  // Interpolate a path from the start point to the current goal point
  // Initialize Path and add the start pose
  Path path;
  path.header.frame_id = this->fixedFrame;
  path.header.stamp = this->now();
  PoseStamped startPose;
  startPose.header.frame_id = path.header.frame_id;
  startPose.header.stamp = path.header.stamp;
  startPose.pose.position.x = start.getPosition().x();
  startPose.pose.position.y = start.getPosition().y();
  startPose.pose.position.z = start.getPosition().z();
  startPose.pose.orientation.x = start.getOrientation().x();
  startPose.pose.orientation.y = start.getOrientation().y();
  startPose.pose.orientation.z = start.getOrientation().z();
  startPose.pose.orientation.w = start.getOrientation().w();
  path.poses.push_back(startPose);
  // Interpolate through the potential field until the goal is reached with the given delta time
  SpatialVector currentPose = start;
  while (currentPose.euclideanDistance(this->pField.getGoalPose()) > goalTolerance) {
    // Evaluate the velocity at the current pose
    SpatialVector velocity = this->pField.evaluateVelocityAtPose(currentPose);
    // Update the current pose based on the velocity and delta time
    Eigen::Vector3d newPosition = currentPose.getPosition() + (velocity.getPosition() * deltaTime);
    Eigen::Quaterniond newOrientation = this->pField.integrateAngularVelocity(currentPose.getOrientation(), deltaTime);
    currentPose.setPosition(newPosition);
    currentPose.setOrientation(newOrientation);
    // Add the new pose to the path
    PoseStamped newPose;
    newPose.header.frame_id = path.header.frame_id;
    newPose.header.stamp = this->now();
    newPose.pose.position.x = currentPose.getPosition().x();
    newPose.pose.position.y = currentPose.getPosition().y();
    newPose.pose.position.z = currentPose.getPosition().z();
    newPose.pose.orientation.x = currentPose.getOrientation().x();
    newPose.pose.orientation.y = currentPose.getOrientation().y();
    newPose.pose.orientation.z = currentPose.getOrientation().z();
    newPose.pose.orientation.w = currentPose.getOrientation().w();
    path.poses.push_back(newPose);
  }
  return path;
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
  // auto queryPointMarker = this->createQueryPointMarker();
  // markerArray.markers.insert(markerArray.markers.cend(), queryPointMarker.markers.cbegin(),
  //   queryPointMarker.markers.cend());
  // Publish the marker array
  this->markerPub->publish(markerArray);
}

MarkerArray PotentialFieldManager::createObstacleMarkers() {
  MarkerArray markerArray;
  int id = 0;
  for (const auto& obstacle : this->pField.getObstacles()) {
    Marker obstacleMarker;
    obstacleMarker.header.frame_id = this->fixedFrame;
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
    influenceMarker.header.frame_id = this->fixedFrame;
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
  goalMarker.header.frame_id = this->fixedFrame;
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
    axis.header.frame_id = this->fixedFrame;
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
    }
    else if (i == 1) { // Y-axis (green cylinder)
      axis.color.g = 1.0f;
      axisRotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
    }
    else if (i == 2) { // Z-axis (blue cylinder)
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
  // TODO: Decide limits based on farthest entity in the field
  // TODO: Parameterize these limits and resolution
  const double lowerLimit = -5.0f; // Lower limit of the grid
  const double upperLimit = 5.0f; // Upper limit of the grid
  const double fieldResolution = 0.5f; // Resolution of the grid
  const double Z = this->pField.getGoalPose().getPosition().z();
  for (double x = lowerLimit; x <= upperLimit; x += fieldResolution) {
    for (double y = lowerLimit; y <= upperLimit; y += fieldResolution) {
      for (double z = lowerLimit; z <= upperLimit; z += fieldResolution) {
        // Skip the Z-axis since we are only interested in the XY plane for now
        if (std::abs(z - Z) > 1e-6) { continue; }
        // Skip any points that are inside obstacle radius
        Eigen::Vector3d point(x, y, z);
        if (this->pField.isPointInsideObstacle(point)) { continue; }
        Marker vectorMarker;
        SpatialVector position{point};
        SpatialVector velocity = this->pField.evaluateVelocityAtPose(position);
        double magnitude = velocity.getPosition().norm();
        // Normalize the velocity vector since we want to show direction
        velocity.normalizePosition();
        vectorMarker.header.frame_id = this->fixedFrame;
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

void PotentialFieldManager::exportFieldDataToCSV(const std::string& base_filename) {
  // Write obstacle positions to a CSV file
  std::string obstacles_filename = "data/" + base_filename + "_obstacles.csv";
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
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", obstacles_filename.c_str());
  }

  // Write velocity vectors to a separate CSV file
  std::string vectors_filename = "data/" + base_filename + "_vectors.csv";
  std::ofstream vectors_file(vectors_filename);
  if (vectors_file.is_open()) {
    vectors_file << "grid_x,grid_y,grid_z,vel_x,vel_y,vel_z\n";
    //TODO: Parameterize the grid resolution and limits
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
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", vectors_filename.c_str());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
