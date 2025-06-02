#include "rclcpp/rclcpp.hpp"
#include "pfield_manager.hpp"
#include "pfield.hpp"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->timerFreq = this->declare_parameter("timer_frequency", 100.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [N]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [N]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [N]
  this->maxForce = this->declare_parameter("max_force", 10.0f); // [N]
  this->urdfFilePath = this->declare_parameter("robot_description", "urdf/robot.urdf"); // Path to the URDF file
  // Get parameters from yaml file
  this->timerFreq = this->get_parameter("timer_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxForce = this->get_parameter("max_force").as_double();
  this->urdfFilePath = this->get_parameter("robot_description").as_string();

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

  // Setup TF2 broadcaster, buffer, and listener
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);
  RCLCPP_INFO(this->get_logger(), "TF2 broadcaster, buffer, and listener initialized");

  // Initialize the potential field
  this->pField = PotentialField(SpatialVector{Eigen::Vector3d::Zero()}, this->attractiveGain, this->rotationalAttractiveGain);
  // Delay for the TF2 listener to populate the buffer
  rclcpp::sleep_for(std::chrono::milliseconds(1000)); // TODO: Put this logic into a timer, this is just to visualize the robot for now
  // Load the URDF model
  RCLCPP_INFO(this->get_logger(), "Loading URDF model from %s", this->urdfFilePath.c_str());
  urdf::Model robotModel;
  if (!robotModel.initFile(this->urdfFilePath)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model from %s", this->urdfFilePath.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Successfully parsed URDF: robot name = '%s'", robotModel.getName().c_str());
    // Extract collision geometries from the URDF model and add them as obstacles
    int obstacleID = 0;
    for (const auto& link : robotModel.links_) {
      // Create a potential field obstacle from the collision geometry
      // Get the link's transform in the world frame
      std::string link_name = link.second->name;
      if (!link.second->collision) {
        RCLCPP_WARN(this->get_logger(), "Link '%s' has no collision geometry, skipping", link_name.c_str());
        continue;
      }
      // Get the transform from the world frame to the link's frame
      // Since robot_state_publisher should be publishing transforms
      TransformStamped linkTransform;
      try {
        linkTransform = this->tfBuffer->lookupTransform("world", link_name, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get transform for link '%s': %s", link_name.c_str(), ex.what());
      }
      const urdf::Pose& collisionOrigin = link.second->collision->origin;
      // Convert TF to Eigen::Affine3d
      Eigen::Affine3d world_T_link = tf2::transformToEigen(linkTransform.transform);
      // Construct link_T_collision from URDF collision origin
      Eigen::Affine3d link_T_col =
        Eigen::Translation3d(
          collisionOrigin.position.x,
          collisionOrigin.position.y,
          collisionOrigin.position.z) *
        Eigen::Quaterniond(
          collisionOrigin.rotation.w,
          collisionOrigin.rotation.x,
          collisionOrigin.rotation.y,
          collisionOrigin.rotation.z);

      // Compose to get world_T_collision
      Eigen::Affine3d world_T_col = world_T_link * link_T_col;

      // Extract final obstacle pose
      Eigen::Vector3d obstCenter = world_T_col.translation();
      Eigen::Quaterniond obstOrientation(world_T_col.rotation());

      auto obst = this->obstacleFromCollisionObject(
        obstacleID++,
        *link.second->collision,
        obstCenter,
        obstOrientation,
        2.0, // Influence zone scale
        this->repulsiveGain // Repulsive gain
      );
      this->pField.addObstacle(obst);
    }
  }


  // Create a CSV file to store the potential field data for python to plot
  // std::string filename = "pfield_data";
  // this->createCSV(filename);

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
  // this->updateTransforms();
}

Path PotentialFieldManager::interpolatePath(const SpatialVector& start, double deltaTime) {
  // Interpolate a path from the start point to the current goal point
  const double goalThreshold = 0.01; // Threshold to consider the goal reached [m]
  // Initialize Path and add the start pose
  Path path;
  path.header.frame_id = "world";
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
  while (currentPose.euclideanDistance(this->pField.getGoalPose()) > goalThreshold) {
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

PotentialFieldObstacle PotentialFieldManager::obstacleFromCollisionObject(
  int id,
  const urdf::Collision& collisionObject,
  const Eigen::Vector3d& position,
  const Eigen::Quaterniond& orientation,
  double influenceZoneScale,
  double repulsiveGain) {
  ObstacleType type;
  ObstacleGeometry geom{};

  auto* geometry = collisionObject.geometry.get();
  if (urdf::Box* b = dynamic_cast<urdf::Box*>(geometry)) {
    type = ObstacleType::BOX;
    geom.length = b->dim.x;
    geom.width = b->dim.y;
    geom.height = b->dim.z;
  } else if (urdf::Sphere* s = dynamic_cast<urdf::Sphere*>(geometry)) {
    type = ObstacleType::SPHERE;
    geom.radius = s->radius;
  } else if (urdf::Cylinder* c = dynamic_cast<urdf::Cylinder*>(geometry)) {
    type = ObstacleType::CYLINDER;
    geom.radius = c->radius;
    geom.height = c->length;
  } else if (urdf::Mesh* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    // Approximate mesh as a box for now
    type = ObstacleType::BOX;
    // TODO: Handle mesh geometry to assign Box size
  } else {
    // throw std::runtime_error("Unhandled URDF geometry type");
    RCLCPP_ERROR(this->get_logger(),
      "Unhandled URDF geometry type for collision object with id %d", id);
  }

  return PotentialFieldObstacle{
    id, position, orientation, type, geom, influenceZoneScale, repulsiveGain
  };
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
