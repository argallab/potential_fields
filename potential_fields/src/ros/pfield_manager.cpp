/// @file pfield_manager.cpp
/// @brief Manages a PotentialField instance and visualizes obstacles, the goal, and planned paths.
/// @author Sharwin Patil
/// @date October 6, 2025
///
/// PARAMETERS:
///   timer_frequency (float64): The frequency for the timer updating the potential field visualization [Hz]
///   attractive_gain (float64): Gain for the attractive force towards the goal [Ns/m]
///   rotational_attractive_gain (float64): Gain for the rotational attractive force [Ns/m]
///   repulsive_gain (float64): Gain for the repulsive force from obstacles [Ns/m]
///   max_force (float64): Maximum force allowed by the potential field [N]
///   fixed_frame (string): The fixed frame for RViz visualization and potential field computation
///   influence_zone_scale (float64): Scaling factor for the influence zone of obstacles
///   field_resolution (float64): Resolution of the potential field grid when visualizing [m]
///
/// SERVICES:
///   ~/pfield/plan_path (potential_fields_interfaces::srv::PlanPath): Interpolates a path from a start pose to the goal pose
///   ~/pfield/compute_autonomy_vector (potential_fields_interfaces::srv::ComputeAutonomyVector): Computes velocity vector at pose
///
/// SUBSCRIBERS:
///   ~/goal_pose (geometry_msgs::msg::PoseStamped): Updates the goal pose in the potential field
///   ~/pfield/obstacles (potential_fields_interfaces::msg::ObstacleArray): All PF Obstacles
///   ~/pfield/planning_obstacles (potential_fields_interfaces::msg::ObstacleArray): Planning PF Obstacles
///
/// PUBLISHERS:
///   ~/pfield/markers (visualization_msgs::msg::MarkerArray): Markers for PF visualization in RViz
///   ~/pfield/planning_markers (visualization_msgs::msg::MarkerArray): Markers for Planning PF visualization in RViz
///   ~/planning_joint_states (sensor_msgs::msg::JointState): Joint states for the planning robot for path planning

#include "ros/pfield_manager.hpp"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->visualizerFrequency = this->declare_parameter("visualize_pf_frequency", 100.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [Ns/m]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [Ns/m]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [Ns/m]
  this->maxForce = this->declare_parameter("max_force", 10.0f); // [N]
  this->influenceZoneScale = this->declare_parameter("influence_zone_scale", 2.0f); // Influence zone scaling factor
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->visualizerBufferArea = this->declare_parameter("visualizer_buffer_area", 1.0f); // Extra area to visualize the PF [m]
  this->fieldResolution = this->declare_parameter("field_resolution", 0.5f); // Resolution of the potential field grid [m]
  // Get parameters from yaml file
  this->visualizerFrequency = this->get_parameter("visualize_pf_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxForce = this->get_parameter("max_force").as_double();
  this->influenceZoneScale = this->get_parameter("influence_zone_scale").as_double();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->visualizerBufferArea = this->get_parameter("visualizer_buffer_area").as_double();
  this->fieldResolution = this->get_parameter("field_resolution").as_double();

  // Initialize the potential fields
  this->pField = std::make_shared<PotentialField>(this->attractiveGain, this->rotationalAttractiveGain);
  this->planningPField = std::make_shared<PotentialField>(this->attractiveGain, this->rotationalAttractiveGain);

  // Setup TF2 broadcaster, buffer, and listener
  this->dynamicTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer, this);
  RCLCPP_INFO(this->get_logger(), "TF2 broadcaster, buffer, and listener initialized");

  // Setup marker publisher
  // Use reliable and transient_local QoS for RViz MarkerArray publisher
  auto markerPubQos = rclcpp::QoS(rclcpp::KeepLast(100))
    .reliable()
    .transient_local();
  this->pFieldMarkerPub = this->create_publisher<MarkerArray>("pfield/markers", markerPubQos);
  RCLCPP_INFO(this->get_logger(), "PF Markers publishing on: %s", this->pFieldMarkerPub->get_topic_name());
  this->planningPFieldMarkerPub = this->create_publisher<MarkerArray>("pfield/planning_markers", markerPubQos);
  RCLCPP_INFO(this->get_logger(), "Planning markers publishing on: %s", this->planningPFieldMarkerPub->get_topic_name());

  // Setup planning joint state publisher
  this->planningJointStatePub = this->create_publisher<JointState>("planning_joint_states", 10);
  RCLCPP_INFO(this->get_logger(), "Planning joint states publishing on: %s", this->planningJointStatePub->get_topic_name());

  // Setup goal pose subscriber
  this->goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received goal pose");
    const SpatialVector goalPose(
      Eigen::Vector3d(
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z
      ),
      Eigen::Quaterniond(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
      )
    );
    this->pField->updateGoalPosition(goalPose);
    this->planningPField->updateGoalPosition(goalPose);
  }
  );

  // Setup obstacle subscriber
  auto obstacleSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
  this->obstacleSub = this->create_subscription<ObstacleArray>("pfield/obstacles", obstacleSubQos,
    [this](const ObstacleArray::SharedPtr msg) {
    const auto& obstacles = msg->obstacles;
    for (const auto& obst : obstacles) {
      PotentialFieldObstacle obstacle(
        obst.frame_id,
        Eigen::Vector3d(obst.pose.position.x, obst.pose.position.y, obst.pose.position.z),
        Eigen::Quaterniond(obst.pose.orientation.w, obst.pose.orientation.x, obst.pose.orientation.y, obst.pose.orientation.z),
        stringToObstacleType(obst.type),
        stringToObstacleGroup(obst.group),
        ObstacleGeometry{obst.radius, obst.length, obst.width, obst.height},
        this->influenceZoneScale,
        this->repulsiveGain,
        obst.mesh_resource,
        Eigen::Vector3d(obst.scale_x, obst.scale_y, obst.scale_z)
      );
      RCLCPP_DEBUG(this->get_logger(), "Added/Updated obstacle: %s", obst.frame_id.c_str());
      this->pField->addObstacle(obstacle);
    }
  }
  );

  // Setup Planning Obstacle Subscriber
  this->planningObstacleSub = this->create_subscription<ObstacleArray>("pfield/planning_obstacles", obstacleSubQos,
    [this](const ObstacleArray::SharedPtr msg) {
    const auto& obstacles = msg->obstacles;
    for (const auto& obst : obstacles) {
      PotentialFieldObstacle obstacle(
        obst.frame_id,
        Eigen::Vector3d(obst.pose.position.x, obst.pose.position.y, obst.pose.position.z),
        Eigen::Quaterniond(obst.pose.orientation.w, obst.pose.orientation.x, obst.pose.orientation.y, obst.pose.orientation.z),
        stringToObstacleType(obst.type),
        stringToObstacleGroup(obst.group),
        ObstacleGeometry{obst.radius, obst.length, obst.width, obst.height},
        this->influenceZoneScale,
        this->repulsiveGain,
        obst.mesh_resource,
        Eigen::Vector3d(obst.scale_x, obst.scale_y, obst.scale_z)
      );
      RCLCPP_DEBUG(this->get_logger(), "Added/Updated obstacle: %s", obst.frame_id.c_str());
      this->planningPField->addObstacle(obstacle);
    }
  }
  );

  // Create service to get planned path from a query pose to the goal pose
  this->pathPlanningService = this->create_service<PlanPath>("pfield/plan_path",
    [this](const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received path request");
    // Interpolate the path from the start pose to the goal pose
    SpatialVector startPose(
      Eigen::Vector3d(
        request->start.pose.position.x,
        request->start.pose.position.y,
        request->start.pose.position.z
      ),
      Eigen::Quaterniond(
        request->start.pose.orientation.w, request->start.pose.orientation.x,
        request->start.pose.orientation.y, request->start.pose.orientation.z
      )
    );
    PlanningResult result = this->interpolatePath(startPose, request->delta_time, request->goal_tolerance);
    if (!result.success) {
      RCLCPP_WARN(this->get_logger(), "Path planning failed");
      response->success = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Path generated with %zu poses", result.path.poses.size());
    RCLCPP_INFO(this->get_logger(), "Joint trajectory has %zu points", result.jointTrajectory.points.size());
    response->plan.header.frame_id = this->fixedFrame;
    response->plan.header.stamp = this->now();
    response->plan = result.path;
    response->joint_trajectory = result.jointTrajectory;
    response->success = true;
  }
  );

  // Create service to compute the autonomy vector at a given pose
  this->autonomyVectorService = this->create_service<ComputeAutonomyVector>("pfield/compute_autonomy_vector",
    [this](const ComputeAutonomyVector::Request::SharedPtr request, ComputeAutonomyVector::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Received autonomy vector request");
    // Compute the autonomy vector at the given pose
    SpatialVector queryPose(
      Eigen::Vector3d(
        request->start.pose.position.x,
        request->start.pose.position.y,
        request->start.pose.position.z
      ),
      Eigen::Quaterniond(
        request->start.pose.orientation.w, request->start.pose.orientation.x,
        request->start.pose.orientation.y, request->start.pose.orientation.z
      )
    );
    SpatialVector autonomyVector = this->pField->evaluateVelocityAtPose(queryPose);
    response->autonomy_vector.header.frame_id = this->fixedFrame;
    response->autonomy_vector.header.stamp = this->now();
    response->autonomy_vector.twist.linear.x = autonomyVector.getPosition().x();
    response->autonomy_vector.twist.linear.y = autonomyVector.getPosition().y();
    response->autonomy_vector.twist.linear.z = autonomyVector.getPosition().z();
    // For angular velocity, convert quaternion to euler angles (yaw, pitch, roll)
    Eigen::Vector3d eulerAngles = autonomyVector.getOrientation().toRotationMatrix().eulerAngles(2, 1, 0);
    response->autonomy_vector.twist.angular.x = eulerAngles[2]; // roll
    response->autonomy_vector.twist.angular.y = eulerAngles[1]; // pitch
    response->autonomy_vector.twist.angular.z = eulerAngles[0]; // yaw
    RCLCPP_INFO(
      this->get_logger(),
      "Autonomy vector computed at pose: pos=(%.2f, %.2f, %.2f), RPY=(%.2f, %.2f, %.2f)",
      queryPose.getPosition().x(), queryPose.getPosition().y(), queryPose.getPosition().z(),
      eulerAngles[2], eulerAngles[1], eulerAngles[0]
    );
  }
  );

  // Create a CSV file to store the potential field data for python to plot
  // std::string filename = "pfield_data";
  // this->exportFieldDataToCSV(filename);

  // Run the timer for visualizing the potential field
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->visualizerFrequency), // Timer period based on frequency
    [this]() {
    MarkerArray pfieldMarkers = this->visualizePF(this->pField);
    MarkerArray planningPFMarkers = this->visualizePF(this->planningPField);
    this->pFieldMarkerPub->publish(pfieldMarkers);
    this->planningPFieldMarkerPub->publish(planningPFMarkers);
  }
  );
}

PlanningResult PotentialFieldManager::interpolatePath(const SpatialVector& start, double deltaTime, double goalTolerance) {
  // TODO(Sharwin): Implement the new functionality here
  // interpolatePath will need to account for robot motion influencing the PF
  // during the motion so the function will need to be re-written.
  // Start is not guaranteed to be at the robot's current EE position since
  // planning should be supported for any arbitrary starting Pose
  // 1. Given the current EE Pose (starting at Start), compute the robot's joint angles with an IKSolver attached to PFM
  //    - For picking one of many IK solutions, use solution most similar to current robot state OR neutral/home robot state
  //    - Also initialize/reset our planning PField if not initialized already.
  // 2. Now that we have joint angles from the IK solver, publish these joint angles to the planning-copy of the JointStates
  //    - Once planning JS are published, the planning copy of the robot's TF frames will update and RobotParser
  //      will handle them and publish planning obstacles.
  // 3. Take a look at the planning obstacles that were published by RobotParser and update the planning PField with them
  //    - The planning PField should now have new obstacles but the same goal as the normal PField
  // 4. Compute a new velocity from the planning PField
  // 5. Project the new velocity onto the current EE pose using the deltaTime to obtain a new EE Pose
  // 6. The new EE Pose is used for the loop and we repeat steps 1-5 until the EE Pose and the goal Pose are within the tolerance
  // 7. Return the EE Path (nav_msgs/Path) and the Robot's joint positions (trajectory_msgs/JointTrajectory)
  // 8. Compute the joint velocities and put the joint velocity path into the msg (trajectory_msgs/JointTrajectory)
  //    - In order to do this, the IKSolver must offer the Jacobian to convert EE Velocites into Joint Velocities
  PlanningResult result;
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
  while (currentPose.euclideanDistance(this->pField->getGoalPose()) > goalTolerance) {
    // Evaluate the velocity at the current pose
    SpatialVector velocity = this->pField->evaluateVelocityAtPose(currentPose);
    // Update the current pose based on the velocity and delta time
    Eigen::Vector3d newPosition = currentPose.getPosition() + (velocity.getPosition() * deltaTime);
    Eigen::Quaterniond newOrientation = this->pField->integrateAngularVelocity(currentPose.getOrientation(), deltaTime);
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
  result.path = path;
  result.success = true;
  return result;
}

PFLimits PotentialFieldManager::getPFLimits(std::shared_ptr<PotentialField> pf) {
  PFLimits limits;
  // Determine the limits of the potential field based on obstacle positions and goal position
  auto obstacles = pf->getObstacles();
  Eigen::Vector3d goalPos = pf->getGoalPose().getPosition();
  if (obstacles.empty()) {
    // If no obstacles, set limits around the goal position
    limits.minX = goalPos.x();
    limits.maxX = goalPos.x();
    limits.minY = goalPos.y();
    limits.maxY = goalPos.y();
    limits.minZ = goalPos.z();
    limits.maxZ = goalPos.z();
  }
  else {
    // Initialize limits based on the first obstacle
    Eigen::Vector3d firstPos = obstacles[0].getPosition();
    limits.minX = firstPos.x();
    limits.maxX = firstPos.x();
    limits.minY = firstPos.y();
    limits.maxY = firstPos.y();
    limits.minZ = firstPos.z();
    limits.maxZ = firstPos.z();
    // Iterate through obstacles to find overall min/max
    for (const auto& obst : obstacles) {
      Eigen::Vector3d pos = obst.getPosition();
      if (pos.x() < limits.minX) limits.minX = pos.x();
      if (pos.x() > limits.maxX) limits.maxX = pos.x();
      if (pos.y() < limits.minY) limits.minY = pos.y();
      if (pos.y() > limits.maxY) limits.maxY = pos.y();
      if (pos.z() < limits.minZ) limits.minZ = pos.z();
      if (pos.z() > limits.maxZ) limits.maxZ = pos.z();
    }
    // Expand limits to include the goal position if outside current bounds
    if (goalPos.x() < limits.minX) limits.minX = goalPos.x();
    if (goalPos.x() > limits.maxX) limits.maxX = goalPos.x();
    if (goalPos.y() < limits.minY) limits.minY = goalPos.y();
    if (goalPos.y() > limits.maxY) limits.maxY = goalPos.y();
    if (goalPos.z() < limits.minZ) limits.minZ = goalPos.z();
    if (goalPos.z() > limits.maxZ) limits.maxZ = goalPos.z();
  }
  // Increase the limits by a buffer area for better visualization
  limits.minX -= this->visualizerBufferArea;
  limits.maxX += this->visualizerBufferArea;
  limits.minY -= this->visualizerBufferArea;
  limits.maxY += this->visualizerBufferArea;
  limits.minZ -= this->visualizerBufferArea;
  limits.maxZ += this->visualizerBufferArea;
  return limits;
}

MarkerArray PotentialFieldManager::visualizePF(std::shared_ptr<PotentialField> pf) {
  MarkerArray markerArray;
  MarkerArray goalMarkerArray = this->createGoalMarker(pf);
  markerArray.markers.insert(markerArray.markers.cend(), goalMarkerArray.markers.cbegin(), goalMarkerArray.markers.cend());
  auto obstacleMarkers = this->createObstacleMarkers(pf);
  markerArray.markers.insert(markerArray.markers.cend(), obstacleMarkers.markers.cbegin(),
    obstacleMarkers.markers.cend());
  auto potentialVectorMarkers = this->createPotentialVectorMarkers(pf);
  markerArray.markers.insert(markerArray.markers.cend(), potentialVectorMarkers.markers.cbegin(),
    potentialVectorMarkers.markers.cend());
  return markerArray;
}

MarkerArray PotentialFieldManager::createObstacleMarkers(std::shared_ptr<PotentialField> pf) {
  MarkerArray markerArray;
  std::vector<PotentialFieldObstacle> obstacles = pf->getObstacles();
  for (const auto& obstacle : obstacles) {
    int hashID = createHashID(obstacle);
    Marker obstacleMarker;
    obstacleMarker.header.frame_id = this->fixedFrame;
    obstacleMarker.header.stamp = this->now();
    obstacleMarker.frame_locked = true;
    obstacleMarker.ns = "obstacle";
    obstacleMarker.id = hashID;
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
    case ObstacleType::MESH: {
      if (!obstacle.getMeshResource().empty()) {
        obstacleMarker.type = Marker::MESH_RESOURCE;
        obstacleMarker.mesh_resource = obstacle.getMeshResource();
        obstacleMarker.mesh_use_embedded_materials = true;
        // Use the meshScale for visualization; if geometry dims are provided and non-zero, multiply scale accordingly
        Eigen::Vector3d scale = obstacle.getMeshScale();
        obstacleMarker.scale.x = scale.x();
        obstacleMarker.scale.y = scale.y();
        obstacleMarker.scale.z = scale.z();
      }
      else {
        // Fallback to box approximation
        obstacleMarker.type = Marker::CUBE;
        obstacleMarker.scale.x = obstacle.getGeometry().length;
        obstacleMarker.scale.y = obstacle.getGeometry().width;
        obstacleMarker.scale.z = obstacle.getGeometry().height;
      }
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
    influenceMarker.frame_locked = true;
    influenceMarker.ns = "obstacle_influence";
    influenceMarker.id = hashID; // mirror id for influence volume
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
    case ObstacleType::MESH: {
      // Represent mesh influence as a scaled bounding box around the mesh
      influenceMarker.type = Marker::MESH_RESOURCE;
      Eigen::Vector3d scale = obstacle.getMeshScale();
      influenceMarker.scale.x = obstacle.getInfluenceZoneScale() * scale.x();
      influenceMarker.scale.y = obstacle.getInfluenceZoneScale() * scale.y();
      influenceMarker.scale.z = obstacle.getInfluenceZoneScale() * scale.z();
      break;
    }
    }
    influenceMarker.color.r = 1.0f;
    influenceMarker.color.g = 1.0f;
    influenceMarker.color.b = 0.0f;
    influenceMarker.color.a = 0.5f; // Semi-transparent
    influenceMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(influenceMarker);
  }
  return markerArray;
}

MarkerArray PotentialFieldManager::createGoalMarker(std::shared_ptr<PotentialField> pf) {
  // Create a green sphere marker
  Marker goalMarker;
  goalMarker.header.frame_id = this->fixedFrame;
  goalMarker.header.stamp = this->now();
  goalMarker.frame_locked = true;
  goalMarker.ns = "goal";
  goalMarker.id = 0;
  goalMarker.type = Marker::SPHERE;
  goalMarker.action = Marker::ADD;
  SpatialVector goalPose = pf->getGoalPose();
  goalMarker.pose.position.x = goalPose.getPosition().x();
  goalMarker.pose.position.y = goalPose.getPosition().y();
  goalMarker.pose.position.z = goalPose.getPosition().z();
  goalMarker.pose.orientation.x = goalPose.getOrientation().x();
  goalMarker.pose.orientation.y = goalPose.getOrientation().y();
  goalMarker.pose.orientation.z = goalPose.getOrientation().z();
  goalMarker.pose.orientation.w = goalPose.getOrientation().w();
  goalMarker.scale.x = 0.15;
  goalMarker.scale.y = 0.15;
  goalMarker.scale.z = 0.15;
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

MarkerArray PotentialFieldManager::createPotentialVectorMarkers(std::shared_ptr<PotentialField> pf) {
  MarkerArray markerArray;
  int id = 0;
  const auto limits = this->getPFLimits(pf);
  for (double x = limits.minX; x <= limits.maxX; x += this->fieldResolution) {
    for (double y = limits.minY; y <= limits.maxY; y += this->fieldResolution) {
      for (double z = limits.minZ; z <= limits.maxZ; z += this->fieldResolution) {
        // Skip any points that are inside obstacle radius
        Eigen::Vector3d point(x, y, z);
        if (pf->isPointInsideObstacle(point)) { continue; }
        Marker vectorMarker;
        SpatialVector position{point};
        SpatialVector velocity = pf->evaluateVelocityAtPose(position);
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
        vectorMarker.scale.x = 0.15f; // Length of the arrow
        vectorMarker.scale.y = 0.05f; // Shaft diameter
        vectorMarker.scale.z = 0.1f; // Head diameter
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

void PotentialFieldManager::exportFieldDataToCSV(std::shared_ptr<PotentialField> pf, const std::string& base_filename) {
  // Write obstacle positions to a CSV file
  std::string obstacles_filename = "data/" + base_filename + "_obstacles.csv";
  std::ofstream obstacles_file(obstacles_filename);
  if (obstacles_file.is_open()) {
    obstacles_file << "obstacle_x,obstacle_y,obstacle_z,type,influence,repulsive_gain,radius,length,width,height\n";
    for (const auto& obstacle : pf->getObstacles()) {
      const Eigen::Vector3d& position = obstacle.getPosition();
      const ObstacleGeometry& g = obstacle.getGeometry();
      obstacles_file << position.x() << "," << position.y() << "," << position.z() << ","
        << obstacleTypeToString(obstacle.getType()) << "," << obstacle.getInfluenceZoneScale() << ","
        << obstacle.getRepulsiveGain() << ","
        << g.radius << "," << g.length << "," << g.width << "," << g.height
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
    const double z = 0.0;
    const double res = 1.0; // 10x10 grid from -5 to 5
    for (double x = -5.0; x <= 5.0; x += res) {
      for (double y = -5.0; y <= 5.0; y += res) {
        Eigen::Vector3d point(x, y, z);
        if (pf->isPointInsideObstacle(point)) {
          continue;
        }
        SpatialVector position{point};
        SpatialVector velocity = pf->evaluateVelocityAtPose(position);
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
