/// @file pfield_manager.cpp
/// @brief Manages a PotentialField instance and visualizes obstacles, the goal, and planned paths.
/// @author Sharwin Patil
/// @date November 2, 2025
/// @details
///   ROS 2 node wrapping the potential field library. It:
///   - Publishes RViz markers for goal, obstacles, influence zones, and velocity vectors
///   - Integrates a live "query pose" each timer tick using opposing-force removal and motion constraints
///   - Selects a MotionPlugin (e.g., Null, Franka) and assigns its IKSolver to the PotentialField
///   - Optionally initializes kinematics from a URDF and adapts influence distance based on robot extent
///
/// PARAMETERS (declared as ROS 2 parameters):
///   visualize_pf_frequency (float64): Timer frequency for PF updates [Hz]
///   attractive_gain (float64): Gain for translational attraction [Ns/m]
///   rotational_attractive_gain (float64): Gain for rotational attraction [Ns·m/rad]
///   repulsive_gain (float64): Gain for obstacle repulsion [Ns/m]
///   max_linear_velocity (float64): Linear velocity soft cap [m/s]
///   max_angular_velocity (float64): Angular velocity soft cap [rad/s]
///   max_linear_acceleration (float64): Linear acceleration limit [m/s^2]
///   max_angular_acceleration (float64): Angular acceleration limit [rad/s^2]
///   influence_distance (float64): Absolute distance from obstacle surface where repulsion acts [m]
///   visualizer_buffer_area (float64): Extra margin around limits for RViz markers [m]
///   field_resolution (float64): Grid spacing when sampling the field for visualization [m]
///   fixed_frame (string): RViz fixed frame for visualization and PF computation
///   urdf_file_path (string): Optional path to a URDF; enables kinematics and extent estimation
///   motion_plugin_type (string): Motion plugin to use (e.g., "null", "franka")
///
/// SERVICES:
///   pfield/plan_path (potential_fields_interfaces::srv::PlanPath): Plans a path from a start pose to the PF goal
///   pfield/compute_autonomy_vector (potential_fields_interfaces::srv::ComputeAutonomyVector): Computes velocity at a pose
///
/// SUBSCRIBERS:
///   pfield/planning_goal_pose (geometry_msgs::msg::PoseStamped): Updates the PF goal pose
///   pfield/obstacles (potential_fields_interfaces::msg::ObstacleArray): Adds/updates external PF obstacles
///   pfield/query_pose (geometry_msgs::msg::Pose): Sets the live query pose used for visualization
///
/// PUBLISHERS:
///   pfield/markers (visualization_msgs::msg::MarkerArray): RViz markers (reliable + transient_local QoS)
///   pfield/planned_path (nav_msgs::msg::Path): Planned end-effector path

#include "ros/pfield_manager.hpp"
#include "robot_plugins/null_motion_plugin.hpp"
#include "robot_plugins/franka_plugin.hpp"
#include "robot_plugins/xarm_plugin.hpp"

PotentialFieldManager::PotentialFieldManager() : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->visualizerFrequency = this->declare_parameter("visualize_pf_frequency", 100.0); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0); // [Ns/m]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7); // [Ns/m]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 0.1); // [Ns/m]
  this->maxLinearVelocity = this->declare_parameter("max_linear_velocity", 1.0); // [m/s]
  this->maxAngularVelocity = this->declare_parameter("max_angular_velocity", 1.0); // [rad/s]
  this->maxLinearAcceleration = this->declare_parameter("max_linear_acceleration", 1.0); // [m/s^2]
  this->maxAngularAcceleration = this->declare_parameter("max_angular_acceleration", 1.0); // [rad/s^2]
  this->influenceDistance = this->declare_parameter("influence_distance", 1.0); // Influence distance for obstacle repulsion
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->visualizerBufferArea = this->declare_parameter("visualizer_buffer_area", 1.0); // Extra area to visualize the PF [m]
  this->fieldResolution = this->declare_parameter("field_resolution", 0.5); // Resolution of the potential field grid [m]
  this->urdfFileName = this->declare_parameter("urdf_file_path", std::string());
  this->motionPluginType = this->declare_parameter("motion_plugin_type", std::string()); // Motion Plugin Type [e.g. "franka"]
  // Get parameters from yaml file
  this->visualizerFrequency = this->get_parameter("visualize_pf_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxLinearVelocity = this->get_parameter("max_linear_velocity").as_double();
  this->maxAngularVelocity = this->get_parameter("max_angular_velocity").as_double();
  this->maxLinearAcceleration = this->get_parameter("max_linear_acceleration").as_double();
  this->maxAngularAcceleration = this->get_parameter("max_angular_acceleration").as_double();
  this->influenceDistance = this->get_parameter("influence_distance").as_double();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->visualizerBufferArea = this->get_parameter("visualizer_buffer_area").as_double();
  this->fieldResolution = this->get_parameter("field_resolution").as_double();
  this->urdfFileName = this->get_parameter("urdf_file_path").as_string();
  this->motionPluginType = this->get_parameter("motion_plugin_type").as_string();

  // Initialize the potential fields
  this->pField = std::make_shared<PotentialField>(
    this->attractiveGain, this->repulsiveGain, this->rotationalAttractiveGain,
    this->maxLinearVelocity, this->maxAngularVelocity,
    this->maxLinearAcceleration, this->maxAngularAcceleration,
    this->influenceDistance
  );

  // Initialize the motion plugin
  std::transform(
    this->motionPluginType.cbegin(),
    this->motionPluginType.cend(),
    this->motionPluginType.begin(),
    [](unsigned char c) { return std::tolower(c); }
  );
  if (this->motionPluginType.empty()) {
    this->motionPlugin = std::make_unique<NullMotionPlugin>();
  }
  #ifdef USING_FRANKA
  else if (this->motionPluginType == "franka") {
    std::string frankaHostname = this->declare_parameter("franka_hostname", std::string());
    frankaHostname = this->get_parameter("franka_hostname").as_string();
    this->motionPlugin = std::make_unique<FrankaPlugin>(frankaHostname);
  }
  #endif // USING_FRANKA
  else if (this->motionPluginType == "xarm") {
    this->motionPlugin = std::make_unique<XArmPlugin>();
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Unknown motion plugin type: %s. Using NullMotionPlugin", this->motionPluginType.c_str());
    this->motionPlugin = std::make_unique<NullMotionPlugin>();
  }
  RCLCPP_INFO(this->get_logger(), "Using Motion Plugin: %s", this->motionPlugin->getName().c_str());

  // Save the IKSolver from the motion plugin
  this->ikSolver = this->motionPlugin->getIKSolver();
  if (!this->ikSolver) {
    RCLCPP_WARN(this->get_logger(), "IKSolver not available from motionPlugin");
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Using IKSolver: %s", this->ikSolver->getName().c_str());
  }

  // Once IKSolver is initialized, assign it to the PF instance
  this->pField->assignIKSolver(this->ikSolver);

  // Allow the user to not use a URDF
  if (!this->urdfFileName.empty() && this->urdfFileName.ends_with(".urdf")) {
    try {
      this->pField->initializeKinematics(this->urdfFileName, this->ikSolver->getJointNames());
      RCLCPP_INFO(this->get_logger(), "PF Kinematics initialized from URDF: %s", this->urdfFileName.c_str());
      RCLCPP_INFO(this->get_logger(),
        "PF Kinematics estimated influence distance from robot extend to be: %f", this->pField->getInfluenceDistance());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize PF Kinematics from URDF: %s", e.what());
    }
  }
  else {
    RCLCPP_WARN(this->get_logger(), "URDF file path is empty. Kinematics not initialized.");
  }

  // Setup marker publisher
  // Use reliable and transient_local QoS for RViz MarkerArray publisher
  auto markerPubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
  this->pFieldMarkerPub = this->create_publisher<MarkerArray>("pfield/markers", markerPubQos);
  RCLCPP_INFO(this->get_logger(), "PF Markers publishing on: %s", this->pFieldMarkerPub->get_topic_name());

  // Setup goal pose subscriber
  auto goalPoseQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  this->goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("pfield/planning_goal_pose",
    goalPoseQos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
    this->pField->setGoalPose(goalPose);
  }
  );

  // Setup obstacle subscriber for external obstacles
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
        obst.mesh_resource,
        Eigen::Vector3d(obst.scale_x, obst.scale_y, obst.scale_z)
      );
      RCLCPP_DEBUG(this->get_logger(), "Added/Updated obstacle: %s", obst.frame_id.c_str());
      this->pField->addObstacle(obstacle);
    }
  }
  );

  // Setup query pose subscriber (for visualizing paths in "real-time")
  auto queryPoseQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  this->queryPoseSub = this->create_subscription<geometry_msgs::msg::Pose>("pfield/query_pose",
    queryPoseQos,
    [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
    const SpatialVector queryPose(
      Eigen::Vector3d(
        msg->position.x,
        msg->position.y,
        msg->position.z
      ),
      Eigen::Quaterniond(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
      )
    );
    this->queryPose = queryPose;
  }
  );

  // Setup planned end-effector path publisher
  this->plannedEndEffectorPathPub = this->create_publisher<Path>("pfield/planned_path", 10);
  RCLCPP_INFO(this->get_logger(), "Planned EE path publishing on: %s", this->plannedEndEffectorPathPub->get_topic_name());

  // Create service to compute the autonomy vector at a given pose
  this->autonomyVectorService = this->create_service<ComputeAutonomyVector>(
    "pfield/compute_autonomy_vector",
    std::bind(&PotentialFieldManager::handleComputeAutonomyVector, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create the PlanPath service
  this->pathPlanningService = this->create_service<PlanPath>(
    "pfield/plan_path",
    std::bind(&PotentialFieldManager::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create a CSV file to store the potential field data for python to plot
  // std::string filename = "pfield_data";
  // this->exportFieldDataToCSV(filename);

  // Run the timer for visualizing the potential field
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->visualizerFrequency),
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  // Get updated obstacles from PFKinematics and update internal PF
  // this->pField->updateObstaclesFromKinematics(this->motionPlugin->getCurrentJointAngles());
  // Update the query pose position based on integrating the PF velocity
  this->integrateQueryPoseFromField();
  MarkerArray pfieldMarkers = this->visualizePF(this->pField);
  this->pFieldMarkerPub->publish(pfieldMarkers);
}

void PotentialFieldManager::integrateQueryPoseFromField() {
  // Advance pose using constrained interpolation (acceleration limits applied internally)
  const double dt = this->now().seconds() - this->lastQueryUpdate.seconds();
  this->lastQueryUpdate = this->now();
  TaskSpaceWrench wrench = this->pField->evaluateWrenchAtPoseWithOpposingForceRemoval(this->queryPose);
  TaskSpaceTwist twist = this->pField->applyMotionConstraints(
    this->pField->wrenchToTwist(wrench), this->prevQueryTwist, dt);
  Eigen::Vector3d nextPosition = this->pField->integrateLinearVelocity(
    this->queryPose.getPosition(), twist.linearVelocity, dt);
  Eigen::Quaterniond nextOrientation = this->pField->integrateAngularVelocity(
    this->queryPose.getOrientation(), twist.angularVelocity, dt);
  this->queryPose = SpatialVector(nextPosition, nextOrientation);
  // Supply velocity-limited twist as previous for next acceleration limiting
  this->prevQueryTwist = twist;
}

void PotentialFieldManager::handleComputeAutonomyVector(
  const ComputeAutonomyVector::Request::SharedPtr request, ComputeAutonomyVector::Response::SharedPtr response) {
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
  TaskSpaceTwist autonomyVector = this->pField->evaluateLimitedVelocityAtPose(queryPose);
  response->autonomy_vector.header.frame_id = this->fixedFrame;
  response->autonomy_vector.header.stamp = this->now();
  response->autonomy_vector.twist.linear.x = autonomyVector.getLinearVelocity().x();
  response->autonomy_vector.twist.linear.y = autonomyVector.getLinearVelocity().y();
  response->autonomy_vector.twist.linear.z = autonomyVector.getLinearVelocity().z();
  response->autonomy_vector.twist.angular.x = autonomyVector.getAngularVelocity().x();
  response->autonomy_vector.twist.angular.y = autonomyVector.getAngularVelocity().y();
  response->autonomy_vector.twist.angular.z = autonomyVector.getAngularVelocity().z();
  Eigen::Vector3d eulerAngles = queryPose.getOrientation().toRotationMatrix().eulerAngles(0, 1, 2);
  const double roll = eulerAngles[0];
  const double pitch = eulerAngles[1];
  const double yaw = eulerAngles[2];
  RCLCPP_INFO(
    this->get_logger(),
    "Autonomy vector computed at pose: pos=(%.2f, %.2f, %.2f), RPY=(%.2f, %.2f, %.2f)",
    queryPose.getPosition().x(), queryPose.getPosition().y(), queryPose.getPosition().z(),
    roll, pitch, yaw
  );
}

void PotentialFieldManager::handlePlanPath(const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(),
    "PlanPath request: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) delta_time=%.4f goal_tolerance=%.6f max_steps=%d",
    request->start.pose.position.x, request->start.pose.position.y, request->start.pose.position.z,
    request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z,
    request->delta_time, request->goal_tolerance, request->max_iterations
  );

  auto startSV = SpatialVector(
    Eigen::Vector3d(
      request->start.pose.position.x,
      request->start.pose.position.y,
      request->start.pose.position.z
    ),
    Eigen::Quaterniond(
      request->start.pose.orientation.w,
      request->start.pose.orientation.x,
      request->start.pose.orientation.y,
      request->start.pose.orientation.z
    )
  );

  // Plan a path using the request parameters and store the result
  auto planningResult = this->pField->planPath(
    /*startPose=*/startSV,
    /*dt=*/request->delta_time,
    /*goalTolerance=*/request->goal_tolerance,
    /*maxIterations=*/request->max_iterations
  );

  // Establish a consistent time base for the planned trajectory
  const double stepDt = (request->delta_time > 0.0) ? request->delta_time : 0.1;
  const rclcpp::Time t0 = this->now();

  // Create Path msg from EE Poses, stamp each pose at t0 + i*dt
  nav_msgs::msg::Path path;
  path.header.frame_id = this->fixedFrame;
  path.header.stamp = t0;
  for (size_t i = 0; i < planningResult.poses.size(); ++i) {
    const auto& pose = planningResult.poses[i];
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.header.frame_id = this->fixedFrame;
    poseStamped.header.stamp = t0 + rclcpp::Duration::from_seconds(static_cast<double>(i) * stepDt);
    poseStamped.pose.position.x = pose.getPosition().x();
    poseStamped.pose.position.y = pose.getPosition().y();
    poseStamped.pose.position.z = pose.getPosition().z();
    poseStamped.pose.orientation.x = pose.getOrientation().x();
    poseStamped.pose.orientation.y = pose.getOrientation().y();
    poseStamped.pose.orientation.z = pose.getOrientation().z();
    poseStamped.pose.orientation.w = pose.getOrientation().w();
    path.poses.push_back(poseStamped);
  }

  // Create JointTrajectory from vector of joint angles
  trajectory_msgs::msg::JointTrajectory jointTrajectory;
  jointTrajectory.header.stamp = t0;
  for (size_t i = 0; i < planningResult.jointAngles.size(); ++i) {
    const auto& joints = planningResult.jointAngles[i];
    trajectory_msgs::msg::JointTrajectoryPoint jtp;
    jtp.positions = joints;
    jtp.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * stepDt);
    jointTrajectory.points.push_back(jtp);
  }

  // Create EE Velocity Trajectory
  std::vector<geometry_msgs::msg::TwistStamped> eeVelocityTrajectory;
  eeVelocityTrajectory.reserve(planningResult.twists.size());
  for (size_t i = 0; i < planningResult.twists.size(); ++i) {
    const auto& twist = planningResult.twists[i];
    geometry_msgs::msg::TwistStamped eeVel;
    eeVel.header.frame_id = this->fixedFrame;
    eeVel.header.stamp = t0 + rclcpp::Duration::from_seconds(static_cast<double>(i) * stepDt);
    eeVel.twist.linear.x = twist.getLinearVelocity().x();
    eeVel.twist.linear.y = twist.getLinearVelocity().y();
    eeVel.twist.linear.z = twist.getLinearVelocity().z();
    eeVel.twist.angular.x = twist.getAngularVelocity().x();
    eeVel.twist.angular.y = twist.getAngularVelocity().y();
    eeVel.twist.angular.z = twist.getAngularVelocity().z();
    eeVelocityTrajectory.push_back(eeVel);
  }

  // Publish the planned path and fill in the response
  this->plannedEndEffectorPathPub->publish(path);
  response->end_effector_path = path;
  response->joint_trajectory = jointTrajectory;
  response->end_effector_velocity_trajectory = eeVelocityTrajectory;
  response->success = planningResult.success;
  RCLCPP_INFO(this->get_logger(),
    "Planning finished: success=%s, waypoints=%zu, joint_points=%zu, velocities=%zu",
    response->success ? "true" : "false",
    response->end_effector_path.poses.size(),
    response->joint_trajectory.points.size(),
    response->end_effector_velocity_trajectory.size()
  );
  // Right before exiting, update the query pose to the start of the planned path
  // to visualize the planned trajectory from the new query pose
  if (response->success) {
    RCLCPP_INFO(this->get_logger(),
      "Updating query pose to start of planned path at pos=(%.3f, %.3f, %.3f)",
      startSV.getPosition().x(), startSV.getPosition().y(), startSV.getPosition().z()
    );
    this->queryPose = startSV;
  }
  if (!response->success) {
    // If planning failed, log final pose and distance to goal
    const auto& finalPose = planningResult.poses.back();
    const Eigen::Vector3d goalPos = this->pField->getGoalPose().getPosition();
    const double distanceToGoal = (finalPose.getPosition() - goalPos).norm();
    RCLCPP_WARN(this->get_logger(),
      "Planning failed to reach goal. Final pose: pos=(%.3f, %.3f, %.3f), distance to goal=%.6f m",
      finalPose.getPosition().x(), finalPose.getPosition().y(), finalPose.getPosition().z(),
      distanceToGoal
    );
  }
}

PFLimits PotentialFieldManager::getPFLimits(std::shared_ptr<PotentialField> pf) {
  PFLimits limits;
  // Determine the limits of the potential field based on obstacle positions, goal position, and query pose
  auto obstacles = pf->getObstacles();
  Eigen::Vector3d goalPos = pf->getGoalPose().getPosition();
  Eigen::Vector3d queryPos = this->queryPose.getPosition();
  if (obstacles.empty()) {
    // If no obstacles, set limits around the goal position and query pose
    limits.minX = std::min(goalPos.x(), queryPos.x());
    limits.maxX = std::max(goalPos.x(), queryPos.x());
    limits.minY = std::min(goalPos.y(), queryPos.y());
    limits.maxY = std::max(goalPos.y(), queryPos.y());
    limits.minZ = std::min(goalPos.z(), queryPos.z());
    limits.maxZ = std::max(goalPos.z(), queryPos.z());
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
    // Expand limits to include the query pose if outside current bounds
    if (queryPos.x() < limits.minX) limits.minX = queryPos.x();
    if (queryPos.x() > limits.maxX) limits.maxX = queryPos.x();
    if (queryPos.y() < limits.minY) limits.minY = queryPos.y();
    if (queryPos.y() > limits.maxY) limits.maxY = queryPos.y();
    if (queryPos.z() < limits.minZ) limits.minZ = queryPos.z();
    if (queryPos.z() > limits.maxZ) limits.maxZ = queryPos.z();
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
  MarkerArray obstacleMarkers = this->createObstacleMarkers(pf);
  markerArray.markers.insert(markerArray.markers.cend(), obstacleMarkers.markers.cbegin(),
    obstacleMarkers.markers.cend());
  MarkerArray potentialVectorMarkers = this->createPotentialVectorMarkers(pf);
  markerArray.markers.insert(markerArray.markers.cend(), potentialVectorMarkers.markers.cbegin(),
    potentialVectorMarkers.markers.cend());
  MarkerArray queryPoseMarkerArray = this->createQueryPoseMarker();
  markerArray.markers.insert(markerArray.markers.cend(), queryPoseMarkerArray.markers.cbegin(),
    queryPoseMarkerArray.markers.cend());
  return markerArray;
}

MarkerArray PotentialFieldManager::createQueryPoseMarker() {
  MarkerArray markerArray;
  // Mirror goal visualization: a center sphere (blue) and 3 RGB unit arrows for +X,+Y,+Z
  const SpatialVector qp = this->queryPose;
  // Center sphere (blue)
  Marker qpSphere;
  qpSphere.header.frame_id = this->fixedFrame;
  qpSphere.header.stamp = this->now();
  qpSphere.frame_locked = true;
  qpSphere.ns = "query_pose";
  qpSphere.id = 0;
  qpSphere.type = Marker::SPHERE;
  qpSphere.action = Marker::ADD;
  qpSphere.pose.position.x = qp.getPosition().x();
  qpSphere.pose.position.y = qp.getPosition().y();
  qpSphere.pose.position.z = qp.getPosition().z();
  qpSphere.pose.orientation.x = qp.getOrientation().x();
  qpSphere.pose.orientation.y = qp.getOrientation().y();
  qpSphere.pose.orientation.z = qp.getOrientation().z();
  qpSphere.pose.orientation.w = qp.getOrientation().w();
  qpSphere.scale.x = 0.15;
  qpSphere.scale.y = 0.15;
  qpSphere.scale.z = 0.15;
  qpSphere.color.r = 0.0f;
  qpSphere.color.g = 0.0f;
  qpSphere.color.b = 1.0f; // Blue center sphere for query pose
  qpSphere.color.a = 1.0f;
  qpSphere.lifetime = rclcpp::Duration(0, 0);
  markerArray.markers.push_back(qpSphere);

  // RGB unit arrows aligned with query orientation
  for (int i = 0; i < 3; ++i) {
    Marker axis;
    axis.header.frame_id = this->fixedFrame;
    axis.header.stamp = this->now();
    axis.ns = "query_pose";
    axis.id = i + 1;
    axis.type = Marker::ARROW;
    axis.action = Marker::ADD;
    axis.frame_locked = true;
    axis.pose.position.x = qp.getPosition().x();
    axis.pose.position.y = qp.getPosition().y();
    axis.pose.position.z = qp.getPosition().z();
    // Rotate default +X arrow to +X,+Y,+Z of the query frame
    Eigen::AngleAxisd axisRotation = Eigen::AngleAxisd::Identity();
    if (i == 0) { // +X (red)
      axis.color.r = 1.0f;
      axisRotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }
    else if (i == 1) { // +Y (green)
      axis.color.g = 1.0f;
      axisRotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    }
    else { // +Z (blue)
      axis.color.b = 1.0f;
      axisRotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
    }
    Eigen::Quaterniond Q = qp.getOrientation() * Eigen::Quaterniond(axisRotation);
    axis.pose.orientation.x = Q.x();
    axis.pose.orientation.y = Q.y();
    axis.pose.orientation.z = Q.z();
    axis.pose.orientation.w = Q.w();
    // ARROW scale: x=length, y=shaft diameter, z=head diameter
    axis.scale.x = 0.5f;  // length
    axis.scale.y = 0.04f; // shaft diameter
    axis.scale.z = 0.10f; // head diameter
    axis.color.a = 0.9f;
    axis.lifetime = rclcpp::Duration(0, 0);
    markerArray.markers.push_back(axis);
  }
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
    obstacleMarker.ns = "obstacles";
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
    influenceMarker.ns = "influence_zones";
    influenceMarker.id = hashID; // mirror id for influence volume
    influenceMarker.action = Marker::ADD;
    influenceMarker.pose.position.x = position.x();
    influenceMarker.pose.position.y = position.y();
    influenceMarker.pose.position.z = position.z();
    influenceMarker.pose.orientation.x = orientation.x();
    influenceMarker.pose.orientation.y = orientation.y();
    influenceMarker.pose.orientation.z = orientation.z();
    influenceMarker.pose.orientation.w = orientation.w();
    influenceMarker.color.r = 1.0f;
    influenceMarker.color.g = 1.0f;
    influenceMarker.color.b = 0.0f;
    influenceMarker.color.a = 0.35f; // Semi-transparent
    influenceMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    const double influenceDistance = pf->getInfluenceDistance();
    switch (obstacle.getType()) {
    case ObstacleType::SPHERE: {
      // Inflated sphere diameter = 2 * (radius + influenceDistance).
      influenceMarker.type = Marker::SPHERE;
      const double influenceZoneDiameter = 2.0 * (obstacle.getGeometry().radius + influenceDistance);
      influenceMarker.scale.x = influenceZoneDiameter;
      influenceMarker.scale.y = influenceZoneDiameter;
      influenceMarker.scale.z = influenceZoneDiameter;
      break;
    }
    case ObstacleType::BOX: {
      // Inflated box dimensions = base dims + 2 * influenceDistance along each axis.
      influenceMarker.type = Marker::CUBE;
      influenceMarker.scale.x = obstacle.getGeometry().length + 2.0 * influenceDistance;
      influenceMarker.scale.y = obstacle.getGeometry().width + 2.0 * influenceDistance;
      influenceMarker.scale.z = obstacle.getGeometry().height + 2.0 * influenceDistance;
      break;
    }
    case ObstacleType::CYLINDER: {
      // Inflated cylinder: diameter = 2 * (radius + d), height = height + 2 * d.
      influenceMarker.type = Marker::CYLINDER;
      const double r = obstacle.getGeometry().radius;
      influenceMarker.scale.x = 2.0 * (r + influenceDistance);
      influenceMarker.scale.y = 2.0 * (r + influenceDistance);
      influenceMarker.scale.z = obstacle.getGeometry().height + 2.0 * influenceDistance;
      break;
    }
    case ObstacleType::MESH: {
      // Visualize the influence zone as an "inflated" version of the original mesh.
      // We approximate a Minkowski sum by scaling the mesh per-axis so its AABB grows by +2d.
      // This preserves the silhouette and is a better visual cue than a plain inflated AABB cube.
      if (!obstacle.getMeshResource().empty()) {
        influenceMarker.type = Marker::MESH_RESOURCE;
        influenceMarker.mesh_resource = obstacle.getMeshResource();
        influenceMarker.mesh_use_embedded_materials = false; // keep our semi-transparent color
        // Base axis-aligned dimensions for the mesh in obstacle frame
        const Eigen::Vector3d baseHalf = obstacle.halfDimensions();
        const Eigen::Vector3d baseDims = 2.0 * baseHalf; // L, W, H
        const Eigen::Vector3d baseScale = obstacle.getMeshScale(); // current mesh scale used for the obstacle
        auto inflateScale = [influenceDistance](double baseDim, double baseScaleVal) -> double {
          const double eps = 1e-9;
          if (std::abs(baseDim) < eps) return baseScaleVal; // degenerate axis; leave as-is
          // New scale multiplies existing mesh scale so that (scaledDim) = baseDim + 2d
          // => scaleFactor = (baseDim + 2d) / baseDim
          const double scaleFactor = (baseDim + 2.0 * influenceDistance) / baseDim;
          return baseScaleVal * scaleFactor;
        };
        influenceMarker.scale.x = inflateScale(baseDims.x(), baseScale.x());
        influenceMarker.scale.y = inflateScale(baseDims.y(), baseScale.y());
        influenceMarker.scale.z = inflateScale(baseDims.z(), baseScale.z());
      }
      else {
        // Fallback: render an inflated AABB cube if no mesh resource provided
        influenceMarker.type = Marker::CUBE;
        const Eigen::Vector3d baseHalf = obstacle.halfDimensions();
        const Eigen::Vector3d baseDims = 2.0 * baseHalf; // L, W, H
        influenceMarker.scale.x = baseDims.x() + 2.0 * influenceDistance;
        influenceMarker.scale.y = baseDims.y() + 2.0 * influenceDistance;
        influenceMarker.scale.z = baseDims.z() + 2.0 * influenceDistance;
      }
      break;
    }
    }
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
    axis.type = Marker::ARROW;
    axis.action = Marker::ADD;
    axis.frame_locked = true;
    axis.pose.position.x = goalPose.getPosition().x();
    axis.pose.position.y = goalPose.getPosition().y();
    axis.pose.position.z = goalPose.getPosition().z();
    // ARROW marker points along +X by default. Rotate it to +X,+Y,+Z of the goal frame.
    Eigen::AngleAxisd axisRotation = Eigen::AngleAxisd::Identity();
    if (i == 0) { // +X (red)
      axis.color.r = 1.0f;
      axisRotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }
    else if (i == 1) { // +Y (green) : rotate +90deg about Z to map +X → +Y
      axis.color.g = 1.0f;
      axisRotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    }
    else if (i == 2) { // +Z (blue) : rotate -90deg about Y to map +X → +Z
      axis.color.b = 1.0f;
      axisRotation = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
    }
    Eigen::Quaterniond Q = goalPose.getOrientation() * Eigen::Quaterniond(axisRotation);
    axis.pose.orientation.x = Q.x();
    axis.pose.orientation.y = Q.y();
    axis.pose.orientation.z = Q.z();
    axis.pose.orientation.w = Q.w();
    // ARROW scale: x=length, y=shaft diameter, z=head diameter
    axis.scale.x = 0.5f;  // 0.5 meter length
    axis.scale.y = 0.04f; // shaft diameter
    axis.scale.z = 0.10f; // head diameter
    axis.color.a = 0.9f; // slightly opaque
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
        TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
        const Eigen::Vector3d v = velocity.getLinearVelocity();
        const double magnitude = v.norm();
        vectorMarker.header.frame_id = this->fixedFrame;
        vectorMarker.header.stamp = this->now();
        vectorMarker.ns = "potential_vectors";
        vectorMarker.id = id++;
        vectorMarker.type = Marker::ARROW;
        vectorMarker.action = Marker::ADD;
        vectorMarker.pose.position.x = position.getPosition().x();
        vectorMarker.pose.position.y = position.getPosition().y();
        vectorMarker.pose.position.z = position.getPosition().z();
        // Set the orientation of the arrow to point in the 3D direction of the velocity vector
        Eigen::Quaterniond forceOrientation = Eigen::Quaterniond::Identity();
        if (v.norm() > 1e-9) {
          // Arrow mesh assumes +X is forward. Rotate X-axis onto v.
          forceOrientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), v.normalized());
        }
        vectorMarker.pose.orientation.x = forceOrientation.x();
        vectorMarker.pose.orientation.y = forceOrientation.y();
        vectorMarker.pose.orientation.z = forceOrientation.z();
        vectorMarker.pose.orientation.w = forceOrientation.w();
        vectorMarker.scale.x = 0.15f; // Length of the arrow
        vectorMarker.scale.y = 0.05f; // Shaft diameter
        vectorMarker.scale.z = 0.1f; // Head diameter
        // Color the arrows using a gradient depending on the magnitude of the velocity vector
        // max velocity is red and 0 is blue
        double colorScale = std::min<double>(magnitude / this->maxLinearVelocity, 1.0f);
        vectorMarker.color.r = colorScale;
        vectorMarker.color.g = 0.0f;
        vectorMarker.color.b = 1.0 - colorScale;
        vectorMarker.color.a = 0.75f; // Semi-transparent
        vectorMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
        markerArray.markers.push_back(vectorMarker);
      }
    }
  }
  return markerArray;
}

geometry_msgs::msg::Twist PotentialFieldManager::fuseTwists(
  const geometry_msgs::msg::Twist::SharedPtr twist1,
  const geometry_msgs::msg::Twist::SharedPtr twist2,
  const double alpha) {
  geometry_msgs::msg::Twist fusedTwist;

  // Alpha = 0 -> A, Alpha = 1 -> B
  auto fuseValue = [](double A, double B, double alpha) {
    return alpha * B + (1.0 - alpha) * A;
  };

  // Simple linear fusion based on alpha parameter. Missing inputs are treated as zeros.
  fusedTwist.linear.x = fuseValue(twist1->linear.x, twist2->linear.x, alpha);
  fusedTwist.linear.y = fuseValue(twist1->linear.y, twist2->linear.y, alpha);
  fusedTwist.linear.z = fuseValue(twist1->linear.z, twist2->linear.z, alpha);
  fusedTwist.angular.x = fuseValue(twist1->angular.x, twist2->angular.x, alpha);
  fusedTwist.angular.y = fuseValue(twist1->angular.y, twist2->angular.y, alpha);
  fusedTwist.angular.z = fuseValue(twist1->angular.z, twist2->angular.z, alpha);

  return fusedTwist;
}

geometry_msgs::msg::Twist clampTwist(const geometry_msgs::msg::Twist& twist, const geometry_msgs::msg::Twist& limits) {
  geometry_msgs::msg::Twist clampedTwist = twist; // Start with the input twist
  // Clamp linear velocities
  clampedTwist.linear.x = std::clamp(clampedTwist.linear.x, -limits.linear.x, limits.linear.x);
  clampedTwist.linear.y = std::clamp(clampedTwist.linear.y, -limits.linear.y, limits.linear.y);
  clampedTwist.linear.z = std::clamp(clampedTwist.linear.z, -limits.linear.z, limits.linear.z);
  // Clamp angular velocities
  clampedTwist.angular.x = std::clamp(clampedTwist.angular.x, -limits.angular.x, limits.angular.x);
  clampedTwist.angular.y = std::clamp(clampedTwist.angular.y, -limits.angular.y, limits.angular.y);
  clampedTwist.angular.z = std::clamp(clampedTwist.angular.z, -limits.angular.z, limits.angular.z);
  return clampedTwist;
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
        << obstacleTypeToString(obstacle.getType()) << "," << this->pField->getInfluenceDistance() << ","
        << this->pField->getRepulsiveGain() << ","
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
        TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
        vectors_file << x << "," << y << "," << z << ","
          << velocity.getLinearVelocity().x() << ","
          << velocity.getLinearVelocity().y() << ","
          << velocity.getLinearVelocity().z() << "\n";
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
