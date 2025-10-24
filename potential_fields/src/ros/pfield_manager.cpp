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
///
/// PUBLISHERS:
///   ~/pfield/markers (visualization_msgs::msg::MarkerArray): Markers for PF visualization in RViz

#include "ros/pfield_manager.hpp"
#include "robot_plugins/null_motion_plugin.hpp"
#include "robot_plugins/franka_plugin.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

PotentialFieldManager::PotentialFieldManager()
  : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->visualizerFrequency = this->declare_parameter("visualize_pf_frequency", 100.0f); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0f); // [Ns/m]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7f); // [Ns/m]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 1.0f); // [Ns/m]
  this->maxLinearVelocity = this->declare_parameter("max_linear_velocity", 1.0f); // [m/s]
  this->maxAngularVelocity = this->declare_parameter("max_angular_velocity", 1.0f); // [rad/s]
  this->maxLinearAcceleration = this->declare_parameter("max_linear_acceleration", 1.0f); // [m/s^2]
  this->maxAngularAcceleration = this->declare_parameter("max_angular_acceleration", 1.0f); // [rad/s^2]
  this->influenceZoneScale = this->declare_parameter("influence_zone_scale", 2.0f); // Influence zone scaling factor
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->visualizerBufferArea = this->declare_parameter("visualizer_buffer_area", 1.0f); // Extra area to visualize the PF [m]
  this->fieldResolution = this->declare_parameter("field_resolution", 0.5f); // Resolution of the potential field grid [m]
  this->urdfFileName = this->declare_parameter("urdf_file_path", "urdf/robot.urdf");
  this->eeLinkName = this->declare_parameter("end_effector_link_name", std::string()); // End-effector link name
  // Get parameters from yaml file
  this->visualizerFrequency = this->get_parameter("visualize_pf_frequency").as_double();
  this->attractiveGain = this->get_parameter("attractive_gain").as_double();
  this->rotationalAttractiveGain = this->get_parameter("rotational_attractive_gain").as_double();
  this->repulsiveGain = this->get_parameter("repulsive_gain").as_double();
  this->maxLinearVelocity = this->get_parameter("max_linear_velocity").as_double();
  this->maxAngularVelocity = this->get_parameter("max_angular_velocity").as_double();
  this->maxLinearAcceleration = this->get_parameter("max_linear_acceleration").as_double();
  this->maxAngularAcceleration = this->get_parameter("max_angular_acceleration").as_double();
  this->influenceZoneScale = this->get_parameter("influence_zone_scale").as_double();
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();
  this->visualizerBufferArea = this->get_parameter("visualizer_buffer_area").as_double();
  this->fieldResolution = this->get_parameter("field_resolution").as_double();
  this->urdfFileName = this->get_parameter("urdf_file_path").as_string();
  this->eeLinkName = this->get_parameter("end_effector_link_name").as_string();

  // Initialize the potential fields
  this->pField = std::make_shared<PotentialField>(
    this->attractiveGain, this->rotationalAttractiveGain,
    this->maxLinearVelocity, this->maxAngularVelocity,
    this->maxLinearAcceleration, this->maxAngularAcceleration
  );

  // Initialize the motion plugin
  const std::string frankaHostname = std::string();
  // this->motionPlugin = std::make_unique<NullMotionPlugin>();
  this->motionPlugin = std::make_unique<FrankaPlugin>(frankaHostname);
  RCLCPP_INFO(this->get_logger(), "Using Motion Plugin: %s", this->motionPlugin->getName().c_str());

  // Setup Pinocchio kinematic model for converting joint angles to PF Obstacles
  this->kinematicModel = PFKinematics(this->urdfFileName);
  // Log some information about the loaded model
  auto pinModel = this->kinematicModel.getModel();
  RCLCPP_INFO(this->get_logger(), "Loaded URDF model with %u joints and %u frames from file %s",
    pinModel.njoints, pinModel.nframes, this->urdfFileName.c_str());
  // Print the name of each frame
  for (const auto& frame : pinModel.frames) {
    RCLCPP_DEBUG(this->get_logger(), "\tFrame: %s", frame.name.c_str());
  }
  if (this->eeLinkName.empty()) {
    // Store end-effector link name for TF broadcasting
    this->eeLinkName = pinModel.frames[pinModel.nframes - 1].name;
  }
  RCLCPP_INFO(this->get_logger(), "Using end-effector link name: %s", this->eeLinkName.c_str());
  // Parse URDF once to build the collision catalog
  if (this->robotModel.initFile(this->urdfFileName)) {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded URDF model from file");
    this->collisionCatalog = this->buildCollisionCatalog(this->robotModel, true);
  }
  else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model");
  }
  // Save the initial joint states (home position) if IKSolver is available
  try {
    auto ikInit = this->motionPlugin->getIKSolver();
    if (!ikInit) {
      RCLCPP_WARN(this->get_logger(), "IKSolver not available at initialization; skipping initial obstacle update");
    }
    else {
      JointState initialJointState;
      initialJointState.header.stamp = this->now();
      initialJointState.header.frame_id = this->fixedFrame;
      initialJointState.name = ikInit->getJointNames();
      initialJointState.position = ikInit->getHomeConfiguration();
      this->updateObstaclesFromJointStates(initialJointState);
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during initial obstacle update: %s", e.what());
  }

  // Setup marker publisher
  // Use reliable and transient_local QoS for RViz MarkerArray publisher
  auto markerPubQos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
  this->pFieldMarkerPub = this->create_publisher<MarkerArray>("pfield/markers", markerPubQos);
  RCLCPP_INFO(this->get_logger(), "PF Markers publishing on: %s", this->pFieldMarkerPub->get_topic_name());

  // Setup goal pose subscriber
  auto goalPoseQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  this->goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pfield/planning_goal_pose",
    goalPoseQos,
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
  MarkerArray pfieldMarkers = this->visualizePF(this->pField);
  this->pFieldMarkerPub->publish(pfieldMarkers);
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
    "PlanPath request: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) delta_time=%.4f goal_tolerance=%.6f",
    request->start.pose.position.x, request->start.pose.position.y, request->start.pose.position.z,
    request->goal.pose.position.x, request->goal.pose.position.y, request->goal.pose.position.z,
    request->delta_time, request->goal_tolerance
  );

  // Save the IKSolver
  if (!this->motionPlugin) {
    RCLCPP_WARN(this->get_logger(), "motionPlugin not initialized");
    response->success = false;
    return;
  }
  auto ikSolver = this->motionPlugin->getIKSolver();
  if (!ikSolver) {
    RCLCPP_WARN(this->get_logger(), "IKSolver not available from motionPlugin");
    response->success = false;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Using IK Solver: %s", ikSolver->getName().c_str());

  auto getJointAngles = [this, ikSolver](const geometry_msgs::msg::PoseStamped& pose) -> std::vector<double> {
    // Use current robot state as seed if available
    sensor_msgs::msg::JointState js;
    geometry_msgs::msg::PoseStamped currentEEPose;
    if (!this->motionPlugin->readRobotState(js, currentEEPose)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read robot state for IK");
      return {};
    }
    std::vector<double> seed = js.position;
    std::vector<double> solution;
    // Build target isometry from the provided pose
    Eigen::Isometry3d targetIso;
    tf2::fromMsg(pose.pose, targetIso);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J;
    std::string errorMsg;
    if (!ikSolver->solve(targetIso, seed, solution, /*J=*/J, errorMsg)) {
      RCLCPP_ERROR(this->get_logger(), "IKSolver failed to find a solution: %s", errorMsg.c_str());
      return {};
    }
    return solution;
  };

  auto checkReached = [this](const geometry_msgs::msg::PoseStamped& current,
    const geometry_msgs::msg::PoseStamped& goal, double tolerance) -> bool {
    double dx = current.pose.position.x - goal.pose.position.x;
    double dy = current.pose.position.y - goal.pose.position.y;
    double dz = current.pose.position.z - goal.pose.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist <= tolerance;
  };

  auto toTwistStamped = [this](const TaskSpaceTwist& twist, const rclcpp::Time& stamp) -> geometry_msgs::msg::TwistStamped {
    geometry_msgs::msg::TwistStamped ts;
    ts.header.frame_id = this->fixedFrame;
    ts.header.stamp = stamp;
    ts.twist.linear.x = twist.getLinearVelocity().x();
    ts.twist.linear.y = twist.getLinearVelocity().y();
    ts.twist.linear.z = twist.getLinearVelocity().z();
    ts.twist.angular.x = twist.getAngularVelocity().x();
    ts.twist.angular.y = twist.getAngularVelocity().y();
    ts.twist.angular.z = twist.getAngularVelocity().z();
    return ts;
  };

  auto toPoseStamped = [this](const SpatialVector& sv, const rclcpp::Time& stamp) -> geometry_msgs::msg::PoseStamped {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = this->fixedFrame;
    ps.header.stamp = stamp;
    ps.pose.position.x = sv.getPosition().x();
    ps.pose.position.y = sv.getPosition().y();
    ps.pose.position.z = sv.getPosition().z();
    ps.pose.orientation.w = sv.getOrientation().w();
    ps.pose.orientation.x = sv.getOrientation().x();
    ps.pose.orientation.y = sv.getOrientation().y();
    ps.pose.orientation.z = sv.getOrientation().z();
    return ps;
  };

  auto buildJointStateMsg = [this](const std::vector<std::string>& jointNames,
    const std::vector<double>& jointPositions,
    const std::string& frameId,
    const rclcpp::Time& stamp) -> sensor_msgs::msg::JointState {
    sensor_msgs::msg::JointState js;
    js.header.frame_id = frameId;
    js.header.stamp = stamp;
    js.name = jointNames;
    js.position = jointPositions;
    return js;
  };

  // Build initial pose
  geometry_msgs::msg::PoseStamped currentPose = request->start;
  RCLCPP_INFO(this->get_logger(), "Starting planning from start pose (%.3f, %.3f, %.3f)",
    currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z);
  nav_msgs::msg::Path path;
  const auto baseTime = this->now();
  path.header.frame_id = this->fixedFrame; // ensure consistent frame
  path.header.stamp = baseTime;
  trajectory_msgs::msg::JointTrajectory jointTrajectory;
  jointTrajectory.header = path.header;
  jointTrajectory.joint_names = ikSolver->getJointNames();
  // Container to accumulate end-effector velocity (TwistStamped) trajectory
  std::vector<geometry_msgs::msg::TwistStamped> eeVelocityTrajectory;
  // Interpolate until goal reached or max iterations
  const size_t max_iters = 30000; // TODO(Sharwin): Parameterize this (or derive from MotionPlugin)
  size_t iter = 0;
  bool reached = false;
  // Deterministic time base: t_i = baseTime + i * delta_time
  const double dt = (request->delta_time > 0.0) ? request->delta_time : 0.1;
  std::size_t step = 0;
  const double euclidDistance = std::sqrt(
    std::pow(request->goal.pose.position.x - request->start.pose.position.x, 2) +
    std::pow(request->goal.pose.position.y - request->start.pose.position.y, 2) +
    std::pow(request->goal.pose.position.z - request->start.pose.position.z, 2)
  );
  const auto iterationsGuess = static_cast<size_t>(euclidDistance / dt);
  RCLCPP_INFO(this->get_logger(),
    "Estimated iterations to reach goal: %zu (euclid_dist=%.3f, dt=%.3f)", iterationsGuess, euclidDistance, dt
  );
  while (iter < max_iters && !reached) {
    // Check if we reached the goal within the tolerance and exit early if so
    if (checkReached(currentPose, request->goal, request->goal_tolerance)) {
      reached = true;
      break;
    }
    // Save current pose (stamped deterministically)
    {
      const rclcpp::Time stamp_i = baseTime + rclcpp::Duration::from_seconds(step * dt);
      auto stampedPose = currentPose;
      stampedPose.header.frame_id = this->fixedFrame;
      stampedPose.header.stamp = stamp_i;
      path.poses.push_back(stampedPose);
    }
    // Call IK on current pose to get current joint angles
    auto jointAngles = getJointAngles(currentPose);
    if (jointAngles.empty() && !jointTrajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "Failed to get joint angles from IK at iter=%zu for pose (%.3f, %.3f, %.3f), aborting plan",
        iter, currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z);
      response->success = false;
      return;
    }
    // Save joint angles for JointTrajectory
    trajectory_msgs::msg::JointTrajectoryPoint jointTrajectoryPoint;
    jointTrajectoryPoint.positions = jointTrajectory.points.empty() ? ikSolver->getHomeConfiguration() : jointAngles;
    // Deterministic time_from_start aligned with step index
    jointTrajectoryPoint.time_from_start = rclcpp::Duration::from_seconds(step * dt);
    jointTrajectory.points.push_back(jointTrajectoryPoint);
    // Publish joint angles to planned joint state for PFManager to update its internal PF
    this->updateObstaclesFromJointStates(
      buildJointStateMsg(
        jointTrajectory.joint_names,
        jointTrajectoryPoint.positions,
        this->fixedFrame,
        baseTime + rclcpp::Duration::from_seconds(step * dt)
      )
    );
    // Evaluate the velocity at the current pose, and use it and the previous twist to apply motion constraints
    SpatialVector currentPoseSV(
      Eigen::Vector3d(
        currentPose.pose.position.x,
        currentPose.pose.position.y,
        currentPose.pose.position.z
      ),
      Eigen::Quaterniond(
        currentPose.pose.orientation.w, currentPose.pose.orientation.x,
        currentPose.pose.orientation.y, currentPose.pose.orientation.z
      )
    );
    TaskSpaceTwist twistAtCurrentPose = this->pField->evaluateVelocityAtPose(currentPoseSV);
    TaskSpaceTwist prevTwist = eeVelocityTrajectory.empty() ? TaskSpaceTwist() : TaskSpaceTwist(
      Eigen::Vector3d(
        eeVelocityTrajectory.back().twist.linear.x,
        eeVelocityTrajectory.back().twist.linear.y,
        eeVelocityTrajectory.back().twist.linear.z
      ),
      Eigen::Vector3d(
        eeVelocityTrajectory.back().twist.angular.x,
        eeVelocityTrajectory.back().twist.angular.y,
        eeVelocityTrajectory.back().twist.angular.z
      )
    );
    TaskSpaceTwist constrainedTwist = this->pField->applyMotionConstraints(twistAtCurrentPose, prevTwist, request->delta_time);
    Eigen::Vector3d nextPosition = this->pField->integrateLinearVelocity(
      currentPoseSV.getPosition(), constrainedTwist.getLinearVelocity(), request->delta_time);
    Eigen::Quaterniond nextOrientation = this->pField->integrateAngularVelocity(
      currentPoseSV.getOrientation(), constrainedTwist.getAngularVelocity(), request->delta_time);
    SpatialVector nextPose(nextPosition, nextOrientation);
    if (iter % (iterationsGuess / 15) == 0) {
      RCLCPP_DEBUG(this->get_logger(),
        "Planning iter=%zu: path_len=%zu, joint_points=%zu", iter, path.poses.size(), jointTrajectory.points.size());
      RCLCPP_DEBUG(this->get_logger(), "iter=%zu autonomy linear=(%.4f, %.4f, %.4f) next_pos=(%.4f, %.4f, %.4f)", iter,
        constrainedTwist.getLinearVelocity().x(),
        constrainedTwist.getLinearVelocity().y(),
        constrainedTwist.getLinearVelocity().z(),
        nextPose.getPosition().x(), nextPose.getPosition().y(), nextPose.getPosition().z());
    }
    // Store the autonomy vector (end-effector velocity) with deterministic stamp
    const rclcpp::Time stamp_i = baseTime + rclcpp::Duration::from_seconds(step * dt);
    eeVelocityTrajectory.push_back(toTwistStamped(constrainedTwist, stamp_i));
    // Update the current pose before moving to next iteration; stamp next pose for continuity
    const rclcpp::Time stamp_next = baseTime + rclcpp::Duration::from_seconds((step + 1) * dt);
    currentPose = toPoseStamped(nextPose, stamp_next);
    // Move our step along before the next iteration
    ++step;
    ++iter;
  }
  this->plannedEndEffectorPathPub->publish(path);
  response->end_effector_path = path;
  response->joint_trajectory = jointTrajectory;
  // Move accumulated EE velocity trajectory into the response
  response->end_effector_velocity_trajectory = eeVelocityTrajectory;
  response->success = reached;
  RCLCPP_INFO(this->get_logger(),
    "Planning finished: success=%s, waypoints=%zu, joint_points=%zu, velocities=%zu, iterations=%zu",
    response->success ? "true" : "false",
    response->end_effector_path.poses.size(),
    response->joint_trajectory.points.size(), response->end_effector_velocity_trajectory.size(), iter);
  if (reached) RCLCPP_INFO(this->get_logger(), "Plan path succeeded in %zu iterations", iter);
  else if (iter >= max_iters) RCLCPP_WARN(this->get_logger(), "Plan path reached iteration limit without reaching goal");
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
    obstacleMarker.ns = "obstacle_" + obstacleTypeToString(obstacle.getType());
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
    influenceMarker.ns = "obstacle_influence_" + obstacleTypeToString(obstacle.getType());
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
      // Represent mesh influence by rendering the same mesh, uniformly scaled by the influence factor
      influenceMarker.type = Marker::MESH_RESOURCE;
      influenceMarker.mesh_resource = obstacle.getMeshResource();
      influenceMarker.mesh_use_embedded_materials = false; // use our color/alpha below
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
        TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
        double magnitude = velocity.getLinearVelocity().norm();
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
        const auto unitDirectionVector = velocity.getLinearVelocity().normalized();
        double yaw = std::atan2(unitDirectionVector.y(), unitDirectionVector.x());
        vectorMarker.pose.orientation = PotentialFieldManager::getQuaternionFromYaw(yaw);
        vectorMarker.scale.x = 0.15f; // Length of the arrow
        vectorMarker.scale.y = 0.05f; // Shaft diameter
        vectorMarker.scale.z = 0.1f; // Head diameter
        // Color the arrows using a gradient depending on
        // the magnitude of the velocity vector
        // max velocity is red and 0 is blue
        double colorScale = std::min<double>(magnitude / this->maxLinearVelocity, 1.0f);
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

std::vector<CollisionCatalogEntry> PotentialFieldManager::buildCollisionCatalog(urdf::Model& model, bool logCatalog) {
  std::vector<CollisionCatalogEntry> catalog;
  this->obstacleGeometryTemplates.clear();
  for (const auto& [link_name, link] : model.links_) {
    if (!link) { continue; }
    RCLCPP_INFO(this->get_logger(), "Processing link: %s", link_name.c_str());
    // Lambda helper to add a collision object to the catalog with the correct naming
    auto addCollisionEntry = [this, &link_name, &catalog](const urdf::CollisionSharedPtr& col_ptr, size_t index) {
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
      // Build cached obstacle geometry templates aligned to catalog entries
      PotentialFieldObstacle templateObstacle = this->obstacleFromCollisionObject(
        e.id, *e.col, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
      );
      this->obstacleGeometryTemplates.push_back(templateObstacle);
      catalog.push_back(std::move(e));
      this->collisionLinkNames.push_back(link_name);
    };

    // Add collision objects, specifying the index for collision array objects
    if (!link->collision_array.empty()) {
      for (size_t i = 0; i < link->collision_array.size(); ++i) {
        addCollisionEntry(link->collision_array[i], i);
      }
    }
    else if (link->collision) {
      addCollisionEntry(link->collision, 0);
    }
  }
  if (logCatalog) {
    // Emit a one-time detailed listing of expected collision-derived obstacles
    for (const auto& entry : catalog) {
      if (!entry.col || !entry.col->geometry) {
        RCLCPP_WARN(this->get_logger(), "Collision entry '%s' has no geometry", entry.id.c_str());
        continue;
      }
      std::string gType = "Unknown";
      if (dynamic_cast<urdf::Box*>(entry.col->geometry.get())) gType = "Box";
      else if (dynamic_cast<urdf::Sphere*>(entry.col->geometry.get())) gType = "Sphere";
      else if (dynamic_cast<urdf::Cylinder*>(entry.col->geometry.get())) gType = "Cylinder";
      else if (dynamic_cast<urdf::Mesh*>(entry.col->geometry.get())) gType = "Mesh";
      RCLCPP_INFO(this->get_logger(), "\tCatalog: id=%s link=%s type=%s",
        entry.id.c_str(), entry.linkName.c_str(), gType.c_str()
      );
    }
    RCLCPP_INFO(this->get_logger(), "Robot has %zu links", model.links_.size());
    RCLCPP_INFO(this->get_logger(), "Robot has %zu collision objects in catalog", catalog.size());
  }
  return catalog;
}

std::vector<PotentialFieldObstacle> PotentialFieldManager::buildObstaclesFromTransforms(
  const std::vector<Eigen::Affine3d>& transforms) {
  std::vector<PotentialFieldObstacle> collisionObstacles;
  collisionObstacles.reserve(this->collisionCatalog.size());
  // Each catalog entry aligns with collisionLinkNames and templates
  size_t N = this->collisionCatalog.size();
  for (size_t i = 0; i < N; ++i) {
    const auto& entry = this->collisionCatalog[i];
    // Find link index corresponding to this entry
    // We built collisionLinkNames in the same order as catalog push_back; i aligns with link name index.
    const Eigen::Affine3d& world_T_link = transforms[i];
    const urdf::Pose& origin = entry.col->origin;
    Eigen::Affine3d link_T_col =
      Eigen::Translation3d(origin.position.x, origin.position.y, origin.position.z) *
      Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    Eigen::Affine3d world_T_col = world_T_link * link_T_col;
    // Clone template and update pose only
    PotentialFieldObstacle obst = this->obstacleGeometryTemplates[i];
    obst.setPose(
      Eigen::Vector3d(
        world_T_col.translation().x(),
        world_T_col.translation().y(),
        world_T_col.translation().z()
      ),
      Eigen::Quaterniond(world_T_col.rotation())
    );
    collisionObstacles.push_back(obst);
  }
  return collisionObstacles;
}

PotentialFieldObstacle PotentialFieldManager::obstacleFromCollisionObject(
  const std::string& frameID, const urdf::Collision& collisionObject,
  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  ObstacleType obstacleType;
  double radius = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  bool isMesh = false;
  std::string meshResource;

  auto* geometry = collisionObject.geometry.get();
  if (urdf::Box* b = dynamic_cast<urdf::Box*>(geometry)) {
    obstacleType = ObstacleType::BOX;
    length = b->dim.x;
    width = b->dim.y;
    height = b->dim.z;
  }
  else if (urdf::Sphere* s = dynamic_cast<urdf::Sphere*>(geometry)) {
    obstacleType = ObstacleType::SPHERE;
    radius = s->radius;
  }
  else if (urdf::Cylinder* c = dynamic_cast<urdf::Cylinder*>(geometry)) {
    obstacleType = ObstacleType::CYLINDER;
    radius = c->radius;
    height = c->length;
  }
  else if (urdf::Mesh* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    obstacleType = ObstacleType::MESH;
    length = m->scale.x;
    width = m->scale.y;
    height = m->scale.z;
    isMesh = true;
    meshResource = m->filename;
  }
  else {
    RCLCPP_ERROR(this->get_logger(),
      "Unhandled URDF geometry type for collision object with id: %s", frameID.c_str());
  }
  ObstacleGeometry obstacleGeom(radius, length, width, height);
  PotentialFieldObstacle obstacle(
    frameID, position, orientation, obstacleType, ObstacleGroup::ROBOT,
    obstacleGeom, this->influenceZoneScale, this->repulsiveGain
  );
  // If mesh, set mesh resource and scale
  if (isMesh) { obstacle.setMeshProperties(meshResource, Eigen::Vector3d(length, width, height)); }
  return obstacle;
}

void PotentialFieldManager::updateObstaclesFromJointStates(JointState jointStateMsg) {
  if (!this->kinematicsCachesInitialized) {
    // Initialize caches with the runtime joint order on first message
    this->kinematicModel.initializeCaches(jointStateMsg.name, this->collisionLinkNames);
    this->kinematicsCachesInitialized = true;
    RCLCPP_INFO(this->get_logger(), "Initialized kinematics caches with %zu joints and %zu links",
      jointStateMsg.name.size(), this->collisionLinkNames.size());
  }
  std::vector<Eigen::Affine3d> transforms;
  transforms.reserve(this->collisionLinkNames.size());
  this->kinematicModel.computeLinkTransforms(jointStateMsg.position, transforms);
  this->latestTransforms = transforms; // Store latest transforms for TF broadcasting
  std::vector<PotentialFieldObstacle> obsArray = this->buildObstaclesFromTransforms(transforms);
  this->pField->addObstacles(obsArray);
  RCLCPP_DEBUG(this->get_logger(), "Updated %zu obstacles from new joint angles", obsArray.size());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
