// Copyright 2025 Sharwin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/// @file pfield_manager.cpp
/// @brief Manages a PotentialField instance and visualizes obstacles, the goal, and planned paths.
/// @author Sharwin Patil
/// @date December 12, 2025
/// @details
///   ROS 2 node wrapping the potential field library. It:
///   - Publishes RViz markers for goal, obstacles, influence zones, and velocity vectors
///   - Integrates a live "query pose" each timer tick using opposing-force removal and motion constraints
///   - Selects a MotionPlugin (e.g., Null, Franka) and assigns its IKSolver to the PotentialField
///   - Optionally initializes kinematics from a URDF and adapts influence distance based on robot extent
///
/// PARAMETERS:
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
///   pfield/planning_goal_pose DefaultQ(geometry_msgs::msg::Pose): Updates the PF goal pose
///   pfield/query_pose (geometry_msgs::msg::Pose): Sets the live query pose used for visualization
///   pfield/obstacles (potential_fields_interfaces::msg::ObstacleArray): Adds/updates external PF obstacles
///
/// PUBLISHERS:
///   pfield/markers (visualization_msgs::msg::MarkerArray): RViz markers (reliable + transient_local QoS)
///   pfield/planned_path (nav_msgs::msg::Path): Planned end-effector path

#include "ros/pfield_manager.hpp"


/// @brief Converts an HSV color to RGB format.
/// @param hue        Hue [0, 360) degrees.
/// @param saturation Saturation in [0, 1].
/// @param value      Value (brightness) in [0, 1].
/// @return RGB components in [0, 1].
static std::array<double, 3> convertHSVToRGB(double hue, double saturation, double value) {
  const double c = value * saturation;
  const double x = c * (1.0 - std::fabs(std::fmod(hue / 60.0, 2.0) - 1.0));
  const double m = value - c;
  double rp{}, gp{}, bp{};
  if (hue < 60.0) { rp = c; gp = x; bp = 0; }
  else if (hue < 120.0) { rp = x; gp = c; bp = 0; }
  else if (hue < 180.0) { rp = 0; gp = c; bp = x; }
  else if (hue < 240.0) { rp = 0; gp = x; bp = c; }
  else if (hue < 300.0) { rp = x; gp = 0; bp = c; }
  else { rp = c; gp = 0; bp = x; }
  return {rp + m, gp + m, bp + m};
}

#include "robot_plugins/null_motion_plugin.hpp"
#include "robot_plugins/franka_plugin.hpp"
#include "robot_plugins/xarm_plugin.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "pfield/mesh_collision.hpp"
#include "pfield/pf_obstacle_geometry.hpp"

PotentialFieldManager::PotentialFieldManager() : Node("potential_field_manager") {
  RCLCPP_INFO(this->get_logger(), "PotentialFieldManager Initialized");
  // Declare parameters
  this->visualizerFrequency = this->declare_parameter("visualize_pf_frequency", 60.0); // [Hz]
  this->attractiveGain = this->declare_parameter("attractive_gain", 1.0); // [Ns/m]
  this->rotationalAttractiveGain = this->declare_parameter("rotational_attractive_gain", 0.7); // [Ns/m]
  this->repulsiveGain = this->declare_parameter("repulsive_gain", 0.1); // [Ns/m]
  this->maxLinearVelocity = this->declare_parameter("max_linear_velocity", 1.0); // [m/s]
  this->maxAngularVelocity = this->declare_parameter("max_angular_velocity", 1.0); // [rad/s]
  this->maxLinearAcceleration = this->declare_parameter("max_linear_acceleration", 1.0); // [m/s^2]
  this->maxAngularAcceleration = this->declare_parameter("max_angular_acceleration", 1.0); // [rad/s^2]
  this->influenceDistance = this->declare_parameter("influence_distance", 1.0); // Influence distance for obstacle repulsion
  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  this->eeFrame = this->declare_parameter("end_effector_frame", "ee_link"); // End-effector frame name
  this->visualizerBufferArea = this->declare_parameter("visualizer_buffer_area", 1.0); // Extra area to visualize the PF [m]
  this->fieldResolution = this->declare_parameter("field_resolution", 0.5); // Resolution of the potential field grid [m]
  this->visualizeFieldVectors = this->declare_parameter("visualize_field_vectors", false); // Whether to visualize the PF vectors
  this->urdfFileName = this->declare_parameter("urdf_file_path", std::string());
  this->motionPluginType = this->declare_parameter("motion_plugin_type", std::string()); // Motion Plugin Type [e.g. "franka"]
  this->visualizationMethod = this->declare_parameter("visualization_method", std::string()); // Method for how query pose looks
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
  this->eeFrame = this->get_parameter("end_effector_frame").as_string();
  this->visualizerBufferArea = this->get_parameter("visualizer_buffer_area").as_double();
  this->fieldResolution = this->get_parameter("field_resolution").as_double();
  this->visualizeFieldVectors = this->get_parameter("visualize_field_vectors").as_bool();
  this->urdfFileName = this->get_parameter("urdf_file_path").as_string();
  this->motionPluginType = this->get_parameter("motion_plugin_type").as_string();
  this->visualizationMethod = this->get_parameter("visualization_method").as_string();

  // Initialize the potential fields
  this->pField = std::make_shared<pfield::PotentialField>(
    this->attractiveGain, this->repulsiveGain, this->rotationalAttractiveGain,
    this->maxLinearVelocity, this->maxAngularVelocity,
    this->maxLinearAcceleration, this->maxAngularAcceleration,
    this->influenceDistance
  );
  this->pField->enableDynamicQuadraticThreshold(false);
  this->pField->setQuadraticThreshold(1.0);

  // Display PF Parameters
  RCLCPP_INFO(this->get_logger(),
    "PF Parameters:\n"
    "\tAttractive Gain: %.3f [Ns/m]\n"
    "\tRotational Attractive Gain: %.3f [Ns·m/rad]\n"
    "\tRepulsive Gain: %.3f [Ns/m]\n"
    "\tMax Linear Velocity: %.3f [m/s]\n"
    "\tMax Angular Velocity: %.3f [rad/s]\n"
    "\tMax Linear Acceleration: %.3f [m/s^2]\n"
    "\tMax Angular Acceleration: %.3f [rad/s^2]\n"
    "\tInfluence Distance: %.3f [m]\n"
    "\tQuadratic Threshold: %.3f [m]\n",
    this->attractiveGain,
    this->rotationalAttractiveGain,
    this->repulsiveGain,
    this->maxLinearVelocity,
    this->maxAngularVelocity,
    this->maxLinearAcceleration,
    this->maxAngularAcceleration,
    this->influenceDistance,
    this->pField->getQuadraticThreshold()
  );

  // Initialize the query visualization method
  std::transform(
    this->visualizationMethod.cbegin(),
    this->visualizationMethod.cend(),
    this->visualizationMethod.begin(),
    [](unsigned char c) { return std::tolower(c); }
  );
  if (this->visualizationMethod.empty()
    || this->visualizationMethod != "task_space"
    || this->visualizationMethod != "whole_body_velocity") {
    this->visualizationMethod = "task_space";
  }

  // Initialize the motion plugin
  std::transform(
    this->motionPluginType.cbegin(),
    this->motionPluginType.cend(),
    this->motionPluginType.begin(),
    [](unsigned char c) { return std::tolower(c); }
  );
  if (this->motionPluginType.empty() || this->motionPluginType == "null") {
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
  if (this->motionPluginType.empty() || this->motionPluginType == "null") {
    this->ikSolver = nullptr;
    RCLCPP_INFO(this->get_logger(), "Motion plugin is null; using internal Pinocchio IK solver.");
  }
  else {
    this->ikSolver = this->motionPlugin->getIKSolver();
  }

  if (!this->ikSolver && !this->motionPluginType.empty() && this->motionPluginType != "null") {
    RCLCPP_WARN(this->get_logger(), "IKSolver not available from motionPlugin");
  }
  else if (this->ikSolver) {
    RCLCPP_INFO(this->get_logger(), "Using IKSolver: %s", this->ikSolver->getName().c_str());
  }

  // Once IKSolver is initialized, assign it to the PF instance
  this->pField->setIKSolver(this->ikSolver);

  // Allow the user to not use a URDF
  if (!this->urdfFileName.empty() && this->urdfFileName.ends_with(".urdf")) {
    try {
      this->pField->initializeKinematics(this->urdfFileName, this->eeFrame);
      RCLCPP_INFO(this->get_logger(), "PF Kinematics initialized from URDF: %s", this->urdfFileName.c_str());
      RCLCPP_INFO(this->get_logger(),
        "PF Kinematics estimated influence distance from robot extend to be: %.4f [m]", this->pField->getInfluenceDistance());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize PF Kinematics from URDF: %s", e.what());
    }
  }
  else {
    RCLCPP_WARN(this->get_logger(), "URDF file path is empty. Kinematics not initialized.");
  }

  // Setup marker publisher
  // Use reliable and transient_local QoS for RViz MarkerArrayMsg publisher
  auto markerPubQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().transient_local();
  this->pFieldMarkerPub = this->create_publisher<MarkerArrayMsg>("pfield/markers", markerPubQos);
  RCLCPP_INFO(this->get_logger(), "PF Markers publishing on '%s' at %.1f Hz", this->pFieldMarkerPub->get_topic_name(),
    this->visualizerFrequency
  );

  // Setup goal pose subscriber
  auto goalPoseQos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  this->goalPoseSub = this->create_subscription<PoseMsg>("pfield/planning_goal_pose",
    goalPoseQos,
    [this](const PoseMsg::SharedPtr msg) {
    const pfield::SpatialVector goalPose(
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
    this->pField->setGoalPose(goalPose);
  }
  );

  // Setup obstacle subscriber for external obstacles
  auto obstacleSubQos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
  this->obstacleSub = this->create_subscription<ObstacleArrayMsg>("pfield/obstacles", obstacleSubQos,
    [this](const ObstacleArrayMsg::SharedPtr msg) {
    const auto& obstacles = msg->obstacles;
    for (const auto& obst : obstacles) {
      // Build the typed geometry from the message's flat parameters
      std::unique_ptr<pfield::ObstacleGeometry> geom;
      const pfield::ObstacleType obstType = pfield::stringToObstacleType(obst.type);
      switch (obstType) {
      case pfield::ObstacleType::SPHERE:
        geom = std::make_unique<pfield::SphereGeometry>(obst.radius);
        break;
      case pfield::ObstacleType::BOX:
        geom = std::make_unique<pfield::BoxGeometry>(obst.length, obst.width, obst.height);
        break;
      case pfield::ObstacleType::CYLINDER:
        geom = std::make_unique<pfield::CylinderGeometry>(obst.radius, obst.height);
        break;
      case pfield::ObstacleType::ELLIPSOID:
        geom = std::make_unique<pfield::EllipsoidGeometry>(obst.semi_x, obst.semi_y, obst.semi_z);
        break;
      case pfield::ObstacleType::CAPSULE:
        geom = std::make_unique<pfield::CapsuleGeometry>(obst.radius, obst.height);
        break;
      case pfield::ObstacleType::MESH:
        geom = std::make_unique<pfield::MeshGeometry>(nullptr);
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown obstacle type: %s", obst.type.c_str());
        continue;
      }
      pfield::PotentialFieldObstacle obstacle(
        obst.frame_id,
        Eigen::Vector3d(obst.pose.position.x, obst.pose.position.y, obst.pose.position.z),
        Eigen::Quaterniond(obst.pose.orientation.w, obst.pose.orientation.x, obst.pose.orientation.y, obst.pose.orientation.z),
        pfield::stringToObstacleGroup(obst.group),
        std::move(geom),
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
  this->queryPoseSub = this->create_subscription<PoseMsg>("pfield/query_pose",
    queryPoseQos,
    [this](const PoseMsg::SharedPtr msg) {
    const pfield::SpatialVector newQuery(
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
    Eigen::Vector3d eulerAngles = newQuery.getOrientation().toRotationMatrix().eulerAngles(0, 1, 2);
    const double roll = eulerAngles[0];
    const double pitch = eulerAngles[1];
    const double yaw = eulerAngles[2];
    RCLCPP_INFO(
      this->get_logger(),
      "Query Pose set: pos=(%.2f, %.2f, %.2f) [m], RPY=(%.2f, %.2f, %.2f) [rad]",
      newQuery.getPosition().x(), newQuery.getPosition().y(), newQuery.getPosition().z(),
      roll, pitch, yaw
    );
    this->queryPose = newQuery;
  }
  );

  // Setup planned end-effector path publisher
  this->plannedEndEffectorPathPub = this->create_publisher<PathMsg>("pfield/planned_path", 10);
  RCLCPP_INFO(this->get_logger(), "Planned EE path publishing on: %s", this->plannedEndEffectorPathPub->get_topic_name());

  // Create service to compute the autonomy vector at a given pose
  this->autonomyVectorService = this->create_service<ComputeAutonomyVectorSrv>(
    "pfield/compute_autonomy_vector",
    std::bind(&PotentialFieldManager::handleComputeAutonomyVector, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create the PlanPath service
  this->pathPlanningService = this->create_service<PlanPathSrv>(
    "pfield/plan_path",
    std::bind(&PotentialFieldManager::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Create a CSV file to store the potential field data for python to plot
  // std::string filename = "pfield_data";
  // this->exportFieldDataToCSV(filename);

  // Initialize currentJointAngles and queryPose
  this->queryPose = pfield::SpatialVector();
  this->currentJointAngles.resize(this->pField->getNumJoints(), 0.0);

  // Run the timer for visualizing the potential field
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / this->visualizerFrequency),
    std::bind(&PotentialFieldManager::timerCallback, this)
  );
}

void PotentialFieldManager::timerCallback() {
  // Update the query pose position based on integrating the PF velocity
  this->integrateQueryPoseFromField();
  MarkerArrayMsg pfieldMarkers = this->visualizePF(this->pField);
  this->pFieldMarkerPub->publish(pfieldMarkers);
}

void PotentialFieldManager::integrateQueryPoseFromField() {
  // Advance pose using constrained interpolation (acceleration limits applied internally)
  const double dt = this->now().seconds() - this->lastQueryUpdate.seconds();
  this->lastQueryUpdate = this->now();

  // Use the helper function to perform integration
  // We use "task_space" as the default method for visualization unless specified otherwise
  auto [nextPose, twist] = this->pField->stepIntegration(
    this->queryPose,
    this->prevQueryTwist,
    dt,
    this->visualizationMethod,
    this->currentJointAngles
    // prevJointVelocities defaults to empty
  );

  // Update the query pose
  this->queryPose = nextPose;
  // Supply velocity-limited twist as previous for next acceleration limiting
  this->prevQueryTwist = twist;
}

void PotentialFieldManager::handleComputeAutonomyVector(
  const ComputeAutonomyVectorSrv::Request::SharedPtr request, ComputeAutonomyVectorSrv::Response::SharedPtr response) {
  // Compute the autonomy vector at the given pose
  pfield::SpatialVector queryPose(
    Eigen::Vector3d(
      request->query_pose.pose.position.x,
      request->query_pose.pose.position.y,
      request->query_pose.pose.position.z
    ),
    Eigen::Quaterniond(
      request->query_pose.pose.orientation.w, request->query_pose.pose.orientation.x,
      request->query_pose.pose.orientation.y, request->query_pose.pose.orientation.z
    )
  );

  pfield::TaskSpaceTwist autonomyVector;
  if (request->planning_method == PlanPathSrv::Request::PLANNING_METHOD_WHOLE_BODY) {
    // Update obstacles first if joint angles are provided
    std::vector<double> jointAngles;
    if (!request->joint_angles.empty()) {
      jointAngles.assign(request->joint_angles.begin(), request->joint_angles.end());
      this->pField->updateObstaclesFromKinematics(jointAngles);
    }
    else {
      // Fallback to current joint angles if not provided
      jointAngles = this->currentJointAngles;
      RCLCPP_WARN(this->get_logger(), "Whole-body planning requested but no joint angles provided. Using current state.");
    }
    const std::vector<double> prevJointVelocities(request->prev_joint_velocities.cbegin(), request->prev_joint_velocities.cend());
    autonomyVector = this->pField->evaluateWholeBodyTaskSpaceTwistAtConfiguration(
      jointAngles, prevJointVelocities, queryPose, request->delta_time
    );
  }
  else {
    // Default to task_space
    // Update obstacles if joint angles are provided (optional but good for consistency)
    if (!request->joint_angles.empty()) {
      std::vector<double> jointAngles(request->joint_angles.begin(), request->joint_angles.end());
      this->pField->updateObstaclesFromKinematics(jointAngles);
    }
    autonomyVector = this->pField->evaluateLimitedVelocityAtPose(queryPose);
  }

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
  RCLCPP_DEBUG(
    this->get_logger(),
    "Autonomy vector computed at pose: pos=(%.2f, %.2f, %.2f), RPY=(%.2f, %.2f, %.2f)",
    queryPose.getPosition().x(), queryPose.getPosition().y(), queryPose.getPosition().z(),
    roll, pitch, yaw
  );
}

void PotentialFieldManager::handlePlanPath(
  const PlanPathSrv::Request::SharedPtr request, PlanPathSrv::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(),
    "PlanPath request: start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f) delta_time=%.4f goal_tolerance=%.6f max_steps=%d",
    request->start.position.x, request->start.position.y, request->start.position.z,
    request->goal.position.x, request->goal.position.y, request->goal.position.z,
    request->delta_time, request->goal_tolerance, request->max_iterations
  );
  // Create SpatialVector for start pose
  auto startSV = pfield::SpatialVector(
    Eigen::Vector3d(
      request->start.position.x,
      request->start.position.y,
      request->start.position.z
    ),
    Eigen::Quaterniond(
      request->start.orientation.w,
      request->start.orientation.x,
      request->start.orientation.y,
      request->start.orientation.z
    )
  );
  // Update the PF goal pose from the request
  auto goalSV = pfield::SpatialVector(
    Eigen::Vector3d(
      request->goal.position.x,
      request->goal.position.y,
      request->goal.position.z
    ),
    Eigen::Quaterniond(
      request->goal.orientation.w,
      request->goal.orientation.x,
      request->goal.orientation.y,
      request->goal.orientation.z
    )
  );
  this->pField->setGoalPose(goalSV);
  // Convert starting_joint_angles (std::vector<float>) to std::vector<double> expected by planning methods
  std::vector<double> startJointAnglesDouble(
    request->starting_joint_angles.cbegin(), request->starting_joint_angles.cend()
  );
  if (startJointAnglesDouble.empty()) {
    RCLCPP_WARN(this->get_logger(), "No starting joint angles provided in PlanPath request. Using IK");
    startJointAnglesDouble = this->pField->computeInverseKinematics(startSV, this->currentJointAngles);
    std::string jointAnglesStr;
    for (size_t i = 0; i < startJointAnglesDouble.size(); ++i) {
      jointAnglesStr += std::to_string(startJointAnglesDouble[i]);
      if (i < startJointAnglesDouble.size() - 1) {
        jointAnglesStr += ", ";
      }
    }
    RCLCPP_INFO(this->get_logger(), "IK Found StartJointAngles: [%s]", jointAnglesStr.c_str());
  }
  else {
    RCLCPP_INFO(this->get_logger(), "Using provided %zu joint angles.", startJointAnglesDouble.size());
  }
  pfield::PlannedPath planningResult;
  try {
    // Plan a path using the request parameters and store the result
    if (request->planning_method == PlanPathSrv::Request::PLANNING_METHOD_TASK_SPACE) {
      RCLCPP_INFO(this->get_logger(), "Using Task-Space Wrench Planning");
      planningResult = this->pField->planPathFromTaskSpaceWrench(
        /*startPose=*/startSV,
        /*startJointAngles=*/startJointAnglesDouble,
        /*dt=*/request->delta_time,
        /*goalTolerance=*/request->goal_tolerance,
        /*maxIterations=*/request->max_iterations
      );
    }
    else if (request->planning_method == PlanPathSrv::Request::PLANNING_METHOD_WHOLE_BODY) {
      RCLCPP_INFO(this->get_logger(), "Using Whole-Body Joint Velocity Planning");
      planningResult = this->pField->planPathFromWholeBodyJointVelocities(
        /*startJointAngles=*/startJointAnglesDouble,
        /*dt=*/request->delta_time,
        /*goalTolerance=*/request->goal_tolerance,
        /*maxIterations=*/request->max_iterations
      );
    }
    else {
      // Default to Task Space Wrench
      if (!request->planning_method.empty() && request->planning_method != PlanPathSrv::Request::PLANNING_METHOD_TASK_SPACE) {
        RCLCPP_WARN(this->get_logger(), "Unknown planning method '%s'", request->planning_method.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Defaulting to Task-Space Wrench Planning");
      planningResult = this->pField->planPathFromTaskSpaceWrench(
        /*startPose=*/startSV,
        /*startJointAngles=*/startJointAnglesDouble,
        /*dt=*/request->delta_time,
        /*goalTolerance=*/request->goal_tolerance,
        /*maxIterations=*/request->max_iterations
      );
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during path planning: %s", e.what());
    response->success = false;
    response->error_message = e.what();
    return;
  }

  // Establish a consistent time base for the planned trajectory
  const double stepDt = (request->delta_time > 0.0) ? request->delta_time : 0.1;
  const rclcpp::Time t0 = this->now();

  // Create Path msg from EE Poses, stamp each pose at t0 + i*dt
  PathMsg path;
  path.header.frame_id = this->fixedFrame;
  path.header.stamp = t0;
  for (size_t i = 0; i < planningResult.poses.size(); ++i) {
    const auto& pose = planningResult.poses[i];
    PoseStampedMsg poseStamped;
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
  JointTrajectoryMsg jointTrajectory;
  jointTrajectory.header.stamp = t0;
  for (size_t i = 0; i < planningResult.jointAngles.size(); ++i) {
    JointTrajectoryPointMsg jtp;
    jtp.positions = planningResult.jointAngles[i];
    jtp.velocities = planningResult.jointVelocities[i];
    jtp.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * stepDt);
    jointTrajectory.points.push_back(jtp);
  }

  // Create EE Velocity Trajectory
  std::vector<TwistStampedMsg> eeVelocityTrajectory;
  eeVelocityTrajectory.reserve(planningResult.twists.size());
  for (size_t i = 0; i < planningResult.twists.size(); ++i) {
    const auto& twist = planningResult.twists[i];
    TwistStampedMsg eeVel;
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
  if (response->success && !planningResult.poses.empty()) {
    this->queryPose = planningResult.poses.front();
    // Update currentJointAngles to match the start of the path so IK has a good seed
    // this->currentJointAngles = startJointAnglesDouble;
    RCLCPP_INFO(this->get_logger(),
      "Updating query pose to start of planned path at pos=(%.3f, %.3f, %.3f)",
      this->queryPose.getPosition().x(), this->queryPose.getPosition().y(), this->queryPose.getPosition().z()
    );
  }
  if (!response->success) {
    // If planning failed, log final pose and distance to goal
    if (planningResult.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Planning failed with no poses generated.");
      response->error_message = "Planning failed with no poses generated.";
      return;
    }
    const auto& finalPose = planningResult.poses.back();
    const auto& goal = this->pField->getGoalPose();
    const double distanceToGoal = (finalPose.getPosition() - goal.getPosition()).norm();
    const double angleToGoal = (finalPose.angularDistance(goal));
    RCLCPP_WARN(
      this->get_logger(),
      "Planning failed to reach goal. Final pose: pos=(%.3f, %.3f, %.3f), "
      "distance to goal = %.4f / %.4f m, angle to goal = %.3f / %.3f rad",
      finalPose.getPosition().x(), finalPose.getPosition().y(), finalPose.getPosition().z(),
      distanceToGoal, request->goal_tolerance, angleToGoal, this->pField->getRotationalThreshold()
    );
    response->error_message = "Planning failed to reach goal within tolerance.";
  }
  // Create a CSV file of the planned path for analysis
  RCLCPP_INFO(this->get_logger(), "Creating planned path CSV file...");
  RCLCPP_INFO(this->get_logger(), "Planning success: %s", planningResult.success ? "true" : "false");
  try {
    const std::string csvFilePath = "data/planned_path.csv";
    if (!this->pField->createPlannedPathCSV(planningResult, csvFilePath)) {
      RCLCPP_WARN(this->get_logger(), "Failed to create planned path CSV at %s", csvFilePath.c_str());
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Planned path CSV created at %s", csvFilePath.c_str());
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while creating planned path CSV: %s", e.what());
  }
}


MarkerArrayMsg PotentialFieldManager::visualizePF(std::shared_ptr<pfield::PotentialField> pf) {
  MarkerArrayMsg markerArray;
  auto start = this->now();
  MarkerArrayMsg goalMarkerArrayMsg = this->createGoalMarker(pf);
  markerArray.markers.insert(markerArray.markers.cend(), goalMarkerArrayMsg.markers.cbegin(), goalMarkerArrayMsg.markers.cend());
  auto endGoal = this->now();
  MarkerArrayMsg obstacleMarkers = this->createObstacleMarkers(pf);
  markerArray.markers.insert(markerArray.markers.cend(), obstacleMarkers.markers.cbegin(), obstacleMarkers.markers.cend());
  auto endObstacles = this->now();
  MarkerArrayMsg queryPoseMarkerArrayMsg = this->createQueryPoseMarker();
  markerArray.markers.insert(markerArray.markers.cend(), queryPoseMarkerArrayMsg.markers.cbegin(),
    queryPoseMarkerArrayMsg.markers.cend());
  auto endQueryPose = this->now();
  MarkerArrayMsg thresholdMarkers = this->createThresholdMarkers(pf);
  // markerArray.markers.insert(markerArray.markers.cend(), thresholdMarkers.markers.cbegin(),
  //   thresholdMarkers.markers.cend());
  auto endThreshold = this->now();
  if (this->visualizeFieldVectors) {
    MarkerArrayMsg potentialVectorMarkers = this->createPotentialVectorMarkers(pf);
    markerArray.markers.insert(markerArray.markers.cend(), potentialVectorMarkers.markers.cbegin(),
      potentialVectorMarkers.markers.cend());
    auto endVectors = this->now();
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "PF Visualization Timing: Goal=%.3f ms, Obstacles=%.3f ms, Query Pose=%.3f ms, Threshold=%.3f ms, Velocity Vectors=%.3f ms",
      (endGoal - start).seconds() * 1000.0,
      (endObstacles - endGoal).seconds() * 1000.0,
      (endQueryPose - endObstacles).seconds() * 1000.0,
      (endThreshold - endQueryPose).seconds() * 1000.0,
      (endVectors - endThreshold).seconds() * 1000.0
    );
  }
  else {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "PF Visualization Timing: Goal=%.3f ms, Obstacles=%.3f ms, Query Pose=%.3f ms, Threshold=%.3f ms",
      (endGoal - start).seconds() * 1000.0,
      (endObstacles - endGoal).seconds() * 1000.0,
      (endQueryPose - endObstacles).seconds() * 1000.0,
      (endThreshold - endQueryPose).seconds() * 1000.0
    );
  }
  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createQueryPoseMarker() {
  MarkerArrayMsg markerArray;
  // Mirror goal visualization: a center sphere (blue) and 3 RGB unit arrows for +X,+Y,+Z
  const pfield::SpatialVector qp = this->queryPose;
  // Center sphere (blue)
  MarkerMsg qpSphere;
  qpSphere.header.frame_id = this->fixedFrame;
  qpSphere.header.stamp = this->now();
  qpSphere.frame_locked = true;
  qpSphere.ns = "query_pose";
  qpSphere.id = 0;
  qpSphere.type = MarkerMsg::SPHERE;
  qpSphere.action = MarkerMsg::ADD;
  qpSphere.pose.position.x = qp.getPosition().x();
  qpSphere.pose.position.y = qp.getPosition().y();
  qpSphere.pose.position.z = qp.getPosition().z();
  qpSphere.pose.orientation.x = qp.getOrientation().x();
  qpSphere.pose.orientation.y = qp.getOrientation().y();
  qpSphere.pose.orientation.z = qp.getOrientation().z();
  qpSphere.pose.orientation.w = qp.getOrientation().w();
  qpSphere.scale.x = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  qpSphere.scale.y = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  qpSphere.scale.z = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  qpSphere.color.r = 0.0f;
  qpSphere.color.g = 0.0f;
  qpSphere.color.b = 1.0f; // Blue center sphere for query pose
  qpSphere.color.a = 1.0f;
  qpSphere.lifetime = rclcpp::Duration(0, 0);
  markerArray.markers.push_back(qpSphere);

  // RGB unit arrows aligned with query orientation
  for (int i = 0; i < 3; ++i) {
    MarkerMsg axis;
    axis.header.frame_id = this->fixedFrame;
    axis.header.stamp = this->now();
    axis.ns = "query_pose";
    axis.id = i + 1;
    axis.type = MarkerMsg::ARROW;
    axis.action = MarkerMsg::ADD;
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
    axis.scale.x = this->TASK_SPACE_AXIS_LENGTH;
    axis.scale.y = this->TASK_SPACE_AXIS_SHAFT_DIAMETER;
    axis.scale.z = this->TASK_SPACE_AXIS_HEAD_DIAMETER;
    axis.color.a = 0.9f;
    axis.lifetime = rclcpp::Duration(0, 0);
    markerArray.markers.push_back(axis);
  }
  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createThresholdMarkers(std::shared_ptr<pfield::PotentialField> pf) {
  MarkerArrayMsg markerArray;
  if (!pf->isGoalSet()) { return markerArray; }
  // Create a translucent sphere representing the dStarThreshold around the goal
  const pfield::SpatialVector goalPose = pf->getGoalPose();
  MarkerMsg thresholdMarker;
  thresholdMarker.header.frame_id = this->fixedFrame;
  thresholdMarker.header.stamp = this->now();
  thresholdMarker.ns = "dstar_threshold";
  thresholdMarker.id = 0;
  thresholdMarker.type = MarkerMsg::SPHERE;
  thresholdMarker.action = MarkerMsg::ADD;
  thresholdMarker.pose.position.x = goalPose.getPosition().x();
  thresholdMarker.pose.position.y = goalPose.getPosition().y();
  thresholdMarker.pose.position.z = goalPose.getPosition().z();
  thresholdMarker.pose.orientation.x = goalPose.getOrientation().x();
  thresholdMarker.pose.orientation.y = goalPose.getOrientation().y();
  thresholdMarker.pose.orientation.z = goalPose.getOrientation().z();
  thresholdMarker.pose.orientation.w = goalPose.getOrientation().w();
  const double dStarThreshold = pf->isUsingDynamicQuadraticThreshold() ?
    pf->computeDynamicQuadraticThreshold(this->queryPose) :
    pf->getQuadraticThreshold();
  thresholdMarker.scale.x = dStarThreshold * 2.0; // Diameter
  thresholdMarker.scale.y = dStarThreshold * 2.0;
  thresholdMarker.scale.z = dStarThreshold * 2.0;
  thresholdMarker.color.r = 0.0f;
  thresholdMarker.color.g = 1.0f;
  thresholdMarker.color.b = 1.0f; // Cyan sphere for d* threshold
  thresholdMarker.color.a = 0.15f; // Semi-transparent
  thresholdMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
  markerArray.markers.push_back(thresholdMarker);
  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createObstacleMarkers(std::shared_ptr<pfield::PotentialField> pf) {
  MarkerArrayMsg markerArray;

  // Clear previous markers to prevent trails
  for (const auto& ns : {"robot_obstacles", "environment_obstacles",
    "environment_influence_zones", "robot_control_points"}) {
    MarkerMsg del;
    del.action = MarkerMsg::DELETEALL;
    del.ns = ns;
    markerArray.markers.push_back(del);
  }

  const std::vector<pfield::PotentialFieldObstacle> envObstacles = pf->getEnvObstacles();
  const std::vector<pfield::PotentialFieldObstacle> robotObstacles = pf->getRobotObstacles();

  std::vector<pfield::PotentialFieldObstacle> allObstacles;
  allObstacles.insert(allObstacles.cend(), envObstacles.begin(), envObstacles.end());
  allObstacles.insert(allObstacles.cend(), robotObstacles.begin(), robotObstacles.end());

  const MarkerArrayMsg obstacleMarkers = this->createObstaclesWithInfluenceZoneMarkerArray(
    allObstacles, pf->getInfluenceDistance());
  markerArray.markers.insert(markerArray.markers.cend(),
    obstacleMarkers.markers.begin(), obstacleMarkers.markers.end());

  const MarkerArrayMsg ghostMarkers = this->createGhostMeshOverlayMarkerArray(robotObstacles);
  markerArray.markers.insert(markerArray.markers.cend(),
    ghostMarkers.markers.begin(), ghostMarkers.markers.end());

  const MarkerArrayMsg cpMarkers = this->createRobotLinkControlPointsMarkerArray(robotObstacles);
  markerArray.markers.insert(markerArray.markers.cend(),
    cpMarkers.markers.begin(), cpMarkers.markers.end());

  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createObstaclesWithInfluenceZoneMarkerArray(
  const std::vector<pfield::PotentialFieldObstacle>& obstacles,
  double influenceDistance) {
  MarkerArrayMsg markerArray;
  std::unordered_set<int> usedIDs;

  for (const auto& obstacle : obstacles) {
    int hashID = createHashID(obstacle);
    // Ensure unique ID in this array
    while (usedIDs.count(hashID)) {
      hashID++;
    }
    usedIDs.insert(hashID);

    MarkerMsg obstacleMarker;
    obstacleMarker.header.frame_id = this->fixedFrame;
    obstacleMarker.header.stamp = this->now();
    obstacleMarker.frame_locked = true;
    if (obstacle.getGroup() == pfield::ObstacleGroup::ROBOT) {
      obstacleMarker.ns = "robot_obstacles";
    }
    else {
      obstacleMarker.ns = "environment_obstacles";
    }
    obstacleMarker.id = hashID;
    obstacleMarker.action = MarkerMsg::ADD;
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
    case pfield::ObstacleType::SPHERE: {
      // Scale is the Diameter of the Sphere
      const auto& sg = static_cast<const pfield::SphereGeometry&>(obstacle.getGeometry());
      obstacleMarker.type = MarkerMsg::SPHERE;
      obstacleMarker.scale.x = sg.radius * 2.0f;
      obstacleMarker.scale.y = sg.radius * 2.0f;
      obstacleMarker.scale.z = sg.radius * 2.0f;
      break;
    }
    case pfield::ObstacleType::BOX: {
      const auto& bg = static_cast<const pfield::BoxGeometry&>(obstacle.getGeometry());
      obstacleMarker.type = MarkerMsg::CUBE;
      obstacleMarker.scale.x = bg.length;
      obstacleMarker.scale.y = bg.width;
      obstacleMarker.scale.z = bg.height;
      // If the box was OBB-fitted from a mesh, shift the marker center to the OBB centroid
      // and compose the PCA rotation with the obstacle orientation.
      if (bg.centroidOffset.norm() > 1e-6) {
        const Eigen::Vector3d worldCenter = obstacle.getPosition() + obstacle.getOrientation() * bg.centroidOffset;
        obstacleMarker.pose.position.x = worldCenter.x();
        obstacleMarker.pose.position.y = worldCenter.y();
        obstacleMarker.pose.position.z = worldCenter.z();
      }
      if (!bg.axes.isIdentity(1e-6)) {
        const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
        const Eigen::Quaterniond obbQ(bg.axes);
        const Eigen::Quaterniond markerQ = obstacleQ * obbQ;
        obstacleMarker.pose.orientation.x = markerQ.x();
        obstacleMarker.pose.orientation.y = markerQ.y();
        obstacleMarker.pose.orientation.z = markerQ.z();
        obstacleMarker.pose.orientation.w = markerQ.w();
      }
      break;
    }
    case pfield::ObstacleType::CYLINDER: {
      const auto& cg = static_cast<const pfield::CylinderGeometry&>(obstacle.getGeometry());
      obstacleMarker.type = MarkerMsg::CYLINDER;
      obstacleMarker.scale.x = cg.radius * 2.0f;  // Diameter
      obstacleMarker.scale.y = cg.radius * 2.0f;  // Diameter
      obstacleMarker.scale.z = cg.height;          // Height
      break;
    }
    case pfield::ObstacleType::MESH: {
      if (!obstacle.getMeshResource().empty()) {
        obstacleMarker.type = MarkerMsg::MESH_RESOURCE;
        obstacleMarker.mesh_resource = obstacle.getMeshResource();
        obstacleMarker.mesh_use_embedded_materials = true;
        Eigen::Vector3d scale = obstacle.getMeshScale();
        obstacleMarker.scale.x = scale.x();
        obstacleMarker.scale.y = scale.y();
        obstacleMarker.scale.z = scale.z();
      }
      else {
        // Fallback to inflated AABB cube
        obstacleMarker.type = MarkerMsg::CUBE;
        const Eigen::Vector3d halfDims = obstacle.halfDimensions();
        obstacleMarker.scale.x = halfDims.x() * 2.0;
        obstacleMarker.scale.y = halfDims.y() * 2.0;
        obstacleMarker.scale.z = halfDims.z() * 2.0;
      }
      break;
    }
    case pfield::ObstacleType::ELLIPSOID: {
      // RViz SPHERE with non-uniform scale renders as an ellipsoid (scale = diameter per axis).
      // Compose the obstacle orientation with the PCA rotation so the marker aligns with the
      // ellipsoid's principal axes in the world frame.
      const auto& eg = static_cast<const pfield::EllipsoidGeometry&>(obstacle.getGeometry());
      obstacleMarker.type = MarkerMsg::SPHERE;
      obstacleMarker.scale.x = eg.semiX * 2.0;
      obstacleMarker.scale.y = eg.semiY * 2.0;
      obstacleMarker.scale.z = eg.semiZ * 2.0;
      {
        const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
        const Eigen::Quaterniond pcaQ(eg.axes);
        const Eigen::Quaterniond markerQ = obstacleQ * pcaQ;
        obstacleMarker.pose.orientation.x = markerQ.x();
        obstacleMarker.pose.orientation.y = markerQ.y();
        obstacleMarker.pose.orientation.z = markerQ.z();
        obstacleMarker.pose.orientation.w = markerQ.w();
      }
      break;
    }
    case pfield::ObstacleType::CAPSULE: {
      // RViz has no capsule primitive. Render as a cylinder for the shaft — endcap spheres are
      // emitted as separate markers below, after this switch block.
      const auto& capg = static_cast<const pfield::CapsuleGeometry&>(obstacle.getGeometry());
      const double r = capg.radius;
      const double shaft = capg.height;
      const Eigen::Vector3d& centroidOffset = capg.centroidOffset;
      const Eigen::Matrix3d& capsuleAxes = capg.axes;
      // Shift marker position to OBB centroid
      if (centroidOffset.norm() > 1e-6) {
        const Eigen::Vector3d worldCenter = obstacle.getPosition()
          + obstacle.getOrientation() * centroidOffset;
        obstacleMarker.pose.position.x = worldCenter.x();
        obstacleMarker.pose.position.y = worldCenter.y();
        obstacleMarker.pose.position.z = worldCenter.z();
      }
      // Compose obstacle orientation with capsule PCA rotation
      {
        const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
        const Eigen::Quaterniond capsuleQ(capsuleAxes);
        const Eigen::Quaterniond markerQ = obstacleQ * capsuleQ;
        obstacleMarker.pose.orientation.x = markerQ.x();
        obstacleMarker.pose.orientation.y = markerQ.y();
        obstacleMarker.pose.orientation.z = markerQ.z();
        obstacleMarker.pose.orientation.w = markerQ.w();
      }
      // Guard against zero-scale warning in RViz: if the shaft is zero or near-zero,
      // render as a sphere (the capsule degenerates to a sphere).
      if (shaft < 1e-4) {
        obstacleMarker.type = MarkerMsg::SPHERE;
        obstacleMarker.scale.x = r * 2.0;
        obstacleMarker.scale.y = r * 2.0;
        obstacleMarker.scale.z = r * 2.0;
      }
      else {
        obstacleMarker.type = MarkerMsg::CYLINDER;
        obstacleMarker.scale.x = r * 2.0;
        obstacleMarker.scale.y = r * 2.0;
        obstacleMarker.scale.z = shaft;
      }
      break;
    }
    }
    if (obstacle.getGroup() == pfield::ObstacleGroup::ROBOT) {
      // Robot obstacles in green
      obstacleMarker.color.r = 0.0f;
      obstacleMarker.color.g = 1.0f;
      obstacleMarker.color.b = 0.0f;
    }
    else {
      // Environment obstacles in red
      obstacleMarker.color.r = 1.0f;
      obstacleMarker.color.g = 0.0f;
      obstacleMarker.color.b = 0.0f;
    }
    if (obstacle.getGroup() == pfield::ObstacleGroup::ROBOT) {
      obstacleMarker.color.a = 0.65f; // Transparent for robot obstacles
    }
    else {
      obstacleMarker.color.a = 0.95f; // Opaque for environment obstacles
    }
    obstacleMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
    markerArray.markers.push_back(obstacleMarker);

    // For capsule obstacles, emit two additional sphere markers for the hemispherical endcaps.
    if (obstacle.getType() == pfield::ObstacleType::CAPSULE) {
      const auto& capg2 = static_cast<const pfield::CapsuleGeometry&>(obstacle.getGeometry());
      const double r = capg2.radius;
      const double halfShaft = capg2.height / 2.0;
      const Eigen::Vector3d& centroidOffset = capg2.centroidOffset;
      const Eigen::Matrix3d& capsuleAxes = capg2.axes;
      const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
      const Eigen::Quaterniond capsuleQ(capsuleAxes);
      const Eigen::Quaterniond markerQ = obstacleQ * capsuleQ;
      // Capsule axis = column 2 of capsuleAxes, in world frame = markerQ * Z
      const Eigen::Vector3d capsuleAxisWorld = markerQ * Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d center = obstacle.getPosition() + obstacleQ * centroidOffset;

      for (int cap = 0; cap < 2; ++cap) {
        const double sign = (cap == 0) ? 1.0 : -1.0;
        const Eigen::Vector3d capCenter = center + sign * halfShaft * capsuleAxisWorld;
        MarkerMsg capMarker;
        capMarker.header = obstacleMarker.header;
        capMarker.ns = obstacleMarker.ns;
        capMarker.id = hashID + 10000 + cap; // offset to avoid ID collision
        capMarker.action = MarkerMsg::ADD;
        capMarker.type = MarkerMsg::SPHERE;
        capMarker.pose.position.x = capCenter.x();
        capMarker.pose.position.y = capCenter.y();
        capMarker.pose.position.z = capCenter.z();
        capMarker.pose.orientation = obstacleMarker.pose.orientation;
        capMarker.scale.x = r * 2.0;
        capMarker.scale.y = r * 2.0;
        capMarker.scale.z = r * 2.0;
        capMarker.color = obstacleMarker.color;
        capMarker.lifetime = rclcpp::Duration(0, 0);
        markerArray.markers.push_back(capMarker);
      }
    }

    if (obstacle.getGroup() == pfield::ObstacleGroup::ROBOT) {
      // Skip influence zone visualization for robot obstacles
      continue;
    }
    // Create a transparent volume representing the influence zone
    MarkerMsg influenceMarker;
    influenceMarker.header.frame_id = this->fixedFrame;
    influenceMarker.header.stamp = this->now();
    influenceMarker.frame_locked = true;
    influenceMarker.ns = "environment_influence_zones";
    influenceMarker.id = hashID; // mirror id for influence volume
    influenceMarker.action = MarkerMsg::ADD;
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
    switch (obstacle.getType()) {
    case pfield::ObstacleType::SPHERE: {
      // Inflated sphere diameter = 2 * (radius + influenceDistance).
      const auto& sg2 = static_cast<const pfield::SphereGeometry&>(obstacle.getGeometry());
      influenceMarker.type = MarkerMsg::SPHERE;
      const double influenceZoneDiameter = 2.0 * (sg2.radius + influenceDistance);
      influenceMarker.scale.x = influenceZoneDiameter;
      influenceMarker.scale.y = influenceZoneDiameter;
      influenceMarker.scale.z = influenceZoneDiameter;
      break;
    }
    case pfield::ObstacleType::BOX: {
      // Inflated box dimensions = base dims + 2 * influenceDistance along each axis.
      const auto& bg2 = static_cast<const pfield::BoxGeometry&>(obstacle.getGeometry());
      influenceMarker.type = MarkerMsg::CUBE;
      influenceMarker.scale.x = bg2.length + 2.0 * influenceDistance;
      influenceMarker.scale.y = bg2.width + 2.0 * influenceDistance;
      influenceMarker.scale.z = bg2.height + 2.0 * influenceDistance;
      // Apply OBB orientation if present
      if (!bg2.axes.isIdentity(1e-6)) {
        const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
        const Eigen::Quaterniond obbQ(bg2.axes);
        const Eigen::Quaterniond markerQ = obstacleQ * obbQ;
        influenceMarker.pose.orientation.x = markerQ.x();
        influenceMarker.pose.orientation.y = markerQ.y();
        influenceMarker.pose.orientation.z = markerQ.z();
        influenceMarker.pose.orientation.w = markerQ.w();
      }
      break;
    }
    case pfield::ObstacleType::CYLINDER: {
      // Inflated cylinder: diameter = 2 * (radius + d), height = height + 2 * d.
      const auto& cg2 = static_cast<const pfield::CylinderGeometry&>(obstacle.getGeometry());
      influenceMarker.type = MarkerMsg::CYLINDER;
      influenceMarker.scale.x = 2.0 * (cg2.radius + influenceDistance);
      influenceMarker.scale.y = 2.0 * (cg2.radius + influenceDistance);
      influenceMarker.scale.z = cg2.height + 2.0 * influenceDistance;
      break;
    }
    case pfield::ObstacleType::MESH: {
      // Visualize the influence zone as an "inflated" version of the original mesh.
      // We approximate a Minkowski sum by scaling the mesh per-axis so its AABB grows by +2d.
      // This preserves the silhouette and is a better visual cue than a plain inflated AABB cube.
      if (!obstacle.getMeshResource().empty()) {
        influenceMarker.type = MarkerMsg::MESH_RESOURCE;
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
        influenceMarker.type = MarkerMsg::CUBE;
        const Eigen::Vector3d baseHalf = obstacle.halfDimensions();
        const Eigen::Vector3d baseDims = 2.0 * baseHalf; // L, W, H
        influenceMarker.scale.x = baseDims.x() + 2.0 * influenceDistance;
        influenceMarker.scale.y = baseDims.y() + 2.0 * influenceDistance;
        influenceMarker.scale.z = baseDims.z() + 2.0 * influenceDistance;
      }
      break;
    }
    case pfield::ObstacleType::ELLIPSOID: {
      // Inflated ellipsoid: each semi-axis grows by influenceDistance.
      // Apply PCA orientation so the marker matches the ellipsoid's principal axes.
      const auto& eg2 = static_cast<const pfield::EllipsoidGeometry&>(obstacle.getGeometry());
      influenceMarker.type = MarkerMsg::SPHERE;
      influenceMarker.scale.x = 2.0 * (eg2.semiX + influenceDistance);
      influenceMarker.scale.y = 2.0 * (eg2.semiY + influenceDistance);
      influenceMarker.scale.z = 2.0 * (eg2.semiZ + influenceDistance);
      {
        const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
        const Eigen::Quaterniond pcaQ(eg2.axes);
        const Eigen::Quaterniond markerQ = obstacleQ * pcaQ;
        influenceMarker.pose.orientation.x = markerQ.x();
        influenceMarker.pose.orientation.y = markerQ.y();
        influenceMarker.pose.orientation.z = markerQ.z();
        influenceMarker.pose.orientation.w = markerQ.w();
      }
      break;
    }
    case pfield::ObstacleType::CAPSULE: {
      // Inflated capsule: radius + influenceDistance, shaft unchanged.
      const auto& capg3 = static_cast<const pfield::CapsuleGeometry&>(obstacle.getGeometry());
      const double r = capg3.radius + influenceDistance;
      const double shaft = capg3.height;
      const Eigen::Quaterniond obstacleQ(obstacle.getOrientation());
      const Eigen::Quaterniond capsuleQ(capg3.axes);
      const Eigen::Quaterniond markerQ = obstacleQ * capsuleQ;
      const Eigen::Vector3d worldCenter = obstacle.getPosition()
        + obstacleQ * capg3.centroidOffset;
      influenceMarker.pose.position.x = worldCenter.x();
      influenceMarker.pose.position.y = worldCenter.y();
      influenceMarker.pose.position.z = worldCenter.z();
      influenceMarker.pose.orientation.x = markerQ.x();
      influenceMarker.pose.orientation.y = markerQ.y();
      influenceMarker.pose.orientation.z = markerQ.z();
      influenceMarker.pose.orientation.w = markerQ.w();
      influenceMarker.type = MarkerMsg::CYLINDER;
      influenceMarker.scale.x = r * 2.0;
      influenceMarker.scale.y = r * 2.0;
      influenceMarker.scale.z = shaft;
      break;
    }
    }
    markerArray.markers.push_back(influenceMarker);
  }

  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createGhostMeshOverlayMarkerArray(
  const std::vector<pfield::PotentialFieldObstacle>& robotObstacles) {
  MarkerArrayMsg markerArray;
  int ghostID = 0;

  for (const auto& obstacle : robotObstacles) {
    if (obstacle.getType() != pfield::ObstacleType::CAPSULE) { continue; }
    const auto& ghostCapg = static_cast<const pfield::CapsuleGeometry&>(obstacle.getGeometry());
    const std::string& meshURI = ghostCapg.sourceMeshResource;
    if (meshURI.empty()) { continue; }

    MarkerMsg ghostMarker;
    ghostMarker.header.frame_id = this->fixedFrame;
    ghostMarker.header.stamp = this->now();
    ghostMarker.frame_locked = true;
    ghostMarker.ns = "robot_mesh_ghost";
    ghostMarker.id = ghostID++;
    ghostMarker.action = MarkerMsg::ADD;
    ghostMarker.type = MarkerMsg::MESH_RESOURCE;
    ghostMarker.mesh_resource = meshURI;
    ghostMarker.mesh_use_embedded_materials = false;

    const auto pos = obstacle.getPosition();
    const auto ori = obstacle.getOrientation();
    ghostMarker.pose.position.x = pos.x();
    ghostMarker.pose.position.y = pos.y();
    ghostMarker.pose.position.z = pos.z();
    ghostMarker.pose.orientation.x = ori.x();
    ghostMarker.pose.orientation.y = ori.y();
    ghostMarker.pose.orientation.z = ori.z();
    ghostMarker.pose.orientation.w = ori.w();

    const Eigen::Vector3d& s = ghostCapg.sourceMeshScale;
    ghostMarker.scale.x = s.x();
    ghostMarker.scale.y = s.y();
    ghostMarker.scale.z = s.z();

    // Semi-transparent white so the capsule (green) is visible through the mesh
    ghostMarker.color.r = 0.9f;
    ghostMarker.color.g = 0.9f;
    ghostMarker.color.b = 0.9f;
    ghostMarker.color.a = 0.35f;
    ghostMarker.lifetime = rclcpp::Duration(0, 0);

    markerArray.markers.push_back(ghostMarker);
  }

  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createRobotLinkControlPointsMarkerArray(
  const std::vector<pfield::PotentialFieldObstacle>& robotObstacles) {
  MarkerArrayMsg markerArray;
  const double CP_RADIUS = 0.015; // sphere diameter = 3 cm

  for (const auto& link : robotObstacles) {
    const auto controlPoints = pfield::PotentialField::buildControlPoints(link);
    // Deterministic color per link: hash the frame_id to a hue in [0, 360)
    const std::size_t hashValue = std::hash<std::string>{}(link.getFrameID());
    const double hue = static_cast<double>(hashValue % 360);
    const auto& [r, g, b] = convertHSVToRGB(hue, 0.9, 0.9);
    // Stable per-link ID base: keeps IDs consistent across frames so RViz doesn't
    // see duplicate ns+id pairs when DELETEALL and ADD markers coexist in one message.
    const int linkIDBase = static_cast<int>(hashValue & 0x7FFFFFFF);
    int slotIndex = 0;
    for (const auto& [cp, surfaceRadius] : controlPoints) {
      MarkerMsg cpMarker;
      cpMarker.header.frame_id = this->fixedFrame;
      cpMarker.header.stamp = this->now();
      cpMarker.frame_locked = true;
      cpMarker.ns = "robot_control_points";
      cpMarker.id = linkIDBase + slotIndex++;
      cpMarker.action = MarkerMsg::ADD;
      cpMarker.type = MarkerMsg::SPHERE;
      cpMarker.pose.position.x = cp.x();
      cpMarker.pose.position.y = cp.y();
      cpMarker.pose.position.z = cp.z();
      cpMarker.pose.orientation.w = 1.0;
      cpMarker.scale.x = CP_RADIUS * 2.0;
      cpMarker.scale.y = CP_RADIUS * 2.0;
      cpMarker.scale.z = CP_RADIUS * 2.0;
      cpMarker.color.r = r;
      cpMarker.color.g = g;
      cpMarker.color.b = b;
      cpMarker.color.a = 1.0f;
      cpMarker.lifetime = rclcpp::Duration(0, 0);
      markerArray.markers.push_back(cpMarker);
    }
  }

  return markerArray;
}

MarkerArrayMsg PotentialFieldManager::createGoalMarker(std::shared_ptr<pfield::PotentialField> pf) {
  // Create a green sphere marker
  MarkerArrayMsg markerArray;
  if (!pf->isGoalSet()) { return markerArray; }
  MarkerMsg goalMarker;
  goalMarker.header.frame_id = this->fixedFrame;
  goalMarker.header.stamp = this->now();
  goalMarker.frame_locked = true;
  goalMarker.ns = "goal";
  goalMarker.id = 0;
  goalMarker.type = MarkerMsg::SPHERE;
  goalMarker.action = MarkerMsg::ADD;
  pfield::SpatialVector goalPose = pf->getGoalPose();
  goalMarker.pose.position.x = goalPose.getPosition().x();
  goalMarker.pose.position.y = goalPose.getPosition().y();
  goalMarker.pose.position.z = goalPose.getPosition().z();
  goalMarker.pose.orientation.x = goalPose.getOrientation().x();
  goalMarker.pose.orientation.y = goalPose.getOrientation().y();
  goalMarker.pose.orientation.z = goalPose.getOrientation().z();
  goalMarker.pose.orientation.w = goalPose.getOrientation().w();
  goalMarker.scale.x = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  goalMarker.scale.y = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  goalMarker.scale.z = this->TASK_SPACE_POSE_SPHERE_DIAMETER;
  goalMarker.color.r = 0.0f;
  goalMarker.color.g = 1.0f;
  goalMarker.color.b = 0.0f;
  goalMarker.color.a = 1.0f; // Opaque
  goalMarker.lifetime = rclcpp::Duration(0, 0); // No lifetime
  std::vector<MarkerMsg> goalAxes;
  goalAxes.reserve(3);
  for (int i = 0; i < 3; i++) {
    MarkerMsg axis;
    axis.header.frame_id = this->fixedFrame;
    axis.header.stamp = this->now();
    axis.ns = "goal";
    axis.id = i + 1;
    axis.type = MarkerMsg::ARROW;
    axis.action = MarkerMsg::ADD;
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
    axis.scale.x = this->TASK_SPACE_AXIS_LENGTH;
    axis.scale.y = this->TASK_SPACE_AXIS_SHAFT_DIAMETER;
    axis.scale.z = this->TASK_SPACE_AXIS_HEAD_DIAMETER;
    axis.color.a = 0.9f; // slightly opaque
    axis.lifetime = rclcpp::Duration(0, 0); // No lifetime
    goalAxes.push_back(axis);
  }
  MarkerArrayMsg goalMarkerArrayMsg;
  goalMarkerArrayMsg.markers.push_back(goalMarker);
  goalMarkerArrayMsg.markers.insert(
    goalMarkerArrayMsg.markers.cend(), goalAxes.cbegin(), goalAxes.cend());
  return goalMarkerArrayMsg;
}

MarkerArrayMsg PotentialFieldManager::createPotentialVectorMarkers(std::shared_ptr<pfield::PotentialField> pf) {
  MarkerArrayMsg markerArray;
  int id = 0;
  const auto limits = pf->computeFieldBounds(this->queryPose, this->visualizerBufferArea);
  const double resolution = std::max(this->fieldResolution, 1.0); // Resolution must be at least 1.0
  for (double x = limits.minX; x <= limits.maxX; x += resolution) {
    for (double y = limits.minY; y <= limits.maxY; y += resolution) {
      for (double z = limits.minZ; z <= limits.maxZ; z += resolution) {
        // Skip any points that are inside obstacle radius
        Eigen::Vector3d point(x, y, z);
        if (pf->isPointInsideObstacle(point)) { continue; }
        MarkerMsg vectorMarker;
        pfield::SpatialVector position{point};
        pfield::TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
        const Eigen::Vector3d v = velocity.getLinearVelocity();
        const double magnitude = v.norm();
        vectorMarker.header.frame_id = this->fixedFrame;
        vectorMarker.header.stamp = this->now();
        vectorMarker.ns = "potential_vectors";
        vectorMarker.id = id++;
        vectorMarker.type = MarkerMsg::ARROW;
        vectorMarker.action = MarkerMsg::ADD;
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

TwistMsg PotentialFieldManager::fuseTwists(
  const TwistMsg::SharedPtr twist1,
  const TwistMsg::SharedPtr twist2,
  const double alpha) {
  TwistMsg fusedTwist;

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

TwistMsg clampTwist(const TwistMsg& twist, const TwistMsg& limits) {
  TwistMsg clampedTwist = twist; // Start with the input twist
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

void PotentialFieldManager::exportFieldDataToCSV(std::shared_ptr<pfield::PotentialField> pf, const std::string& base_filename) {
  // Write obstacle positions to a CSV file
  std::string obstacles_filename = "data/" + base_filename + "_obstacles.csv";
  std::ofstream obstacles_file(obstacles_filename);
  if (obstacles_file.is_open()) {
    obstacles_file << "obstacle_x,obstacle_y,obstacle_z,type,influence,repulsive_gain,params\n";
    for (const auto& obstacle : pf->getEnvObstacles()) {
      const Eigen::Vector3d& position = obstacle.getPosition();
      const std::vector<double> params = obstacle.getGeometry().asVector();
      obstacles_file << position.x() << "," << position.y() << "," << position.z() << ","
        << pfield::obstacleTypeToString(obstacle.getType()) << "," << this->pField->getInfluenceDistance() << ","
        << this->pField->getRepulsiveGain();
      for (double p : params) { obstacles_file << "," << p; }
      obstacles_file << "\n";
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
        pfield::SpatialVector position{point};
        pfield::TaskSpaceTwist velocity = pf->evaluateLimitedVelocityAtPose(position);
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
  // Set the URI resolver for the pfield library to handle package:// URIs
  pfield::setURIResolver([](const std::string& uri) -> std::string {
    const std::string packagePrefix = "package://";
    if (uri.find(packagePrefix) == 0) {
      size_t start = packagePrefix.length();
      size_t end = uri.find('/', start);
      if (end != std::string::npos) {
        std::string packageName = uri.substr(start, end - start);
        std::string relativePath = uri.substr(end); // includes the leading slash
        try {
          std::string packagePath = ament_index_cpp::get_package_share_directory(packageName);
          return packagePath + relativePath;
        }
        catch (const std::exception& e) {
          std::cerr << "Error resolving package URI: " << uri << " - " << e.what() << std::endl;
          return uri; // Fallback to original
        }
      }
    }
    return uri;
  });

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentialFieldManager>());
  rclcpp::shutdown();
  return 0;
}
