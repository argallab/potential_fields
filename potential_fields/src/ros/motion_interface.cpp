#include "ros/motion_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include "robot_plugins/null_motion_plugin.hpp"

MotionInterface::MotionInterface() : Node("motion_interface") {
  RCLCPP_INFO(this->get_logger(), "MotionInterface Initialized");

  // Declare parameters
  std::string robotPluginType = this->declare_parameter("robot_plugin_type", "franka");
  std::string robotHostname = this->declare_parameter("robot_hostname", "192.168.18.1");
  std::string baseLinkFrame = this->declare_parameter("base_link_frame", "base_link");
  std::string endEffectorFrame = this->declare_parameter("end_effector_frame", "ee_link");
  this->fusionAlpha = this->declare_parameter("fusion_alpha", 1.0);
  double vLinMax = this->declare_parameter("limits.v_lin_max", 0.20);
  double vAngMax = this->declare_parameter("limits.v_ang_max", 0.80);
  double aLinMax = this->declare_parameter("limits.a_lin_max", 0.40);
  double aAngMax = this->declare_parameter("limits.a_ang_max", 1.60);

  // Initialize twistLimits and assign values
  this->twistLimits = std::make_shared<geometry_msgs::msg::Twist>();
  this->twistLimits->linear.x = vLinMax;
  this->twistLimits->linear.y = vLinMax;
  this->twistLimits->linear.z = vLinMax;
  this->twistLimits->angular.x = vAngMax;
  this->twistLimits->angular.y = vAngMax;
  this->twistLimits->angular.z = vAngMax;

  RCLCPP_INFO(this->get_logger(), "Loaded velocity limits - Linear: %.2f m/s, Angular: %.2f rad/s",
    vLinMax, vAngMax);
  RCLCPP_INFO(this->get_logger(), "Loaded acceleration limits - Linear: %.2f m/s², Angular: %.2f rad/s²",
    aLinMax, aAngMax);

  // Setup Publishers
  this->fusedTwistPub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "pfield/fused_twist", rclcpp::QoS(10)
  );
  // Setup Subscribers
  this->humanTwistSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "pfield/human_twist", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    this->lastHumanTwist = msg;
  }
  );
  this->autonomyTwistSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "pfield/autonomy_twist", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    this->lastAutonomyTwist = msg;
  }
  );
  this->goalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pfield/goal_pose", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    this->currentGoal = msg;
  }
  );

  // Setup planning joint state publisher
  this->planningJointStatePub = this->create_publisher<JointState>("planning_joint_states", 10);
  RCLCPP_INFO(this->get_logger(), "Planning joint states publishing on: %s", this->planningJointStatePub->get_topic_name());

  // Create a client to request single PF steps from the PFieldManager
  this->pfieldStepClient = this->create_client<PFieldStep>("pfield/step");

  // Initialize a null motion plugin by default so the node can run without a real robot
  this->motionPlugin = std::make_unique<NullMotionPlugin>();

  // Create the PlanPath service here (moved from PFieldManager)
  this->pathPlanningService = this->create_service<PlanPath>(
    "pfield/plan_path",
    std::bind(&MotionInterface::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void MotionInterface::handlePlanPath(const PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "Received plan_path request");
  // Ensure PF step service is available
  if (!this->pfieldStepClient->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "pfield/step service not available");
    response->success = false;
    return;
  }

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

  auto getJointAngles = [this, ikSolver](const geometry_msgs::msg::PoseStamped& pose) -> std::vector<double> {
    // Use current robot state as seed
    sensor_msgs::msg::JointState js;
    geometry_msgs::msg::PoseStamped currentEEPose;
    if (!this->motionPlugin->readRobotState(js, currentEEPose)) {
      RCLCPP_WARN(this->get_logger(), "Failed to read robot state for IK");
      return {};
    }
    std::vector<double> seed = js.position;
    std::vector<double> solution;
    // Build target isometry from the provided pose (not from current EE pose)
    Eigen::Isometry3d targetIso;
    tf2::fromMsg(pose.pose, targetIso);
    Eigen::Matrix<double, 6, Eigen::Dynamic> J;
    if (!ikSolver->solve(targetIso, seed, solution, /*J=*/J)) {
      RCLCPP_WARN(this->get_logger(), "IK solver failed to find a solution for target pose");
      return {};
    }
    return solution;
  };

  auto pfStep = [this](const geometry_msgs::msg::PoseStamped currentPose, const double dt, const double goalTol) -> PFieldStep::Response {
    // Prepare PF step request
    auto pfStepRequest = std::make_shared<PFieldStep::Request>();
    pfStepRequest->current_pose = currentPose;
    pfStepRequest->delta_time = dt;
    pfStepRequest->goal_tolerance = goalTol;
    // Get Service future and spin until it's done
    auto future = this->pfieldStepClient->async_send_request(pfStepRequest);
    // Wait for response (synchronous)
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Failed to call pfield/step service");
      auto pfStepResponse = PFieldStep::Response();
      pfStepResponse.success = false;
      return pfStepResponse;
    }
    auto res = future.get();
    if (!res) {
      PFieldStep::Response pfStepResponse;
      pfStepResponse.success = false;
      return pfStepResponse;
    }
    return *res;
  };

  auto publishPlanningJointStates = [this](const std::vector<double>& jointPositions) {
    JointState js;
    js.header.stamp = this->now();
    js.name = this->motionPlugin->getIKSolver()->getJointNames();
    js.position = jointPositions;
    this->planningJointStatePub->publish(js);
  };

  // Build initial pose
  geometry_msgs::msg::PoseStamped currentPose = request->start;
  nav_msgs::msg::Path path;
  path.header.frame_id = "world"; // default; could be parameterized
  path.header.stamp = this->now();
  trajectory_msgs::msg::JointTrajectory jointTrajectory;
  jointTrajectory.header = path.header;
  jointTrajectory.joint_names = ikSolver->getJointNames();
  // Container to accumulate end-effector velocity (TwistStamped) trajectory
  std::vector<geometry_msgs::msg::TwistStamped> eeVelocityTrajectory;
  // Interpolate until goal reached or max iterations
  const int max_iters = 30000; // TODO(Sharwin): Parameterize this (or derive from MotionPlugin)
  unsigned int iter = 0;
  bool reached = false;
  const auto startTime = this->now();
  while (iter++ < max_iters) {
    // Check if we reached the goal within the tolerance and exit early if so
    if (reached) break;
    // Save current pose and current joint state
    path.poses.push_back(currentPose);
    // Call IK on current pose to get current joint angles
    auto jointAngles = getJointAngles(currentPose);
    if (jointAngles.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to get joint angles from IK, aborting plan");
      response->success = false;
      return;
    }
    // Save joint angles for JointTrajectory
    trajectory_msgs::msg::JointTrajectoryPoint jointTrajectoryPoint;
    jointTrajectoryPoint.positions = jointAngles;
    const auto timeFromStart = this->now() - startTime;
    jointTrajectoryPoint.time_from_start = rclcpp::Duration::from_seconds(timeFromStart.seconds());
    jointTrajectory.points.push_back(jointTrajectoryPoint);
    // Publish joint angles to planned joint state for PFManager to update its internal PF
    publishPlanningJointStates(jointAngles);
    // Call PFStep to get next pose from current pose (handles waiting for PF to update from PJSP update)
    auto pfResponse = pfStep(currentPose, /*dt=*/request->delta_time, /*goalTol=*/request->goal_tolerance);
    // Validate PF response
    if (!pfResponse.success) {
      RCLCPP_WARN(this->get_logger(), "pfield/step returned unsuccessful");
      response->success = false;
      return;
    }
    // Store the autonomy vector (end-effector velocity) returned by PF step
    eeVelocityTrajectory.push_back(pfResponse.autonomy_vector);
    // Update the current pose before moving to next iteration
    reached = pfResponse.reached_goal;
    currentPose = pfResponse.next_pose;
  }
  response->end_effector_path = path;
  response->joint_trajectory = jointTrajectory;
  // Move accumulated EE velocity trajectory into the response
  response->end_effector_velocity_trajectory = eeVelocityTrajectory;
  response->success = reached;
  if (reached) RCLCPP_INFO(this->get_logger(), "Plan path succeeded in %u iterations", iter);
  else if (iter >= max_iters) RCLCPP_WARN(this->get_logger(), "Plan path reached iteration limit without reaching goal");
}

geometry_msgs::msg::TwistStamped MotionInterface::fuseTwists(
  const geometry_msgs::msg::TwistStamped::SharedPtr humanTwist,
  const geometry_msgs::msg::TwistStamped::SharedPtr autonomyTwist) {
  geometry_msgs::msg::TwistStamped fusedTwist;
  fusedTwist.header.stamp = this->now();

  // If autonomyTwist is null, use a default frame and zero twist
  std::string frame = "";
  if (autonomyTwist) frame = autonomyTwist->header.frame_id;
  fusedTwist.header.frame_id = frame;

  // Helper to extract values safely (treat missing twist as zero)
  auto safe_linear = [](const geometry_msgs::msg::TwistStamped::SharedPtr t, int idx) {
    if (!t) return 0.0;
    switch (idx) {
    case 0: return t->twist.linear.x;
    case 1: return t->twist.linear.y;
    default: return t->twist.linear.z;
    }
  };
  auto safe_angular = [](const geometry_msgs::msg::TwistStamped::SharedPtr t, int idx) {
    if (!t) return 0.0;
    switch (idx) {
    case 0: return t->twist.angular.x;
    case 1: return t->twist.angular.y;
    default: return t->twist.angular.z;
    }
  };

  auto fuseValue = [](double humanValue, double autonomyValue, double alpha) {
    return alpha * autonomyValue + (1.0 - alpha) * humanValue;
  };

  // Simple linear fusion based on alpha parameter. Missing inputs are treated as zeros.
  fusedTwist.twist.linear.x = fuseValue(safe_linear(humanTwist, 0), safe_linear(autonomyTwist, 0), this->fusionAlpha);
  fusedTwist.twist.linear.y = fuseValue(safe_linear(humanTwist, 1), safe_linear(autonomyTwist, 1), this->fusionAlpha);
  fusedTwist.twist.linear.z = fuseValue(safe_linear(humanTwist, 2), safe_linear(autonomyTwist, 2), this->fusionAlpha);
  fusedTwist.twist.angular.x = fuseValue(safe_angular(humanTwist, 0), safe_angular(autonomyTwist, 0), this->fusionAlpha);
  fusedTwist.twist.angular.y = fuseValue(safe_angular(humanTwist, 1), safe_angular(autonomyTwist, 1), this->fusionAlpha);
  fusedTwist.twist.angular.z = fuseValue(safe_angular(humanTwist, 2), safe_angular(autonomyTwist, 2), this->fusionAlpha);

  return fusedTwist;
}

geometry_msgs::msg::TwistStamped MotionInterface::clampTwist(const geometry_msgs::msg::TwistStamped& twist) {
  geometry_msgs::msg::TwistStamped clampedTwist = twist; // Start with the input twist

  // Clamp linear velocities
  clampedTwist.twist.linear.x = std::clamp(clampedTwist.twist.linear.x, -this->twistLimits->linear.x, this->twistLimits->linear.x);
  clampedTwist.twist.linear.y = std::clamp(clampedTwist.twist.linear.y, -this->twistLimits->linear.y, this->twistLimits->linear.y);
  clampedTwist.twist.linear.z = std::clamp(clampedTwist.twist.linear.z, -this->twistLimits->linear.z, this->twistLimits->linear.z);
  // Clamp angular velocities
  clampedTwist.twist.angular.x = std::clamp(clampedTwist.twist.angular.x, -this->twistLimits->angular.x, this->twistLimits->angular.x);
  clampedTwist.twist.angular.y = std::clamp(clampedTwist.twist.angular.y, -this->twistLimits->angular.y, this->twistLimits->angular.y);
  clampedTwist.twist.angular.z = std::clamp(clampedTwist.twist.angular.z, -this->twistLimits->angular.z, this->twistLimits->angular.z);

  return clampedTwist;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionInterface>());
  rclcpp::shutdown();
  return 0;
}
