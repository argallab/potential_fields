#include "ros/motion_interface.hpp"
#include "rclcpp/rclcpp.hpp"

explicit MotionInterface::MotionInterface(const rclcpp::NodeOptions& opts)
  : Node("motion_interface", opts) {
  RCLCPP_INFO(this->get_logger(), "MotionInterface Initialized");

  // Declare parameters
  std::string robotPluginType = this->declare_parameter("robot_plugin_type", "franka");
  std::string robotHostname = this->declare_parameter("robot_hostname", "192.168.18.1");
  std::string baseLinkFrame = this->declare_parameter("base_link_frame", "base_link");
  std::string endEffectorFrame = this->declare_parameter("end_effector_frame", "ee_link");
  this->fusionAlpha = this->declare_parameter("fusion_alpha", 1.0);
  this->controlLoopFreq = this->declare_parameter("control_loop_frequency", 10.0); // [Hz]
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

  // Initialize control state as disabled, which gets toggled by start/stop services
  this->controlEnabled = false;

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
  // Setup Services
  this->startSrv = this->create_service<std_srvs::srv::Empty>(
    "pfield/start_control",
    [this](const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
      std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
    RCLCPP_INFO(this->get_logger(), "Starting control");
    this->controlEnabled = true;
  }
  );
  this->stopSrv = this->create_service<std_srvs::srv::Empty>(
    "pfield/stop_control",
    [this](const std_srvs::srv::Empty::Request::SharedPtr /*req*/,
      std_srvs::srv::Empty::Response::SharedPtr /*res*/) {
    RCLCPP_INFO(this->get_logger(), "Stopping control");
    this->controlEnabled = false;
  }
  );

  // Setup planning joint state publisher
  this->planningJointStatePub = this->create_publisher<JointState>("planning_joint_states", 10);
  RCLCPP_INFO(this->get_logger(), "Planning joint states publishing on: %s", this->planningJointStatePub->get_topic_name());

  // Create a client to request single PF steps from the PFieldManager
  this->pfieldStepClient = this->create_client<PFieldStep>("pfield/step");

  // Create the PlanPath service here (moved from PFieldManager)
  this->pathPlanningService = this->create_service<PlanPath>(
    "pfield/plan_path",
    std::bind(&MotionInterface::handlePlanPath, this, std::placeholders::_1, std::placeholders::_2)
  );

  // Setup the timer running the control loop
  this->controlTimer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / controlLoopFreq),
    std::bind(&MotionInterface::controlLoop, this)
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
    return res;
  };

  // Build initial pose
  geometry_msgs::msg::PoseStamped currentPose = request->start;
  nav_msgs::msg::Path path;
  path.header.frame_id = "world"; // default; could be parameterized
  path.header.stamp = this->now();
  path.poses.push_back(currentPose);
  // Interpolate until goal reached or max iterations
  const int max_iters = 10000; // TODO(Sharwin): Parameterize this (or derive from MotionPlugin)
  unsigned int iter = 0;
  bool reached = false;
  while (iter++ < max_iters) {
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
    // Check if we reached the goal within the tolerance and exit early if so
    // Call IK on current pose to get current joint angles
    // Publish joint angles to planned joint state for PFManager to update its internal PF
    // Save joint angles for JointTrajectory
    // Call PFStep to get next pose from current pose (handles waiting for PF to update from PJSP update)
    // TODO(Sharwin): Implement internal waiting for PF update from PJSP
    // Save pose for EE path
  }

  response->plan = path;
  response->joint_trajectory = trajectory_msgs::msg::JointTrajectory();
  response->success = reached;
  if (!reached) RCLCPP_WARN(this->get_logger(), "Plan path reached iteration limit without reaching goal");
}

void MotionInterface::controlLoop() {
  // The latest human and autonomy twists are stored in lastHumanTwist and lastAutonomyTwist
  // populated from their respective subscriber callbacks
  // Generate a fused velocity vector with the human twist and autonomy twist
  auto fusedTwist = this->fuseTwists(this->lastHumanTwist, this->lastAutonomyTwist);
  // Clamp velocity vector to be within the defined limits
  auto clampedTwist = this->clampTwist(fusedTwist);
  // Send the velocity vector to the robot and start it's controller
  // The max duration of the commanded velocity should be the period of this control loop
  // so once the motion finishes, the next command is sent
  auto period = std::chrono::duration<double>(1.0 / this->controlLoopFreq);
  this->motionPlugin->sendCartesianTwist(clampedTwist.twist);
}

geometry_msgs::msg::TwistStamped MotionInterface::fuseTwists(
  const geometry_msgs::msg::TwistStamped::SharedPtr humanTwist,
  const geometry_msgs::msg::TwistStamped::SharedPtr autonomyTwist) {
  geometry_msgs::msg::TwistStamped fusedTwist;
  fusedTwist.header.stamp = this->now();
  fusedTwist.header.frame_id = autonomyTwist->header.frame_id;

  auto fuseValue = [](double humanValue, double autonomyValue, double alpha) {
    return alpha * autonomyValue + (1.0 - alpha) * humanValue;
  };

  // Simple linear fusion based on alpha parameter
  // When alpha = 1.0, the output is only the autonomy command
  fusedTwist.twist.linear.x = fuseValue(humanTwist->twist.linear.x, autonomyTwist->twist.linear.x, this->fusionAlpha);
  fusedTwist.twist.linear.y = fuseValue(humanTwist->twist.linear.y, autonomyTwist->twist.linear.y, this->fusionAlpha);
  fusedTwist.twist.linear.z = fuseValue(humanTwist->twist.linear.z, autonomyTwist->twist.linear.z, this->fusionAlpha);
  fusedTwist.twist.angular.x = fuseValue(humanTwist->twist.angular.x, autonomyTwist->twist.angular.x, this->fusionAlpha);
  fusedTwist.twist.angular.y = fuseValue(humanTwist->twist.angular.y, autonomyTwist->twist.angular.y, this->fusionAlpha);
  fusedTwist.twist.angular.z = fuseValue(humanTwist->twist.angular.z, autonomyTwist->twist.angular.z, this->fusionAlpha);

  return fusedTwist;
}
