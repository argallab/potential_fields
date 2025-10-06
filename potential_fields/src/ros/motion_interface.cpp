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

  // Setup the timer running the control loop
  this->controlTimer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / controlLoopFreq),
    std::bind(&MotionInterface::controlLoop, this)
  );
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
