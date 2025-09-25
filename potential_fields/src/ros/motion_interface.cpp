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
  double fusionAlpha = this->declare_parameter("fusion_alpha", 1.0);
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

}