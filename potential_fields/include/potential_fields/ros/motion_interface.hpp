#ifndef MOTION_INTERFACE_HPP
#define MOTION_INTERFACE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "robot_plugins/motion_plugin.hpp"

class MotionInterface : public rclcpp::Node {
public:
  explicit MotionInterface(const rclcpp::NodeOptions& opts);

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr humanTwistSub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr autonomyTwistPub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr fusedTwistPub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPoseSub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr startSrv, stopSrv;
  rclcpp::TimerBase::SharedPtr controlTimer;

  // Instead of storing a shared_ptr, we will need to reach into the PF Manager node
  // via ROS interfaces to get the data and functions we need
  // std::shared_ptr<PotentialField> pField;
  std::unique_ptr<MotionPlugin> motionPlugin;

  geometry_msgs::msg::Twist::SharedPtr twistLimits;
  geometry_msgs::msg::PoseStamped::SharedPtr currentGoal;
  geometry_msgs::msg::TwistStamped::SharedPtr lastHumanTwist;

  void controlLoop();
};

#endif // !MOTION_INTERFACE_HPP