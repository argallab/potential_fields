#ifndef MOTION_PLUGIN_HPP
#define MOTION_PLUGIN_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class MotionPlugin {
public:
  virtual ~MotionPlugin() = default;
  virtual bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) = 0;
  virtual bool sendJointStates(const sensor_msgs::msg::JointState& js) = 0;
  virtual bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) = 0;
  virtual void assignSharedClock(rclcpp::Clock::SharedPtr clock) { this->clock = clock; }
protected:
  rclcpp::Clock::SharedPtr clock;
};

#endif // !MOTION_PLUGIN_HPP
