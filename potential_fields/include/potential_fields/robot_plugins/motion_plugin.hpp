#ifndef MOTION_PLUGIN_HPP
#define MOTION_PLUGIN_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class MotionPlugin {
public:
  virtual ~MotionPlugin() = default;
  virtual bool configure() = 0;
  virtual bool start() = 0;
  virtual bool stop() = 0;
  virtual bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) = 0;
  virtual bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) = 0;
};

#endif // !MOTION_PLUGIN_HPP
