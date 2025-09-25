#ifndef FRANKA_PLUGIN_HPP
#define FRANKA_PLUGIN_HPP

// An interface with libfranka to send
// an end-effector velocity command to the robot
// using the velocity calculated by the potential field


#include <cmath>
#include <iostream>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "motion_plugin.hpp"

class FrankaPlugin : public MotionPlugin {
public:
  explicit FrankaPlugin(double speed_factor = 0.2);
  ~FrankaPlugin() override = default;

  bool configure() override;
  bool start() override;
  bool stop() override;
  bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) override;
  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override;

};


#endif // !FRANKA_PLUGIN_HPP