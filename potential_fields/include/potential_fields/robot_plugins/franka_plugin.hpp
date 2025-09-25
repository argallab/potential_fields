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
  FrankaPlugin(const std::string& hostname);
  ~FrankaPlugin() override = default;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) override;
  bool sendJointStates(const sensor_msgs::msg::JointState& js) override;
  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override;

  void startControlLoop(const franka::Duration& movementDuration);
private:
  std::unique_ptr<franka::Robot> robot;
  std::unique_ptr<franka::CartesianVelocities> currentEEVelocity;

  void initializeRobot(const std::string& hostname);
};


#endif // !FRANKA_PLUGIN_HPP