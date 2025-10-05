#ifndef FRANKA_PLUGIN_HPP
#define FRANKA_PLUGIN_HPP

// An interface with libfranka to send
// an end-effector velocity command to the robot
// using the velocity calculated by the potential field


#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "motion_plugin.hpp"

class FrankaIKSolver : public IKSolver {
public:
  FrankaIKSolver();
  ~FrankaIKSolver() override = default;

  bool computeIK(const geometry_msgs::msg::PoseStamped& endEffectorPose,
    sensor_msgs::msg::JointState& jointState) override;

private:
  // The joint angles of the franka in the "home" position [rad]
  const std::array<double, 7> homeJointAngles = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  // The homogeneous transform from the robot's base link [O] to the end-effector [E] in the "home" position
  Eigen::Matrix4d homeTransformOE;
};

class FrankaPlugin : public MotionPlugin {
public:
  explicit FrankaPlugin(const std::string& hostname);
  ~FrankaPlugin() override = default;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) override;
  bool sendJointStates(const sensor_msgs::msg::JointState& js) override;
  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override;

  bool startControlLoop(const franka::Duration& movementDuration);
private:
  std::unique_ptr<franka::Robot> robot;
  std::unique_ptr<franka::CartesianVelocities> currentEEVelocity;

  bool initializeRobot(const std::string& hostname);
};


#endif // !FRANKA_PLUGIN_HPP
