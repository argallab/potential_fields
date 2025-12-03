#ifndef XARM_PLUGIN_HPP
#define XARM_PLUGIN_HPP

#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "solvers/xarm_ik_solver.hpp"
#include "motion_plugin.hpp"

class XArmPlugin : public MotionPlugin {
public:
  XArmPlugin();
  ~XArmPlugin() override;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) override;
  bool sendJointStates(const sensor_msgs::msg::JointState& js) override;
  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override;
};


#endif // !XARM_PLUGIN_HPP
