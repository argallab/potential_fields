// Copyright 2025 argallab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
