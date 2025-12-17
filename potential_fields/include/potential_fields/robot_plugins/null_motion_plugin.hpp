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

#ifndef NULL_MOTION_PLUGIN_HPP
#define NULL_MOTION_PLUGIN_HPP

#include "motion_plugin.hpp"
#include <vector>
#include <string>
#include <memory>

// A minimal IK solver that returns the seed as the solution (useful for offline testing)
class NullIKSolver : public pfield::IKSolver {
public:
  NullIKSolver() : pfield::IKSolver("NullIKSolver") {}
  ~NullIKSolver() override = default;

  bool solve([[maybe_unused]] const Eigen::Isometry3d& targetPose, const std::vector<double>& seed,
    std::vector<double>& solution, Eigen::Matrix<double, 6, Eigen::Dynamic>& J, std::string& errorMsg) override {
    if (seed.empty()) { solution = {}; }
    else { solution = seed; }
    J.resize(6, solution.size());
    J.setZero();
    errorMsg.clear();
    return true;
  }

  bool computeJacobian(const std::vector<double>& jointPositions, Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    J.resize(6, jointPositions.size());
    J.setZero();
    return true;
  }

  std::vector<std::string> getJointNames() const override { return {}; }

  std::vector<double> getHomeConfiguration() const override { return {}; }
};

// A null MotionPlugin which performs no real robot IO and is suitable for offline testing
class NullMotionPlugin : public MotionPlugin {
public:
  NullMotionPlugin() : MotionPlugin("NullMotionPlugin") {
    this->assignIKSolver(std::make_shared<NullIKSolver>());
    this->assignSharedClock(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  }
  ~NullMotionPlugin() override = default;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& /*endEffectorTwist*/) override {
    // No-op for offline testing
    return true;
  }

  bool sendJointStates(const sensor_msgs::msg::JointState& /*js*/) override {
    // No-op
    return true;
  }

  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override {
    // Provide a neutral robot state for testing
    auto names = this->getIKSolver()->getJointNames();
    js.name = names;
    js.position = std::vector<double>(names.size(), 0.0);
    js.header.stamp = this->clock->now();

    endEffectorPose.header.stamp = this->clock->now();
    endEffectorPose.header.frame_id = "world";
    endEffectorPose.pose.orientation.w = 1.0;
    endEffectorPose.pose.position.x = 0.0;
    endEffectorPose.pose.position.y = 0.0;
    endEffectorPose.pose.position.z = 0.0;
    return true;
  }
};

#endif // NULL_MOTION_PLUGIN_HPP
