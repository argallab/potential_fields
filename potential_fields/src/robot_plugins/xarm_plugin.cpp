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

// Implementation file for XArm IK + Motion plugin.
// Keep constructors/destructors out-of-line so the translation unit
// becomes the key location for vtable emission, avoiding undefined reference
// to vtable errors if this TU was previously not linked.

#include <robot_plugins/xarm_plugin.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

XArmPlugin::XArmPlugin() : MotionPlugin("xarm_motion_plugin") {
  std::string urdf_path;
  try {
    urdf_path = ament_index_cpp::get_package_share_directory("potential_fields_demos") + "/urdf/xarm7.urdf";
  }
  catch (const std::exception& e) {
    std::cerr << "Error finding potential_fields_demos package: " << e.what() << std::endl;
  }
  this->ikSolver = std::make_shared<pfield::XArmIKSolver>(urdf_path);
}

XArmPlugin::~XArmPlugin() = default;

bool XArmPlugin::sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) {
  return false;
}
bool XArmPlugin::sendJointStates(const sensor_msgs::msg::JointState& js) {
  return false;
}
bool XArmPlugin::readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) {
  return false;
}
