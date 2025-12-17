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

#ifdef USING_FRANKA
#include "robot_plugins/franka_plugin.hpp"
#include "solvers/weighted_ik.h"
#include <cmath>

FrankaIKSolver::FrankaIKSolver(IKSolverSearchParameters params) : IKSolver("GeoFIK"), ikParams(params) {
  this->homeTransformOE = franka_fk(this->homeJointAngles);
  this->solver = std::make_unique<WeightedIKSolver>(
    /*neutral_pose=*/ this->homeJointAngles,
    /*weight_manip=*/ ikParams.manipulabilityWeight,
    /*weight_neutral=*/ ikParams.neutralWeight,
    /*weight_current=*/ ikParams.currentWeight,
    /*verbose=*/ false
  );
}

bool FrankaIKSolver::solve(
  const Eigen::Isometry3d& targetPose,
  const std::vector<double>& seed,
  std::vector<double>& solution,
  Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
  std::string& errorMsg) {
  // Use provided seed if valid, otherwise fall back to home configuration
  const bool seed_ok = seed.size() == 7;
  const std::array<double, 7> currentConfiguration = seed_ok
    ? std::array<double, 7>{seed[0], seed[1], seed[2], seed[3], seed[4], seed[5], seed[6]}
  : this->homeJointAngles;

  const std::array<double, 3> targetPosition = {
    targetPose.translation().x(), targetPose.translation().y(), targetPose.translation().z()
  };
  const std::array<double, 9> targetOrientation = {
    targetPose.rotation()(0, 0), targetPose.rotation()(0, 1), targetPose.rotation()(0, 2),
    targetPose.rotation()(1, 0), targetPose.rotation()(1, 1), targetPose.rotation()(1, 2),
    targetPose.rotation()(2, 0), targetPose.rotation()(2, 1), targetPose.rotation()(2, 2)
  };
  try {
    WeightedIKResult result = this->solver->solve_q7_optimized(
      /*target_position=*/targetPosition,
      /*target_orientation=*/targetOrientation,
      /*current_pose=*/currentConfiguration,
      /*q7_min=*/ this->ikParams.q7Min,
      /*q7_max=*/ this->ikParams.q7Max,
      /*tolerance=*/ this->ikParams.ikTolerance,
      /*max_iterations=*/ this->ikParams.ikMaxIterations
    );
    // Copy the result joint angles to the output solution vector
    solution = std::vector<double>(result.joint_angles.cbegin(), result.joint_angles.cend());
    // Copy the result Jacobian to the output J matrix, resizing to 6x7 since  Franka is 7-DOF
    J.resize(6, 7);
    for (int col = 0; col < J.cols(); ++col) {
      for (int row = 0; row < J.rows(); ++row) {
        J(row, col) = result.jacobian.at(col).at(row);
      }
    }
    return result.success;
  }
  catch (const std::exception& e) {
    errorMsg = e.what();
    return false;
  }
}

FrankaPlugin::FrankaPlugin(const std::string& hostname) : MotionPlugin("FrankaPlugin") {
  IKSolverSearchParameters ikSolverParams;
  this->assignIKSolver(std::make_shared<FrankaIKSolver>(ikSolverParams));
  if (!hostname.empty()) this->initializeRobot(hostname);
  this->currentJointAngles = this->getIKSolver()->getHomeConfiguration();
}

bool FrankaPlugin::initializeRobot(const std::string& hostname) {
  try {
    this->robot = std::make_unique<franka::Robot>(hostname);
    setDefaultBehavior(*this->robot);
  }
  catch (const franka::Exception& e) {
    std::cerr << "Failed to connect to Franka robot at " << hostname << ": " << e.what() << std::endl;
    return false;
  }
  // Successfully initialized the robot
  return true;
}

bool FrankaPlugin::startControlLoop(const franka::Duration& movementDuration) {
  if (!this->currentEEVelocity) {
    std::cerr << "No end-effector velocity command set" << std::endl;
    return false;
  }

  auto controlTime = franka::Duration(0.0);
  auto controlCallback = [this, &controlTime, &movementDuration](
    [[maybe_unused]] const franka::RobotState& robotState,
    franka::Duration period) -> franka::CartesianVelocities {
    // Move along the controlTime to see if we should finish the motion
    controlTime += period;
    if (controlTime >= movementDuration) {
      return franka::MotionFinished(*this->currentEEVelocity);
    }
    return *this->currentEEVelocity;
  };

  try {
    // Start the control using joint impedance mode
    bool motionFinished = false;
    auto activeControl = this->robot->startCartesianVelocityControl(
      research_interface::robot::Move::ControllerMode::kJointImpedance
    );
    while (!motionFinished) {
      auto robotStateAndDuration = activeControl->readOnce();
      const franka::RobotState& robotState = robotStateAndDuration.first;
      const franka::Duration& period = robotStateAndDuration.second;
      auto cartesianVelocities = controlCallback(robotState, period);
      motionFinished = cartesianVelocities.motion_finished;
      activeControl->writeOnce(cartesianVelocities);
    }
  }
  catch (const franka::Exception& e) {
    std::cerr << "Control loop failed: " << e.what() << std::endl;
    return false;
  }
  return true;
}

bool FrankaPlugin::sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) {
  const auto velocityCommand = franka::CartesianVelocities({
    endEffectorTwist.linear.x,
    endEffectorTwist.linear.y,
    endEffectorTwist.linear.z,
    endEffectorTwist.angular.x,
    endEffectorTwist.angular.y,
    endEffectorTwist.angular.z
    });
  this->currentEEVelocity = std::make_unique<franka::CartesianVelocities>(velocityCommand);
  return true;
}

bool FrankaPlugin::sendJointStates([[maybe_unused]] const sensor_msgs::msg::JointState& js) { return false; }

bool FrankaPlugin::readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) {
  // Offline/simulation fallback: if robot connection is not initialized, provide a safe dummy state
  if (!this->robot) {
    if (this->clock) { js.header.stamp = this->clock->now(); }
    else { js.header.stamp = rclcpp::Time(0); }
    js.name = this->getIKSolver()->getJointNames();
    js.position = std::vector<double>(7, 0.0);
    js.velocity = std::vector<double>(7, 0.0);
    js.effort = std::vector<double>(7, 0.0);
    this->currentJointAngles = js.position;

    if (this->clock) { endEffectorPose.header.stamp = this->clock->now(); }
    else { endEffectorPose.header.stamp = rclcpp::Time(0); }
    endEffectorPose.header.frame_id = "fer_link8";
    endEffectorPose.pose.position.x = 0.0;
    endEffectorPose.pose.position.y = 0.0;
    endEffectorPose.pose.position.z = 0.0;
    endEffectorPose.pose.orientation.w = 1.0;
    endEffectorPose.pose.orientation.x = 0.0;
    endEffectorPose.pose.orientation.y = 0.0;
    endEffectorPose.pose.orientation.z = 0.0;

    return true;
  }
  try {
    franka::RobotState robotState = this->robot->readOnce();
    // Convert franka::RobotState to sensor_msgs::msg::JointState
    if (this->clock) { js.header.stamp = this->clock->now(); }
    else { js.header.stamp = rclcpp::Time(0); }
    js.name = this->getIKSolver()->getJointNames();
    js.position = std::vector<double>(robotState.q.data(), robotState.q.data() + 7);
    js.velocity = std::vector<double>(robotState.dq.data(), robotState.dq.data() + 7);
    js.effort = std::vector<double>(robotState.tau_J.data(), robotState.tau_J.data() + 7);
    this->currentJointAngles = js.position;
    // Set end-effector pose
    if (this->clock) { endEffectorPose.header.stamp = this->clock->now(); }
    else { endEffectorPose.header.stamp = rclcpp::Time(0); }
    endEffectorPose.header.frame_id = "fer_link8";
    endEffectorPose.pose.position.x = robotState.O_T_EE[12];
    endEffectorPose.pose.position.y = robotState.O_T_EE[13];
    endEffectorPose.pose.position.z = robotState.O_T_EE[14];
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix <<
      robotState.O_T_EE[0], robotState.O_T_EE[1], robotState.O_T_EE[2],
      robotState.O_T_EE[4], robotState.O_T_EE[5], robotState.O_T_EE[6],
      robotState.O_T_EE[8], robotState.O_T_EE[9], robotState.O_T_EE[10];
    Eigen::Quaterniond quaternion(rotationMatrix);
    endEffectorPose.pose.orientation.x = quaternion.x();
    endEffectorPose.pose.orientation.y = quaternion.y();
    endEffectorPose.pose.orientation.z = quaternion.z();
    endEffectorPose.pose.orientation.w = quaternion.w();

    return true;
  }
  catch (const franka::Exception& e) {
    std::cerr << "Failed to read Franka robot state: " << e.what() << std::endl;
    return false;
  }
}

#endif // USING_FRANKA
