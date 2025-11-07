#ifndef XARM_PLUGIN_HPP
#define XARM_PLUGIN_HPP

#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "motion_plugin.hpp"

class XArmIKSolver : public IKSolver {
public:
  XArmIKSolver();
  ~XArmIKSolver() override = default;

  /**
   * @brief Solve the IK problem for the XArm robot given a target end-effector pose, a seed joint configuration.
   *        Solving both the JointAngles and the Jacobian in one call to avoid redundant computations.
   *
   * @param[in] targetPose The desired end-effector pose expressed in the robot's base frame
   * @param[in] seed The current joint configuration to use as a seed for the IK solver [rad]
   * @param[out] solution The resulting joint configuration [rad]
   * @param[out] J The resulting Jacobian matrix (6xN) where N is the number of joints
   * @param[out] errorMsg An error message string to be populated if the IK fails
   * @return true if the IK problem was solved successfully, false otherwise
   */
  bool solve(const Eigen::Isometry3d& targetPose, const std::vector<double>& seed,
    std::vector<double>& solution, Eigen::Matrix<double, 6, Eigen::Dynamic>& J, std::string& errorMsg) override;

  // Jacobian-only computation to satisfy IKSolver interface,
  // but XArm Jacobian needs targetPose so this function is not implemented
  bool computeJacobian(
    [[maybe_unused]] const std::vector<double>& jointPositions,
    [[maybe_unused]] Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    return false;
  }

  std::vector<std::string> getJointNames() const override {
    return {"xarm_joint1", "xarm_joint2", "xarm_joint3", "xarm_joint4",
            "xarm_joint5", "xarm_joint6", "xarm_joint7"};
  }

  std::vector<double> getHomeConfiguration() const override {
    return std::vector<double>(this->homeJointAngles.cbegin(), this->homeJointAngles.cend());
  }

private:
  // The joint angles of the XArm in the "home" position [rad]
  const std::array<double, 7> homeJointAngles = {0,0,0,0,0,0,0};
  // The homogeneous transform from the robot's base link [O] to the end-effector [E] in the "home" position
  Eigen::Matrix4d homeTransformOE;
};

class XArmPlugin : public MotionPlugin {
public:
  XArmPlugin();
  ~XArmPlugin() override;

  bool sendCartesianTwist(const geometry_msgs::msg::Twist& endEffectorTwist) override;
  bool sendJointStates(const sensor_msgs::msg::JointState& js) override;
  bool readRobotState(sensor_msgs::msg::JointState& js, geometry_msgs::msg::PoseStamped& endEffectorPose) override;
};


#endif // !XARM_PLUGIN_HPP
