#ifndef FRANKA_PLUGIN_HPP
#define FRANKA_PLUGIN_HPP

#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"
#include "weighted_ik.h"
#include "motion_plugin.hpp"

/**
 * @brief A struct to hold parameters for the IK solver search
 *
 * @note These parameters were obtained from the original WeightedIk implementation
 *       by Zhengyang Kris Weng.
 *
 */
struct IKSolverSearchParameters {
  double q7Min = -1.0; // Minimum joint 7 angle [rad]
  double q7Max = 2.0; // Maximum joint 7 angle [rad]
  double ikTolerance = 1e-6; // Joint Position tolerance for IK [rad]
  int ikMaxIterations = 50; // Maximum number of iterations for IK
  double manipulabilityWeight = 1.0; // Weight for manipulability in the IK cost function
  double neutralWeight = 0.1; // Weight for distance from neutral pose in the IK cost function
  double currentWeight = 0.2; // Weight for distance from current pose in the IK cost function
};

class FrankaIKSolver : public IKSolver {
public:
  explicit FrankaIKSolver(IKSolverSearchParameters params);
  ~FrankaIKSolver() override = default;

  /**
   * @brief Solve the IK problem for the Franka robot given a target end-effector pose, a seed joint configuration.
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
  // but Franka Jacobian needs targetPose so this function is not implemented
  bool computeJacobian(
    [[maybe_unused]] const std::vector<double>& jointPositions,
    [[maybe_unused]] Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override {
    return false;
  }

  std::vector<std::string> getJointNames() const override {
    return {"fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
            "fer_joint5", "fer_joint6", "fer_joint7"};
  }

  std::vector<double> getHomeConfiguration() const override {
    return std::vector<double>(this->homeJointAngles.cbegin(), this->homeJointAngles.cend());
  }

private:
  // The joint angles of the franka in the "home" position [rad]
  const std::array<double, 7> homeJointAngles = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  // The homogeneous transform from the robot's base link [O] to the end-effector [E] in the "home" position
  Eigen::Matrix4d homeTransformOE;
  std::unique_ptr<WeightedIKSolver> solver;
  const IKSolverSearchParameters ikParams;
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
