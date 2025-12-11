#ifndef XARM_IK_SOLVER_HPP
#define XARM_IK_SOLVER_HPP

#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <array>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "solvers/ik_solver.hpp"

namespace pfield {

  class XArmIKSolver : public IKSolver {
  public:
    explicit XArmIKSolver(const std::string& urdf_path);
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

    // Jacobian-only computation to satisfy IKSolver interface
    bool computeJacobian(
      const std::vector<double>& jointPositions,
      Eigen::Matrix<double, 6, Eigen::Dynamic>& J) override;

    std::vector<std::string> getJointNames() const override {
      return {"joint1", "joint2", "joint3", "joint4",
              "joint5", "joint6", "joint7"};
    }

    std::vector<double> getHomeConfiguration() const override {
      return std::vector<double>(this->homeJointAngles.cbegin(), this->homeJointAngles.cend());
    }

  private:
    // The joint angles of the XArm in the "home" position [rad]
    const std::array<double, 7> homeJointAngles = {0, 0, 0, 0, 0, -M_PI_2, 0};
    // The homogeneous transform from the robot's base link [O] to the end-effector [E] in the "home" position
    Eigen::Matrix4d homeTransformOE;

    // Pinocchio members
    pinocchio::Model model;
    pinocchio::Data data;
    bool initialized_ = false;

    // Jparse parameters
    double gamma_ = 0.2;
    double singular_direction_gain_position_ = 1.0;
    double singular_direction_gain_angular_ = 1.0;

    // Helper methods
    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& J, double tol = 1e-6);
    void Jparse_calculation(
      const Eigen::MatrixXd& J, Eigen::MatrixXd& J_parse, Eigen::MatrixXd& J_safety_nullspace,
      std::vector<int>& jparse_singular_index, Eigen::MatrixXd& U_safety, Eigen::VectorXd& S_new_safety,
      Eigen::MatrixXd& U_new_proj, Eigen::VectorXd& S_new_proj, Eigen::MatrixXd& U_new_sing, Eigen::VectorXd& Phi,
      double& gamma, double& singular_direction_gain_position, double& singular_direction_gain_angular);
  };

} // namespace pfield

#endif // !XARM_IK_SOLVER_HPP
