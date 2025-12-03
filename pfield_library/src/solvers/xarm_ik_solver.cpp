#include "solvers/xarm_ik_solver.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <Eigen/SVD>
#include <iostream>

namespace pfield {

  XArmIKSolver::XArmIKSolver(const std::string& urdf_path) : IKSolver("xarm_ik_solver") {
    this->homeTransformOE = Eigen::Matrix4d::Identity();
    this->homeTransformOE(0, 3) = 0.4; // example x offset
    this->homeTransformOE(1, 3) = 0.0; // example y offset
    this->homeTransformOE(2, 3) = 0.4; // example z offset

    try {
      pinocchio::urdf::buildModel(urdf_path, this->model);
    }
    catch (const std::exception& e) {
      std::cerr << "Failed to load URDF from " << urdf_path << ": " << e.what() << std::endl;
      return;
    }

    this->data = pinocchio::Data(this->model);
    initialized_ = true;
  }

  bool XArmIKSolver::solve(
    const Eigen::Isometry3d& targetPose,
    const std::vector<double>& seed,
    std::vector<double>& solution,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
    std::string& errorMsg) {
    if (!initialized_) {
      errorMsg = "XArmIKSolver not initialized (failed to load URDF)";
      return false;
    }

    std::vector<std::string> joint_names = getJointNames();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

    // Initialize q from seed or home
    const std::vector<double>& initial_config = (seed.size() == 7) ? seed : std::vector<double>(homeJointAngles.begin(), homeJointAngles.end());

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model.existJointName(joint_names[i])) {
        pinocchio::JointIndex id = model.getJointId(joint_names[i]);
        int idx_q = model.joints[id].idx_q();
        if (idx_q >= 0) {
          q(idx_q) = initial_config[i];
        }
      }
    }

    int max_iter = 500;
    double tol = 1e-4;
    double lambda = 0.5; // step size

    if (!model.existFrame("link_eef")) {
      errorMsg = "Frame 'link_eef' not found in URDF";
      return false;
    }
    pinocchio::FrameIndex frameId = model.getFrameId("link_eef");

    for (int iter = 0; iter < max_iter; ++iter) {
      // FK
      pinocchio::framesForwardKinematics(model, data, q);
      const pinocchio::SE3& current_pose = data.oMf[frameId];

      // Error
      Eigen::Vector3d p_target = targetPose.translation();
      Eigen::Vector3d p_current = current_pose.translation();
      Eigen::Vector3d p_err = p_target - p_current;

      Eigen::Quaterniond q_target(targetPose.rotation());
      Eigen::Quaterniond q_current(current_pose.rotation());

      if (q_current.dot(q_target) < 0) q_current.coeffs() *= -1;

      Eigen::AngleAxisd aa(q_target * q_current.inverse());
      Eigen::Vector3d r_err = aa.axis() * aa.angle();

      Eigen::Matrix<double, 6, 1> err;
      err << p_err, r_err;

      if (err.norm() < tol) {
        // Converged
        solution.resize(7);
        for (size_t i = 0; i < joint_names.size(); ++i) {
          if (model.existJointName(joint_names[i])) {
            pinocchio::JointIndex id = model.getJointId(joint_names[i]);
            int idx_q = model.joints[id].idx_q();
            if (idx_q >= 0) {
              solution[i] = q(idx_q);
            }
          }
        }

        // Compute Jacobian for output
        pinocchio::computeJointJacobians(model, data, q);
        pinocchio::Data::Matrix6x J_full(6, model.nv);
        J_full.setZero();
        pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

        J.resize(6, 7);
        for (size_t i = 0; i < joint_names.size(); ++i) {
          if (model.existJointName(joint_names[i])) {
            pinocchio::JointIndex id = model.getJointId(joint_names[i]);
            int idx_v = model.joints[id].idx_v();
            if (idx_v >= 0) {
              J.col(i) = J_full.col(idx_v);
            }
          }
        }
        return true;
      }

      // Jacobian
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::Data::Matrix6x J_full(6, model.nv);
      J_full.setZero();
      pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

      Eigen::MatrixXd J_eigen(6, 7);
      for (size_t i = 0; i < joint_names.size(); ++i) {
        if (model.existJointName(joint_names[i])) {
          pinocchio::JointIndex id = model.getJointId(joint_names[i]);
          int idx_v = model.joints[id].idx_v();
          if (idx_v >= 0) {
            J_eigen.col(i) = J_full.col(idx_v);
          }
        }
      }

      // Jparse
      Eigen::MatrixXd J_parse, J_safety_nullspace;
      std::vector<int> jparse_singular_index(J_eigen.rows(), 0);
      Eigen::MatrixXd U_safety, U_new_proj, U_new_sing;
      Eigen::VectorXd S_new_safety, S_new_proj, Phi;

      Jparse_calculation(
        J_eigen, J_parse, J_safety_nullspace, jparse_singular_index, U_safety,
        S_new_safety, U_new_proj, S_new_proj, U_new_sing, Phi, gamma_,
        singular_direction_gain_position_, singular_direction_gain_angular_
      );

      // Update q
      Eigen::VectorXd dq = J_parse * err;

      for (size_t i = 0; i < joint_names.size(); ++i) {
        if (model.existJointName(joint_names[i])) {
          pinocchio::JointIndex id = model.getJointId(joint_names[i]);
          int idx_q = model.joints[id].idx_q();
          int idx_v = model.joints[id].idx_v();
          if (idx_q >= 0 && idx_v >= 0) {
            q(idx_q) += lambda * dq(i);
          }
        }
      }
    }

    errorMsg = "Failed to converge";
    return false;
  }

  bool XArmIKSolver::computeJacobian(
    const std::vector<double>& joint_positions,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J) {

    if (!initialized_) {
      return false;
    }

    std::vector<std::string> joint_names = getJointNames();
    if (joint_positions.size() != 7) {
      return false;
    }

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model.existJointName(joint_names[i])) {
        pinocchio::JointIndex id = model.getJointId(joint_names[i]);
        int idx_q = model.joints[id].idx_q();
        if (idx_q >= 0) {
          q(idx_q) = joint_positions[i];
        }
      }
    }

    if (!model.existFrame("link_eef")) {
      return false;
    }
    pinocchio::FrameIndex frameId = model.getFrameId("link_eef");

    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::Data::Matrix6x J_full(6, model.nv);
    J_full.setZero();
    pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    J.resize(6, 7);
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (model.existJointName(joint_names[i])) {
        pinocchio::JointIndex id = model.getJointId(joint_names[i]);
        int idx_v = model.joints[id].idx_v();
        if (idx_v >= 0) {
          J.col(i) = J_full.col(idx_v);
        }
      }
    }

    return true;
  }

  Eigen::MatrixXd XArmIKSolver::pseudoInverse(const Eigen::MatrixXd& J, double tol) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd& singularValues = svd.singularValues();
    Eigen::MatrixXd S_pinv = Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().cols());

    for (int i = 0; i < singularValues.size(); ++i) {
      if (singularValues(i) > tol) {
        S_pinv(i, i) = 1.0 / singularValues(i);
      }
    }

    return svd.matrixV() * S_pinv * svd.matrixU().transpose();
  }

  void XArmIKSolver::Jparse_calculation(
    const Eigen::MatrixXd& J,
    Eigen::MatrixXd& J_parse,
    Eigen::MatrixXd& J_safety_nullspace,
    std::vector<int>& jparse_singular_index,
    Eigen::MatrixXd& U_safety,
    Eigen::VectorXd& S_new_safety,
    Eigen::MatrixXd& U_new_proj,
    Eigen::VectorXd& S_new_proj,
    Eigen::MatrixXd& U_new_sing,
    Eigen::VectorXd& Phi,
    double& gamma,
    double& singular_direction_gain_position,
    double& singular_direction_gain_angular) {
    /*
    Steps are as follows:
    1. Find the SVD of J
    2. Find the adjusted condition number and Jparse singular index
    3. Find the Projection Jacobian
    4. Find the Safety Jacobian
    5. Find the singular direction projection components
    6. Find the pseudo inverse of J_safety and J_proj
    7. Find Jparse
    8. Find the null space of J_safety
    */

    //1. Find the SVD of J
    Eigen::MatrixXd U, V;
    Eigen::VectorXd S;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();

    if (U.rows() == U.cols() && U.determinant() < 0.0) {
      int k = U.cols() - 1;     // pick the last column
      U.col(k) *= -1.0;
      V.col(k) *= -1.0;
    }

    U_safety = U; //Safety Jacobian shares the same U as the SVD of J

    //2. find the adjusted condition number
    double max_singular_value = S.maxCoeff();
    std::vector<double> adjusted_condition_numbers(S.size());
    for (int i = 0; i < S.size(); ++i) {
      adjusted_condition_numbers[i] = S(i) / max_singular_value;
    }

    //3. Find the Projection Jacobian
    std::vector<int> valid_indices;

    for (int i = 0; i < S.size(); ++i) {
      // keep only the elements whose singular value is greater than the threshold
      if (S(i) > gamma * max_singular_value) {
        valid_indices.push_back(i);
      }
      else {
        jparse_singular_index[i] = 1; // set the index to 1 if the singular value is less than the threshold
      }
    }

    U_new_proj = Eigen::MatrixXd(U.rows(), valid_indices.size());
    S_new_proj = Eigen::VectorXd(valid_indices.size());

    for (size_t i = 0; i < valid_indices.size(); ++i) {
      U_new_proj.col(i) = U.col(valid_indices[i]);
      S_new_proj(i) = S(valid_indices[i]);
    }
    Eigen::MatrixXd S_new_proj_matrix = Eigen::MatrixXd::Zero(U_new_proj.cols(), V.rows());
    for (int i = 0; i < S_new_proj.size(); ++i) {
      S_new_proj_matrix(i, i) = S_new_proj(i);
    }
    Eigen::MatrixXd J_proj = U_new_proj * S_new_proj_matrix * V.transpose();

    //4. Find the Safety Jacobian
    S_new_safety = Eigen::VectorXd(S.size());
    for (int i = 0; i < S.size(); ++i) {
      //if the singular value is greater than the threshold, keep it otherwise set it to the threshold
      if ((S(i) / max_singular_value) > gamma) {
        S_new_safety(i) = S(i);
      }
      else {
        S_new_safety(i) = gamma * max_singular_value;
      }
    }


    Eigen::MatrixXd S_new_safety_matrix = Eigen::MatrixXd::Zero(U.rows(), V.cols());
    for (int i = 0; i < S_new_safety.size(); ++i) {
      S_new_safety_matrix(i, i) = S_new_safety(i);
    }
    Eigen::MatrixXd J_safety = U * S_new_safety_matrix * V.transpose();

    //5. Find the singular direction projection components
    Eigen::MatrixXd Phi_matrix, Kp_singular, Phi_singular;
    std::vector<int> valid_indices_sing;
    bool set_empty_bool = true; // set to true if the valid indices are empty
    for (int i = 0; i < S.size(); ++i) {
      if (adjusted_condition_numbers[i] <= gamma) {
        set_empty_bool = false;
        valid_indices_sing.push_back(i);
      }
    }

    U_new_sing = Eigen::MatrixXd(U.rows(), valid_indices_sing.size());
    Phi = Eigen::VectorXd(valid_indices_sing.size());

    if (set_empty_bool == false) {
      for (size_t i = 0; i < valid_indices_sing.size(); ++i) {
        U_new_sing.col(i) = U.col(valid_indices_sing[i]);
        Phi(i) = adjusted_condition_numbers[valid_indices_sing[i]] / gamma;
      }

      Phi_matrix = Eigen::MatrixXd::Zero(Phi.size(), Phi.size());
      for (int i = 0; i < Phi.size(); ++i) {
        Phi_matrix(i, i) = Phi(i);
      }

      Kp_singular = Eigen::MatrixXd::Zero(U.rows(), U.cols());
      for (int i = 0; i < 3; ++i) {
        Kp_singular(i, i) = singular_direction_gain_position;
      }
      if (Kp_singular.cols() > 3) //checks if position only (J_v) or full jacobian
      {
        for (int i = 3; i < 6; ++i) {
          Kp_singular(i, i) = singular_direction_gain_angular;
        }
      }
      Phi_singular = U_new_sing * Phi_matrix * U_new_sing.transpose() * Kp_singular; // put it all together
    }

    //6. Find pseudo inverse of J_safety and J_proj
    Eigen::MatrixXd J_safety_pinv = pseudoInverse(J_safety);
    Eigen::MatrixXd J_proj_pinv = pseudoInverse(J_proj);

    //7. Find the Jparse
    if (set_empty_bool == false) {
      J_parse = J_safety_pinv * J_proj * J_proj_pinv + J_safety_pinv * Phi_singular;
    }
    else {
      J_parse = J_safety_pinv * J_proj * J_proj_pinv;
    }

    //8. Find the null space of J_safety
    J_safety_nullspace = Eigen::MatrixXd::Identity(J_safety.cols(), J_safety.cols()) - J_safety_pinv * J_safety;
  }

} // namespace pfield
