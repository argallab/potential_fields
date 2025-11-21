// Implementation file for XArm IK + Motion plugin.
// Keep constructors/destructors out-of-line so the translation unit
// becomes the key location for vtable emission, avoiding undefined reference
// to vtable errors if this TU was previously not linked.

#include <robot_plugins/xarm_plugin.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/SVD>

XArmIKSolver::XArmIKSolver() : IKSolver("xarm_ik_solver") {
  this->homeTransformOE = Eigen::Matrix4d::Identity();
  this->homeTransformOE(0, 3) = 0.4; // example x offset
  this->homeTransformOE(1, 3) = 0.0; // example y offset
  this->homeTransformOE(2, 3) = 0.4; // example z offset

  // Initialize KDL chain
  std::string urdf_path;
  try {
    urdf_path = ament_index_cpp::get_package_share_directory("pfields_demo") + "/urdf/xarm7.urdf";
  } catch (const std::exception& e) {
    std::cerr << "Error finding pfields_demo package: " << e.what() << std::endl;
    return;
  }

  urdf::Model model;
  if (!model.initFile(urdf_path)) {
    std::cerr << "Failed to parse URDF file " << urdf_path << std::endl;
    return;
  }

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    std::cerr << "Failed to construct KDL tree from URDF" << std::endl;
    return;
  }

  std::string root = "link_base";
  std::string tip = "link_eef";

  if (!kdl_tree.getChain(root, tip, kdl_chain_)) {
    std::cerr << "Failed to get KDL chain from " << root << " to " << tip << std::endl;
    return;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  initialized_ = true;
}

bool XArmIKSolver::solve(
  const Eigen::Isometry3d& targetPose,
  const std::vector<double>& seed,
  std::vector<double>& solution,
  Eigen::Matrix<double, 6, Eigen::Dynamic>& J,
  std::string& errorMsg) {
  
  if (!initialized_) {
    errorMsg = "XArmIKSolver not initialized (failed to load URDF or KDL chain)";
    return false;
  }

  // Use provided seed if valid, otherwise fall back to home configuration
  KDL::JntArray q(kdl_chain_.getNrOfJoints());
  if (seed.size() == kdl_chain_.getNrOfJoints()) {
    for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); ++i) q(i) = seed[i];
  } else {
    for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); ++i) q(i) = homeJointAngles[i];
  }

  int max_iter = 500;
  double tol = 1e-4;
  double lambda = 0.5; // step size

  for (int iter = 0; iter < max_iter; ++iter) {
    // FK
    KDL::Frame current_pose;
    if (fk_solver_->JntToCart(q, current_pose) < 0) {
      errorMsg = "FK solver failed";
      return false;
    }
    
    // Error
    Eigen::Vector3d p_target = targetPose.translation();
    Eigen::Vector3d p_current(current_pose.p.x(), current_pose.p.y(), current_pose.p.z());
    Eigen::Vector3d p_err = p_target - p_current;
    
    Eigen::Quaterniond q_target(targetPose.rotation());
    
    Eigen::Matrix3d R_current;
    for(int i=0; i<3; ++i)
      for(int j=0; j<3; ++j)
        R_current(i,j) = current_pose.M(i,j);

    Eigen::Quaterniond q_current(R_current);
    if (q_current.dot(q_target) < 0) q_current.coeffs() *= -1;
    
    Eigen::AngleAxisd aa(q_target * q_current.inverse());
    Eigen::Vector3d r_err = aa.axis() * aa.angle();

    Eigen::Matrix<double, 6, 1> err;
    err << p_err, r_err;

    if (err.norm() < tol) {
      // Converged
      solution.resize(kdl_chain_.getNrOfJoints());
      for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); ++i) solution[i] = q(i);
      
      KDL::Jacobian J_kdl(kdl_chain_.getNrOfJoints());
      jac_solver_->JntToJac(q, J_kdl);
      J = J_kdl.data;
      return true;
    }

    // Jacobian
    KDL::Jacobian J_kdl(kdl_chain_.getNrOfJoints());
    jac_solver_->JntToJac(q, J_kdl);
    Eigen::MatrixXd J_eigen = J_kdl.data;

    // Jparse
    Eigen::MatrixXd J_parse, J_safety_nullspace;
    std::vector<int> jparse_singular_index(J_eigen.rows(), 0);
    Eigen::MatrixXd U_safety, U_new_proj, U_new_sing;
    Eigen::VectorXd S_new_safety, S_new_proj, Phi;
    
    Jparse_calculation(J_eigen, J_parse, J_safety_nullspace, jparse_singular_index, U_safety, S_new_safety, U_new_proj, S_new_proj, U_new_sing, Phi, gamma_, singular_direction_gain_position_, singular_direction_gain_angular_);

    // Update q
    Eigen::VectorXd dq = J_parse * err;
    
    for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); ++i) {
      q(i) += lambda * dq(i);
    }
  }

  errorMsg = "Failed to converge";
  return false;
}

bool XArmIKSolver::computeJacobian(
    const std::vector<double>& jointPositions,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J) {
  if (!initialized_) return false;
  if (jointPositions.size() != kdl_chain_.getNrOfJoints()) return false;
  
  KDL::JntArray q(kdl_chain_.getNrOfJoints());
  for(unsigned int i=0; i<kdl_chain_.getNrOfJoints(); ++i) q(i) = jointPositions[i];
  
  KDL::Jacobian J_kdl(kdl_chain_.getNrOfJoints());
  jac_solver_->JntToJac(q, J_kdl);
  J = J_kdl.data;
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

void XArmIKSolver::Jparse_calculation(const Eigen::MatrixXd& J, Eigen::MatrixXd& J_parse, Eigen::MatrixXd& J_safety_nullspace, std::vector<int>& jparse_singular_index, Eigen::MatrixXd& U_safety, Eigen::VectorXd& S_new_safety, Eigen::MatrixXd& U_new_proj, Eigen::VectorXd& S_new_proj, Eigen::MatrixXd& U_new_sing, Eigen::VectorXd& Phi, double& gamma, double& singular_direction_gain_position, double& singular_direction_gain_angular) {
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

XArmPlugin::XArmPlugin() : MotionPlugin("xarm_motion_plugin") {
  this->ikSolver = std::make_shared<XArmIKSolver>();
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
