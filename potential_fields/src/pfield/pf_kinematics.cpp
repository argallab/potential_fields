#include "pfield/pf_kinematics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>

PFKinematics::PFKinematics(const std::string& urdfFileName) {
  if (urdfFileName.empty()) {
    throw std::invalid_argument("PFKinematics: urdfFileName is empty; expected a path to a URDF file");
  }
  try {
    pinocchio::urdf::buildModel(urdfFileName, this->model);
  }
  catch (const std::exception& e) {
    throw std::runtime_error(std::string("PFKinematics: Failed to load URDF model from '") + urdfFileName + "': " + e.what());
  }
  this->data = pinocchio::Data(this->model);
}

std::unordered_map<std::string, Eigen::Affine3d> PFKinematics::jointAnglesToLinkTransforms(
  const std::unordered_map<std::string, double>& jointAngles,
  const std::vector<std::string>& linkNames) {
  // Create configuration vector
  Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
  for (const auto& [jointName, angle] : jointAngles) {
    int jointId = this->model.getJointId(jointName);
    if (jointId == -1) {
      throw std::runtime_error("Joint name " + jointName + " not found in model joints");
    }
    int idx = this->model.joints[jointId].idx_q();
    int nq = this->model.joints[jointId].nq();
    if (nq != 1) {
      throw std::runtime_error("Joint " + jointName + " has nq != 1; only single-DoF joints are supported");
    }
    q(idx) = angle;
  }
  // Perform forward kinematics
  pinocchio::forwardKinematics(this->model, this->data, q);
  pinocchio::updateFramePlacements(this->model, this->data);
  // Get transforms for requested links
  std::unordered_map<std::string, Eigen::Affine3d> linkToTransformMap;
  for (const auto& linkName : linkNames) {
    int frameId = this->model.getFrameId(linkName);
    if (frameId == -1) {
      throw std::runtime_error("Link name " + linkName + " not found in model frames");
    }
    const pinocchio::SE3& oMf = this->data.oMf[frameId];
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.linear() = oMf.rotation();
    transform.translation() = oMf.translation();
    linkToTransformMap[linkName] = transform;
  }
  return linkToTransformMap;
}
