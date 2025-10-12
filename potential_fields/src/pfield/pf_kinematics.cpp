#include "pfield/pf_kinematics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>

PFKinematics::PFKinematics(const std::string& urdfFileName) {
  pinocchio::urdf::buildModel(urdfFileName, this->model);
  this->data = pinocchio::Data(this->model);
}

std::vector<Eigen::Affine3d> PFKinematics::jointAnglesToLinkTransforms(
  const std::vector<double>& jointAngles,
  const std::vector<std::string>& linkNames) {
  // Check joint angles size
  if (jointAngles.size() != this->jointOrder.size()) {
    throw std::runtime_error("Joint angles size does not match joint order size");
  }
  // Create configuration vector
  Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
  for (size_t i = 0; i < jointAngles.size(); ++i) {
    const std::string& jointName = this->jointOrder[i];
    auto it = this->jointNameToIndex.find(jointName);
    if (it != this->jointNameToIndex.end()) {
      int idx = it->second;
      q(idx) = jointAngles[i];
    }
    else {
      throw std::runtime_error("Joint name " + jointName + " not found in jointNameToIndex map");
    }
  }
  // Perform forward kinematics
  pinocchio::forwardKinematics(this->model, this->data, q);
  pinocchio::updateFramePlacements(this->model, this->data);
  // Get transforms for requested links
  std::vector<Eigen::Affine3d> transforms;
  for (const auto& linkName : linkNames) {
    int frameId = this->model.getFrameId(linkName);
    if (frameId == -1) {
      throw std::runtime_error("Link name " + linkName + " not found in model frames");
    }
    const pinocchio::SE3& oMf = this->data.oMf[frameId];
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());
    transform.linear() = oMf.rotation();
    transform.translation() = oMf.translation();
    transforms.push_back(transform);
  }
  return transforms;
}

std::vector<PotentialFieldObstacle> PFKinematics::getObstaclesFromJointAngles(
  const std::vector<double>& jointAngles,
  const std::vector<std::string>& linkNames) {
  std::vector<PotentialFieldObstacle> obstacles;
  auto transforms = this->jointAnglesToLinkTransforms(jointAngles, linkNames);
  for (const auto& transform : transforms) {
    auto position = transform.translation();
    Eigen::Quaterniond quat(transform.rotation());
    obstacles.push_back(PotentialFieldObstacle(
      /*frameID=*/"",
      /*centerPosition=*/position, /*orientation=*/quat,
      /*type=*/ObstacleType::BOX, /*group=*/ObstacleGroup::ROBOT,
      /*geometry=*/ObstacleGeometry(0.1, 0.1, 0.1, 0.1), // Example box geometry
      /*influenceZoneScale=*/1.5, /*repulsiveGain=*/1.0
    ));
  }
  return obstacles;
}
