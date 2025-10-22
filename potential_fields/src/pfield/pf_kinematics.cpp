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

void PFKinematics::initializeCaches(
  const std::vector<std::string>& jointNames,
  const std::vector<std::string>& linkNames) {
  jointNamesCache = jointNames;
  linkNamesCache = linkNames;
  jointQIndices.assign(jointNames.size(), -1);
  frameIDCache.assign(linkNames.size(), -1);

  // Map joints to q indices (only 1-DoF joints supported for now)
  for (size_t i = 0; i < jointNames.size(); ++i) {
    const auto& jn = jointNames[i];
    if (!model.existJointName(jn)) continue;
    pinocchio::JointIndex jid = model.getJointId(jn);
    const auto& jmodel = model.joints[jid];
    if (jmodel.nq() == 1) {
      jointQIndices[i] = jmodel.idx_q();
    }
  }

  // Map links to frame IDs
  for (size_t k = 0; k < linkNames.size(); ++k) {
    const auto& ln = linkNames[k];
    if (!model.existFrame(ln)) continue;
    frameIDCache[k] = model.getFrameId(ln);
  }

  this->cachesReady = true;
}

void PFKinematics::computeLinkTransforms(const std::vector<double>& jointPositions,
  std::vector<Eigen::Affine3d>& out) {
  if (!cachesReady) {
    throw std::runtime_error("PFKinematics caches not initialized");
  }
  if (jointPositions.size() != jointNamesCache.size()) {
    throw std::runtime_error("PFKinematics: jointPositions size does not match cached joint names size");
  }

  // Fill q using cached indices
  // Note: We only set indices present; others remain from previous call (set nearest to zero if desired)
  Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
  for (size_t i = 0; i < jointPositions.size(); ++i) {
    int qi = jointQIndices[i];
    if (qi >= 0) {
      q(qi) = jointPositions[i];
    }
  }

  // Fast FK for frames
  pinocchio::framesForwardKinematics(model, data, q);

  // Write out transforms aligned to linkNamesCache
  out.resize(frameIDCache.size());
  for (size_t k = 0; k < frameIDCache.size(); ++k) {
    int fid = frameIDCache[k];
    if (fid >= 0) {
      const pinocchio::SE3& oMf = data.oMf[fid];
      Eigen::Affine3d tf(Eigen::Affine3d::Identity());
      tf.linear() = oMf.rotation();
      tf.translation() = oMf.translation();
      out[k] = tf;
    }
    else {
      out[k] = Eigen::Affine3d::Identity();
    }
  }
}
