#ifndef PF_KINEMATICS_HPP
#define PF_KINEMATICS_HPP
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

// Pinocchio full headers (Model and Data must be complete in the header)
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pf_obstacle.hpp"

class PFKinematics {
public:
  PFKinematics() = delete;
  explicit PFKinematics(const std::string& urdfFileName);
  ~PFKinematics() = default;

  void setJointOrder(const std::vector<std::string>& jointNames);

  std::vector<Eigen::Affine3d> jointAnglesToLinkTransforms(
    const std::vector<double>& jointAngles,
    const std::vector<std::string>& linkNames);

  std::vector<PotentialFieldObstacle> getObstaclesFromJointAngles(
    const std::vector<double>& jointAngles,
    const std::vector<std::string>& linkNames);

private:
  std::vector<std::string> jointOrder; // Order of joints for IK
  std::unordered_map<std::string, int> jointNameToIndex; // Map joint name to index in jointOrder
  // Pinocchio model and data
  pinocchio::Model model;
  pinocchio::Data data;

};

#endif // PF_KINEMATICS_HPP
