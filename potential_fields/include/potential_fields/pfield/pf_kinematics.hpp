#ifndef PF_KINEMATICS_HPP
#define PF_KINEMATICS_HPP
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <unordered_map>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

class PFKinematics {
public:
  PFKinematics() = default;
  explicit PFKinematics(const std::string& urdfFileName);
  ~PFKinematics() = default;

  std::unordered_map<std::string, Eigen::Affine3d> jointAnglesToLinkTransforms(
    const std::unordered_map<std::string, double>& jointAngles,
    const std::vector<std::string>& linkNames);

  pinocchio::Model& getModel() { return this->model; }

private:
  // std::unordered_map<std::string, double> currentJointAngles; // (Joint Name -> angle) [rad]
  pinocchio::Model model;
  pinocchio::Data data;
};

#endif // PF_KINEMATICS_HPP
