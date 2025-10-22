#ifndef PF_KINEMATICS_HPP
#define PF_KINEMATICS_HPP
#include <string>
#include <vector>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

class PFKinematics {
public:
  PFKinematics() = default;
  explicit PFKinematics(const std::string& urdfFileName);
  ~PFKinematics() = default;

  pinocchio::Model& getModel() { return this->model; }

  // Caching API for fast repeated FK on the same joint/link sets
  void initializeCaches(
    const std::vector<std::string>& jointNames,
    const std::vector<std::string>& linkNames);

  // Compute transforms using cached indices; 'out' is aligned with cached linkNames
  void computeLinkTransforms(
    const std::vector<double>& jointPositions,
    std::vector<Eigen::Affine3d>& out);

  bool areCachesInitialized() const { return this->cachesReady; }
  const std::vector<std::string>& cachedJointNames() const { return this->jointNamesCache; }
  const std::vector<std::string>& cachedLinkNames() const { return this->linkNamesCache; }

private:
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<std::string> jointNamesCache;
  std::vector<std::string> linkNamesCache;
  std::vector<int> jointQIndices; // idx_q per cached joint (or -1 if not found/unsupported)
  std::vector<int> frameIDCache;       // frame id per cached link (or -1 if not found)
  bool cachesReady = false;
};

#endif // PF_KINEMATICS_HPP
