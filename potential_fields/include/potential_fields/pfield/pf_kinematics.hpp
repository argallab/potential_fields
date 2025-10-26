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

#include "pfield/pf_obstacle.hpp"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"

struct CollisionCatalogEntry {
  std::string id;         // unique obstacle id (link::name or link::colN)
  std::string linkName;  // link this collision belongs to
  urdf::CollisionSharedPtr col; // collision element
};


class PFKinematics {
public:
  PFKinematics() = default;
  PFKinematics(
    const std::string& urdfFileName, const std::vector<std::string>& jointNames,
    const double influenceZoneScale, const double repulsiveGain);
  ~PFKinematics() = default;

  pinocchio::Model& getModel() { return this->model; }

  // Caching API for fast repeated FK on the same joint/link sets
  void initializeCaches(
    const std::vector<std::string>& jointNames,
    const std::vector<std::string>& linkNames);

  // Compute transforms using cached indices
  std::vector<Eigen::Affine3d> computeLinkTransforms(const std::vector<double>& jointPositions);

  /**
   * @brief Using cached transforms vector aligned to collisionLinkNames,
   *        build obstacles using updated poses
   *
   * @param transforms The new poses of collision links, aligned to collisionLinkNames
   * @return std::vector<PotentialFieldObstacle> PotentialFieldObstacle objects with updated poses
   */
  std::vector<PotentialFieldObstacle> buildObstaclesFromTransforms(const std::vector<Eigen::Affine3d>& transforms);

  /**
   * @brief Builds the collision catalog from the URDF model, holding info about
   *        each collision object's link, name, and Collision pointer.
   *
   * @param model The URDF model
   * @param influenceZoneScale The influence zone scale for the robot obstacles
   * @param repulsiveGain The repulsive gain for the robot obstacles
   * @return std::vector<CollisionCatalogEntry> The built collision catalog
   */
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model,
    const double influenceZoneScale, const double repulsiveGain);

  /**
   * @brief Builds an Obstacle message from a URDF Collision object and given pose
   *
   * @param frameID The frame ID for the obstacle's pose to be defined in
   * @param influenceZoneScale The influence zone scale for the obstacle
   * @param collisionObject The URDF Collision object, defining geometry and type
   * @param position The position of the obstacle
   * @param orientation The orientation of the obstacle
   * @return PotentialFieldObstacle The constructed PFObstacle to be inserted in the PF
   */
  PotentialFieldObstacle obstacleFromCollisionObject(
    const std::string& frameID,
    const double influenceZoneScale, const double repulsiveGain,
    const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  std::vector<PotentialFieldObstacle> updateObstaclesFromJointAngles(const std::vector<double>& jointAngles);

  bool areCachesInitialized() const { return this->cachesReady; }
  const std::vector<std::string>& cachedJointNames() const { return this->jointNamesCache; }
  const std::vector<std::string>& cachedLinkNames() const { return this->linkNamesCache; }

private:
  pinocchio::Model model;
  pinocchio::Data data;
  urdf::Model robotModel;
  std::vector<std::string> jointNamesCache;
  std::vector<std::string> linkNamesCache;
  std::vector<int> jointQIndices; // idx_q per cached joint (or -1 if not found/unsupported)
  std::vector<int> frameIDCache;       // frame id per cached link (or -1 if not found)
  bool cachesReady = false;
  std::vector<std::string> collisionLinkNames; // Names of links with collision geometry
  std::vector<CollisionCatalogEntry> collisionCatalog; // Catalog of collision objects from URDF
  std::vector<PotentialFieldObstacle> obstacleGeometryTemplates; // pose will be updated per callback

};

#endif // PF_KINEMATICS_HPP
