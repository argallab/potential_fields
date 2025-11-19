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

#include <coal/collision.h>

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
  PFKinematics(const std::string& urdfFileName, const std::vector<std::string>& jointNames);
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
   * @param repulsiveGain The repulsive gain for the robot obstacles
   * @return std::vector<CollisionCatalogEntry> The built collision catalog
   */
  std::vector<CollisionCatalogEntry> buildCollisionCatalog(urdf::Model& model);

  /**
   * @brief Builds an Obstacle message from a URDF Collision object and given pose
   *
   * @param frameID The frame ID for the obstacle's pose to be defined in
   * @param collisionObject The URDF Collision object, defining geometry and type
   * @param position The position of the obstacle
   * @param orientation The orientation of the obstacle
   * @return PotentialFieldObstacle The constructed PFObstacle to be inserted in the PF
   */
  PotentialFieldObstacle obstacleFromCollisionObject(
    const std::string& frameID,
    const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);

  std::vector<PotentialFieldObstacle> updateObstaclesFromJointAngles(const std::vector<double>& jointAngles);

  bool areCachesInitialized() const { return this->cachesReady; }
  const std::vector<std::string>& cachedJointNames() const { return this->jointNamesCache; }
  const std::vector<std::string>& cachedLinkNames() const { return this->linkNamesCache; }

  /**
   * @brief Estimate a conservative bounding-sphere radius of the robot from its URDF
   *        using collision geometry. The radius is computed as the maximum of
   *        ||p_base|| + r_local across all collision objects, where p_base is the
   *        collision origin expressed in the base frame (at nominal zero joint values)
   *        and r_local is the local bounding-sphere radius of the collision geometry.
   *
   * Geometry handling:
   *  - Box(l,w,h): r_local = 0.5 * sqrt(l^2 + w^2 + h^2)
   *  - Sphere(r): r_local = r
   *  - Cylinder(r,len): r_local = sqrt(r^2 + (len/2)^2)
   *  - Mesh(file,scale): if mesh extents are unavailable, uses a conservative heuristic:
   *      r_local ≈ meshFallbackRadius * max(scale.x, scale.y, scale.z)
   *
   * @return double Estimated robot radius in meters. Returns 0.0 if no collision
   *         geometry is found or the URDF model is not initialized.
   */
  double estimateRobotExtentRadius();

private:
  pinocchio::Model model;
  pinocchio::Data data;
  urdf::Model robotModel;
  std::vector<std::string> jointNamesCache;
  std::vector<std::string> linkNamesCache;
  std::vector<int> jointQIndices; // idx_q per cached joint (or -1 if not found/unsupported)
  std::vector<int> frameIDCache;       // frame id per cached link (or -1 if not found)
  std::vector<std::string> collisionLinkNames; // Names of links with collision geometry
  std::vector<CollisionCatalogEntry> collisionCatalog; // Catalog of collision objects from URDF
  std::vector<PotentialFieldObstacle> obstacleGeometryTemplates; // pose will be updated per callback
  bool cachesReady = false;
};

#endif // PF_KINEMATICS_HPP
