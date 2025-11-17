#include "pfield/pf_kinematics.hpp"
#include "pfield/pfield_common.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>

PFKinematics::PFKinematics(const std::string& urdfFileName, const std::vector<std::string>& jointNames) {
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
  this->robotModel.initFile(urdfFileName);
  this->collisionCatalog = this->buildCollisionCatalog(this->robotModel);
  this->initializeCaches(jointNames, this->collisionLinkNames);
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

std::vector<Eigen::Affine3d> PFKinematics::computeLinkTransforms(const std::vector<double>& jointPositions) {
  if (!cachesReady) {
    throw std::runtime_error("PFKinematics caches not initialized");
  }
  if (jointPositions.size() != jointNamesCache.size()) {
    throw std::runtime_error(
      std::string("PFKinematics: jointPositions size (") + std::to_string(jointPositions.size()) +
      ") does not match cached joint names size (" + std::to_string(jointNamesCache.size()) + ")"
    );
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
  std::vector<Eigen::Affine3d> out(frameIDCache.size());
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
  return out;
}

std::vector<CollisionCatalogEntry> PFKinematics::buildCollisionCatalog(urdf::Model& model) {
  std::vector<CollisionCatalogEntry> catalog;
  this->obstacleGeometryTemplates.clear();
  for (const auto& [link_name, link] : model.links_) {
    if (!link) { continue; }
    // Lambda helper to add a collision object to the catalog with the correct naming
    auto addCollisionEntry = [&](const urdf::CollisionSharedPtr& col_ptr, size_t index) {
      if (!col_ptr || !col_ptr->geometry) { return; }
      CollisionCatalogEntry e;
      e.linkName = link_name;
      if (!col_ptr->name.empty()) {
        e.id = link_name + "::" + col_ptr->name;
      }
      else {
        e.id = link_name + "::col" + std::to_string(index);
      }
      e.col = col_ptr;
      // Build cached obstacle geometry templates aligned to catalog entries
      PotentialFieldObstacle templateObstacle = this->obstacleFromCollisionObject(
        e.id, *e.col, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
      );
      this->obstacleGeometryTemplates.push_back(templateObstacle);
      catalog.push_back(std::move(e));
      this->collisionLinkNames.push_back(link_name);
    };

    // Add collision objects, specifying the index for collision array objects
    if (!link->collision_array.empty()) {
      for (size_t i = 0; i < link->collision_array.size(); ++i) {
        addCollisionEntry(link->collision_array[i], i);
      }
    }
    else if (link->collision) {
      addCollisionEntry(link->collision, 0);
    }
  }
  return catalog;
}

std::vector<PotentialFieldObstacle> PFKinematics::buildObstaclesFromTransforms(const std::vector<Eigen::Affine3d>& transforms) {
  std::vector<PotentialFieldObstacle> collisionObstacles;
  collisionObstacles.reserve(this->collisionCatalog.size());
  // Each catalog entry aligns with collisionLinkNames and templates
  size_t N = this->collisionCatalog.size();
  for (size_t i = 0; i < N; ++i) {
    const auto& entry = this->collisionCatalog[i];
    // Find link index corresponding to this entry
    // We built collisionLinkNames in the same order as catalog push_back; i aligns with link name index.
    const Eigen::Affine3d& world_T_link = transforms[i];
    const urdf::Pose& origin = entry.col->origin;
    Eigen::Affine3d link_T_col =
      Eigen::Translation3d(origin.position.x, origin.position.y, origin.position.z) *
      Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    Eigen::Affine3d world_T_col = world_T_link * link_T_col;
    // Clone template and update pose only
    PotentialFieldObstacle obst = this->obstacleGeometryTemplates[i];
    obst.setPose(
      Eigen::Vector3d(
        world_T_col.translation().x(),
        world_T_col.translation().y(),
        world_T_col.translation().z()
      ),
      Eigen::Quaterniond(world_T_col.rotation())
    );
    collisionObstacles.push_back(obst);
  }
  return collisionObstacles;
}

PotentialFieldObstacle PFKinematics::obstacleFromCollisionObject(
  const std::string& frameID,
  const urdf::Collision& collisionObject,
  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
  ObstacleType obstacleType;
  double radius = 0.0;
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  bool isMesh = false;
  std::string meshResource;

  auto* geometry = collisionObject.geometry.get();
  if (urdf::Box* b = dynamic_cast<urdf::Box*>(geometry)) {
    obstacleType = ObstacleType::BOX;
    length = b->dim.x;
    width = b->dim.y;
    height = b->dim.z;
  }
  else if (urdf::Sphere* s = dynamic_cast<urdf::Sphere*>(geometry)) {
    obstacleType = ObstacleType::SPHERE;
    radius = s->radius;
  }
  else if (urdf::Cylinder* c = dynamic_cast<urdf::Cylinder*>(geometry)) {
    obstacleType = ObstacleType::CYLINDER;
    radius = c->radius;
    height = c->length;
  }
  else if (urdf::Mesh* m = dynamic_cast<urdf::Mesh*>(geometry)) {
    obstacleType = ObstacleType::MESH;
    length = m->scale.x;
    width = m->scale.y;
    height = m->scale.z;
    isMesh = true;
    meshResource = m->filename;
  }
  else {
    throw std::runtime_error("obstacleFromCollisionObject: Unsupported geometry type in collision object");
  }
  ObstacleGeometry obstacleGeom(radius, length, width, height);
  PotentialFieldObstacle obstacle(
    frameID, position, orientation, obstacleType, ObstacleGroup::ROBOT,
    obstacleGeom
  );
  // If mesh, set mesh resource and scale
  if (isMesh) { obstacle.setMeshProperties(meshResource, Eigen::Vector3d(length, width, height)); }
  return obstacle;
}

std::vector<PotentialFieldObstacle> PFKinematics::updateObstaclesFromJointAngles(const std::vector<double>& jointAngles) {
  return this->buildObstaclesFromTransforms(this->computeLinkTransforms(jointAngles));
}

double PFKinematics::estimateRobotExtentRadius() {
  // Require a valid URDF model
  if (this->robotModel.links_.empty()) { return 0.0; }

  // Prepare transforms for links that have collision geometry. Use nominal zero joint values.
  std::vector<double> q_zero;
  if (!this->jointNamesCache.empty()) {
    q_zero.assign(this->jointNamesCache.size(), 0.0);
  }
  else {
    // If caches are not initialized, we can still rely on Pinocchio model dimensions
    // but computeLinkTransforms requires caches. In that edge case, return 0.0 gracefully.
    return 0.0;
  }

  std::vector<Eigen::Affine3d> world_T_link;
  try {
    world_T_link = this->computeLinkTransforms(q_zero);
  }
  catch (...) {
    return 0.0;
  }

  // The collision catalog and collisionLinkNames align with the templates/order used
  const size_t N = std::min(world_T_link.size(), this->collisionCatalog.size());
  double max_extent = 0.0;

  for (size_t i = 0; i < N; ++i) {
    const auto& entry = this->collisionCatalog[i];
    if (!entry.col || !entry.col->geometry) { continue; }

    // Link pose in world/base frame and collision origin in link frame
    const Eigen::Affine3d& T_world_link = world_T_link[i];
    const Eigen::Affine3d T_link_col = urdfPoseToEigen(entry.col->origin);
    const Eigen::Affine3d T_world_col = T_world_link * T_link_col;
    const Eigen::Vector3d p_world = T_world_col.translation();

    // Compute local bounding-sphere radius r_local from geometry
    double r_local = 0.0;
    if (auto box = std::dynamic_pointer_cast<urdf::Box>(entry.col->geometry)) {
      const double l = box->dim.x, w = box->dim.y, h = box->dim.z;
      r_local = 0.5 * std::sqrt(l * l + w * w + h * h);
    }
    else if (auto sph = std::dynamic_pointer_cast<urdf::Sphere>(entry.col->geometry)) {
      r_local = sph->radius;
    }
    else if (auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(entry.col->geometry)) {
      r_local = std::sqrt(cyl->radius * cyl->radius + 0.25 * cyl->length * cyl->length);
    }
    else if (auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(entry.col->geometry)) {
      const double sx = mesh->scale.x > 0.0 ? mesh->scale.x : 1.0;
      const double sy = mesh->scale.y > 0.0 ? mesh->scale.y : 1.0;
      const double sz = mesh->scale.z > 0.0 ? mesh->scale.z : 1.0;
      const double smax = std::max(sx, std::max(sy, sz));
      // TODO(Sharwin24): Load mesh and compute real extents if possible
      const double meshRadius = 0.30; // default fallback radius in meters
      r_local = meshRadius * smax; // conservative heuristic without loading mesh
    }
    else {
      // Unsupported type; skip
      continue;
    }

    const double extent = p_world.norm() + r_local;
    if (extent > max_extent) { max_extent = extent; }
  }

  return max_extent; // 0.0 if nothing found
}

std::vector<double> PFKinematics::getMinClearancesFromJointAngles(const std::vector<double>& jointAngles,
  const std::vector<PotentialFieldObstacle>& obstacles) {
  // Lambda to compute min distance between a link obstacle and all environment obstacles
  auto minClearanceDistance = [&](const PotentialFieldObstacle& linkObstacle) -> double {
    double minDist = std::numeric_limits<double>::infinity();
    for (const auto& obst : obstacles) {
      Eigen::Vector3d normal;
      double dist = linkObstacle.computeMinimumDistanceTo(obst, normal);
      if (dist < minDist) {
        minDist = dist;
      }
    }
    return minDist;
  };
  auto robotLinkObstacles = this->buildObstaclesFromTransforms(this->computeLinkTransforms(jointAngles));
  std::vector<double> minClearances;
  minClearances.reserve(robotLinkObstacles.size());
  for (size_t i = 0; i < robotLinkObstacles.size(); ++i) {
    minClearances.push_back(minClearanceDistance(robotLinkObstacles[i]));
  }
  return minClearances;
}
