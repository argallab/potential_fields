#include "pfield/pf_kinematics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>

PFKinematics::PFKinematics(
  const std::string& urdfFileName, const std::vector<std::string>& jointNames,
  const double influenceZoneScale, const double repulsiveGain) {
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
  this->collisionCatalog = this->buildCollisionCatalog(this->robotModel, influenceZoneScale, repulsiveGain);
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

std::vector<CollisionCatalogEntry> PFKinematics::buildCollisionCatalog(
  urdf::Model& model,
  const double influenceZoneScale,
  const double repulsiveGain) {
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
        e.id, influenceZoneScale, repulsiveGain, *e.col, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()
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
  const std::string& frameID, const double influenceZoneScale, const double repulsiveGain,
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
    obstacleGeom, influenceZoneScale, repulsiveGain
  );
  // If mesh, set mesh resource and scale
  if (isMesh) { obstacle.setMeshProperties(meshResource, Eigen::Vector3d(length, width, height)); }
  return obstacle;
}

std::vector<PotentialFieldObstacle> PFKinematics::updateObstaclesFromJointAngles(const std::vector<double>& jointAngles) {
  return this->buildObstaclesFromTransforms(this->computeLinkTransforms(jointAngles));
}
