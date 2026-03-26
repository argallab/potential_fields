// Copyright 2025 Sharwin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pfield/pf_kinematics.hpp"
#include "pfield/pfield_common.hpp"
#include "pfield/mesh_collision.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Geometry>

namespace pfield {

  PFKinematics::PFKinematics(const std::string& urdfFileName) {
    if (urdfFileName.empty()) {
      throw std::invalid_argument("PFKinematics Constructor: urdfFileName is empty; expected a path to a URDF file");
    }
    try {
      pinocchio::urdf::buildModel(urdfFileName, this->model);
    }
    catch (const std::exception& e) {
      throw std::runtime_error(
        std::string("PFKinematics Constructor: Failed to load URDF model from '") + urdfFileName + "': " + e.what()
      );
    }
    this->data = pinocchio::Data(this->model);
    this->robotModel = urdf::parseURDFFile(urdfFileName);
    if (!this->robotModel) {
      throw std::runtime_error("PFKinematics Constructor: Failed to parse URDF file '" + urdfFileName + "'");
    }
    // Build collision catalog (this also initializes caches internally)
    this->collisionCatalog = this->buildCollisionCatalog(*this->robotModel);
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
    this->numJoints = jointNames.size();
    this->numLinks = linkNames.size();
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

  std::vector<CollisionCatalogEntry> PFKinematics::buildCollisionCatalog(const urdf::ModelInterface& model) {
    std::vector<CollisionCatalogEntry> catalog;
    this->obstacleGeometryTemplates.clear();

    // Extract joint names from the model (only 1-DoF joints)
    std::vector<std::string> jointNames;
    jointNames.reserve(model.joints_.size());
    for (const auto& [jointName, joint] : model.joints_) {
      if (!joint) { continue; }
      if (joint->type == urdf::Joint::UNKNOWN || joint->type == urdf::Joint::FIXED) { continue; }
      if (!this->model.existJointName(jointName)) { continue; }
      const pinocchio::JointIndex jid = this->model.getJointId(jointName);
      if (jid >= this->model.joints.size()) { continue; }
      if (this->model.joints[jid].nq() != 1) { continue; }
      jointNames.push_back(jointName);
    }

    // Build collision catalog
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
    // Initialize caches with extracted joint names and collision link names
    this->initializeCaches(jointNames, this->collisionLinkNames);
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
      // Robot link mesh geometry: approximate with an ellipsoid for smooth, continuous SDF.
      // Semi-axes are derived from the mesh AABB half-extents scaled by the URDF mesh scale.
      const double sx = m->scale.x > 0.0 ? m->scale.x : 1.0;
      const double sy = m->scale.y > 0.0 ? m->scale.y : 1.0;
      const double sz = m->scale.z > 0.0 ? m->scale.z : 1.0;

      // Attempt to load the mesh and derive semi-axes from its actual AABB
      double semi_x = 0.0, semi_y = 0.0, semi_z = 0.0;
      bool aabbFromMesh = false;
      try {
        auto meshData = loadMesh(m->filename);
        if (meshData) {
          const Eigen::Vector3d halfExtents = 0.5 * (meshData->aabbMax - meshData->aabbMin).cwiseAbs();
          semi_x = halfExtents.x() * sx;
          semi_y = halfExtents.y() * sy;
          semi_z = halfExtents.z() * sz;
          aabbFromMesh = true;
        }
      }
      catch (const std::exception&) {
        // Mesh load failed; fall through to scale-based fallback
      }

      if (!aabbFromMesh) {
        // Fallback: treat URDF scale as full-extent diameter → semi-axes = scale / 2
        semi_x = sx / 2.0;
        semi_y = sy / 2.0;
        semi_z = sz / 2.0;
      }

      // Ensure no degenerate zero semi-axis (clamp to a small positive value)
      const double minSemi = 1e-3;
      semi_x = std::max(semi_x, minSemi);
      semi_y = std::max(semi_y, minSemi);
      semi_z = std::max(semi_z, minSemi);

      ObstacleGeometry ellipsoidGeom(semi_x, semi_y, semi_z, /*ellipsoid_tag=*/true);
      return PotentialFieldObstacle(
        frameID, position, orientation, ObstacleType::ELLIPSOID, ObstacleGroup::ROBOT,
        ellipsoidGeom
      );
    }
    else {
      throw std::runtime_error("obstacleFromCollisionObject: Unsupported geometry type in collision object");
    }
    ObstacleGeometry obstacleGeom(radius, length, width, height);
    return PotentialFieldObstacle(
      frameID, position, orientation, obstacleType, ObstacleGroup::ROBOT, obstacleGeom
    );
  }

  std::vector<PotentialFieldObstacle> PFKinematics::updateObstaclesFromJointAngles(const std::vector<double>& jointAngles) {
    return this->buildObstaclesFromTransforms(this->computeLinkTransforms(jointAngles));
  }

  double PFKinematics::estimateRobotExtentRadius() {
    // Require a valid URDF model
    if (!this->robotModel || this->robotModel->links_.empty()) { return 0.0; }

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

  Eigen::MatrixXd PFKinematics::getJacobianAtPoint(const std::string& linkName, const Eigen::Vector3d& pointInWorldFrame,
    const std::vector<double>& jointAngles) {
    // 1. Update model state (ensure forward kinematics was called recently or call it here)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
    for (size_t i = 0; i < jointAngles.size(); ++i) {
      // Map your jointAngles to Pinocchio q vector (handling indices as you do in computeLinkTransforms)
      int qi = this->jointQIndices[i];
      if (qi >= 0) q(qi) = jointAngles[i];
    }

    // 2. Get the Frame ID
    if (!model.existFrame(linkName)) {
      throw std::runtime_error("Link not found: " + linkName);
    }
    pinocchio::FrameIndex frameId = model.getFrameId(linkName);

    // 3. Compute the Jacobian at the Frame Origin (Local World Aligned)
    // J_frame is 6xN (3 linear, 3 angular)
    pinocchio::Data::Matrix6x J_frame(6, model.nv);
    J_frame.setZero();

    // computes J in the world frame centered at the joint
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // 4. Transport Jacobian to the Witness Point
    // The Jacobian we got tells us how the Frame Origin moves.
    // We need to know how 'pointInWorldFrame' moves.
    // v_point = v_origin + w x r  (where r is vector from origin to point)

    Eigen::Vector3d linkOrigin = data.oMf[frameId].translation();
    Eigen::Vector3d r = pointInWorldFrame - linkOrigin; // Vector from link origin to collision point

    // Linear velocity at point P is: J_linear - skew(r) * J_angular
    Eigen::MatrixXd J_point = J_frame.topRows(3) -
      pinocchio::skew(r) * J_frame.bottomRows(3);

    return J_point; // Returns a 3xN matrix
  }

  Eigen::MatrixXd PFKinematics::getSpatialJacobianAtPoint(const std::string& linkName, const Eigen::Vector3d& pointInWorldFrame,
    const std::vector<double>& jointAngles) {
    // 1. Update model state (ensure forward kinematics was called recently or call it here)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
    for (size_t i = 0; i < jointAngles.size(); ++i) {
      int qi = this->jointQIndices[i];
      if (qi >= 0) q(qi) = jointAngles[i];
    }

    // 2. Get the Frame ID
    if (!model.existFrame(linkName)) {
      throw std::runtime_error("Link not found: " + linkName);
    }
    pinocchio::FrameIndex frameId = model.getFrameId(linkName);

    // 3. Compute the Jacobian at the Frame Origin (Local World Aligned)
    // J_frame is 6xN (3 linear, 3 angular)
    pinocchio::Data::Matrix6x J_frame(6, model.nv);
    J_frame.setZero();

    // computes J in the world frame centered at the joint
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

    // 4. Transport Jacobian to the Witness Point
    Eigen::Vector3d p_origin = data.oMf[frameId].translation();
    Eigen::Vector3d r = pointInWorldFrame - p_origin; // Vector from link origin to point

    // Linear velocity at point P is: J_linear - skew(r) * J_angular
    Eigen::MatrixXd J_linear = J_frame.topRows(3) -
      pinocchio::skew(r) * J_frame.bottomRows(3);
    Eigen::MatrixXd J_angular = J_frame.bottomRows(3);

    // Combine into 6xN spatial Jacobian
    Eigen::MatrixXd J_spatial(6, model.nv);
    J_spatial.topRows(3) = J_linear;
    J_spatial.bottomRows(3) = J_angular;

    return J_spatial;
  }

  SpatialVector PFKinematics::computeEndEffectorPose(const std::vector<double>& jointAngles, const std::string& eeLinkName) {
    // Convert joint angles to Eigen::VectorXd
    Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
    for (size_t i = 0; i < jointAngles.size() && i < this->jointQIndices.size(); ++i) {
      int qi = this->jointQIndices[i];
      if (qi >= 0 && qi < this->model.nq) {
        q[qi] = jointAngles[i];
      }
    }

    // Compute forward kinematics for all frames
    pinocchio::framesForwardKinematics(this->model, this->data, q);

    // Get the frame ID for the end-effector link
    if (!this->model.existFrame(eeLinkName)) {
      throw std::runtime_error("PFKinematics::computeEndEffectorPose: End-effector link '" + eeLinkName + "' not found in model");
    }
    pinocchio::FrameIndex eeFrameId = this->model.getFrameId(eeLinkName);

    // Extract the end-effector transform
    const pinocchio::SE3& eeTransform = this->data.oMf[eeFrameId];

    // Convert to SpatialVector (position + quaternion)
    Eigen::Vector3d position = eeTransform.translation();
    Eigen::Quaterniond orientation(eeTransform.rotation());

    return SpatialVector(position, orientation);
  }

  std::vector<double> PFKinematics::computeInverseKinematics(
    const SpatialVector& targetPose,
    const std::vector<double>& seedJointAngles,
    const std::string& eeLinkName,
    int maxIterations,
    double tolerance) {

    if (!this->model.existFrame(eeLinkName)) {
      // TODO(Sharwin24): Log error properly
      std::cerr << "[PFKinematics ERROR]: computeInverseKinematics: End-effector link '"
        << eeLinkName << "' not found in model" << std::endl;
      return {};
    }
    pinocchio::FrameIndex frameId = this->model.getFrameId(eeLinkName);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
    // Initialize q from seed
    for (size_t i = 0; i < seedJointAngles.size() && i < this->jointQIndices.size(); ++i) {
      int qi = this->jointQIndices[i];
      if (qi >= 0 && qi < this->model.nq) {
        q[qi] = seedJointAngles[i];
      }
    }

    const double lambda = 0.01; // Damping factor

    for (int i = 0; i < maxIterations; ++i) {
      pinocchio::framesForwardKinematics(this->model, this->data, q);
      const pinocchio::SE3& currentTransform = this->data.oMf[frameId];

      Eigen::Vector3d currentPos = currentTransform.translation();
      Eigen::Quaterniond currentRot(currentTransform.rotation());

      Eigen::Vector3d errPos = targetPose.getPosition() - currentPos;
      Eigen::Quaterniond errRotQuat = targetPose.getOrientation() * currentRot.conjugate();
      // Ensure shortest path
      if (errRotQuat.w() < 0) {
        errRotQuat.coeffs() *= -1;
      }
      Eigen::AngleAxisd errRotAA(errRotQuat);
      Eigen::Vector3d errRot = errRotAA.angle() * errRotAA.axis();

      Eigen::VectorXd error(6);
      error << errPos, errRot;

      if (error.norm() < tolerance) {
        std::vector<double> result(this->numJoints, 0.0);
        for (size_t j = 0; j < this->numJoints; ++j) {
          int qi = this->jointQIndices[j];
          if (qi >= 0) result[j] = q[qi];
        }
        return result;
      }

      pinocchio::Data::Matrix6x J(6, this->model.nv);
      J.setZero();
      pinocchio::computeJointJacobians(this->model, this->data, q);
      pinocchio::getFrameJacobian(this->model, this->data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J);

      // Damped Least Squares: dq = J^T * (J * J^T + lambda^2 * I)^-1 * error
      Eigen::MatrixXd JJT = J * J.transpose();
      JJT.diagonal().array() += lambda * lambda;
      Eigen::VectorXd dq = J.transpose() * JJT.ldlt().solve(error);

      q += dq;
      pinocchio::normalize(this->model, q);
    }

    return {}; // Failed to converge
  }

  double PFKinematics::getEndEffectorMass(const std::string& eeLinkName, const double fallBackMass) const {
    if (!this->robotModel) { return fallBackMass; }
    if (this->robotModel->links_.count(eeLinkName) > 0) {
      const auto& eeLink = this->robotModel->links_.at(eeLinkName);
      if (eeLink && eeLink->inertial) {
        // End Effector Mass has to be at least 1 gram, smaller than that we consider it negligible and use fallback
        return eeLink->inertial->mass > 1e-3 ? eeLink->inertial->mass : fallBackMass;
      }
    }
    return fallBackMass;
  }

  Eigen::VectorXd PFKinematics::jointValuesToVector(const std::vector<double>& jointValues) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(this->model.nq);
    for (size_t i = 0; i < jointValues.size() && i < this->jointQIndices.size(); ++i) {
      int qi = this->jointQIndices[i];
      if (qi >= 0 && qi < this->model.nq) {
        q[qi] = jointValues[i];
      }
    }
    return q;
  }

  Eigen::MatrixXd PFKinematics::getMassMatrix(const std::vector<double>& jointAngles, const double lambda) {
    Eigen::VectorXd q = this->jointValuesToVector(jointAngles);

    // Compute the mass matrix using CRBA and store the transpose to fill the lower triangle
    pinocchio::crba(this->model, this->data, q);
    this->data.M.triangularView<Eigen::StrictlyLower>() = this->data.M.transpose();

    // Extract only the rows/columns corresponding to the active joints
    const size_t numJoints = jointAngles.size();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(numJoints, numJoints);
    for (size_t i = 0; i < numJoints; ++i) {
      for (size_t j = 0; j < numJoints; ++j) {
        M(i, j) = this->data.M(this->jointQIndices[i], this->jointQIndices[j]);
      }
    }

    // Regularize Mass Matrix
    // Near singularities, the mass matrix is not very usable and inverting it
    // creates large velocity spikes, so we can add a small damper to the diagonal
    M.diagonal().array() += lambda;

    return M;
  }

  Eigen::VectorXd PFKinematics::getCoriolisVector(
    const std::vector<double>& jointAngles, const std::vector<double>& jointVelocities) {
    Eigen::VectorXd q = this->jointValuesToVector(jointAngles);
    Eigen::VectorXd v = this->jointValuesToVector(jointVelocities);

    // Compute the Coriolis term: C(q, q_dot) * q_dot
    Eigen::VectorXd nle = pinocchio::nonLinearEffects(this->model, this->data, q, v);
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(this->model, this->data, q);
    Eigen::VectorXd coriolisFull = nle - g;

    // Extract only the entries corresponding to the active joints
    const size_t numJoints = jointAngles.size();
    Eigen::VectorXd C = Eigen::VectorXd::Zero(numJoints);
    for (size_t i = 0; i < numJoints; ++i) {
      C(i) = coriolisFull(this->jointQIndices[i]);
    }
    return C;
  }

  Eigen::VectorXd PFKinematics::getGravityVector(const std::vector<double>& jointAngles) {
    Eigen::VectorXd q = this->jointValuesToVector(jointAngles);
    Eigen::VectorXd gravityVector = pinocchio::computeGeneralizedGravity(this->model, this->data, q);

    // Extract only the entries corresponding to the active joints
    const size_t numJoints = jointAngles.size();
    Eigen::VectorXd G = Eigen::VectorXd::Zero(numJoints);
    for (size_t i = 0; i < numJoints; ++i) {
      G(i) = gravityVector(this->jointQIndices[i]);
    }
    return G;
  }

} // namespace pfield
