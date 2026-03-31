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

#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"
#include <algorithm>
#include <limits>
#include <fstream>
#include <iostream>

#include <coal/collision.h>
#include <coal/distance.h>

namespace pfield {

  std::pair<SpatialVector, TaskSpaceTwist> PotentialField::stepIntegration(
    const SpatialVector& currentPose,
    const TaskSpaceTwist& prevTwist,
    const double dt,
    const std::string& integrationMethod,
    std::vector<double>& jointAngles,
    const std::vector<double>& jointVelocities) {

    TaskSpaceTwist twist;

    if (integrationMethod == "whole_body") {
      // Use provided joint angles/velocities
      twist = this->evaluateWholeBodyTaskSpaceTwistAtConfiguration(
        jointAngles, jointVelocities, currentPose, dt
      );
      twist = this->applyMotionConstraints(twist, prevTwist, dt);
    }
    else {
      // Task Space
      TaskSpaceWrench wrench = this->evaluateWrenchAtPoseWithOpposingForceRemoval(currentPose);
      twist = this->applyMotionConstraints(this->wrenchToTwist(wrench), prevTwist, dt);
    }

    // Integrate
    Eigen::Vector3d nextPosition = this->integrateLinearVelocity(
      currentPose.getPosition(), twist.linearVelocity, dt);
    Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(
      currentPose.getOrientation(), twist.angularVelocity, dt);
    SpatialVector nextPose(nextPosition, nextOrientation);

    // Update Kinematics/Obstacles
    std::vector<double> seed = jointAngles.empty() ? std::vector<double>(this->getNumJoints(), 0.0) : jointAngles;
    std::vector<double> newJoints = this->computeInverseKinematics(nextPose, seed);

    if (!newJoints.empty()) {
      jointAngles = newJoints;
      this->updateObstaclesFromKinematics(jointAngles);
    }

    return {nextPose, twist};
  }

  void PotentialField::initializeKinematics(const std::string& urdfFilePath, const std::string& eeLinkName) {
    this->urdfFileName = urdfFilePath;
    this->eeLinkName = eeLinkName;
    this->pfKinematics = std::make_unique<PFKinematics>(this->urdfFileName);
    // Assign influence distance using maximum robot extent or the default, whichever is more generous
    // TODO(Sharwin24): Uncomment this once the estimateRobotExtenRadius function is fixed and verified to be accurate
    // const double maxExtent = this->pfKinematics->estimateRobotExtentRadius();
    // this->influenceDistance = std::max(this->influenceDistance, maxExtent);
  }

  void PotentialField::updateObstaclesFromKinematics(const std::vector<double>& jointAngles) {
    if (!this->pfKinematics) { return; }
    std::vector<PotentialFieldObstacle> newObstacles = this->pfKinematics->updateObstaclesFromJointAngles(jointAngles);
    this->addObstacles(newObstacles);
  }

  void PotentialField::addObstacle(PotentialFieldObstacle obstacle) {
    const std::string frameID = obstacle.getFrameID();
    if (obstacle.getGroup() == ObstacleGroup::ROBOT) {
      auto itIndex = this->robotObstacleIndexMap.find(frameID);
      if (itIndex != this->robotObstacleIndexMap.end()) {
        // Update existing robot obstacle
        this->robotObstacles[itIndex->second] = obstacle;
      }
      else {
        // Append new robot obstacle
        this->robotObstacles.push_back(obstacle);
        this->robotObstacleIndexMap.emplace(frameID, this->robotObstacles.size() - 1);
      }
    }
    else {
      auto itIndex = this->envObstacleIndexMap.find(frameID);
      if (itIndex != this->envObstacleIndexMap.end()) {
        // Update existing environment obstacle
        this->envObstacles[itIndex->second] = obstacle;
      }
      else {
        // Append new environment obstacle
        this->envObstacles.push_back(obstacle);
        this->envObstacleIndexMap.emplace(frameID, this->envObstacles.size() - 1);
      }
    }
  }

  void PotentialField::addObstacles(const std::vector<PotentialFieldObstacle>& obstacles) {
    for (const auto& obst : obstacles) { this->addObstacle(obst); }
  }

  bool PotentialField::removeObstacle(const std::string& obstacleFrameID) {
    // Try environment obstacles first
    auto itEnv = this->envObstacleIndexMap.find(obstacleFrameID);
    if (itEnv != this->envObstacleIndexMap.end()) {
      size_t idx = itEnv->second;
      size_t last = this->envObstacles.size() - 1;
      if (idx != last) {
        std::swap(this->envObstacles[idx], this->envObstacles[last]);
        // update moved obstacle's index
        this->envObstacleIndexMap[this->envObstacles[idx].getFrameID()] = idx;
      }
      this->envObstacles.pop_back();
      this->envObstacleIndexMap.erase(itEnv);
      return true;
    }

    // Check robot obstacles next
    auto itRobot = this->robotObstacleIndexMap.find(obstacleFrameID);
    if (itRobot != this->robotObstacleIndexMap.end()) {
      size_t idx = itRobot->second;
      size_t last = this->robotObstacles.size() - 1;
      if (idx != last) {
        std::swap(this->robotObstacles[idx], this->robotObstacles[last]);
        // update moved obstacle's index
        this->robotObstacleIndexMap[this->robotObstacles[idx].getFrameID()] = idx;
      }
      this->robotObstacles.pop_back();
      this->robotObstacleIndexMap.erase(itRobot);
      return true;
    }

    // Not found in either map
    return false;
  }

  void PotentialField::clearObstacles() { this->envObstacles.clear(); this->envObstacleIndexMap.clear(); }

  PotentialFieldObstacle PotentialField::getObstacleByID(const std::string& obstacleFrameID) const {
    auto itIndex = this->envObstacleIndexMap.find(obstacleFrameID);
    if (itIndex == this->envObstacleIndexMap.end()) {
      throw std::invalid_argument("Obstacle with the given ID does not exist.");
    }
    return this->envObstacles[itIndex->second];
  }

  std::vector<PotentialFieldObstacle> PotentialField::getObstaclesByGroup(ObstacleGroup group) const {
    switch (group) {
    case ObstacleGroup::STATIC:
    case ObstacleGroup::DYNAMIC:
      return this->envObstacles;
    case ObstacleGroup::ROBOT:
      return this->robotObstacles;
    default:
      // For now, return empty list instead of throwing an exception
      return {};
    }
  }

  bool PotentialField::isPointInsideObstacle(Eigen::Vector3d worldPoint) const {
    for (const auto& obst : this->envObstacles) {
      if (obst.withinObstacle(worldPoint)) { return true; }
    }
    return false;
  }

  bool PotentialField::isPointWithinInfluenceZone(Eigen::Vector3d worldPoint) const {
    for (const auto& obst : this->envObstacles) {
      if (obst.withinInfluenceZone(worldPoint, this->influenceDistance)) { return true; }
    }
    return false;
  }

  PFLimits PotentialField::computeFieldBounds(const SpatialVector& queryPose, double bufferArea) const {
    PFLimits limits;
    // Determine the limits of the potential field based on obstacle positions, goal position, and query pose
    // Use envObstacles directly since we are inside the class
    Eigen::Vector3d goalPos = this->goalPose.getPosition();
    Eigen::Vector3d queryPos = queryPose.getPosition();

    if (this->envObstacles.empty()) {
      // If no obstacles, set limits around the goal position and query pose
      limits.minX = std::min(goalPos.x(), queryPos.x());
      limits.maxX = std::max(goalPos.x(), queryPos.x());
      limits.minY = std::min(goalPos.y(), queryPos.y());
      limits.maxY = std::max(goalPos.y(), queryPos.y());
      limits.minZ = std::min(goalPos.z(), queryPos.z());
      limits.maxZ = std::max(goalPos.z(), queryPos.z());
    }
    else {
      // Initialize limits with extreme values
      limits.minX = std::numeric_limits<double>::max();
      limits.maxX = std::numeric_limits<double>::lowest();
      limits.minY = std::numeric_limits<double>::max();
      limits.maxY = std::numeric_limits<double>::lowest();
      limits.minZ = std::numeric_limits<double>::max();
      limits.maxZ = std::numeric_limits<double>::lowest();

      // Iterate through obstacles to find overall min/max
      for (const auto& obst : this->envObstacles) {
        Eigen::Vector3d pos = obst.getPosition();
        // Use the bounding sphere radius to account for obstacle size and rotation
        double radius = obst.halfDimensions().norm();

        if (pos.x() - radius < limits.minX) limits.minX = pos.x() - radius;
        if (pos.x() + radius > limits.maxX) limits.maxX = pos.x() + radius;
        if (pos.y() - radius < limits.minY) limits.minY = pos.y() - radius;
        if (pos.y() + radius > limits.maxY) limits.maxY = pos.y() + radius;
        if (pos.z() - radius < limits.minZ) limits.minZ = pos.z() - radius;
        if (pos.z() + radius > limits.maxZ) limits.maxZ = pos.z() + radius;
      }

      // Expand limits to include the goal position if outside current bounds
      if (goalPos.x() < limits.minX) limits.minX = goalPos.x();
      if (goalPos.x() > limits.maxX) limits.maxX = goalPos.x();
      if (goalPos.y() < limits.minY) limits.minY = goalPos.y();
      if (goalPos.y() > limits.maxY) limits.maxY = goalPos.y();
      if (goalPos.z() < limits.minZ) limits.minZ = goalPos.z();
      if (goalPos.z() > limits.maxZ) limits.maxZ = goalPos.z();

      // Expand limits to include the query pose if outside current bounds
      if (queryPos.x() < limits.minX) limits.minX = queryPos.x();
      if (queryPos.x() > limits.maxX) limits.maxX = queryPos.x();
      if (queryPos.y() < limits.minY) limits.minY = queryPos.y();
      if (queryPos.y() > limits.maxY) limits.maxY = queryPos.y();
      if (queryPos.z() < limits.minZ) limits.minZ = queryPos.z();
      if (queryPos.z() > limits.maxZ) limits.maxZ = queryPos.z();
    }
    // Increase the limits by a buffer area for better visualization
    limits.minX -= bufferArea;
    limits.maxX += bufferArea;
    limits.minY -= bufferArea;
    limits.maxY += bufferArea;
    limits.minZ -= bufferArea;
    limits.maxZ += bufferArea;
    return limits;
  }

  Eigen::VectorXd PotentialField::evaluateWholeBodyJointTorquesAtConfiguration(
    const std::vector<double>& jointAngles, const std::vector<double>& prevJointVelocities, const SpatialVector& eePose,
    const double dt,
    std::vector<Eigen::Vector3d>* outLinkForces,
    std::vector<double>* outLinkClearances,
    Eigen::Vector3d* outAttractionForce) {
    // --- 1. Compute End-Effector Attraction Joint Torques ---
    Eigen::VectorXd attractionTorques = this->computeEndEffectorAttractionJointTorques(eePose, jointAngles);

    if (outAttractionForce) {
      *outAttractionForce = this->computeAttractiveForceLinear(eePose);
    }

    // --- 2. Compute Whole-Body Obstacle Repulsion Joint Torques ---
    Eigen::VectorXd repulsionTorques = this->computeWholeBodyRepulsionJointTorques(
      jointAngles, outLinkForces, outLinkClearances
    );

    // --- 3. Combine Torques ---
    Eigen::VectorXd totalJointTorques = attractionTorques + repulsionTorques;

    return totalJointTorques;
  }

  Eigen::VectorXd PotentialField::convertJointTorquesToJointVelocities(
    const Eigen::VectorXd& jointTorques, const std::vector<double>& jointAngles,
    const std::vector<double>& prevJointVelocities, const double dt) const {
    // TODO(Sharwin24): Fix this method and figure out why it's unstable
    // https://github.com/argallab/potential_fields/issues/23
    const bool useRobotDynamicsEquation = false;
    if (!this->pfKinematics || !useRobotDynamicsEquation) {
      // If we don't have access to PFKinematics, use a simple proportional mapping
      return this->torqueToVelocityGain * jointTorques;
    }

    // Retrieve Mass Matrix, Coriolis, and Gravity
    Eigen::MatrixXd M = this->pfKinematics->getMassMatrix(jointAngles);
    // C contains the result of C * q_dot since it's simpler to compute
    Eigen::VectorXd C = this->pfKinematics->getCoriolisVector(jointAngles, prevJointVelocities);
    Eigen::VectorXd G = this->pfKinematics->getGravityVector(jointAngles);

    // Add joint damping to stabilize the motion (M*q_ddot = Tau - D*q_dot)
    // A simple constant damping prevents oscillation from the conservative potential field
    const double dampingGain = 20.0;
    Eigen::VectorXd prevJointVels = Eigen::Map<const Eigen::VectorXd>(prevJointVelocities.data(), prevJointVelocities.size());
    Eigen::VectorXd dampingTorque = dampingGain * prevJointVels;

    Eigen::VectorXd netTorques = jointTorques - C - G - dampingTorque;
    Eigen::VectorXd jointAccelerations;
    try {
      jointAccelerations = M.llt().solve(netTorques);
    }
    catch (const std::exception& e) {
      std::cerr << "Error solving for joint accelerations: " << e.what() << std::endl;
      jointAccelerations = netTorques; // Fallback to direct mapping
    }

    // --- Integrate joint accelerations to get joint velocities ---
    Eigen::VectorXd jointVelocities = prevJointVels;
    if (isPositiveFinite(dt)) {
      jointVelocities += jointAccelerations * dt;
    }
    else {
      return jointAccelerations; // If dt is invalid, fallback to direct mapping
    }

    // --- Apply Joint Velocity/Acceleration Limits ---
    if (this->pfKinematics) {
      const auto& model = this->pfKinematics->getModel();
      for (int i = 0; i < jointVelocities.size() && i < model.velocityLimit.size(); ++i) {
        double limit = model.velocityLimit[i];
        // Pinocchio sometimes sets limits to infinity or very large values if undefined
        if (limit > 1e-3 && limit < 1e10) {
          jointVelocities[i] = std::clamp(jointVelocities[i], -limit, limit);
        }
      }
    }

    return jointVelocities;
  }

  TaskSpaceTwist PotentialField::evaluateWholeBodyTaskSpaceTwistAtConfiguration(
    const std::vector<double>& jointAngles, const std::vector<double>& prevJointVelocities,
    const SpatialVector& eePose, const double dt) {
    if (!this->pfKinematics) {
      return TaskSpaceTwist();
    }

    // 1. Compute whole-body joint velocities
    Eigen::VectorXd jointTorques = this->evaluateWholeBodyJointTorquesAtConfiguration(
      jointAngles, prevJointVelocities, eePose, dt
    );
    Eigen::VectorXd jointVelocities = this->convertJointTorquesToJointVelocities(
      jointTorques, jointAngles, prevJointVelocities, dt
    );

    // 2. Get End-Effector Jacobian
    Eigen::MatrixXd J = this->pfKinematics->getSpatialJacobianAtPoint(
      this->eeLinkName, eePose.getPosition(), jointAngles
    );

    // 3. Map joint velocities to task-space twist: V = J * q_dot
    Eigen::VectorXd twistVec = J * jointVelocities;

    return TaskSpaceTwist(twistVec.head(3), twistVec.tail(3));
  }


  TaskSpaceWrench PotentialField::evaluateWrenchAtPose(const SpatialVector& queryPose) const {
    Eigen::Vector3d attractionForceVector = this->computeAttractiveForceLinear(queryPose);
    Eigen::Vector3d attractionMomentVector = this->computeAttractiveMoment(queryPose);
    Eigen::Vector3d repulsionForceVector = this->computeRepulsiveForceLinear(queryPose);
    Eigen::Vector3d forceVector = attractionForceVector + repulsionForceVector;
    return TaskSpaceWrench(forceVector, attractionMomentVector);
  }

  TaskSpaceTwist PotentialField::evaluateVelocityAtPose(const SpatialVector& queryPose) const {
    return this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose));
  }

  TaskSpaceTwist PotentialField::evaluateLimitedVelocityAtPose(
    const SpatialVector& queryPose, const TaskSpaceTwist& prevTwist, double dt) const {
    return this->applyMotionConstraints(this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose)), prevTwist, dt);
  }

  TaskSpaceTwist PotentialField::wrenchToTwist(const TaskSpaceWrench& wrench) const {
    // If a mass/inertia model is available, it can be used here to convert force/torque to velocity
    const double linearGain = 1.0;  // (m/s) per N
    const double angularGain = 1.0; // (rad/s) per Nm
    return TaskSpaceTwist(wrench.force * linearGain, wrench.torque * angularGain);
  }

  TaskSpaceTwist PotentialField::applyMotionConstraints(
    const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double dt) const {
    // Soft-saturate velocities by norm
    Eigen::Vector3d limitedLinear = softSaturateNorm(twist.linearVelocity, this->maxLinearVelocity, this->softSatBeta);
    Eigen::Vector3d limitedAngular = softSaturateNorm(twist.angularVelocity, this->maxAngularVelocity, this->softSatBeta);

    // If dt is valid, apply rate limits (acceleration limits)
    if (isPositiveFinite(dt)) {
      const double dVMaxLinear = this->maxLinearAcceleration * dt; // m/s^2 * s = m/s
      const double dVMaxAngular = this->maxAngularAcceleration * dt; // rad/s^2 * s = rad/s
      limitedLinear = rateLimitStep(prevTwist.linearVelocity, limitedLinear, dVMaxLinear);
      limitedAngular = rateLimitStep(prevTwist.angularVelocity, limitedAngular, dVMaxAngular);

      // Re-apply soft-saturation to prevent velocity limits
      limitedLinear = softSaturateNorm(limitedLinear, this->maxLinearVelocity, this->softSatBeta);
      limitedAngular = softSaturateNorm(limitedAngular, this->maxAngularVelocity, this->softSatBeta);
    }
    return TaskSpaceTwist(limitedLinear, limitedAngular);
  }

  Eigen::Vector3d PotentialField::removeOpposingForce(const Eigen::Vector3d& attractionForce,
    const Eigen::Vector3d& repulsiveForce) const {
    // Default to returning the original repulsive force unchanged
    Eigen::Vector3d resultantForce = repulsiveForce;
    // Use the direction of the attractive force to determine opposing (parallel) components
    const double attractiveNorm = attractionForce.norm();
    if (attractiveNorm > NEAR_ZERO_THRESHOLD) {
      const Eigen::Vector3d u_att = attractionForce / attractiveNorm;
      const double dot = repulsiveForce.dot(u_att);
      // If repulsion has a component opposite to attraction (dot < 0),
      // subtract the opposing component along u_att
      if (dot < 0.0) {
        resultantForce = repulsiveForce - (dot * u_att);
      }
    }
    return resultantForce;
  }

  TaskSpaceWrench PotentialField::evaluateWrenchAtPoseWithOpposingForceRemoval(const SpatialVector& queryPose) const {
    // Compute attractive and repulsive forces
    Eigen::Vector3d attractiveForce = this->computeAttractiveForceLinear(queryPose);
    Eigen::Vector3d repulsiveForce = this->computeRepulsiveForceLinear(queryPose);
    // Remove only the component of the repulsive force that opposes attraction,
    // then combine with the attractive force to form the final resultant force
    Eigen::Vector3d filteredRepulsive = this->removeOpposingForce(attractiveForce, repulsiveForce);
    Eigen::Vector3d resultantForce = attractiveForce + filteredRepulsive;
    Eigen::Vector3d resultantTorque = this->computeAttractiveMoment(queryPose);
    return TaskSpaceWrench(resultantForce, resultantTorque);
  }

  SpatialVector PotentialField::interpolateNextPose(
    const SpatialVector& currentPose, const TaskSpaceTwist& prevTwist, const double dt) {
    // Compute the TaskSpaceTwist at the current pose
    TaskSpaceTwist vel = this->evaluateVelocityAtPose(currentPose);
    TaskSpaceTwist velLimited = this->applyMotionConstraints(vel, prevTwist, dt);
    // Integrate linear and angular velocities to get next pose
    Eigen::Vector3d nextPosition = this->integrateLinearVelocity(currentPose.getPosition(), velLimited.linearVelocity, dt);
    Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(
      currentPose.getOrientation(), velLimited.angularVelocity, dt
    );
    return SpatialVector(nextPosition, nextOrientation);
  }

  Eigen::Vector3d PotentialField::integrateLinearVelocity(const Eigen::Vector3d& currentPosition,
    const Eigen::Vector3d& linearVelocity, double deltaTime) {
    return currentPosition + (linearVelocity * deltaTime);
  }

  Eigen::Quaterniond PotentialField::integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation,
    const Eigen::Vector3d& angularVelocity,
    double deltaTime) {
    const double wnorm = angularVelocity.norm();
    if (wnorm < 1e-6 || deltaTime <= 0.0) return currentOrientation;
    // Exponential map: dq = exp( [w]^ * dt ) ≈ AngleAxis( |w|*dt, w/|w| )
    Eigen::Quaterniond dq(Eigen::AngleAxisd(wnorm * deltaTime, angularVelocity / wnorm));
    return (currentOrientation * dq).normalized();
  }

  TaskSpaceTwist PotentialField::constrainedTwistAtPose(
    const SpatialVector& pose,
    const TaskSpaceTwist& prevTwist, const double dt) {
    TaskSpaceWrench W = this->evaluateWrenchAtPoseWithOpposingForceRemoval(pose);
    return this->applyMotionConstraints(this->wrenchToTwist(W), prevTwist, dt);
  }

  std::pair<SpatialVector, TaskSpaceTwist> PotentialField::rungeKuttaStep(const SpatialVector& currentPose,
    const TaskSpaceTwist& prevTwist, const double dt) {
    // Stage 1
    TaskSpaceTwist k1 = this->constrainedTwistAtPose(currentPose, prevTwist, dt);
    // Pose at dt/2 using k1
    SpatialVector T2(
      this->integrateLinearVelocity(currentPose.getPosition(), k1.getLinearVelocity(), 0.5 * dt),
      this->integrateAngularVelocity(currentPose.getOrientation(), k1.getAngularVelocity(), 0.5 * dt)
    );
    // Stage 2 (rate-limit vs k1 over dt/2)
    TaskSpaceTwist k2 = this->constrainedTwistAtPose(T2, k1, 0.5 * dt);
    // Pose at dt/2 using k2
    SpatialVector T3(
      this->integrateLinearVelocity(currentPose.getPosition(), k2.getLinearVelocity(), 0.5 * dt),
      this->integrateAngularVelocity(currentPose.getOrientation(), k2.getAngularVelocity(), 0.5 * dt)
    );
    // Stage 3 (rate-limit vs k2 over dt/2)
    TaskSpaceTwist k3 = this->constrainedTwistAtPose(T3, k2, 0.5 * dt);
    // Pose at dt using k3
    SpatialVector T4(
      this->integrateLinearVelocity(currentPose.getPosition(), k3.getLinearVelocity(), dt),
      this->integrateAngularVelocity(currentPose.getOrientation(), k3.getAngularVelocity(), dt)
    );
    // Stage 4 (rate-limit vs k3 over full dt)
    TaskSpaceTwist k4 = this->constrainedTwistAtPose(T4, k3, dt);

    // RK4 weighted average twist
    Eigen::Vector3d v_bar = (k1.getLinearVelocity()
      + 2.0 * k2.getLinearVelocity()
      + 2.0 * k3.getLinearVelocity()
      + k4.getLinearVelocity()) / 6.0;
    Eigen::Vector3d w_bar = (k1.getAngularVelocity()
      + 2.0 * k2.getAngularVelocity()
      + 2.0 * k3.getAngularVelocity()
      + k4.getAngularVelocity()) / 6.0;

    // Final soft-saturation to ensure we still respect velocity caps
    TaskSpaceTwist rk4WeightedTwist(v_bar, w_bar);
    rk4WeightedTwist = this->applyMotionConstraints(rk4WeightedTwist, prevTwist, dt);

    // Advance once using the averaged twist
    SpatialVector nextPose(
      this->integrateLinearVelocity(currentPose.getPosition(), rk4WeightedTwist.getLinearVelocity(), dt),
      this->integrateAngularVelocity(currentPose.getOrientation(), rk4WeightedTwist.getAngularVelocity(), dt)
    );

    return {nextPose, rk4WeightedTwist};
  }

  Eigen::Vector3d PotentialField::computeAttractiveForceLinear(const SpatialVector& queryPose) const {
    if (!this->goalSet) return Eigen::Vector3d::Zero();
    // Choset Attractive Potential: F = zeta * (q - q_goal)
    const Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
    const double magnitude = direction.norm();
    // TODO(Sharwin24): Investigate why Conical block has problems with oscillations
    return -this->attractiveGain * direction;
    // if (magnitude <= dStar) {
    //   // Quadratic Attraction Region
    //   return -this->attractiveGain * direction;
    // }
    // else {
    //   // Conical Attraction Region
    //   // The Choset attractive potential defines a threshold for switching to quadratic behavior from conical
    //   const double dStar = this->dynamicQuadraticThresholdEnabled ? this->computeDynamicQuadraticThreshold(queryPose)
    //   : this->dStarThreshold;
    //   return  (dStar * (-this->attractiveGain * direction)) / magnitude;
    // }
  }

  double PotentialField::minObstacleClearanceAt(const Eigen::Vector3d& point) const {
    if (this->envObstacles.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double minClearance = std::numeric_limits<double>::infinity();
    for (const auto& obst : this->envObstacles) {
      double signedDistance = 0.0; // signed distance: >0 outside, <0 inside
      Eigen::Vector3d normal = Eigen::Vector3d::Zero();
      obst.computeSignedDistanceAndNormal(point, signedDistance, normal);
      double clearance = std::max(0.0, signedDistance); // treat inside as 0 clearance
      if (clearance < minClearance) minClearance = clearance;
    }
    return minClearance;
  }

  double PotentialField::minClearanceAlongSegment(const Eigen::Vector3d& from, const Eigen::Vector3d& to, int samples) const {
    if (this->envObstacles.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    samples = std::max(2, samples);
    double minClearance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
      double u = static_cast<double>(i) / static_cast<double>(samples - 1);
      Eigen::Vector3d p = from + u * (to - from);
      double clearance = this->minObstacleClearanceAt(p);
      if (clearance < minClearance) minClearance = clearance;
    }
    return minClearance;
  }

  double PotentialField::computeDynamicQuadraticThreshold(const SpatialVector& queryPose) const {
    // Baseline from existing parameter
    double baseline = this->dStarThreshold;
    if (!this->isUsingDynamicQuadraticThreshold() || !this->goalSet) { return baseline; }

    // Clearance near goal and along straight-line path to goal
    const Eigen::Vector3d goalPosition = this->goalPose.getPosition();
    const Eigen::Vector3d queryPosition = queryPose.getPosition();
    const double clearanceAtGoal = this->minObstacleClearanceAt(goalPosition);
    const double pathClearance = this->minClearanceAlongSegment(queryPosition, goalPosition, 7);

    // Use the more conservative clearance estimate
    double baselineClearance = std::min(clearanceAtGoal, pathClearance);

    // Stopping distance based on limits (ensure smooth approach)
    double stoppingDistance = 0.0;
    if (this->maxLinearAcceleration > NEAR_ZERO_THRESHOLD) {
      stoppingDistance = 0.5 * (this->maxLinearVelocity * this->maxLinearVelocity) / this->maxLinearAcceleration;
    }

    // If we have clearance data, increase d* in clutter (small clearance) to start quadratic earlier.
    // If free space (large clearance), keep near baseline.
    double dStar = baseline;
    if (std::isfinite(baselineClearance)) {
      const double Q = std::max(this->influenceDistance, 0.0);
      const double clutterScale = std::max(0.0, Q - baselineClearance); // larger when tighter space
      // Blend baseline, clutter and stopping distance
      dStar = baseline + 0.5 * clutterScale + 0.5 * stoppingDistance;
    }
    else {
      // No obstacles: just add a portion of stopping distance
      dStar = baseline + 0.5 * stoppingDistance;
    }
    // Clamp to reasonable bounds
    const double minimumDStarValue = 0.02;
    const double dStarMin = std::max(5.0 * this->translationalTolerance, minimumDStarValue);
    const double dStarMax = std::max(this->influenceDistance, baseline);
    return std::clamp(dStar, dStarMin, dStarMax);
  }

  Eigen::Vector3d PotentialField::computeAttractiveMoment(const SpatialVector& queryPose) const {
    if (!this->goalSet) return Eigen::Vector3d::Zero();
    // Quaternion error that rotates current -> goal (shortest path)
    Eigen::Quaterniond quatError = (
      this->goalPose.getOrientation() * queryPose.getOrientation().conjugate()
      ).normalized();

    // Enforce shortest-path and continuity: q and -q represent same rotation; choose w >= 0
    if (quatError.w() < 0.0) {
      quatError.coeffs() *= -1.0; // flip sign of (x,y,z,w)
    }

    // Extract axis-angle from quatError in a numerically stable way
    const double vnorm = quatError.vec().norm();
    // const double angle = 2.0 * std::atan2(vnorm, std::abs(quatError.w())); // in [0, pi]

    // Guard against divide-by-zero when angle ~ 0
    if (vnorm < NEAR_ZERO_THRESHOLD) { return Eigen::Vector3d::Zero(); }
    // Use bounded sine-form error for smoother large-angle behavior:
    // 2*sin(angle/2) * axis == 2 * quatError.vec() (since quatError is normalized and w >= 0)
    // Matches k*angle*axis for small angles and reduces aggressiveness near pi
    const Eigen::Vector3d boundedError = 2.0 * quatError.vec();
    return this->rotationalAttractiveGain * boundedError;
  }

  Eigen::Vector3d PotentialField::computeRepulsiveForceLinear(const SpatialVector& queryPose) const {
    Eigen::Vector3d F = Eigen::Vector3d::Zero();
    for (const auto& obst : this->envObstacles) {
      // Use true surface signed distance and outward normal for direction and magnitude
      double signedDistance = 0.0; // signed distance: > 0 outside, < 0 inside
      Eigen::Vector3d normalToObstSurface = Eigen::Vector3d::Zero(); // outward normal at closest surface point (world frame)
      obst.computeSignedDistanceAndNormal(queryPose.getPosition(), signedDistance, normalToObstSurface);
      // Ensure a valid normal; if degenerate, skip this obstacle
      const double normalMagnitude = normalToObstSurface.norm();
      if (normalMagnitude < NEAR_ZERO_THRESHOLD) continue;
      normalToObstSurface /= normalMagnitude;
      // Absolute influence semantics already enforced by withinInfluenceZone.
      // Define an effective distance d for the classic repulsive potential:
      //  - outside (sd >= 0): d = max(sd, eps)
      //  - inside (sd < 0): treat as very close to surface to generate strong outward push
      const double Q = std::max(this->influenceDistance, NEAR_ZERO_THRESHOLD);
      const double d = (signedDistance >= 0.0) ? std::max(signedDistance, NEAR_ZERO_THRESHOLD) : NEAR_ZERO_THRESHOLD;
      // Only contribute if within the influence distance
      if (d <= Q) {
        // Magnitude per Khatib potential [1]: eta * (1/d - 1/Q) * (1/d^2)
        const double inverseDistance = 1.0 / d;
        const double inverseDistanceSquared = inverseDistance * inverseDistance;
        const double inverseInfluenceDistance = 1.0 / Q;
        const double magnitude = this->repulsiveGain * (inverseDistance - inverseInfluenceDistance) * inverseDistanceSquared;
        if (magnitude > NEAR_ZERO_THRESHOLD) { F += normalToObstSurface * magnitude; }
      }
    }
    return F;
  }

  Eigen::VectorXd PotentialField::computeEndEffectorAttractionJointTorques(
    const SpatialVector& eePose, const std::vector<double>& jointAngles) const {
    // Compute task-space wrench for the end-effector (Attraction ONLY)
    // We do not include repulsion here because whole-body repulsion is computed separately
    // and we want to avoid double-counting repulsion on the end-effector link.
    Eigen::Vector3d attractionForceVector = this->computeAttractiveForceLinear(eePose);
    Eigen::Vector3d attractionMomentVector = this->computeAttractiveMoment(eePose);

    // Get end-effector Jacobian (6xN)
    Eigen::MatrixXd J_ee;
    double eeMass = 5.0; // Fallback to 5kg
    if (this->pfKinematics) {
      J_ee = this->pfKinematics->getSpatialJacobianAtPoint(
        this->eeLinkName, eePose.getPosition(), jointAngles
      );
      eeMass = this->pfKinematics->getEndEffectorMass(this->eeLinkName);
    }
    else if (this->ikSolver) {
      Eigen::Matrix<double, 6, Eigen::Dynamic> J_temp;
      this->ikSolver->computeJacobian(jointAngles, J_temp);
      J_ee = J_temp;
    }
    else {
      return Eigen::VectorXd::Zero(jointAngles.size());
    }

    // Enforce Maximum Force based on maximum linear acceleration allowed
    const double maxForce = this->maxLinearAcceleration * eeMass;
    const double forceNorm = attractionForceVector.norm();
    if (forceNorm > maxForce && forceNorm > NEAR_ZERO_THRESHOLD) {
      attractionForceVector = (attractionForceVector / forceNorm) * maxForce;
    }

    // Pack wrench into 6D vector
    Eigen::VectorXd taskForce(6);
    taskForce << attractionForceVector, attractionMomentVector;

    // Convert task-space force to joint torques: tau = J^T * F
    return J_ee.transpose() * taskForce;
  }

  Eigen::VectorXd PotentialField::computeWholeBodyRepulsionJointTorques(
    const std::vector<double>& jointAngles,
    std::vector<Eigen::Vector3d>* outLinkForces,
    std::vector<double>* outLinkClearances) const {
    // Get robot links and environment obstacles
    auto robotLinks = this->getObstaclesByGroup(ObstacleGroup::ROBOT);
    auto envObstacles = this->getObstaclesByGroup(ObstacleGroup::STATIC);

    // Initialize joint torques to zero (size will be set on first Jacobian computation)
    Eigen::VectorXd jointTorques;
    bool initializedJointTorqueSize = false;

    // Initialize outputs if provided
    if (outLinkForces) {
      outLinkForces->clear();
      outLinkForces->reserve(robotLinks.size());
    }
    if (outLinkClearances) {
      outLinkClearances->clear();
      outLinkClearances->reserve(robotLinks.size());
    }

    // Build a helper that returns world-frame control points for a link obstacle.
    // For each control point the associated "link surface radius" is the distance from that
    // point to the link surface, so SDF(controlPoint) - surfaceRadius = clearance to obstacle.
    //
    // Textbook (4.7.3): use a floating control point per link — the point on the link boundary
    // closest to any obstacle. We approximate this by sampling exact geometric points on the link
    // and picking the one with the smallest clearance per obstacle pair. This is smooth because
    // all sample points move continuously with the joint configuration.
    //
    // Control point sets per type:
    //   CAPSULE  — endcap centers (± halfShaft along capsule axis) + shaft midpoint (= link origin
    //              after centroid offset). surfaceRadius = capsule radius for all three.
    //   BOX      — 8 corners of the OBB. surfaceRadius = 0 (corners are on the surface).
    //   SPHERE   — link center. surfaceRadius = radius.
    //   CYLINDER — two endcap centers + shaft midpoint. surfaceRadius = radius.
    //   default  — link center. surfaceRadius = halfDimensions().norm().
    //
    // Each entry is {worldPoint, surfaceRadius}.
    using ControlPoint = std::pair<Eigen::Vector3d, double>;
    auto buildControlPoints = [](const PotentialFieldObstacle& link)
      -> std::vector<ControlPoint>
    {
      const ObstacleGeometry& g = link.getGeometry();
      const Eigen::Vector3d& pos = link.getPosition();
      const Eigen::Quaterniond& ori = link.getOrientation();

      switch (link.getType()) {
      case ObstacleType::CAPSULE: {
        // Capsule axis = column 2 of ellipsoid_axes (capsule Z), in world frame
        const Eigen::Vector3d capsuleAxisWorld =
          ori * (g.ellipsoid_axes * Eigen::Vector3d::UnitZ());
        const double halfShaft = g.height / 2.0;
        // OBB centroid in world frame
        const Eigen::Vector3d center = pos + ori * g.centroid_offset;
        return {
          {center + halfShaft * capsuleAxisWorld, g.radius},   // top endcap center
          {center - halfShaft * capsuleAxisWorld, g.radius},   // bottom endcap center
          {center,                                g.radius},   // shaft midpoint
        };
      }
      case ObstacleType::BOX: {
        // 8 corners of the OBB; corners are on the surface so surfaceRadius = 0
        const Eigen::Vector3d center = pos + ori * g.centroid_offset;
        const double hx = g.length / 2.0;
        const double hy = g.width  / 2.0;
        const double hz = g.height / 2.0;
        const Eigen::Matrix3d& axes = g.ellipsoid_axes;
        const Eigen::Vector3d ax = ori * axes.col(0);
        const Eigen::Vector3d ay = ori * axes.col(1);
        const Eigen::Vector3d az = ori * axes.col(2);
        std::vector<ControlPoint> pts;
        pts.reserve(8);
        for (int sx : {-1, 1})
          for (int sy : {-1, 1})
            for (int sz : {-1, 1})
              pts.push_back({center + sx*hx*ax + sy*hy*ay + sz*hz*az, 0.0});
        return pts;
      }
      case ObstacleType::SPHERE: {
        return {{pos, g.radius}};
      }
      case ObstacleType::CYLINDER: {
        const double halfH = g.height / 2.0;
        const Eigen::Vector3d axisWorld = ori * Eigen::Vector3d::UnitZ();
        return {
          {pos + halfH * axisWorld, g.radius},
          {pos - halfH * axisWorld, g.radius},
          {pos,                     g.radius},
        };
      }
      default: {
        const double r = link.halfDimensions().norm();
        return {{pos, r}};
      }
      }
    };

    // Extract link name from frameID ("linkName::collisionName" → "linkName")
    auto extractLinkName = [](const std::string& frameID) -> std::string {
      const size_t sep = frameID.find("::");
      return (sep != std::string::npos) ? frameID.substr(0, sep) : frameID;
    };

    for (const auto& link : robotLinks) {
      Eigen::Vector3d linkNetForce = Eigen::Vector3d::Zero();
      double linkMinDist = std::numeric_limits<double>::infinity();

      const std::string linkName = extractLinkName(link.getFrameID());
      const std::vector<ControlPoint> controlPoints = buildControlPoints(link);

      for (const auto& obs : envObstacles) {
        // Find the control point that gives the tightest (minimum) clearance to this obstacle.
        // This is the floating control point in the textbook sense.
        double bestDist = std::numeric_limits<double>::infinity();
        Eigen::Vector3d bestPoint = controlPoints[0].first;
        Eigen::Vector3d bestNormal = Eigen::Vector3d::UnitZ();

        for (const auto& [cpWorld, cpRadius] : controlPoints) {
          double sdf = 0.0;
          Eigen::Vector3d normal = Eigen::Vector3d::Zero();
          obs.computeSignedDistanceAndNormal(cpWorld, sdf, normal);
          const double d = sdf - cpRadius; // clearance from link surface to obstacle surface
          if (d < bestDist) {
            bestDist = d;
            bestPoint = cpWorld - normal * cpRadius; // point on link surface closest to obstacle
            bestNormal = normal;
          }
        }

        if (bestDist < linkMinDist) {
          linkMinDist = bestDist;
        }

        if (bestDist < this->influenceDistance) {
          Eigen::Vector3d direction = bestNormal;
          if (direction.norm() < NEAR_ZERO_THRESHOLD) {
            direction = Eigen::Vector3d::UnitZ();
          }

          const double d = std::max(bestDist, NEAR_ZERO_THRESHOLD);
          const double Q = this->influenceDistance;
          const double magnitude = this->repulsiveGain * (1.0 / d - 1.0 / Q) * (1.0 / (d * d));
          const Eigen::Vector3d repulsiveForce = direction * magnitude;

          linkNetForce += repulsiveForce;

          // Jacobian at the floating control point on the link surface
          Eigen::MatrixXd J_link = this->pfKinematics->getJacobianAtPoint(
            linkName, bestPoint, jointAngles
          );

          if (!initializedJointTorqueSize) {
            jointTorques = Eigen::VectorXd::Zero(J_link.cols());
            initializedJointTorqueSize = true;
          }

          jointTorques += J_link.transpose() * repulsiveForce;
        }
      }

      if (outLinkForces) outLinkForces->push_back(linkNetForce);
      if (outLinkClearances) outLinkClearances->push_back(linkMinDist);
    }

    // If no repulsive forces were computed, return zero vector
    if (!initializedJointTorqueSize) {
      const size_t numJoints = jointAngles.size();
      jointTorques = Eigen::VectorXd::Zero(numJoints);
    }

    return jointTorques;
  }

  std::vector<double> PotentialField::computeInverseKinematics(
    const SpatialVector& targetPose, const std::vector<double>& seedJointAngles) const {
    std::vector<double> jointAngles;
    bool success = false;
    if (this->ikSolver) {
      Eigen::Isometry3d targetPoseIsometry = Eigen::Isometry3d::Identity();
      targetPoseIsometry.translate(targetPose.getPosition());
      targetPoseIsometry.rotate(targetPose.getOrientation());
      std::string errorMsg;
      Eigen::Matrix<double, 6, Eigen::Dynamic> J;
      success = this->ikSolver->solve(targetPoseIsometry, seedJointAngles, jointAngles, J, errorMsg);
      if (!success) {
        // TODO(Sharwin24): Need to log or handle this error case properly. Propogate error message up
        // std::cerr << "[PFKinematics ERROR]: IKSolver failed solve()" << std::endl;
      }
    }
    if (this->pfKinematics && this->ikSolver && !success) {
      // Fallback to internal numerical IK if no external solver is provided
      // std::cerr << "[PFKinematics WARNING]: Falling back to pinocchio numerical IK solver." << std::endl;
      jointAngles = this->pfKinematics->computeInverseKinematics(targetPose, seedJointAngles, this->eeLinkName);
    }
    return jointAngles;
  }

  PlannedPath PotentialField::planPathFromTaskSpaceWrench(
    const SpatialVector& startPose,
    const std::vector<double>& startJointAngles,
    const double dt,
    const double goalTolerance,
    const size_t maxIters) {
    // Create path and initialize loop variables
    PlannedPath path;
    path.planningMethod = "task_space";
    path.goalTolerance = goalTolerance;
    path.rotationalTolerance = this->rotationalThreshold;
    const double stepDt = (dt > 0.0) ? dt : 0.1;
    SpatialVector current = startPose;
    std::vector<double> currentJointAngles = startJointAngles;
    TaskSpaceTwist prevTwist; // previous applied twist (starts zero)
    double timeStamp = 0.0;
    for (size_t iter = 0; iter < maxIters; ++iter) {
      // Calculate forces for recording
      Eigen::Vector3d attForce = this->computeAttractiveForceLinear(current);
      Eigen::Vector3d repForce = this->computeRepulsiveForceLinear(current);

      // Perform RK4 integration step to get next pose and the applied twist
      // including removal of opposing repulsive force components and enforcement of motion constraints
      auto [nextPoseRK4, appliedTwist] = this->rungeKuttaStep(current, prevTwist, stepDt);
      currentJointAngles = this->computeInverseKinematics(nextPoseRK4, currentJointAngles);

      // Record current state
      path.recordPathPoint(
        timeStamp,
        current,
        appliedTwist,
        currentJointAngles,
        {}, // joint velocities
        {}, // joint torques
        {}, // Link Clearances
        attForce,
        {repForce} // repulsive forces (using EE repulsion)
      );

      // Check goal after recording
      const double translationalError = (current.getPosition() - this->goalPose.getPosition()).norm();
      const double rotationalError = current.angularDistance(this->goalPose);
      if (translationalError <= goalTolerance && rotationalError <= this->rotationalThreshold) {
        path.success = true;
        break;
      }

      // Update obstacles from new JointAngles
      this->updateObstaclesFromKinematics(currentJointAngles);

      // Update loop variables
      current = nextPoseRK4;
      // Estimate velocity at the end of the interval for the next step
      // v_bar = (v_prev + v_next) / 2  ->  v_next = 2 * v_bar - v_prev
      Eigen::Vector3d linVelNext = 2.0 * appliedTwist.getLinearVelocity() - prevTwist.getLinearVelocity();
      Eigen::Vector3d angVelNext = 2.0 * appliedTwist.getAngularVelocity() - prevTwist.getAngularVelocity();
      prevTwist = TaskSpaceTwist(linVelNext, angVelNext);
      timeStamp += stepDt;
    }

    if (!path.success) {
      path.failureReason = "Max iterations reached without converging to goal.";
    }

    path.dt = stepDt;
    path.numPoints = static_cast<unsigned int>(path.poses.size());
    if (!path.timeStamps.empty()) {
      path.duration = path.timeStamps.back();
    }
    else {
      path.duration = 0.0;
    }
    return path;
  }

  PlannedPath PotentialField::planPathFromWholeBodyJointVelocities(
    const std::vector<double>& startJointAngles,
    const double dt,
    const double goalTolerance,
    const size_t maxIters) {
    // Create path and initialize loop variables
    PlannedPath path;
    path.planningMethod = "whole_body_velocity";
    path.goalTolerance = goalTolerance;
    path.rotationalTolerance = this->rotationalThreshold;
    const double stepDt = (dt > 0.0) ? dt : 0.1;
    double timeStamp = 0.0;
    // Set up stagnation and position tolerance counters/limits
    size_t stagnationCounter = 0;
    const double stagnationLimitSeconds = 0.75; // Stop if stuck this many seconds
    const size_t stagnationLimitIterations = static_cast<size_t>(std::ceil(stagnationLimitSeconds / stepDt));
    const size_t stagnationLimit = std::max(static_cast<size_t>(1), stagnationLimitIterations);
    const double stagnationThreshold = 1e-4; // rad/s
    size_t positionToleranceCounter = 0;
    const double positionToleranceLimitSeconds = 1.0; // Stop if position is met for 1 seconds
    const size_t positionToleranceLimit = static_cast<size_t>(std::ceil(positionToleranceLimitSeconds / stepDt));

    if (!this->pfKinematics) {
      // Cannot proceed without kinematics
      path.success = false;
      path.failureReason = "Kinematics module not initialized.";
      return path;
    }
    // Compute initial end-effector pose from joint angles using forward kinematics
    std::vector<double> currentJointAngles = startJointAngles;
    SpatialVector currentEEPose = this->pfKinematics->computeEndEffectorPose(currentJointAngles, this->eeLinkName);
    std::vector<double> prevJointVelocities(currentJointAngles.size(), 0.0);

    for (size_t iter = 0; iter < maxIters; ++iter) {
      // --- 1. Update robot obstacles from current joint configuration ---
      this->updateObstaclesFromKinematics(currentJointAngles);

      // --- 2. Compute whole-body joint velocities considering end-effector attraction and obstacle repulsion ---
      std::vector<Eigen::Vector3d> linkRepulsiveForces;
      std::vector<double> linkClearances;
      Eigen::Vector3d attractionForce;

      Eigen::VectorXd jointTorques = this->evaluateWholeBodyJointTorquesAtConfiguration(
        currentJointAngles, prevJointVelocities, currentEEPose, stepDt,
        &linkRepulsiveForces, &linkClearances, &attractionForce
      );

      Eigen::VectorXd jointVelocities = this->convertJointTorquesToJointVelocities(
        jointTorques, currentJointAngles, prevJointVelocities, stepDt
      );
      // Update prevJointVelocities for next iteration
      prevJointVelocities.assign(jointVelocities.data(), jointVelocities.data() + jointVelocities.size());

      // --- 3. Integrate joint velocities to get next joint configuration (Euler) ---
      std::vector<double> nextJointAngles(currentJointAngles.size());
      for (size_t i = 0; i < currentJointAngles.size(); ++i) {
        nextJointAngles[i] = currentJointAngles[i] + (jointVelocities[i] * stepDt);
      }

      // --- 4. Compute end-effector pose for the new joint configuration ---
      SpatialVector nextEEPose = this->pfKinematics->computeEndEffectorPose(nextJointAngles, this->eeLinkName);

      // --- 5. Compute end-effector twist (velocity) using Jacobian ---
      // Instead of finite differencing, we use the analytical Jacobian at the current configuration
      // to get the instantaneous task-space velocity corresponding to the computed joint velocities.
      Eigen::MatrixXd J = this->pfKinematics->getSpatialJacobianAtPoint(
        this->eeLinkName, currentEEPose.getPosition(), currentJointAngles
      );
      Eigen::VectorXd twistVec = J * jointVelocities; // jointVelocities is from step 2 (k1 equivalent)
      TaskSpaceTwist eeTwist(twistVec.head(3), twistVec.tail(3));

      // --- 6. Record current state in path ---
      path.recordPathPoint(
        timeStamp,
        currentEEPose,
        eeTwist,
        currentJointAngles,
        std::vector<double>(jointVelocities.data(), jointVelocities.data() + jointVelocities.size()),
        std::vector<double>(jointTorques.data(), jointTorques.data() + jointTorques.size()),
        linkClearances,
        attractionForce,
        linkRepulsiveForces
      );

      // --- 7. Check goal tolerance ---
      const double translationalError = (currentEEPose.getPosition() - this->goalPose.getPosition()).norm();
      const double rotationalError = currentEEPose.angularDistance(this->goalPose);
      if (translationalError <= goalTolerance) {
        if (rotationalError <= this->rotationalThreshold) {
          path.success = true;
          break;
        }
        // Position met, but rotation not yet
        positionToleranceCounter++;
        if (positionToleranceCounter >= positionToleranceLimit) {
          // We have held position for a while, likely unable to reach orientation
          // Terminate with success (or partial success)
          path.success = true;
          path.failureReason = "Position met, but not orientation";
          break;
        }
      }
      else {
        positionToleranceCounter = 0;
      }

      // --- 8. Check for collisions ---
      if (this->isRobotInCollisionWithEnvironment(0.05)) {
        // Robot is in collision - path planning failed
        path.success = false;
        path.failureReason = "Robot collided with environment.";
        break;
      }

      // --- 9. Check for stagnation ---
      if (jointVelocities.cwiseAbs().maxCoeff() < stagnationThreshold) {
        stagnationCounter++;
      }
      else {
        stagnationCounter = 0;
      }
      if (stagnationCounter >= stagnationLimit) {
        // Robot has stopped moving, likely reached a local minimum or goal
        // Check if we are close enough to be considered successful?
        // For now, we rely on the strict check in step 7. If we are here, we haven't met the strict tolerance.
        // But we should stop planning to avoid waiting for maxIters.
        path.failureReason = "Stagnation detected (local minimum or stuck).";
        path.success = false;
        break;
      }

      // --- 10. Update loop variables ---
      currentJointAngles = nextJointAngles;
      currentEEPose = nextEEPose;
      timeStamp += stepDt;
    }

    if (!path.success && path.failureReason.empty()) {
      path.failureReason = "Max iterations reached without converging to goal.";
    }

    // Finalize path metadata
    path.dt = stepDt;
    path.numPoints = static_cast<unsigned int>(path.poses.size());
    if (!path.timeStamps.empty()) {
      path.duration = path.timeStamps.back();
    }
    else {
      path.duration = 0.0;
    }

    return path;
  }

  bool PotentialField::isRobotInCollisionWithEnvironment(double clearanceThreshold) const {
    auto robotObstacles = this->getObstaclesByGroup(ObstacleGroup::ROBOT);
    auto envObstacles = this->getObstaclesByGroup(ObstacleGroup::STATIC);
    //TODO(Sharwin24): Handle Dynamic Obstacles? For now, we ignore them.

    // Setup collision request and result before looping over robot obstacles
    coal::CollisionRequest collisionRequest;
    if (clearanceThreshold > 0.0) {
      collisionRequest.security_margin = static_cast<coal::CoalScalar>(clearanceThreshold);
    }
    coal::CollisionResult collisionResult;

    for (const auto& robotLink : robotObstacles) {
      auto robotCollisionObj = robotLink.getCoalCollisionObject();
      if (!robotCollisionObj) continue;
      for (const auto& envObst : envObstacles) {
        auto envCollisionObj = envObst.getCoalCollisionObject();
        if (!envCollisionObj) continue;

        collisionResult.clear();
        coal::collide(robotCollisionObj.get(), envCollisionObj.get(), collisionRequest, collisionResult);
        if (collisionResult.isCollision()) { return true; }
      }
    }
    // Only after checking all robot-environment pairs, return false if no collisions detected
    return false;
  }

  bool PotentialField::createPlannedPathCSV(const PlannedPath& path, const std::string& filePath) const {
    std::ofstream csvFile;
    try {
      // Open file for writing first to ensure it's accessible
      csvFile.open(filePath, std::ios::out);
      if (!csvFile.is_open()) {
        return false;
      }
    }
    catch (const std::exception& e) {
      std::cerr << "[PotentialField ERROR]: Failed to open CSV file for writing: " << e.what() << std::endl;
      return false;
    }

    // Creates a CSV file from the given PlannedPath with the following
    const unsigned int numJoints = (path.numPoints > 0 && !path.jointAngles.empty()) ?
      static_cast<unsigned int>(path.jointAngles[0].size()) : 0;
    const unsigned int numLinks = (path.numPoints > 0 && !path.linkObstacleClearances.empty()) ?
      static_cast<unsigned int>(path.linkObstacleClearances[0].size()) : 0;

    auto jointPositionHeaders = [numJoints]() -> std::string {
      std::string jointHeaders;
      for (unsigned int j = 0; j < numJoints; ++j) {
        jointHeaders += "joint_" + std::to_string(j + 1) + "_rad,";
      }
      if (!jointHeaders.empty()) {
        // Insert leading comma
        jointHeaders = "," + jointHeaders;
        // Remove trailing comma
        jointHeaders.pop_back();
      }
      return jointHeaders;
    };
    auto jointVelocityHeaders = [numJoints]() -> std::string {
      std::string jointVelHeaders;
      for (unsigned int j = 0; j < numJoints; ++j) {
        jointVelHeaders += "joint_vel_" + std::to_string(j + 1) + "_rad_s,";
      }
      if (!jointVelHeaders.empty()) {
        // Insert leading comma
        jointVelHeaders = "," + jointVelHeaders;
        // Remove trailing comma
        jointVelHeaders.pop_back();
      }
      return jointVelHeaders;
    };
    auto jointTorqueHeaders = [numJoints]() -> std::string {
      std::string jointTorqueHeaders;
      for (unsigned int j = 0; j < numJoints; ++j) {
        jointTorqueHeaders += "joint_torque_" + std::to_string(j + 1) + "_N_m,";
      }
      if (!jointTorqueHeaders.empty()) {
        // Insert leading comma
        jointTorqueHeaders = "," + jointTorqueHeaders;
        // Remove trailing comma
        jointTorqueHeaders.pop_back();
      }
      return jointTorqueHeaders;
    };
    auto linkClearanceHeaders = [numLinks]() -> std::string {
      std::string headers;
      for (unsigned int j = 0; j < numLinks; ++j) {
        headers += "link_" + std::to_string(j) + "_clearance_m,";
      }
      if (!headers.empty()) {
        headers = "," + headers;
        headers.pop_back();
      }
      return headers;
    };
    auto linkRepulsiveForceHeaders = [numLinks]() -> std::string {
      std::string headers;
      for (unsigned int j = 0; j < numLinks; ++j) {
        headers += "link_" + std::to_string(j + 1) + "_rep_force_x_N," +
          "link_" + std::to_string(j + 1) + "_rep_force_y_N," +
          "link_" + std::to_string(j + 1) + "_rep_force_z_N,";
      }
      if (!headers.empty()) {
        headers = "," + headers;
        headers.pop_back();
      }
      return headers;
    };

    // CSV Header with dynamic joint columns based on number of joints
    const std::string header =
      "time_s,"
      "pos_x_m,pos_y_m,pos_z_m,"
      "q_x,q_y,q_z,q_w,"
      "vel_x_m_s,vel_y_m_s,vel_z_m_s,"
      "ang_vel_x_rad_s,ang_vel_y_rad_s,ang_vel_z_rad_s,"
      "min_obstacle_clearance_m,"
      "att_force_x_N,att_force_y_N,att_force_z_N"
      + jointPositionHeaders() + jointVelocityHeaders() + jointTorqueHeaders()
      + linkClearanceHeaders() + linkRepulsiveForceHeaders();
    try {
      // Write metadata as commented header lines
      const Eigen::Vector3d goalPosition = this->goalPose.getPosition();
      const Eigen::Quaterniond goalOri = this->goalPose.getOrientation();
      csvFile << "# Goal Position: [" << goalPosition.x() << ", " << goalPosition.y() << ", " << goalPosition.z() << "]\n";
      csvFile << "# Goal Orientation: [" << goalOri.x() << ", " << goalOri.y() << ", " << goalOri.z() << ", "
        << goalOri.w() << "]\n";
      csvFile << "# Goal Tolerance: " << path.goalTolerance << "\n";
      csvFile << "# Angular Tolerance: " << path.rotationalTolerance << "\n";
      csvFile << "# Num Joints: " << numJoints << "\n";
      csvFile << "# Planning Method: " << path.planningMethod << "\n";

      // Write header
      csvFile << header << "\n";

      // Write data rows
      for (unsigned int i = 0; i < path.numPoints; ++i) {
        // Safely obtain pose, twist, and timestamp with bounds checks
        SpatialVector pose = (i < path.poses.size()) ? path.poses[i] : SpatialVector();
        TaskSpaceTwist twist = (i < path.twists.size()) ? path.twists[i] : TaskSpaceTwist();
        double timeStamp = (i < path.timeStamps.size()) ? path.timeStamps[i] : 0.0;
        const Eigen::Vector3d position = pose.getPosition();
        const Eigen::Quaterniond orientation = pose.getOrientation();
        const double minClearance = this->minObstacleClearanceAt(position);

        // Write: time (s)
        csvFile << timeStamp << ",";

        // Write: position (x, y, z)
        csvFile << position.x() << "," << position.y() << "," << position.z() << ",";

        // Write: orientation quaternion (x, y, z, w)
        csvFile << orientation.x() << "," << orientation.y() << ","
          << orientation.z() << "," << orientation.w() << ",";

        // Write: linear velocity (x, y, z)
        csvFile << twist.linearVelocity.x() << "," << twist.linearVelocity.y() << ","
          << twist.linearVelocity.z() << ",";

        // Write: angular velocity (x, y, z)
        csvFile << twist.angularVelocity.x() << "," << twist.angularVelocity.y() << ","
          << twist.angularVelocity.z() << ",";

        // Write: minimum obstacle clearance (clearance)
        csvFile << minClearance << ",";

        // Write: attraction force (x, y, z)
        if (i < path.attractionForces.size()) {
          const auto& f = path.attractionForces[i];
          csvFile << f.x() << "," << f.y() << "," << f.z();
        }
        else {
          csvFile << "0,0,0";
        }

        // Helper lambda to safely write an inner double element or NaN if missing
        auto writeInnerDouble = [&](const std::vector<std::vector<double>>& mat, unsigned int row, unsigned int col) {
          return (row < mat.size() && col < mat[row].size()) ? mat[row][col] : std::numeric_limits<double>::quiet_NaN();
        };

        // Helper lambda to safely write an inner Vector3d element or NaNs if missing
        auto writeInnerVector3d = [&](const std::vector<std::vector<Eigen::Vector3d>>& mat, unsigned int row, unsigned int col) {
          if (row < mat.size() && col < mat[row].size()) {
            const auto& v = mat[row][col];
            csvFile << "," << v.x() << "," << v.y() << "," << v.z();
          } else {
            csvFile << "," << std::numeric_limits<double>::quiet_NaN()
                    << "," << std::numeric_limits<double>::quiet_NaN()
                    << "," << std::numeric_limits<double>::quiet_NaN();
          }
        };

        // Write: joint angles (rad)
        for (unsigned int j = 0; j < numJoints; ++j) {
          csvFile << "," << writeInnerDouble(path.jointAngles, i, j);
        }

        // Write: joint velocities
        for (unsigned int j = 0; j < numJoints; ++j) {
          csvFile << "," << writeInnerDouble(path.jointVelocities, i, j);
        }

        // Write: joint torques
        for (unsigned int j = 0; j < numJoints; ++j) {
          csvFile << "," << writeInnerDouble(path.jointTorques, i, j);
        }

        // Write: link clearances
        for (unsigned int j = 0; j < numLinks; ++j) {
          csvFile << "," << writeInnerDouble(path.linkObstacleClearances, i, j);
        }

        // Write: link repulsive forces
        for (unsigned int j = 0; j < numLinks; ++j) {
          writeInnerVector3d(path.repulsiveForces, i, j);
        }

        csvFile << "\n";
      }
    }
    catch (const std::exception& e) {
      std::cerr << "[PotentialField ERROR]: Failed to write to CSV file: " << e.what() << std::endl;
      return false;
    }

    csvFile.close();
    return true;
  }

} // namespace pfield
