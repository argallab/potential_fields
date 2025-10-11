#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"

void PotentialField::updateGoalPosition(SpatialVector newGoalPose) {
  this->goalPose = newGoalPose;
}

void PotentialField::updateAttractiveGain(double newAttractiveGain) {
  this->attractiveGain = newAttractiveGain;
}

void PotentialField::addObstacle(PotentialFieldObstacle obstacle) {
  const std::string frameID = obstacle.getFrameID();
  auto itIndex = this->obstacleIndex.find(frameID);
  if (itIndex != this->obstacleIndex.end()) {
    // Update existing obstacle in place
    this->obstacles[itIndex->second] = obstacle;
  }
  else {
    // Append new obstacle and record index
    this->obstacles.push_back(obstacle);
    this->obstacleIndex.emplace(frameID, this->obstacles.size() - 1);
  }
}

bool PotentialField::removeObstacle(const std::string& obstacleFrameID) {
  auto itIndex = this->obstacleIndex.find(obstacleFrameID);
  if (itIndex == this->obstacleIndex.end()) { return false; }
  size_t idx = itIndex->second;
  // Swap erase to keep indices valid with minimal moves
  size_t last = this->obstacles.size() - 1;
  if (idx != last) {
    std::swap(this->obstacles[idx], this->obstacles[last]);
    // Update moved obstacle's index map
    this->obstacleIndex[this->obstacles[idx].getFrameID()] = idx;
  }
  this->obstacles.pop_back();
  this->obstacleIndex.erase(itIndex);
  return true;
}

void PotentialField::clearObstacles() { this->obstacles.clear(); this->obstacleIndex.clear(); }

PotentialFieldObstacle PotentialField::getObstacleByID(const std::string& obstacleFrameID) const {
  auto itIndex = this->obstacleIndex.find(obstacleFrameID);
  if (itIndex == this->obstacleIndex.end()) {
    throw std::invalid_argument("Obstacle with the given ID does not exist.");
  }
  return this->obstacles[itIndex->second];
}

SpatialVector PotentialField::evaluateVelocityAtPose(SpatialVector queryPose) {
  SpatialVector attractiveForce = this->computeAttractiveForces(queryPose);
  SpatialVector repulsiveForce = this->computeRepulsiveForces(queryPose);
  Eigen::Vector3d totalForce = attractiveForce.getPosition() + repulsiveForce.getPosition();
  Eigen::Quaterniond totalOrientation = attractiveForce.getOrientation() * repulsiveForce.getOrientation();
  totalOrientation.normalize();
  return SpatialVector(totalForce, totalOrientation);
}

Eigen::Vector3d PotentialField::angularVelocityFromQuaternion(const Eigen::Quaterniond& q, double deltaTime) {
  Eigen::Vector3d axis;
  double angle;
  if (q.w() > 1.0) {
    // Normalize the quaternion
    Eigen::Quaterniond normalizedQ = q.normalized();
    axis = normalizedQ.vec();
    angle = 2.0 * std::acos(normalizedQ.w());
  }
  else {
    axis = q.vec();
    angle = 2.0 * std::acos(q.w());
  }
  // Angular velocity is the axis of rotation scaled by the angle over time
  return axis.normalized() * (angle / deltaTime);
}

SpatialVector PotentialField::interpolateNextPose(const SpatialVector& currentPose, const double deltaTime) {
  // 1) Linear velocity from the field
  SpatialVector vel = this->evaluateVelocityAtPose(currentPose);
  // 2) Rotational velocity from orientation error (not from the absolute quaternion)
  Eigen::Vector3d w = this->computeRotationalVelocity(currentPose);
  // 3) Integrate
  Eigen::Vector3d nextPosition = this->integrateLinearVelocity(currentPose.getPosition(), vel.getPosition(), deltaTime);
  Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(currentPose.getOrientation(), w, deltaTime);
  return SpatialVector(nextPosition, nextOrientation);
}

Eigen::Quaterniond PotentialField::integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation,
  const Eigen::Vector3d& angularVelocity,
  double deltaTime) {
  const double wnorm = angularVelocity.norm();
  if (wnorm < 1e-9 || deltaTime <= 0.0) return currentOrientation;
  // Exponential map: dq = exp( [w]^ * dt ) ≈ AngleAxis( |w|*dt, w/|w| )
  Eigen::Quaterniond dq(Eigen::AngleAxisd(wnorm * deltaTime, angularVelocity / wnorm));
  return (currentOrientation * dq).normalized();
}

Eigen::Vector3d PotentialField::computeRotationalVelocity(const SpatialVector& queryPose) const {
  // Orientation error from current to goal
  Eigen::Quaterniond q_err = queryPose.getOrientation().conjugate() * this->goalPose.getOrientation();
  q_err.normalize();
  Eigen::AngleAxisd aa(q_err);
  // Drive along error axis, proportional to error angle
  // (Negative sign pulls toward goal, mirroring your translational attractive force)
  return -this->rotationalAttractiveGain * (aa.axis() * aa.angle());
}

Eigen::Vector3d PotentialField::integrateLinearVelocity(const Eigen::Vector3d& currentPosition,
  const Eigen::Vector3d& linearVelocity, double deltaTime) {
  return currentPosition + (linearVelocity * deltaTime);
}

SpatialVector PotentialField::getGoalPose() const { return this->goalPose; }
std::vector<PotentialFieldObstacle> PotentialField::getObstacles() const { return this->obstacles; }

bool PotentialField::isPointInsideObstacle(Eigen::Vector3d point) const {
  for (const auto& obst : this->obstacles) {
    if (obst.withinObstacle(point)) { return true; }
  }
  return false;
}

bool PotentialField::isPointWithinInfluenceZone(Eigen::Vector3d point) const {
  for (const auto& obst : this->obstacles) {
    if (obst.withinInfluenceZone(point)) { return true; }
  }
  return false;
}

SpatialVector PotentialField::computeAttractiveForces(SpatialVector queryPose) {
  SpatialVector attractiveForce;
  // Attractive force towards the goal position (pos - goal)
  Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
  double distance = direction.norm();
  // If distance is (near) zero, the translational force is zero
  if (distance > this->translationalTolerance) {
    direction.normalize();
    // Attractive force is negative
    double magnitude = -this->attractiveGain * distance;
    Eigen::Vector3d forceVector = direction * magnitude;
    attractiveForce.setPosition(forceVector);
  }
  // Determine the orientation attraction to "rotate" the position towards the goal orientation
  // We want to rotate the position towards the goal orientation
  // So we can apply a rotational force proportional to the geodesic distance
  Eigen::Quaterniond orientationDiff = queryPose.getOrientation().inverse() * this->goalPose.getOrientation();
  double angularDistance = queryPose.angularDistance(this->goalPose);
  // If geodesic distance is (near) zero, don't apply rotational force
  if (angularDistance < this->rotationalThreshold) {
    return attractiveForce;
  }
  else {
    // Apply a rotational force proportional to the geodesic distance and the gain
    double rotationalMagnitude = -this->rotationalAttractiveGain * angularDistance;
    // Apply the rotational force
    orientationDiff.x() *= rotationalMagnitude;
    orientationDiff.y() *= rotationalMagnitude;
    orientationDiff.z() *= rotationalMagnitude;
    orientationDiff.w() *= rotationalMagnitude;
    orientationDiff.normalize();
    attractiveForce.setOrientation(orientationDiff);
  }
  return attractiveForce;
}

SpatialVector PotentialField::computeRepulsiveForces(SpatialVector queryPose) {
  SpatialVector repulsiveForce;
  // Copy the orientation from the query
  repulsiveForce.setOrientation(queryPose.getOrientation());
  Eigen::Vector3d repulsiveForceVector = Eigen::Vector3d::Zero();
  for (const auto& obst : this->obstacles) {
    // Each obstacle is a sphere
    // Only calculate repulsive force if within influence radius
    if (obst.withinInfluenceZone(queryPose.getPosition())) {
      Eigen::Vector3d obstPosition = obst.getPosition();
      Eigen::Vector3d direction = queryPose.getPosition() - obstPosition;
      // Normalize the direction vector
      double distance = direction.norm();
      // If distance is (near) zero, don't apply force
      if (distance < this->translationalTolerance) { continue; }
      direction.normalize();
      // Calculate the repulsive force magnitude
      double magnitude = obst.getRepulsiveGain() *
        ((1 / distance) - (1 / obst.getInfluenceZoneScale())) * (1.0f / (distance * distance));
      // Add the repulsive forces together
      repulsiveForceVector += (direction * magnitude);
    }
  }
  repulsiveForce.setPosition(repulsiveForceVector);
  return repulsiveForce;
}
