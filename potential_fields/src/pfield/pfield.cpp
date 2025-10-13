#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"

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

TaskSpaceTwist PotentialField::wrenchToTwist(const TaskSpaceWrench& wrench) const {
  // If a mass/inertia model is available, it can be used here to convert force/torque to velocity
  const double linearGain = 1.0;  // (m/s) per N
  const double angularGain = 1.0; // (rad/s) per Nm
  return TaskSpaceTwist(wrench.force * linearGain, wrench.torque * angularGain);
}

TaskSpaceTwist PotentialField::applyVelocityLimits(
  const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double deltaTime) const {
  Eigen::Vector3d limitedLinear = twist.linearVelocity;
  Eigen::Vector3d limitedAngular = twist.angularVelocity;
  // Approximate acceleration using previous twist
  Eigen::Vector3d linearAccel = (limitedLinear - prevTwist.linearVelocity) / deltaTime;
  Eigen::Vector3d angularAccel = (limitedAngular - prevTwist.angularVelocity) / deltaTime;
  double linearAccelNorm = linearAccel.norm();
  double angularAccelNorm = angularAccel.norm();
  // Clamp velocities and accelerations if they exceed limits
  if (limitedLinear.norm() > this->maxLinearVelocity) {
    limitedLinear = (limitedLinear / limitedLinear.norm()) * this->maxLinearVelocity;
  }
  if (limitedAngular.norm() > this->maxAngularVelocity) {
    limitedAngular = (limitedAngular / limitedAngular.norm()) * this->maxAngularVelocity;
  }
  if (linearAccelNorm > this->maxLinearAcceleration) {
    linearAccel = (linearAccel / linearAccelNorm) * this->maxLinearAcceleration;
    limitedLinear = prevTwist.linearVelocity + linearAccel * deltaTime;
  }
  if (angularAccelNorm > this->maxAngularAcceleration) {
    angularAccel = (angularAccel / angularAccelNorm) * this->maxAngularAcceleration;
    limitedAngular = prevTwist.angularVelocity + angularAccel * deltaTime;
  }
  return TaskSpaceTwist(limitedLinear, limitedAngular);
}

SpatialVector PotentialField::interpolateNextPose(const SpatialVector& currentPose, const double deltaTime) {
  // Compute the TaskSpaceTwist at the current pose
  TaskSpaceTwist vel = this->wrenchToTwist(this->evaluateWrenchAtPose(currentPose));
  // Integrate linear and angular velocities to get next pose
  Eigen::Vector3d nextPosition = this->integrateLinearVelocity(currentPose.getPosition(), vel.linearVelocity, deltaTime);
  Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(currentPose.getOrientation(), vel.angularVelocity, deltaTime);
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

Eigen::Vector3d PotentialField::computeAttractiveForceLinear(const SpatialVector& queryPose) const {
  const Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
  const double euclideanDistance = direction.norm();
  if (euclideanDistance <= this->translationalTolerance) return Eigen::Vector3d::Zero();
  else return direction.normalized() * (-this->attractiveGain * euclideanDistance);
}

Eigen::Vector3d PotentialField::computeAttractiveMoment(const SpatialVector& queryPose) const {
  Eigen::Quaterniond orientationDiff = queryPose.getOrientation().conjugate() * this->goalPose.getOrientation();
  orientationDiff.normalize();
  const double angularDistance = queryPose.angularDistance(this->goalPose);
  if (angularDistance <= this->rotationalThreshold) return Eigen::Vector3d::Zero();
  Eigen::Vector3d u = orientationDiff.vec() / orientationDiff.vec().norm();
  return -this->rotationalAttractiveGain * (angularDistance * u);
}

Eigen::Vector3d PotentialField::computeRepulsiveForceLinear(const SpatialVector& queryPose) const {
  Eigen::Vector3d F = Eigen::Vector3d::Zero();
  for (const auto& obst : this->obstacles) {
    if (!obst.withinInfluenceZone(queryPose.getPosition())) continue;
    Eigen::Vector3d direction = queryPose.getPosition() - obst.getPosition();
    const double distance = direction.norm();
    const double distanceReciprocal = 1.0 / distance;
    const double distanceReciprocalSquared = 1.0 / (distance * distance);
    const double influenceReciprocal = 1.0 / obst.getInfluenceZoneScale();
    const double magnitude = obst.getRepulsiveGain() * (distanceReciprocal - influenceReciprocal) * distanceReciprocalSquared;
    if (magnitude > 0.0) F += direction.normalized() * magnitude;
  }
  return F;
}
