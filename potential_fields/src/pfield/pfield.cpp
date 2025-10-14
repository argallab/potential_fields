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
  // Apply Velocity Limits by passing in dt=0.0
  return this->applyMotionConstraints(
    this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose)),
    TaskSpaceTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
    0.0
  );
}

TaskSpaceTwist PotentialField::wrenchToTwist(const TaskSpaceWrench& wrench) const {
  // If a mass/inertia model is available, it can be used here to convert force/torque to velocity
  const double linearGain = 1.0;  // (m/s) per N
  const double angularGain = 1.0; // (rad/s) per Nm
  return TaskSpaceTwist(wrench.force * linearGain, wrench.torque * angularGain);
}

TaskSpaceTwist PotentialField::applyMotionConstraints(
  const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double dt) const {
  auto isPositiveFinite = [](const double val) -> bool { return std::isfinite(val) && val > 1e-12; };
  auto clampNorm = [isPositiveFinite](const Eigen::Vector3d& vec, double maxNorm)-> Eigen::Vector3d {
    if (maxNorm <= 0.0) return vec;
    const double norm = vec.norm();
    return norm > maxNorm && isPositiveFinite(norm) ? (vec / norm) * maxNorm : vec;
  };
  if (!isPositiveFinite(dt)) {
    // If dt is too small, negative, or zero, only apply velocity limits
    return TaskSpaceTwist(
      clampNorm(twist.linearVelocity, this->maxLinearVelocity),
      clampNorm(twist.angularVelocity, this->maxAngularVelocity));
  }
  // Apply velocity limits first
  Eigen::Vector3d limitedLinear = clampNorm(twist.linearVelocity, this->maxLinearVelocity);
  Eigen::Vector3d limitedAngular = clampNorm(twist.angularVelocity, this->maxAngularVelocity);

  // Approximate acceleration using previous twist and dt
  const double dVMaxLinear = this->maxLinearAcceleration * dt; // m/s^2 * s = m/s
  const double dVMaxAngular = this->maxAngularAcceleration * dt; // rad/s^2 * s = rad/s

  auto limitStep = [isPositiveFinite](const Eigen::Vector3d& prev, const Eigen::Vector3d& curr, double dVMax) -> Eigen::Vector3d {
    if (dVMax <= 0.0) return curr; // No acceleration limit
    Eigen::Vector3d deltaV = curr - prev;
    const double deltaVNorm = deltaV.norm();
    if (deltaVNorm > dVMax && isPositiveFinite(deltaVNorm)) {
      return prev + deltaV * (dVMax / deltaVNorm);
    }
    return curr;
  };

  Eigen::Vector3d accelLimitedLinear = limitStep(prevTwist.linearVelocity, limitedLinear, dVMaxLinear);
  Eigen::Vector3d accelLimitedAngular = limitStep(prevTwist.angularVelocity, limitedAngular, dVMaxAngular);

  // Re-apply velocity limits after acceleration limiting to prevent overshoot and return
  return TaskSpaceTwist(
    clampNorm(accelLimitedLinear, this->maxLinearVelocity),
    clampNorm(accelLimitedAngular, this->maxAngularVelocity)
  );
}

SpatialVector PotentialField::interpolateNextPose(
  const SpatialVector& currentPose, const TaskSpaceTwist& prevTwist, const double dt) {
  // Compute the TaskSpaceTwist at the current pose
  TaskSpaceTwist vel = this->wrenchToTwist(this->evaluateWrenchAtPose(currentPose));
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
