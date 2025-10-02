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
  int id = obstacle.getID();
  const auto it = std::find_if(this->obstacles.begin(), this->obstacles.end(),
    [id](const PotentialFieldObstacle& obs) {return obs.getID() == id;});
  if (it != this->obstacles.end()) {
    // Obstacle with the same ID already exists, update it
    *it = obstacle;
    return;
  }
  else {
    // New obstacle, add it to the list
    this->obstacles.push_back(obstacle);
  }
}

bool PotentialField::removeObstacle(int obstacleID) {
  const auto it = std::remove_if(this->obstacles.begin(), this->obstacles.end(),
    [obstacleID](const PotentialFieldObstacle& obs) {return obs.getID() == obstacleID;});
  if (it != obstacles.end()) {
    obstacles.erase(it, obstacles.end());
    return true;
  }
  return false;
}

void PotentialField::clearObstacles() { this->obstacles.clear(); }

PotentialFieldObstacle PotentialField::getObstacleByID(int obstacleID) const {
  const auto it = std::find_if(this->obstacles.cbegin(), this->obstacles.cend(),
    [obstacleID](const PotentialFieldObstacle& obs) {return obs.getID() == obstacleID;});
  if (it != this->obstacles.cend()) {
    return *it;
  }
  else {
    throw std::invalid_argument("Obstacle with the given ID does not exist.");
  }
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

Eigen::Quaterniond PotentialField::integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation, double deltaTime) {
  Eigen::Vector3d angularVelocity = angularVelocityFromQuaternion(currentOrientation, deltaTime);
  const double epsilon = 1e-6; // Small value to avoid division by zero
  if (angularVelocity.norm() < epsilon) {
    // If angular velocity is near zero, return the current orientation
    return currentOrientation;
  }
  Eigen::Quaterniond deltaOrientation;
  deltaOrientation = Eigen::AngleAxisd(angularVelocity.norm() * deltaTime, angularVelocity.normalized());
  return (currentOrientation * deltaOrientation).normalized();
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
