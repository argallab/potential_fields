#include "pfield.hpp"
#include <algorithm>

void PotentialField::updateGoalPosition(SpatialVector newGoalPose) {
  this->goalPose = newGoalPose;
}

void PotentialField::updateAttractiveGain(double newAttractiveGain) {
  this->attractiveGain = newAttractiveGain;
}

void PotentialField::addObstacle(SphereObstacle obstacle) {
  int id = obstacle.getID();
  const auto it = std::find_if(this->obstacles.begin(), this->obstacles.end(),
    [id](const SphereObstacle& obs) {return obs.getID() == id;});
  if (it != this->obstacles.end()) {
    // Obstacle with the same ID already exists, update it
    *it = obstacle;
    return;
  } else {
    // New obstacle, add it to the list
    this->obstacles.push_back(obstacle);
  }
}

bool PotentialField::removeObstacle(int obstacleID) {
  const auto it = std::remove_if(this->obstacles.begin(), this->obstacles.end(),
    [obstacleID](const SphereObstacle& obs) {return obs.getID() == obstacleID;});
  if (it != obstacles.end()) {
    obstacles.erase(it, obstacles.end());
    return true;
  }
  return false;
}

void PotentialField::clearObstacles() {
  this->obstacles.clear();
}

SpatialVector PotentialField::evaluateVelocityAtPose(SpatialVector queryPose) {
  SpatialVector attractiveForce = this->computeAttractiveForces(queryPose);
  SpatialVector repulsiveForce = this->computeRepulsiveForces(queryPose);
  Eigen::Vector3d totalForce = attractiveForce.getPosition() + repulsiveForce.getPosition();
  Eigen::Quaterniond totalOrientation = attractiveForce.getOrientation() * repulsiveForce.getOrientation();
  // Normalize the total orientation quaternion
  totalOrientation.normalize();
  return SpatialVector(totalForce, totalOrientation);
}

SpatialVector PotentialField::computeAttractiveForces(SpatialVector queryPose) {
  SpatialVector attractiveForce;
  // Attractive force towards the goal position (pos - goal)
  Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
  // Normalize the direction vector
  double distance = direction.norm();
  // If distance is (near) zero, the translational force is zero
  if (distance > this->translationalTolerance) {
    direction /= distance; // Normalize
    // Attractive force is negative
    double magnitude = -this->attractiveGain * distance;
    Eigen::Vector3d forceVector = direction * magnitude;
    attractiveForce.setPosition(forceVector);
  }
  // Determine the orientation attraction to "rotate" the position towards the goal orientation
  // We want to rotate the position towards the goal orientation
  // So we can apply a rotational force proportional to the geodesic distance
  Eigen::Quaterniond orientationDiff = queryPose.getOrientation().inverse() * this->goalPose.getOrientation();
  double geodesicDistance = queryPose.geodesicDistance(this->goalPose);
  // If geodesic distance is (near) zero, don't apply rotational force
  if (geodesicDistance < this->rotationalThreshold) {
    return attractiveForce;
  } else {
    // Apply a rotational force proportional to the geodesic distance and the gain
    double rotationalMagnitude = -this->rotationalAttractiveGain * geodesicDistance;
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
    if (obst.withinInfluenceRadius(queryPose.getPosition())) {
      Eigen::Vector3d obstPosition = obst.getPosition();
      Eigen::Vector3d direction = queryPose.getPosition() - obstPosition;
      // Normalize the direction vector
      double distance = direction.norm();
      // If distance is (near) zero, don't apply force
      if (distance < this->translationalTolerance) { continue; }
      direction /= distance; // Normalize the direction
      // Calculate the repulsive force magnitude
      double magnitude = obst.getRepulsiveGain() *
        ((1 / distance) - (1 / obst.getInfluenceRadius())) * (1.0f / (distance * distance));
      // Add the repulsive forces together
      repulsiveForceVector += (direction * magnitude);
    }
  }
  repulsiveForce.setPosition(repulsiveForceVector);
  return repulsiveForce;
}
