#include "pfield.hpp"
#include <algorithm>

void PotentialField::updateGoalPosition(SpatialVector newGoalPosition) {
  this->goalPosition = newGoalPosition;
}

void PotentialField::updateAttractiveGain(float newAttractiveGain) {
  this->attractiveGain = newAttractiveGain;
}

void PotentialField::addObstacle(SphereObstacle obstacle) {
  int id = obstacle.getID();
  const auto it = std::find_if(this->obstacles.begin(), this->obstacles.end(),
    [id](const SphereObstacle& obs) { return obs.getID() == id; });
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
    [obstacleID](const SphereObstacle& obs) { return obs.getID() == obstacleID; });
  if (it != obstacles.end()) {
    obstacles.erase(it, obstacles.end());
    return true;
  }
  return false;
}

void PotentialField::clearObstacles() {
  this->obstacles.clear();
}

SpatialVector PotentialField::computeVelocityAtPosition(SpatialVector position) {
  SpatialVector attractiveForce = this->computeAttractiveForces(position);
  SpatialVector repulsiveForce = this->computeRepulsiveForces(position);
  return attractiveForce + repulsiveForce;
}

SpatialVector PotentialField::computeAttractiveForces(SpatialVector position) {
  // Attractive force towards the goal position (pos - goal)
  SpatialVector direction = position - this->goalPosition;
  float distance = position.euclideanDistance(this->goalPosition);
  // If distance is (near) zero, return zero force
  /// TODO: Parameterize this small threshold
  if (distance < 1e-3) { return SpatialVector{0, 0, 0}; }
  direction /= distance; // Normalize the direction
  // Attractive force is negative
  float magnitude = -this->attractiveGain * distance;
  SpatialVector attractiveForce = direction * magnitude;
  // Determine the orientation attraction to "rotate" the position towards the goal orientation
  // We want to rotate the position towards the goal orientation
  // So we can apply a rotational force proportional to the geodesic distance
  SpatialVector orientationDiff = position.quaternionDifference(this->goalPosition);
  float geodesicDistance = position.geodesicDistance(this->goalPosition);
  // If geodesic distance is (near) zero, don't apply rotational force
  /// TODO: Parameterize this small threshold (obtained from original pfields repo)
  if (geodesicDistance < 0.06f) {
    return attractiveForce;
  } else {
    attractiveForce.setOrientationQuaternion(
      orientationDiff.getQX() * this->rotationalAttractiveGain,
      orientationDiff.getQY() * this->rotationalAttractiveGain,
      orientationDiff.getQZ() * this->rotationalAttractiveGain,
      orientationDiff.getQW()
    );
  }
  return attractiveForce;
}

SpatialVector PotentialField::computeRepulsiveForces(SpatialVector position) {
  SpatialVector repulsiveForce{0, 0, 0};
  for (const auto& obst : this->obstacles) {
    // Each obstacle is a sphere
    // Only calculate repulsive force if within influence radius
    if (obst.withinInfluenceRadius(position)) {
      SpatialVector obstPosition = obst.getPosition();
      SpatialVector direction = position - obstPosition;
      float distance = position.euclideanDistance(obstPosition);
      // If distance is (near) zero, don't apply force
      /// TODO: Parameterize this small threshold
      if (distance < 1e-3) { continue; }
      direction /= distance; // Normalize the direction
      // Calculate the repulsive force magnitude
      float magnitude = obst.getRepulsiveGain() * ((1 / distance) - (1 / obst.getInfluenceRadius())) * (1.0f / (distance * distance));
      // Add the repulsive forces together
      repulsiveForce += (direction * magnitude);
    }
  }
  return repulsiveForce;
}