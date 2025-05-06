#include "pfields.hpp"
#include <algorithm>

void PotentialField::updateGoalPosition(Vector newGoalPosition) {
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

Vector PotentialField::computeVelocityAtPosition(Vector position) {
  Vector attractiveForce = this->computeAttractiveForces(position);
  Vector repulsiveForce = this->computeRepulsiveForces(position);
  return attractiveForce + repulsiveForce;
}

Vector PotentialField::computeAttractiveForces(Vector position) {
  // Attractive force towards the goal position (pos - goal)
  float distance = position.euclideanDistance(this->goalPosition);
  Vector direction = position - this->goalPosition;
  // If distance is (near) zero, return zero force
  /// TODO: Parameterize this small threshold
  if (distance < 1e-3) { return Vector{0, 0, 0}; }
  direction /= distance; // Normalize the direction
  // Attractive force is negative
  float magnitude = -this->attractiveGain * distance;
  return direction * magnitude;
}

Vector PotentialField::computeRepulsiveForces(Vector position) {
  Vector repulsiveForce{0, 0, 0};
  for (const auto& obst : this->obstacles) {
    // Each obstacle is a sphere
    // Only calculate repulsive force if within influence radius
    if (obst.withinInfluenceRadius(position)) {
      Vector obstPosition = obst.getPosition();
      Vector direction = position - obstPosition;
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