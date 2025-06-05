/**
 * @file pfield.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Library for managing a vector field overlaid onto a
 *        robot's task-space (3D World) as a velocity field
 *        to plan motion trajectories.
 * @version 1.0
 * @date 2025-05-08
 *
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef PFIELDS_HPP
#define PFIELDS_HPP
#include <vector>
#include <stdexcept>
#include <math.h>
#include "spatial_vector.hpp"
#include "potential_field_obstacle.hpp"
#include <eigen3/Eigen/Dense>

 // Functionality to support:
 // 1. Track a Goal Position (Initialization and Update)
 // 2. Create obstacles with a specified geometry
 // 3. Compute velocity vector at a given position

class PotentialField {
public:

  PotentialField() = default;

  /**
   * @brief Constructs a PotentialField with the specified goal position.
   *        The attractive gain and rotational attractive gain are set to default values.
   *
   * @param goalPose The pose in 3D space that will generate an attractive force.
   */
  PotentialField(SpatialVector goalPose) :
    goalPose(goalPose) {
  }

  PotentialField(SpatialVector goalPose, double attractiveGain, double rotationalAttractiveGain) : attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    goalPose(goalPose) {
  }

  ~PotentialField() = default;

  /**
 * @brief Updates the goal position (3D Vector) in the potential field.
 *
 * @note The Goal Position creates an attractive force towards it
 *
 * @param newGoalPose The new goal position to be set.
 */
  void updateGoalPosition(SpatialVector newGoalPose) {
    this->goalPose = newGoalPose;
  }

  /**
   * @brief Updates the attractive gain, scaling the force
   *        applied pulling points towards the goal.
   *
   * @param newAttractiveGain The new attractive gain to be set.
   */
  void updateAttractiveGain(double newAttractiveGain) {
    this->attractiveGain = newAttractiveGain;
  }

  /**
   * @brief Adds a new obstacle to the potential field.
   *
   * @param obstacle The obstacle to be added.
   */
  void addObstacle(PotentialFieldObstacle obstacle) {
    int id = obstacle.getID();
    const auto it = std::find_if(this->obstacles.begin(), this->obstacles.end(),
      [id](const PotentialFieldObstacle& obs) {return obs.getID() == id;});
    if (it != this->obstacles.end()) {
      // Obstacle with the same ID already exists, update it
      *it = obstacle;
      return;
    } else {
      // New obstacle, add it to the list
      this->obstacles.push_back(obstacle);
    }
  }

  /**
   * @brief Attempts to remove an obstacle by ID.
   *
   * @note If the obstacle is not found, no action is taken.
   * @param obstacleID  the ID of the obstacle obtained on obstacle creation.
   * @return true if the obstacle was removed successfully.
   */
  bool removeObstacle(int obstacleID) {
    const auto it = std::remove_if(this->obstacles.begin(), this->obstacles.end(),
      [obstacleID](const PotentialFieldObstacle& obs) {return obs.getID() == obstacleID;});
    if (it != obstacles.end()) {
      obstacles.erase(it, obstacles.end());
      return true;
    }
    return false;
  }

  /**
   * @brief Clears all obstacles from the potential field.
   */
  void clearObstacles() { this->obstacles.clear(); }

  PotentialFieldObstacle getObstacleByID(int obstacleID) const {
    const auto it = std::find_if(this->obstacles.cbegin(), this->obstacles.cend(),
      [obstacleID](const PotentialFieldObstacle& obs) {return obs.getID() == obstacleID;});
    if (it != this->obstacles.cend()) {
      return *it;
    } else {
      throw std::invalid_argument("Obstacle with the given ID does not exist.");
    }
  }

  /**
   * @brief Given a 3D position, computes the velocity vector
   *        by combining attractive and repulsive forces.
   *
   * @param queryPose The pose in 3D space to compute the velocity vector.
   * @return SpatialVector The resultant velocity vector.
   */
  SpatialVector evaluateVelocityAtPose(SpatialVector queryPose) {
    SpatialVector attractiveForce = this->computeAttractiveForces(queryPose);
    SpatialVector repulsiveForce = this->computeRepulsiveForces(queryPose);
    Eigen::Vector3d totalForce = attractiveForce.getPosition() + repulsiveForce.getPosition();
    Eigen::Quaterniond totalOrientation = attractiveForce.getOrientation() * repulsiveForce.getOrientation();
    totalOrientation.normalize();
    return SpatialVector(totalForce, totalOrientation);
  }

  Eigen::Vector3d angularVelocityFromQuaternion(const Eigen::Quaterniond& q, double deltaTime) {
    Eigen::Vector3d axis;
    double angle;
    if (q.w() > 1.0) {
      // Normalize the quaternion
      Eigen::Quaterniond normalizedQ = q.normalized();
      axis = normalizedQ.vec();
      angle = 2.0 * std::acos(normalizedQ.w());
    } else {
      axis = q.vec();
      angle = 2.0 * std::acos(q.w());
    }
    // Angular velocity is the axis of rotation scaled by the angle over time
    return axis.normalized() * (angle / deltaTime);
  }

  /**
   * @brief Given a current orientation and a delta time, computes the angular velocity
   *        to compute the new orientation after the time step.
   *
   * @param currentOrientation The starting orientation as a quaternion.
   * @param deltaTime The time step over which to integrate the angular velocity [s].
   * @return Eigen::Quaterniond The resulting orientation after integrating the angular velocity.
   */
  Eigen::Quaterniond integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation, double deltaTime) {
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

  /**
   * @brief Given a current position, an instantaneous linear velocity vector, and a delta time,
   *        integrates the linear velocity to compute the new position after the time step.
   *
   * @param currentPosition The starting position as a 3D vector.
   * @param linearVelocity The instantaneous linear velocity vector in meters per second.
   * @param deltaTime The time step over which to integrate the linear velocity [s].
   * @return Eigen::Vector3d The resulting position after integrating the linear velocity.
   */
  Eigen::Vector3d integrateLinearVelocity(const Eigen::Vector3d& currentPosition,
    const Eigen::Vector3d& linearVelocity, double deltaTime) {
    return currentPosition + (linearVelocity * deltaTime);
  }

  SpatialVector getGoalPose() const { return this->goalPose; }
  std::vector<PotentialFieldObstacle> getObstacles() const { return this->obstacles; }

  bool isPointInsideObstacle(Eigen::Vector3d point) const {
    for (const auto& obst : this->obstacles) {
      if (obst.withinObstacle(point)) { return true; }
    }
    return false;
  }

  bool isPointWithinInfluenceZone(Eigen::Vector3d point) const {
    for (const auto& obst : this->obstacles) {
      if (obst.withinInfluenceZone(point)) { return true; }
    }
    return false;
  }

private:
  double attractiveGain = 1.0f; // Gain for attractive force
  double rotationalAttractiveGain = 0.7f; // Gain for rotational attractive force
  double translationalTolerance = 1e-3f; // Threshold for distances to the goal and obstacles
  double rotationalThreshold = 0.06f; // Threshold for rotational geodesic distance
  SpatialVector goalPose;
  std::vector<PotentialFieldObstacle> obstacles;

  /**
   * @brief Computes the attractive force towards the goal pose. Also computes the
   * rotational force to align the query pose with the goal pose.
   *
   * @note Equation: F = -attractiveGain * (distace * direction)
   *       where direction is a unit vector pointing towards the goal.
   *
   * @param queryPose The pose in 3D space to compute the force from.
   * @return SpatialVector The attractive force vector.
   */
  SpatialVector computeAttractiveForces(SpatialVector queryPose) {
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
    } else {
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

  /**
   * @brief Computes the repulsive forces from all obstacles
   *        and returns the resultant vector.
   *
   * @note Equation: F = repulsiveGain * (1/distance - 1/influence) * (1 / distance^2) * direction
   *       where direction is a unit vector pointing away from the obstacle.
   *
   * @param queryPose The position in 3D space to compute the force from.
   * @return SpatialVector The repulsive force vector.
   */
  SpatialVector computeRepulsiveForces(SpatialVector queryPose) {
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
};

#endif // PFIELDS_HPP
