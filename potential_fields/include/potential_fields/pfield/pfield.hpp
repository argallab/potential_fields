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
#include "pf_obstacle.hpp"
#include <eigen3/Eigen/Dense>

 // Functionality to support:
 // 1. Track a Goal Position (Initialization and Update)
 // 2. Create obstacles with a specified geometry
 // 3. Compute velocity vector at a given position

class PotentialField {
public:

  PotentialField();

  /**
   * @brief Constructs a PotentialField with the specified goal position.
   *        The attractive gain and rotational attractive gain are set to default values.
   *
   * @param goalPose The pose in 3D space that will generate an attractive force.
   */
  PotentialField(SpatialVector goalPose);

  PotentialField(SpatialVector goalPose, double attractiveGain, double rotationalAttractiveGain);

  ~PotentialField() = default;

  /**
 * @brief Updates the goal position (3D Vector) in the potential field.
 *
 * @note The Goal Position creates an attractive force towards it
 *
 * @param newGoalPose The new goal position to be set.
 */
  void updateGoalPosition(SpatialVector newGoalPose);

  /**
   * @brief Updates the attractive gain, scaling the force
   *        applied pulling points towards the goal.
   *
   * @param newAttractiveGain The new attractive gain to be set.
   */
  void updateAttractiveGain(double newAttractiveGain);

  /**
   * @brief Adds a new obstacle to the potential field.
   *
   * @param obstacle The obstacle to be added.
   */
  void addObstacle(PotentialFieldObstacle obstacle);

  /**
   * @brief Attempts to remove an obstacle by ID.
   *
   * @note If the obstacle is not found, no action is taken.
   * @param obstacleID  the ID of the obstacle obtained on obstacle creation.
   * @return true if the obstacle was removed successfully.
   */
  bool removeObstacle(int obstacleID);

  /**
   * @brief Clears all obstacles from the potential field.
   */
  void clearObstacles();

  PotentialFieldObstacle getObstacleByID(int obstacleID) const;

  /**
   * @brief Given a 3D position, computes the velocity vector
   *        by combining attractive and repulsive forces.
   *
   * @param queryPose The pose in 3D space to compute the velocity vector.
   * @return SpatialVector The resultant velocity vector.
   */
  SpatialVector evaluateVelocityAtPose(SpatialVector queryPose);

  Eigen::Vector3d angularVelocityFromQuaternion(const Eigen::Quaterniond& q, double deltaTime);

  /**
   * @brief Given a current orientation and a delta time, computes the angular velocity
   *        to compute the new orientation after the time step.
   *
   * @param currentOrientation The starting orientation as a quaternion.
   * @param deltaTime The time step over which to integrate the angular velocity [s].
   * @return Eigen::Quaterniond The resulting orientation after integrating the angular velocity.
   */
  Eigen::Quaterniond integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation, double deltaTime);

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
    const Eigen::Vector3d& linearVelocity, double deltaTime);

  SpatialVector getGoalPose() const;
  std::vector<PotentialFieldObstacle> getObstacles() const;

  bool isPointInsideObstacle(Eigen::Vector3d point) const;

  bool isPointWithinInfluenceZone(Eigen::Vector3d point) const;

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
  SpatialVector computeAttractiveForces(SpatialVector queryPose);

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
  SpatialVector computeRepulsiveForces(SpatialVector queryPose);
};

#endif // PFIELDS_HPP
