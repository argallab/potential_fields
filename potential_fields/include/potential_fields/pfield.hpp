/**
 * @file potential_field.hpp
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
#include "sphere_obstacle.hpp"

 // Functionality to support:
 // 1. Track a Goal Position (Initialization and Update)
 // 2. Create obstacles with a specified geometry
 // 3. Compute velocity vector at a given position

class PotentialField {
public:
  PotentialField() = default;
  PotentialField(SpatialVector goalPosition, float attractiveGain)
  : attractiveGain(attractiveGain), goalPosition(goalPosition)
  {
  }
  ~PotentialField() = default;

  /**
 * @brief Updates the goal position (3D Vector) in the potential field.
 *
 * @note The Goal Position creates an attractive force towards it
 *
 * @param newGoalPosition The new goal position to be set.
 */
  void updateGoalPosition(SpatialVector newGoalPosition);

  /**
   * @brief Updates the attractive gain, scaling the force
   *        applied pulling points towards the goal.
   *
   * @param newAttractiveGain The new attractive gain to be set.
   */
  void updateAttractiveGain(float newAttractiveGain);

  /**
   * @brief Adds a new obstacle to the potential field.
   *
   * @param obstacle The obstacle to be added.
   */
  void addObstacle(SphereObstacle obstacle);

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

  /**
   * @brief Given a 3D position, computes the velocity vector
   *        by combining attractive and repulsive forces.
   *
   * @param position The position in 3D space to compute the velocity vector.
   * @return Vector The resultant velocity vector.
   */
  SpatialVector computeVelocityAtPosition(SpatialVector position);

  SpatialVector getGoalPosition() const {return goalPosition;}
  std::vector<SphereObstacle> getObstacles() const {return obstacles;}

private:
  float attractiveGain = 1.0f; // Gain for attractive force
  float rotationalAttractiveGain = 0.7f; // Gain for rotational attractive force
  SpatialVector goalPosition;
  std::vector<SphereObstacle> obstacles;

  /**
   * @brief Computes the attractive force towards the goal position.
   *
   * @note Equation: F = attractiveGain * (distace * direction)
   *       where direction is a unit vector pointing towards the goal.
   *
   * @param position The position in 3D space to compute the force from.
   * @return Vector The attractive force vector.
   */
  SpatialVector computeAttractiveForces(SpatialVector position);

  /**
   * @brief Computes the repulsive forces from all obstacles
   *        and returns the resultant vector.
   *
   * @note Equation: F = repulsiveGain * (1/distance - 1/influence) * (1 / distance^2) * direction
   *       where direction is a unit vector pointing away from the obstacle.
   *
   * @param position The position in 3D space to compute the force from.
   * @return Vector The repulsive force vector.
   */
  SpatialVector computeRepulsiveForces(SpatialVector position);
};

#endif // PFIELDS_HPP
