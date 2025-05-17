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
#include "sphere_obstacle.hpp"
#include <eigen3/Eigen/Dense>

 // Functionality to support:
 // 1. Track a Goal Position (Initialization and Update)
 // 2. Create obstacles with a specified geometry
 // 3. Compute velocity vector at a given position

class PotentialField {
public:
  PotentialField() = default;
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
   * @param queryPose The pose in 3D space to compute the velocity vector.
   * @return SpatialVector The resultant velocity vector.
   */
  SpatialVector evaluateVelocityAtPose(SpatialVector queryPose);

  SpatialVector getGoalPose() const { return this->goalPose; }
  std::vector<SphereObstacle> getObstacles() const { return this->obstacles; }

private:
  double attractiveGain = 1.0f; // Gain for attractive force
  double rotationalAttractiveGain = 0.7f; // Gain for rotational attractive force
  double translationalTolerance = 1e-3f; // Threshold for distances to the goal and obstacles
  double rotationalThreshold = 0.06f; // Threshold for rotational geodesic distance
  SpatialVector goalPose;
  std::vector<SphereObstacle> obstacles;

  /**
   * @brief Computes the attractive force towards the goal pose. Also computes the
   * rotational force to align the query pose with the goal pose.
   *
   * @note Equation: F = attractiveGain * (distace * direction)
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
