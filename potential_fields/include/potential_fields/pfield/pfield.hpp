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

#include <math.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "spatial_vector.hpp"
#include "pf_obstacle.hpp"

class PotentialField {
public:
  PotentialField() = default;
  ~PotentialField() = default;

  PotentialField(const PotentialField& other) :
    attractiveGain(other.attractiveGain),
    rotationalAttractiveGain(other.rotationalAttractiveGain),
    goalPose(other.goalPose),
    obstacles(other.obstacles) {}

  PotentialField& operator=(const PotentialField& other) {
    if (this != &other) {
      this->attractiveGain = other.attractiveGain;
      this->rotationalAttractiveGain = other.rotationalAttractiveGain;
      this->goalPose = other.goalPose;
      this->obstacles = other.obstacles;
    }
    return *this;
  }

  /**
   * @brief Constructs a PotentialField with the specified goal position.
   *        The attractive gain and rotational attractive gain are set to default values.
   *
   * @param goalPose The pose in 3D space that will generate an attractive force.
   */
  explicit PotentialField(SpatialVector goalPose) :
    attractiveGain(1.0),
    rotationalAttractiveGain(0.7),
    goalPose(goalPose),
    maxVelocity(1.0) {}

  PotentialField(SpatialVector goalPose, double attractiveGain, double rotationalAttractiveGain) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    goalPose(goalPose),
    maxVelocity(1.0) {}

  PotentialField(double attractiveGain, double rotationalAttractiveGain, double maxVelocity) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())),
    maxVelocity(maxVelocity) {}

  /**
 * @brief Updates the goal position (3D Vector) in the potential field.
 *
 * @note The Goal Position creates an attractive force towards it
 *
 * @param newGoalPose The new goal position to be set.
 */
  void updateGoalPosition(SpatialVector newGoalPose) { this->goalPose = newGoalPose; }

  /**
   * @brief Updates the attractive gain, scaling the force
   *        applied pulling points towards the goal.
   *
   * @param newAttractiveGain The new attractive gain to be set.
   */
  void updateAttractiveGain(double newAttractiveGain) { this->attractiveGain = newAttractiveGain; }

  /**
   * @brief Updates the maximum velocity the potential field is allowed to express
   *
   * @param newMaxVelocity The new max velocity [m/s]
   */
  void updateMaxVelocity(double newMaxVelocity) { this->maxVelocity = newMaxVelocity; }

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
   * @param obstacleFrameID  the frame ID of the obstacle obtained on obstacle creation.
   * @return true if the obstacle was removed successfully.
   */
  bool removeObstacle(const std::string& obstacleFrameID);

  /**
   * @brief Clears all obstacles from the potential field.
   */
  void clearObstacles();

  PotentialFieldObstacle getObstacleByID(const std::string& obstacleFrameID) const;

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
   * @brief Given a current pose and a delta time, computes the next pose after the time step.
   *
   * @note This function uses both integrateLinearVelocity and integrateAngularVelocity and
   *       combines their results to compute the next pose.
   *
   * @param currentPose The starting pose as a SpatialVector.
   * @param deltaTime The time step over which to compute the next pose [s].
   * @return SpatialVector The resulting pose after applying the velocity field for the time step.
   */
  SpatialVector interpolateNextPose(const SpatialVector& currentPose, const double deltaTime);

  /**
   * @brief Given a current orientation and a delta time, computes the angular velocity
   *        to compute the new orientation after the time step.
   *
   * @param currentOrientation The starting orientation as a quaternion.
   * @param angularVelocity The instantaneous angular velocity vector in radians per second.
   * @param deltaTime The time step over which to integrate the angular velocity [s].
   * @return Eigen::Quaterniond The resulting orientation after integrating the angular velocity.
   */
  Eigen::Quaterniond integrateAngularVelocity(
    const Eigen::Quaterniond& currentOrientation,
    const Eigen::Vector3d& angularVelocity,
    double deltaTime);

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

  Eigen::Vector3d computeRotationalVelocity(const SpatialVector& queryPose) const;

  SpatialVector getGoalPose() const;
  std::vector<PotentialFieldObstacle> getObstacles() const;

  bool isPointInsideObstacle(Eigen::Vector3d point) const;

  bool isPointWithinInfluenceZone(Eigen::Vector3d point) const;

  bool operator==(const PotentialField& other) const {
    auto obstaclesEqual = [this, &other]() -> bool {
      if (this->obstacles.size() != other.obstacles.size()) return false;
      // Convert both obstacle lists to unordered_sets for comparison
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> thisObstaclesSet(
        this->obstacles.cbegin(), this->obstacles.cend()
      );
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> otherObstaclesSet(
        other.obstacles.cbegin(), other.obstacles.cend()
      );
      return thisObstaclesSet == otherObstaclesSet;
    }();
    return this->attractiveGain == other.attractiveGain &&
      this->rotationalAttractiveGain == other.rotationalAttractiveGain &&
      this->goalPose == other.goalPose &&
      obstaclesEqual;
  }

  bool operator!=(const PotentialField& other) const { return !(*this == other); }

private:
  double attractiveGain; // Gain for attractive force
  double rotationalAttractiveGain; // Gain for rotational attractive force
  double translationalTolerance = 1e-3; // Threshold for distances to the goal and obstacles
  double rotationalThreshold = 0.06; // Threshold for rotational geodesic distance
  SpatialVector goalPose; // Current GoalPose
  std::vector<PotentialFieldObstacle> obstacles; // Obstacle list
  // Fast lookup for obstacle updates/removals by ID
  std::unordered_map<std::string, size_t> obstacleIndex;
  double maxVelocity; // Max linear velocity [m/s]

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
