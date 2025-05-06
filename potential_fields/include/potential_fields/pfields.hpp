/**
 * @file potential_field.hpp
 * @author Sharwin Patil (you@domain.com)
 * @brief Library for managing a vector field overlaid onto a
 *        robot's task-space (3D World) as a velocity field
 *        to plan motion trajectories.
 * @version 1.0
 * @date 2025-05-08
 *
 * TODO: Utilize Eigen for vector operations.
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef PFIELDS_HPP
#define PFIELDS_HPP
#include <vector>
#include <stdexcept>
#include <math.h>

 // Functionality to support:
 // 1. Track a Goal Position (Initialization and Update)
 // 2. Create obstacles with a specified geometry
 // 3. Compute velocity vector at a given position

struct Vector {
  float x; // X-component of vector [m]
  float y; // Y-component of vector [m]
  float z; // Z-component of vector [m]

  float euclideanDistance(const Vector& other) const {
    return std::hypot(
      x - other.x, y - other.y, z - other.z
    );
  }

  Vector operator-() const {
    return Vector{-x, -y, -z};
  }

  Vector operator+(const Vector& other) const {
    return Vector{x + other.x, y + other.y, z + other.z};
  }

  Vector operator+=(const Vector& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Vector operator-(const Vector& other) const {
    return Vector{x - other.x, y - other.y, z - other.z};
  }

  Vector operator-=(const Vector& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  Vector operator*(float scalar) const {
    return Vector{x * scalar, y * scalar, z * scalar};
  }

  Vector operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  Vector operator/(float scalar) const {
    if (scalar != 0) {
      return Vector{x / scalar, y / scalar, z / scalar};
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  Vector operator/=(float scalar) {
    if (scalar != 0) {
      x /= scalar;
      y /= scalar;
      z /= scalar;
      return *this;
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  bool operator==(const Vector& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator!=(const Vector& other) const {
    return !(*this == other);
  }
};

class SphereObstacle {
public:
  SphereObstacle() = default;
  SphereObstacle(int id, Vector position, float radius, float influenceRadius, float repulsiveGain)
    : id(id),
    position(position),
    radius(radius),
    influenceRadius(influenceRadius),
    repulsiveGain(repulsiveGain) {
  }
  ~SphereObstacle() = default;

  Vector getPosition() const { return position; }
  float getRadius() const { return radius; }
  int getID() const { return id; }
  float getInfluenceRadius() const { return influenceRadius; }
  float getRepulsiveGain() const { return repulsiveGain; }

  bool withinInfluenceRadius(Vector pos) const {
    return (this->position.euclideanDistance(pos) <= influenceRadius);
  }

  bool operator==(const SphereObstacle& other) const {
    return (position == other.position && radius == other.radius);
  }
  bool operator!=(const SphereObstacle& other) const {
    return !(*this == other);
  }

private:
  int id = 0; // Unique ID for the obstacle
  Vector position; // Center Position of the obstacle in 3D space
  float radius; // Sphere's radius [m]
  float influenceRadius; // Sphere's influence radius [m]
  float repulsiveGain = 1.0f; // Gain for repulsive force
};

class PotentialField {
public:
  PotentialField() = default;
  PotentialField(Vector goalPosition, float attractiveGain)
    : goalPosition(goalPosition),
    attractiveGain(attractiveGain) {
  }
  ~PotentialField() = default;

  /**
 * @brief Updates the goal position (3D Vector) in the potential field.
 *
 * @note The Goal Position creates an attractive force towards it
 *
 * @param newGoalPosition The new goal position to be set.
 */
  void updateGoalPosition(Vector newGoalPosition);

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
  Vector computeVelocityAtPosition(Vector position);

private:
  float attractiveGain = 1.0f; // Gain for attractive force
  Vector goalPosition;
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
  Vector computeAttractiveForces(Vector position);

  /**
   * @brief Computes the repulsive forces from all obstacles
   *        and returns the resultant vector.
   *
   * @note Equation: F = repulsiveGain * (1 / distance^2) * direction
   *       where direction is a unit vector pointing away from the obstacle.
   *
   * @param position The position in 3D space to compute the force from.
   * @return Vector The repulsive force vector.
   */
  Vector computeRepulsiveForces(Vector position);
};

#endif // PFIELDS_HPP