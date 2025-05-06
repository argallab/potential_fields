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
    return std::sqrt(
      std::pow(x - other.x, 2) +
      std::pow(y - other.y, 2) +
      std::pow(z - other.z, 2)
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

  bool updateGoalPosition(Vector newGoalPosition);
  void updateAttractiveGain(float newAttractiveGain);
  void addObstacle(SphereObstacle obstacle);
  bool removeObstacle(int obstacleID);
  void clearObstacles();

  Vector computeVelocityAtPosition(Vector position);

private:
  float attractiveGain = 1.0f; // Gain for attractive force
  Vector goalPosition;
  std::vector<SphereObstacle> obstacles;

  Vector computeAttractiveForces(Vector position);
  Vector computeRepulsiveForces(Vector position);
};

#endif // PFIELDS_HPP