/**
 * @file spatial_vector.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief A simple vector class for representing a "spatial vector"
 *        in 3D space consisting of both position and orientation (pose).
 * @version 1.0
 * @date 2025-05-14
 *
 * TODO: Utilize Eigen/Armadillo for vector operations.
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef SPATIAL_VECTOR_HPP
#define SPATIAL_VECTOR_HPP
#include <cmath>
#include <stdexcept>

struct SpatialVector {
  float x; // X-component of vector
  float y; // Y-component of vector
  float z; // Z-component of vector

  float euclideanDistance(const SpatialVector& other) const {
    return std::hypot(
      x - other.x, y - other.y, z - other.z
    );
  }

  SpatialVector operator-() const {
    return SpatialVector{-x, -y, -z};
  }

  SpatialVector operator+(const SpatialVector& other) const {
    return SpatialVector{x + other.x, y + other.y, z + other.z};
  }

  SpatialVector operator+=(const SpatialVector& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  SpatialVector operator-(const SpatialVector& other) const {
    return SpatialVector{x - other.x, y - other.y, z - other.z};
  }

  SpatialVector operator-=(const SpatialVector& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  SpatialVector operator*(float scalar) const {
    return SpatialVector{x * scalar, y * scalar, z * scalar};
  }

  SpatialVector operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  SpatialVector operator/(float scalar) const {
    if (scalar != 0) {
      return SpatialVector{x / scalar, y / scalar, z / scalar};
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  SpatialVector operator/=(float scalar) {
    if (scalar != 0) {
      x /= scalar;
      y /= scalar;
      z /= scalar;
      return *this;
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  bool operator==(const SpatialVector& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }

  bool operator!=(const SpatialVector& other) const {
    return !(*this == other);
  }
};

#endif // SPATIAL_VECTOR_HPP