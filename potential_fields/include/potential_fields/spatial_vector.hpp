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
#include <algorithm>

class SpatialVector {
public:
  // Constructors and Destructor
  SpatialVector()
    :x(0.0f), y(0.0f), z(0.0f),
    qx(0.0f), qy(0.0f), qz(0.0f), qw(1.0f) {
  }

  SpatialVector(float x, float y, float z)
    : x(x), y(y), z(z), qx(0.0f), qy(0.0f), qz(0.0f), qw(1.0f) {
  }

  SpatialVector(
    float x, float y, float z,
    float qx, float qy, float qz, float qw)
    : x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) {
  }

  ~SpatialVector() = default;

public:
  // Getters and Setters

  float getX() const { return this->x; }
  float getY() const { return this->y; }
  float getZ() const { return this->z; }
  float getQX() const { return this->qx; }
  float getQY() const { return this->qy; }
  float getQZ() const { return this->qz; }
  float getQW() const { return this->qw; }

  void setOrientationEuler(float roll, float pitch, float yaw) {
    // Convert Euler angles to quaternion
    euler2Quaternion(roll, pitch, yaw, this->qx, this->qy, this->qz, this->qw);
  }

  void setOrientationQuaternion(float qx, float qy, float qz, float qw) {
    this->qx = qx;
    this->qy = qy;
    this->qz = qz;
    this->qw = qw;
  }

public:
  // Vector Operations

  float euclideanDistance(const SpatialVector& other) const {
    return std::hypot(
      this->x - other.x, this->y - other.y, this->z - other.z
    );
  }

  void normalize() {
    float magnitude = std::hypot(this->x, this->y, this->z);
    if (magnitude > 0) {
      this->x /= magnitude;
      this->y /= magnitude;
      this->z /= magnitude;
    } else {
      throw std::invalid_argument("Cannot normalize a zero vector");
    }
  }

  float geodesicDistance(const SpatialVector& other) const {
    // Compute the geodesic distance between two quaternions
    float dot = this->qx * other.qx + this->qy * other.qy +
      this->qz * other.qz + this->qw * other.qw;
    return 2.0f * acos(std::abs(dot));
  }

  SpatialVector quaternionDifference(const SpatialVector& other) const {
    // Get the quaternion representing the difference
    // between two quaternions using geodesic distance
    // 1. Invert this quaternion
    float qx_inv = -this->qx;
    float qy_inv = -this->qy;
    float qz_inv = -this->qz;
    float qw_inv = this->qw;
    // 2. Multiply other quaternion with the inverted quaternion
    float qx_diff = other.qw * qx_inv + other.qx * qw_inv + other.qy * qz_inv - other.qz * qy_inv;
    float qy_diff = other.qw * qy_inv - other.qx * qz_inv + other.qy * qw_inv + other.qz * qx_inv;
    float qz_diff = other.qw * qz_inv + other.qx * qy_inv - other.qy * qx_inv + other.qz * qw_inv;
    float qw_diff = other.qw * qw_inv - other.qx * qx_inv - other.qy * qy_inv - other.qz * qz_inv;
    // 3. Normalize the resulting quaternion
    float magnitude = std::sqrt(qx_diff * qx_diff + qy_diff * qy_diff + qz_diff * qz_diff +
      qw_diff * qw_diff);
    if (magnitude > 0) {
      qx_diff /= magnitude;
      qy_diff /= magnitude;
      qz_diff /= magnitude;
      qw_diff /= magnitude;
    } else {
      throw std::invalid_argument("Cannot normalize a zero quaternion");
    }
    // 4. Compute the angle of rotation
    float theta = 2.0f * acos(std::clamp(qw_diff, -1.0f, 1.0f));
    // 5. Wrap angle to [0, pi]
    if (theta > M_PI) {
      theta = (2.0f * M_PI) - theta;
      qx_diff = -qx_diff;
      qy_diff = -qy_diff;
      qz_diff = -qz_diff;
    }
    // 6. Avoid division by zero
    float denom = std::sqrt(1.0f - qw_diff * qw_diff);
    // If angle is small, direction is not well-defined so
    if (denom > 1e-6) {
      qx_diff /= denom;
      qy_diff /= denom;
      qz_diff /= denom;
    }
    return SpatialVector{this->x, this->y, this->z, qx_diff, qy_diff, qz_diff, qw_diff};
  }

public:
  // Operator Overloads

  SpatialVector operator-() const {
    return SpatialVector{-this->x, -this->y, -this->z};
  }

  SpatialVector operator+(const SpatialVector& other) const {
    return SpatialVector{this->x + other.x, this->y + other.y, this->z + other.z};
  }

  SpatialVector operator+=(const SpatialVector& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }

  SpatialVector operator-(const SpatialVector& other) const {
    return SpatialVector{
      this->x - other.x, this->y - other.y, this->z - other.z,
      other.qx, other.qy, other.qz, other.qw
    };
  }

  SpatialVector operator-=(const SpatialVector& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    return *this;
  }

  SpatialVector operator*(float scalar) const {
    return SpatialVector{this->x * scalar, this->y * scalar, this->z * scalar};
  }

  SpatialVector operator*=(float scalar) {
    this->x *= scalar;
    this->y *= scalar;
    this->z *= scalar;
    return *this;
  }

  SpatialVector operator/(float scalar) const {
    if (scalar != 0) {
      return SpatialVector{this->x / scalar, this->y / scalar, this->z / scalar};
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  SpatialVector operator/=(float scalar) {
    if (scalar != 0) {
      this->x /= scalar;
      this->y /= scalar;
      this->z /= scalar;
      return *this;
    } else {
      throw std::invalid_argument("Division by zero");
    }
  }

  bool operator==(const SpatialVector& other) const {
    return  this->x == other.x && this->y == other.y && this->z == other.z;
  }

  bool operator!=(const SpatialVector& other) const {
    return !(*this == other);
  }

private:
  // Private Members
// Position components
  float x; // X-component of vector
  float y; // Y-component of vector
  float z; // Z-component of vector
  // Orientation components (quaternion)
  float qx; // X-component of quaternion
  float qy; // Y-component of quaternion
  float qz; // Z-component of quaternion
  float qw; // W-component of quaternion

private:
  // Private Methods
  void euler2Quaternion(
    float roll, float pitch, float yaw,
    float& qx, float& qy, float& qz, float& qw) {
    // Convert Euler angles to quaternion
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    qw = cy * cp * cr + sy * sp * sr;
    qx = cy * cp * sr - sy * sp * cr;
    qy = sy * cp * cr + cy * sp * sr;
    qz = sy * cp * sr - cy * sp * cr;
  }
};

#endif // SPATIAL_VECTOR_HPP
