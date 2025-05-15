/**
 * @file spatial_vector.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief A simple vector class for representing a "spatial vector"
 *        in 3D space consisting of both position and orientation (pose).
 * @version 1.0
 * @date 2025-05-14
 *
 * TODO: Utilize Eigen/Armadillo for vector operations.
 * TODO: Split functionality into a PositionVector and OrientationVector and pack both into SpatialVector
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

  void getEulerAngles(float& roll, float& pitch, float& yaw) const {
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (this->qw * this->qy - this->qz * this->qx);
    if (std::abs(sinp) >= 1.0f) {
      pitch = std::copysign(M_PI / 2.0f, sinp);  // clamp to pi/2
    } else {
      pitch = std::asin(sinp);
    }
    // Roll (x-axis rotation)
    roll = std::atan2(2.0f * (this->qw * this->qx + this->qy * this->qz),
      1.0f - 2.0f * (this->qx * this->qx + this->qy * this->qy));
    // Yaw (z-axis rotation)
    yaw = std::atan2(2.0f * (this->qw * this->qz + this->qx * this->qy),
      1.0f - 2.0f * (this->qy * this->qy + this->qz * this->qz));
  }


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

  void normalizePosition() {
    float magnitude = std::hypot(this->x, this->y, this->z);
    if (magnitude > 0) {
      this->x /= magnitude;
      this->y /= magnitude;
      this->z /= magnitude;
    } else {
      throw std::invalid_argument("Cannot normalize a zero vector");
    }
  }

  void normalizeQuaternion() {
    float qMagnitude = std::sqrt(this->qx * this->qx + this->qy * this->qy + this->qz * this->qz + this->qw * this->qw);
    if (qMagnitude > 0) {
      this->qx /= qMagnitude;
      this->qy /= qMagnitude;
      this->qz /= qMagnitude;
      this->qw /= qMagnitude;
    } else {
      throw std::invalid_argument("Cannot normalize a zero quaternion");
    }
  }

  float geodesicDistance(const SpatialVector& other) {
    // Compute the geodesic distance between two quaternions
    // self.diff_quat = tfs.quaternion_multiply(tfs.quaternion_inverse(self.eef_quat), self.goal_quat)
    //   self.diff_quat = self.diff_quat / np.linalg.norm(self.diff_quat)  # normalize
    this->inverseQuaternion();
    this->quaternionMultiply(other);
    // float dot = this->qx * other.getQX() +
    //   this->qy * other.getQY() + this->qz * other.getQZ() + this->qw * other.getQW();
    // // Normalize the quaternion difference
    // float norm = std::sqrt(
    //   this->qx * this->qx + this->qy * this->qy +
    //   this->qz * this->qz + this->qw * this->qw);
    // if (norm == 0) {
    //   throw std::invalid_argument("Cannot normalize a zero quaternion");
    // }
    // this->qx /= norm;
    // this->qy /= norm;
    // this->qz /= norm;
    // this->qw /= norm;
    // // Clamp the dot product to avoid NaN from acos
    // dot = std::clamp(dot, -1.0f, 1.0f);
    float theta = 2.0f * acos(this->qw); // [0, 2pi]
    // Wrap to [0, pi]
    if (theta > M_PI) {
      // if self.theta_to_goal > math.pi:  # wrap angle
      //   self.theta_to_goal -= 2 * math.pi
      //   self.theta_to_goal = abs(self.theta_to_goal)
      //   self.diff_quat = -self.diff_quat
      theta -= (2.0f * M_PI);
      theta = std::abs(theta);
      this->qx = -this->qx;
      this->qy = -this->qy;
      this->qz = -this->qz;
      this->qw = -this->qw;
    }
    return theta;
  }

  SpatialVector quaternionDifference(const SpatialVector& other) {
    // Get the quaternion representing the difference
    // between two quaternions using geodesic distance
    this->inverseQuaternion();
    // Multiply this quaternion with the other quaternion
    this->quaternionMultiply(other);
    this->geodesicDistance(other);
    // float qx = this->qw * other.getQX() + this->qx * other.getQW() + this->qy * other.getQZ() - this->qz * other.getQY();
    // float qy = this->qw * other.getQY() + this->qy * other.getQW() + this->qz * other.getQX() - this->qx * other.getQZ();
    // float qz = this->qw * other.getQZ() + this->qz * other.getQW() + this->qx * other.getQY() - this->qy * other.getQX();
    // float qw = this->qw * other.getQW() - this->qx * other.getQX() - this->qy * other.getQY() - this->qz * other.getQZ();
    float norm_denominator = std::sqrt(1 - this->qw * this->qw);
    if (norm_denominator >= 1e-3) {
      this->qx /= norm_denominator;
      this->qy /= norm_denominator;
      this->qz /= norm_denominator;
    }
    return SpatialVector{
      this->x, this->y, this->z,
      this->qx, this->qy, this->qz, this->qw
    };
  }

public:
  // Operator Overloads

  SpatialVector operator-() const {
    return SpatialVector{-this->x, -this->y, -this->z};
  }

  SpatialVector operator+(const SpatialVector& other) const {
    if (other.qx == 0 && other.qy == 0 && other.qz == 0 && other.qw == 1) {
      return SpatialVector{
        this->x + other.x, this->y + other.y, this->z + other.z,
        this->qx, this->qy, this->qz, this->qw
      };
    } else if (this->qx == 0 && this->qy == 0 && this->qz == 0 && this->qw == 1) {
      return SpatialVector{
        this->x + other.x, this->y + other.y, this->z + other.z,
        other.qx, other.qy, other.qz, other.qw
      };
    } else { // Both vectors have orientation (for now just pick this one)
      return SpatialVector{
        this->x + other.x, this->y + other.y, this->z + other.z,
        this->qx, this->qy, this->qz, this->qw
      };
    }

  }

  SpatialVector operator+=(const SpatialVector& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
    return *this;
  }

  SpatialVector operator-(const SpatialVector& other) const {
    if (other.qx == 0 && other.qy == 0 && other.qz == 0 && other.qw == 1) {
      return SpatialVector{
        this->x - other.x, this->y - other.y, this->z - other.z,
        this->qx, this->qy, this->qz, this->qw
      };
    } else if (this->qx == 0 && this->qy == 0 && this->qz == 0 && this->qw == 1) {
      return SpatialVector{
        this->x - other.x, this->y - other.y, this->z - other.z,
        other.qx, other.qy, other.qz, other.qw
      };
    } else { // Both vectors have orientation (for now just pick this one)
      return SpatialVector{
        this->x - other.x, this->y - other.y, this->z - other.z,
        other.qx, other.qy, other.qz, other.qw
      };
    }
  }

  SpatialVector operator-=(const SpatialVector& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
    return *this;
  }

  SpatialVector operator*(const SpatialVector& other) {
    this->quaternionMultiply(other);
    return SpatialVector{
      this->x * other.x, this->y * other.y, this->z * other.z,
      this->qx, this->qy, this->qz, this->qw
    };
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

  void conjugate() {
    this->qx = -this->qx;
    this->qy = -this->qy;
    this->qz = -this->qz;
    this->qw = this->qw;
  }

  void inverseQuaternion() {
    this->conjugate();
    float dot = this->qx * this->qx + this->qy * this->qy +
      this->qz * this->qz + this->qw * this->qw;
    if (dot != 0) {
      this->qx /= dot;
      this->qy /= dot;
      this->qz /= dot;
      this->qw /= dot;
    } else {
      throw std::invalid_argument("Cannot normalize a zero quaternion");
    }
  }

  void quaternionMultiply(const SpatialVector& other) {
    this->qx = this->qw * other.getQX() + this->qx * other.getQW() + this->qy * other.getQZ() - this->qz * other.getQY();
    this->qy = this->qw * other.getQY() + this->qy * other.getQW() + this->qz * other.getQX() - this->qx * other.getQZ();
    this->qz = this->qw * other.getQZ() + this->qz * other.getQW() + this->qx * other.getQY() - this->qy * other.getQX();
    this->qw = this->qw * other.getQW() - this->qx * other.getQX() - this->qy * other.getQY() - this->qz * other.getQZ();
    // Normalize the quaternion
    float norm = std::sqrt(
      this->qx * this->qx + this->qy * this->qy +
      this->qz * this->qz + this->qw * this->qw);
    if (norm != 0) {
      this->qx /= norm;
      this->qy /= norm;
      this->qz /= norm;
      this->qw /= norm;
    } else {
      throw std::invalid_argument("Cannot normalize a zero quaternion");
    }
  }
};

#endif // SPATIAL_VECTOR_HPP
