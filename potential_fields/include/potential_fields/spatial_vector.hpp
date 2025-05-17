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
#include <eigen3/Eigen/Dense>

class SpatialVector {
public:
  // Constructors and Destructor

  SpatialVector() :
    position(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()) {
  }

  SpatialVector(const Eigen::Vector3d& position) :
    position(position),
    orientation(Eigen::Quaterniond::Identity()) {
  }

  SpatialVector(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) :
    position(position),
    orientation(orientation) {
  }

  ~SpatialVector() = default;

public:
  // Getters and Setters

  const Eigen::Vector3d& getPosition() const { return this->position; }
  const Eigen::Quaterniond& getOrientation() const { return this->orientation; }
  Eigen::Vector3d getOrientationEuler() const {
    // Returns the Euler angles (roll, pitch, yaw) from the quaternion
    return this->orientation.toRotationMatrix().eulerAngles(0, 1, 2);
  }

  void setPosition(const Eigen::Vector3d& position) { this->position = position; }
  void setOrientation(const Eigen::Quaterniond& orientation) { this->orientation = orientation; }
  void setOrientationEuler(double roll, double pitch, double yaw) {
    this->orientation = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  }

public:
  // Vector Operations

  double euclideanDistance(const SpatialVector& other) const {
    return (this->position - other.position).norm();
  }

  void normalizePosition() {
    // Normalize the position vector
    this->position.normalize();
  }

  double geodesicDistance(const SpatialVector& other) {
    // Compute the geodesic distance between two quaternions
    Eigen::Quaterniond q1 = this->orientation;
    Eigen::Quaterniond q2 = other.orientation;
    // Normalize the quaternions
    q1.normalize();
    q2.normalize();
    // Compute the quaternion difference
    Eigen::Quaterniond q_diff = q1.inverse() * q2;
    // Compute the angle of rotation
    double angle = 2.0f * std::acos(q_diff.w());
    // Wrap to [0, pi]
    if (angle > M_PI) {
      angle -= (2.0f * M_PI);
      angle = std::abs(angle);
      q_diff.x() = -q_diff.x();
      q_diff.y() = -q_diff.y();
      q_diff.z() = -q_diff.z();
      q_diff.w() = -q_diff.w();
    }
    return angle;
  }

public:
  // Operator Overloads
  bool operator==(const SpatialVector& other) const {
    return  this->position == other.position && this->orientation == other.orientation;
  }

  bool operator!=(const SpatialVector& other) const {
    return !(*this == other);
  }

private:
  // Position Vector
  Eigen::Vector3d position;

  // Orientation Vector
  Eigen::Quaterniond orientation;
};

#endif // SPATIAL_VECTOR_HPP
