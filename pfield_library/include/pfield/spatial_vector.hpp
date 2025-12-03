/**
 * @file spatial_vector.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief A simple vector class for representing a "spatial vector"
 *        in 3D space consisting of both position and orientation (pose).
 * @version 1.0
 * @date 2025-05-14
 *
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef SPATIAL_VECTOR_HPP
#define SPATIAL_VECTOR_HPP

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace pfield {

class SpatialVector {
public:
  SpatialVector() :
    position(Eigen::Vector3d::Zero()),
    orientation(Eigen::Quaterniond::Identity()) {}

  explicit SpatialVector(const Eigen::Vector3d& position) :
    position(position),
    orientation(Eigen::Quaterniond::Identity()) {}

  SpatialVector(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) :
    position(position),
    orientation(orientation) {}

  ~SpatialVector() = default;

  const Eigen::Vector3d& getPosition() const { return this->position; }
  const Eigen::Quaterniond& getOrientation() const { return this->orientation; }

  /**
   * @brief Returns the internal quaternion converted to euler angles [yaw, pitch, roll]
   *
   * @return Eigen::Vector3d The euler angles representing the orientation as [yaw, pitch, roll]
   */
  Eigen::Vector3d getOrientationEuler() const;

  void setPosition(const Eigen::Vector3d& position) { this->position = position; }

  /**
   * @brief Set the orientation of the spatial vector using an Eigen::Quaterniond
   *
   * @note The quaternion is normalized upon setting to maintain validity.
   *
   * @param orientation The Eigen::Quaterniond representing the orientation
   */
  void setOrientation(const Eigen::Quaterniond& orientation);

  /**
   * @brief Sets the internal quaternion rotation given an euler angle representation
   *
   * @note That euler angles are represented with the intrinsic order [Z→Y→X] = [yaw, pitch, roll]
   *
   * @param yaw The rotation around the Z-axis [rad]
   * @param pitch The rotation around the Y-axis [rad]
   * @param roll The rotation around the X-axis [rad]
   */
  void setOrientationEuler(double yaw, double pitch, double roll);

  void normalizePosition();

  /**
   * @brief Computes the euclidean distance between this spatial vector's position
   *        and the other spatial vector's position.
   *
   * @param other the other spatial vector to compare its position
   * @return double  the euclidean distance in the same units as the position vector [m]
   */
  double euclideanDistance(const SpatialVector& other) const;

  /**
   * @brief Returns the geodesic (angular) distance between this spatial vector's orientation
   *        and the other spatial vector's orientation.
   *
   * @note The distance is in radians and bounded by [0, pi]
   *
   * @param other the other spatial vector to compare its orientation
   * @return double the geodesic distance [rad]
   */
  double angularDistance(const SpatialVector& other) const;

  bool operator==(const SpatialVector& other) const {
    return  this->position == other.position && this->orientation == other.orientation;
  }
  bool operator!=(const SpatialVector& other) const { return !(*this == other); }

private:
  // Position Vector
  Eigen::Vector3d position;

  // Orientation Vector
  Eigen::Quaterniond orientation;
};

} // namespace pfield

#endif // SPATIAL_VECTOR_HPP
