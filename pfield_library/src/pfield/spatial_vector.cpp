#include "pfield/spatial_vector.hpp"

namespace pfield {

  Eigen::Vector3d SpatialVector::getOrientationEuler() const {
    // Returns the Euler angles (yaw, pitch, roll) from the quaternion
    return this->orientation.toRotationMatrix().eulerAngles(2, 1, 0);
  }

  void SpatialVector::setOrientation(const Eigen::Quaterniond& orientation) {
    this->orientation = orientation;
    this->orientation.normalize();
  }

  void SpatialVector::setOrientationEuler(double yaw, double pitch, double roll) {
    this->orientation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    this->orientation.normalize();
  }

  void SpatialVector::normalizePosition() { this->position.normalize(); }

  double SpatialVector::euclideanDistance(const SpatialVector& other) const {
    return (this->position - other.position).norm();
  }

  double SpatialVector::angularDistance(const SpatialVector& other) const {
    return this->orientation.angularDistance(other.getOrientation());
  }

} // namespace pfield
