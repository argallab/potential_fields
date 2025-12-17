// Copyright 2025 Sharwin Patil
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
