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

#include "pfield/pf_obstacle.hpp"
#include "pfield/pf_obstacle_geometry.hpp"
#include "pfield/mesh_collision.hpp"
#include "pfield/pfield_common.hpp"

#include <coal/narrowphase/narrowphase.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

namespace pfield {

  bool PotentialFieldObstacle::withinInfluenceZone(
    Eigen::Vector3d worldPoint, double influenceDistance) const {
    double signedDistance;
    Eigen::Vector3d normal;  // unused
    this->computeSignedDistanceAndNormal(worldPoint, signedDistance, normal);
    return signedDistance <= influenceDistance;
  }

  bool PotentialFieldObstacle::withinObstacle(Eigen::Vector3d worldPoint) const {
    double signedDistance;
    Eigen::Vector3d normal;  // unused
    this->computeSignedDistanceAndNormal(worldPoint, signedDistance, normal);
    return signedDistance <= 0.0;
  }

  Eigen::Vector3d PotentialFieldObstacle::toObstacleFrame(const Eigen::Vector3d& worldPoint) const {
    return this->orientationConjugate * (worldPoint - this->position);
  }

  Eigen::Vector3d PotentialFieldObstacle::halfDimensions() const {
    return this->geometry->halfDimensions(this->meshScale);
  }

  void PotentialFieldObstacle::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& worldPoint, double& signedDistance, Eigen::Vector3d& normal) const {
    const Eigen::Vector3d localPoint = this->toObstacleFrame(worldPoint);
    this->geometry->computeSignedDistanceAndNormal(
      localPoint, this->orientation, signedDistance, normal);
  }

  std::shared_ptr<coal::CollisionObject> PotentialFieldObstacle::toCoalCollisionObject() {
    if (this->coalCollisionObject) { return this->coalCollisionObject; }

    coal::Transform3s transform;
    this->coalCollisionGeometry = this->geometry->buildCoalGeometry(
      this->position, this->orientation, transform);

    this->coalCollisionObject = std::make_shared<coal::CollisionObject>(
      this->coalCollisionGeometry, transform);
    return this->coalCollisionObject;
  }

  void PotentialFieldObstacle::updateCoalCollisionObjectPose() const {
    if (!this->coalCollisionObject) { return; }
    const coal::Transform3s transform =
      this->geometry->computeUpdatedCoalTransform(this->position, this->orientation);
    this->coalCollisionObject->setTransform(transform);
    this->coalCollisionObject->computeAABB();
  }

}  // namespace pfield
