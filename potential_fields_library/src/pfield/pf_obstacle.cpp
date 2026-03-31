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
#include "pfield/mesh_collision.hpp"
#include "pfield/pfield_common.hpp"

#include <coal/narrowphase/narrowphase.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

namespace pfield {

  bool PotentialFieldObstacle::withinInfluenceZone(Eigen::Vector3d worldPoint, double influenceDistance) const {
    double signedDistance;
    Eigen::Vector3d normal; // unused
    this->computeSignedDistanceAndNormal(worldPoint, signedDistance, normal);
    return signedDistance <= influenceDistance;
  }

  bool PotentialFieldObstacle::withinObstacle(Eigen::Vector3d worldPoint) const {
    double signedDistance;
    Eigen::Vector3d normal; // unused
    this->computeSignedDistanceAndNormal(worldPoint, signedDistance, normal);
    return signedDistance <= 0.0;
  }

  Eigen::Vector3d PotentialFieldObstacle::toObstacleFrame(const Eigen::Vector3d& worldPoint) const {
    return this->orientationConjugate * (worldPoint - this->position);
  }

  Eigen::Vector3d PotentialFieldObstacle::halfDimensions() const {
    // Returns half the dimensions of the obstacle based on its geometry
    switch (this->type) {
    case ObstacleType::SPHERE:
      return Eigen::Vector3d(this->geometry.radius, this->geometry.radius, this->geometry.radius);
    case ObstacleType::BOX:
      return Eigen::Vector3d(this->geometry.length / 2.0, this->geometry.width / 2.0, this->geometry.height / 2.0);
    case ObstacleType::CYLINDER:
      return Eigen::Vector3d(this->geometry.radius, this->geometry.radius, this->geometry.height / 2.0);
    case ObstacleType::MESH:
      // If geometry carries bounding box dims, use them. Otherwise fall back to meshScale as dims.
      if (this->geometry.length > 0.0 && this->geometry.width > 0.0 && this->geometry.height > 0.0) {
        return Eigen::Vector3d(this->geometry.length / 2.0, this->geometry.width / 2.0, this->geometry.height / 2.0);
      }
      else {
        return Eigen::Vector3d(this->meshScale.x() / 2.0, this->meshScale.y() / 2.0, this->meshScale.z() / 2.0);
      }
    case ObstacleType::ELLIPSOID:
      return Eigen::Vector3d(this->geometry.semi_x, this->geometry.semi_y, this->geometry.semi_z);
    case ObstacleType::CAPSULE:
      // Bounding box of a capsule: radial extent = radius in X/Y, total half-length = height/2 + radius in Z
      return Eigen::Vector3d(
        this->geometry.radius,
        this->geometry.radius,
        this->geometry.height / 2.0 + this->geometry.radius
      );
    default:
      throw std::invalid_argument("Unknown obstacle type");
    }
  }

  void PotentialFieldObstacle::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& worldPoint, double& signedDistance, Eigen::Vector3d& normal) const {
    Eigen::Vector3d localPoint = this->toObstacleFrame(worldPoint);
    switch (this->type) {
    case ObstacleType::SPHERE: {
      double distance = localPoint.norm();
      signedDistance = distance - this->geometry.radius;
      // If the distance from localPoint to obstacle origin is near zero, set normal to arbitrary unit vector
      // and distance will be close to -radius
      normal = (distance > NEAR_ZERO_THRESHOLD) ?
        rotateVector(this->orientation, localPoint / distance) :
        rotateVector(this->orientation, Eigen::Vector3d::UnitX());
      break;
    }
    case ObstacleType::BOX: {
      // Transform into OBB principal-axis frame (centroid-relative, then PCA-rotated).
      // For ordinary axis-aligned boxes, ellipsoid_axes = Identity and centroid_offset = Zero,
      // so this reduces to the original behaviour.
      const Eigen::Vector3d pOBB = this->geometry.ellipsoid_axes.transpose()
        * (localPoint - this->geometry.centroid_offset);
      Eigen::Vector3d halfDims = this->halfDimensions();
      Eigen::Vector3d boundedLocalPoint = pOBB.cwiseMax(-halfDims).cwiseMin(halfDims);
      Eigen::Vector3d distanceToBoundedPoint = pOBB - boundedLocalPoint;
      const double distanceToSurface = distanceToBoundedPoint.norm();
      Eigen::Vector3d normalOBB;
      if (distanceToSurface > NEAR_ZERO_THRESHOLD) {
        // Outside box: normal points from closest point on box to pOBB
        normalOBB = distanceToBoundedPoint / distanceToSurface;
        signedDistance = distanceToSurface;
      }
      else {
        // Inside box: find closest face
        Eigen::Vector3d distancesToFaces = halfDims - pOBB.cwiseAbs();
        int axisIndex;
        distancesToFaces.minCoeff(&axisIndex);
        normalOBB = Eigen::Vector3d::Zero();
        normalOBB[axisIndex] = (pOBB[axisIndex] >= 0.0) ? 1.0 : -1.0;
        signedDistance = -distancesToFaces[axisIndex];
      }
      // Rotate normal: OBB frame → obstacle frame → world frame
      const Eigen::Vector3d normalObstacle = this->geometry.ellipsoid_axes * normalOBB;
      normal = rotateVector(this->orientation, normalObstacle);
      break;
    }
    case ObstacleType::CYLINDER: {
      // Get the XY radial distance of the point to the cylinder axis
      Eigen::Vector2d radialPoint(localPoint.x(), localPoint.y());
      const double radialDist = radialPoint.norm();
      const double halfHeight = this->geometry.height / 2.0;
      Eigen::Vector3d normalLocal = Eigen::Vector3d::Zero();
      // Check if inside the infinite cylinder along Z axis and within height bounds
      if (radialDist <= this->geometry.radius && std::abs(localPoint.z()) <= halfHeight) {
        // Inside: nearest feature between side surface, top, bottom
        double distanceToSide = this->geometry.radius - radialDist;
        double distanceToTopPlane = halfHeight - localPoint.z();
        double distanceToBottomPlane = halfHeight + localPoint.z();

        // Determine closest feature
        if (distanceToTopPlane < distanceToSide && distanceToTopPlane < distanceToBottomPlane) {
          // The top plane is the closest feature
          signedDistance = -distanceToTopPlane;
          normalLocal = Eigen::Vector3d::UnitZ();
        }
        else if (distanceToBottomPlane < distanceToSide && distanceToBottomPlane < distanceToTopPlane) {
          // The bottom plane is the closest feature
          signedDistance = -distanceToBottomPlane;
          normalLocal = -Eigen::Vector3d::UnitZ();
        }
        else {
          // The side surface is the closest feature
          signedDistance = -distanceToSide;
          normalLocal = (radialDist > NEAR_ZERO_THRESHOLD) ?
            Eigen::Vector3d(-localPoint.x() / radialDist, -localPoint.y() / radialDist, 0.0) :
            -Eigen::Vector3d::UnitX();
        }
      }
      else {
        // Outside
        // Compute closest point on cylinder surface
        Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();

        // Radial component
        if (radialDist > this->geometry.radius) {
          if (radialDist > NEAR_ZERO_THRESHOLD) {
            closestPoint.head<2>() = radialPoint / radialDist * this->geometry.radius;
          }
          else {
            closestPoint.head<2>() = Eigen::Vector2d(this->geometry.radius, 0); // Arbitrary
          }
        }
        else {
          closestPoint.head<2>() = radialPoint;
        }

        // Z component
        if (localPoint.z() > halfHeight) closestPoint.z() = halfHeight;
        else if (localPoint.z() < -halfHeight) closestPoint.z() = -halfHeight;
        else closestPoint.z() = localPoint.z();

        Eigen::Vector3d diff = localPoint - closestPoint;
        signedDistance = diff.norm();
        if (signedDistance > NEAR_ZERO_THRESHOLD) {
          normalLocal = diff / signedDistance;
        }
        else {
          normalLocal = Eigen::Vector3d::UnitX();
        }
      }

      // Rotate the output normal back to world frame
      normal = rotateVector(this->orientation, normalLocal);
      break;
    }
    case ObstacleType::ELLIPSOID: {
      const double a = this->geometry.semi_x;
      const double b = this->geometry.semi_y;
      const double c = this->geometry.semi_z;
      if (a < NEAR_ZERO_THRESHOLD || b < NEAR_ZERO_THRESHOLD || c < NEAR_ZERO_THRESHOLD) {
        throw std::runtime_error("Ellipsoid obstacle has zero or near-zero semi-axis");
      }
      // Shift to ellipsoid center (mesh centroid in obstacle-local frame) then rotate
      // into the PCA principal-axis frame so the SDF axes align with the ellipsoid axes.
      const Eigen::Vector3d pLocal = this->geometry.ellipsoid_axes.transpose()
        * (localPoint - this->geometry.centroid_offset);
      // Implicit function f(p) = (px/a)^2 + (py/b)^2 + (pz/c)^2 - 1
      // f < 0 => inside, f > 0 => outside
      const double f = (pLocal.x() * pLocal.x()) / (a * a)
        + (pLocal.y() * pLocal.y()) / (b * b)
        + (pLocal.z() * pLocal.z()) / (c * c)
        - 1.0;
      // Gradient of f in PCA frame (outward normal direction)
      const Eigen::Vector3d gradPCA(
        2.0 * pLocal.x() / (a * a),
        2.0 * pLocal.y() / (b * b),
        2.0 * pLocal.z() / (c * c)
      );
      const double gradNorm = gradPCA.norm();
      // First-order signed distance approximation: d ≈ f / ||∇f||
      // Smooth and exact on the surface (f=0), accurate near it
      if (gradNorm > NEAR_ZERO_THRESHOLD) {
        signedDistance = f / gradNorm;
        // Rotate gradient back: PCA frame → obstacle frame → world frame
        const Eigen::Vector3d gradObstacle = this->geometry.ellipsoid_axes * (gradPCA / gradNorm);
        normal = rotateVector(this->orientation, gradObstacle);
      }
      else {
        // Point is exactly at the ellipsoid center; set arbitrary outward normal
        signedDistance = -std::min({a, b, c});
        normal = rotateVector(this->orientation, Eigen::Vector3d::UnitX());
      }
      break;
    }
    case ObstacleType::CAPSULE: {
      // Capsule: cylinder of radius `radius` and shaft length `height`, with hemispherical endcaps.
      // The axis runs along the Z axis of the obstacle frame; endcap centers at ±height/2.
      // Like the OBB, capsules fitted from mesh use ellipsoid_axes (orientation) and centroid_offset.
      const Eigen::Vector3d pCap = this->geometry.ellipsoid_axes.transpose()
        * (localPoint - this->geometry.centroid_offset);
      const double halfShaft = this->geometry.height / 2.0;
      // Project onto the capsule axis (Z): clamp to the shaft segment
      const double zClamped = std::clamp(pCap.z(), -halfShaft, halfShaft);
      // Vector from the nearest point on the axis segment to the query point
      const Eigen::Vector3d toPoint(pCap.x(), pCap.y(), pCap.z() - zClamped);
      const double dist = toPoint.norm();
      signedDistance = dist - this->geometry.radius;
      Eigen::Vector3d normalCap;
      if (dist > NEAR_ZERO_THRESHOLD) {
        normalCap = toPoint / dist;
      }
      else {
        // Query point is on the capsule axis — push outward in X
        normalCap = Eigen::Vector3d::UnitX();
      }
      // Rotate back: capsule frame → obstacle frame → world frame
      normal = rotateVector(this->orientation, this->geometry.ellipsoid_axes * normalCap);
      break;
    }
    case ObstacleType::MESH: {
      if (!this->meshCollisionData) {
        throw std::runtime_error("Mesh collision data not available for obstacle");
      }
      // Transform point to mesh local frame and check inside and distance
      Eigen::Vector3d localPoint = this->toObstacleFrame(worldPoint);
      const bool inside = pointInsideMesh(*this->meshCollisionData, localPoint);
      double distanceToSurface = computeUnsignedDistanceToMesh(*this->meshCollisionData, localPoint);
      // Get closest point on the mesh to build normal
      Eigen::Vector3d closestPointOnMesh;
      const bool gotClosest = getClosestPointOnMesh(*this->meshCollisionData, localPoint, closestPointOnMesh);
      Eigen::Vector3d normalLocal = Eigen::Vector3d::UnitX(); // default normal
      if (gotClosest) {
        Eigen::Vector3d vecToClosest = localPoint - closestPointOnMesh;
        double distanceToClosestPoint = vecToClosest.norm();
        if (inside) {
          // If inside, outward normal should point from inside to outside
          // use opposite direction for vector to closest point
          vecToClosest = -vecToClosest;
          distanceToClosestPoint = vecToClosest.norm();
        }
        // If distance is positive and finite, assign normalLocal
        if (distanceToClosestPoint > NEAR_ZERO_THRESHOLD) { normalLocal = vecToClosest / distanceToClosestPoint; }
      }
      else {
        // Fallback: use AABB gradient direction
        const Eigen::Vector3d midpointAABB = 0.5 * (this->meshCollisionData->aabbMax + this->meshCollisionData->aabbMin);
        normalLocal = localPoint - midpointAABB;
        const double normalLocalNorm = normalLocal.norm();
        if (normalLocalNorm > NEAR_ZERO_THRESHOLD) { normalLocal /= normalLocalNorm; }
      }
      // Set signed distance based on inside/outside
      signedDistance = inside ? -std::abs(distanceToSurface) : std::abs(distanceToSurface);
      // Rotate the output normal back to world frame
      normal = rotateVector(this->orientation, normalLocal).normalized();
      break;
    }
    default:
      throw std::invalid_argument("Unknown obstacle type");
    }
  }

  std::shared_ptr<coal::CollisionObject> PotentialFieldObstacle::toCoalCollisionObject() {
    // First, check the cached COAL collision object
    if (this->coalCollisionObject) { return this->coalCollisionObject; }

    // Create COAL transform from obstacle pose
    coal::Transform3s transform = coal::Transform3s::Identity();

    // Set translation
    transform.setTranslation(coal::Vec3s(
      static_cast<coal::CoalScalar>(this->position.x()),
      static_cast<coal::CoalScalar>(this->position.y()),
      static_cast<coal::CoalScalar>(this->position.z())
    ));

    // Set rotation from quaternion (COAL uses w,x,y,z order)
    coal::Quatf quat(
      static_cast<coal::CoalScalar>(this->orientation.w()),
      static_cast<coal::CoalScalar>(this->orientation.x()),
      static_cast<coal::CoalScalar>(this->orientation.y()),
      static_cast<coal::CoalScalar>(this->orientation.z())
    );
    transform.setQuatRotation(quat);

    // Create appropriate COAL shape based on obstacle type
    std::shared_ptr<coal::CollisionGeometry> shape;

    switch (this->type) {
    case ObstacleType::SPHERE: {
      shape = std::make_shared<coal::Sphere>(
        static_cast<coal::CoalScalar>(this->geometry.radius)
      );
      break;
    }

    case ObstacleType::BOX: {
      shape = std::make_shared<coal::Box>(
        static_cast<coal::CoalScalar>(this->geometry.length),
        static_cast<coal::CoalScalar>(this->geometry.width),
        static_cast<coal::CoalScalar>(this->geometry.height)
      );
      // For OBB-fitted boxes (PCA from mesh), shift the Coal transform to the OBB center
      // and compose the OBB rotation with the obstacle orientation.
      if (this->geometry.centroid_offset.norm() > 1e-6 ||
          !this->geometry.ellipsoid_axes.isIdentity(1e-6)) {
        const Eigen::Vector3d worldCenter = this->position
          + this->orientation * this->geometry.centroid_offset;
        const Eigen::Quaterniond markerQ = this->orientation
          * Eigen::Quaterniond(this->geometry.ellipsoid_axes);
        transform.setTranslation(coal::Vec3s(
          static_cast<coal::CoalScalar>(worldCenter.x()),
          static_cast<coal::CoalScalar>(worldCenter.y()),
          static_cast<coal::CoalScalar>(worldCenter.z())
        ));
        coal::Quatf obbQuat(
          static_cast<coal::CoalScalar>(markerQ.w()),
          static_cast<coal::CoalScalar>(markerQ.x()),
          static_cast<coal::CoalScalar>(markerQ.y()),
          static_cast<coal::CoalScalar>(markerQ.z())
        );
        transform.setQuatRotation(obbQuat);
      }
      break;
    }

    case ObstacleType::CYLINDER: {
      shape = std::make_shared<coal::Cylinder>(
        static_cast<coal::CoalScalar>(this->geometry.radius),
        static_cast<coal::CoalScalar>(this->geometry.height)
      );
      break;
    }

    case ObstacleType::ELLIPSOID: {
      shape = std::make_shared<coal::Ellipsoid>(
        static_cast<coal::CoalScalar>(this->geometry.semi_x),
        static_cast<coal::CoalScalar>(this->geometry.semi_y),
        static_cast<coal::CoalScalar>(this->geometry.semi_z)
      );
      break;
    }

    case ObstacleType::CAPSULE: {
      // coal::Capsule(radius, lz): lz is the shaft length; endcaps add radius at each end.
      shape = std::make_shared<coal::Capsule>(
        static_cast<coal::CoalScalar>(this->geometry.radius),
        static_cast<coal::CoalScalar>(this->geometry.height)
      );
      // Apply OBB orientation and centroid offset (same pattern as OBB-fitted BOX)
      if (this->geometry.centroid_offset.norm() > 1e-6 ||
          !this->geometry.ellipsoid_axes.isIdentity(1e-6)) {
        const Eigen::Vector3d worldCenter = this->position
          + this->orientation * this->geometry.centroid_offset;
        const Eigen::Quaterniond markerQ = this->orientation
          * Eigen::Quaterniond(this->geometry.ellipsoid_axes);
        transform.setTranslation(coal::Vec3s(
          static_cast<coal::CoalScalar>(worldCenter.x()),
          static_cast<coal::CoalScalar>(worldCenter.y()),
          static_cast<coal::CoalScalar>(worldCenter.z())
        ));
        coal::Quatf capsuleQuat(
          static_cast<coal::CoalScalar>(markerQ.w()),
          static_cast<coal::CoalScalar>(markerQ.x()),
          static_cast<coal::CoalScalar>(markerQ.y()),
          static_cast<coal::CoalScalar>(markerQ.z())
        );
        transform.setQuatRotation(capsuleQuat);
      }
      break;
    }

    case ObstacleType::MESH: {
      if (!this->meshCollisionData || !this->meshCollisionData->bvh) {
        std::string errorMsg = "Mesh collision data not available for obstacle: " + this->frameID;
        if (!this->meshResource.empty()) {
          errorMsg += "\n  Attempted to load mesh from: " + this->meshResource;
          errorMsg += "\n  Mesh scale: [" + std::to_string(this->meshScale.x()) + ", "
            + std::to_string(this->meshScale.y()) + ", "
            + std::to_string(this->meshScale.z()) + "]";
          errorMsg += "\n  Possible causes:";
          errorMsg += "\n    - Mesh file does not exist at the specified path";
          errorMsg += "\n    - ROS package path is not resolved (package:// URI requires ROS environment)";
          errorMsg += "\n    - Mesh file format is not supported or corrupted";
          errorMsg += "\n    - Insufficient permissions to read the mesh file";
        }
        else {
          errorMsg += "\n  No mesh resource path was provided";
        }
        throw std::runtime_error(errorMsg);
      }
      // Use the existing BVH from mesh collision data
      shape = this->meshCollisionData->bvh;
      break;
    }

    default:
      throw std::invalid_argument("Unknown obstacle type: " + obstacleTypeToString(this->type));
    }

    // Create and return collision object
    this->coalCollisionObject = std::make_shared<coal::CollisionObject>(shape, transform);
    return this->coalCollisionObject;
  }

  void PotentialFieldObstacle::updateCoalCollisionObjectPose() const {
    if (!this->coalCollisionObject) { return; }
    coal::Transform3s transform = coal::Transform3s::Identity();

    Eigen::Vector3d worldPos = this->position;
    Eigen::Quaterniond worldOri = this->orientation;

    // For OBB-fitted BOX obstacles, the Coal object is centred at the OBB centroid
    // with the OBB rotation baked in — replicate that here on every pose update.
    if ((this->type == ObstacleType::BOX || this->type == ObstacleType::CAPSULE) &&
        (this->geometry.centroid_offset.norm() > 1e-6 ||
         !this->geometry.ellipsoid_axes.isIdentity(1e-6))) {
      worldPos = this->position + this->orientation * this->geometry.centroid_offset;
      worldOri = this->orientation * Eigen::Quaterniond(this->geometry.ellipsoid_axes);
    }

    transform.setTranslation(
      Eigen::Vector3d(
        static_cast<coal::CoalScalar>(worldPos.x()),
        static_cast<coal::CoalScalar>(worldPos.y()),
        static_cast<coal::CoalScalar>(worldPos.z())
      )
    );
    transform.setQuatRotation(
      coal::Quatf(
        static_cast<coal::CoalScalar>(worldOri.w()),
        static_cast<coal::CoalScalar>(worldOri.x()),
        static_cast<coal::CoalScalar>(worldOri.y()),
        static_cast<coal::CoalScalar>(worldOri.z())
      )
    );
    this->coalCollisionObject->setTransform(transform);
    this->coalCollisionObject->computeAABB();
  }

} // namespace pfield
