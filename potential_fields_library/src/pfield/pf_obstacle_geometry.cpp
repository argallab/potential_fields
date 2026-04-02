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

#include "pfield/pf_obstacle_geometry.hpp"
#include "pfield/pf_obstacle.hpp"       // For ObstacleType enum definition
#include "pfield/mesh_collision.hpp"
#include "pfield/pfield_common.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pfield {

  // ---------------------------------------------------------------------------
  // Internal helper: build the base Coal transform from position + orientation
  // ---------------------------------------------------------------------------
  static coal::Transform3s makeCoalTransform(
    const Eigen::Vector3d& pos, const Eigen::Quaterniond& ori) {
    coal::Transform3s t = coal::Transform3s::Identity();
    t.setTranslation(coal::Vec3s(
      static_cast<coal::CoalScalar>(pos.x()),
      static_cast<coal::CoalScalar>(pos.y()),
      static_cast<coal::CoalScalar>(pos.z())
    ));
    t.setQuatRotation(coal::Quatf(
      static_cast<coal::CoalScalar>(ori.w()),
      static_cast<coal::CoalScalar>(ori.x()),
      static_cast<coal::CoalScalar>(ori.y()),
      static_cast<coal::CoalScalar>(ori.z())
    ));
    return t;
  }

  // ---------------------------------------------------------------------------
  // Internal helper: build an OBB-shifted Coal transform
  //   worldCenter = pos + ori * centroidOffset
  //   worldOri    = ori * Quaterniond(axes)
  // ---------------------------------------------------------------------------
  static coal::Transform3s makeOBBCoalTransform(
    const Eigen::Vector3d& pos,
    const Eigen::Quaterniond& ori,
    const Eigen::Matrix3d& axes,
    const Eigen::Vector3d& centroidOffset) {
    const Eigen::Vector3d worldCenter = pos + ori * centroidOffset;
    const Eigen::Quaterniond worldOri = ori * Eigen::Quaterniond(axes);
    return makeCoalTransform(worldCenter, worldOri);
  }

  // ---------------------------------------------------------------------------
  // Internal helper: decide whether an OBB adjustment is needed
  // ---------------------------------------------------------------------------
  static bool needsOBBAdjustment(
    const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset) {
    return centroidOffset.norm() > 1e-6 || !axes.isIdentity(1e-6);
  }

  // ============================================================================
  // SphereGeometry
  // ============================================================================

  SphereGeometry::SphereGeometry(double radius) : radius(radius) {}

  ObstacleType SphereGeometry::getType() const { return ObstacleType::SPHERE; }

  std::vector<double> SphereGeometry::asVector() const { return {radius}; }

  Eigen::Vector3d SphereGeometry::halfDimensions(const Eigen::Vector3d& /*meshScale*/) const {
    return Eigen::Vector3d(radius, radius, radius);
  }

  void SphereGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    const double distance = localPoint.norm();
    signedDistance = distance - radius;
    // If the query point coincides with the sphere center, normal is arbitrary.
    normal = (distance > NEAR_ZERO_THRESHOLD)
      ? rotateVector(obstacleOrientation, localPoint / distance)
      : rotateVector(obstacleOrientation, Eigen::Vector3d::UnitX());
  }

  std::shared_ptr<coal::CollisionGeometry> SphereGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    return std::make_shared<coal::Sphere>(static_cast<coal::CoalScalar>(radius));
  }

  coal::Transform3s SphereGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool SphereGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const SphereGeometry*>(&other);
    return o && radius == o->radius;
  }

  std::unique_ptr<ObstacleGeometry> SphereGeometry::clone() const {
    return std::make_unique<SphereGeometry>(*this);
  }

  // ============================================================================
  // BoxGeometry
  // ============================================================================

  BoxGeometry::BoxGeometry(double length, double width, double height)
    : length(length), width(width), height(height),
    axes(Eigen::Matrix3d::Identity()),
    centroidOffset(Eigen::Vector3d::Zero()) {}

  BoxGeometry::BoxGeometry(double length, double width, double height,
    const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset)
    : length(length), width(width), height(height),
    axes(axes), centroidOffset(centroidOffset) {}

  ObstacleType BoxGeometry::getType() const { return ObstacleType::BOX; }

  std::vector<double> BoxGeometry::asVector() const { return {length, width, height}; }

  Eigen::Vector3d BoxGeometry::halfDimensions(const Eigen::Vector3d& /*meshScale*/) const {
    return Eigen::Vector3d(length / 2.0, width / 2.0, height / 2.0);
  }

  void BoxGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    // Transform into OBB principal-axis frame (centroid-relative, then PCA-rotated).
    // For ordinary axis-aligned boxes, axes = Identity and centroidOffset = Zero,
    // so this reduces to the standard box SDF.
    const Eigen::Vector3d pOBB = axes.transpose() * (localPoint - centroidOffset);
    const Eigen::Vector3d halfDims(length / 2.0, width / 2.0, height / 2.0);

    const Eigen::Vector3d boundedLocalPoint = pOBB.cwiseMax(-halfDims).cwiseMin(halfDims);
    const Eigen::Vector3d distanceToBoundedPoint = pOBB - boundedLocalPoint;
    const double distanceToSurface = distanceToBoundedPoint.norm();

    Eigen::Vector3d normalOBB;
    if (distanceToSurface > NEAR_ZERO_THRESHOLD) {
      // Outside box: normal from closest surface point to query point
      normalOBB = distanceToBoundedPoint / distanceToSurface;
      signedDistance = distanceToSurface;
    }
    else {
      // Inside box: find the closest face and set the inward signed distance
      const Eigen::Vector3d distancesToFaces = halfDims - pOBB.cwiseAbs();
      int axisIndex;
      distancesToFaces.minCoeff(&axisIndex);
      normalOBB = Eigen::Vector3d::Zero();
      normalOBB[axisIndex] = (pOBB[axisIndex] >= 0.0) ? 1.0 : -1.0;
      signedDistance = -distancesToFaces[axisIndex];
    }

    // Rotate normal: OBB frame → obstacle frame → world frame
    const Eigen::Vector3d normalObstacle = axes * normalOBB;
    normal = rotateVector(obstacleOrientation, normalObstacle);
  }

  std::shared_ptr<coal::CollisionGeometry> BoxGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    if (needsOBBAdjustment(axes, centroidOffset)) {
      coalTransform = makeOBBCoalTransform(obstaclePosition, obstacleOrientation,
        axes, centroidOffset);
    }
    else {
      coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    }
    return std::make_shared<coal::Box>(
      static_cast<coal::CoalScalar>(length),
      static_cast<coal::CoalScalar>(width),
      static_cast<coal::CoalScalar>(height)
    );
  }

  coal::Transform3s BoxGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    if (needsOBBAdjustment(axes, centroidOffset)) {
      return makeOBBCoalTransform(obstaclePosition, obstacleOrientation,
        axes, centroidOffset);
    }
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool BoxGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const BoxGeometry*>(&other);
    return o && length == o->length && width == o->width && height == o->height
      && centroidOffset == o->centroidOffset && axes == o->axes;
  }

  std::unique_ptr<ObstacleGeometry> BoxGeometry::clone() const {
    return std::make_unique<BoxGeometry>(*this);
  }

  // ============================================================================
  // CylinderGeometry
  // ============================================================================

  CylinderGeometry::CylinderGeometry(double radius, double height)
    : radius(radius), height(height) {}

  ObstacleType CylinderGeometry::getType() const { return ObstacleType::CYLINDER; }

  std::vector<double> CylinderGeometry::asVector() const { return {radius, height}; }

  Eigen::Vector3d CylinderGeometry::halfDimensions(const Eigen::Vector3d& /*meshScale*/) const {
    return Eigen::Vector3d(radius, radius, height / 2.0);
  }

  void CylinderGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    const Eigen::Vector2d radialPoint(localPoint.x(), localPoint.y());
    const double radialDist = radialPoint.norm();
    const double halfHeight = height / 2.0;
    Eigen::Vector3d normalLocal = Eigen::Vector3d::Zero();

    if (radialDist <= radius && std::abs(localPoint.z()) <= halfHeight) {
      // Inside the cylinder: find the nearest feature (side, top cap, or bottom cap)
      const double distanceToSide = radius - radialDist;
      const double distanceToTopPlane = halfHeight - localPoint.z();
      const double distanceToBottomPlane = halfHeight + localPoint.z();

      if (distanceToTopPlane < distanceToSide && distanceToTopPlane < distanceToBottomPlane) {
        signedDistance = -distanceToTopPlane;
        normalLocal = Eigen::Vector3d::UnitZ();
      }
      else if (distanceToBottomPlane < distanceToSide &&
        distanceToBottomPlane < distanceToTopPlane) {
        signedDistance = -distanceToBottomPlane;
        normalLocal = -Eigen::Vector3d::UnitZ();
      }
      else {
        signedDistance = -distanceToSide;
        normalLocal = (radialDist > NEAR_ZERO_THRESHOLD)
          ? Eigen::Vector3d(-localPoint.x() / radialDist, -localPoint.y() / radialDist, 0.0)
          : -Eigen::Vector3d::UnitX();
      }
    }
    else {
      // Outside: find closest point on the cylinder surface
      Eigen::Vector3d closestPoint = Eigen::Vector3d::Zero();

      if (radialDist > radius) {
        closestPoint.head<2>() = (radialDist > NEAR_ZERO_THRESHOLD)
          ? radialPoint / radialDist * radius
          : Eigen::Vector2d(radius, 0.0);
      }
      else {
        closestPoint.head<2>() = radialPoint;
      }

      closestPoint.z() = std::clamp(localPoint.z(), -halfHeight, halfHeight);

      const Eigen::Vector3d diff = localPoint - closestPoint;
      signedDistance = diff.norm();
      if (signedDistance > NEAR_ZERO_THRESHOLD) {
        normalLocal = diff / signedDistance;
      } else {
        normalLocal = Eigen::Vector3d::UnitX();
      }
    }

    normal = rotateVector(obstacleOrientation, normalLocal);
  }

  std::shared_ptr<coal::CollisionGeometry> CylinderGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    return std::make_shared<coal::Cylinder>(
      static_cast<coal::CoalScalar>(radius),
      static_cast<coal::CoalScalar>(height)
    );
  }

  coal::Transform3s CylinderGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool CylinderGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const CylinderGeometry*>(&other);
    return o && radius == o->radius && height == o->height;
  }

  std::unique_ptr<ObstacleGeometry> CylinderGeometry::clone() const {
    return std::make_unique<CylinderGeometry>(*this);
  }

  // ============================================================================
  // EllipsoidGeometry
  // ============================================================================

  EllipsoidGeometry::EllipsoidGeometry(double semiX, double semiY, double semiZ)
    : semiX(semiX), semiY(semiY), semiZ(semiZ),
    axes(Eigen::Matrix3d::Identity()),
    centroidOffset(Eigen::Vector3d::Zero()) {}

  EllipsoidGeometry::EllipsoidGeometry(double semiX, double semiY, double semiZ,
    const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset)
    : semiX(semiX), semiY(semiY), semiZ(semiZ),
    axes(axes), centroidOffset(centroidOffset) {}

  ObstacleType EllipsoidGeometry::getType() const { return ObstacleType::ELLIPSOID; }

  std::vector<double> EllipsoidGeometry::asVector() const { return {semiX, semiY, semiZ}; }

  Eigen::Vector3d EllipsoidGeometry::halfDimensions(const Eigen::Vector3d& /*meshScale*/) const {
    return Eigen::Vector3d(semiX, semiY, semiZ);
  }

  void EllipsoidGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    if (semiX < NEAR_ZERO_THRESHOLD || semiY < NEAR_ZERO_THRESHOLD ||
      semiZ < NEAR_ZERO_THRESHOLD) {
      throw std::runtime_error("EllipsoidGeometry: zero or near-zero semi-axis");
    }

    // Shift to ellipsoid center (mesh centroid in obstacle-local frame) then rotate
    // into the PCA principal-axis frame so the SDF axes align with the ellipsoid axes.
    const Eigen::Vector3d pLocal = axes.transpose() * (localPoint - centroidOffset);

    // Implicit function f(p) = (px/a)^2 + (py/b)^2 + (pz/c)^2 - 1
    // f < 0 => inside, f > 0 => outside
    const double f = (pLocal.x() * pLocal.x()) / (semiX * semiX)
      + (pLocal.y() * pLocal.y()) / (semiY * semiY)
      + (pLocal.z() * pLocal.z()) / (semiZ * semiZ)
      - 1.0;

    // Gradient of f in PCA frame (outward normal direction)
    const Eigen::Vector3d gradPCA(
      2.0 * pLocal.x() / (semiX * semiX),
      2.0 * pLocal.y() / (semiY * semiY),
      2.0 * pLocal.z() / (semiZ * semiZ)
    );
    const double gradNorm = gradPCA.norm();

    // First-order signed distance approximation: d ≈ f / ||∇f||
    if (gradNorm > NEAR_ZERO_THRESHOLD) {
      signedDistance = f / gradNorm;
      // Rotate gradient back: PCA frame → obstacle frame → world frame
      const Eigen::Vector3d gradObstacle = axes * (gradPCA / gradNorm);
      normal = rotateVector(obstacleOrientation, gradObstacle);
    }
    else {
      // Point is exactly at the ellipsoid center; set arbitrary outward normal
      signedDistance = -std::min({semiX, semiY, semiZ});
      normal = rotateVector(obstacleOrientation, Eigen::Vector3d::UnitX());
    }
  }

  std::shared_ptr<coal::CollisionGeometry> EllipsoidGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    return std::make_shared<coal::Ellipsoid>(
      static_cast<coal::CoalScalar>(semiX),
      static_cast<coal::CoalScalar>(semiY),
      static_cast<coal::CoalScalar>(semiZ)
    );
  }

  coal::Transform3s EllipsoidGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool EllipsoidGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const EllipsoidGeometry*>(&other);
    return o && semiX == o->semiX && semiY == o->semiY && semiZ == o->semiZ
      && centroidOffset == o->centroidOffset && axes == o->axes;
  }

  std::unique_ptr<ObstacleGeometry> EllipsoidGeometry::clone() const {
    return std::make_unique<EllipsoidGeometry>(*this);
  }

  // ============================================================================
  // CapsuleGeometry
  // ============================================================================

  CapsuleGeometry::CapsuleGeometry(double radius, double height)
    : radius(radius), height(height),
    axes(Eigen::Matrix3d::Identity()),
    centroidOffset(Eigen::Vector3d::Zero()) {}

  CapsuleGeometry::CapsuleGeometry(double radius, double height,
    const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset)
    : radius(radius), height(height), axes(axes), centroidOffset(centroidOffset) {}

  ObstacleType CapsuleGeometry::getType() const { return ObstacleType::CAPSULE; }

  std::vector<double> CapsuleGeometry::asVector() const { return {radius, height}; }

  Eigen::Vector3d CapsuleGeometry::halfDimensions(const Eigen::Vector3d& /*meshScale*/) const {
    // Bounding box: radial extent = radius in X/Y, half-length = height/2 + radius in Z
    return Eigen::Vector3d(radius, radius, height / 2.0 + radius);
  }

  void CapsuleGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    // Transform into capsule-local frame (centroid-relative, then PCA-rotated).
    // The capsule axis runs along the local Z axis; endcap centers at ±height/2.
    const Eigen::Vector3d pCap = axes.transpose() * (localPoint - centroidOffset);
    const double halfShaft = height / 2.0;

    // Project onto the capsule axis (Z) and clamp to the shaft segment
    const double zClamped = std::clamp(pCap.z(), -halfShaft, halfShaft);

    // Vector from the nearest point on the axis segment to the query point
    const Eigen::Vector3d toPoint(pCap.x(), pCap.y(), pCap.z() - zClamped);
    const double dist = toPoint.norm();
    signedDistance = dist - radius;

    Eigen::Vector3d normalCap;
    if (dist > NEAR_ZERO_THRESHOLD) {
      normalCap = toPoint / dist;
    }
    else {
      // Query point is on the capsule axis — push outward in X
      normalCap = Eigen::Vector3d::UnitX();
    }

    // Rotate back: capsule frame → obstacle frame → world frame
    normal = rotateVector(obstacleOrientation, axes * normalCap);
  }

  std::shared_ptr<coal::CollisionGeometry> CapsuleGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    if (needsOBBAdjustment(axes, centroidOffset)) {
      coalTransform = makeOBBCoalTransform(obstaclePosition, obstacleOrientation,
        axes, centroidOffset);
    }
    else {
      coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    }
    // coal::Capsule(radius, lz): lz is the shaft length; endcaps add radius at each end.
    return std::make_shared<coal::Capsule>(
      static_cast<coal::CoalScalar>(radius),
      static_cast<coal::CoalScalar>(height)
    );
  }

  coal::Transform3s CapsuleGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    if (needsOBBAdjustment(axes, centroidOffset)) {
      return makeOBBCoalTransform(obstaclePosition, obstacleOrientation,
        axes, centroidOffset);
    }
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool CapsuleGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const CapsuleGeometry*>(&other);
    return o && radius == o->radius && height == o->height
      && centroidOffset == o->centroidOffset && axes == o->axes;
  }

  std::unique_ptr<ObstacleGeometry> CapsuleGeometry::clone() const {
    return std::make_unique<CapsuleGeometry>(*this);
  }

  // ============================================================================
  // MeshGeometry
  // ============================================================================

  MeshGeometry::MeshGeometry(std::shared_ptr<MeshCollisionData> meshCollisionData)
    : meshCollisionData(std::move(meshCollisionData)) {}

  ObstacleType MeshGeometry::getType() const { return ObstacleType::MESH; }

  std::vector<double> MeshGeometry::asVector() const { return {}; }

  Eigen::Vector3d MeshGeometry::halfDimensions(const Eigen::Vector3d& meshScale) const {
    return Eigen::Vector3d(meshScale.x() / 2.0, meshScale.y() / 2.0, meshScale.z() / 2.0);
  }

  void MeshGeometry::computeSignedDistanceAndNormal(
    const Eigen::Vector3d& localPoint,
    const Eigen::Quaterniond& obstacleOrientation,
    double& signedDistance,
    Eigen::Vector3d& normal) const {
    if (!meshCollisionData) {
      throw std::runtime_error("MeshGeometry: mesh collision data not available");
    }

    const bool inside = pointInsideMesh(*meshCollisionData, localPoint);
    const double distanceToSurface = computeUnsignedDistanceToMesh(*meshCollisionData, localPoint);

    Eigen::Vector3d closestPointOnMesh;
    const bool gotClosest = getClosestPointOnMesh(*meshCollisionData, localPoint, closestPointOnMesh);

    Eigen::Vector3d normalLocal = Eigen::Vector3d::UnitX();  // fallback
    if (gotClosest) {
      Eigen::Vector3d vecToClosest = localPoint - closestPointOnMesh;
      if (inside) { vecToClosest = -vecToClosest; }
      const double d = vecToClosest.norm();
      if (d > NEAR_ZERO_THRESHOLD) { normalLocal = vecToClosest / d; }
    }
    else {
      // Fallback: AABB centroid gradient direction
      const Eigen::Vector3d midpointAABB =
        0.5 * (meshCollisionData->aabbMax + meshCollisionData->aabbMin);
      normalLocal = localPoint - midpointAABB;
      const double n = normalLocal.norm();
      if (n > NEAR_ZERO_THRESHOLD) { normalLocal /= n; }
    }

    signedDistance = inside ? -std::abs(distanceToSurface) : std::abs(distanceToSurface);
    normal = rotateVector(obstacleOrientation, normalLocal).normalized();
  }

  std::shared_ptr<coal::CollisionGeometry> MeshGeometry::buildCoalGeometry(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation,
    coal::Transform3s& coalTransform) const {
    if (!meshCollisionData || !meshCollisionData->bvh) {
      throw std::runtime_error("MeshGeometry: mesh collision data not available for Coal object");
    }
    coalTransform = makeCoalTransform(obstaclePosition, obstacleOrientation);
    return meshCollisionData->bvh;
  }

  coal::Transform3s MeshGeometry::computeUpdatedCoalTransform(
    const Eigen::Vector3d& obstaclePosition,
    const Eigen::Quaterniond& obstacleOrientation) const {
    return makeCoalTransform(obstaclePosition, obstacleOrientation);
  }

  bool MeshGeometry::operator==(const ObstacleGeometry& other) const {
    const auto* o = dynamic_cast<const MeshGeometry*>(&other);
    // Two MeshGeometry instances are equal if they share the same collision data pointer.
    return o && meshCollisionData == o->meshCollisionData;
  }

  std::unique_ptr<ObstacleGeometry> MeshGeometry::clone() const {
    return std::make_unique<MeshGeometry>(*this);
  }

}  // namespace pfield
