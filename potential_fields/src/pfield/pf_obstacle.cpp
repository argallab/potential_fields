#include "pfield/pf_obstacle.hpp"
#include "pfield/mesh_collision.hpp"
#include "pfield/pfield_common.hpp"

#include <coal/narrowphase/narrowphase.h>
#include <coal/collision.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

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
    Eigen::Vector3d halfDims = this->halfDimensions();
    Eigen::Vector3d boundedLocalPoint = localPoint.cwiseMax(-halfDims).cwiseMin(halfDims); // clamp to box
    Eigen::Vector3d distanceToBoundedPoint = localPoint - boundedLocalPoint;
    const double distanceToSurface = distanceToBoundedPoint.norm();
    Eigen::Vector3d normalLocal;
    if (distanceToSurface > NEAR_ZERO_THRESHOLD) {
      // Outside box: normal points from closest point on box to localPoint
      normalLocal = distanceToBoundedPoint / distanceToSurface;
      signedDistance = distanceToSurface;
    }
    else {
      // Inside box: find closest face
      Eigen::Vector3d distancesToFaces = halfDims - localPoint.cwiseAbs();
      int axisIndex;
      distancesToFaces.minCoeff(&axisIndex);
      normalLocal = Eigen::Vector3d::Zero();
      normalLocal[axisIndex] = (localPoint[axisIndex] >= 0.0) ? 1.0 : -1.0;
      signedDistance = -distancesToFaces[axisIndex];
    }
    // Rotate the output normal back to world frame
    normal = rotateVector(this->orientation, normalLocal);
    break;
  }
  case ObstacleType::CYLINDER: {
    // Get the XY radial distance of the point to the cylinder axis
    // You can think of this as projecting the point onto the XY plane and the cylinder axis along its height (Z) axis
    Eigen::Vector2d radialPoint(localPoint.x(), localPoint.y());
    const double radialDist = radialPoint.norm();
    // Get distance along height (Z) axis
    const double halfHeight = this->geometry.height / 2.0;
    const double distanceToTop = localPoint.z() - halfHeight;
    // Outside distance to finite cylinder is distance to union of infinite cylinder and top/bottom planes
    Eigen::Vector3d normalLocal = Eigen::Vector3d::Zero();
    double distanceToSurface = 0.0;
    if (radialDist > this->geometry.radius && std::abs(localPoint.z()) <= halfHeight) {
      // Outside on the lateral surface region (within height): distance from side surface
      distanceToSurface = radialDist - this->geometry.radius;
      normalLocal = (radialDist > NEAR_ZERO_THRESHOLD) ?
        Eigen::Vector3d(localPoint.x() / radialDist, localPoint.y() / radialDist, 0.0) :
        Eigen::Vector3d::UnitX();
    }
    else if (localPoint.z() > halfHeight) {
      // Above top plane
      distanceToSurface = distanceToTop;
      normalLocal = Eigen::Vector3d::UnitZ();
    }
    else if (localPoint.z() < -halfHeight) {
      // Below bottom plane
      distanceToSurface = (-halfHeight - localPoint.z());
      normalLocal = -Eigen::Vector3d::UnitZ();
    }
    else {
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
      normal = rotateVector(this->orientation, normalLocal);
      return;
    }
    signedDistance = distanceToSurface;
    // Rotate the output normal back to world frame
    normal = rotateVector(this->orientation, normalLocal);
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

std::shared_ptr<coal::CollisionObject> PotentialFieldObstacle::toCoalCollisionObject() const {
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
    break;
  }

  case ObstacleType::CYLINDER: {
    shape = std::make_shared<coal::Cylinder>(
      static_cast<coal::CoalScalar>(this->geometry.radius),
      static_cast<coal::CoalScalar>(this->geometry.height)
    );
    break;
  }

  case ObstacleType::MESH: {
    if (!this->meshCollisionData || !this->meshCollisionData->bvh) {
      throw std::runtime_error("Mesh collision data not available for obstacle: " + this->frameID);
    }
    // Use the existing BVH from mesh collision data
    shape = this->meshCollisionData->bvh;
    break;
  }

  default:
    throw std::invalid_argument("Unknown obstacle type: " + obstacleTypeToString(this->type));
  }

  // Create and return collision object
  return std::make_shared<coal::CollisionObject>(shape, transform);
}

double PotentialFieldObstacle::computeMinimumDistanceTo(
  const PotentialFieldObstacle& otherObstacle, Eigen::Vector3d& normalToOther) const {
  // Use COAL library for robust distance computation between arbitrary convex shapes
  try {
    // Convert both obstacles to COAL collision objects
    auto coalObj1 = this->toCoalCollisionObject();
    auto coalObj2 = otherObstacle.toCoalCollisionObject();

    // Setup distance request with nearest points enabled
    coal::DistanceRequest request;
    request.enable_nearest_points = true;
    request.rel_err = 0.0;  // No relative error tolerance
    request.abs_err = 0.0;  // No absolute error tolerance

    // Perform distance query
    coal::DistanceResult result;
    coal::CoalScalar distance = coal::distance(coalObj1.get(), coalObj2.get(), request, result);

    // Extract nearest points (in world frame)
    // nearest_points[0] is on this obstacle, nearest_points[1] is on other obstacle
    const coal::Vec3s& p1 = result.nearest_points[0];
    const coal::Vec3s& p2 = result.nearest_points[1];

    // Compute normal from this obstacle to other obstacle
    Eigen::Vector3d point1(p1[0], p1[1], p1[2]);
    Eigen::Vector3d point2(p2[0], p2[1], p2[2]);
    Eigen::Vector3d diff = point2 - point1;

    double dist = diff.norm();
    if (dist > NEAR_ZERO_THRESHOLD) {
      normalToOther = diff / dist;
    }
    else {
      // Objects are in contact or penetrating, use fallback normal
      // Try to get a reasonable normal from center-to-center direction
      Eigen::Vector3d centerDiff = otherObstacle.position - this->position;
      double centerDist = centerDiff.norm();
      if (centerDist > NEAR_ZERO_THRESHOLD) {
        normalToOther = centerDiff / centerDist;
      }
      else {
        // Objects have same center, use arbitrary direction
        normalToOther = Eigen::Vector3d::UnitX();
      }
    }

    return static_cast<double>(distance);
  }
  catch (const std::exception& e) {
    // If COAL fails for any reason, fall back to sampling-based approach
    // This can happen with degenerate geometries or numerical issues
    return computeMinimumDistanceSampling(otherObstacle, normalToOther, 10);
  }
}

double PotentialFieldObstacle::computeMinimumDistanceSampling(
  const PotentialFieldObstacle& otherObstacle, Eigen::Vector3d& normalToOther, int numSamplesPerAxis) const {
  // Sampling-based fallback for complex obstacle pairs
  double minDistance = std::numeric_limits<double>::max();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::Zero();

  for (int i = 0; i < numSamplesPerAxis; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(numSamplesPerAxis - 1);
    for (int j = 0; j < numSamplesPerAxis; ++j) {
      double v = static_cast<double>(j) / static_cast<double>(numSamplesPerAxis - 1);
      for (int k = 0; k < numSamplesPerAxis; ++k) {
        double w = static_cast<double>(k) / static_cast<double>(numSamplesPerAxis - 1);
        // Sample point on this obstacle's bounding box
        Eigen::Vector3d halfDimsThis = this->halfDimensions();
        Eigen::Vector3d localPointThis(
          (u - 0.5) * 2.0 * halfDimsThis.x(),
          (v - 0.5) * 2.0 * halfDimsThis.y(),
          (w - 0.5) * 2.0 * halfDimsThis.z());
        Eigen::Vector3d worldPointThis = this->orientation * localPointThis + this->position;

        // Compute signed distance to other obstacle
        double signedDistanceToOther;
        Eigen::Vector3d normalToOtherLocal;
        otherObstacle.computeSignedDistanceAndNormal(worldPointThis, signedDistanceToOther, normalToOtherLocal);
        double absDistance = std::abs(signedDistanceToOther);

        if (absDistance < minDistance) {
          minDistance = absDistance;
          bestNormal = normalToOtherLocal;
        }
      }
    }
  }

  normalToOther = bestNormal;
  return minDistance;
}
