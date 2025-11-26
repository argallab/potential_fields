#include "pfield_library/pfield/pf_obstacle.hpp"
#include "pfield_library/pfield/mesh_collision.hpp"
#include "pfield_library/pfield/pfield_common.hpp"

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
  transform.setTranslation(
    Eigen::Vector3d(
      static_cast<coal::CoalScalar>(this->position.x()),
      static_cast<coal::CoalScalar>(this->position.y()),
      static_cast<coal::CoalScalar>(this->position.z())
    )
  );
  transform.setQuatRotation(
    coal::Quatf(
      static_cast<coal::CoalScalar>(this->orientation.w()),
      static_cast<coal::CoalScalar>(this->orientation.x()),
      static_cast<coal::CoalScalar>(this->orientation.y()),
      static_cast<coal::CoalScalar>(this->orientation.z())
    )
  );
  this->coalCollisionObject->setTransform(transform);
  this->coalCollisionObject->computeAABB();
}
