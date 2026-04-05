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

#ifndef PF_OBSTACLE_HPP
#define PF_OBSTACLE_HPP

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

#include "pfield/spatial_vector.hpp"
#include "pfield/mesh_collision.hpp"
#include "pfield/pf_obstacle_geometry.hpp"

namespace pfield {

  /**
   * @brief Enumeration of different obstacle types.
   *
   * Each value corresponds to a concrete ObstacleGeometry subclass.
   */
  enum class ObstacleType {
    SPHERE,     ///< Sphere — defined by radius.
    BOX,        ///< Box (axis-aligned or OBB) — defined by length, width, height.
    CYLINDER,   ///< Cylinder — defined by radius and height.
    MESH,       ///< Triangle mesh — geometry defined by an external mesh resource.
    ELLIPSOID,  ///< Ellipsoid — defined by three semi-axes; smooth analytic SDF, used for robot link approximation.
    CAPSULE     ///< Capsule — cylindrical shaft (height) with hemispherical endcaps (radius); total length = height + 2*radius.
  };

  enum class ObstacleGroup {
    STATIC,   ///< Static environment obstacles.
    DYNAMIC,  ///< Dynamic environment obstacles (Unused).
    ROBOT     ///< Robot links, used for self-collision avoidance.
  };

  inline std::string obstacleTypeToString(const ObstacleType& type) {
    switch (type) {
    case ObstacleType::SPHERE:
      return "Sphere";
    case ObstacleType::BOX:
      return "Box";
    case ObstacleType::CYLINDER:
      return "Cylinder";
    case ObstacleType::MESH:
      return "Mesh";
    case ObstacleType::ELLIPSOID:
      return "Ellipsoid";
    case ObstacleType::CAPSULE:
      return "Capsule";
    default:
      throw std::invalid_argument("Unknown obstacle type");
    }
  }

  inline ObstacleType stringToObstacleType(const std::string& typeStr) {
    if (typeStr == "Sphere") {
      return ObstacleType::SPHERE;
    }
    else if (typeStr == "Box") {
      return ObstacleType::BOX;
    }
    else if (typeStr == "Cylinder") {
      return ObstacleType::CYLINDER;
    }
    else if (typeStr == "Mesh") {
      return ObstacleType::MESH;
    }
    else if (typeStr == "Ellipsoid") {
      return ObstacleType::ELLIPSOID;
    }
    else if (typeStr == "Capsule") {
      return ObstacleType::CAPSULE;
    }
    else {
      throw std::invalid_argument("Unknown obstacle type string: " + typeStr);
    }
  }

  inline std::string obstacleGroupToString(const ObstacleGroup& group) {
    switch (group) {
    case ObstacleGroup::STATIC:
      return "Static";
    case ObstacleGroup::DYNAMIC:
      return "Dynamic";
    case ObstacleGroup::ROBOT:
      return "Robot";
    default:
      throw std::invalid_argument("Unknown obstacle group");
    }
  }

  inline ObstacleGroup stringToObstacleGroup(const std::string& groupStr) {
    if (groupStr == "Static") {
      return ObstacleGroup::STATIC;
    }
    else if (groupStr == "Dynamic") {
      return ObstacleGroup::DYNAMIC;
    }
    else if (groupStr == "Robot") {
      return ObstacleGroup::ROBOT;
    }
    else {
      throw std::invalid_argument("Unknown obstacle group string: " + groupStr);
    }
  }

  /**
   * @brief Represents a single obstacle in the potential field environment.
   *
   * Holds a world-frame pose (position + orientation), a group classification,
   * and a concrete ObstacleGeometry that describes the shape. All signed-distance,
   * normal, and Coal collision-object operations are delegated to the geometry.
   */
  class PotentialFieldObstacle {
  public:
    PotentialFieldObstacle() = delete;
    ~PotentialFieldObstacle() = default;

    /**
     * @brief Constructs a PotentialFieldObstacle.
     *
     * @param frameID      Unique identifier for this obstacle (e.g. "link1::collision").
     * @param centerPosition  World-frame center position.
     * @param orientation     World-frame orientation.
     * @param group           Obstacle group (STATIC, DYNAMIC, ROBOT).
     * @param geometry        Concrete geometry describing the obstacle shape.
     *                        Ownership is transferred via unique_ptr.
     * @param meshResource    URI to the mesh file (used only when geometry is MeshGeometry).
     * @param meshScale       Scale applied to the mesh for visualization.
     */
    PotentialFieldObstacle(
      std::string frameID,
      Eigen::Vector3d centerPosition, Eigen::Quaterniond orientation,
      ObstacleGroup group, std::unique_ptr<ObstacleGeometry> geometry,
      const std::string& meshResource = std::string(),
      const Eigen::Vector3d& meshScale = Eigen::Vector3d::Ones())
      : frameID(std::move(frameID)),
      position(std::move(centerPosition)),
      orientation(orientation),
      orientationConjugate(orientation.conjugate()),
      group(group),
      geometry(std::move(geometry)),
      meshResource(meshResource),
      meshScale(meshScale) {
      if (this->geometry->getType() == ObstacleType::MESH && !meshResource.empty()) {
        try {
          auto meshData = loadMesh(meshResource);
          auto* meshGeom = dynamic_cast<MeshGeometry*>(this->geometry.get());
          if (meshGeom && meshData) {
            meshGeom->meshCollisionData = meshData;
          }
        }
        catch (const std::exception&) {}
      }
      // Only attempt to create the COAL object if we have valid data.
      // For MESH types this means meshCollisionData must be loaded.
      if (this->geometry->getType() != ObstacleType::MESH) {
        this->toCoalCollisionObject();
      } else {
        auto* meshGeom2 = dynamic_cast<MeshGeometry*>(this->geometry.get());
        if (meshGeom2 && meshGeom2->meshCollisionData) {
          this->toCoalCollisionObject();
        }
      }
    }

    PotentialFieldObstacle(const PotentialFieldObstacle& other)
      : frameID(other.frameID),
      position(other.position),
      orientation(other.orientation),
      orientationConjugate(other.orientationConjugate),
      group(other.group),
      geometry(other.geometry->clone()),
      meshResource(other.meshResource),
      meshScale(other.meshScale) {
      this->toCoalCollisionObject();
    }

    PotentialFieldObstacle(PotentialFieldObstacle&& other) noexcept
      : frameID(std::move(other.frameID)),
      position(std::move(other.position)),
      orientation(std::move(other.orientation)),
      orientationConjugate(std::move(other.orientationConjugate)),
      group(other.group),
      geometry(std::move(other.geometry)),
      meshResource(std::move(other.meshResource)),
      meshScale(std::move(other.meshScale)) {
      this->toCoalCollisionObject();
    }

    PotentialFieldObstacle& operator=(const PotentialFieldObstacle& other) {
      if (this != &other) {
        this->frameID = other.frameID;
        this->position = other.position;
        this->orientation = other.orientation;
        this->orientationConjugate = other.orientationConjugate;
        this->group = other.group;
        this->geometry = other.geometry->clone();
        this->meshResource = other.meshResource;
        this->meshScale = other.meshScale;
        this->toCoalCollisionObject();
      }
      return *this;
    }

    bool operator==(const PotentialFieldObstacle& other) const {
      return this->position == other.position && *this->geometry == *other.geometry;
    }
    bool operator!=(const PotentialFieldObstacle& other) const { return !(*this == other); }

    [[nodiscard]] std::string getFrameID() const { return this->frameID; }
    [[nodiscard]] ObstacleGroup getGroup() const { return this->group; }
    [[nodiscard]] Eigen::Vector3d getPosition() const { return this->position; }
    [[nodiscard]] Eigen::Quaterniond getOrientation() const { return this->orientation; }
    [[nodiscard]] ObstacleType getType() const { return this->geometry->getType(); }
    [[nodiscard]] const ObstacleGeometry& getGeometry() const { return *this->geometry; }
    [[nodiscard]] const std::string& getMeshResource() const { return this->meshResource; }
    [[nodiscard]] Eigen::Vector3d getMeshScale() const { return this->meshScale; }
    [[nodiscard]] std::shared_ptr<coal::CollisionObject> getCoalCollisionObject() const { return this->coalCollisionObject; }

    void setPose(const Eigen::Vector3d newPosition, const Eigen::Quaterniond newOrientation) {
      this->position = newPosition;
      this->orientation = newOrientation;
      this->orientationConjugate = newOrientation.conjugate();
      this->updateCoalCollisionObjectPose();
    }

    void setPosition(Eigen::Vector3d newPosition) {
      this->position = newPosition;
      this->updateCoalCollisionObjectPose();
    }

    void setOrientation(Eigen::Quaterniond newOrientation) {
      this->orientation = newOrientation;
      this->orientationConjugate = newOrientation.conjugate();
      this->updateCoalCollisionObjectPose();
    }

    void setMeshCollisionData(const std::shared_ptr<MeshCollisionData>& meshData) {
      auto* meshGeom = dynamic_cast<MeshGeometry*>(this->geometry.get());
      if (meshGeom) { meshGeom->meshCollisionData = meshData; }
    }

    void setMeshProperties(const std::string& meshResource, const Eigen::Vector3d& meshScale) {
      this->meshResource = meshResource;
      this->meshScale = meshScale;
      if (this->geometry->getType() == ObstacleType::MESH && !meshResource.empty()) {
        try {
          auto meshData = loadMesh(meshResource);
          auto* meshGeom = dynamic_cast<MeshGeometry*>(this->geometry.get());
          if (meshGeom && meshData) {
            meshGeom->meshCollisionData = meshData;
          }
        }
        catch (const std::exception&) {}
      }
    }

    bool withinInfluenceZone(Eigen::Vector3d worldPoint, double influenceDistance) const;
    bool withinObstacle(Eigen::Vector3d worldPoint) const;

    /**
     * @brief Rotate the given point into the obstacle's local frame.
     *
     * @param worldPoint  Point in world coordinates.
     * @return The point expressed in the obstacle's local frame.
     */
    Eigen::Vector3d toObstacleFrame(const Eigen::Vector3d& worldPoint) const;

    /**
     * @brief Returns the half-dimensions of the obstacle's axis-aligned bounding box.
     *
     * Used for broad-phase influence-zone checks.
     */
    Eigen::Vector3d halfDimensions() const;

    /**
     * @brief Computes the signed distance and outward normal from a world point to
     *        the obstacle surface.
     *
     * @param[in]  worldPoint      Query point in world coordinates.
     * @param[out] signedDistance  Distance to the surface (negative if inside).
     * @param[out] normal          Outward unit normal in world coordinates.
     */
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& worldPoint,
      double& signedDistance,
      Eigen::Vector3d& normal) const;

    /**
     * @brief Creates (or returns the cached) Coal collision object for this obstacle.
     *
     * @return Shared pointer to the Coal collision object.
     */
    std::shared_ptr<coal::CollisionObject> toCoalCollisionObject();

    /**
     * @brief Updates the cached Coal collision object's pose to match the current
     *        obstacle position and orientation.
     */
    void updateCoalCollisionObjectPose() const;

  private:
    std::string frameID;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Quaterniond orientationConjugate;
    ObstacleGroup group;
    std::unique_ptr<ObstacleGeometry> geometry;
    std::string meshResource;
    Eigen::Vector3d meshScale;
    std::shared_ptr<coal::CollisionObject> coalCollisionObject;
    std::shared_ptr<coal::CollisionGeometry> coalCollisionGeometry;
  };

  struct PotentialFieldObstacleHash {
    std::size_t operator()(const PotentialFieldObstacle& obstacle) const {
      return std::hash<std::string>()(obstacle.getFrameID()) ^
        std::hash<std::string>()(obstacleTypeToString(obstacle.getType())) ^
        std::hash<std::string>()(obstacleGroupToString(obstacle.getGroup()));
    }
  };

  inline int createHashID(const PotentialFieldObstacle& obstacle) {
    PotentialFieldObstacleHash hasher;
    return static_cast<int>(hasher(obstacle) & 0x7FFFFFFF);  // keep positive
  }

}  // namespace pfield

#endif  // PF_OBSTACLE_HPP
