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

namespace pfield {

  /**
   * @brief Enumeration of different obstacle types.
   *
   * @note Each type determines which parameters in ObstacleGeometry are used.
   *
   */
  enum class ObstacleType {
    SPHERE, // [center, radius]
    BOX, // [center, length, width, height]
    CYLINDER, // [center, radius, height]
    MESH, // [center, scale_x, scale_y, scale_z]
    ELLIPSOID, // [center, semi_x, semi_y, semi_z] — smooth analytic SDF, used for robot link approximation
    CAPSULE // [center, radius, height] — cylinder shaft (height) with hemispherical endcaps (radius); total length = height + 2*radius
  };

  enum class ObstacleGroup {
    STATIC, // Static environment obstacles
    DYNAMIC, // Dynamic environment obstacles
    ROBOT // The Robot links themselves, used for collision avoidance
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

  struct ObstacleGeometry {
    double radius = 0.0; // Radius for sphere and cylinder, unused for box
    double length = 0.0; // Length for box, unused for sphere and cylinder
    double width = 0.0; // Width for box, unused for sphere and cylinder
    double height = 0.0; // Height for cylinder and box, unused for sphere
    double semi_x = 0.0; // Semi-axis along X for ellipsoid, unused for other types
    double semi_y = 0.0; // Semi-axis along Y for ellipsoid, unused for other types
    double semi_z = 0.0; // Semi-axis along Z for ellipsoid, unused for other types
    // Rotation from obstacle frame to ellipsoid principal-axis frame (from PCA).
    // Identity for axis-aligned ellipsoids; non-identity when PCA axes differ from mesh origin axes.
    Eigen::Matrix3d ellipsoid_axes = Eigen::Matrix3d::Identity();
    // Offset from the collision-origin frame to the ellipsoid center (mesh centroid in mesh-local frame).
    Eigen::Vector3d centroid_offset = Eigen::Vector3d::Zero();
    // Original mesh URI and scale for ELLIPSOID obstacles fitted from a mesh, used for ghost visualization.
    // Empty for non-mesh-derived ellipsoids.
    std::string source_mesh_resource;
    Eigen::Vector3d source_mesh_scale = Eigen::Vector3d::Ones();

    ObstacleGeometry() = default;
    // Existing 4-arg constructor — unchanged, semi-axes default to 0
    ObstacleGeometry(double radius, double length, double width, double height)
      : radius(radius), length(length), width(width), height(height) {}
    // Ellipsoid constructor (axis-aligned, centered at origin)
    ObstacleGeometry(double semi_x, double semi_y, double semi_z, bool /*ellipsoid_tag*/)
      : semi_x(semi_x), semi_y(semi_y), semi_z(semi_z) {}
    // Ellipsoid constructor with PCA axes and centroid offset
    ObstacleGeometry(double semi_x, double semi_y, double semi_z,
      const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroid)
      : semi_x(semi_x), semi_y(semi_y), semi_z(semi_z),
        ellipsoid_axes(axes), centroid_offset(centroid) {}

    bool operator==(const ObstacleGeometry& other) const {
      return radius == other.radius && length == other.length &&
        width == other.width && height == other.height &&
        semi_x == other.semi_x && semi_y == other.semi_y && semi_z == other.semi_z &&
        centroid_offset == other.centroid_offset &&
        ellipsoid_axes == other.ellipsoid_axes;
    }
    bool operator!=(const ObstacleGeometry& other) const {
      return !(*this == other);
    }

    std::vector<double> asVector(const ObstacleType& type) const {
      switch (type) {
      case ObstacleType::SPHERE:
        return {radius};
      case ObstacleType::BOX:
        return {length, width, height};
      case ObstacleType::CYLINDER:
        return {radius, height};
      case ObstacleType::MESH:
        return {length, width, height};
      case ObstacleType::ELLIPSOID:
        return {semi_x, semi_y, semi_z};
      case ObstacleType::CAPSULE:
        return {radius, height};
      default:
        throw std::invalid_argument("Unknown obstacle type");
      }
    }
  };

  class PotentialFieldObstacle {
  public:
    PotentialFieldObstacle() = delete;
    ~PotentialFieldObstacle() = default;

    PotentialFieldObstacle(
      std::string frameID,
      Eigen::Vector3d centerPosition, Eigen::Quaterniond orientation,
      ObstacleType type, ObstacleGroup group, ObstacleGeometry geometry,
      const std::string& meshResource = std::string(),
      const Eigen::Vector3d& meshScale = Eigen::Vector3d::Ones())
      : frameID(frameID),
      position(centerPosition),
      orientation(orientation),
      orientationConjugate(orientation.conjugate()),
      type(type),
      group(group),
      geometry(geometry),
      meshResource(meshResource),
      meshScale(meshScale) {
      if (type == ObstacleType::MESH && !meshResource.empty()) {
        try {
          meshCollisionData = loadMesh(meshResource);
        }
        catch (const std::exception&) {
          meshCollisionData.reset();
        }
      }
      // Only attempt to create the COAL object if we have valid data.
      // For MESH types, this means meshCollisionData must be loaded.
      if (type != ObstacleType::MESH || meshCollisionData) {
        this->toCoalCollisionObject();
      }
    }

    PotentialFieldObstacle(const PotentialFieldObstacle& other) :
      frameID(other.frameID),
      position(other.position),
      orientation(other.orientation),
      orientationConjugate(other.orientationConjugate),
      type(other.type),
      group(other.group),
      geometry(other.geometry),
      meshResource(other.meshResource),
      meshScale(other.meshScale) {
      // Shallow copy of shared mesh data
      this->meshCollisionData = other.meshCollisionData;
      this->toCoalCollisionObject();
    }

    PotentialFieldObstacle(PotentialFieldObstacle&& other) noexcept :
      frameID(std::move(other.frameID)),
      position(std::move(other.position)),
      orientation(std::move(other.orientation)),
      orientationConjugate(std::move(other.orientationConjugate)),
      type(other.type),
      group(other.group),
      geometry(std::move(other.geometry)),
      meshResource(std::move(other.meshResource)),
      meshScale(std::move(other.meshScale)) {
      this->meshCollisionData = std::move(other.meshCollisionData);
      this->toCoalCollisionObject();
    }

    PotentialFieldObstacle& operator=(const PotentialFieldObstacle& other) {
      if (this != &other) {
        this->frameID = other.frameID;
        this->position = other.position;
        this->orientation = other.orientation;
        this->orientationConjugate = other.orientationConjugate;
        this->type = other.type;
        this->group = other.group;
        this->geometry = other.geometry;
        this->meshResource = other.meshResource;
        this->meshScale = other.meshScale;
        this->meshCollisionData = other.meshCollisionData; // share existing (may be null)
        this->toCoalCollisionObject();
      }
      return *this;
    }

    bool operator==(const PotentialFieldObstacle& other) const {
      return this->position == other.position && this->geometry == other.geometry;
    }
    bool operator!=(const PotentialFieldObstacle& other) const { return !(*this == other); }

    std::string getFrameID() const { return this->frameID; }
    ObstacleGroup getGroup() const { return this->group; }
    Eigen::Vector3d getPosition() const { return this->position; }
    Eigen::Quaterniond getOrientation() const { return this->orientation; }
    ObstacleType getType() const { return this->type; }
    ObstacleGeometry getGeometry() const { return this->geometry; }
    const std::string& getMeshResource() const { return this->meshResource; }
    Eigen::Vector3d getMeshScale() const { return this->meshScale; }
    std::shared_ptr<coal::CollisionObject> getCoalCollisionObject() const { return this->coalCollisionObject; }

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

    void setMeshProperties(const std::string& meshResource, const Eigen::Vector3d& meshScale) {
      this->meshResource = meshResource;
      this->meshScale = meshScale;
      if (type == ObstacleType::MESH && !meshResource.empty()) {
        try {
          meshCollisionData = loadMesh(meshResource);
        }
        catch (const std::exception&) {
          meshCollisionData.reset();
        }
      }
    }

    void setMeshCollisionData(const std::shared_ptr<MeshCollisionData>& meshData) { this->meshCollisionData = meshData; }

    bool withinInfluenceZone(Eigen::Vector3d worldPoint, double influenceDistance) const;
    bool withinObstacle(Eigen::Vector3d worldPoint) const;

    /**
     * @brief Rotate the given point to the obstacle's frame of reference
     *
     * @note Useful for computing distances and collision checks in the obstacle's local frame
     *
     * @param worldPoint the point to transform into obstacle frame
     * @return Eigen::Vector3d the point in the obstacle's local frame
     */
    Eigen::Vector3d toObstacleFrame(const Eigen::Vector3d& worldPoint) const;

    /**
     * @brief Get the half dimensions of the obstacle
     *
     * @note Used for collision checking for "radius" style measurements
     *
     * @return Eigen::Vector3d Half dimensions [half_length, half_width, half_height]
     */
    Eigen::Vector3d halfDimensions() const;

    /**
     * @brief Computes the signed distance and normal from a world point to the obstacle surface
     *
     * @param[in] worldPoint The point in world coordinates
     * @param[out] signedDistance The distance to the obstacle surface (negative if inside)
     * @param[out] normal The outward normal vector at the closest point on the obstacle surface
     */
    void computeSignedDistanceAndNormal(const Eigen::Vector3d& worldPoint, double& signedDistance, Eigen::Vector3d& normal) const;

    /**
     * @brief Creates a COAL collision object from this obstacle for distance/collision queries
     *
     * @return std::shared_ptr<coal::CollisionObject> The COAL collision object representing this obstacle
     */
    std::shared_ptr<coal::CollisionObject> toCoalCollisionObject();

    /**
     * @brief Updates the COAL collision object pose to match the obstacle's current pose
     *
     */
    void updateCoalCollisionObjectPose() const;

  private:
    std::string frameID; // Frame ID for the obstacle
    Eigen::Vector3d position; // Center Position of the obstacle in 3D space
    Eigen::Quaterniond orientation; // Orientation of the obstacle in 3D space
    Eigen::Quaterniond orientationConjugate; // Cached conjugate of the orientation for efficiency
    ObstacleType type; // Type of the obstacle
    ObstacleGroup group; // Obstacle group/category
    ObstacleGeometry geometry; // Geometry of the obstacle, containing relevant dimensions
    std::string meshResource; // URI or file path to the mesh resource (e.g., package://, file://)
    Eigen::Vector3d meshScale; // Scale for mesh visualization if using MESH_RESOURCE
    std::shared_ptr<MeshCollisionData> meshCollisionData; // populated when mesh resource loaded (shared to avoid deep copies)
    std::shared_ptr<coal::CollisionObject> coalCollisionObject; // Cached COAL collision object for distance queries
    std::shared_ptr<coal::CollisionGeometry> coalCollisionGeometry; // Cached COAL collision geometry for distance queries
  };

  struct PotentialFieldObstacleHash {
    std::size_t operator()(const PotentialFieldObstacle& obstacle) const {
      return std::hash<std::string>()(obstacle.getFrameID()) ^
        std::hash<std::string>()(obstacleTypeToString(obstacle.getType())) ^
        std::hash<std::string>()(obstacleGroupToString(obstacle.getGroup()));
    }
  };

  static int inline createHashID(const PotentialFieldObstacle& obstacle) {
    PotentialFieldObstacleHash hasher;
    return static_cast<int>(hasher(obstacle) & 0x7FFFFFFF); // keep positive
  }

} // namespace pfield

#endif // PF_OBSTACLE_HPP
