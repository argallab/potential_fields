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

#ifndef PF_OBSTACLE_GEOMETRY_HPP_
#define PF_OBSTACLE_GEOMETRY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <coal/collision_object.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

namespace pfield {

  // Forward-declared so geometry subclasses can reference it without a circular include.
  enum class ObstacleType;

  // ============================================================================
  // Abstract base class
  // ============================================================================

  /**
   * @brief Abstract base class for obstacle geometry.
   *
   * Each concrete subclass fully describes one obstacle shape (Sphere, Box,
   * Cylinder, Ellipsoid, Capsule, Mesh) and implements the geometry-specific
   * operations needed by PotentialFieldObstacle: signed-distance / normal
   * computation, half-dimension queries, Coal shape construction, and control-
   * point generation.
   *
   * All operations that depend on the world-frame obstacle pose (position +
   * orientation) are passed in as parameters so that the geometry object
   * remains pose-independent and can be reused after a pose update.
   */
  class ObstacleGeometry {
  public:
    virtual ~ObstacleGeometry() = default;

    /// Returns the ObstacleType discriminator for this geometry.
    virtual ObstacleType getType() const = 0;

    /**
     * @brief Returns a vector of the geometry's defining parameters.
     *
     * The vector layout is type-specific:
     *   Sphere    → {radius}
     *   Box       → {length, width, height}
     *   Cylinder  → {radius, height}
     *   Ellipsoid → {semi_x, semi_y, semi_z}
     *   Capsule   → {radius, height}
     *   Mesh      → {length, width, height}  (bounding-box dims or zeros)
     */
    virtual std::vector<double> asVector() const = 0;

    /**
     * @brief Returns the half-dimensions of the obstacle's axis-aligned bounding box.
     *
     * Used for broad-phase influence-zone checks.
     *
     * @param meshScale  Scale of the mesh resource (used only by MeshGeometry;
     *                   ignored by all other types).
     */
    virtual Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const = 0;

    /**
     * @brief Computes the signed distance and outward surface normal from a
     *        point expressed in the obstacle's local frame.
     *
     * The caller is responsible for transforming the world-frame query point
     * into the obstacle's local frame before calling this method.
     *
     * @param[in]  localPoint      Query point in the obstacle's local frame.
     * @param[in]  obstacleOrientation  World-frame orientation of the obstacle
     *             (quaternion), used to rotate the computed normal back to
     *             world coordinates.
     * @param[out] signedDistance  Signed distance to the obstacle surface
     *             (negative if inside, positive if outside).
     * @param[out] normal          Outward unit normal in world coordinates at
     *             the closest surface point.
     */
    virtual void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const = 0;

    /**
     * @brief Builds the Coal collision geometry for this obstacle.
     *
     * The returned shared_ptr is used to construct a coal::CollisionObject.
     * The caller is responsible for supplying the correct world-frame transform.
     *
     * @param obstaclePosition     World-frame position of the obstacle origin.
     * @param obstacleOrientation  World-frame orientation of the obstacle.
     * @param[out] coalTransform   The Coal transform that should be passed to
     *             coal::CollisionObject together with the returned geometry.
     *             For most types this equals the obstacle pose, but OBB-fitted
     *             shapes (Box, Capsule) may shift and rotate it to the OBB center.
     * @return Shared pointer to the Coal collision geometry.
     */
    virtual std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const = 0;

    /**
     * @brief Computes the world-frame Coal transform to use when updating a
     *        cached collision object's pose.
     *
     * For most types this simply encodes (obstaclePosition, obstacleOrientation).
     * OBB-fitted types (Box, Capsule) adjust for the centroid offset and PCA
     * rotation baked into the Coal object at construction time.
     *
     * @param obstaclePosition     New world-frame position of the obstacle.
     * @param obstacleOrientation  New world-frame orientation of the obstacle.
     * @return The Coal transform to apply to the cached collision object.
     */
    virtual coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const = 0;

    /// Equality comparison — only types with the same concrete class and parameters are equal.
    virtual bool operator==(const ObstacleGeometry& other) const = 0;
    bool operator!=(const ObstacleGeometry& other) const { return !(*this == other); }

    /// Deep-copy factory, so PotentialFieldObstacle can clone its geometry.
    virtual std::unique_ptr<ObstacleGeometry> clone() const = 0;

  protected:
    ObstacleGeometry() = default;
    ObstacleGeometry(const ObstacleGeometry&) = default;
    ObstacleGeometry& operator=(const ObstacleGeometry&) = default;
  };

  // ============================================================================
  // SphereGeometry
  // ============================================================================

  /**
   * @brief Geometry for a sphere obstacle.
   *
   * Defined by a single radius. The sphere is centered at the obstacle origin.
   *
   * SDF:   d(p) = ||p|| - radius
   * Normal: p / ||p||  (or UnitX when p ≈ 0)
   */
  class SphereGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs a SphereGeometry.
     * @param radius  Radius of the sphere (must be > 0).
     */
    explicit SphereGeometry(double radius);

    double radius;  ///< Sphere radius.

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

  // ============================================================================
  // BoxGeometry
  // ============================================================================

  /**
   * @brief Geometry for a box (axis-aligned or oriented bounding box) obstacle.
   *
   * Defined by full extents (length × width × height) along the local X, Y, Z
   * axes respectively. Supports OBB-fitted boxes (e.g. fitted to a mesh via PCA):
   * in that case @p axes holds the PCA rotation and @p centroidOffset holds the
   * displacement from the collision-origin frame to the OBB center.
   *
   * SDF: closest-point distance to the box surface (negative inside).
   */
  class BoxGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs an axis-aligned BoxGeometry.
     * @param length  Full extent along local X.
     * @param width   Full extent along local Y.
     * @param height  Full extent along local Z.
     */
    BoxGeometry(double length, double width, double height);

    /**
     * @brief Constructs an OBB-fitted BoxGeometry.
     * @param length         Full extent along the OBB X axis.
     * @param width          Full extent along the OBB Y axis.
     * @param height         Full extent along the OBB Z axis.
     * @param axes           Rotation from the obstacle frame to the OBB
     *                       principal-axis frame (columns are OBB axes).
     * @param centroidOffset Offset from the collision-origin to the OBB center,
     *                       expressed in the obstacle's local frame.
     */
    BoxGeometry(double length, double width, double height,
      const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset);

    double length;                              ///< Full extent along OBB X.
    double width;                               ///< Full extent along OBB Y.
    double height;                              ///< Full extent along OBB Z.
    Eigen::Matrix3d axes;                       ///< OBB rotation (identity for AA boxes).
    Eigen::Vector3d centroidOffset;             ///< OBB center offset (zero for AA boxes).

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

  // ============================================================================
  // CylinderGeometry
  // ============================================================================

  /**
   * @brief Geometry for a cylinder obstacle.
   *
   * Defined by a radius and a full height. The cylinder axis is aligned with
   * the local Z axis and is centered at the obstacle origin.
   *
   * SDF: composite of radial and axial distances to the curved surface and end caps.
   */
  class CylinderGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs a CylinderGeometry.
     * @param radius  Radius of the circular cross-section (must be > 0).
     * @param height  Full height of the cylinder along Z (must be > 0).
     */
    CylinderGeometry(double radius, double height);

    double radius;  ///< Cylinder radius.
    double height;  ///< Full cylinder height.

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

  // ============================================================================
  // EllipsoidGeometry
  // ============================================================================

  /**
   * @brief Geometry for an ellipsoid obstacle.
   *
   * Defined by three semi-axes (a, b, c) along the principal axes. Supports
   * PCA-rotated ellipsoids fitted to mesh geometry: @p axes holds the rotation
   * from the obstacle frame to the principal-axis frame, and @p centroidOffset
   * holds the mesh centroid displacement.
   *
   * SDF: first-order approximation  d ≈ f(p) / ||∇f(p)||  where
   *      f(p) = (px/a)² + (py/b)² + (pz/c)² - 1.
   */
  class EllipsoidGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs an axis-aligned EllipsoidGeometry centered at the origin.
     * @param semiX  Semi-axis along X (must be > 0).
     * @param semiY  Semi-axis along Y (must be > 0).
     * @param semiZ  Semi-axis along Z (must be > 0).
     */
    EllipsoidGeometry(double semiX, double semiY, double semiZ);

    /**
     * @brief Constructs a PCA-rotated EllipsoidGeometry.
     * @param semiX          Semi-axis along the PCA X principal axis.
     * @param semiY          Semi-axis along the PCA Y principal axis.
     * @param semiZ          Semi-axis along the PCA Z principal axis.
     * @param axes           Rotation from obstacle frame to PCA frame
     *                       (columns are principal axes).
     * @param centroidOffset Offset from the collision-origin to the ellipsoid
     *                       center, expressed in the obstacle's local frame.
     */
    EllipsoidGeometry(double semiX, double semiY, double semiZ,
      const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset);

    double semiX;                               ///< Semi-axis along principal X.
    double semiY;                               ///< Semi-axis along principal Y.
    double semiZ;                               ///< Semi-axis along principal Z.
    Eigen::Matrix3d axes;                       ///< PCA rotation (identity if axis-aligned).
    Eigen::Vector3d centroidOffset;             ///< Centroid offset (zero if centered).

    /// Mesh URI from which this ellipsoid was fitted (empty if not mesh-derived).
    std::string sourceMeshResource;
    /// Mesh scale corresponding to @p sourceMeshResource.
    Eigen::Vector3d sourceMeshScale = Eigen::Vector3d::Ones();

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

  // ============================================================================
  // CapsuleGeometry
  // ============================================================================

  /**
   * @brief Geometry for a capsule obstacle.
   *
   * A capsule is a cylinder of radius @p radius and shaft length @p height
   * capped at each end by a hemisphere of the same radius. The capsule axis
   * is aligned with the local Z axis; total length = height + 2 * radius.
   *
   * Supports PCA-fitted capsules via @p axes (OBB orientation, column 2 is the
   * capsule Z axis) and @p centroidOffset (offset to OBB center).
   *
   * SDF: distance from point to the nearest point on the capsule axis segment,
   *      minus the radius.
   */
  class CapsuleGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs a CapsuleGeometry.
     * @param radius  Radius of the cylinder shaft and hemispherical end caps.
     * @param height  Length of the cylindrical shaft (not including end caps).
     */
    CapsuleGeometry(double radius, double height);

    /**
     * @brief Constructs an OBB-fitted CapsuleGeometry.
     * @param radius         Radius of the shaft and end caps.
     * @param height         Shaft length (not including end caps).
     * @param axes           OBB rotation; column 2 is the capsule Z axis.
     * @param centroidOffset Offset from the collision-origin to the OBB center,
     *                       expressed in the obstacle's local frame.
     */
    CapsuleGeometry(double radius, double height,
      const Eigen::Matrix3d& axes, const Eigen::Vector3d& centroidOffset);

    double radius;                              ///< Shaft and end-cap radius.
    double height;                              ///< Shaft length (excluding end caps).
    Eigen::Matrix3d axes;                       ///< OBB rotation (identity by default).
    Eigen::Vector3d centroidOffset;             ///< OBB center offset (zero by default).

    /// Mesh URI from which this capsule was fitted (empty if not mesh-derived).
    std::string sourceMeshResource;
    /// Mesh scale corresponding to @p sourceMeshResource.
    Eigen::Vector3d sourceMeshScale = Eigen::Vector3d::Ones();

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

  // ============================================================================
  // MeshGeometry
  // ============================================================================

  /**
   * @brief Geometry for a triangle-mesh obstacle.
   *
   * Mesh obstacles delegate signed-distance and normal queries to per-triangle
   * ray / closest-point routines in mesh_collision.hpp. The Coal collision
   * object is built from a pre-loaded BVH rather than a parametric shape.
   *
   * @note MeshGeometry holds a non-owning pointer to the MeshCollisionData
   *       loaded by PotentialFieldObstacle; it does not manage its lifetime.
   */
  class MeshGeometry : public ObstacleGeometry {
  public:
    /**
     * @brief Constructs a MeshGeometry.
     *
     * @param meshCollisionData  Shared pointer to the loaded mesh data.
     *                           Must be non-null before calling
     *                           computeSignedDistanceAndNormal() or
     *                           buildCoalGeometry().
     */
    explicit MeshGeometry(std::shared_ptr<struct MeshCollisionData> meshCollisionData);

    std::shared_ptr<struct MeshCollisionData> meshCollisionData;  ///< Loaded mesh data.

    ObstacleType getType() const override;
    std::vector<double> asVector() const override;
    Eigen::Vector3d halfDimensions(const Eigen::Vector3d& meshScale) const override;
    void computeSignedDistanceAndNormal(
      const Eigen::Vector3d& localPoint,
      const Eigen::Quaterniond& obstacleOrientation,
      double& signedDistance,
      Eigen::Vector3d& normal) const override;
    std::shared_ptr<coal::CollisionGeometry> buildCoalGeometry(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation,
      coal::Transform3s& coalTransform) const override;
    coal::Transform3s computeUpdatedCoalTransform(
      const Eigen::Vector3d& obstaclePosition,
      const Eigen::Quaterniond& obstacleOrientation) const override;
    bool operator==(const ObstacleGeometry& other) const override;
    std::unique_ptr<ObstacleGeometry> clone() const override;
  };

}  // namespace pfield

#endif  // PF_OBSTACLE_GEOMETRY_HPP_
