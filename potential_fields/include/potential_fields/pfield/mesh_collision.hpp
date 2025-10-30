#ifndef MESH_COLLISION_HPP
#define MESH_COLLISION_HPP

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Core>

#include <coal/BVH/BVH_model.h>
#include <coal/BV/OBBRSS.h>
#include <coal/collision_object.h>
#include <coal/math/transform.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shape_to_BVH_model.h>
#include <coal/data_types.h>

/**
 * @brief Contains the collision data for a mesh, including its BVH representation and AABB.
 *
 * @note AABB is axis-aligned bounding box, useful for quick rejection tests
 * @note BVH is bounding volume hierarchy used for efficient collision detection and distance queries
 *
 */
struct MeshCollisionData {
  // The BVH model of the mesh for collision queries
  std::shared_ptr<coal::BVHModel<coal::OBBRSS>> bvh;
  // Pre-created collision object for distance queries
  std::shared_ptr<coal::CollisionObject> meshObj;
  // Axis-aligned bounding box (AABB) of the mesh (minimum corner)
  Eigen::Vector3d aabbMin;
  // Axis-aligned bounding box (AABB) of the mesh (maximum corner)
  Eigen::Vector3d aabbMax;
  // Centroid of the mesh vertices
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  // Maximum radius from centroid to any vertex
  double radius{0.0};
  // Maximum extent of the mesh along any axis
  double maxExtent{0.0};
  // Vertices of the mesh
  std::vector<Eigen::Vector3d> vertices;
  // Triangles of the mesh (indices into vertices)
  std::vector<Eigen::Vector3i> triangles;
};

/**
 * @brief Loads a mesh from the given URI and returns a shared pointer to the MeshCollisionData.
 *
 * @param uri The URI of the mesh to load, e.g., "file://path/to/mesh.obj"
 * @return std::shared_ptr<MeshCollisionData> The loaded mesh collision data
 */
std::shared_ptr<MeshCollisionData> loadMesh(const std::string& uri);

/**
 * @brief Determines if a point is inside the mesh defined by the given MeshCollisionData
 *
 * @param meshData The mesh collision data
 * @param pointInMeshFrame The point to test, expressed in the mesh's local frame
 * @return true if the point is inside the mesh or not
 */
bool pointInsideMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame);

/**
 * @brief Computes the distance from a point to the mesh defined by the given MeshCollisionData
 *        including the sign (negative if inside, positive if outside)
 *
 * @param meshData The mesh collision data
 * @param pointInMeshFrame The point to compute the distance to, expressed in the mesh's local frame
 * @return double The unsigned distance from the point to the mesh
 */
double computeUnsignedDistanceToMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame);

/**
 * @brief Finds the closest point on the mesh to the given point
 *
 * @note Returns true if successful, false otherwise and populates closestPoint accordingly
 *
 * @param[in] meshData The mesh collision data
 * @param[in] pointInMeshFrame The point to find the closest point to, expressed in the mesh's local frame
 * @param[out] closestPoint Output parameter to hold the closest point on the mesh
 * @return true if closest point was successfully found
 */
bool getClosestPointOnMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame,
  Eigen::Vector3d& closestPoint);

#endif // MESH_COLLISION_HPP
