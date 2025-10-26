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

struct MeshCollisionData {
  std::shared_ptr<coal::BVHModel<coal::OBBRSS>> bvh;
  std::shared_ptr<coal::CollisionObject> meshObj;
  Eigen::Vector3d aabbMin, aabbMax;
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double maxExtent{0.0};
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
};

std::shared_ptr<MeshCollisionData> loadMesh(const std::string& uri);

bool pointInsideMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame);

double distanceToMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame);

#endif // MESH_COLLISION_HPP
