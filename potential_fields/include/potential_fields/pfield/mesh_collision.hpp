#ifndef MESH_COLLISION_HPP
#define MESH_COLLISION_HPP
#include <Eigen/Core>
#include <vector>
#include <string>
#include <fcl/fcl.h>

struct MeshCollisionData {
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> bvh;
  std::shared_ptr<fcl::CollisionObjectd> meshObj;
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
