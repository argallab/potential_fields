#include "pfield/mesh_collision.hpp"

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>
#include <geometric_shapes/mesh_operations.h>      // createMeshFromResource
#include <geometric_shapes/shape_operations.h>     // shapes::...

constexpr double POINT_SPHERE_RADIUS = 1e-9;


// Simple cache so identical mesh resources share BVH + triangles
std::shared_ptr<MeshCollisionData> loadMesh(const std::string& uri) {
  if (uri.empty()) return nullptr;
  static std::unordered_map<std::string, std::weak_ptr<MeshCollisionData>> cache;
  static std::mutex cacheMutex;
  {
    std::lock_guard<std::mutex> lk(cacheMutex);
    auto it = cache.find(uri);
    if (it != cache.end()) {
      if (auto existing = it->second.lock()) {
        return existing;
      }
    }
  }
  shapes::Mesh* shapeMesh = shapes::createMeshFromResource(uri);
  if (!shapeMesh) throw std::runtime_error("Failed to load mesh: " + uri);

  auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
  std::vector<coal::Vec3s> verts;
  std::vector<coal::Triangle> tris;

  verts.reserve(shapeMesh->vertex_count);
  for (size_t i = 0; i < shapeMesh->vertex_count; ++i) {
    verts.emplace_back(shapeMesh->vertices[3 * i + 0],
      shapeMesh->vertices[3 * i + 1],
      shapeMesh->vertices[3 * i + 2]);
  }
  tris.reserve(shapeMesh->triangle_count);
  for (size_t i = 0; i < shapeMesh->triangle_count; ++i) {
    const unsigned int* t = &shapeMesh->triangles[3 * i];
    tris.emplace_back(t[0], t[1], t[2]);
  }

  model->beginModel();
  model->addSubModel(verts, tris);
  model->endModel();

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (auto& v : verts) centroid += Eigen::Vector3d(v[0], v[1], v[2]);
  centroid /= static_cast<double>(verts.size());

  Eigen::Vector3d minB = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d maxB = -minB;
  double maxRadiusSq = 0.0;
  for (auto& v : verts) {
    minB = minB.cwiseMin(Eigen::Vector3d(v[0], v[1], v[2]));
    maxB = maxB.cwiseMax(Eigen::Vector3d(v[0], v[1], v[2]));
    Eigen::Vector3d diff = Eigen::Vector3d(v[0], v[1], v[2]) - centroid;
    double d2 = diff.squaredNorm();
    if (d2 > maxRadiusSq) maxRadiusSq = d2;
  }
  double maxExtent = (maxB - minB).maxCoeff();

  auto result = std::make_shared<MeshCollisionData>();
  result->bvh = model;
  result->centroid = centroid;
  result->maxExtent = maxExtent;
  result->aabbMin = minB;
  result->aabbMax = maxB;
  result->radius = std::sqrt(maxRadiusSq);
  result->vertices.reserve(verts.size());
  for (auto& v : verts) result->vertices.emplace_back(v[0], v[1], v[2]);
  result->triangles.reserve(tris.size());
  for (auto& t : tris) result->triangles.emplace_back(t[0], t[1], t[2]);
  // Pre-create collision object for distance queries
  result->meshObj = std::make_shared<coal::CollisionObject>(result->bvh, coal::Transform3s::Identity());
  delete shapeMesh;
  {
    std::lock_guard<std::mutex> lk(cacheMutex);
    cache[uri] = result;
  }
  return result;
}


bool pointInsideMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame) {
  // Cheap broad-phase rejections: AABB then bounding sphere
  if ((pointInMeshFrame.array() < meshData.aabbMin.array()).any() ||
    (pointInMeshFrame.array() > meshData.aabbMax.array()).any()) {
    return false;
  }
  const double EPS = 1e-9;
  Eigen::Vector3d rel = pointInMeshFrame - meshData.centroid;
  if (rel.squaredNorm() > meshData.radius * meshData.radius) {
    return false;
  }
  else if (rel.squaredNorm() < EPS) {
    return true;
  }
  // Ray cast in +X direction using Möller–Trumbore intersection
  const Eigen::Vector3d dir(1, 0, 0);
  int crossings = 0;
  const auto& verts = meshData.vertices;
  const auto& triangles = meshData.triangles;
  for (const auto& tri : triangles) {
    const Eigen::Vector3d& v0 = verts[tri[0]];
    const Eigen::Vector3d& v1 = verts[tri[1]];
    const Eigen::Vector3d& v2 = verts[tri[2]];
    Eigen::Vector3d e1 = v1 - v0;
    Eigen::Vector3d e2 = v2 - v0;
    Eigen::Vector3d h = dir.cross(e2);
    double a = e1.dot(h);
    if (std::abs(a) < EPS) continue; // Parallel
    double f = 1.0 / a;
    Eigen::Vector3d s = pointInMeshFrame - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) continue;
    Eigen::Vector3d q = s.cross(e1);
    double v = f * dir.dot(q);
    if (v < 0.0 || (u + v) > 1.0) continue;
    double t = f * e2.dot(q);
    if (t > EPS) { // intersection distance along +X
      ++crossings;
    }
  }
  return (crossings % 2) == 1;
}

double distanceToMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame) {
  if (!meshData.bvh) return std::numeric_limits<double>::infinity();
  // Lazy create point sphere shape (radius approximately zero)
  static std::shared_ptr<coal::Sphere> sphere_ptr = std::make_shared<coal::Sphere>(POINT_SPHERE_RADIUS);
  coal::Transform3s pt_tf = coal::Transform3s::Identity();
  pt_tf.translation() = pointInMeshFrame;
  coal::CollisionObject ptObj(sphere_ptr, pt_tf);
  coal::DistanceRequest req;
  coal::DistanceResult res;
  const coal::CollisionObject* meshObjPtr = meshData.meshObj ? meshData.meshObj.get() : nullptr;
  if (!meshObjPtr) {
    coal::CollisionObject temp(meshData.bvh, coal::Transform3s::Identity());
    coal::distance(&temp, &ptObj, req, res);
  }
  else {
    coal::distance(meshObjPtr, &ptObj, req, res);
  }
  return res.min_distance;
}
