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


bool pointInsideMesh(const MeshCollisionData& meshData,
  const Eigen::Vector3d& pointInMeshFrame) {
  // --- Broad phase ---
  if ((pointInMeshFrame.array() < meshData.aabbMin.array()).any() ||
    (pointInMeshFrame.array() > meshData.aabbMax.array()).any()) {
    return false;
  }
  // EPS is tolerance for ray-triangle intersection tests
  const double EPS = 1e-9;

  Eigen::Vector3d rel = pointInMeshFrame - meshData.centroid;
  const double r2 = rel.squaredNorm();
  if (r2 > meshData.radius * meshData.radius + 1e-12) {
    return false;
  }

  // Optional: treat near-surface as inside to avoid flicker with signed distance logic.
  // If you have computeSignedDistanceToMesh available here, you can:
  // if (std::abs(computeSignedDistanceToMesh(meshData, pointInMeshFrame)) <= 1e-9) return true;

  // --- Ray cast parity test ---
  // Slightly perturbed +X ray to reduce degeneracy at edges/vertices.
  // (Keeps direction dominantly +X, but breaks symmetry.)
  Eigen::Vector3d dir(1.0, 1e-7, 2e-7);

  int crossings = 0;
  const auto& V = meshData.vertices;
  const auto& Tr = meshData.triangles;

  for (const auto& tri : Tr) {
    const Eigen::Vector3d& v0 = V[tri[0]];
    const Eigen::Vector3d& v1 = V[tri[1]];
    const Eigen::Vector3d& v2 = V[tri[2]];

    const Eigen::Vector3d e1 = v1 - v0;
    const Eigen::Vector3d e2 = v2 - v0;

    const Eigen::Vector3d h = dir.cross(e2);
    const double a = e1.dot(h);
    if (std::abs(a) < EPS) continue;  // Ray parallel to triangle.

    const double f = 1.0 / a;
    const Eigen::Vector3d s = pointInMeshFrame - v0;
    const double u = f * s.dot(h);
    if (u < -EPS || u > 1.0 + EPS) continue;

    const Eigen::Vector3d q = s.cross(e1);
    const double v = f * dir.dot(q);
    // Half-open rule to avoid double counting along shared edges:
    //  - v must be strictly greater than 0
    //  - u can be >= 0 (within eps)
    //  - u + v <= 1 (within eps)
    if (v <= EPS) continue;
    if (u + v > 1.0 + EPS) continue;

    const double t = f * e2.dot(q);
    if (t > EPS) {  // intersection in positive ray direction
      ++crossings;
    }
  }
  // Odd → inside
  return (crossings & 1) == 1;
}


double computeUnsignedDistanceToMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame) {
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

// Return false if COAL couldn't compute nearest points; 'closestOut' is mesh-frame.
bool getClosestPointOnMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame,
  Eigen::Vector3d& closestPoint) {
  if (!meshData.meshObj) return false;

  // Tiny sphere to query point-to-mesh distance with nearest points.
  const double eps = 1e-6; // small but > 0
  coal::Sphere sphere(eps);

  // COAL transform type is column-major float; we build a transform in mesh frame.
  coal::Transform3s Ts; // identity
  Ts.setIdentity();
  Ts.setTranslation(coal::Vec3s(static_cast<float>(pointInMeshFrame.x()),
    static_cast<float>(pointInMeshFrame.y()),
    static_cast<float>(pointInMeshFrame.z())));

  // Build a temporary collision object for the sphere
  coal::CollisionObject sphereObj(std::shared_ptr<coal::CollisionGeometry>(&sphere, [](coal::CollisionGeometry*) {}),
    Ts);

  // Distance query setup
  coal::DistanceRequest req;
  req.enable_nearest_points = true;
  req.rel_err = 0.0f;
  req.abs_err = 0.0f;

  coal::DistanceResult res;

  // Both objects (sphere, mesh) are already expressed in the same mesh frame.
  // NOTE: meshData.meshObj may have its own transform set; if your meshObj is not identity,
  // it's still OK: COAL uses each object's internal transform consistently and returns nearest
  // points in the WORLD frame derived from those. We will map back to mesh frame if needed.
  const coal::CollisionObject* meshObj = meshData.meshObj.get();

  // Perform the distance query
  auto dist = coal::distance(&sphereObj, meshObj, req, res);

  // We need the nearest point ON THE MESH. COAL returns two points:
  // res.nearest_points[0] on the sphere, res.nearest_points[1] on the mesh, both in "world".
  // Because the sphere is placed in mesh frame, and meshObj transform is also in mesh frame
  // (or applied consistently to both), we can treat res.nearest_points[1] as "mesh frame".
  if (!std::isfinite(dist)) return false;

  // Extract mesh nearest point (res stores Vec3f)
  const coal::Vec3s pm = res.nearest_points[1];
  closestPoint = Eigen::Vector3d(pm[0], pm[1], pm[2]);
  return true;
}
