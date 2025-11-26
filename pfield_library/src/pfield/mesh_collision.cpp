#include "pfield_library/pfield/mesh_collision.hpp"

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <functional>

#include <coal/shape/geometric_shapes.h>
#include <coal/math/transform.h>

// Assimp includes
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

constexpr double POINT_SPHERE_RADIUS = 1e-9;

static URIResolver g_uriResolver = nullptr;

void setURIResolver(URIResolver resolver) {
  g_uriResolver = resolver;
}

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

  std::string path = uri;
  // Use the custom resolver if available
  if (g_uriResolver) {
    path = g_uriResolver(uri);
  } else {
    // Fallback: Handle "package://" or "file://" prefixes
    if (path.find("file://") == 0) {
        path = path.substr(7);
    } else if (path.find("package://") == 0) {
        // Warn that package:// is not fully supported without ROS
        std::cerr << "Warning: package:// URI used in ROS-agnostic build. Assuming path relative to current directory or absolute path after stripping prefix." << std::endl;
        // Naive stripping: remove package://
        path = path.substr(10); 
    }
  }

  Assimp::Importer importer;
  // aiProcess_Triangulate is important because we need triangles
  const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
      throw std::runtime_error("Failed to load mesh: " + uri + " (" + importer.GetErrorString() + ")");
  }

  auto model = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
  std::vector<coal::Vec3s> verts;
  std::vector<coal::Triangle> tris;

  size_t vertex_offset = 0;
  
  // Recursive function to process nodes
  std::function<void(const aiNode*, const aiMatrix4x4&)> processNode;
  processNode = [&](const aiNode* node, const aiMatrix4x4& parentTransform) {
      aiMatrix4x4 transform = parentTransform * node->mTransformation;
      
      for (unsigned int m = 0; m < node->mNumMeshes; ++m) {
          aiMesh* mesh = scene->mMeshes[node->mMeshes[m]];
          
          // Apply transform to vertices
          for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
              aiVector3D v = mesh->mVertices[i];
              v *= transform; // Transform vertex
              verts.emplace_back(v.x, v.y, v.z);
          }
          
          for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
              aiFace face = mesh->mFaces[i];
              if (face.mNumIndices == 3) {
                  tris.emplace_back(vertex_offset + face.mIndices[0], vertex_offset + face.mIndices[1], vertex_offset + face.mIndices[2]);
              }
          }
          vertex_offset += mesh->mNumVertices;
      }
      
      for (unsigned int i = 0; i < node->mNumChildren; ++i) {
          processNode(node->mChildren[i], transform);
      }
  };

  processNode(scene->mRootNode, aiMatrix4x4());

  model->beginModel();
  model->addSubModel(verts, tris);
  model->endModel();

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (auto& v : verts) centroid += Eigen::Vector3d(v[0], v[1], v[2]);
  if (!verts.empty()) centroid /= static_cast<double>(verts.size());

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
  result->collisionObject = std::make_shared<coal::CollisionObject>(result->bvh, coal::Transform3s::Identity());
  // delete shapeMesh; // No longer needed as we use Assimp
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
  // Odd -> inside
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
  const coal::CollisionObject* meshObjPtr = meshData.collisionObject ? meshData.collisionObject.get() : nullptr;
  if (!meshObjPtr) {
    coal::CollisionObject temp(meshData.bvh, coal::Transform3s::Identity());
    coal::distance(&temp, &ptObj, req, res);
  }
  else {
    coal::distance(meshObjPtr, &ptObj, req, res);
  }
  return res.min_distance;
}

bool getClosestPointOnMesh(const MeshCollisionData& meshData, const Eigen::Vector3d& pointInMeshFrame,
  Eigen::Vector3d& closestPoint) {
  if (!meshData.collisionObject) return false;

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
  coal::CollisionObject sphereObj(
    std::shared_ptr<coal::CollisionGeometry>(&sphere, [](coal::CollisionGeometry*) {}),
    Ts
  );

  // Distance query setup
  coal::DistanceRequest req;
  req.enable_nearest_points = true;
  req.rel_err = 0.0f;
  req.abs_err = 0.0f;

  coal::DistanceResult res;
  const coal::CollisionObject* meshObj = meshData.collisionObject.get();

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
