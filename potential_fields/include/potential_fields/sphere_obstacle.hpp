#ifndef SPHERE_OBSTACLE_HPP
#define SPHERE_OBSTACLE_HPP
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>


class SphereObstacle {
public:
  SphereObstacle() = default;
  SphereObstacle(int id, Eigen::Vector3d position, double radius, double influenceRadius, double repulsiveGain)
    : id(id),
    position(position),
    radius(radius),
    influenceRadius(influenceRadius),
    repulsiveGain(repulsiveGain) {
  }
  ~SphereObstacle() = default;

  Eigen::Vector3d getPosition() const { return this->position; }
  double getRadius() const { return this->radius; }
  int getID() const { return this->id; }
  double getInfluenceRadius() const { return this->influenceRadius; }
  double getRepulsiveGain() const { return this->repulsiveGain; }

  bool withinInfluenceRadius(Eigen::Vector3d pos) const {
    double euclideanDistance = (pos - this->position).norm();
    return euclideanDistance <= this->influenceRadius;
  }

  bool withinRadius(Eigen::Vector3d pos) const {
    double euclideanDistance = (pos - this->position).norm();
    return euclideanDistance <= this->radius;
  }

  bool operator==(const SphereObstacle& other) const {
    return  this->position == other.position && this->radius == other.radius;
  }
  bool operator!=(const SphereObstacle& other) const {
    return !(*this == other);
  }

private:
  // Private Members
  int id = 0; // Unique ID for the obstacle
  Eigen::Vector3d position; // Center Position of the obstacle in 3D space
  double radius; // Sphere's radius [m]
  double influenceRadius; // Sphere's influence radius [m]
  double repulsiveGain = 1.0f; // Gain for repulsive force
};

#endif // SPHERE_OBSTACLE_HPP
