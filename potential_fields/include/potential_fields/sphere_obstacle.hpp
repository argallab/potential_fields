#ifndef SPHERE_OBSTACLE_HPP
#define SPHERE_OBSTACLE_HPP
#include "spatial_vector.hpp"

class SphereObstacle {
public: // Constructors and Destructor
  SphereObstacle() = default;
  SphereObstacle(int id, SpatialVector position, float radius, float influenceRadius, float repulsiveGain)
    : id(id),
    position(position),
    radius(radius),
    influenceRadius(influenceRadius),
    repulsiveGain(repulsiveGain) {
  }
  ~SphereObstacle() = default;

public: // Getters and Setters
  SpatialVector getPosition() const { return this->position; }
  float getRadius() const { return this->radius; }
  int getID() const { return this->id; }
  float getInfluenceRadius() const { return this->influenceRadius; }
  float getRepulsiveGain() const { return this->repulsiveGain; }

public: // Class Methods

  bool withinInfluenceRadius(SpatialVector pos) const {
    return (this->position.euclideanDistance(pos) <= this->influenceRadius);
  }

public: // Operator Overloads

  bool operator==(const SphereObstacle& other) const {
    return (this->position == other.position && this->radius == other.radius);
  }
  bool operator!=(const SphereObstacle& other) const {
    return !(*this == other);
  }

private: // Private Members
  int id = 0; // Unique ID for the obstacle
  SpatialVector position; // Center Position of the obstacle in 3D space
  float radius; // Sphere's radius [m]
  float influenceRadius; // Sphere's influence radius [m]
  float repulsiveGain = 1.0f; // Gain for repulsive force
};

#endif // SPHERE_OBSTACLE_HPP