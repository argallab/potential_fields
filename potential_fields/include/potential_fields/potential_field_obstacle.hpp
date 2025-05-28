#ifndef POTENTIAL_FIELD_OBSTACLE_HPP
#define POTENTIAL_FIELD_OBSTACLE_HPP
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>


enum class ObstacleType {
  SPHERE, // [center, radius]
  BOX, // [center, length, width, height]
  CYLINDER // [center, radius, height]
};

struct ObstacleGeometry {
  double radius = 0.0; // Radius for sphere and cylinder, unused for box
  double length = 0.0; // Length for box, unused for sphere and cylinder
  double width = 0.0; // Width for box, unused for sphere and cylinder
  double height = 0.0; // Height for cylinder and box, unused for sphere

  bool operator==(const ObstacleGeometry& other) const {
    return radius == other.radius && length == other.length &&
      width == other.width && height == other.height;
  }
  bool operator!=(const ObstacleGeometry& other) const {
    return !(*this == other);
  }
};

class PotentialFieldObstacle {
public:
  PotentialFieldObstacle() = delete;
  PotentialFieldObstacle(int id, Eigen::Vector3d centerPosition, ObstacleType type, ObstacleGeometry geometry, double influenceZoneScale, double repulsiveGain)
    : id(id),
    position(centerPosition),
    type(type),
    geometry(geometry),
    influenceZoneScale(influenceZoneScale),
    repulsiveGain(repulsiveGain) {
  }
  ~PotentialFieldObstacle() = default;

  int getID() const { return this->id; }
  Eigen::Vector3d getPosition() const { return this->position; }
  ObstacleType getType() const { return this->type; }
  ObstacleGeometry getGeometry() const { return this->geometry; }
  double getInfluenceZoneScale() const { return this->influenceZoneScale; }
  double getRepulsiveGain() const { return this->repulsiveGain; }

  bool withinInfluenceZone(Eigen::Vector3d pos) const {
    // influenceZoneScale is the scalar that applies to each dimension of the obstacle's geometry
    switch (this->type) {
    case ObstacleType::SPHERE: {
      double distance = (pos - this->position).norm();
      return distance <= (this->geometry.radius * this->influenceZoneScale);
    }
    case ObstacleType::BOX: {
      Eigen::Vector3d halfDimensions(this->geometry.length * this->influenceZoneScale / 2.0,
        this->geometry.width * this->influenceZoneScale / 2.0,
        this->geometry.height * this->influenceZoneScale / 2.0);
      return (pos.array() >= (this->position - halfDimensions).array()).all() &&
        (pos.array() <= (this->position + halfDimensions).array()).all();
    }
    case ObstacleType::CYLINDER: {
      // distance in XY plane from point to cylinder center Z axis
      double distance = std::sqrt(std::pow(pos.x() - this->position.x(), 2) + std::pow(pos.y() - this->position.y(), 2));
      return distance <= (this->geometry.radius * this->influenceZoneScale) &&
        std::abs(pos.z() - this->position.z()) <= (this->geometry.height * this->influenceZoneScale / 2.0);
    }
    }
  }

  bool withinObstacle(Eigen::Vector3d pos) const {
    switch (this->type) {
    case ObstacleType::SPHERE: {
      double distance = (pos - this->position).norm();
      return distance <= this->geometry.radius;
    }
    case ObstacleType::BOX: {
      Eigen::Vector3d halfDimensions(this->geometry.length / 2.0, this->geometry.width / 2.0, this->geometry.height / 2.0);
      return (pos.array() >= (this->position - halfDimensions).array()).all() &&
        (pos.array() <= (this->position + halfDimensions).array()).all();
    }
    case ObstacleType::CYLINDER: {
      // distance in XY plane from point to cylinder center Z axis
      double distance = std::sqrt(std::pow(pos.x() - this->position.x(), 2) + std::pow(pos.y() - this->position.y(), 2));
      return distance <= this->geometry.radius && std::abs(pos.z() - this->position.z()) <= (this->geometry.height / 2.0);
    }
    default:
      // return false;
      throw std::invalid_argument("Unknown obstacle type");
    }
  }

  bool operator==(const PotentialFieldObstacle& other) const {
    return  this->position == other.position && this->geometry == other.geometry;
  }

  bool operator!=(const PotentialFieldObstacle& other) const {
    return !(*this == other);
  }

private:
  // Private Members
  int id = 0; // Unique ID for the obstacle
  Eigen::Vector3d position; // Center Position of the obstacle in 3D space
  ObstacleType type; // Type of the obstacle
  ObstacleGeometry geometry; // Geometry of the obstacle, containing relevant dimensions
  double influenceZoneScale; // The scale value of the volume of the obstacle that becomes the influence zone for repulsive forces
  double repulsiveGain; // Gain for the repulsive force
};

#endif // POTENTIAL_FIELD_OBSTACLE_HPP
