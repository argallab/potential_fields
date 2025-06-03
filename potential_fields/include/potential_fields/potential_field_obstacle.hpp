#ifndef POTENTIAL_FIELD_OBSTACLE_HPP
#define POTENTIAL_FIELD_OBSTACLE_HPP
#include "spatial_vector.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>

enum class ObstacleType {
  SPHERE, // [center, radius]
  BOX, // [center, length, width, height]
  CYLINDER // [center, radius, height]
};

std::string obstacleTypeToString(const ObstacleType& type) {
  switch (type) {
  case ObstacleType::SPHERE:
    return "Sphere";
  case ObstacleType::BOX:
    return "Box";
  case ObstacleType::CYLINDER:
    return "Cylinder";
  default:
    throw std::invalid_argument("Unknown obstacle type");
  }
}

ObstacleType stringToObstacleType(const std::string& typeStr) {
  if (typeStr == "Sphere") {
    return ObstacleType::SPHERE;
  } else if (typeStr == "Box") {
    return ObstacleType::BOX;
  } else if (typeStr == "Cylinder") {
    return ObstacleType::CYLINDER;
  } else {
    throw std::invalid_argument("Unknown obstacle type string: " + typeStr);
  }
}


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

  std::vector<double> asVector(const ObstacleType& type) const {
    switch (type) {
    case ObstacleType::SPHERE:
      return {radius};
    case ObstacleType::BOX:
      return {length, width, height};
    case ObstacleType::CYLINDER:
      return {radius, height};
    default:
      throw std::invalid_argument("Unknown obstacle type");
    }
  }
};

class PotentialFieldObstacle {
public:
  PotentialFieldObstacle() = delete;
  PotentialFieldObstacle(int id, Eigen::Vector3d centerPosition, Eigen::Quaterniond orientation,
    ObstacleType type, ObstacleGeometry geometry, double influenceZoneScale, double repulsiveGain)
    : id(id),
    position(centerPosition),
    orientation(orientation),
    orientationConjugate(orientation.conjugate()),
    type(type),
    geometry(geometry),
    influenceZoneScale(influenceZoneScale),
    repulsiveGain(repulsiveGain) {
  }
  ~PotentialFieldObstacle() = default;

  int getID() const { return this->id; }
  Eigen::Vector3d getPosition() const { return this->position; }
  Eigen::Quaterniond getOrientation() const { return this->orientation; }
  ObstacleType getType() const { return this->type; }
  ObstacleGeometry getGeometry() const { return this->geometry; }
  double getInfluenceZoneScale() const { return this->influenceZoneScale; }
  double getRepulsiveGain() const { return this->repulsiveGain; }

  void setPosition(Eigen::Vector3d newPosition) { this->position = newPosition; }
  void setOrientation(Eigen::Quaterniond newOrientation) {
    this->orientation = newOrientation;
    this->orientationConjugate = newOrientation.conjugate();
  }

  bool withinInfluenceZone(Eigen::Vector3d pos) const {
    Eigen::Vector3d localPos = this->toObstacleFrame(pos);
    // influenceZoneScale is the scalar that applies to each dimension of the obstacle's geometry
    switch (this->type) {
    case ObstacleType::SPHERE: {
      double distance = localPos.norm();
      return distance <= (this->geometry.radius * this->influenceZoneScale);
    }
    case ObstacleType::BOX: {
      Eigen::Vector3d halfDimensions = this->halfDimensions() * this->influenceZoneScale;
      return (localPos.array().abs() <= halfDimensions.array()).all();
    }
    case ObstacleType::CYLINDER: {
      // distance in XY plane from point to cylinder center Z axis
      double distance = std::sqrt(std::pow(localPos.x(), 2) + std::pow(localPos.y(), 2));
      return distance <= (this->geometry.radius * this->influenceZoneScale) &&
        std::abs(localPos.z()) <= (this->geometry.height * this->influenceZoneScale / 2.0);
    }
    default:
      // return false;
      throw std::invalid_argument("Unknown obstacle type");
    }
  }

  bool withinObstacle(Eigen::Vector3d pos) const {
    Eigen::Vector3d localPos = this->toObstacleFrame(pos);
    switch (this->type) {
    case ObstacleType::SPHERE: {
      double distance = localPos.norm();
      return distance <= this->geometry.radius;
    }
    case ObstacleType::BOX: {
      Eigen::Vector3d halfDimensions = this->halfDimensions();
      return (localPos.array().abs() <= halfDimensions.array()).all();
    }
    case ObstacleType::CYLINDER: {
      // distance in XY plane from point to cylinder center Z axis
      double distance = std::sqrt(std::pow(localPos.x(), 2) + std::pow(localPos.y(), 2));
      return distance <= this->geometry.radius &&
        std::abs(localPos.z()) <= (this->geometry.height / 2.0);
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
  Eigen::Quaterniond orientation; // Orientation of the obstacle in 3D space
  Eigen::Quaterniond orientationConjugate; // Cached conjugate of the orientation for efficiency
  ObstacleType type; // Type of the obstacle
  ObstacleGeometry geometry; // Geometry of the obstacle, containing relevant dimensions
  double influenceZoneScale; // The scale value of the volume of the obstacle that becomes the influence zone for repulsive forces
  double repulsiveGain; // Gain for the repulsive force

  Eigen::Vector3d toObstacleFrame(const Eigen::Vector3d& point) const {
    // Rotate the point to the obstacle's frame of reference
    return this->orientationConjugate * (point - this->position);
  }

  Eigen::Vector3d halfDimensions() const {
    // Returns half the dimensions of the obstacle based on its geometry
    switch (this->type) {
    case ObstacleType::SPHERE:
      return Eigen::Vector3d(this->geometry.radius, this->geometry.radius, this->geometry.radius);
    case ObstacleType::BOX:
      return Eigen::Vector3d(this->geometry.length / 2.0, this->geometry.width / 2.0, this->geometry.height / 2.0);
    case ObstacleType::CYLINDER:
      return Eigen::Vector3d(this->geometry.radius, this->geometry.radius, this->geometry.height / 2.0);
    default:
      throw std::invalid_argument("Unknown obstacle type");
    }
  }
};

#endif // POTENTIAL_FIELD_OBSTACLE_HPP
