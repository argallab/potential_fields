#ifndef PF_OBSTACLE_HPP
#define PF_OBSTACLE_HPP
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

enum class ObstacleGroup {
  STATIC, // Static environment obstacles
  DYNAMIC, // Dynamic environment obstacles
  ROBOT // The Robot itself as an obstacle, used for self-collision avoidance
};

inline std::string obstacleTypeToString(const ObstacleType& type) {
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

inline ObstacleType stringToObstacleType(const std::string& typeStr) {
  if (typeStr == "Sphere") {
    return ObstacleType::SPHERE;
  }
  else if (typeStr == "Box") {
    return ObstacleType::BOX;
  }
  else if (typeStr == "Cylinder") {
    return ObstacleType::CYLINDER;
  }
  else {
    throw std::invalid_argument("Unknown obstacle type string: " + typeStr);
  }
}

inline std::string obstacleGroupToString(const ObstacleGroup& group) {
  switch (group) {
  case ObstacleGroup::STATIC:
    return "Static";
  case ObstacleGroup::DYNAMIC:
    return "Dynamic";
  case ObstacleGroup::ROBOT:
    return "Robot";
  default:
    throw std::invalid_argument("Unknown obstacle group");
  }
}

inline ObstacleGroup stringToObstacleGroup(const std::string& groupStr) {
  if (groupStr == "Static") {
    return ObstacleGroup::STATIC;
  }
  else if (groupStr == "Dynamic") {
    return ObstacleGroup::DYNAMIC;
  }
  else if (groupStr == "Robot") {
    return ObstacleGroup::ROBOT;
  }
  else {
    throw std::invalid_argument("Unknown obstacle group string: " + groupStr);
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
  PotentialFieldObstacle(int id,
    Eigen::Vector3d centerPosition, Eigen::Quaterniond orientation,
    ObstacleType type, ObstacleGroup group, ObstacleGeometry geometry,
    double influenceZoneScale, double repulsiveGain)
    : id(id),
    position(centerPosition),
    orientation(orientation),
    orientationConjugate(orientation.conjugate()),
    type(type),
    group(group),
    geometry(geometry),
    influenceZoneScale(influenceZoneScale),
    repulsiveGain(repulsiveGain) {}
  ~PotentialFieldObstacle() = default;

  int getID() const { return this->id; }
  ObstacleGroup getGroup() const { return this->group; }
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

  bool withinInfluenceZone(Eigen::Vector3d pos) const;

  bool withinObstacle(Eigen::Vector3d pos) const;

  bool operator==(const PotentialFieldObstacle& other) const {
    return  this->position == other.position && this->geometry == other.geometry;
  }

  bool operator!=(const PotentialFieldObstacle& other) const {
    return !(*this == other);
  }

private:
  int id; // Unique ID for the obstacle
  Eigen::Vector3d position; // Center Position of the obstacle in 3D space
  Eigen::Quaterniond orientation; // Orientation of the obstacle in 3D space
  Eigen::Quaterniond orientationConjugate; // Cached conjugate of the orientation for efficiency
  ObstacleType type; // Type of the obstacle
  ObstacleGroup group; // Obstacle group/category
  ObstacleGeometry geometry; // Geometry of the obstacle, containing relevant dimensions
  double influenceZoneScale; // The scale value of the volume of the obstacle that becomes the influence zone for repulsive forces
  double repulsiveGain; // Gain for the repulsive force

  Eigen::Vector3d toObstacleFrame(const Eigen::Vector3d& point) const;

  Eigen::Vector3d halfDimensions() const;
};

struct PotentialFieldObstacleHash {
  std::size_t operator()(const PotentialFieldObstacle& obstacle) const {
    return std::hash<int>()(obstacle.getID()) ^
      std::hash<std::string>()(obstacleTypeToString(obstacle.getType())) ^
      std::hash<std::string>()(obstacleGroupToString(obstacle.getGroup()));
  }
};

#endif // PF_OBSTACLE_HPP
