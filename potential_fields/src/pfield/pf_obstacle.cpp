#include "pfield/pf_obstacle.hpp"


PotentialFieldObstacle::PotentialFieldObstacle(int id,
  Eigen::Vector3d centerPosition, Eigen::Quaterniond orientation,
  ObstacleType type, ObstacleGeometry geometry,
  double influenceZoneScale, double repulsiveGain)
  : id(id),
  position(centerPosition),
  orientation(orientation),
  orientationConjugate(orientation.conjugate()),
  type(type),
  geometry(geometry),
  influenceZoneScale(influenceZoneScale),
  repulsiveGain(repulsiveGain) {}

bool PotentialFieldObstacle::withinInfluenceZone(Eigen::Vector3d pos) const {
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

bool PotentialFieldObstacle::withinObstacle(Eigen::Vector3d pos) const {
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

Eigen::Vector3d PotentialFieldObstacle::toObstacleFrame(const Eigen::Vector3d& point) const {
  // Rotate the point to the obstacle's frame of reference
  return this->orientationConjugate * (point - this->position);
}

Eigen::Vector3d PotentialFieldObstacle::halfDimensions() const {
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