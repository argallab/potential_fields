/**
 * @file pfield.hpp
 * @author Sharwin Patil (sharwinpatil@u.northwestern.edu)
 * @brief Library for managing a vector field overlaid onto a
 *        robot's task-space (3D World) as a potential force field
 *        to plan motion trajectories.
 *
 * @details The field produces a task-space wrench (F) assuming a massless system: linear force [F] and torque [Nm]
 *          The wrench is converted to a velocity vector (V): linear velocity [m/s] and angular velocity [rad/s]
 *
 * @version 2.0
 * @date 2025-05-08
 *
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef PFIELDS_HPP
#define PFIELDS_HPP

#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "spatial_vector.hpp"
#include "pf_obstacle.hpp"
#include "pf_kinematics.hpp"
#include "pfield_common.hpp"

#include "robot_plugins/ik_solver.hpp"

struct TaskSpaceWrench {
  Eigen::Vector3d force{Eigen::Vector3d::Zero()}; // Linear Force [N]
  Eigen::Vector3d torque{Eigen::Vector3d::Zero()}; // Torque [Nm]

  TaskSpaceWrench(Eigen::Vector3d force, Eigen::Vector3d torque)
    : force(force), torque(torque) {}
};

struct TaskSpaceTwist {
  Eigen::Vector3d linearVelocity{Eigen::Vector3d::Zero()}; // Linear Velocity [m/s]
  Eigen::Vector3d angularVelocity{Eigen::Vector3d::Zero()}; // Angular Velocity [rad/s]

  TaskSpaceTwist() = default;
  TaskSpaceTwist(Eigen::Vector3d linearVelocity, Eigen::Vector3d angularVelocity)
    : linearVelocity(linearVelocity), angularVelocity(angularVelocity) {}

  Eigen::Vector3d getLinearVelocity() const { return this->linearVelocity; }
  Eigen::Vector3d getAngularVelocity() const { return this->angularVelocity; }
};

struct PlannedPath {
  std::vector<SpatialVector> poses; // End-effector pose
  std::vector<TaskSpaceTwist> twists; // End-effector velocity
  std::vector<std::vector<double>> jointAngles; // Joint angles for each point in the path [rad]
  std::vector<double> timeStamps; // Time stamps for each point in the path [s]
  unsigned int numPoints; // The number of points in the planned path, should be equal across all vectors
  double duration; // Total duration of the path [s]
  double dt; // Time difference between consecutive points [s]

  PlannedPath()
    : numPoints(0), duration(0.0), dt(0.0) {}

  void addPoint(const SpatialVector& pose, const TaskSpaceTwist& twist, std::vector<double> jointAngles, double timeStamp) {
    this->poses.push_back(pose);
    this->twists.push_back(twist);
    this->jointAngles.push_back(jointAngles);
    this->timeStamps.push_back(timeStamp);
    this->numPoints = static_cast<unsigned int>(this->poses.size());
    if (this->numPoints > 1) {
      this->dt = this->timeStamps.back() - this->timeStamps[this->numPoints - 2];
      this->duration = this->timeStamps.back() - this->timeStamps.front();
    }
    else {
      this->dt = 0.0;
      this->duration = 0.0;
    }
  }
};

// Default values
constexpr double DEFAULT_ATTRACTIVE_GAIN = 1.0; // Gain for attractive force [Ns/m]
constexpr double DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN = 0.7; // Gain for rotational attractive force [Ns/m]
constexpr double DEFAULT_MAX_LINEAR_VELOCITY = 5.0; // [m/s]
constexpr double DEFAULT_MAX_ANGULAR_VELOCITY = 1.0; // [rad/s]
constexpr double DEFAULT_MAX_LINEAR_ACCELERATION = 1.0; // [m/s^2]
constexpr double DEFAULT_MAX_ANGULAR_ACCELERATION = 1.0; // [rad/s^2]

class PotentialField {
public:
  // =========== Constructors and Operators ===========
  PotentialField()
    : attractiveGain(DEFAULT_ATTRACTIVE_GAIN),
    rotationalAttractiveGain(DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}
  ~PotentialField() = default;

  PotentialField(const PotentialField& other) :
    attractiveGain(other.attractiveGain),
    rotationalAttractiveGain(other.rotationalAttractiveGain),
    maxLinearVelocity(other.maxLinearVelocity),
    maxAngularVelocity(other.maxAngularVelocity),
    maxLinearAcceleration(other.maxLinearAcceleration),
    maxAngularAcceleration(other.maxAngularAcceleration),
    goalPose(other.goalPose),
    obstacles(other.obstacles) {}

  /**
   * @brief Constructs a PotentialField with the specified goal position.
   *        The attractive gain and rotational attractive gain are set to default values.
   *
   * @param goalPose The pose in 3D space that will generate an attractive force.
   */
  explicit PotentialField(SpatialVector goalPose) :
    attractiveGain(1.0),
    rotationalAttractiveGain(0.7),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    goalPose(goalPose) {}

  PotentialField(SpatialVector goalPose, double attractiveGain, double rotationalAttractiveGain) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    goalPose(goalPose) {}

  PotentialField(
    double attractiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}

  PotentialField(
    double attractiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity,
    double maxLinearAcceleration, double maxAngularAcceleration) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(maxLinearAcceleration),
    maxAngularAcceleration(maxAngularAcceleration),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}

  PotentialField(
    SpatialVector goalPose,
    double attractiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity,
    double maxLinearAcceleration, double maxAngularAcceleration) :
    attractiveGain(attractiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(maxLinearAcceleration),
    maxAngularAcceleration(maxAngularAcceleration),
    goalPose(goalPose) {}


  PotentialField& operator=(const PotentialField& other) {
    if (this != &other) {
      this->attractiveGain = other.attractiveGain;
      this->rotationalAttractiveGain = other.rotationalAttractiveGain;
      this->goalPose = other.goalPose;
      this->obstacles = other.obstacles;
    }
    return *this;
  }

  bool operator==(const PotentialField& other) const {
    auto obstaclesEqual = [this, &other]() -> bool {
      if (this->obstacles.size() != other.obstacles.size()) return false;
      // Convert both obstacle lists to unordered_sets for comparison
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> thisObstaclesSet(
        this->obstacles.cbegin(), this->obstacles.cend()
      );
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> otherObstaclesSet(
        other.obstacles.cbegin(), other.obstacles.cend()
      );
      return thisObstaclesSet == otherObstaclesSet;
    }();
    return this->attractiveGain == other.attractiveGain &&
      this->rotationalAttractiveGain == other.rotationalAttractiveGain &&
      this->goalPose == other.goalPose &&
      obstaclesEqual;
  }

  bool operator!=(const PotentialField& other) const { return !(*this == other); }

  void initializeKinematics(const std::string& urdfFilePath, const double influenceZoneScale, const double repulsiveGain);

  // ============ Getters and Setters ============
  void setAttractiveGain(double newAttractiveGain) { this->attractiveGain = newAttractiveGain; }
  void setRotationalAttractiveGain(double newRotationalAttractiveGain) {
    this->rotationalAttractiveGain = newRotationalAttractiveGain;
  }
  void setMaxLinearVelocity(double newMaxLinearVelocity) { this->maxLinearVelocity = newMaxLinearVelocity; }
  void setMaxAngularVelocity(double newMaxAngularVelocity) { this->maxAngularVelocity = newMaxAngularVelocity; }
  void setMaxLinearAcceleration(double newMaxLinearAcceleration) { this->maxLinearAcceleration = newMaxLinearAcceleration; }
  void setMaxAngularAcceleration(double newMaxAngularAcceleration) { this->maxAngularAcceleration = newMaxAngularAcceleration; }
  void setGoalPose(SpatialVector newGoalPose) { this->goalPose = newGoalPose; }
  double getAttractiveGain() const { return this->attractiveGain; }
  double getRotationalAttractiveGain() const { return this->rotationalAttractiveGain; }
  double getMaxLinearVelocity() const { return this->maxLinearVelocity; }
  double getMaxAngularVelocity() const { return this->maxAngularVelocity; }
  double getMaxLinearAcceleration() const { return this->maxLinearAcceleration; }
  double getMaxAngularAcceleration() const { return this->maxAngularAcceleration; }
  SpatialVector getGoalPose() const { return this->goalPose; }
  std::vector<PotentialFieldObstacle> getObstacles() const { return this->obstacles; }

  // ============ Obstacle Management ============

  /**
   * @brief Adds a new obstacle to the potential field.
   *
   * @param obstacle The obstacle to be added.
   */
  void addObstacle(PotentialFieldObstacle obstacle);

  /**
   * @brief Adds multiple obstacles to the potential field.
   *
   * @param obstacles The obstacles to be added.
   */
  void addObstacles(const std::vector<PotentialFieldObstacle>& obstacles);

  /**
   * @brief Attempts to remove an obstacle by ID.
   *
   * @note If the obstacle is not found, no action is taken.
   * @param obstacleFrameID  the frame ID of the obstacle obtained on obstacle creation.
   * @return true if the obstacle was removed successfully.
   */
  bool removeObstacle(const std::string& obstacleFrameID);

  /**
   * @brief Clears all obstacles from the potential field.
   */
  void clearObstacles();

  PotentialFieldObstacle getObstacleByID(const std::string& obstacleFrameID) const;

  bool isPointInsideObstacle(Eigen::Vector3d point) const;
  bool isPointWithinInfluenceZone(Eigen::Vector3d point) const;

  // ============ Force and Velocity Computation ============

  /**
   * @brief Given a 3D position, computes the task-space wrench
   *        by combining attractive and repulsive forces.
   *
   * @param queryPose The pose in 3D space to compute the wrench.
   * @return TaskSpaceWrench The resultant task-space wrench [N, Nm]
   */
  TaskSpaceWrench evaluateWrenchAtPose(const SpatialVector& queryPose) const;

  /**
   * @brief Given a 3D position, computes the task-space twist (velocity)
   *        by combining attractive and repulsive forces and converting to velocity vectors
   *
   * @param queryPose The pose in 3D space to compute the velocity
   * @return TaskSpaceTwist The resultant task-space twist [m/s, rad/s]
   */
  TaskSpaceTwist evaluateVelocityAtPose(const SpatialVector& queryPose) const;

  /**
   * @brief Performs the same operation as evaluateVelocityAtPose but applies
   *        velocity limits to the resulting twist.
   */
  TaskSpaceTwist evaluateLimitedVelocityAtPose(const SpatialVector& queryPose) const;

  /**
   * @brief Converts a task-space wrench to a task-space twist (velocity)
   *        using gain parameters converting force to velocity [m/s per N] and [rad/s per Nm]
   *
   * @note This function assumes a massless system where linearGain [(m/s)/N] and angularGain [(rad/s)/Nm] are both 1.0
   *
   * @param wrench The task-space wrench to convert.
   * @return TaskSpaceTwist The resultant task-space twist [m/s, rad/s].
   */
  TaskSpaceTwist wrenchToTwist(const TaskSpaceWrench& wrench) const;

  /**
   * @brief Applies velocity and acceleration limits to a task-space twist
   *
   * @param twist The input task-space twist to be limited
   * @param prevTwist The previous task-space twist for acceleration limiting, can use zero if not available
   * @param dt The time step over which to apply acceleration limits [s]
   * @return TaskSpaceTwist The limited task-space twist
   */
  TaskSpaceTwist applyMotionConstraints(
    const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double dt) const;

  /**
   * @brief Given a current pose and a delta time, computes the next pose after the time step.
   *
   * @note This function uses both integrateLinearVelocity and integrateAngularVelocity and
   *       combines their results to compute the next pose.
   *
   * @param currentPose The starting pose as a SpatialVector.
   * @param prevTwist The previous task-space twist for acceleration limiting.
   * @param dt The time step over which to compute the next pose [s].
   * @return SpatialVector The resulting pose after applying the velocity field for the time step.
   */
  SpatialVector interpolateNextPose(const SpatialVector& currentPose, const TaskSpaceTwist& prevTwist, const double dt);

  /**
   * @brief Given a current position, an instantaneous linear velocity vector, and a delta time,
   *        integrates the linear velocity to compute the new position after the time step.
   *
   * @param currentPosition The starting position as a 3D vector.
   * @param linearVelocity The instantaneous linear velocity vector in meters per second.
   * @param deltaTime The time step over which to integrate the linear velocity [s].
   * @return Eigen::Vector3d The resulting position after integrating the linear velocity.
   */
  Eigen::Vector3d integrateLinearVelocity(const Eigen::Vector3d& currentPosition,
    const Eigen::Vector3d& linearVelocity, double deltaTime);

  /**
   * @brief Given a current orientation and a delta time, computes the angular velocity
   *        to compute the new orientation after the time step.
   *
   * @param currentOrientation The starting orientation as a quaternion.
   * @param angularVelocity The instantaneous angular velocity vector in radians per second.
   * @param deltaTime The time step over which to integrate the angular velocity [s].
   * @return Eigen::Quaterniond The resulting orientation after integrating the angular velocity.
   */
  Eigen::Quaterniond integrateAngularVelocity(
    const Eigen::Quaterniond& currentOrientation,
    const Eigen::Vector3d& angularVelocity,
    double deltaTime);

  // ============ Path Planning ============

  /**
   * @brief Plans a path from the start pose to the goal pose using the potential field.
   *
   *
   * @param startPose The starting pose as a SpatialVector.
   * @param dt The time step for each iteration of the path planning [s].
   * @param goalTolerance The tolerance for reaching the goal pose [m].
   * @param ikSolver A shared pointer to an IKSolver for computing joint angles.
   * @param maxIters The maximum number of iterations to perform for path planning, defaults to 30000.
   * @return PlannedPath The planned path containing poses, twists, joint angles, and timestamps.
   */
  PlannedPath planPath(const SpatialVector& startPose, const double dt, const double goalTolerance,
    std::shared_ptr<IKSolver> ikSolver, const size_t maxIters = 30000);

private:
  double attractiveGain; // Gain for attractive force
  double rotationalAttractiveGain; // Gain for rotational attractive force
  double maxLinearVelocity; // [m/s]
  double maxAngularVelocity; // [rad/s]
  double maxLinearAcceleration; // [m/s^2]
  double maxAngularAcceleration; // [rad/s^2]
  SpatialVector goalPose; // Current GoalPose
  std::vector<PotentialFieldObstacle> obstacles; // Obstacle list
  std::unordered_map<std::string, size_t> obstacleIndex; // Fast lookup for obstacle updates/removals by ID
  const double translationalTolerance = 1e-3; // Threshold for distances to the goal and obstacles [m]
  const double rotationalThreshold = 0.02; // Threshold for rotational geodesic distance [rad]
  std::string urdfFileName; // URDF file path for kinematic model
  PFKinematics pfKinematics; // Kinematics helper for obstacle updates via joint angles

  /**
   * @brief Computes the attractive force towards the goal position
   *
   * @note Equation: F = -attractiveGain * (distance * direction)
   *
   * @param queryPose The pose in 3D space to compute the force from
   * @return Eigen::Vector3d The attractive force vector [N]
   */
  Eigen::Vector3d computeAttractiveForceLinear(const SpatialVector& queryPose) const;

  /**
   * @brief Computes the attractive moment (torque) to align the query pose's orientation with the goal pose's orientation
   *
   * @note Equation: M = -rotationalAttractiveGain * (angle * axis)
   *
   * @param queryPose The pose in 3D space to compute the moment from
   * @return Eigen::Vector3d The attractive moment vector [Nm]
   */
  Eigen::Vector3d computeAttractiveMoment(const SpatialVector& queryPose) const;

  /**
   * @brief Computes the repulsive linear force from all obstacles
   *
   * @note Equation: F = repulsiveGain * (1/distance - 1/influence) * (1 / distance^2) * direction
   *
   * @param queryPose The position in 3D space to compute the force from
   * @return Eigen::Vector3d The repulsive force vector [N]
   */
  Eigen::Vector3d computeRepulsiveForceLinear(const SpatialVector& queryPose) const;
};

#endif // PFIELDS_HPP
