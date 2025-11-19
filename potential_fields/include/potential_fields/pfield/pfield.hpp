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
 * @note The reference for these equations and implementation is:
 *       Principles of Robot Motion: Theory, Algorithms, and Implementations
 *       by Choset et al, 2005, MIT Press.
 *       https://ieeexplore.ieee.org/book/6267238
 *
 * @version 3.0
 * @date 2025-11-03
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
#include <memory>
#include <utility>

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

struct SegmentCollisionInfo {
  unsigned int startIdx; // Start index of the collision segment in the path
  unsigned int endIdx;   // End index of the collision segment in the path
  std::vector<EnvironmentCollisionInfo> collisions; // Details of collisions in this segment

  SegmentCollisionInfo(unsigned int startIdx, unsigned int endIdx, std::vector<EnvironmentCollisionInfo> collisions)
    : startIdx(startIdx), endIdx(endIdx), collisions(std::move(collisions)) {}
};

struct PlannedPath {
  std::vector<double> timeStamps; // Time stamps for each point in the path [s]
  std::vector<SpatialVector> poses; // End-effector pose
  std::vector<TaskSpaceTwist> twists; // End-effector velocity
  std::vector<std::vector<double>> jointAngles; // Joint angles for each point in the path [rad]
  std::vector<std::vector<double>> jointVelocities; // Joint velocities for each point in the path [rad/s]
  std::vector<SegmentCollisionInfo> collisionSegments; // Segments of the path with collisions
  unsigned int numPoints; // The number of points in the planned path, should be equal across all vectors
  double duration; // Total duration of the path [s]
  double dt; // Time difference between consecutive points [s]
  bool success = false; // Whether the path planning was successful

  PlannedPath() : numPoints(0), duration(0.0), dt(0.0) {}

  /**
   * @brief Records a path point with the given pose, twist, joint angles, and timestamp.
   *
   * @param timeStamp The time stamp for the path point [s]
   * @param pose The EE pose at the path point
   * @param twist The EE twist at the path point
   * @param jointAngles The joint angles at the path point [rad]
   */
  void recordPathPoint(double timeStamp, const SpatialVector& pose, const TaskSpaceTwist& twist,
    std::vector<double> jointAngles) {
    this->poses.push_back(pose);
    this->twists.push_back(twist);
    this->jointAngles.push_back(jointAngles);
    this->timeStamps.push_back(timeStamp);
    this->numPoints = static_cast<unsigned int>(this->poses.size());
    if (numPoints == 1) {
      this->dt = 0.0;
      this->duration = 0.0;
      this->jointVelocities.clear();
      // Joint Velocity at first point is zero
      this->jointVelocities.resize(jointAngles.size(), std::vector<double>(jointAngles.size(), 0.0));
      this->jointVelocities.push_back(std::vector<double>(jointAngles.size(), 0.0));
    }
    else if (this->numPoints > 1) {
      this->dt = this->timeStamps.back() - this->timeStamps[this->numPoints - 2];
      this->duration = this->timeStamps.back() - this->timeStamps.front();
      // Add an estimation of joint velocities
      const double dtSec = this->dt > 0.0 ? this->dt : 1.0;
      const size_t numJoints = this->jointAngles.back().size();
      std::vector<double> velocities(numJoints, 0.0);
      for (size_t j = 0; j < numJoints; ++j) {
        velocities[j] = (this->jointAngles.back()[j] - this->jointAngles[this->numPoints - 2][j]) / dtSec;
      }
      this->jointVelocities.insert(this->jointVelocities.cend(), velocities.cbegin(), velocities.cend());
    }
    else {
      this->dt = 0.0;
      this->duration = 0.0;
    }
  }
};

class PotentialField {
public:
  // =========== Constructors and Operators ===========
  PotentialField()
    : attractiveGain(DEFAULT_ATTRACTIVE_GAIN),
    repulsiveGain(DEFAULT_REPULSIVE_GAIN),
    rotationalAttractiveGain(DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    influenceDistance(DEFAULT_INFLUENCE_DISTANCE),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}
  ~PotentialField() = default;

  PotentialField(const PotentialField& other) :
    attractiveGain(other.attractiveGain),
    repulsiveGain(other.repulsiveGain),
    rotationalAttractiveGain(other.rotationalAttractiveGain),
    maxLinearVelocity(other.maxLinearVelocity),
    maxAngularVelocity(other.maxAngularVelocity),
    maxLinearAcceleration(other.maxLinearAcceleration),
    maxAngularAcceleration(other.maxAngularAcceleration),
    influenceDistance(other.influenceDistance),
    goalPose(other.goalPose),
    envObstacles(other.envObstacles),
    robotObstacles(other.robotObstacles) {}

  /**
   * @brief Constructs a PotentialField with the specified goal position.
   *        The attractive gain and rotational attractive gain are set to default values.
   *
   * @param goalPose The pose in 3D space that will generate an attractive force.
   */
  explicit PotentialField(SpatialVector goalPose) :
    attractiveGain(DEFAULT_ATTRACTIVE_GAIN),
    repulsiveGain(DEFAULT_REPULSIVE_GAIN),
    rotationalAttractiveGain(DEFAULT_ROTATIONAL_ATTRACTIVE_GAIN),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    influenceDistance(DEFAULT_INFLUENCE_DISTANCE),
    goalPose(goalPose) {}

  PotentialField(SpatialVector goalPose, double attractiveGain, double repulsiveGain, double rotationalAttractiveGain) :
    attractiveGain(attractiveGain),
    repulsiveGain(repulsiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    influenceDistance(DEFAULT_INFLUENCE_DISTANCE),
    goalPose(goalPose) {}

  PotentialField(double attractiveGain, double repulsiveGain, double rotationalAttractiveGain) :
    attractiveGain(attractiveGain),
    repulsiveGain(repulsiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(DEFAULT_MAX_LINEAR_VELOCITY),
    maxAngularVelocity(DEFAULT_MAX_ANGULAR_VELOCITY),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    influenceDistance(DEFAULT_INFLUENCE_DISTANCE),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}

  PotentialField(
    double attractiveGain, double repulsiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity,
    double influenceDistance) :
    attractiveGain(attractiveGain),
    repulsiveGain(repulsiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(DEFAULT_MAX_LINEAR_ACCELERATION),
    maxAngularAcceleration(DEFAULT_MAX_ANGULAR_ACCELERATION),
    influenceDistance(influenceDistance),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}

  PotentialField(
    double attractiveGain, double repulsiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity,
    double maxLinearAcceleration, double maxAngularAcceleration,
    double influenceDistance) :
    attractiveGain(attractiveGain),
    repulsiveGain(repulsiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(maxLinearAcceleration),
    maxAngularAcceleration(maxAngularAcceleration),
    influenceDistance(influenceDistance),
    goalPose(SpatialVector(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity())) {}

  PotentialField(
    SpatialVector goalPose,
    double attractiveGain, double repulsiveGain, double rotationalAttractiveGain,
    double maxLinearVelocity, double maxAngularVelocity,
    double maxLinearAcceleration, double maxAngularAcceleration,
    double influenceDistance) :
    attractiveGain(attractiveGain),
    repulsiveGain(repulsiveGain),
    rotationalAttractiveGain(rotationalAttractiveGain),
    maxLinearVelocity(maxLinearVelocity),
    maxAngularVelocity(maxAngularVelocity),
    maxLinearAcceleration(maxLinearAcceleration),
    maxAngularAcceleration(maxAngularAcceleration),
    influenceDistance(influenceDistance),
    goalPose(goalPose) {}


  PotentialField& operator=(const PotentialField& other) {
    if (this != &other) {
      this->attractiveGain = other.attractiveGain;
      this->repulsiveGain = other.repulsiveGain;
      this->rotationalAttractiveGain = other.rotationalAttractiveGain;
      this->goalPose = other.goalPose;
      this->envObstacles = other.envObstacles;
      this->robotObstacles = other.robotObstacles;
    }
    return *this;
  }

  bool operator==(const PotentialField& other) const {
    auto obstaclesEqual = [this, &other]() -> bool {
      if (this->envObstacles.size() != other.envObstacles.size()) return false;
      // Convert both obstacle lists to unordered_sets for comparison
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> thisEnvObstaclesSet(
        this->envObstacles.cbegin(), this->envObstacles.cend()
      );
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> thisRobotObstaclesSet(
        this->robotObstacles.cbegin(), this->robotObstacles.cend()
      );
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> otherEnvObstaclesSet(
        other.envObstacles.cbegin(), other.envObstacles.cend()
      );
      std::unordered_set<PotentialFieldObstacle, PotentialFieldObstacleHash> otherRobotObstaclesSet(
        other.robotObstacles.cbegin(), other.robotObstacles.cend()
      );
      return thisEnvObstaclesSet == otherEnvObstaclesSet && thisRobotObstaclesSet == otherRobotObstaclesSet;
    }();
    return this->attractiveGain == other.attractiveGain &&
      this->rotationalAttractiveGain == other.rotationalAttractiveGain &&
      this->goalPose == other.goalPose &&
      obstaclesEqual;
  }
  bool operator!=(const PotentialField& other) const { return !(*this == other); }

  void initializeKinematics(const std::string& urdfFilePath, const std::vector<std::string>& jointNames);
  void assignIKSolver(std::shared_ptr<IKSolver> ikSolver) { this->ikSolver = ikSolver; }

  // ============ Getters and Setters ============
  void setAttractiveGain(double newAttractiveGain) { this->attractiveGain = newAttractiveGain; }
  void setRepulsiveGain(double newRepulsiveGain) { this->repulsiveGain = newRepulsiveGain; }
  void setRotationalAttractiveGain(double newRotationalAttractiveGain) {
    this->rotationalAttractiveGain = newRotationalAttractiveGain;
  }
  void setMaxLinearVelocity(double newMaxLinearVelocity) { this->maxLinearVelocity = newMaxLinearVelocity; }
  void setMaxAngularVelocity(double newMaxAngularVelocity) { this->maxAngularVelocity = newMaxAngularVelocity; }
  void setMaxLinearAcceleration(double newMaxLinearAcceleration) { this->maxLinearAcceleration = newMaxLinearAcceleration; }
  void setMaxAngularAcceleration(double newMaxAngularAcceleration) { this->maxAngularAcceleration = newMaxAngularAcceleration; }
  void setInfluenceDistance(double newInfluenceDistance) { this->influenceDistance = newInfluenceDistance; }
  void setGoalPose(SpatialVector newGoalPose) { this->goalPose = newGoalPose; }
  void useDynamicQuadraticThreshold(bool enabled) { this->dynamicQuadraticThresholdEnabled = enabled; }
  double getAttractiveGain() const { return this->attractiveGain; }
  double getRepulsiveGain() const { return this->repulsiveGain; }
  double getRotationalAttractiveGain() const { return this->rotationalAttractiveGain; }
  double getMaxLinearVelocity() const { return this->maxLinearVelocity; }
  double getMaxAngularVelocity() const { return this->maxAngularVelocity; }
  double getMaxLinearAcceleration() const { return this->maxLinearAcceleration; }
  double getMaxAngularAcceleration() const { return this->maxAngularAcceleration; }
  double getInfluenceDistance() const { return this->influenceDistance; }
  bool isUsingDynamicQuadraticThreshold() const { return this->dynamicQuadraticThresholdEnabled; }
  SpatialVector getGoalPose() const { return this->goalPose; }
  std::vector<PotentialFieldObstacle> getEnvObstacles() const { return this->envObstacles; }
  std::vector<PotentialFieldObstacle> getRobotObstacles() const { return this->robotObstacles; }

  // ============ Obstacle Management ============

  /**
   * @brief Given a new set of joint angles, updates the internal obstacles
   *        based on the robot kinematics.
   *
   * @note This function requires that PFKinematics has been initialized using
   *       initializeKinematics prior to calling this function.
   *
   * @param jointAngles The new joint angles to compute the updated obstacle poses [rad]
   */
  void updateObstaclesFromKinematics(const std::vector<double>& jointAngles);

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

  /**
   * @brief Gets an obstacle by its frame ID.
   *
   * @param obstacleFrameID the frame ID of the obstacle to retrieve.
   * @return PotentialFieldObstacle The obstacle with the specified frame ID.
   */
  PotentialFieldObstacle getObstacleByID(const std::string& obstacleFrameID) const;

  /**
   * @brief Gets all of the obstacles belonging to a specific group.
   *
   * @note Obstacle groups are [STATIC, DYNAMIC, ROBOT]
   *
   * @param group The group to filter obstacles by.
   * @return std::vector<PotentialFieldObstacle> A vector of obstacles belonging to the specified group.
   */
  std::vector<PotentialFieldObstacle> getObstaclesByGroup(ObstacleGroup group) const;

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
   *
   * @note This function allows you to provide the previous twist and time step for
   *       acceleration limiting, but can be called with default parameters to only apply velocity limits
   */
  TaskSpaceTwist evaluateLimitedVelocityAtPose(const SpatialVector& queryPose,
    const TaskSpaceTwist& prevTwist = TaskSpaceTwist(), const double dt = 0.0) const;

  /**
   * @brief Given attraction and repulsive force vectors, mitigate local minima near broad surfaces
   *        by removing opposing components of the repulsive force.
   *
   * Removes the component of the repulsive force that directly opposes the
   * attractive force, while preserving tangential components that encourage
   * sliding around obstacles. Formally, if u_att = F_att/||F_att|| and
   * dot = F_rep·u_att < 0, return F_rep - dot·u_att; otherwise return F_rep.
   * When ||F_att|| is near zero, the repulsive force is returned unchanged.
   *
   * @note This is typically applied only in planning (not in evaluateWrenchAtPose)
   *       to avoid altering the visualized field and unit-test semantics.
   *
   * @param attractionForce Attractive linear force vector, typically calculated from computeAttractiveForceLinear.
   * @param repulsiveForce Repulsive linear force vector, typically calculated from computeRepulsiveForceLinear.
   * @return Eigen::Vector3d Resultant force vector with opposing components removed
   */
  Eigen::Vector3d removeOpposingForce(const Eigen::Vector3d& attractionForce, const Eigen::Vector3d& repulsiveForce) const;

  /**
   * @brief Given a 3D position, computes the task-space wrench (force/torque)
   *        by combining attractive and repulsive forces and removing any repulsive components
   *        that oppose the attractive force to help escape local minima.
   *
   * @note This function is primarily intended for use during path planning and
   *       simply coordinates the use of computeAttractiveForceLinear, computeRepulsiveForceLinear,
   *       and removeOpposingForce to produce the final wrench.
   *
   * @param queryPose The pose in 3D space to compute the wrench.
   * @return TaskSpaceWrench The resultant task-space wrench [N, Nm]
   */
  TaskSpaceWrench evaluateWrenchAtPoseWithOpposingForceRemoval(const SpatialVector& queryPose) const;

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
  TaskSpaceTwist applyMotionConstraints(const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double dt) const;

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
   * @note Equation: M = rotationalAttractiveGain * (angle * axis)
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

  // ============ Path Planning ============

  /**
   * @brief Computes the constrained twist at a given pose using the previous twist and the time step
   *        for velocity and acceleration limiting.
   *
   * @param pose The current pose to compute the constrained twist for
   * @param prevTwist The previous twist to use for limiting
   * @param dt The time step for the integration
   * @return TaskSpaceTwist The computed constrained twist
   */
  TaskSpaceTwist constrainedTwistAtPose(const SpatialVector& pose, const TaskSpaceTwist& prevTwist, const double dt);

  /**
   * @brief Computes the next pose and twist using the Runge-Kutta 4-step (RK4) integration method.
   *
   * @param currentPose The current pose to compute the next pose from
   * @param prevTwist The previous twist to use for the integration
   * @param dt The time step for the integration
   * @return std::pair<SpatialVector, TaskSpaceTwist> The computed next pose and twist
   */
  std::pair<SpatialVector, TaskSpaceTwist> rungeKuttaStep(const SpatialVector& currentPose,
    const TaskSpaceTwist& prevTwist, const double dt);

  /**
   * @brief Compute a per-step switch distance d* that blends goal/path clearance and stopping distance.
   *
   * @note When dynamic thresholding is disabled, the fixed defaultDStarThreshold is used.
   *
   * @param queryPose The current pose to compute the dynamic threshold for
   * @return double The computed dynamic quadratic threshold distance d* [m]
   */
  double computeDynamicQuadraticThreshold(const SpatialVector& queryPose) const;

  /**
   * @brief Computes the minimum obstacle clearance at a given point.
   *
   * @note Returns +infinity if no obstacles are present as `std::numeric_limits<double>::infinity()`.
   *
   * @param point The point in 3D space to compute the clearance for
   * @return double The minimum clearance distance to the nearest obstacle [m]
   */
  double minObstacleClearanceAt(const Eigen::Vector3d& point) const;

  /**
   * @brief Computes the minimum obstacle clearance along a line segment.
   *
   * @note Returns +infinity if no obstacles are present as `std::numeric_limits<double>::infinity()`.
   *
   * @param from The starting point of the line segment
   * @param to The ending point of the line segment
   * @param samples The number of samples to take along the segment (default is 7)
   * @return double The minimum clearance distance along the segment [m]
   */
  double minClearanceAlongSegment(const Eigen::Vector3d& from, const Eigen::Vector3d& to, int samples = 7) const;

  /**
   * @brief Checks if the robot is within a certain clearance distance from any environment obstacles.
   *
   * @param clearanceThreshold The clearance distance to check against [m]
   * @return true If the robot is in collision with an environment obstacle
   * @return false If the robot is not in collision with any environment obstacles
   */
  bool isRobotInCollisionWithEnvironment(double clearanceThreshold = 0.0) const;

  /**
 * @brief Plans a path from the start pose to the goal pose using the potential field.
 *
 * @note When stagnation is detected (i.e., minimal position change over a set number of iterations),
 *       the force computation changes to remove opposing repulsive force components to help escape local minima
 *       using the removeOpposingForce method.
 *
 *
 * @param startPose The starting pose as a SpatialVector.
 * @param startJointAngles The starting joint angles for the robot [rad].
 * @param dt The time step for each iteration of the path planning [s].
 * @param goalTolerance The tolerance for reaching the goal pose [m].
 * @param stagnationLimit The number of iterations to consider for stagnation detection, defaults to 100.
 * @param stagnationThreshold The threshold for detecting stagnation in position change, defaults to 1e-4 [m].
 * @param maxIters The maximum number of iterations to perform for path planning, defaults to 30000.
 * @return PlannedPath The planned path containing poses, twists, joint angles, and timestamps.
 */
  PlannedPath planPath(
    const SpatialVector& startPose,
    const std::vector<double>& startJointAngles,
    const double dt,
    const double goalTolerance,
    const size_t maxIters = 30000
  );

  /**
   * @brief Identifies segments of the path that are in collision with obstacles.
   *
   * @param path The planned path to check for collisions.
   * @param clearanceThreshold The minimum clearance distance to consider for a collision.
   * @return std::vector<SegmentCollisionInfo> The list of collision segments along with their details.
   */
  std::vector<SegmentCollisionInfo> identifyPathCollisions(PlannedPath path, double clearanceThreshold);

  std::vector<double> computeInverseKinematics(const SpatialVector& targetPose, const std::vector<double>& seedJointAngles) const;

  /**
   * @brief Creates a CSV file from a planned path with the following columns:
   *        Time [s],
   *        Position X [m], Position Y [m], Position Z [m],
   *        Orientation Qx, Orientation Qy, Orientation Qz, Orientation Qw,
   *        Linear Velocity X [m/s], Linear Velocity Y [m/s], Linear Velocity Z [m/s],
   *        Angular Velocity X [rad/s], Angular Velocity Y [rad/s], Angular Velocity Z [rad/s],
   *        Minimum Obstacle Clearance [m],
   *        Number of Joints
   *        Joint 1 [rad], Joint 2 [rad], ..., Joint N [rad]
   *
   * @note The real header looks like:
   *       time_s, pos_x_m, pos_y_m, pos_z_m,
   *       q_x, q_y, q_z, q_w,
   *       vel_x_m_s, vel_y_m_s, vel_z_m_s,
   *       ang_vel_x_rad_s, ang_vel_y_rad_s, ang_vel_z_rad_s,
   *       min_obstacle_clearance_m,
   *       num_joints,
   *       joint_1_rad, joint_2_rad, ..., joint_N_rad
   *
   * @param path The planned path to convert to CSV
   * @param filePath The file path to save the CSV
   * @return true If the CSV was created successfully
   * @return false If there was an error creating the CSV
   */
  bool createPlannedPathCSV(const PlannedPath& path, const std::string& filePath) const;

private:
  double attractiveGain; // Gain for attractive force
  double repulsiveGain; // Gain for repulsive force
  double rotationalAttractiveGain; // Gain for rotational attractive force
  double maxLinearVelocity; // [m/s]
  double maxAngularVelocity; // [rad/s]
  double maxLinearAcceleration; // [m/s^2]
  double maxAngularAcceleration; // [rad/s^2]
  double influenceDistance; // [m] global influence distance
  SpatialVector goalPose; // Current GoalPose
  std::vector<PotentialFieldObstacle> envObstacles; // Obstacle list
  std::vector<PotentialFieldObstacle> robotObstacles; // Obstacle list
  std::unordered_map<std::string, size_t> envObstacleIndexMap; // Fast lookup for obstacle updates/removals by ID
  std::unordered_map<std::string, size_t> robotObstacleIndexMap; // Fast lookup for obstacle updates/removals by ID
  std::string urdfFileName; // URDF file path for kinematic model
  std::unique_ptr<PFKinematics> pfKinematics; // Kinematics helper for obstacle updates via joint angles
  std::shared_ptr<IKSolver> ikSolver; // Inverse kinematics solver for joint angle computation
  const double translationalTolerance = 1e-3; // Threshold for distances to the goal and obstacles [m]
  const double rotationalThreshold = 0.02; // Threshold for rotational geodesic distance [rad]
  const double defaultDStarThreshold = 1.0; // [m] distance for switching to quadratic attractive potential from conical
  const double softSatBeta = 1.0; // Soft-saturation parameter, higher = more aggressive curve
  bool dynamicQuadraticThresholdEnabled = false; // if true, use dynamic d* based on obstacles/kinematics
};

#endif // PFIELDS_HPP
