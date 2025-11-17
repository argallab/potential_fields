#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"
#include <algorithm>
#include <limits>

#include <coal/collision.h>

void PotentialField::initializeKinematics(
  const std::string& urdfFilePath,
  const std::vector<std::string>& jointNames) {
  this->urdfFileName = urdfFilePath;
  this->pfKinematics = std::make_unique<PFKinematics>(this->urdfFileName, jointNames);
  const double maxExtent = this->pfKinematics->estimateRobotExtentRadius();
  // Assign influence distance using maximum robot extent or the default, whichever is more generous
  this->influenceDistance = std::max(this->influenceDistance, maxExtent);
}

void PotentialField::updateObstaclesFromKinematics(const std::vector<double>& jointAngles) {
  if (!this->pfKinematics) return;
  std::vector<PotentialFieldObstacle> newObstacles = this->pfKinematics->updateObstaclesFromJointAngles(jointAngles);
  this->addObstacles(newObstacles);
}

void PotentialField::addObstacle(PotentialFieldObstacle obstacle) {
  const std::string frameID = obstacle.getFrameID();
  auto itIndex = this->envObstacleIndexMap.find(frameID);
  if (itIndex != this->envObstacleIndexMap.end()) {
    // Update existing obstacle in place
    if (obstacle.getGroup() == ObstacleGroup::ROBOT) {
      this->robotObstacles[itIndex->second] = obstacle;
    }
    else {
      this->envObstacles[itIndex->second] = obstacle;
    }
  }
  else {
    // Append new obstacle and record index
    if (obstacle.getGroup() == ObstacleGroup::ROBOT) {
      this->robotObstacles.push_back(obstacle);
      this->robotObstacleIndexMap.emplace(frameID, this->robotObstacles.size() - 1);
    }
    else {
      this->envObstacles.push_back(obstacle);
      this->envObstacleIndexMap.emplace(frameID, this->envObstacles.size() - 1);
    }
  }
}

void PotentialField::addObstacles(const std::vector<PotentialFieldObstacle>& obstacles) {
  for (const auto& obst : obstacles) { this->addObstacle(obst); }
}

bool PotentialField::removeObstacle(const std::string& obstacleFrameID) {
  // Try environment obstacles first
  auto itEnv = this->envObstacleIndexMap.find(obstacleFrameID);
  if (itEnv != this->envObstacleIndexMap.end()) {
    size_t idx = itEnv->second;
    size_t last = this->envObstacles.size() - 1;
    if (idx != last) {
      std::swap(this->envObstacles[idx], this->envObstacles[last]);
      // update moved obstacle's index
      this->envObstacleIndexMap[this->envObstacles[idx].getFrameID()] = idx;
    }
    this->envObstacles.pop_back();
    this->envObstacleIndexMap.erase(itEnv);
    return true;
  }

  // Check robot obstacles next
  auto itRobot = this->robotObstacleIndexMap.find(obstacleFrameID);
  if (itRobot != this->robotObstacleIndexMap.end()) {
    size_t idx = itRobot->second;
    size_t last = this->robotObstacles.size() - 1;
    if (idx != last) {
      std::swap(this->robotObstacles[idx], this->robotObstacles[last]);
      // update moved obstacle's index
      this->robotObstacleIndexMap[this->robotObstacles[idx].getFrameID()] = idx;
    }
    this->robotObstacles.pop_back();
    this->robotObstacleIndexMap.erase(itRobot);
    return true;
  }

  // Not found in either map
  return false;
}

void PotentialField::clearObstacles() { this->envObstacles.clear(); this->envObstacleIndexMap.clear(); }

PotentialFieldObstacle PotentialField::getObstacleByID(const std::string& obstacleFrameID) const {
  auto itIndex = this->envObstacleIndexMap.find(obstacleFrameID);
  if (itIndex == this->envObstacleIndexMap.end()) {
    throw std::invalid_argument("Obstacle with the given ID does not exist.");
  }
  return this->envObstacles[itIndex->second];
}

std::vector<PotentialFieldObstacle> PotentialField::getObstaclesByGroup(ObstacleGroup group) const {
  switch (group) {
  case ObstacleGroup::STATIC:
  case ObstacleGroup::DYNAMIC:
    return this->envObstacles;
  case ObstacleGroup::ROBOT:
    return this->robotObstacles;
  default:
    // For now, return empty list instead of throwing an exception
    return {};
  }
}

bool PotentialField::isPointInsideObstacle(Eigen::Vector3d point) const {
  for (const auto& obst : this->envObstacles) {
    if (obst.withinObstacle(point)) { return true; }
  }
  return false;
}

bool PotentialField::isPointWithinInfluenceZone(Eigen::Vector3d point) const {
  for (const auto& obst : this->envObstacles) {
    if (obst.withinInfluenceZone(point, this->influenceDistance)) { return true; }
  }
  return false;
}

TaskSpaceWrench PotentialField::evaluateWrenchAtPose(const SpatialVector& queryPose) const {
  Eigen::Vector3d attractionForceVector = this->computeAttractiveForceLinear(queryPose);
  Eigen::Vector3d attractionMomentVector = this->computeAttractiveMoment(queryPose);
  Eigen::Vector3d repulsionForceVector = this->computeRepulsiveForceLinear(queryPose);
  Eigen::Vector3d forceVector = attractionForceVector + repulsionForceVector;
  return TaskSpaceWrench(forceVector, attractionMomentVector);
}

TaskSpaceTwist PotentialField::evaluateVelocityAtPose(const SpatialVector& queryPose) const {
  return this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose));
}

TaskSpaceTwist PotentialField::evaluateLimitedVelocityAtPose(
  const SpatialVector& queryPose, const TaskSpaceTwist& prevTwist, double dt) const {
  return this->applyMotionConstraints(this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose)), prevTwist, dt);
}

TaskSpaceTwist PotentialField::wrenchToTwist(const TaskSpaceWrench& wrench) const {
  // If a mass/inertia model is available, it can be used here to convert force/torque to velocity
  const double linearGain = 1.0;  // (m/s) per N
  const double angularGain = 1.0; // (rad/s) per Nm
  return TaskSpaceTwist(wrench.force * linearGain, wrench.torque * angularGain);
}

TaskSpaceTwist PotentialField::applyMotionConstraints(
  const TaskSpaceTwist& twist, const TaskSpaceTwist& prevTwist, const double dt) const {
  // Soft-saturate velocities by norm
  Eigen::Vector3d limitedLinear = softSaturateNorm(twist.linearVelocity, this->maxLinearVelocity, this->softSatBeta);
  Eigen::Vector3d limitedAngular = softSaturateNorm(twist.angularVelocity, this->maxAngularVelocity, this->softSatBeta);

  // If dt is valid, apply rate limits (acceleration limits)
  if (isPositiveFinite(dt)) {
    const double dVMaxLinear = this->maxLinearAcceleration * dt; // m/s^2 * s = m/s
    const double dVMaxAngular = this->maxAngularAcceleration * dt; // rad/s^2 * s = rad/s
    limitedLinear = rateLimitStep(prevTwist.linearVelocity, limitedLinear, dVMaxLinear);
    limitedAngular = rateLimitStep(prevTwist.angularVelocity, limitedAngular, dVMaxAngular);

    // Re-apply soft-saturation to prevent velocity limits
    limitedLinear = softSaturateNorm(limitedLinear, this->maxLinearVelocity, this->softSatBeta);
    limitedAngular = softSaturateNorm(limitedAngular, this->maxAngularVelocity, this->softSatBeta);
  }
  return TaskSpaceTwist(limitedLinear, limitedAngular);
}

Eigen::Vector3d PotentialField::removeOpposingForce(const Eigen::Vector3d& attractionForce,
  const Eigen::Vector3d& repulsiveForce) const {
  // Default to returning the original repulsive force unchanged
  Eigen::Vector3d resultantForce = repulsiveForce;
  // Use the direction of the attractive force to determine opposing (parallel) components
  const double attractiveNorm = attractionForce.norm();
  if (attractiveNorm > NEAR_ZERO_THRESHOLD) {
    const Eigen::Vector3d u_att = attractionForce / attractiveNorm;
    const double dot = repulsiveForce.dot(u_att);
    // If repulsion has a component opposite to attraction (dot < 0),
    // subtract the opposing component along u_att
    if (dot < 0.0) {
      resultantForce = repulsiveForce - (dot * u_att);
    }
  }
  return resultantForce;
}

TaskSpaceWrench PotentialField::evaluateWrenchAtPoseWithOpposingForceRemoval(const SpatialVector& queryPose) const {
  // Compute attractive and repulsive forces
  Eigen::Vector3d attractiveForce = this->computeAttractiveForceLinear(queryPose);
  Eigen::Vector3d repulsiveForce = this->computeRepulsiveForceLinear(queryPose);
  // Remove only the component of the repulsive force that opposes attraction,
  // then combine with the attractive force to form the final resultant force
  Eigen::Vector3d filteredRepulsive = this->removeOpposingForce(attractiveForce, repulsiveForce);
  Eigen::Vector3d resultantForce = attractiveForce + filteredRepulsive;
  Eigen::Vector3d resultantTorque = this->computeAttractiveMoment(queryPose);
  return TaskSpaceWrench(resultantForce, resultantTorque);
}

SpatialVector PotentialField::interpolateNextPose(
  const SpatialVector& currentPose, const TaskSpaceTwist& prevTwist, const double dt) {
  // Compute the TaskSpaceTwist at the current pose
  TaskSpaceTwist vel = this->evaluateVelocityAtPose(currentPose);
  TaskSpaceTwist velLimited = this->applyMotionConstraints(vel, prevTwist, dt);
  // Integrate linear and angular velocities to get next pose
  Eigen::Vector3d nextPosition = this->integrateLinearVelocity(currentPose.getPosition(), velLimited.linearVelocity, dt);
  Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(
    currentPose.getOrientation(), velLimited.angularVelocity, dt
  );
  return SpatialVector(nextPosition, nextOrientation);
}

Eigen::Vector3d PotentialField::integrateLinearVelocity(const Eigen::Vector3d& currentPosition,
  const Eigen::Vector3d& linearVelocity, double deltaTime) {
  return currentPosition + (linearVelocity * deltaTime);
}

Eigen::Quaterniond PotentialField::integrateAngularVelocity(const Eigen::Quaterniond& currentOrientation,
  const Eigen::Vector3d& angularVelocity,
  double deltaTime) {
  const double wnorm = angularVelocity.norm();
  if (wnorm < 1e-6 || deltaTime <= 0.0) return currentOrientation;
  // Exponential map: dq = exp( [w]^ * dt ) ≈ AngleAxis( |w|*dt, w/|w| )
  Eigen::Quaterniond dq(Eigen::AngleAxisd(wnorm * deltaTime, angularVelocity / wnorm));
  return (currentOrientation * dq).normalized();
}

TaskSpaceTwist PotentialField::constrainedTwistAtPose(
  const SpatialVector& pose,
  const TaskSpaceTwist& prevTwist, const double dt) {
  TaskSpaceWrench W = this->evaluateWrenchAtPoseWithOpposingForceRemoval(pose);
  return this->applyMotionConstraints(this->wrenchToTwist(W), prevTwist, dt);
}

std::pair<SpatialVector, TaskSpaceTwist> PotentialField::rungeKuttaStep(const SpatialVector& currentPose,
  const TaskSpaceTwist& prevTwist, const double dt) {
  // Stage 1
  TaskSpaceTwist k1 = this->constrainedTwistAtPose(currentPose, prevTwist, dt);
  // Pose at dt/2 using k1
  SpatialVector T2(
    this->integrateLinearVelocity(currentPose.getPosition(), k1.getLinearVelocity(), 0.5 * dt),
    this->integrateAngularVelocity(currentPose.getOrientation(), k1.getAngularVelocity(), 0.5 * dt)
  );
  // Stage 2 (rate-limit vs k1 over dt/2)
  TaskSpaceTwist k2 = this->constrainedTwistAtPose(T2, k1, 0.5 * dt);
  // Pose at dt/2 using k2
  SpatialVector T3(
    this->integrateLinearVelocity(currentPose.getPosition(), k2.getLinearVelocity(), 0.5 * dt),
    this->integrateAngularVelocity(currentPose.getOrientation(), k2.getAngularVelocity(), 0.5 * dt)
  );
  // Stage 3 (rate-limit vs k2 over dt/2)
  TaskSpaceTwist k3 = this->constrainedTwistAtPose(T3, k2, 0.5 * dt);
  // Pose at dt using k3
  SpatialVector T4(
    this->integrateLinearVelocity(currentPose.getPosition(), k3.getLinearVelocity(), dt),
    this->integrateAngularVelocity(currentPose.getOrientation(), k3.getAngularVelocity(), dt)
  );
  // Stage 4 (rate-limit vs k3 over full dt)
  TaskSpaceTwist k4 = this->constrainedTwistAtPose(T4, k3, dt);

  // RK4 weighted average twist
  Eigen::Vector3d v_bar = (k1.getLinearVelocity()
    + 2.0 * k2.getLinearVelocity()
    + 2.0 * k3.getLinearVelocity()
    + k4.getLinearVelocity()) / 6.0;
  Eigen::Vector3d w_bar = (k1.getAngularVelocity()
    + 2.0 * k2.getAngularVelocity()
    + 2.0 * k3.getAngularVelocity()
    + k4.getAngularVelocity()) / 6.0;

  // Final soft-saturation to ensure we still respect velocity caps
  TaskSpaceTwist rk4WeightedTwist(v_bar, w_bar);
  rk4WeightedTwist = this->applyMotionConstraints(rk4WeightedTwist, prevTwist, dt);

  // Advance once using the averaged twist
  SpatialVector nextPose(
    this->integrateLinearVelocity(currentPose.getPosition(), rk4WeightedTwist.getLinearVelocity(), dt),
    this->integrateAngularVelocity(currentPose.getOrientation(), rk4WeightedTwist.getAngularVelocity(), dt)
  );

  return {nextPose, rk4WeightedTwist};
}

Eigen::Vector3d PotentialField::computeAttractiveForceLinear(const SpatialVector& queryPose) const {
  // Choset Attractive Potential: F = zeta * (q - q_goal)
  const Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
  const double magnitude = direction.norm();
  if (magnitude <= this->translationalTolerance) return Eigen::Vector3d::Zero();
  // else return -this->attractiveGain * direction;
  // The Choset attractive potential defines a threshold for switching to quadratic behavior from conical
  const double dStarThreshold = this->dynamicQuadraticThresholdEnabled ? this->computeDynamicQuadraticThreshold(queryPose)
    : this->defaultDStarThreshold;
  if (magnitude <= dStarThreshold) {
    // Quadratic Attraction Region
    return -this->attractiveGain * direction;
  }
  else {
    // Conical Attraction Region
    return  (dStarThreshold * (-this->attractiveGain * direction)) / magnitude;
  }
}

double PotentialField::minObstacleClearanceAt(const Eigen::Vector3d& point) const {
  if (this->envObstacles.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double minClearance = std::numeric_limits<double>::infinity();
  for (const auto& obst : this->envObstacles) {
    double signedDistance = 0.0; // signed distance: >0 outside, <0 inside
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    obst.computeSignedDistanceAndNormal(point, signedDistance, normal);
    double clearance = std::max(0.0, signedDistance); // treat inside as 0 clearance
    if (clearance < minClearance) minClearance = clearance;
  }
  return minClearance;
}

double PotentialField::minClearanceAlongSegment(const Eigen::Vector3d& from, const Eigen::Vector3d& to, int samples) const {
  if (this->envObstacles.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  samples = std::max(2, samples);
  double minClearance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < samples; ++i) {
    double u = static_cast<double>(i) / static_cast<double>(samples - 1);
    Eigen::Vector3d p = from + u * (to - from);
    double clearance = this->minObstacleClearanceAt(p);
    if (clearance < minClearance) minClearance = clearance;
  }
  return minClearance;
}

double PotentialField::computeDynamicQuadraticThreshold(const SpatialVector& queryPose) const {
  // Baseline from existing parameter
  double baseline = this->defaultDStarThreshold;
  if (!this->isUsingDynamicQuadraticThreshold()) { return baseline; }

  // Clearance near goal and along straight-line path to goal
  const Eigen::Vector3d goalPosition = this->goalPose.getPosition();
  const Eigen::Vector3d queryPosition = queryPose.getPosition();
  const double clearanceAtGoal = this->minObstacleClearanceAt(goalPosition);
  const double pathClearance = this->minClearanceAlongSegment(queryPosition, goalPosition, 7);

  // Use the more conservative clearance estimate
  double baselineClearance = std::min(clearanceAtGoal, pathClearance);

  // Stopping distance based on limits (ensure smooth approach)
  double stoppingDistance = 0.0;
  if (this->maxLinearAcceleration > NEAR_ZERO_THRESHOLD) {
    stoppingDistance = 0.5 * (this->maxLinearVelocity * this->maxLinearVelocity) / this->maxLinearAcceleration;
  }

  // If we have clearance data, increase d* in clutter (small clearance) to start quadratic earlier.
  // If free space (large clearance), keep near baseline.
  double dStar = baseline;
  if (std::isfinite(baselineClearance)) {
    const double Q = std::max(this->influenceDistance, 0.0);
    const double clutterScale = std::max(0.0, Q - baselineClearance); // larger when tighter space
    // Blend baseline, clutter and stopping distance
    dStar = baseline + 0.5 * clutterScale + 0.5 * stoppingDistance;
  }
  else {
    // No obstacles: just add a portion of stopping distance
    dStar = baseline + 0.5 * stoppingDistance;
  }
  // Clamp to reasonable bounds
  const double minimumDStarValue = 0.02;
  const double dStarMin = std::max(5.0 * this->translationalTolerance, minimumDStarValue);
  const double dStarMax = std::max(this->influenceDistance, baseline);
  return std::clamp(dStar, dStarMin, dStarMax);
}

Eigen::Vector3d PotentialField::computeAttractiveMoment(const SpatialVector& queryPose) const {
  // Quaternion error that rotates current -> goal (shortest path)
  Eigen::Quaterniond quatError = (
    this->goalPose.getOrientation() * queryPose.getOrientation().conjugate()
    ).normalized();

  // Enforce shortest-path and continuity: q and -q represent same rotation; choose w >= 0
  if (quatError.w() < 0.0) {
    quatError.coeffs() *= -1.0; // flip sign of (x,y,z,w)
  }

  // Extract axis-angle from quatError in a numerically stable way
  const double vnorm = quatError.vec().norm();
  const double angle = 2.0 * std::atan2(vnorm, std::abs(quatError.w())); // in [0, pi]
  if (angle <= this->rotationalThreshold) { return Eigen::Vector3d::Zero(); }

  // Guard against divide-by-zero when angle ~ 0
  if (vnorm < NEAR_ZERO_THRESHOLD) { return Eigen::Vector3d::Zero(); }
  // Use bounded sine-form error for smoother large-angle behavior:
  // 2*sin(angle/2) * axis == 2 * quatError.vec() (since quatError is normalized and w >= 0)
  // Matches k*angle*axis for small angles and reduces aggressiveness near pi
  const Eigen::Vector3d boundedError = 2.0 * quatError.vec();
  return this->rotationalAttractiveGain * boundedError;
}

Eigen::Vector3d PotentialField::computeRepulsiveForceLinear(const SpatialVector& queryPose) const {
  Eigen::Vector3d F = Eigen::Vector3d::Zero();
  for (const auto& obst : this->envObstacles) {
    // Use true surface signed distance and outward normal for direction and magnitude
    double signedDistance = 0.0; // signed distance: > 0 outside, < 0 inside
    Eigen::Vector3d normalToObstSurface = Eigen::Vector3d::Zero(); // outward normal at closest surface point (world frame)
    obst.computeSignedDistanceAndNormal(queryPose.getPosition(), signedDistance, normalToObstSurface);
    // Ensure a valid normal; if degenerate, skip this obstacle
    const double normalMagnitude = normalToObstSurface.norm();
    if (normalMagnitude < NEAR_ZERO_THRESHOLD) continue;
    normalToObstSurface /= normalMagnitude;
    // Absolute influence semantics already enforced by withinInfluenceZone.
    // Define an effective distance d for the classic repulsive potential:
    //  - outside (sd >= 0): d = max(sd, eps)
    //  - inside (sd < 0): treat as very close to surface to generate strong outward push
    const double Q = std::max(this->influenceDistance, NEAR_ZERO_THRESHOLD);
    const double d = (signedDistance >= 0.0) ? std::max(signedDistance, NEAR_ZERO_THRESHOLD) : NEAR_ZERO_THRESHOLD;
    // Only contribute if within the influence distance
    if (d <= Q) {
      // Magnitude per Khatib potential [1]: eta * (1/d - 1/Q) * (1/d^2)
      const double inverseDistance = 1.0 / d;
      const double inverseDistanceSquared = inverseDistance * inverseDistance;
      const double inverseInfluenceDistance = 1.0 / Q;
      const double magnitude = this->repulsiveGain * (inverseDistance - inverseInfluenceDistance) * inverseDistanceSquared;
      if (magnitude > NEAR_ZERO_THRESHOLD) { F += normalToObstSurface * magnitude; }
    }
  }
  return F;
}

std::vector<double> PotentialField::computeInverseKinematics(
  const SpatialVector& targetPose, const std::vector<double>& seedJointAngles) const {
  std::vector<double> jointAngles;
  if (this->ikSolver) {
    Eigen::Isometry3d targetPoseIsometry = Eigen::Isometry3d::Identity();
    targetPoseIsometry.translate(targetPose.getPosition());
    targetPoseIsometry.rotate(targetPose.getOrientation());
    std::string errorMsg;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J;
    bool success = this->ikSolver->solve(targetPoseIsometry, seedJointAngles, jointAngles, J, errorMsg);
    if (!success) {
      jointAngles = std::vector<double>{}; // Return empty on failure
    }
  }
  return jointAngles;
}

PlannedPath PotentialField::planPath(
  const SpatialVector& startPose,
  const std::vector<double>& startJointAngles,
  const double dt,
  const double goalTolerance,
  const size_t maxIterations) {

  // Create path and initialize loop variables
  PlannedPath path;
  const double stepDt = (dt > 0.0) ? dt : 0.1;
  SpatialVector current = startPose;
  std::vector<double> jointAngles = startJointAngles;
  TaskSpaceTwist prevTwist; // previous applied twist (starts zero)
  double timeStamp = 0.0;

  // TODO(Sharwin24): Implement backtracking or escape maneuvers if robot links collide with environment
  // during path planning
  for (size_t iter = 0; iter < maxIterations; ++iter) {
    // Perform RK4 integration step to get next pose and the applied twist
    // including removal of opposing repulsive force components and enforcement of motion constraints
    auto [nextPoseRK4, appliedTwist] = this->rungeKuttaStep(current, prevTwist, stepDt);
    jointAngles = this->computeInverseKinematics(nextPoseRK4, jointAngles);

    // Record current state
    path.recordPathPoint(
      current, appliedTwist,
      jointAngles,
      timeStamp
    );

    // Check goal after recording
    const double translationalError = (current.getPosition() - this->goalPose.getPosition()).norm();
    const double rotationalError = current.angularDistance(this->goalPose);
    if (translationalError <= goalTolerance && rotationalError <= this->rotationalThreshold) {
      path.success = true;
      break;
    }

    // Update obstacles from new JointAngles
    this->updateObstaclesFromKinematics(jointAngles);

    // Update loop variables
    current = nextPoseRK4;
    prevTwist = appliedTwist; // supply velocity-limited twist as previous for next acceleration limiting
    timeStamp += stepDt;
  }

  path.dt = stepDt;
  path.numPoints = static_cast<unsigned int>(path.poses.size());
  if (!path.timeStamps.empty()) {
    path.duration = path.timeStamps.back();
  }
  else {
    path.duration = 0.0;
  }
  return path;
}

bool PotentialField::isRobotInCollisionWithEnvironment(double clearanceThreshold) const {
  auto robotObstacles = this->getObstaclesByGroup(ObstacleGroup::ROBOT);
  auto envObstacles = this->getObstaclesByGroup(ObstacleGroup::STATIC);
  //TODO(Sharwin24): Handle Dynamic Obstacles?

  // Setup collision request and result before looping over robot obstacles
  coal::CollisionRequest collisionRequest;
  if (clearanceThreshold > 0.0) {
    collisionRequest.security_margin = static_cast<coal::CoalScalar>(clearanceThreshold);
  }
  coal::CollisionResult collisionResult;

  for (const auto& robotLink : robotObstacles) {
    auto robotCollisionObj = robotLink.getCoalCollisionObject();
    if (!robotCollisionObj) continue;
    for (const auto& envObst : envObstacles) {
      auto envCollisionObj = envObst.getCoalCollisionObject();
      if (!envCollisionObj) continue;

      collisionResult.clear();
      coal::collide(robotCollisionObj.get(), envCollisionObj.get(), collisionRequest, collisionResult);
      if (collisionResult.isCollision()) { return true; }
    }
  }
  // Only after checking all robot-environment pairs, return false if no collisions detected
  return false;
}

std::vector<std::pair<double, double>> PotentialField::identifyPathCollisions(PlannedPath path, double clearanceThreshold) {
  // Return a list of <startTime, endTime> pairs indicating segments of the path in collision
  std::vector<std::pair<double, double>> collisionSegments;
  if (path.numPoints == 0) return collisionSegments;
  bool inCollision = false;
  double segmentStartTime = 0.0;
  for (size_t i = 0; i < path.numPoints; ++i) {
    SpatialVector pose = path.poses[i];
    // Update obstacles from kinematics at this pose
    const std::vector<double> jointAngles = path.jointAngles[i];
    updateObstaclesFromKinematics(jointAngles);
    bool collisionAtPose = this->isRobotInCollisionWithEnvironment(clearanceThreshold);
    if (collisionAtPose && !inCollision) {
      // Starting a new collision segment
      inCollision = true;
      segmentStartTime = path.timeStamps[i];
    }
    else if (!collisionAtPose && inCollision) {
      // Ending a collision segment
      inCollision = false;
      double segmentEndTime = path.timeStamps[i];
      collisionSegments.emplace_back(segmentStartTime, segmentEndTime);
    }
  }
  // If still in collision at the end of the path, close the final segment
  if (inCollision) {
    double segmentEndTime = path.timeStamps.back();
    collisionSegments.emplace_back(segmentStartTime, segmentEndTime);
  }
  return collisionSegments;
}
