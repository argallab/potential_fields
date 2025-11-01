#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"

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
  auto itIndex = this->obstacleIndex.find(frameID);
  if (itIndex != this->obstacleIndex.end()) {
    // Update existing obstacle in place
    this->obstacles[itIndex->second] = obstacle;
  }
  else {
    // Append new obstacle and record index
    this->obstacles.push_back(obstacle);
    this->obstacleIndex.emplace(frameID, this->obstacles.size() - 1);
  }
}

void PotentialField::addObstacles(const std::vector<PotentialFieldObstacle>& obstacles) {
  for (const auto& obst : obstacles) { this->addObstacle(obst); }
}

bool PotentialField::removeObstacle(const std::string& obstacleFrameID) {
  auto itIndex = this->obstacleIndex.find(obstacleFrameID);
  if (itIndex == this->obstacleIndex.end()) { return false; }
  size_t idx = itIndex->second;
  // Swap erase to keep indices valid with minimal moves
  size_t last = this->obstacles.size() - 1;
  if (idx != last) {
    std::swap(this->obstacles[idx], this->obstacles[last]);
    // Update moved obstacle's index map
    this->obstacleIndex[this->obstacles[idx].getFrameID()] = idx;
  }
  this->obstacles.pop_back();
  this->obstacleIndex.erase(itIndex);
  return true;
}

void PotentialField::clearObstacles() { this->obstacles.clear(); this->obstacleIndex.clear(); }

PotentialFieldObstacle PotentialField::getObstacleByID(const std::string& obstacleFrameID) const {
  auto itIndex = this->obstacleIndex.find(obstacleFrameID);
  if (itIndex == this->obstacleIndex.end()) {
    throw std::invalid_argument("Obstacle with the given ID does not exist.");
  }
  return this->obstacles[itIndex->second];
}

bool PotentialField::isPointInsideObstacle(Eigen::Vector3d point) const {
  for (const auto& obst : this->obstacles) {
    if (obst.withinObstacle(point)) { return true; }
  }
  return false;
}

bool PotentialField::isPointWithinInfluenceZone(Eigen::Vector3d point) const {
  for (const auto& obst : this->obstacles) {
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

TaskSpaceTwist PotentialField::constrainedTwistAtPose(const SpatialVector& pose, const TaskSpaceTwist& prevTwist, const double dt) {
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
  const Eigen::Vector3d direction = queryPose.getPosition() - this->goalPose.getPosition();
  const double euclideanDistance = direction.norm();
  if (euclideanDistance <= this->translationalTolerance) return Eigen::Vector3d::Zero();
  else return direction.normalized() * (-this->attractiveGain * euclideanDistance);
}

Eigen::Vector3d PotentialField::computeAttractiveMoment(const SpatialVector& queryPose) const {
  Eigen::Quaterniond orientationDiff = queryPose.getOrientation().conjugate() * this->goalPose.getOrientation();
  orientationDiff.normalize();
  const double angularDistance = queryPose.angularDistance(this->goalPose);
  if (angularDistance <= this->rotationalThreshold) return Eigen::Vector3d::Zero();
  Eigen::Vector3d u = orientationDiff.vec() / orientationDiff.vec().norm();
  return -this->rotationalAttractiveGain * (angularDistance * u);
}

Eigen::Vector3d PotentialField::computeRepulsiveForceLinear(const SpatialVector& queryPose) const {
  Eigen::Vector3d F = Eigen::Vector3d::Zero();
  for (const auto& obst : this->obstacles) {
    // if (!obst.withinInfluenceZone(queryPose.getPosition())) continue;
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
    if (d < Q) {
      // Magnitude per Khatib potential: eta * (1/d - 1/Q) * (1/d^2)
      const double inverseDistance = 1.0 / d;
      const double inverseDistanceSquared = inverseDistance * inverseDistance;
      const double inverseInfluenceDistance = 1.0 / Q;
      const double magnitude = this->repulsiveGain * (inverseDistance - inverseInfluenceDistance) * inverseDistanceSquared;
      if (magnitude > NEAR_ZERO_THRESHOLD) { F += normalToObstSurface * magnitude; }
    }
  }
  return F;
}


PlannedPath PotentialField::planPath(
  const SpatialVector& startPose,
  const double dt,
  const double goalTolerance,
  const size_t maxIterations) {
  // Helper function to get joint angles at a given pose
  auto getJointAnglesAtPose = [&](const SpatialVector& sv) -> std::vector<double> {
    std::vector<double> jointAngles;
    if (this->ikSolver) {
      Eigen::Isometry3d targetPose = Eigen::Isometry3d::Identity();
      targetPose.translate(sv.getPosition());
      targetPose.rotate(sv.getOrientation());
      std::vector<double> seed = this->ikSolver->getHomeConfiguration();
      std::string errorMsg;
      Eigen::Matrix<double, 6, Eigen::Dynamic> J;
      bool success = this->ikSolver->solve(targetPose, seed, jointAngles, J, errorMsg);
      if (!success) {
        jointAngles = std::vector<double>{}; // Return empty on failure
      }
    }
    return jointAngles;
  };

  // Create path and initialize loop variables
  PlannedPath path;
  const double stepDt = (dt > 0.0) ? dt : 0.1;
  SpatialVector current = startPose;
  TaskSpaceTwist prevTwist; // previous applied twist (starts zero)
  double timeStamp = 0.0;

  for (size_t iter = 0; iter < maxIterations; ++iter) {
    // Evaluate wrench at current pose after removing opposing repulsive force components
    // to prevent pinning and stagnation in local minima
    // TaskSpaceWrench wrench = this->evaluateWrenchAtPoseWithOpposingForceRemoval(current);
    // TaskSpaceTwist constrainedTwist = this->applyMotionConstraints(this->wrenchToTwist(wrench), prevTwist, stepDt);
    auto [nextPoseRK4, appliedTwist] = this->rungeKuttaStep(current, prevTwist, stepDt);

    // Record current state
    path.recordPathPoint(
      current, appliedTwist,
      getJointAnglesAtPose(current),
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
    auto jointAngles = path.jointAngles.back();
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

bool PotentialField::isPathStagnated(const PlannedPath& path) {
  const size_t N = path.poses.size();
  if (N < static_cast<size_t>(this->stagnationWindowMinPoints)) return false;

  // Analyze the last N points in the path
  const size_t windowSize = this->stagnationWindowMinPoints;
  const size_t startIdx = N - windowSize;

  // Save time difference
  const double t0 = path.timeStamps[startIdx];
  const double t1 = path.timeStamps.back();
  const double timeDiff = std::max(t1 - t0, 1e-6);

  // Compute distances to goal at start and end of window
  const Eigen::Vector3d goalPos = this->goalPose.getPosition();
  const double d0 = (path.poses[startIdx].getPosition() - goalPos).norm();
  const double d1 = (path.poses.back().getPosition() - goalPos).norm();

  // Progress rate toward goal (positive if getting closer)
  const double progressRate = (d0 - d1) / timeDiff;

  // RMS linear speed over the window
  double sumSq = 0.0;
  int count = 0;
  for (size_t i = startIdx; i < N - 1; ++i) {
    const Eigen::Vector3d v = path.twists[i].getLinearVelocity();
    sumSq += v.squaredNorm();
    ++count;
  }
  const double speedRms = std::sqrt(sumSq / std::max(count, 1));

  const bool farFromGoal = (d1 > this->translationalTolerance);

  // Determine stagnation based on thresholds
  return farFromGoal &&
    (progressRate < this->stagnationProgressRateThreshold) &&
    (speedRms < this->stagnationSpeedRmsThreshold);
}
