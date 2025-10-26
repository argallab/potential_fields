#include "pfield/pfield.hpp"
#include "pfield/pf_obstacle.hpp"
#include "pfield/spatial_vector.hpp"

void PotentialField::initializeKinematics(const std::string& urdfFilePath, const double influenceZoneScale, const double repulsiveGain) {
  this->urdfFileName = urdfFilePath;
  this->pfKinematics = PFKinematics(this->urdfFileName, influenceZoneScale, repulsiveGain);
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
    if (obst.withinInfluenceZone(point)) { return true; }
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

TaskSpaceTwist PotentialField::evaluateLimitedVelocityAtPose(const SpatialVector& queryPose) const {
  return this->applyMotionConstraints(
    this->wrenchToTwist(this->evaluateWrenchAtPose(queryPose)),
    TaskSpaceTwist(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()),
    0.0
  );
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
  const double beta = 1.0; // Soft-saturation parameter, higher = more aggressive curve
  Eigen::Vector3d limitedLinear = softSaturateNorm(twist.linearVelocity, this->maxLinearVelocity, beta);
  Eigen::Vector3d limitedAngular = softSaturateNorm(twist.angularVelocity, this->maxAngularVelocity, beta);

  // If dt is valid, apply rate limits (acceleration limits)
  if (isPositiveFinite(dt)) {
    const double dVMaxLinear = this->maxLinearAcceleration * dt; // m/s^2 * s = m/s
    const double dVMaxAngular = this->maxAngularAcceleration * dt; // rad/s^2 * s = rad/s
    limitedLinear = rateLimitStep(prevTwist.linearVelocity, limitedLinear, dVMaxLinear);
    limitedAngular = rateLimitStep(prevTwist.angularVelocity, limitedAngular, dVMaxAngular);

    // Re-apply soft-saturation to prevent velocity limits
    limitedLinear = softSaturateNorm(limitedLinear, this->maxLinearVelocity, beta);
    limitedAngular = softSaturateNorm(limitedAngular, this->maxAngularVelocity, beta);
  }
  return TaskSpaceTwist(limitedLinear, limitedAngular);
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
    if (!obst.withinInfluenceZone(queryPose.getPosition())) continue;
    Eigen::Vector3d direction = queryPose.getPosition() - obst.getPosition();
    const double distance = direction.norm();
    const double distanceReciprocal = 1.0 / distance;
    const double distanceReciprocalSquared = 1.0 / (distance * distance);
    const double influenceReciprocal = 1.0 / obst.getInfluenceZoneScale();
    const double magnitude = obst.getRepulsiveGain() * (distanceReciprocal - influenceReciprocal) * distanceReciprocalSquared;
    if (magnitude > 0.0) F += direction.normalized() * magnitude;
  }
  return F;
}


PlannedPath PotentialField::planPath(
  const SpatialVector& startPose,
  const double dt,
  const double goalTolerance,
  std::shared_ptr<IKSolver> ikSolver,
  const size_t maxIterations) {
  PlannedPath path;
  const double stepDt = (dt > 0.0) ? dt : 0.1;

  auto getJointAnglesAtPose = [&](const SpatialVector& sv) -> std::vector<double> {
    std::vector<double> jointAngles;
    if (ikSolver) {
      Eigen::Isometry3d targetPose = Eigen::Isometry3d::Identity();
      targetPose.translate(sv.getPosition());
      targetPose.rotate(sv.getOrientation());
      std::vector<double> seed = ikSolver->getHomeConfiguration();
      std::string errorMsg;
      Eigen::Matrix<double, 6, Eigen::Dynamic> J;
      bool success = ikSolver->solve(targetPose, seed, jointAngles, J, errorMsg);
      if (!success) {
        jointAngles = std::vector<double>{}; // Return empty on failure
      }
    }
    return jointAngles;
  };

  // Already at goal? Single point path.
  const Eigen::Vector3d startDiff = startPose.getPosition() - this->goalPose.getPosition();
  if (startDiff.norm() <= goalTolerance) {
    path.addPoint(startPose, TaskSpaceTwist(), getJointAnglesAtPose(startPose), 0.0);
    path.dt = stepDt;
    path.duration = 0.0;
    path.numPoints = 1;
    return path;
  }

  SpatialVector current = startPose;
  TaskSpaceTwist prevTwist; // previous applied twist (starts zero)
  double timeStamp = 0.0;

  for (size_t iter = 0; iter < maxIterations; ++iter) {
    // Evaluate (velocity-limited) twist at current pose for logging
    TaskSpaceTwist evalTwist = this->evaluateVelocityAtPose(current);
    TaskSpaceTwist limitedTwist = this->applyMotionConstraints(evalTwist, prevTwist, stepDt);
    // Record current state
    path.addPoint(
      current, evalTwist,
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

    // Advance pose using constrained interpolation (acceleration limits applied internally)
    Eigen::Vector3d nextPosition = this->integrateLinearVelocity(
      current.getPosition(), limitedTwist.linearVelocity, stepDt);
    Eigen::Quaterniond nextOrientation = this->integrateAngularVelocity(
      current.getOrientation(), limitedTwist.angularVelocity, stepDt);
    // Update obstacles from new JointAngles
    auto jointAngles = path.jointAngles.back();
    this->pfKinematics.updateObstaclesFromJointAngles(jointAngles);

    // Update loop variables
    current = SpatialVector(nextPosition, nextOrientation);
    prevTwist = limitedTwist; // supply velocity-limited twist as previous for next acceleration limiting
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
