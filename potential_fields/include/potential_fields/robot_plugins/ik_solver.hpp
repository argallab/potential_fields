#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// IKSolver Interface for different IK solver implementations

struct PlanningParameters {
  std::string robotBaseLink; // The robot's base link name
  std::string robotEndEffectorLink; // The robot's end-effector link name (tip link for IK)
  double deltaTimeStemp; // Time step for each planning iteration [s]
  double goalTolerance; // Distance tolerance to consider the goal reached [m]
  double ikTimeoutMilliseconds; // Timeout for each IK solve attempt [ms]
  unsigned int maxSteps; // Maximum number of planning steps
  int maxIKFailures; // Maximum number of consecutive IK failures before aborting
  double velocityScale;
  double maxLinearStep;
  double maxAngularStep;
};

class IKSolver {
public:
  explicit IKSolver(const std::string& name) : name(name) {}
  virtual ~IKSolver() = default;

  // Primary IK (targetPose is base->EE as an Isometry) and Jacobian function
  virtual bool solve(
    const Eigen::Isometry3d& targetPose,
    const std::vector<double>& seed,
    std::vector<double>& solution,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J) = 0;

  // Optional Jacobian-only function (for joint velocities from Cartesian twist)
  virtual bool computeJacobian(
    const std::vector<double>& jointPositions,
    Eigen::Matrix<double, 6, Eigen::Dynamic>& J) = 0;

  virtual std::vector<double> getHomeConfiguration() const = 0;

  virtual std::vector<std::string> getJointNames() const = 0;

  virtual std::string getName() const { return this->name; }

protected:
  std::string name;
};

#endif // IK_SOLVER_HPP
