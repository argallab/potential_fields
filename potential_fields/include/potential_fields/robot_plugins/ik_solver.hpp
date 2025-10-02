#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// IKSolver Interface for different IK solver implementations

class IKSolver {
public:
  virtual ~IKSolver() = default;
  virtual bool computeIK(const geometry_msgs::msg::PoseStamped& endEffectorPose,
    sensor_msgs::msg::JointState& jointState) = 0;
};

#endif // !IK_SOLVER_H
