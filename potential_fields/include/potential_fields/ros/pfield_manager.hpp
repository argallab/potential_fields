#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/msg/obstacle_array.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/srv/compute_autonomy_vector.hpp"

#include "pfield/pfield.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Path = nav_msgs::msg::Path;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Obstacle = potential_fields_interfaces::msg::Obstacle;
using ObstacleArray = potential_fields_interfaces::msg::ObstacleArray;
using PlanPath = potential_fields_interfaces::srv::PlanPath;
using ComputeAutonomyVector = potential_fields_interfaces::srv::ComputeAutonomyVector;
using JointState = sensor_msgs::msg::JointState;

class PotentialFieldManager : public rclcpp::Node {
public:
  PotentialFieldManager();
  ~PotentialFieldManager() = default;

  static Quaternion getQuaternionFromYaw(double yaw) {
    Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
    return q;
  }

private:
  double visualizerFrequency; // frequency for updating visualization [Hz]
  double attractiveGain; // Attractive gain [Ns/m]
  double rotationalAttractiveGain; // Rotational attractive gain [Ns/m]
  double repulsiveGain; // Repulsive gain [Ns/m]
  double maxForce; // Maximum force [N]
  double influenceZoneScale; // Influence zone scaling factor
  std::string fixedFrame; // RViz fixed frame for visualization and PF computation
  PotentialField pField; // Potential field instance
  PotentialField planningPField; // PF instance for path planning
  double visualizerBufferArea; // Extra area around obstacles and goal to visualize the PF [m]
  double fieldResolution; // Resolution of the potential field grid [m]

  rclcpp::TimerBase::SharedPtr timer; // Timer to periodically update the potential field
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster; // Dynamic transform broadcaster
  std::shared_ptr<tf2_ros::Buffer> tfBuffer; // TF buffer for transform lookups
  std::shared_ptr<tf2_ros::TransformListener> tfListener; // TF Listener for populating the TF buffer
  rclcpp::Publisher<JointState>::SharedPtr planningJointStatePub; // Publisher for planning-copy of JointStates
  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub; // Publisher for visualization markers
  rclcpp::Subscription<PoseStamped>::SharedPtr goalPoseSub; // Subscriber for the goal pose
  rclcpp::Subscription<ObstacleArray>::SharedPtr obstacleSub; // Subscriber for obstacles
  rclcpp::Service<PlanPath>::SharedPtr pathPlanningService; // Service to obtain a path from a query point to the goal
  rclcpp::Service<ComputeAutonomyVector>::SharedPtr autonomyVectorService; // Service to compute velocity vector at a given pose

  /**
   * @brief Interpolates a path from a start pose to the goal pose.
   *
   * @note Accounts for the robot moving along the path with
   *       its geometry contributing to the potential field as obstacles.
   *
   * @param start The starting pose of the path
   * @param deltaTime The time step for each interpolation segment [s]
   * @param goalTolerance The tolerance for reaching the goal [m]
   * @return Path The interpolated path from start to goal
   */
  Path interpolatePath(const SpatialVector& start, double deltaTime, double goalTolerance);

  void getPFLimits(double& minX, double& maxX, double& minY, double& maxY, double& minZ, double& maxZ);

  void visualizePF();
  MarkerArray createObstacleMarkers();
  MarkerArray createGoalMarker();
  MarkerArray createPotentialVectorMarkers();

  void exportFieldDataToCSV(const std::string& filename);

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP
