#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pfield/pfield.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "urdf/model.h"
#include "potential_fields_interfaces/msg/obstacle.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "potential_fields_interfaces/srv/compute_autonomy_vector.hpp"
#include <vector>
#include <fstream>

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Path = nav_msgs::msg::Path;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Obstacle = potential_fields_interfaces::msg::Obstacle;
using PlanPath = potential_fields_interfaces::srv::PlanPath;
using ComputeAutonomyVector = potential_fields_interfaces::srv::ComputeAutonomyVector;

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
  double timerFreq; // Timer frequency [Hz]
  double attractiveGain; // Attractive gain [Ns/m]
  double rotationalAttractiveGain; // Rotational attractive gain [Ns/m]
  double repulsiveGain; // Repulsive gain [Ns/m]
  double maxForce; // Maximum force [N]
  std::string fixedFrame; // RViz fixed frame for visualization and PF computation
  PotentialField pField; // Potential field instance

  // Timer to perodically update the potential field
  rclcpp::TimerBase::SharedPtr timer;

  // Dynamic transform broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;

  // TF Listener
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  // Publisher for visualization markers
  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub;

  // Publisher for the path of the query point to visualize its trajectory
  rclcpp::Publisher<Path>::SharedPtr pathPub;

  // Subscriber for the goal pose
  rclcpp::Subscription<PoseStamped>::SharedPtr goalPoseSub;

  // Subscriber for obstacles
  rclcpp::Subscription<Obstacle>::SharedPtr obstacleSub;

  // Service to obtain a path from a query point to the goal
  rclcpp::Service<PlanPath>::SharedPtr pathPlanningService;

  // Service to compute the autonomy vector at a given pose
  rclcpp::Service<ComputeAutonomyVector>::SharedPtr autonomyVectorService;

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

  void visualizePF();
  MarkerArray createObstacleMarkers();
  MarkerArray createGoalMarker();
  MarkerArray createPotentialVectorMarkers();

  void exportFieldDataToCSV(const std::string& filename);

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP
