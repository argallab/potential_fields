#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pfield.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include <fstream>

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;
using Path = nav_msgs::msg::Path;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class PotentialFieldManager : public rclcpp::Node {
public:
  PotentialFieldManager();
  ~PotentialFieldManager() = default;

  static Quaternion getQuaternionFromYaw(double yaw) {
    Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2);
    q.w = cos(yaw / 2);
    return q;
  }

private:
  double timerFreq; // Timer frequency [Hz]
  double attractiveGain; // Attractive gain [Ns/m]
  double rotationalAttractiveGain; // Rotational attractive gain [Ns/m]
  double repulsiveGain; // Repulsive gain [Ns/m]
  double maxForce; // Maximum force [N]
  std::string urdfFilePath; // Path to the URDF file describing the robot and scene
  SpatialVector queryPoint; // Query point for the potential field to "animate"
  Path queryPath; // History of query points for visualization of path

  // Potential field object
  PotentialField pField;

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

  Eigen::Affine3d computeLinkTransform(const urdf::LinkConstSharedPtr& link,
    std::map<std::string, Eigen::Affine3d>& transforms);

  void updateQueryPoint();
  void visualizePF();
  void updateTransforms();
  void transformsFromURDF();

  MarkerArray createObstacleMarkers();
  MarkerArray createGoalMarker();
  MarkerArray createPotentialVectorMarkers();
  MarkerArray createQueryPointMarker();

  PotentialFieldObstacle obstacleFromCollisionObject(int id,
    const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    double influenceZoneScale = 2.0, double repulsiveGain = 5.0);

  void createCSV(const std::string& filename);

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP
