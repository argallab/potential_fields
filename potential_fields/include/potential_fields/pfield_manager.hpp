#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pfield.hpp"

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Pose = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using Quaternion = geometry_msgs::msg::Quaternion;

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
  double attractiveGain; // Attractive gain [N]
  double rotationalAttractiveGain; // Rotational attractive gain [N]
  double repulsiveGain; // Repulsive gain [N]
  double maxForce; // Maximum force [N]
  SpatialVector queryPoint; // Query point for the potential field to "animate"

  // Potential field object
  PotentialField pField;

  // Timer to perodically update the potential field
  rclcpp::TimerBase::SharedPtr timer;

  // Publisher for visualization markers
  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub;

  // Subscriber for the goal pose
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalPoseSub;

  void updateQueryPoint();

  void visualizePF();
  MarkerArray createObstacleMarkers();
  MarkerArray createGoalMarker();
  MarkerArray createPotentialVectorMarkers();
  MarkerArray createQueryPointMarker();

  void createCSV(const std::string& filename);

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP
