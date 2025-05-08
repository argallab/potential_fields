#ifndef PFIELD_MANAGER_HPP
#define PFIELD_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
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

private:
  float timerFreq; // Timer frequency [Hz]
  float attractiveGain; // Attractive gain [N]
  float repulsiveGain; // Repulsive gain [N]
  float max_force; // Maximum force [N]
  float influence_radius_scale; // Scale for influence radius

  // Potential field object
  PotentialField pField;

  // Timer to perodically update the potential field
  rclcpp::TimerBase::SharedPtr timer;

  // Publisher for visualization markers
  rclcpp::Publisher<MarkerArray>::SharedPtr markerPub;

  void drawPotentialField();
  MarkerArray createObstacleMarkers();
  Marker createGoalMarker();

  void timerCallback();
};

#endif // PFIELD_MANAGER_HPP