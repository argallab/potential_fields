#ifndef ROBOT_PARSER_HP
#define ROBOT_PARSER_HP

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
#include "pfield/pfield.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "urdf/model.h"
#include "urdf_parser/urdf_parser.h"
#include <fstream>
#include "potential_fields_interfaces/msg/obstacle.hpp"

using Obstacle = potential_fields_interfaces::msg::Obstacle;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class RobotParser : public rclcpp::Node {
public:
  RobotParser();
  ~RobotParser() = default;

private:
  double timerFreq; // Timer frequency [Hz]
  std::string robotDescription; // Robot Description
  std::string fixedFrame; // RViz fixed frame

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Obstacle>::SharedPtr obstaclePub;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamicTfBroadcaster;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  void timerCallback();

  std::vector<Obstacle> parseURDF();

  Obstacle obstacleFromCollisionObject(int id, const urdf::Collision& collisionObject,
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
};

#endif // ROBOT_PARSER_HP