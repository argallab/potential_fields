#ifndef PFIELD_DEMO_HPP
#define PFIELD_DEMO_HPP

#include "rclcpp/rclcpp.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using PlanPath = potential_fields_interfaces::srv::PlanPath;

class PFDemo : public rclcpp::Node {
public:
  PFDemo();
  ~PFDemo() = default;
private:
  std::string fixedFrame; // RViz fixed frame for visualization

  // Create a service client for planning paths
  rclcpp::Client<PlanPath>::SharedPtr planPathClient;
};

#endif // PFIELD_DEMO_HPP