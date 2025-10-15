#ifndef PFIELD_DEMO_HPP
#define PFIELD_DEMO_HPP

#include "rclcpp/rclcpp.hpp"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using PlanPath = potential_fields_interfaces::srv::PlanPath;

class PFDemo : public rclcpp::Node {
public:
  PFDemo();
  ~PFDemo() = default;
private:
  std::string fixedFrame; // RViz fixed frame for visualization

  // Create a service client for planning paths
  rclcpp::Client<PlanPath>::SharedPtr planPathClient;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPosePub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr runPlanPathDemoService;
  // Save a PlanPath response to CSV (same format as pf_demo.py)
  void save_planned_path_response(const std::shared_ptr<potential_fields_interfaces::srv::PlanPath::Response>& res);
};

#endif // PFIELD_DEMO_HPP
