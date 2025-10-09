#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "potential_fields_interfaces/srv/plan_path.hpp"

PFDemo::PFDemo() : Node("pfield_demo") {
  RCLCPP_INFO(this->get_logger(), "PFDemo Initialized");

  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  // Static transform is now published via launch (tf2_ros/static_transform_publisher)

  // Wait for the service to be available
  this->planPathClient = this->create_client<PlanPath>("pfield/plan_path");
  while (!this->planPathClient->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the plan_path service to be available...");
  }
  RCLCPP_INFO(this->get_logger(), "Plan path service is available.");

  // Initialize demo service
  this->runPlanPathDemoService = this->create_service<std_srvs::srv::Empty>(
    "run_plan_path_demo",
    [this](const std_srvs::srv::Empty::Request::SharedPtr request,
      std_srvs::srv::Empty::Response::SharedPtr response) {
    (void)request;
    (void)response;
    RCLCPP_INFO(this->get_logger(), "Running plan path demo...");

    // Create a request for the plan_path service
    auto pathPlanRequest = std::make_shared<PlanPath::Request>();
    // Define start and goal poses
    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.frame_id = this->fixedFrame;
    startPose.pose.position.x = 0.0;
    startPose.pose.position.y = 0.0;
    startPose.pose.position.z = 0.0;
    startPose.pose.orientation.w = 1.0; // Neutral orientation

    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header.frame_id = this->fixedFrame;
    goalPose.pose.position.x = 1.0; // Move to x=1.0
    goalPose.pose.position.y = 1.0; // Move to y=1.0
    goalPose.pose.position.z = 0.0;
    goalPose.pose.orientation.w = 1.0; // Neutral orientation

    pathPlanRequest->start = startPose;
    pathPlanRequest->goal = goalPose;

    // Call the plan_path service
    auto future = this->planPathClient->async_send_request(pathPlanRequest);
    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to call plan_path service");
      return;
    }
    auto pathPlanResponse = future.get();
    if (!pathPlanResponse || !pathPlanResponse->success) {
      RCLCPP_ERROR(this->get_logger(), "Path planning failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Path planning succeeded, received %zu waypoints",
      pathPlanResponse->end_effector_path.poses.size());
  });
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}
