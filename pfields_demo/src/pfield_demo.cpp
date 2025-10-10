#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "potential_fields_interfaces/srv/plan_path.hpp"

template<typename T>
using ServiceResponseFuture = rclcpp::Client<T>::SharedFuture;

PFDemo::PFDemo() : Node("pfield_demo") {
  RCLCPP_INFO(this->get_logger(), "PFDemo Initialized");

  this->fixedFrame = this->declare_parameter("fixed_frame", "world"); // RViz fixed frame
  // Get parameters from yaml file
  this->fixedFrame = this->get_parameter("fixed_frame").as_string();

  this->goalPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

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
    "/pfield_demo/run_plan_path_demo",
    [this](
      [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr request,
      [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr response) {
    RCLCPP_INFO(this->get_logger(), "Running plan path demo...");

    // Create a request for the plan_path service
    auto pathPlanRequest = std::make_shared<PlanPath::Request>();
    // Define start and goal poses
    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.stamp = this->now();
    startPose.header.frame_id = this->fixedFrame;
    startPose.pose.position.x = 0.466;
    startPose.pose.position.y = 0.0;
    startPose.pose.position.z = 0.644;
    startPose.pose.orientation.x = 0.0;
    startPose.pose.orientation.y = 0.0;
    startPose.pose.orientation.z = 0.0;
    startPose.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped goalPose;
    goalPose.header.stamp = this->now();
    goalPose.header.frame_id = this->fixedFrame;
    goalPose.pose.position.x = 0.478;
    goalPose.pose.position.y = -0.117513;
    goalPose.pose.position.z = 0.0;
    goalPose.pose.orientation.y = 0.0;
    goalPose.pose.orientation.z = 0.0;
    goalPose.pose.orientation.x = 0.0;
    goalPose.pose.orientation.w = 1.0;

    pathPlanRequest->start = startPose;
    pathPlanRequest->goal = goalPose;

    // Publish the goal pose
    this->goalPosePub->publish(goalPose);

    // Call the plan_path service
    PlanPath::Response::SharedPtr pathPlanResponse;
    auto future_result = this->planPathClient->async_send_request(
      pathPlanRequest,
      [this, &pathPlanResponse](ServiceResponseFuture<PlanPath> future) {
      auto response = future.get();
      if (response) {
        pathPlanResponse = response;
      }
    });

    // Once received paths, publish a path to /nav_msgs/msg/Path for visualization
    auto pathPub = this->create_publisher<nav_msgs::msg::Path>("/nav_msgs/msg/Path", 10);
    pathPub->publish(pathPlanResponse->end_effector_path);
  });
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}
