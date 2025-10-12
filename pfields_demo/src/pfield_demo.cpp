#include "pfield_demo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "potential_fields_interfaces/srv/plan_path.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <filesystem>

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
    pathPlanRequest->delta_time = 0.1; // 100 ms between waypoints
    pathPlanRequest->goal_tolerance = 0.1; // 10 cm tolerance

    // Publish the goal pose
    this->goalPosePub->publish(goalPose);

    RCLCPP_INFO(this->get_logger(), "Sending plan_path request (async)...");

    // Create a publisher to publish the returned end-effector path when the response arrives
    auto pathPub = this->create_publisher<nav_msgs::msg::Path>("/nav_msgs/msg/Path", 10);

    // Send request asynchronously and attach a callback to process the result
    this->planPathClient->async_send_request(
      pathPlanRequest,
      [this, pathPub](rclcpp::Client<PlanPath>::SharedFuture future) {
      auto pathPlanResponse = future.get();
      if (!pathPlanResponse) {
        RCLCPP_ERROR(this->get_logger(), "plan_path service returned an empty response (async)");
        return;
      }

      // Log a small summary of the returned trajectories to help debug crashes
      size_t ee_path_len = pathPlanResponse->end_effector_path.poses.size();
      size_t jt_points = pathPlanResponse->joint_trajectory.points.size();
      size_t ee_vels = pathPlanResponse->end_effector_velocity_trajectory.size();
      RCLCPP_INFO(this->get_logger(), "(async) Received plan_path response: success=%s, end_effector_path.len=%zu, joint_trajectory.points=%zu, ee_velocity_traj=%zu",
        pathPlanResponse->success ? "true" : "false", ee_path_len, jt_points, ee_vels);

      if (ee_path_len > 0) {
        const auto& p = pathPlanResponse->end_effector_path.poses.front().pose.position;
        RCLCPP_INFO(this->get_logger(), "(async) First EE pose: (%.4f, %.4f, %.4f)", p.x, p.y, p.z);
      }
      if (ee_vels > 0) {
        const auto& v = pathPlanResponse->end_effector_velocity_trajectory.front().twist.linear;
        RCLCPP_INFO(this->get_logger(), "(async) First EE linear velocity: (%.6f, %.6f, %.6f)", v.x, v.y, v.z);
      }

      // Publish the path for visualization (if present)
      if (ee_path_len > 0) {
        pathPub->publish(pathPlanResponse->end_effector_path);
      }

      // Save CSV like the python demo
      try {
        this->save_planned_path_response(pathPlanResponse);
      }
      catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save planned path CSV: %s", e.what());
      }
    }
    );
  });
}

// Helper: save PlanPath response to CSV in data/ with same layout as pf_demo.py
void PFDemo::save_planned_path_response(const std::shared_ptr<PlanPath::Response>& res) {
  using std::chrono::nanoseconds;
  // ensure data directory exists
  std::filesystem::path data_dir("data");
  if (!std::filesystem::exists(data_dir)) {
    std::error_code ec;
    std::filesystem::create_directories(data_dir, ec);
    if (ec) {
      RCLCPP_WARN(this->get_logger(), "Could not create data directory: %s", ec.message().c_str());
    }
  }

  // prepare series
  const auto& path_poses = res->end_effector_path.poses;
  const auto& ee_vels = res->end_effector_velocity_trajectory;
  const auto& jt = res->joint_trajectory;
  const auto jt_points = jt.points;

  size_t n_path = path_poses.size();
  size_t n_vel = ee_vels.size();
  size_t n_jt = jt_points.size();

  std::vector<std::string> joint_names;
  if (!jt.joint_names.empty()) {
    joint_names = jt.joint_names;
  }
  size_t n_joints = joint_names.size();

  // compute times (fallback logic mirrors python)
  std::vector<double> times;
  if (n_path > 0 && (path_poses[0].header.stamp.sec != 0 || path_poses[0].header.stamp.nanosec != 0)) {
    double t0 = path_poses[0].header.stamp.sec + path_poses[0].header.stamp.nanosec * 1e-9;
    for (const auto& p : path_poses) {
      double ts = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9;
      times.push_back(ts - t0);
    }
  }
  else if (n_vel > 0 && (ee_vels[0].header.stamp.sec != 0 || ee_vels[0].header.stamp.nanosec != 0)) {
    double t0 = ee_vels[0].header.stamp.sec + ee_vels[0].header.stamp.nanosec * 1e-9;
    for (const auto& v : ee_vels) {
      double ts = v.header.stamp.sec + v.header.stamp.nanosec * 1e-9;
      times.push_back(ts - t0);
    }
  }
  else if (n_jt > 0) {
    // use time_from_start if available
    double t0 = 0.0;
    if (jt_points[0].time_from_start.sec != 0 || jt_points[0].time_from_start.nanosec != 0) {
      t0 = jt_points[0].time_from_start.sec + jt_points[0].time_from_start.nanosec * 1e-9;
    }
    for (const auto& pt : jt_points) {
      double tfs = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
      times.push_back(tfs - t0);
    }
  }
  else {
    double est_dt = 0.1;
    size_t maxlen = std::max({n_path, n_vel, n_jt, (size_t)1});
    times.resize(maxlen);
    for (size_t i = 0; i < maxlen; ++i) times[i] = i * est_dt;
  }

  size_t n_rows = std::max({times.size(), n_path, n_vel, n_jt});

  // prepare CSV filename
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  std::string filename = (data_dir / ("planned_path_" + ss.str() + ".csv")).string();

  std::ofstream csv(filename);
  if (!csv.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file for writing: %s", filename.c_str());
    return;
  }

  // header
  std::vector<std::string> header = {"time_s", "ee_px", "ee_py", "ee_pz", "ee_qx", "ee_qy", "ee_qz", "ee_qw", "ee_vx", "ee_vy", "ee_vz"};
  if (!joint_names.empty()) {
    for (const auto& jn : joint_names) header.push_back(jn);
  }
  else {
    // placeholder joint names to match python behavior
    size_t cols = (n_rows > 0 ? n_joints : 0);
    for (size_t i = 0; i < cols; ++i) header.push_back("joint_" + std::to_string(i));
  }

  // write header
  for (size_t i = 0; i < header.size(); ++i) {
    if (i) csv << ',';
    csv << header[i];
  }
  csv << '\n';

  // iterate rows, mirroring python's last-known logic
  std::vector<double> last_joints(n_joints, std::numeric_limits<double>::quiet_NaN());
  bool has_last_pose = false;
  geometry_msgs::msg::Pose last_pose;
  bool has_last_vel = false;
  geometry_msgs::msg::Vector3 last_vel; last_vel.x = last_vel.y = last_vel.z = 0.0;

  for (size_t i = 0; i < n_rows; ++i) {
    double tval = (i < times.size()) ? times[i] : (times.empty() ? i * 0.1 : (times.back() + (i - times.size() + 1) * 0.1));

    geometry_msgs::msg::Pose pose;
    if (i < n_path) {
      pose = path_poses[i].pose;
      last_pose = pose;
      has_last_pose = true;
    }
    else if (has_last_pose) {
      pose = last_pose;
    }
    else {
      // default empty pose leaves zeros
    }

    geometry_msgs::msg::Vector3 vel;
    if (i < n_vel) {
      vel = ee_vels[i].twist.linear;
      last_vel = vel;
      has_last_vel = true;
    }
    else if (has_last_vel) {
      vel = last_vel;
    }
    else {
      vel.x = vel.y = vel.z = 0.0;
    }

    // joints
    std::vector<double> joints_row(n_joints, std::numeric_limits<double>::quiet_NaN());
    if (i < n_jt) {
      const auto& positions = jt_points[i].positions;
      for (size_t j = 0; j < n_joints; ++j) {
        if (j < positions.size()) {
          joints_row[j] = positions[j];
          last_joints[j] = positions[j];
        }
        else {
          if (!std::isnan(last_joints[j])) joints_row[j] = last_joints[j];
        }
      }
    }
    else {
      for (size_t j = 0; j < n_joints; ++j) {
        if (!std::isnan(last_joints[j])) joints_row[j] = last_joints[j];
      }
    }

    // write row
    csv << tval << ','
      << pose.position.x << ',' << pose.position.y << ',' << pose.position.z << ','
      << pose.orientation.x << ',' << pose.orientation.y << ',' << pose.orientation.z << ',' << pose.orientation.w << ','
      << vel.x << ',' << vel.y << ',' << vel.z;

    for (size_t j = 0; j < n_joints; ++j) {
      csv << ',';
      if (!std::isnan(joints_row[j])) csv << joints_row[j];
    }
    csv << '\n';
  }

  csv.close();
  RCLCPP_INFO(this->get_logger(), "Saved planned path CSV to %s", std::filesystem::absolute(filename).string().c_str());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PFDemo>());
  rclcpp::shutdown();
  return 0;
}
