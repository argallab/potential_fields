#!/usr/bin/env python3
import csv
import os
from datetime import datetime
import math

import rclpy
from rclpy.node import Node
from potential_fields_interfaces.srv import PlanPath
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class PFDemo(Node):
    def __init__(self):
        super().__init__('pf_demo')
        self.get_logger().info('PFDemo Initialized')
        self.fixed_frame = self.declare_parameter(
            'fixed_frame', 'world').get_parameter_value().string_value

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cli = self.create_client(PlanPath, 'pfield/plan_path')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            if rclpy.ok() is False:
                self.get_logger().error(
                    'Interrupted while waiting for the service. Continuing without service (shutdown requested).')
                break
            self.get_logger().info('Waiting for the plan_path service to be available...')
        self.get_logger().info('Plan path service is available.')

        # Service to trigger demo
        self.create_service(
            Empty, '/pfield_demo/run_plan_path_demo', self.run_demo_callback)
        self.get_logger().info('Ready to run plan path demo via service call.')

    def run_demo_callback(self, request, response):
        self.get_logger().info('Running plan path demo...')
        req = PlanPath.Request()

        start = PoseStamped()
        start.header.frame_id = self.fixed_frame
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.466
        start.pose.position.y = 0.0
        start.pose.position.z = 0.644
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0
        start.pose.orientation.z = 0.0
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = self.fixed_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 0.478
        goal.pose.position.y = -0.117513
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        req.start = start
        req.goal = goal
        req.delta_time = 0.1
        req.goal_tolerance = 0.1

        # publish goal for visualization
        self.goal_pub.publish(goal)

        # call service asynchronously; handle response in done-callback
        self.get_logger().info('Sending plan_path request (async)...')
        future = self.cli.call_async(req)
        # attach a done callback to process the result when ready
        future.add_done_callback(self._on_plan_path_response)
        # return immediately from the demo service; the response will be processed asynchronously
        return response

    def save_planned_path_response(self, plan_path_response):
        # Save a CSV file with the planned path details for offline plotting.
        # CSV columns:
        # time, ee_px, ee_py, ee_pz, ee_qx, ee_qy, ee_qz, ee_qw, ee_vx, ee_vy, ee_vz, <joint_name_0>, <joint_name_1>, ...

        # Determine lengths
        path_poses = plan_path_response.end_effector_path.poses
        ee_vels = plan_path_response.end_effector_velocity_trajectory
        jt = plan_path_response.joint_trajectory
        jt_points = jt.points if jt is not None else []

        n_path = len(path_poses)
        n_vel = len(ee_vels)
        n_jt = len(jt_points)

        # Determine joint names
        joint_names = list(jt.joint_names) if jt and jt.joint_names else []
        n_joints = len(joint_names)

        # Compute time base: prefer EE velocity trajectory stamps (closest to PF output),
        # then path pose stamps, then joint time_from_start, else synthesize index * dt
        times = []
        if n_vel > 0 and ee_vels[0].header.stamp.sec != 0:
            t0 = ee_vels[0].header.stamp.sec + \
                ee_vels[0].header.stamp.nanosec * 1e-9
            for v in ee_vels:
                ts = v.header.stamp.sec + v.header.stamp.nanosec * 1e-9
                times.append(ts - t0)
        else:
            # fallback: use index with assumed dt of 0.1s
            est_dt = 0.1
            maxlen = max(n_path, n_vel, n_jt, 1)
            times = [i * est_dt for i in range(maxlen)]

        # Determine number of rows (use max of available series lengths)
        n_rows = max(len(times), n_path, n_vel, n_jt)

        # Prepare rows
        rows = []
        last_pose = None
        last_vel = None
        last_joints = [math.nan] * n_joints if n_joints > 0 else []

        for i in range(n_rows):
            # time
            t = times[i] if i < len(times) else (
                times[-1] + (i - len(times) + 1) * 0.1)

            # pose
            if i < n_path:
                pose = path_poses[i].pose
                last_pose = pose
            elif last_pose is not None:
                pose = last_pose
            else:
                # default empty pose
                pose = PoseStamped().pose

            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            # velocity
            if i < n_vel:
                vel = ee_vels[i].twist.linear
                last_vel = vel
            elif last_vel is not None:
                vel = last_vel
            else:
                # zero velocity
                class _V:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                vel = _V()

            vx = vel.x
            vy = vel.y
            vz = vel.z

            # joints
            joints_row = []
            if i < n_jt:
                positions = jt_points[i].positions if jt_points[i].positions else [
                ]
                # pad/truncate to n_joints
                for j in range(n_joints):
                    if j < len(positions):
                        joints_row.append(positions[j])
                        last_joints[j] = positions[j]
                    else:
                        joints_row.append(last_joints[j] if not math.isnan(
                            last_joints[j]) else math.nan)
            else:
                # no new joint point, use last known or nan
                for j in range(n_joints):
                    joints_row.append(last_joints[j] if not math.isnan(
                        last_joints[j]) else math.nan)

            row = [t, px, py, pz, qx, qy, qz, qw, vx, vy, vz] + joints_row
            rows.append(row)

        # Write CSV
        filename = f'data/planned_path.csv'
        header = ['time_s', 'ee_px', 'ee_py', 'ee_pz', 'ee_qx',
                  'ee_qy', 'ee_qz', 'ee_qw', 'ee_vx', 'ee_vy', 'ee_vz']
        header += joint_names if joint_names else [
            f'joint_{i}' for i in range(len(rows[0]) - len(header))]

        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)
            for r in rows:
                # replace math.nan with empty string for readability
                out = [('' if (isinstance(x, float) and math.isnan(x)) else x)
                       for x in r]
                writer.writerow(out)

        abs_path = os.path.abspath(filename)
        self.get_logger().info(f'Saved planned path CSV to {abs_path}')

    def _on_plan_path_response(self, future):
        # Called when the async plan_path future is ready
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(
                f'Exception while waiting for plan_path result: {e}')
            return

        if res is None:
            self.get_logger().error('plan_path service returned an empty response (async)')
            return

        ee_path_len = len(res.end_effector_path.poses)
        ee_vels = len(res.end_effector_velocity_trajectory)
        self.get_logger().info(
            f"Received plan_path response: success={res.success}, Path Length={ee_path_len}")
        p = res.end_effector_path.poses[0].pose.position
        self.get_logger().info(
            f"First EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})")
        if ee_vels > 0:
            v = res.end_effector_velocity_trajectory[0].twist.linear
            self.get_logger().info(
                f"First EE linear velocity: ({v.x:.6f}, {v.y:.6f}, {v.z:.6f})")

        # Save CSV for offline plotting
        try:
            self.save_planned_path_response(res)
        except Exception as e:
            self.get_logger().error(
                f"Failed to save planned path response (async): {e}")


def main(args=None):
    """Entry to the node."""
    rclpy.init(args=args)
    node = PFDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
