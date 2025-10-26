#!/usr/bin/env python3
import csv
import math
import os

from geometry_msgs.msg import PoseStamped
from potential_fields_interfaces.srv import PlanPath
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class SphereRobotDemo(Node):
    def __init__(self):
        super().__init__('sphere_robot_demo')
        self.get_logger().info('SphereRobotDemo Initialized')
        self.fixed_frame = self.declare_parameter(
            'fixed_frame', 'world').get_parameter_value().string_value

        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.cli = self.create_client(PlanPath, 'pfield/plan_path')
        # Service to trigger demo
        self.create_service(
            Empty, '/run_sphere_demo', self.run_demo_callback
        )
        self.get_logger().info('Ready to run plan path demo via service call.')

        # Wait for the plan path service
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting again...')

    def run_demo_callback(self, request, response):
        self.get_logger().info('Running Sphere Robot Demo...')

        # Define start and goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.fixed_frame
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.5
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.fixed_frame
        goal_pose.pose.position.x = 3.0
        goal_pose.pose.position.y = 3.0
        goal_pose.pose.position.z = 0.5
        goal_pose.pose.orientation.w = 1.0

        # Publish the goal pose
        self.goal_pub.publish(goal_pose)
        self.get_logger().info('Published goal pose.')

        req = PlanPath.Request()
        req.start = start_pose
        req.goal = goal_pose

        # call service asynchronously; handle response in done-callback
        self.get_logger().info('Sending plan_path request (async)...')
        future = self.cli.call_async(req)
        # attach a done callback to process the result when ready
        future.add_done_callback(self._on_plan_path_response)
        # return immediately from the demo service; the response will be processed asynchronously
        return response

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
            f'Received plan_path response: success={res.success}, Path Length={ee_path_len}')
        if not res.success:
            self.get_logger().warn('Plan path was not successful; skipping further processing.')
            return
        p = res.end_effector_path.poses[0].pose.position
        self.get_logger().info(
            f'First EE pose: ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
        if ee_vels > 0:
            v = res.end_effector_velocity_trajectory[0].twist.linear
            self.get_logger().info(
                f'First EE linear velocity: ({v.x:.6f}, {v.y:.6f}, {v.z:.6f})')

        # Save CSV for offline plotting
        try:
            self.save_planned_path_response(res)
        except Exception as e:
            self.get_logger().error(
                f'Failed to save planned path response (async): {e}')


def main(args=None):
    """Entry to the node."""
    rclpy.init(args=args)
    node = SphereRobotDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
