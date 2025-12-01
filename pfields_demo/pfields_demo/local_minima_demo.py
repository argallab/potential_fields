#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from potential_fields_interfaces.srv import PlanPath
from potential_fields_interfaces.msg import ObstacleArray, Obstacle
from std_srvs.srv import Empty


class LocalMinimaDemo(Node):
    def __init__(self):
        super().__init__('local_minima_demo')
        self.get_logger().info('LocalMinimaDemo Initialized')

        self.fixed_frame = self.declare_parameter(
            'fixed_frame', 'world').get_parameter_value().string_value

        self.goal_pub = self.create_publisher(
            PoseStamped, 'pfield/planning_goal_pose', 10)
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, 'pfield/obstacles', 10)
        self.query_pub = self.create_publisher(Pose, 'pfield/query_pose', 10)

        self.cli = self.create_client(PlanPath, 'pfield/plan_path')

        self.create_service(Empty, 'run_local_minima_demo',
                            self.run_demo_callback)

        # Setup Start and Goal for the classic local minima trap
        # Start at the open end of the U-shape
        self.start = (-4.0, 0.0, 0.0)
        # Goal behind the closed end of the U-shape
        self.goal = (2.0, 0.0, 0.0)

        # Create U-shaped obstacles
        self.obstacles = self.create_u_shape_obstacles()
        self.obstacle_pub.publish(self.obstacles)

        # Publish Goal
        self.publish_goal()

        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_path service...')

        # Publish Query Pose (Start)
        self.publish_start_pose()
        self.get_logger().info('Ready to run local minima demo via service call.')

    def create_u_shape_obstacles(self):
        msg = ObstacleArray()

        # 1. Back Wall (The bottom of the U)
        # Located at x=0, blocking the path to goal at x=2.0
        back_wall = Obstacle()
        back_wall.frame_id = "obstacle/u_shape/back"
        back_wall.type = "Box"
        back_wall.group = "Static"
        back_wall.pose.position.x = 0.0
        back_wall.pose.position.y = 0.0
        back_wall.pose.position.z = 0.0
        back_wall.pose.orientation.w = 1.0
        back_wall.length = 0.5  # Thickness in X
        back_wall.width = 3.0   # Width in Y
        back_wall.height = 1.0  # Height in Z
        msg.obstacles.append(back_wall)

        # 2. Side Wall 1 (Positive Y)
        side1 = Obstacle()
        side1.frame_id = "obstacle/u_shape/side1"
        side1.type = "Box"
        side1.group = "Static"
        # Extends from back wall towards negative X
        side1.pose.position.x = -1.5
        side1.pose.position.y = 1.5
        side1.pose.position.z = 0.0
        side1.pose.orientation.w = 1.0
        side1.length = 3.5  # Length in X
        side1.width = 0.5  # Thickness in Y
        side1.height = 1.0  # Height in Z
        msg.obstacles.append(side1)

        # 3. Side Wall 2 (Negative Y)
        side2 = Obstacle()
        side2.frame_id = "obstacle/u_shape/side2"
        side2.type = "Box"
        side2.group = "Static"
        side2.pose.position.x = -1.5
        side2.pose.position.y = -1.5
        side2.pose.position.z = 0.0
        side2.pose.orientation.w = 1.0
        side2.length = 3.5
        side2.width = 0.5
        side2.height = 1.0
        msg.obstacles.append(side2)

        self.get_logger().info(
            f'Created {len(msg.obstacles)} obstacles for U-shape trap.')
        return msg

    def publish_goal(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.fixed_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = self.goal[2]
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)

    def publish_start_pose(self):
        query_pose = Pose()
        query_pose.position.x = self.start[0]
        query_pose.position.y = self.start[1]
        query_pose.position.z = self.start[2]
        query_pose.orientation.w = 1.0
        self.query_pub.publish(query_pose)

    def run_demo_callback(self, request, response):
        self.get_logger().info('Running Local Minima Demo...')

        # Republish obstacles and goal just in case
        self.obstacle_pub.publish(self.obstacles)
        self.publish_goal()

        req = PlanPath.Request()

        start_pose = PoseStamped()
        start_pose.header.frame_id = self.fixed_frame
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = self.start[0]
        start_pose.pose.position.y = self.start[1]
        start_pose.pose.position.z = self.start[2]
        start_pose.pose.orientation.w = 1.0
        req.start = start_pose

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.fixed_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = self.goal[2]
        goal_pose.pose.orientation.w = 1.0
        req.goal = goal_pose

        req.planning_method = "task_space"
        req.delta_time = 0.01
        req.goal_tolerance = 0.1
        req.max_iterations = 10000

        self.get_logger().info('Sending plan_path request...')
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_plan_path_response)

        return response

    def _on_plan_path_response(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(
                    f'Path planning successful! Length: {len(res.end_effector_path.poses)}')
            else:
                self.get_logger().warn('Path planning failed (likely stuck in local minima).')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalMinimaDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
