#!/usr/bin/env python3
import csv
import math
import os
import random

from geometry_msgs.msg import PoseStamped, Pose
from potential_fields_interfaces.srv import PlanPath
from potential_fields_interfaces.msg import ObstacleArray, Obstacle
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
            PoseStamped, 'pfield/planning_goal_pose', 10
        )
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, 'pfield/obstacles', 10
        )
        self.query_pub = self.create_publisher(
            Pose, 'pfield/query_pose', 10
        )
        self.cli = self.create_client(PlanPath, 'pfield/plan_path')
        # Service to trigger demo
        self.create_service(
            Empty, 'run_sphere_demo', self.run_demo_callback
        )
        self.get_logger().info('Ready to run plan path demo via service call.')
        self.start = (-2.0, 0.0, 0.0)
        self.goal = (3.0,  0.0,  0.0)
        # Publish static obstacles and remember them for clearance computation
        # obstacles = self.create_obstacles_in_the_way()
        obstacles = self.create_clustered_obstacles()
        self.published_obstacles = obstacles  # cache for later CSV enrichment
        self.obstacle_pub.publish(obstacles)
        # Publish the goal pose (ensure a valid identity orientation)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.fixed_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = self.goal[2]
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_pose)
        self.get_logger().info('Published goal pose.')

        # Wait for the plan path service
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting again...')

        # Publish initial query pose at start
        query_pose = Pose()
        query_pose.position.x = self.start[0]
        query_pose.position.y = self.start[1]
        query_pose.position.z = self.start[2]
        query_pose.orientation.x = 0.0
        query_pose.orientation.y = 0.0
        query_pose.orientation.z = 0.0
        query_pose.orientation.w = 1.0
        self.query_pub.publish(query_pose)
        self.get_logger().info('Published initial query pose.')

    def create_clustered_obstacles(self) -> ObstacleArray:
        """Replicate the ClutteredSpheresDemo obstacle pattern as ROS obstacles.

        Places ~10 spheres with radii and positions sampled from ranges used in
        prototyping/pfield_3d_demo.py ClutteredSpheresDemo, with a light corridor
        clearing near y ~ 0.
        """
        rng = random.Random(7)  # match the prototype's default seed
        count = 10
        obstacles_msg = ObstacleArray()
        for i in range(count):
            cx = rng.uniform(-1.5, 2.5)
            cy = rng.uniform(-1.0, 1.0)
            cz = rng.uniform(-0.4, 0.4)
            r = rng.uniform(0.25, 0.55)
            # corridor: keep y near 0 a bit clearer most of the time
            if abs(cy) < 0.2 and rng.random() < 0.7:
                cy += rng.choice([-1.0, 1.0]) * rng.uniform(0.3, 0.6)

            sph = Obstacle()
            sph.frame_id = f"obstacle/sphere_cluster/{i}"
            sph.type = "Sphere"
            sph.group = "Static"
            sph.pose.position.x = cx
            sph.pose.position.y = cy
            sph.pose.position.z = cz
            sph.pose.orientation.x = 0.0
            sph.pose.orientation.y = 0.0
            sph.pose.orientation.z = 0.0
            sph.pose.orientation.w = 1.0
            sph.radius = r
            obstacles_msg.obstacles.append(sph)

        self.get_logger().info(
            f'Created clustered sphere obstacles: {len(obstacles_msg.obstacles)} items')
        return obstacles_msg

    def create_obstacles_in_the_way(self):
        obstacles_msg = ObstacleArray()
        # Replicate the obstacles from prototyping/pfield_3d_demo.py: obstacles_in_the_way()
        #   - Sphere at (1.5, 1.0, 0.7), radius 0.7
        #   - Box at (3.0, -0.5, 0.0), size (1.2, 2.0, 1.0), yaw=25°, pitch=10°, roll=-5° (ZYX)
        #   - Cylinder at (1.0, -1.0, 0.3), radius 0.45, height 1.2, yaw=15° (ZYX)

        def quat_from_euler_zyx(roll: float, pitch: float, yaw: float):
            """Return (x, y, z, w) quaternion for ZYX Euler (roll=X, pitch=Y, yaw=Z)."""
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            return qx, qy, qz, qw

        # Sphere obstacle
        sph = Obstacle()
        # frame_id acts as a unique obstacle identifier in the PF manager
        sph.frame_id = "obstacle/sphere/1"
        sph.type = "Sphere"
        sph.group = "Static"
        sph.pose.position.x = 1.5
        sph.pose.position.y = 1.0
        sph.pose.position.z = 0.7
        # Identity orientation
        sph.pose.orientation.x = 0.0
        sph.pose.orientation.y = 0.0
        sph.pose.orientation.z = 0.0
        sph.pose.orientation.w = 1.0
        sph.radius = 0.7
        obstacles_msg.obstacles.append(sph)

        # Box obstacle (OBB)
        box = Obstacle()
        # frame_id acts as a unique obstacle identifier in the PF manager
        box.frame_id = "obstacle/box/1"
        box.type = "Box"
        box.group = "Static"
        box.pose.position.x = 3.0
        box.pose.position.y = -0.5
        box.pose.position.z = 0.0
        # Apply ZYX euler: roll=-5°, pitch=10°, yaw=25°
        roll = math.radians(-5.0)
        pitch = math.radians(10.0)
        yaw = math.radians(25.0)
        qx, qy, qz, qw = quat_from_euler_zyx(roll, pitch, yaw)
        box.pose.orientation.x = qx
        box.pose.orientation.y = qy
        box.pose.orientation.z = qz
        box.pose.orientation.w = qw
        box.length = 1.2  # X size
        box.width = 2.0   # Y size
        box.height = 1.0  # Z size
        obstacles_msg.obstacles.append(box)

        # Cylinder obstacle
        cyl = Obstacle()
        # frame_id acts as a unique obstacle identifier in the PF manager
        cyl.frame_id = "obstacle/cylinder/1"
        cyl.type = "Cylinder"
        cyl.group = "Static"
        cyl.pose.position.x = 1.0
        cyl.pose.position.y = -1.0
        cyl.pose.position.z = 0.3
        # yaw=15°, pitch=0°, roll=0°
        roll_c = math.radians(0.0)
        pitch_c = math.radians(0.0)
        yaw_c = math.radians(15.0)
        qx, qy, qz, qw = quat_from_euler_zyx(roll_c, pitch_c, yaw_c)
        cyl.pose.orientation.x = qx
        cyl.pose.orientation.y = qy
        cyl.pose.orientation.z = qz
        cyl.pose.orientation.w = qw
        cyl.radius = 0.45
        cyl.height = 1.2
        obstacles_msg.obstacles.append(cyl)
        self.get_logger().info(
            f'Created {len(obstacles_msg.obstacles)} obstacles for the demo.')
        return obstacles_msg

    def run_demo_callback(self, request, response):
        self.get_logger().info('Running Sphere Robot Demo...')

        # Define start and goal poses
        start_pose = PoseStamped()
        start_pose.header.frame_id = self.fixed_frame
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = self.start[0]
        start_pose.pose.position.y = self.start[1]
        start_pose.pose.position.z = self.start[2]
        start_pose.pose.orientation.x = 0.0
        start_pose.pose.orientation.y = 0.0
        start_pose.pose.orientation.z = 0.0
        start_pose.pose.orientation.w = 1.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.fixed_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.goal[0]
        goal_pose.pose.position.y = self.goal[1]
        goal_pose.pose.position.z = self.goal[2]
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        req = PlanPath.Request()
        req.start = start_pose
        req.goal = goal_pose
        req.delta_time = 0.01  # 10ms time step
        req.goal_tolerance = 0.1  # 10cm tolerance
        req.max_iterations = 5000

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
        if ee_path_len > 0:
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

    def save_planned_path_response(self, plan_path_response):
        # Save a CSV file with the planned path details for offline plotting.
        # CSV columns:
        # time, ee_px, ee_py, ee_pz, ee_qx, ee_qy, ee_qz, ee_qw,
        # ee_vx, ee_vy, ee_vz, ee_wx, ee_wy, ee_wz, ee_w_mag, min_clearance_m, joint1, ...

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
            ee_vel_nsec = ee_vels[0].header.stamp.nanosec * 1e-9
            t0 = ee_vels[0].header.stamp.sec + ee_vel_nsec
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
        last_ang = None
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

            # linear velocity
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

            # angular velocity
            if i < n_vel:
                ang = ee_vels[i].twist.angular
                last_ang = ang
            elif last_ang is not None:
                ang = last_ang
            else:
                class _W:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                ang = _W()

            wx = ang.x
            wy = ang.y
            wz = ang.z
            wmag = math.sqrt(wx*wx + wy*wy + wz*wz)

            # minimum clearance to published obstacles (non-negative distance to surface)
            min_clearance = math.nan
            try:
                if hasattr(self, 'published_obstacles') and self.published_obstacles is not None:
                    min_clearance = float('inf')
                    for obs in self.published_obstacles.obstacles:
                        ox = obs.pose.position.x
                        oy = obs.pose.position.y
                        oz = obs.pose.position.z
                        if obs.type.lower() == 'sphere':
                            r = getattr(obs, 'radius', 0.0)
                            d = math.sqrt((px-ox)**2 + (py-oy)
                                          ** 2 + (pz-oz)**2) - r
                            min_clearance = min(min_clearance, max(d, 0.0))
                        elif obs.type.lower() == 'box':
                            # conservative: circumscribed sphere around OBB
                            hx = getattr(obs, 'length', 0.0) * 0.5
                            hy = getattr(obs, 'width', 0.0) * 0.5
                            hz = getattr(obs, 'height', 0.0) * 0.5
                            r = math.sqrt(hx*hx + hy*hy + hz*hz)
                            d = math.sqrt((px-ox)**2 + (py-oy)
                                          ** 2 + (pz-oz)**2) - r
                            min_clearance = min(min_clearance, max(d, 0.0))
                        elif obs.type.lower() == 'cylinder':
                            # conservative: circumscribed sphere around cylinder
                            cr = getattr(obs, 'radius', 0.0)
                            ch = getattr(obs, 'height', 0.0) * 0.5
                            r = math.sqrt(cr*cr + ch*ch)
                            d = math.sqrt((px-ox)**2 + (py-oy)
                                          ** 2 + (pz-oz)**2) - r
                            min_clearance = min(min_clearance, max(d, 0.0))
                        else:
                            # unknown type: skip
                            pass
                    if min_clearance == float('inf'):
                        min_clearance = math.nan
            except Exception as e:
                self.get_logger().warn(
                    f"Failed computing min_clearance at i={i}: {e}")

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

            row = [t, px, py, pz, qx, qy, qz, qw, vx, vy, vz,
                   wx, wy, wz, wmag, min_clearance] + joints_row
            rows.append(row)

        # Write CSV
        filename = 'data/planned_path.csv'
        header = ['time_s', 'ee_px', 'ee_py', 'ee_pz', 'ee_qx',
                  'ee_qy', 'ee_qz', 'ee_qw', 'ee_vx', 'ee_vy', 'ee_vz',
                  'ee_wx', 'ee_wy', 'ee_wz', 'ee_w_mag', 'min_clearance_m']
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


def main(args=None):
    """Entry to the node."""
    rclpy.init(args=args)
    node = SphereRobotDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
