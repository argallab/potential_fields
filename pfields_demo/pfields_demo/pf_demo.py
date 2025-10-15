#!/usr/bin/env python3
import csv
import math
import os

from geometry_msgs.msg import PoseStamped
from potential_fields_interfaces.srv import PlanPath
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTolerance
from control_msgs.action import FollowJointTrajectory
import tf2_ros


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
                    'Interrupted while waiting for the service. '
                    'Continuing without service (shutdown requested).'
                )
                break
            self.get_logger().info('Waiting for the plan_path service to be available...')
        self.get_logger().info('Plan path service is available.')

        # Service to trigger demo
        self.create_service(
            Empty, '/pfield_demo/run_plan_path_demo', self.run_demo_callback)
        self.get_logger().info('Ready to run plan path demo via service call.')

        # Action client for executing joint trajectories
        self.traj_action_client = ActionClient(
            self, FollowJointTrajectory, '/fer_arm_controller/follow_joint_trajectory')
        # Wait for action server until available
        if not self.traj_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for FollowJointTrajectory action server...')
        if self.traj_action_client.server_is_ready():
            self.get_logger().info('FollowJointTrajectory action server is available.')
        else:
            self.get_logger().warn('FollowJointTrajectory action server is not available.')

        # Initialize transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Buffers for action execution logging
        self._traj_feedback_log = []  # list of samples captured from feedback
        self._current_joint_names = []

    def run_demo_callback(self, request, response):
        self.get_logger().info('Running plan path demo...')
        req = PlanPath.Request()

        # Get the transform from world -> fer_hand_tcp and use that tf
        # pose as the start pose
        start = PoseStamped()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame, 'fer_hand_tcp', rclpy.time.Time()
            )
            start.header.frame_id = transform.header.frame_id
            start.header.stamp = self.get_clock().now().to_msg()
            start.pose.position.x = transform.transform.translation.x
            start.pose.position.y = transform.transform.translation.y
            start.pose.position.z = transform.transform.translation.z
            start.pose.orientation = transform.transform.rotation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(
                f"Failed to get fer_hand_tcp transform: {e}"
            )
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

    def send_joint_trajectory(self, joint_trajectory: JointTrajectory):
        # Send a JointTrajectory as a FollowJointTrajectory action goal and log feedback/result
        if not self.traj_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowJointTrajectory action server not available.')
            return

        goal_msg = self.create_action_request(joint_trajectory)

        # Prepare logging buffers
        self._traj_feedback_log = []
        self._current_joint_names = list(
            joint_trajectory.joint_names) if joint_trajectory.joint_names else []

        def _on_goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('FollowJointTrajectory goal was rejected by the action server.')
                return
            self.get_logger().info('FollowJointTrajectory goal accepted; waiting for result...')

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_traj_result)

        self.get_logger().info('Sending FollowJointTrajectory goal...')
        send_goal_future = self.traj_action_client.send_goal_async(
            goal_msg, feedback_callback=self._on_traj_feedback)
        send_goal_future.add_done_callback(_on_goal_response)

    def _on_traj_feedback(self, feedback_msg):
        # Feedback may be wrapped or bare; handle both
        feedback = feedback_msg.feedback if hasattr(
            feedback_msg, 'feedback') else feedback_msg
        try:
            t = 0.0
            if feedback.desired and feedback.desired.time_from_start:
                t = feedback.desired.time_from_start.sec + \
                    feedback.desired.time_from_start.nanosec * 1e-9
            # Compute a simple position error norm if available
            pos_err = None
            if feedback.error and feedback.error.positions:
                pos_err = math.sqrt(
                    sum([e * e for e in feedback.error.positions]))
            # Log desired vs actual first joint as a quick glance
            d0 = feedback.desired.positions[0] if (
                feedback.desired and feedback.desired.positions) else float('nan')
            a0 = feedback.actual.positions[0] if (
                feedback.actual and feedback.actual.positions) else float('nan')
            if pos_err is not None:
                self.get_logger().info(
                    f'[FJT feedback] t={t:.3f}s, first_joint d={d0:.4f} a={a0:.4f}, |pos_err|={pos_err:.4f}')
            else:
                self.get_logger().info(
                    f'[FJT feedback] t={t:.3f}s, first_joint d={d0:.4f} a={a0:.4f}')

            # Store sample for CSV export
            sample = {
                't': t,
                'd_pos': list(feedback.desired.positions) if feedback.desired and feedback.desired.positions else [],
                'a_pos': list(feedback.actual.positions) if feedback.actual and feedback.actual.positions else [],
                'e_pos': list(feedback.error.positions) if feedback.error and feedback.error.positions else [],
                'd_vel': list(feedback.desired.velocities) if feedback.desired and feedback.desired.velocities else [],
                'a_vel': list(feedback.actual.velocities) if feedback.actual and feedback.actual.velocities else [],
                'e_vel': list(feedback.error.velocities) if feedback.error and feedback.error.velocities else [],
            }
            self._traj_feedback_log.append(sample)
        except Exception as e:
            self.get_logger().warn(
                f'Failed to parse FollowJointTrajectory feedback: {e}')

    def _on_traj_result(self, fut):
        try:
            wrapped_result = fut.result()
            status = getattr(wrapped_result, 'status', None)
            result = getattr(wrapped_result, 'result', None)
            if result is not None:
                error_code = getattr(result, 'error_code', None)
                error_string = getattr(result, 'error_string', '')
                self.get_logger().info(
                    f'FollowJointTrajectory result: status={status}, error_code={error_code}, msg="{error_string}"')
                # Save execution log to CSV
                try:
                    self.save_executed_trajectory_response(
                        self._current_joint_names, self._traj_feedback_log, result)
                except Exception as ex:
                    self.get_logger().error(
                        f'Failed to save executed trajectory CSV: {ex}')
            else:
                self.get_logger().info(
                    f'FollowJointTrajectory finished: status={status}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to get FollowJointTrajectory result: {e}')

    def save_executed_trajectory_response(self, joint_names, feedback_samples, result, filename: str = 'data/executed_trajectory.csv'):
        """
        Save the executed trajectory feedback/time series to CSV.
        Columns:
          time_s, then for each joint: d_<name>, a_<name>, e_<name>, dvel_<name>, avel_<name>, evel_<name>

        joint_names: list of joint names
        feedback_samples: list of samples {t, d_pos, a_pos, e_pos, d_vel, a_vel, e_vel}
        result: FollowJointTrajectory result message (for logging)
        filename: output CSV path
        """
        # Determine joint count
        n_joints = len(joint_names)
        # Build header
        header = ['time_s']
        for name in joint_names:
            header += [f'd_{name}', f'a_{name}', f'e_{name}',
                       f'dvel_{name}', f'avel_{name}', f'evel_{name}']

        # Prepare rows
        rows = []
        last = {
            'd_pos': [math.nan] * n_joints,
            'a_pos': [math.nan] * n_joints,
            'e_pos': [math.nan] * n_joints,
            'd_vel': [math.nan] * n_joints,
            'a_vel': [math.nan] * n_joints,
            'e_vel': [math.nan] * n_joints,
        }
        for s in feedback_samples:
            t = s.get('t', 0.0)
            row = [t]
            # helper to pad/retain last

            def vec_or_last(key):
                v = s.get(key) or []
                out = []
                for j in range(n_joints):
                    if j < len(v):
                        last[key][j] = v[j]
                        out.append(v[j])
                    else:
                        out.append(last[key][j])
                return out
            d_pos = vec_or_last('d_pos')
            a_pos = vec_or_last('a_pos')
            e_pos = vec_or_last('e_pos')
            d_vel = vec_or_last('d_vel')
            a_vel = vec_or_last('a_vel')
            e_vel = vec_or_last('e_vel')
            # append in the header order per joint
            for j in range(n_joints):
                row += [d_pos[j], a_pos[j], e_pos[j],
                        d_vel[j], a_vel[j], e_vel[j]]
            rows.append(row)

        # Write CSV
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)
            for r in rows:
                # replace math.nan with empty string for readability
                out = [('' if (isinstance(x, float) and math.isnan(x)) else x)
                       for x in r]
                writer.writerow(out)

        abs_path = os.path.abspath(filename)
        err_code = getattr(result, 'error_code', None)
        err_str = getattr(result, 'error_string', '')
        self.get_logger().info(
            f'Saved executed trajectory CSV to {abs_path} (error_code={err_code}, msg="{err_str}")')

    def save_planned_path_response(self, plan_path_response):
        # Save a CSV file with the planned path details for offline plotting.
        # CSV columns:
        # time, ee_px, ee_py, ee_pz, ee_qx, ee_qy, ee_qz, ee_qw, ee_vx, ee_vy, ee_vz, joint1, ...

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
        filename = 'data/planned_path.csv'
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

    def create_action_request(self, joint_trajectory: JointTrajectory):
        # Create a FollowJointTrajectory action goal from a JointTrajectory message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory

        # Set default tolerances (can be customized as needed)
        goal_msg.path_tolerance = [
            JointTolerance(name=jn, position=0.05,
                           velocity=0.1, acceleration=0.1)
            for jn in joint_trajectory.joint_names
        ]
        goal_msg.goal_tolerance = [
            JointTolerance(name=jn, position=0.02,
                           velocity=0.05, acceleration=0.05)
            for jn in joint_trajectory.joint_names
        ]
        goal_msg.goal_time_tolerance = rclpy.duration.Duration(
            seconds=0.5).to_msg()

        return goal_msg

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

        # If a trajectory was returned, send it to the controller via action
        if res.success and res.joint_trajectory and res.joint_trajectory.points:
            self.get_logger().info('Sending returned JointTrajectory to action server...')
            self.send_joint_trajectory(res.joint_trajectory)
        else:
            self.get_logger().warn('No JointTrajectory points returned; skipping action execution.')


def main(args=None):
    """Entry to the node."""
    rclpy.init(args=args)
    node = PFDemo()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
