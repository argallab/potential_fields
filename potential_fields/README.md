# `potential_fields` ROS Package Overview

This package hosts the `pfield_manager` node, which orchestrates the core library usage to interface with ROS:

- Maintains instances of `PotentialField`, `MotionPlugin`, and the plugin-provided `IKSolver`.
- Optionally initializes PF kinematics from a URDF path (parameter `urdf_file_path`) to estimate robot extent and populate obstacles.
- Publishes RViz markers (goal/query triads, obstacles, influence volumes, velocity vectors) and a planned EE path.
- Exposes a path-planning service and a convenience service for a local autonomy vector.

Topics and services

### Publishers

- `pfield/markers` (`visualization_msgs/MarkerArray`): goal, query pose, obstacles, influence zones, velocity vectors.
- `pfield/planned_path` (`nav_msgs/Path`): end-effector path from the last planning request.

### Subscribers
- `pfield/planning_goal_pose` (`geometry_msgs/PoseStamped`): sets the PF goal pose.
- `pfield/obstacles` (`potential_fields_interfaces/ObstacleArray`): external obstacles; any node may publish here.
- `pfield/query_pose` (`geometry_msgs/Pose`): live “query pose” to visualize field flow.
### Services
- `pfield/plan_path` (`potential_fields_interfaces/srv/PlanPath`): plan a path from a start pose to the current goal.
- `pfield/compute_autonomy_vector` (`potential_fields_interfaces/srv/ComputeAutonomyVector`): compute the limited twist at a pose.

Notes on robot geometry

- You can supply obstacles directly via `pfield/obstacles` from any node.
- Alternatively, provide a URDF path to the node via `urdf_file_path`; the PF library will initialize `PFKinematics`, estimate robot extent (for influence distance), and can update geometry-derived obstacles as planning progresses.