# `potential_fields_interfaces`

ROS 2 message and service definitions for the `potential_fields` package. All nodes in this repository communicate using these types.

## Messages

### `Obstacle.msg`
Describes a single obstacle in the potential field environment.

| Field | Type | Description |
|-------|------|-------------|
| `frame_id` | `string` | Unique identifier for this obstacle (e.g. `"wall_1"`, `"link2::col0"`) |
| `type` | `string` | Shape type: `"Sphere"`, `"Box"`, `"Cylinder"`, or `"Mesh"` |
| `group` | `string` | Obstacle class: `"Static"`, `"Dynamic"`, or `"Robot"` |
| `pose` | `geometry_msgs/Pose` | World-frame center position and orientation |
| `radius` | `float32` | Radius — used by `Sphere` and `Cylinder` |
| `length` | `float32` | Length along X — used by `Box` |
| `width` | `float32` | Width along Y — used by `Box` |
| `height` | `float32` | Height along Z — used by `Box` and `Cylinder` |
| `mesh_resource` | `string` | URI to mesh file — used by `Mesh` (e.g. `package://...`) |
| `scale_x/y/z` | `float32` | Per-axis mesh scale — used by `Mesh` |

### `ObstacleArray.msg`
A stamped array of `Obstacle` messages, published to `pfield/obstacles`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp and frame |
| `obstacles` | `Obstacle[]` | List of obstacles |

**CLI example — publish a cylinder and a box**

```bash
ros2 topic pub --once /pfield/obstacles potential_fields_interfaces/msg/ObstacleArray "{
  header: {frame_id: 'world'},
  obstacles: [
    {
      frame_id: 'pitcher',
      type: 'Cylinder',
      group: 'Static',
      pose: {
        position: {x: 0.51, y: 0.0, z: 0.15},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      radius: 0.09,
      height: 0.3
    },
    {
      frame_id: 'table',
      type: 'Box',
      group: 'Static',
      pose: {
        position: {x: 0.66, y: 0.0, z: 0.3},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      length: 0.15,
      width: 0.7,
      height: 0.6
    }
  ]
}"
```

## Services

### `PlanPath.srv`
Plans a collision-free path from a start pose to a goal pose using the potential field.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `start` | `geometry_msgs/Pose` | Start end-effector pose (task-space planning only) |
| `goal` | `geometry_msgs/Pose` | Goal end-effector pose |
| `starting_joint_angles` | `float32[]` | Initial joint configuration in radians (whole-body planning) |
| `delta_time` | `float32` | Integration timestep in seconds |
| `goal_tolerance` | `float32` | Distance threshold for reaching the goal in meters |
| `max_iterations` | `int32` | Maximum integration steps (default: 30000) |
| `planning_method` | `string` | `"task_space"` or `"whole_body"` (defaults to `"task_space"`) |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether planning reached the goal within tolerance |
| `end_effector_path` | `nav_msgs/Path` | Planned end-effector trajectory |
| `joint_trajectory` | `trajectory_msgs/JointTrajectory` | Joint-space trajectory (whole-body only) |
| `end_effector_velocity_trajectory` | `geometry_msgs/TwistStamped[]` | EE velocity at each planning step |
| `error_message` | `string` | Populated on failure |

**CLI example — task-space planning**

Plan from a current pose to a goal 50 cm in front of the robot:

```bash
ros2 service call /pfield/plan_path potential_fields_interfaces/srv/PlanPath "{
  start: {
    position: {x: 0.227, y: 0.0, z: 0.294},
    orientation: {x: 0.707, y: 0.0, z: 0.707, w: 0.0}
  },
  goal: {
    position: {x: 0.5, y: 0.2, z: 0.4},
    orientation: {x: 0.707, y: 0.0, z: 0.707, w: 0.0}
  },
  delta_time: 0.02,
  goal_tolerance: 0.01,
  max_iterations: 25000,
  planning_method: 'task_space'
}"
```

**CLI example — whole-body planning**

Plan using joint angles as the initial configuration (start pose is ignored):

```bash
ros2 service call /pfield/plan_path potential_fields_interfaces/srv/PlanPath "{
  goal: {
    position: {x: 0.5, y: 0.2, z: 0.4},
    orientation: {x: 0.707, y: 0.0, z: 0.707, w: 0.0}
  },
  starting_joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, -1.5708, 0.0],
  delta_time: 0.02,
  goal_tolerance: 0.01,
  max_iterations: 25000,
  planning_method: 'whole_body'
}"
```

### `ComputeAutonomyVector.srv`
Returns the potential field's recommended velocity at a single query pose, used for shared-control applications.

**Request**

| Field | Type | Description |
|-------|------|-------------|
| `query_pose` | `geometry_msgs/PoseStamped` | Pose at which to evaluate the field |
| `joint_angles` | `float32[]` | Current joint configuration in radians |
| `prev_joint_velocities` | `float32[]` | Previous joint velocities (optional, for dynamic terms) |
| `delta_time` | `float32` | Timestep for dynamic calculations |
| `planning_method` | `string` | `"task_space"` or `"whole_body"` |

**Response**

| Field | Type | Description |
|-------|------|-------------|
| `autonomy_vector` | `geometry_msgs/TwistStamped` | Recommended velocity at the query pose |

**CLI example — task-space autonomy vector**

```bash
ros2 service call /pfield/compute_autonomy_vector potential_fields_interfaces/srv/ComputeAutonomyVector "{
  query_pose: {
    header: {frame_id: 'world'},
    pose: {
      position: {x: 0.3, y: 0.1, z: 0.35},
      orientation: {x: 0.707, y: 0.0, z: 0.707, w: 0.0}
    }
  },
  delta_time: 0.02,
  planning_method: 'task_space'
}"
```

**CLI example — whole-body autonomy vector**

```bash
ros2 service call /pfield/compute_autonomy_vector potential_fields_interfaces/srv/ComputeAutonomyVector "{
  query_pose: {
    header: {frame_id: 'world'},
    pose: {
      position: {x: 0.3, y: 0.1, z: 0.35},
      orientation: {x: 0.707, y: 0.0, z: 0.707, w: 0.0}
    }
  },
  joint_angles: [0.0, 0.0, 0.0, 0.0, 0.0, -1.5708, 0.0],
  prev_joint_velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  delta_time: 0.02,
  planning_method: 'whole_body'
}"
```

## Dependencies

- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `trajectory_msgs`
- `builtin_interfaces`

---

Copyright Notice [Sharwin Patil](https://www.sharwinpatil.info/) 2025
