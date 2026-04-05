# `potential_fields_demos`

Example ROS 2 package demonstrating how to use the `potential_fields` package. Provides a launch file, demo nodes, bundled URDFs, and a configuration file to get started quickly without any hardware.

## Package Contents

```
potential_fields_demos/
├── launch/
│   └── pf_demo.launch.xml      # Primary entry-point launch file
├── src/
│   ├── pfield_demo.cpp         # Demo node: publishes obstacles, calls plan_path service
│   └── pfield_teleop_demo.cpp  # Teleop variant for interactive testing
├── urdf/
│   ├── xarm7.urdf              # UFactory XArm7 (primary reference robot)
│   ├── fer_franka_hand.urdf    # Franka Emika FER with hand
│   └── sphere_robot.urdf       # Minimal sphere robot for unit testing
└── config/
    └── demo.yaml               # ROS parameters for the demo node
```

## Quick Start

Build the workspace and source it, then launch the demo:

```bash
colcon build && source install/setup.bash

# Minimal demo (no robot URDF, no hardware)
ros2 launch potential_fields_demos pf_demo.launch.xml

# XArm7 demo (no hardware required)
ros2 launch potential_fields_demos pf_demo.launch.xml \
  urdf_file_path:=xarm7.urdf \
  motion_plugin_type:=xarm \
  end_effector_frame:=link_eef # TF frame of the end-effector link
```

Once running, trigger the plan path demo via service call:

```bash
ros2 service call /pfield_demo/run_plan_path_demo std_srvs/srv/Empty
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `urdf_file_path` | `""` | URDF filename relative to `urdf/` (e.g. `xarm7.urdf`) |
| `motion_plugin_type` | `""` | Robot plugin to load: `xarm`, `franka`, or empty for `null` |
| `end_effector_frame` | `link_eef` | Name of the end-effector link |
| `use_rviz` | `true` | Launch RViz alongside the demo |
| `rviz_fixed_frame` | `world` | RViz fixed frame |
| `base_frame` | `base` | Robot base link (child of fixed frame TF) |
| `base_x/y/z` | `0.0` | Translation of base frame relative to fixed frame |
| `base_roll/pitch/yaw` | `0.0` | Rotation of base frame relative to fixed frame |

## Demo Nodes

### `pfield_demo_node`
The primary demo node (`pfield_demo.cpp`). On startup it:
1. Publishes a set of static obstacles to `pfield/obstacles`.
2. Waits for the `pfield/plan_path` service to become available.
3. Exposes `/pfield_demo/run_plan_path_demo` (`std_srvs/Empty`) — call this service to trigger a planning request from the current robot pose to a pre-configured goal.

**Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fixed_frame` | `world` | World frame used for obstacle and goal poses |
| `ee_link_name` | `link_tcp` | End-effector link name passed to the planner |

### `pfield_teleop_demo_node`
Interactive variant (`pfield_teleop_demo.cpp`) for testing the autonomy vector in a shared-control loop. Calls `pfield/compute_autonomy_vector` at a configurable rate and forwards the result as a velocity command.

## Adding Your Own Robot

1. Place your robot's URDF in the `urdf/` directory.
2. Launch with `urdf_file_path:=<your_robot>.urdf` and the appropriate `motion_plugin_type` and `end_effector_frame`.
3. If your robot requires a custom plugin, see the [top-level README](../README.md#supporting-your-own-robot) for instructions on implementing `MotionPlugin` and `IKSolver`.

## Integrating into Your Own Package

To reuse the core potential field launch without the demo node, include `pfield.launch.xml` from the `potential_fields` package directly:

```xml
<include file="$(find-pkg-share potential_fields)/launch/pfield.launch.xml">
  <arg name="urdf_file_path" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
  <arg name="motion_plugin_type" value="my_robot"/>
  <arg name="end_effector_frame" value="my_end_effector_frame"/>
</include>
```

---

Copyright Notice [Sharwin Patil](https://www.sharwinpatil.info/) 2025
