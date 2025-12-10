![Header](./images/github-header-banner.png)

---

For information about the mathematical formulation of potential fields used in this package, see [MATH.md](MATH.md). This README provides an overview of the package structure, instructions for building and launching the package, and details about usage.

# ROS Package Structure and Overview

## Package layout (high level)

This package is split into two parts:

1. Core C++ libraries (ROS-agnostic)
2. ROS integration (Single ROS 2 node and associated msg/srv definitions)

```bash
# Core C++ potential field library (ROS-agnostic)
pfield_library
├── cmake
├── include
│   ├── pfield
│   └── solvers
└── src
    ├── pfield
    └── solvers

 # ROS 2 package integrating the core library
potential_fields
├── config # ROS config files (ROS Parameters, RViz config)
├── include
│   └── potential_fields
│       ├── robot_plugins # Headers for custom robot plugins and IK Solvers
│       └── ros # ROS node headers
├── launch
├── src
│   ├── robot_plugins
│   └── ros
└── test
    └── resources

# ROS 2 message and service definitions
potential_fields_interfaces
├── include
│   └── potential_fields_interfaces
├── msg
├── src
└── srv

# Example ROS 2 Package demonstrating potential_fields usage
pfields_demo
├── config
├── include
│   └── pfields_demo
├── launch
├── meshes
├── pfields_demo
├── src
└── urdf
```

## Core C++ libraries (ROS-agnostic)

These headers and sources implement the planning and kinematics logic without depending on ROS:

- `pfield/` — Potential-field primitives and algorithms
    - Spatial math: `SpatialVector` (position + unit quaternion).
    - Field model: attractive/repulsive potentials, wrench -> twist mapping, soft saturation, rate limiting, RK4 integration.
    - Obstacles: spheres, boxes/OBB, cylinders, meshes; signed distance and outward normal queries.
    - Optional kinematics: `PFKinematics` can be initialized from a URDF file to derive robot extent and geometry-driven obstacles.

- `robot_plugins/` — Robot adapters and IK
    - Interfaces: `MotionPlugin` (runtime robot integration), `IKSolver` (task-space IK + Jacobian).
    - Implementations: `NullMotionPlugin` (no hardware), `FrankaPlugin` (example hardware). Plugins provide the `IKSolver` to the PF.

The intent is that this layer stays reusable and testable outside of ROS; the ROS node simply wires it up to topics/services.

## ROS integration: PotentialFieldManager node

`pfield_manager` is the single ROS 2 node that owns and orchestrates the core library:

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

_NOTE: Geometry-derived obstacles from the robot's links are in effort to avoid self-collisions. This will likely be reworked by planning in the configuration space instead of task space._

## Setting up the workspace
Once this repository is cloned, you will need to install the `Eigen` dependency locally on your machine before being able to build and launch the `potential_fields` package:

```bash
sudo apt install libeigen3-dev
```

Ensure that `/usr/include/eigen3` exists and add it to your C++ configurations, below are my settings (_.vscode/c_cpp_properties.json_) for the workspace to allow VS Code to recognize ROS and Eigen types, syntax highlight, and support Ctrl+Click for definitions.

```json
{
"configurations": [
    {
        "name": "Linux",
        "includePath": [
            "${workspaceFolder}/**",
            "/opt/ros/jazzy/include/**",
            "/usr/include/eigen3/**"
        ],
        "defines": [],
        "compilerPath": "/usr/bin/clang",
        "cStandard": "c17",
        "cppStandard": "c++20",
        "intelliSenseMode": "linux-clang-x64",
        "configurationProvider": "ms-vscode.cpptools"
    }
],
"version": 4
}
```
## Building, Testing, and Launching
To build the workspace, use `colcon` to build the project and run the unit tests:

```bash
colcon build
source install/setup.bash
colcon test && colcon test-result --verbose
```

Use `pf_demo.launch.xml` to launch the demo node that also launches the potential_fields package (`pfield.launch.xml`):

```bash
ros2 launch pfields_demo pf_demo.launch.xml
```

Launching the project without any arguments is good enough to launch a basic potential field and visualization. Of course, arguments are necessary to customize the PF package with your robot, RViz config, gain parameters, etc.
