![Header](./images/github-header-banner.png)

---

# Table Of Contents

- [Table Of Contents](#table-of-contents)
- [ROS Package Structure and Overview](#ros-package-structure-and-overview)
  - [Package layout (high level)](#package-layout-high-level)
  - [Core C++ libraries (ROS-agnostic)](#core-c-libraries-ros-agnostic)
  - [ROS integration: PotentialFieldManager node](#ros-integration-potentialfieldmanager-node)
    - [Publishers](#publishers)
    - [Subscribers](#subscribers)
    - [Services](#services)
  - [Setting up the workspace](#setting-up-the-workspace)
    - [Install System Dependencies](#install-system-dependencies)
    - [Install Pinocchio (via robotpkg)](#install-pinocchio-via-robotpkg)
    - [Configure Environment Variables](#configure-environment-variables)
  - [Building, Testing, and Launching](#building-testing-and-launching)

# ROS Package Structure and Overview

## Package layout (high level)

This package is split into two parts:

1. Core C++ libraries (ROS-agnostic)
2. ROS integration (Single ROS 2 node and associated msg/srv definitions)

```bash
# Core C++ potential field library (ROS-agnostic)
potential_fields_library
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
potential_fields_demos
├── config
├── include
│   └── potential_fields_demos
├── launch
├── meshes
├── potential_fields_demos
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

## Setting up the workspace
Once this repository is cloned, you will need to install the required dependencies locally on your machine before being able to build and launch the `potential_fields` package.

### Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y libeigen3-dev liburdfdom-dev libfcl-dev libccd-dev libassimp-dev lsb-release
sudo apt-get install -y ros-${ROS_DISTRO}-urdf
```

### Install Pinocchio (via robotpkg)
Below, I've outlined the steps to install Pinocchio via `robotpkg`. The original documentation can be found on the [pinocchio](https://github.com/stack-of-tasks/pinocchio) repository.

This package uses Pinocchio for rigid body dynamics. It is recommended to install it via `robotpkg`.

```bash
# Add robotpkg to apt sources
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list

# Install pinocchio
sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio
```

### Configure Environment Variables

You need to add the `robotpkg` installation paths to your environment. Add the following to your `~/.bashrc` (or `~/.zshrc`):

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake:$CMAKE_PREFIX_PATH
```

Ensure that `/usr/include/eigen3` and `/opt/openrobots/include` exist and add them to your C++ configurations. Below are my settings (_.vscode/c_cpp_properties.json_) for the workspace to allow VS Code to recognize ROS, Eigen, and Pinocchio types, syntax highlight, and support Ctrl+Click for definitions.

```json
{
"configurations": [
    {
        "name": "Linux",
        "includePath": [
            "${workspaceFolder}/**",
            "/opt/ros/jazzy/include/**",
            "/usr/include/eigen3/**",
            "/opt/openrobots/include/**"
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
ros2 launch potential_fields_demos pf_demo.launch.xml
```

Launching the project without any arguments is good enough to launch a basic potential field and visualization. Of course, arguments are necessary to customize the PF package with your robot, RViz config, gain parameters, etc.

## Running with Docker
The script below showcases the 3 steps to creating a docker image and running it in a container.

```bash
#!/bin/bash

# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================================"
echo "1. Building Docker Image 'potential_fields'"
echo "================================================================"
# Build the image using the Dockerfile in Docker/ folder
# Context is the project root
docker build -t potential_fields -f "$PROJECT_ROOT/Docker/Dockerfile" "$PROJECT_ROOT"

echo ""
echo "================================================================"
echo "2. Preparing to Run Container"
echo "================================================================"
echo " - Mounting workspace: $PROJECT_ROOT -> /home/workspace/src/potential_fields"
echo " - Enabling X11 forwarding (for GUI apps like Rviz)"

# Allow local connections to X server (be careful with security on public networks)
xhost +local:docker > /dev/null 2>&1

echo ""
echo "================================================================"
echo "3. Starting Container"
echo "================================================================"
echo "NOTE: The container will automatically build 'potential_fields_library' on startup."
echo "      Wait for 'potential_fields_library successfully installed' before typing."
echo ""

docker run -it --rm \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PROJECT_ROOT:/home/workspace/src/potential_fields" \
    potential_fields

# Example usage inside container:
# $ colcon build --packages-select potential_fields potential_fields_demos potential_fields_library potential_fields_interfaces
# $ source install/setup.bash
# $ ros2 launch potential_fields ...
```

---

For information about the mathematical formulation of potential fields used in this package, see [MATH.md](MATH.md). This README provides an overview of the package structure, instructions for building and launching the package, and details about usage. There is a [README.md](potential_fields_library/README.md) in the `potential_fields_library` folder that contains information about the independent C++ library.
