![Header](./images/github-header-banner.png)
### Authored by: [_Sharwin Patil_](https://github.com/Sharwin24) [(_LinkedIn_)](https://www.linkedin.com/in/sharwinpatil/) [(_Website_)](https://www.sharwinpatil.info/)

---
[![Build and Test Repository Packages](https://github.com/argallab/potential_fields/actions/workflows/main.yml/badge.svg)](https://github.com/argallab/potential_fields/actions/workflows/main.yml)

---

# Table Of Contents

- [Table Of Contents](#table-of-contents)
  - [Package Overview](#package-overview)
- [Getting Started](#getting-started)
  - [Install System Dependencies](#install-system-dependencies)
  - [Install Pinocchio (via robotpkg)](#install-pinocchio-via-robotpkg)
  - [Configure Environment Variables](#configure-environment-variables)
    - [Configuring Visual Studio Code Environment](#configuring-visual-studio-code-environment)
- [Building, Testing, and Launching](#building-testing-and-launching)
- [Running with Docker](#running-with-docker)
    - [Building the Image](#building-the-image)
    - [Running the Container](#running-the-container)
- [Usage \& Examples](#usage--examples)
    - [Launching the Demo](#launching-the-demo)
    - [Key Launch Arguments](#key-launch-arguments)
    - [Core Launch File](#core-launch-file)
  - [Supporting your own Robot](#supporting-your-own-robot)
    - [The Abstract Base Classes](#the-abstract-base-classes)
    - [How to Implement \& Integrate](#how-to-implement--integrate)
  - [Using C++ Library without ROS](#using-c-library-without-ros)
- [Potential Field Equations](#potential-field-equations)
- [Contributing](#contributing)

## Package Overview
The `potential_fields` repository provides an implementation of artifical potential field methods for path planning and shared control of robotic manipulators. The package is designed to be modular and extensible, allowing users to easily integrate it with different robot platforms and kinematics solvers. This package was developed with ROS 2 Jazzy in mind, but the core C++ library is ROS-agnostic and can be used independently. I plan to extend support to future ROS distributions as they are released.

This package is split into two parts:

1. Core C++ libraries (ROS-agnostic)
2. ROS integration (3 ROS packages for core node, msg/srv definitions, and demos)

<details>
<summary><b>Repository Folder Structure</b></summary>

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

# ROS 2 package integrating the core library with ROS interfaces
potential_fields
├── config # Config files (ROS Parameters, RViz config)
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

</details>


# Getting Started
Clone the repository with either HTTPS or SSH (depending on what you have setup for git):

```bash
# HTTPS Clone
git clone https://github.com/argallab/potential_fields.git

# SSH Clone
git clone git@github.com:argallab/potential_fields.git
```

Once this repository is cloned, you will need to install the required dependencies locally on your machine before being able to build and launch the `potential_fields` package.

## Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y libeigen3-dev liburdfdom-dev libfcl-dev libccd-dev libassimp-dev lsb-release
sudo apt-get install -y ros-${ROS_DISTRO}-urdf
```

## Install Pinocchio (via robotpkg)
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

## Configure Environment Variables

You need to add the `robotpkg` installation paths to your environment. Add the following  lines to the end of your `~/.bashrc` (or `~/.zshrc`):

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake:$CMAKE_PREFIX_PATH
```

### Configuring Visual Studio Code Environment
If you're working in VS Code like I do, it will be helpful to add external dependency include paths to the editor's include path to get access to all of the useful intellisense features. Below are my settings (_.vscode/c_cpp_properties.json_) for the workspace to allow VS Code to recognize ROS, Eigen, and Pinocchio types, syntax highlight, and support Ctrl+Click for definitions.

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

# Building, Testing, and Launching
To build the workspace, use `colcon` to build the project and run the unit tests:

```bash
colcon build
```

```bash
source install/setup.bash
```

```bash
colcon test && colcon test-result --verbose
```

Use `pf_demo.launch.xml` to launch the demo node that also launches the potential_fields package (`pfield.launch.xml`):

```bash
ros2 launch potential_fields_demos pf_demo.launch.xml
```

Launching the project without any arguments is good enough to launch a basic potential field and visualization. Of course, arguments are necessary to customize the PF package with your robot, RViz config, gain parameters, etc.

# Running with Docker
This repository provides two Dockerfiles in the `Docker/` directory:

1. **`Dockerfile` (Default)**: Contains the full environment including the XArm SDK, MoveIt, and other dependencies required to run the full XArm7 demo.
2. **`Dockerfile.minimal`**: A lightweight image containing only the essential dependencies (ROS 2 Jazzy, Pinocchio, Eigen, etc.) required to build and test the core library and basic demos.

### Building the Image

To build the **Full Demo Image** (Recommended for trying out the XArm demo):
```bash
# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build using the default Dockerfile
docker build -t potential_fields -f "$PROJECT_ROOT/Docker/Dockerfile" "$PROJECT_ROOT"
```

To build the **Minimal Image** (Faster build, good for CI or core development):
```bash
# Build using Dockerfile.minimal
docker build -t potential_fields_minimal -f "$PROJECT_ROOT/Docker/Dockerfile.minimal" "$PROJECT_ROOT"
```

### Running the Container

Before running the container, ensure that your X server allows connections from Docker containers so RViz can display properly.

```bash
# Allow local connections to X server (be careful with security on public networks)
xhost +local:docker > /dev/null 2>&1
```

Run the container (replace `potential_fields` with `potential_fields_minimal` if you built the minimal image so you can have both images on your system):

```bash
docker run -it --rm \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$PROJECT_ROOT:/home/workspace/src/potential_fields" \
  potential_fields
```

After building the image, you don't need to rebuild it unless you make changes to the code or dependencies. You can run the container multiple times using the same image:

```bash
# Start the container from the existing image
docker start -it potential_fields
```

# Usage & Examples
Now that you are setup with the package, know how to build and launch the package, and have a working Docker setup (if you choose to use Docker), you can explore the demos in the `potential_fields_demos` package.

### Launching the Demo
The primary entry point for the demos is `pf_demo.launch.xml`. This launch file starts the potential field manager, the visualization (RViz), and a demo node that interacts with the potential field.

To launch the XArm7 demo (The robot tested in the videos on my website):
```bash
ros2 launch potential_fields_demos pf_demo.launch.xml urdf_file_path:=xarm7.urdf use_rviz:=false motion_plugin_type:=xarm end_effector_frame:=link_eef
```

### Key Launch Arguments
You can customize the behavior by passing arguments to the launch command:

- **`urdf_file_path`**: Path to the robot's URDF file (relative to `potential_fields_demos/urdf` for the demo launch).
- **`motion_plugin_type`**: Specifies which robot plugin to load (e.g., `xarm`, `franka`, or leave empty for `null`).
- **`end_effector_frame`**: The name of the link to control (e.g., `link_eef`, `panda_hand`).
- **`rviz_fixed_frame`**: The fixed frame for visualization (default: `world`).
- **`use_rviz`**: Set to `false` to run without the GUI (if using a headless setup or with a MoveIt robot that launches RViz for you).

### Core Launch File
If you are integrating this into your own robot package, you will likely include the core launch file `pfield.launch.xml` from the `potential_fields` package directly.

```xml
<include file="$(find-pkg-share potential_fields)/launch/pfield.launch.xml">
  <arg name="urdf_file_path" value="$(var my_urdf_path)"/>
  <arg name="motion_plugin_type" value="my_robot_plugin"/>
  <!-- ... other arguments ... -->
</include>
```

## Supporting your own Robot
The library uses two Abstract Base Classes (ABCs) to decouple the core potential field logic from specific robot hardware and kinematics solvers.

### The Abstract Base Classes

1. **`pfield::IKSolver`** (`ik_solver.hpp`)
   - **Purpose:** Pure mathematical interface for Inverse Kinematics. It converts a desired Cartesian pose (or twist) into joint configurations (or velocities).
   - **Key Method:** `solve(...)` takes a target pose and seed configuration, returning the joint solution and Jacobian.
   - **When to implement:** If your robot has a custom analytical solver or you want to wrap a specific numerical solver (like KDL, Trac-IK, or a vendor API).

2. **`MotionPlugin`** (`motion_plugin.hpp`)
   - **Purpose:** The hardware abstraction layer. It handles communication with the robot controller.
   - **Key Methods:**
     - `readRobotState(...)`: Fetches current joint angles and EE pose.
     - `sendCartesianTwist(...)` / `sendJointStates(...)`: Sends commands to the robot.
     - `getIKSolver()`: Returns the specific `IKSolver` instance associated with this robot.
   - **When to implement:** For every new robot platform. This class usually instantiates and owns the `IKSolver`.

### How to Implement & Integrate

**Step 1: Create Files**
- Create `MyRobotIKSolver` inheriting from `pfield::IKSolver` in `potential_fields_library`.
- Create `MyRobotPlugin` inheriting from `MotionPlugin` in `potential_fields`. This class should instantiate `MyRobotIKSolver`.

**Step 2: Edit `pfield_manager.cpp`**
To make the manager aware of your new classes, you must manually register them in the source code:

1. **Include Header:** Add the include at the top of `potential_fields/src/ros/pfield_manager.cpp`.
   ```cpp
   #include "robot_plugins/my_robot_plugin.hpp"
   ```
2. **Instantiate:** Add a condition in the constructor (around line 150) to check for your plugin name.
   ```cpp
   else if (this->motionPluginType == "my_robot") {
     // Retrieve any specific parameters if needed
     this->motionPlugin = std::make_unique<MyRobotPlugin>();
   }
   ```


## Using C++ Library without ROS
There is a [README.md](potential_fields_library/README.md) in the `potential_fields_library` folder that contains information about the independent C++ library with regards to:
- Building library with colcon or cmake
- Creating an instance of `PotentialField` and using its functions
- How to pass URDF paths, mesh resource paths, and custom inverse kinematics functions into the library for use when planning.

# Potential Field Equations

For information about the mathematical formulation of potential fields used in this package, see [MATH.md](MATH.md). This README provides an overview of the package structure, instructions for building and launching the package, and details about usage.

# Contributing
See [CONTRIBUTING.md](CONTRIBUTING.md) for details on how to contribute to this repository. There's a bunch of [open issues](https://github.com/argallab/potential_fields/issues) that detail possible improvements, features, and bug fixes that you can help with! Of course, feel free to open new issues or reach out to me if you have any questions.

---

Copyright Notice [Sharwin Patil](https://www.sharwinpatil.info/) 2025

Contact me at:

```txt
sharwinpatil@u.northwestern.edu
```
