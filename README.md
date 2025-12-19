![Header](./images/github-header-banner.png)
### Authored by: [_Sharwin Patil_](https://github.com/Sharwin24)

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

## Overvew of Package layout

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
The script below goes through the steps for creating a docker image and running it in a container.

```bash
#!/bin/bash

# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Build the image using the Dockerfile in Docker/ folder
# Context is the project root
docker build -t potential_fields -f "$PROJECT_ROOT/Docker/Dockerfile" "$PROJECT_ROOT"

# Allow local connections to X server (be careful with security on public networks)
xhost +local:docker > /dev/null 2>&1

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

# Using C++ Library without ROS
There is a [README.md](potential_fields_library/README.md) in the `potential_fields_library` folder that contains information about the independent C++ library with regards to:
- Building library with colcon or cmake
- Creating an instance of `PotentialField` and using its functions
- How to pass URDF paths, mesh resource paths, and custom inverse kinematics functions into the library for use when planning.

# Potential Field Equations

For information about the mathematical formulation of potential fields used in this package, see [MATH.md](MATH.md). This README provides an overview of the package structure, instructions for building and launching the package, and details about usage. 

---

Copyright Notice Sharwin Patil 2025
