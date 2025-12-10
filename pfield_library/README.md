# pfield_library

The `pfield_library` is a core, ROS-agnostic C++ library that implements potential field-based motion planning algorithms. It provides the mathematical primitives, field models, and integration schemes necessary for generating smooth, collision-free trajectories for robotic manipulators and mobile robots.

## Overview and Features

This library is designed to be modular and reusable, separating the core planning logic from any specific middleware (like ROS). Key features include:

*   **Potential Field Primitives:** Implementation of attractive (conical/quadratic) and repulsive potentials.
*   **Spatial Math:** `SpatialVector` class for handling position and orientation (unit quaternions).
*   **Obstacle Representation:** Support for various obstacle geometries including spheres, boxes (OBB), cylinders, and meshes, with efficient signed distance and normal queries.
*   **Trajectory Generation:**
    *   **Task-Space Wrench:** Computes forces and torques at the end-effector.
    *   **Wrench to Twist Mapping:** Converts forces to velocities using admittance gains.
    *   **Motion Constraints:** Soft saturation and rate limiting to respect velocity and acceleration bounds.
    *   **Integration:** Runge-Kutta 4 (RK4) integration for stable and accurate trajectory propagation.
*   **Kinematics Support:** `PFKinematics` class (using KDL/URDF) for forward kinematics, Jacobian computation, and robot extent estimation.
*   **Inverse Kinematics Interfaces:** Abstract `IKSolver` interface for integrating custom IK solvers.

## Dependencies and Build Instructions

### Dependencies

*   **Eigen3:** Required for linear algebra operations.
*   **libcoal (Collision Obstacle Abstraction Library):** Used for underlying collision checking and distance computations.
*   **KDL (Kinematics and Dynamics Library):** Used for kinematic chain parsing and calculations.
*   **URDF:** For parsing robot descriptions.

To install Eigen3 (if not already present):
```bash
sudo apt install libeigen3-dev
```

### Building with Colcon (Recommended)

This library is structured as a colcon package. To build it within a workspace:

```bash
cd <workspace_root>
colcon build --packages-select pfield_library
```

### Building with CMake (Standalone)

You can also build the library using standard CMake commands:

```bash
mkdir build && cd build
cmake ..
make
sudo make install # Optional
```

## Usage

To use `pfield_library` in your C++ project, link against it in your `CMakeLists.txt`:

```cmake
find_package(pfield_library REQUIRED)

add_executable(my_planner main.cpp)
target_link_libraries(my_planner PRIVATE pfield_library::pfield_library)
```

### Basic Example

```cpp
#include <pfield/pfield.hpp>

int main() {
    // Initialize Potential Field
    pfield::PotentialField pf;

    // Set Goal
    pfield::SpatialVector goal(Eigen::Vector3d(1.0, 0.0, 0.5), Eigen::Quaterniond::Identity());
    pf.setGoalPose(goal);

    // Add Obstacle
    auto obstacle = pfield::PotentialFieldObstacle::createSphere("obs1", Eigen::Vector3d(0.5, 0.0, 0.5), 0.2);
    pf.addObstacle(obstacle);

    // Query Field at a point
    pfield::SpatialVector currentPose(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    auto wrench = pf.evaluateWrenchAtPose(currentPose);

    // ... Integrate to find path
    return 0;
}
```

## Important Remarks

*   **Coordinate Frames:** Ensure all poses (start, goal, obstacles) are defined in the same reference frame (typically "world" or "base_link") before passing them to the library.
*   **Parameter Tuning:** The behavior of the potential field is highly dependent on gains (`attractiveGain`, `repulsiveGain`) and the influence distance (`influenceDistance`). These may need tuning for specific robot configurations and environments.
*   **Local Minima:** Like all potential field methods, this approach is susceptible to local minima. The library includes a "planning-only" opposing force removal technique to help mitigate this, but it is not a global planner.
