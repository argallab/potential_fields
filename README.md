![Header](./images/github-header-banner.png)

---

# Potential Fields Motion Planning ROS2 Package
Author: [Sharwin Patil](https://www.sharwinpatil.info/), MSR 2025

[Project Journal](https://docs.google.com/document/d/1IJ0fK89f-ZZ52c72lqh1HTT0OAE9BSruDTp8FuKGbpw/edit?usp=sharing)

# ROS Package Structure and Overview

## Package layout (high level)

This package is split into two parts:

1. Core C++ libraries (ROS-agnostic)
2. ROS integration (one node)

```txt
potential_fields
├── config # ROS config files (ROS Parameters, RViz config)
├── include
│   └── potential_fields
│       ├── pfield # Core C++ PF library
│       ├── robot_plugins # Core C++ robot plugins and IK
│       └── ros # ROS node headers
├── launch
├── src
│   ├── pfield
│   ├── robot_plugins
│   └── ros
└── test
    └── resources
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

Launching the project without any arguments is good enough to launch the basic robot and visualization. Of course, arguments are necessary to customize the PF package with your robot, RViz config, gain parameters, etc.

# Potential Equations
These equations were obtained from the [Principles of Robot Motion: Theory, Algorithms, and Implementations](https://ieeexplore.ieee.org/book/6267238) textbook [2] and [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stampCaCancaCccCfaksdfojas;kfjd;ksldadjf.jsp?tp=&arnumber=1087247) [1], which describes the derivation of the potential equations, how to obtain the gradients, and how to obtain a velocity from the gradients.

In this library, the potential field produces a task-space wrench over the robot’s end-effector pose:

- Linear force $\mathbf{F}(q)$ [N] from the translational potential
- Torque $\boldsymbol{\tau}(q)$ [Nm] from the rotational potential

This wrench is mapped to a task-space twist $T(q)$ using constant gains:

$$\begin{align}
\mathbf{v}(q) &= k_{lin}\ \cdot \mathbf{F}(q) \\
\boldsymbol{\omega}(q) &= k_{ang} \cdot \boldsymbol{\tau}(q) \\
T(q) &= \begin{bmatrix}\mathbf{v}(q) \\ \boldsymbol{\omega}(q) \end{bmatrix}
\end{align}$$

with defaults of $k_{lin} = 1.0\,[(\mathrm{m/s})/\mathrm{N}]$ and $k_{ang} = 1.0\,[(\mathrm{rad/s})/\mathrm{Nm}]$.

The potential functions are scalar fields representing potential energy (Newton-meters [Nm]). The gradient of the potential is represented as a force (Newtons [N]). To obtain velocity vectors, we use some parameter ($\zeta$ and $\eta$) that acts as an inverse damping coefficient (Newton-seconds / meter [Ns/m]) to convert the force into a velocity (meters / second [m/s]).

## Attractive Potential
Attractive Potential is computed using a combined Conical and Quadratic potential function. This approach ensures a constant attractive force at large distances (Conical) to prevent high velocities, while switching to a quadratic behavior near the goal to ensure smooth convergence without chattering.
The transition occurs at a distance $\Gamma$, which can be dynamic based on the environment (e.g., clearance from obstacles) or a fixed parameter.

$$
U_{att}(q) = \begin{cases}
\frac{1}{2}\zeta D(q, q_{goal})^2 & D(q, q_{goal}) \le \Gamma \\
\Gamma \zeta D(q, q_{goal}) - \frac{1}{2}\zeta \Gamma^2 & D(q, q_{goal}) > \Gamma
\end{cases}
$$

The gradient (force) is:

$$
\mathbf{F}_{att}(q) = -\nabla U_{att}(q) = \begin{cases}
-\zeta (q - q_{goal}) & D(q, q_{goal}) \le \Gamma \\
-\frac{\Gamma \zeta}{D(q, q_{goal})} (q - q_{goal}) & D(q, q_{goal}) > \Gamma
\end{cases}
$$

Where:
- $\zeta$ is the *attractive gain* parameter
- $D(q, q_{goal})$ is the Euclidean distance between $q$ and $q_{goal}$
- $\Gamma$ is the quadratic threshold distance

## Rotational Attraction
Let $q_c$ be the current unit quaternion and $q_g$ the goal orientation. The geodesic distance $\theta \in [0,\pi]$ is the shortest rotation aligning $q_c$ to $q_g$. Define the quaternion difference

$$q_{diff} = q_c^{*} \otimes q_g,$$

and the corresponding unit rotation axis $\mathbf{u} = \frac{\vec{q_{diff}}}{\lVert\vec{q_{diff}}\rVert}$. The attractive rotational torque is proportional to the geodesic angle and acts about $\mathbf{u}$:

$$\boldsymbol{\tau}_{att}(q) = -\,\omega\,\theta\,\mathbf{u},$$

applied only when $\theta$ exceeds a small threshold. Here $\omega$ is the rotational attractive gain.

## Repulsive Potential
Repulsion increases with proximity to each obstacle and is summed across obstacles. We use the obstacle surface’s signed distance $d$ and outward normal $\mathbf{n}_{out}$ at the closest point. Outside the obstacle $d \ge 0$; inside $d < 0$ is treated as $d \approx \varepsilon$ to produce a strong outward push. The gradient of the potential yields the repulsive force

$$
\mathbf{F}_{rep}(q) = \begin{cases}
\eta\,\Big(\frac{1}{D(q)} - \frac{1}{Q}\Big)\,\frac{1}{D(q)^2}\,\mathbf{n}_{out} & 0 < D(q) < Q \\
\mathbf{0} & D(q) \ge Q
\end{cases}
$$

Where:
- $\eta$ is the repulsive gain
- $Q$ is the influence distance
- $D(q)$ is the signed distance magnitude to the obstacle surface
- $\mathbf{n}_{out}$ is the outward surface normal at the closest point

## Total Potential, Wrench, and Twist

$$
\begin{align}
U(q) &= U_{att}(q) + \sum_i U_{rep,i}(q) \\
\mathbf{F}(q) &= -\nabla U(q) \quad\text{(linear force)} \\
\boldsymbol{\tau}_{att}(q) &= -\,\omega\,\theta\,\mathbf{u} \quad\text{(rotational attraction)} \\
\mathbf{v}(q) &= k_{lin}\,\mathbf{F}(q) \\
\boldsymbol{\omega}(q) &= k_{ang}\,\boldsymbol{\tau}_{att}(q)
\end{align}
$$

Where:
- $q$ is the pose (position and orientation)
- $\mathbf{F}$ is the net linear force, the negative gradient of the scalar potential
- $\boldsymbol{\tau}_{att}$ is the attractive torque toward the goal orientation
- $\mathbf{v}$ and $\boldsymbol{\omega}$ are the linear and angular velocities after mapping with $(k_{lin}, k_{ang})$

### Integration and Motion Strategy
We conceptually utilize gradient descent to move down the potential’s gradient, but the implementation uses a velocity-constrained Runge–Kutta (RK4) step over the twist for better stability and constraint handling. In pseudocode, gradient descent looks like:

$$
\begin{align*}
&q[0] = q_{start} \\
&i = 0 \\
&\text{while }\Vert\nabla U(q_i)\Vert > \epsilon \\
& q_{i+1} = q_i - \alpha_i \nabla U(q_i) \\
& i = i + 1
\end{align*}
$$

Where:
- $\epsilon$ parameterizes when to stop
- $\alpha_i$ is a step-size (learning rate)

#### Runge-Kutta 4 (RK4) Integration
We integrate over the twist (linear and angular velocities) using a constrained RK4 step. Each stage evaluates a constrained twist at an intermediate pose and time step, then averages the four stages to advance the pose.

Given current pose $q_i = (\mathbf{x}_i, \mathbf{R}_i)$ and previous applied twist $T_{i-1}$, with step size $\Delta t$:

1) Stage 1

$$
\begin{aligned}
\mathbf{k}_1 &= T_c\big(q_i,\, T_{i-1},\, \Delta t\big) \\
q_{i}^{(1/2)} &= \Big(\, \mathbf{x}_i + \tfrac{\Delta t}{2}\,\mathbf{v}_1\,,\; \mathbf{R}_i\;\exp\big( [\boldsymbol{\omega}_1]_\times \tfrac{\Delta t}{2} \big) \Big)
\end{aligned}
$$

2) Stage 2

$$
\begin{aligned}
\mathbf{k}_2 &= T_c\big(q_{i}^{(1/2)},\, \mathbf{k}_1,\, \tfrac{\Delta t}{2}\big) \\
q_{i}^{(1/2)'} &= \Big(\, \mathbf{x}_i + \tfrac{\Delta t}{2}\,\mathbf{v}_2\,,\; \mathbf{R}_i\;\exp\big( [\boldsymbol{\omega}_2]_\times \tfrac{\Delta t}{2} \big) \Big)
\end{aligned}
$$

3) Stage 3

$$
\begin{aligned}
\mathbf{k}_3 &= T_c\big(q_{i}^{(1/2)'},\, \mathbf{k}_2,\, \tfrac{\Delta t}{2}\big) \\
q_{i}^{(1)} &= \Big(\, \mathbf{x}_i + \Delta t\,\mathbf{v}_3\,,\; \mathbf{R}_i\;\exp\big( [\boldsymbol{\omega}_3]_\times \Delta t \big) \Big)
\end{aligned}
$$

4) Stage 4

$$
\mathbf{k}_4 = T_c\big(q_{i}^{(1)},\, \mathbf{k}_3,\, \Delta t\big)
$$

Weighted average twist (component-wise for linear and angular parts):

$$
\bar{\mathbf{v}} = \frac{\mathbf{v}_1 + 2\mathbf{v}_2 + 2\mathbf{v}_3 + \mathbf{v}_4}{6},\quad
\bar{\boldsymbol{\omega}} = \frac{\boldsymbol{\omega}_1 + 2\boldsymbol{\omega}_2 + 2\boldsymbol{\omega}_3 + \boldsymbol{\omega}_4}{6}.
$$

The averaged twist is re-limited (soft saturation and rate limits) to ensure it respects the velocity/acceleration bounds. The pose is then advanced once:

$$
\mathbf{x}_{i+1} = \mathbf{x}_i + \bar{\mathbf{v}}\,\Delta t,\qquad
\mathbf{R}_{i+1} = \mathbf{R}_i\;\exp\big( [\bar{\boldsymbol{\omega}}]_\times\, \Delta t \big),
$$

where $\exp([\boldsymbol{\omega}]_\times\, \Delta t)$ is implemented via an angle–axis exponential map (i.e., $\mathrm{AngleAxis}(|\boldsymbol{\omega}|\,\Delta t,\, \boldsymbol{\omega}/|\boldsymbol{\omega}|)$) and the resulting quaternion is normalized.

Notes:
- $T_c(\cdot)$ applies planning-only opposing-force removal, maps wrench to twist, and enforces motion constraints for each stage.
- Per-stage rate limits use the stage’s effective step size ($\Delta t$ for stages 1 and 4, $\Delta t/2$ for stages 2–3) by supplying the previous stage’s twist to the constraint function.
- After averaging, constraints are applied again before the final integration step (as in the implementation).

### Velocity and Acceleration Limits
Before integrating the twist, linear and angular speeds are soft-saturated by norm and then rate-limited using maximum linear/angular accelerations over the step size $\Delta t$. The library enforces:

- Max linear speed $\lVert\mathbf{v}\rVert \leq v_{max}$
- Max angular speed $\lVert\boldsymbol{\omega}\rVert \leq \omega_{max}$
- Linear/Angular acceleration limits over $\Delta t$

This produces smooth, feasible motions while following the potential field.

#### Soft Saturation
Soft saturation smoothly caps a vector’s norm while preserving its direction. For a vector $\mathbf{v}$, maximum allowed norm $v_{max}$, and softness parameter $\beta$:

$$
\mathbf{v}_{sat} \;=\; \mathbf{v} \cdot \frac{v_{max}\,\tanh\!\left(\beta\,\lVert\mathbf{v}\rVert / v_{max}\right)}{\lVert\mathbf{v}\rVert}.
$$

- If $\lVert\mathbf{v}\rVert \ll v_{max}$, $\tanh(x) \approx x$ and the scaling is nearly linear (no abrupt clipping).
- As $\lVert\mathbf{v}\rVert \to \infty$, the saturated norm approaches $v_{max}$ asymptotically.
- Higher $\beta$ produces a steeper transition near $v_{max}$; lower $\beta$ makes it gentler.

This is applied to both linear and angular velocities before rate limiting.

#### Rate Limiting
Rate limiting bounds the change from the previous vector $\mathbf{v}_{prev}$ to the current target $\mathbf{v}_{curr}$ by a maximum step $\Delta_{max}$ (e.g., from acceleration limits over $\Delta t$):

$$
\mathbf{d} = \mathbf{v}_{curr} - \mathbf{v}_{prev},\quad
\mathbf{v}_{rl} =
\begin{cases}
\mathbf{v}_{curr}, & \lVert\mathbf{d}\rVert \le \Delta_{max} \\
\mathbf{v}_{prev} + \mathbf{d}\,\dfrac{\Delta_{max}}{\lVert\mathbf{d}\rVert}, & \lVert\mathbf{d}\rVert > \Delta_{max}
\end{cases}
$$

This guarantees the step size is never larger than $\Delta_{max}$ while preserving the intended direction of change. After rate limiting, soft saturation is re-applied to ensure the final vector also respects the velocity caps.

### Mitigating Local Minima (Planning-only)
To reduce sticking on broad obstacle faces, the planner removes only the repulsive component that directly opposes the attractive force and keeps tangential components that encourage sliding:

$$
\mathbf{F}_{rep}^{\text{filtered}} = \mathbf{F}_{rep} - \min\big(0,\, \mathbf{F}_{rep}\cdot \mathbf{u}_{att}\big)\,\mathbf{u}_{att},\quad \mathbf{u}_{att} = \frac{\mathbf{F}_{att}}{\lVert\mathbf{F}_{att}\rVert}.
$$

This filtering is applied only during planning; the unfiltered field is kept for visualization and testing semantics.

## Path Planning Algorithm
This library supports two distinct path planning methods, each with different trade-offs regarding computational complexity and collision avoidance coverage.

### Task-Space Wrench Path Planning
This method calculates forces and torques (wrench) directly at the end-effector in the task space (Cartesian space). The wrench is converted into a twist (velocity) and integrated to update the end-effector's pose. Inverse Kinematics (IK) is then used to solve for the joint angles that achieve this pose.

**Main Benefit:** Computationally efficient and provides precise control over the end-effector's trajectory. Best suited when the primary concern is the end-effector's path and the robot arm is relatively unobstructed.

**Key Evaluations:**
1.  **Task-Space Wrench:** Sum of attractive and repulsive forces/torques at the end-effector.

$$ \mathcal{W}_{task} = \begin{bmatrix} \mathbf{F}_{att} + \mathbf{F}_{rep} \\ \boldsymbol{\tau}_{att} \end{bmatrix} $$

2.  **Task-Space Twist:** Wrench converted to velocity using admittance gains ($k_{lin} = k_{ang} = 1.0$).

$$ \mathcal{V}_{task} = \begin{bmatrix} k_{lin} & \mathbf{0} \\ \mathbf{0} & k_{ang} \end{bmatrix} \mathcal{W}_{task} $$

3.  **Integration:** The twist is integrated (using RK4) to find the next end-effector pose $x_{next}$.

$$ x_{next} = \text{RK4}(x_{curr}, \mathcal{V}_{task}, \Delta t) $$

### Whole-Body Velocity Path Planning
This method calculates virtual forces acting on *every* link of the robot body, not just the end-effector. These forces are mapped to joint torques using the Jacobian transpose of each link. The total joint torques are then converted to joint velocities, which are integrated to update the robot's configuration directly.

**Main Benefit:** Provides whole-body collision avoidance. The robot "feels" obstacles along its entire arm and will naturally fold or move its elbows to avoid them while trying to reach the goal. Ideal for cluttered environments.

**Key Evaluations:**
1.  **Joint Torques:** Sum of end-effector attraction and whole-body repulsion mapped to joint space.

$$ \boldsymbol{\tau}_{joints} = J_{ee}^T(\mathbf{q}) \mathcal{W}_{att} + \sum_{i \in \text{links}} J_{i}^T(\mathbf{q}) \mathbf{F}_{rep, i} $$

2.  **Joint Velocities:** Torques converted to velocities using an admittance gain.

$$ \dot{\mathbf{q}} = k_{adm} \boldsymbol{\tau}_{joints} $$

_Soon to be replaced with the [Robot Dynamics Equation](https://github.com/argallab/pfields_2025/issues/23)_

3.  **Integration:** Joint velocities are integrated (Euler) to find the next joint configuration.

$$ \mathbf{q}_{next} = \mathbf{q}_{curr} + \dot{\mathbf{q}} \Delta t $$
