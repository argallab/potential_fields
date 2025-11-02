# pfields_2025
MSR project to revamp current pfields repo for open sourcing

[Project Journal](https://docs.google.com/document/d/1IJ0fK89f-ZZ52c72lqh1HTT0OAE9BSruDTp8FuKGbpw/edit?usp=sharing)

# ROS Package Structure and Overview

## Potential Field C++ Library
The `pfield` library is a ROS-independent C++ API that the ROS package maintains an instance of. Vectors and Quaternions are implemented with [Eigen](https://libeigen.gitlab.io/docs/) and are wrapped into a singular object named `SpatialVector`. The Vector portion defines the translational velocity [m/s] towards the goal and the Quaternion represents the unit quaternion difference between the orientation and the goal orientation. Currently the C++ library is embedded in the include and src directories of the ROS package but will be extracted and presented as a standalone C++ library that the ROS package will need to link with.

## PotentialFieldManager (ROS Node)
Manages the `pfield` instance and visualizes the obstacles, the goal pose, and the "planned" path. Subcribes to `/goal_pose` to get the updated goal pose and updates the internal pfield instance's goal pose. Also subscribes to `/pfield/obstacles` to obtain the obstacles for the potential field. The obstacles can be published from any node and the robot parser node (see below) handles publishing the robot's geometry as pfield obstacles. The manager also handles user arguments to setup RViz and tune the attraction/repulsive gains.

## Robot Parser (ROS Node)
Subscribes to the robot description and publishes the `pfield` obstacles consistent with the URDF available. Requires that the user publishes a transform from a fixed frame (provided as a launch argument) to the base link of the robot. Publishes the collision objects from the URDF as pfield obstacles to the `/pfield/obstacles` topic to send updated obstacles.

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
These equations were obtained from this paper: [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247) [1] which describes the derivation of the potential equations, how to obtain the gradients, and how to obtain a velocity from the gradients.

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
Attractive Potential is computed using a continuously differentiable function (quadratic function of distance)

$$
\begin{align}
U_{att}(q) &= \frac{1}{2}\zeta D\left(q, q_{goal}\right)^2 \\
\nabla U_{att}(q) &= \nabla \left(\frac{1}{2}\zeta D\left(q, q_{goal}\right)^2\right) \\
&= \frac{1}{2}\zeta \nabla D\left(q, q_{goal}\right)^2 \\
\nabla U_{att}(q) &= \zeta \underbrace{\left(q - q_{goal}\right)}_{\text{vector distance}}
\end{align}
$$

Where:
- $\zeta$ is the *attractive gain* parameter
- $D\left(q, q_{goal}\right)$ is the euclidean distance between vector $q$ and $q_{goal}$

## Rotational Attraction
Let $q_c$ be the current unit quaternion and $q_g$ the goal orientation. The geodesic distance $\theta \in [0,\pi]$ is the shortest rotation aligning $q_c$ to $q_g$. Define the quaternion difference

$$q_{diff} = q_c^{*} \otimes q_g,$$

and the corresponding unit rotation axis $\mathbf{u} = \frac{\vec{q_{diff}}}{\lVert\vec{q_{diff}}\rVert}$. The attractive rotational torque is proportional to the geodesic angle and acts about $\mathbf{u}$:

$$\boldsymbol{\tau}_{att}(q) = -\,\omega\,\theta\,\mathbf{u},$$

applied only when $\theta$ exceeds a small threshold. Here $\omega$ is the rotational attractive gain.

## Repulsive Potential
Repulsion increases with proximity to each obstacle and sums across obstacles. We use the obstacle surface’s signed distance $d$ and outward normal $\mathbf{n}_{out}$ at the closest point. Outside the obstacle $d \ge 0$; inside $d < 0$ is treated as $d \approx \varepsilon$ to produce a strong outward push.

The classic Khatib potential [1] yields the repulsive force

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
The path planning algorithm iteratively computes the potential field wrench, maps it to a twist, applies velocity and acceleration limits, and integrates the twist to update the pose. This process continues until the planner reaches the goal within a specified tolerance.

```python
# pseudocode for path planning algorithm
def plan_path(start_pose, goal_pose, goal_tolerance, dt):
    current_pose = start_pose
    path = [current_pose]

    while not at_goal(current_pose, goal_pose, goal_tolerance):
        # 1) Evaluate the potential field wrench at the current pose in the potential field
        wrench = evaluate_wrench_at_pose(current_pose, goal_pose)
        # 2) Convert the wrench to a twist (velocity) and apply velocity/acceleration limits
        twist = convert_wrench_to_twist(wrench)
        twist_limited = apply_velocity_acceleration_limits(twist, dt)
        # 3) Integrate the twist to get the next pose using RK4 while still respecting limits
        current_pose = integrate_twist(current_pose, twist_limited, dt)
        path.append(current_pose)

    return path
```

# References
 [1] [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247)
