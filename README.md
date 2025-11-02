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
These equations were obtained from a [Columbia Presentation on Potential Field Path Planning](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf). The paper: [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247) is the original reference for these equations and describes the derivation of the potential equations, how to obtain the gradients, and how to obtain a velocity from the gradients.

In this library, the potential field produces a task-space wrench over the robot’s end-effector pose:

- Linear force $\mathbf{F}(q)$ [N] from the translational potential
- Torque $\boldsymbol{\tau}(q)$ [Nm] from the rotational potential

This wrench is mapped to a task-space twist (velocity) using constant gains:

$$\begin{align}
\mathbf{v}(q) &= k_{lin}\,\mathbf{F}(q) \\
\boldsymbol{\omega}(q) &= k_{ang}\,\boldsymbol{\tau}(q)
\end{align}$$

with defaults of $k_{lin} = 1.0\,[(\mathrm{m/s})/\mathrm{N}]$ and $k_{ang} = 1.0\,[(\mathrm{rad/s})/\mathrm{Nm}]$. The computed twist is then limited by velocity and acceleration constraints before being integrated to obtain the next pose.

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

and the corresponding unit rotation axis $\mathbf{u} = \frac{\operatorname{vec}(q_{diff})}{\lVert\operatorname{vec}(q_{diff})\rVert}$. The attractive rotational torque is proportional to the geodesic angle and acts about $\mathbf{u}$:

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

### Velocity and Acceleration Limits
Before integrating the twist, linear and angular speeds are soft-saturated by norm and then rate-limited using maximum linear/angular accelerations over the step size $\Delta t$. The library enforces:

- Max linear speed $\lVert\mathbf{v}\rVert \leq v_{max}$
- Max angular speed $\lVert\boldsymbol{\omega}\rVert \leq \omega_{max}$
- Linear/Angular acceleration limits over $\Delta t$

This produces smooth, feasible motions while following the potential field.

### Mitigating Local Minima (Planning-only)
To reduce sticking on broad obstacle faces, the planner removes only the repulsive component that directly opposes the attractive force and keeps tangential components that encourage sliding:

$$
\mathbf{F}_{rep}^{\text{filtered}} = \mathbf{F}_{rep} - \min\big(0,\, \mathbf{F}_{rep}\cdot \mathbf{u}_{att}\big)\,\mathbf{u}_{att},\quad \mathbf{u}_{att} = \frac{\mathbf{F}_{att}}{\lVert\mathbf{F}_{att}\rVert}.
$$

This filtering is applied only during planning; the unfiltered field is kept for visualization and testing semantics.


# References
 [1] [Columbia Presentation on Potential Field Path Planning](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf)

 [2] [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247)
