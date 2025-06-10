# pfields_2025
MSR project to revamp current pfields repo for open sourcing

# ROS Package Structure and Overview

## Potential Field C++ Library
The `pfield` library is a ROS-independent C++ API that the ROS package maintains an instance of. Vectors and Quaternions are implemented with Eigen and are wrapped into a singular object named `SpatialVector`. The Vector portion defines the translational velocity [m/s] towards the goal and the Quaternion represents the unit quaternion difference between the orientation and the goal orientation. Currently the C++ library is embedded in the include folder (header-only implementation) of the ROS package but will be extracted and presented as a standalone C++ library that the ROS package will need to link with.

## PotentialFieldManager (ROS Node)
Manages the `pfield` instance and visualizes the obstacles, the goal pose, and the "planned" path. Subcribes to `/goal_pose` to get the updated goal pose and updates the internal pfield instance's goal pose. Also subscribes to `/pfield/obstacles` to obtain the obstacles for the potential field. The obstacles can be published from any node and the robot parser node (see below) handles publishing the robot's geometry as pfield obstacles. The manager also handles user arguments to setup RViz and tune the attraction/repulsive gains.

## Robot Parser (ROS Node)
Subscribes to the robot description and publishes the `pfield` obstacles consistent with the URDF available. Requires that the user publishes a transform from a fixed frame (provided as a launch argument) to the base link of the robot. Publishes the collision objects from the URDF as pfield obstacles to the `/pfield/obstacles` topic to send updated obstacles.

# Potential Equations
These equations were obtained from a [Columbia Presentation on Potential Field Path Planning](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf). The paper: [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247) is the original reference for these equations and describe the derivation of the potential equations, how to obtain the gradients, and how to obtain a velocity from the gradients.

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
Geodesic distance $\theta$ is the distance between two unit quaternions $q_1$ and $q_2$ (shortest angle of rotation required to align one orientation with the other on $\mathcal{S}^3$), bound to $\theta \in [0, \pi]$ is.

We can use the geodesic distance $\theta$ and a rotational attractive gain parameter $\omega$ ($0.7$ in original implementation) to scale the quaternion representing the difference $q_{diff}$ to act as a rotational attraction force:

$$
\begin{align}
q_{diff} &= \langle\left(q_1\right)^{-1} , q_2 \rangle \\
q_{attr} &= q_{diff} \cdot \omega
\end{align}
$$

Where:
- $\langle q_1, q_2 \rangle$ is the quaternion product of $q_1$ and $q_2$.
- $q_{attr}$ is the rotational attraction to orient the query pose towards the goal orientation.

## Repulsive Potential
Repulsive force increases with proximity to obstacle. Multiple obstacles create commutative forces.

$$
\begin{align}
U_{rep}(q) &= \begin{cases}\frac{1}{2}\eta\left(\frac{1}{D(q)} - \frac{1}{Q^{*}}\right)^2 & \text{if } D(q) \leq Q_{IR} \\ 0 & \text{if } D(q) > Q_{IR}\end{cases} \\
\nabla U_{rep}(q) &= \begin{cases} \eta \left(\frac{1}{D(q)} - \frac{1}{Q_{IR}}\right) \frac{1}{D^2(q)} \nabla D(q)& \text{if } D(q) \leq Q_{IR} \\ 0 & \text{if } D(q) > Q_{IR}\end{cases}
\end{align}
$$

*Note that the gradient result doesn't include a minus sign inside the parenthesis, this is because the negative value is absorbed by the distance calculation, this is verified within the visualization.*

Where:
- $\eta$ is the repulsive gain parameter
- $Q_{IR}$ is the influence radius of the obstacle
- $D(q)$ is the euclidean distance between the vector $q$ and the obstacle

## Total Potential Function

$$
\begin{align}
U(q) &= U_{att}(q) + U_{rep}(q) \\ 
F(q) &= - \nabla U(q) \\
v(q) &= k\cdot F(q) \\
v(q) &= \zeta \nabla U_{att}(q) + \eta \nabla U_{rep}(q)
\end{align}
$$

Where:
- $q$ is a vector in the potiential field (position/pose)
- $k$ is a constant (combination of $\zeta$ and $\eta$) converting force into velocity
- $v(q)$ is the velocity vector computed at vector $q$

We can utilize **gradient descent** to employ a motion-strategy to travel down the potential gradient:

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
- $\epsilon$ parameterizes when to stop gradient descent
- $\alpha_i$ is a step-size (learning rate) to progress towards the gradient (specific to each iteration)


# References
 [1] [Columbia Presentation on Potential Field Path Planning](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf)
 
 [2] [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247)
