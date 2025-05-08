# pfields_2025
MSR project to revamp current pfields repo for open sourcing

## Nodes from ROS1 Implementation
These nodes will be reimplemented in ROS 2 written in C++. The project proposal outlines some core features that will be supported in the reimplementation:
- Consider collisions with all parts of robot and environment
- Consider self-collision
- On-the-fly mesh environment adaptation (from (depth) camera input)
- Goal (attractor potentials) setup and switching

### Hybrid Control Input
- Manages consumption of human velocity input (joystick, keyboard, etc.)
- Smooths input velocity using several measurements
- Publishes human velocity vector to eventually be fused with autonomy velocity vector.

*(Autonomy velocity vector is the vector the potential field generates given an input state: robot state, goal position, obstacles)*

### Potential Field Manager
- Initialization of a vector field in a parameterized environment (size, obstacles, robot, etc.)
- Updating vector field with dynamic updates to environment (obstacles, robot state, other kinematic constraints)
  - *(Use mesh objects for obstacles?)*
  - **Goal (attractor potentials) setup and switching**
- Compute autonomy velocity vector given robot state, goal position, and obstacles
  - **Consider collisions with all parts of robot and environment**
  - **Consider self-collision**

### Robot Controller
- Obtain robot state and maintain persistent state
- Observe goal (continuously, in case of goal updates)
- Query PF Manager to obtain autonomy velocity vector
- Subscribe to human velocity vector
- Fuse human and autonomy velocity vectors to obtain a resulting velocity vector to apply to the robot
- Publish RViz markers for robot state, velocity vectors (human, autonomy, fused), obstacles, and goal positions
- **On-the-fly mesh environment adaptation (from (depth) camera input)**

# Potential Fields Implementation Approaches

1. 3D Map `std::unordered_map<Position3D, PotentialForce>` storing potential force (needs to be hashable) of the task-space.
    - Computationally fast but will use a lot of memory depending on potential container memory footprint and hashing complexity.
    - Dynamically updating will need to update entire vector. Maybe we can do it smartly by only updating relevant areas.
    - Could leave map sparse so we aren't front-loading a lot of precomputation and fill in map as we explore the task-space stemming from the robot-state, obstacles, and the goal.
    - Can use a [`kd-tree`](https://github.com/cdalitz/kdtree-cpp) instead of a hashmap to store positions.
      - Time Complexity: Construction is `O(n log(n))`, nearest neighbor query is `O(n)` (if balanced then `O(log(n))`), insertion is `O(log(n))`
      - Space Complexity: `O(n)`
      - [Wikipedia](https://en.wikipedia.org/wiki/K-d_tree)
    - Can use an [`Octree`](https://github.com/attcs/Octree) to split regions and model the task-space as a recursive 3D volume with subregions
      - `m` is number of occupied nodes, `W` is world size, `R` is resolution of leaf node (volume)
      - Time Complexity: Construction is `O(n log(W/R))`, query is `O(log(W/R))`, insertion is `O(log(W/R))`
      - Space Complexity: `O(m)`
      - [Wikipedia](https://en.wikipedia.org/wiki/Octree)
2. Obstacled-Based, calculate forces using robot-state and set of obstacles without needing to precompute potential in the entire task-space.
    - Computationally slower than precomputation but would work well with small set of obstacles. Uses less memory since we aren't maintaining a huge 3D HashMap
    - Can use meshes to represent obstacles for more complex obstacles

# Potential Equations
These equations were obtained from a [Columbia Presentation on Potential Field Path Planning](https://www.cs.columbia.edu/~allen/F17/NOTES/potentialfield.pdf). The potential functions are scalar fields representing potential energy (Newton-meters [Nm]). The gradient of the potential is represented as a force (Newtons [N]). To obtain velocity vectors, we use some parameter ($\zeta$ and $\eta$) that acts as an inverse damping coefficient (Newton-seconds / meter [Ns/m]) to convert the force into a velocity (meters / second [m/s]).
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

## Repulsive Potential
Repulsive force increases with proximity to obstacle. Multiple obstacles create commutative forces.

$$
\begin{align}
U_{rep}(q) &= \left\{\begin{matrix}\frac{1}{2}\eta\left(\frac{1}{D(q)} - \frac{1}{Q^*}\right)^2 & \text{if } D(q) \leq Q^* \\ 0 & \text{if } D(q) > Q^*\end{matrix}\right. \\
\nabla U_{rep}(q) &= \left\{\begin{matrix} \eta \left(\frac{1}{D(q)} - \frac{1}{Q^*}\right) \frac{1}{D^2(q)} \nabla D(q)& \text{if } D(q) \leq Q^* \\ 0 & \text{if } D(q) > Q^*\end{matrix}\right.
\end{align}
$$

Where:
- $\eta$ is the repulsive gain parameter
- $Q^*$ is the influence radius of the obstacle
- $D(q)$ is the euclidean distance between the vector $q$ and the obstacle

## Total Potential Function

$$
\begin{align*}
U(q) &= U_{att}(q) + U_{rep}(q) \\ 
F(q) &= - \nabla U(q) \\
v &= k\cdot F(q) \\
v &= \zeta \nabla U_{att}(q) + \eta \nabla U_{rep}(q)
\end{align*}
$$

Where:
- $q$ is a vector in the potiential field (position/pose)
- $k$ is a constant (combination of $\zeta$ and $\eta$) converting force into velocity

We can utilize **gradient descent** to employ a motion-strategy to travel down the potential gradient:

$$
\begin{align*}
&q[0] = q_{start} \\
&i = 0 \\
&\text{while}||\nabla U(q_i)|| > \epsilon \\
&\,\,\,\, q_{i+1} = q_i - \alpha_i \nabla U(q_i) \\
&\,\,\,\, i = i + 1
\end{align*}
$$

Where:
- $\epsilon$ parameterizes when to stop gradient descent
- $\alpha_i$ is a step-size (learning rate) to progress towards the gradient (specific to each iteration)