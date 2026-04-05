# Potential Fields Mathematical Overview

These equations are drawn from *Principles of Robot Motion* (Choset et al.) [2] and *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots* (Khatib) [1], which derive the potential functions, their gradients, and how to obtain a velocity from those gradients.

The potential field produces a **task-space wrench** over the robot's end-effector pose, a linear force $\mathbf{F}(q)$ from the translational potential and a torque $\boldsymbol{\tau}(q)$ from the rotational potential. This wrench is converted to a task-space twist using constant admittance gains:

$$
\mathbf{v}(q) = k_{lin}\,\mathbf{F}(q), \qquad \boldsymbol{\omega}(q) = k_{ang}\,\boldsymbol{\tau}(q)
$$

with defaults $k_{lin} = 1.0\,[(\mathrm{m/s})/\mathrm{N}]$ and $k_{ang} = 1.0\,[(\mathrm{rad/s})/\mathrm{Nm}]$.

The potential functions are scalar fields in units of energy [Nm]. Their gradients are forces [N]. The gains $\zeta$ (attractive) and $\eta$ (repulsive) act as inverse damping coefficients [Ns/m], converting forces into velocities [m/s].

---

## Attractive Potential

The attractive potential combines a **quadratic** near the goal (for smooth convergence) with a **conical** far from the goal (for a constant-magnitude pull that avoids high velocities). The transition occurs at distance $\Gamma$.

$$
U_{att}(q) = \begin{cases}
\dfrac{1}{2}\zeta\, D(q, q_{goal})^2 & D(q, q_{goal}) \le \Gamma \\[6pt]
\Gamma\zeta\, D(q, q_{goal}) - \dfrac{1}{2}\zeta\Gamma^2 & D(q, q_{goal}) > \Gamma
\end{cases}
$$

The gradient gives the attractive force directed toward the goal:

$$
\mathbf{F}_{att}(q) = -\nabla U_{att}(q) = \begin{cases}
-\zeta\,(q - q_{goal}) & D(q, q_{goal}) \le \Gamma \\[6pt]
-\dfrac{\Gamma\zeta}{D(q, q_{goal})}\,(q - q_{goal}) & D(q, q_{goal}) > \Gamma
\end{cases}
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $\zeta$ | Attractive gain | Ns/m |
| $q_{goal}$ | Goal pose |  |
| $D(q, q_{goal})$ | Euclidean distance from current pose to goal | m |
| $\Gamma$ | Threshold distance between quadratic and conical regimes | m |

> **Implementation note:** The conical branch is currently disabled in `computeAttractiveForceLinear` (`pfield.cpp`). The function returns the pure quadratic force $-\zeta(q - q_{goal})$ at all distances, because the conical branch was observed to cause oscillations. As a result, $\Gamma$ has no effect at runtime. See [Issue #44](https://github.com/argallab/potential_fields/issues/44) for the ongoing investigation.

---

## Rotational Attraction

Let $q_c$ be the current unit quaternion and $q_g$ the goal orientation. The geodesic distance $\theta \in [0, \pi]$ is the shortest rotation aligning $q_c$ to $q_g$. Define the quaternion difference and its rotation axis:

$$
q_{diff} = q_c^{*} \otimes q_g, \qquad \mathbf{u} = \frac{\vec{q}_{diff}}{\lVert\vec{q}_{diff}\rVert}
$$

The attractive rotational torque is proportional to the geodesic angle, applied only when $\theta$ exceeds a small threshold:

$$
\boldsymbol{\tau}_{att}(q) = -\,\omega\,\theta\,\mathbf{u}
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $\omega$ | Rotational attractive gain | Ns/rad |
| $\theta$ | Geodesic angle between current and goal orientations | rad |
| $\mathbf{u}$ | Unit rotation axis from $q_c$ toward $q_g$ |  |

---

## Repulsive Potential

Repulsion increases with proximity to each obstacle and is zero beyond the influence distance $Q$. The signed distance $d$ to the obstacle surface is positive outside and negative inside; a value inside the obstacle is clamped to a small $\varepsilon > 0$ to produce a strong outward push. The repulsive force points along the outward surface normal $\mathbf{n}_{out}$:

$$
\mathbf{F}_{rep}(q) = \begin{cases}
\eta\,\left(\dfrac{1}{D(q)} - \dfrac{1}{Q}\right)\dfrac{1}{D(q)^2}\,\mathbf{n}_{out} & 0 < D(q) < Q \\[8pt]
\mathbf{0} & D(q) \ge Q
\end{cases}
$$

This is summed over all obstacles in the environment.

| Symbol | Description | Units |
|--------|-------------|-------|
| $\eta$ | Repulsive gain | Ns/m |
| $Q$ | Influence distance (repulsion is zero beyond this) | m |
| $D(q)$ | Signed distance from the query point to the obstacle surface | m |
| $\mathbf{n}_{out}$ | Outward unit normal of the obstacle surface at the closest point |  |

---

## Total Potential, Wrench, and Twist

The total potential is the sum of attractive and all repulsive contributions. Its negative gradient gives the net force, which is combined with the rotational torque to form the wrench, then mapped to a twist:

$$
\begin{align}
U(q) &= U_{att}(q) + \sum_i U_{rep,i}(q) \\[4pt]
\mathbf{F}(q) &= -\nabla U(q) \\[4pt]
\mathbf{v}(q) &= k_{lin}\,\mathbf{F}(q), \qquad \boldsymbol{\omega}(q) = k_{ang}\,\boldsymbol{\tau}_{att}(q)
\end{align}
$$

---

## Mitigating Local Minima (Planning-only)

A local minimum occurs when the attractive and repulsive forces exactly cancel, causing the planner to stall, most commonly when the robot is directly between a goal and an obstacle. To reduce sticking, the planner strips the repulsive component that directly opposes the attractive direction while keeping tangential components that encourage the robot to slide around the obstacle:

$$
\mathbf{F}_{rep}^{\text{filtered}} = \mathbf{F}_{rep} - \min\big(0,\, \mathbf{F}_{rep}\cdot \hat{\mathbf{u}}_{att}\big)\,\hat{\mathbf{u}}_{att}, \qquad \hat{\mathbf{u}}_{att} = \frac{\mathbf{F}_{att}}{\lVert\mathbf{F}_{att}\rVert}
$$

The $\min(0, \cdot)$ ensures only the opposing (negative) projection is removed; repulsion that is already tangential or aligned with attraction is left unchanged. This filtering is applied only during planning, the unfiltered field is used for real-time control and visualization.

---

## Path Planning Algorithm

This library supports two planning methods with different trade-offs between computational cost and collision-avoidance coverage. Both follow the same conceptual strategy: at each step, evaluate the potential field, convert the gradient to a velocity, and integrate to advance the robot's state. The planner stops when the robot reaches the goal within a positional tolerance.

### Task-Space Wrench Path Planning

Forces and torques are evaluated at the **end-effector** in Cartesian space. The resulting twist is integrated directly in task space, and IK solves for the joint configuration at each waypoint.

**Best suited for:** uncluttered environments where precise end-effector path control matters more than link-level collision avoidance.

**Steps:**

1. **Wrench**: sum attractive and repulsive contributions at the end-effector:

$$
\mathcal{W}_{task} = \begin{bmatrix} \mathbf{F}_{att} + \mathbf{F}_{rep} \\ \boldsymbol{\tau}_{att} \end{bmatrix}
$$

2. **Twist**: apply admittance gains:

$$
\mathcal{V}_{task} = \begin{bmatrix} k_{lin} & \mathbf{0} \\ \mathbf{0} & k_{ang} \end{bmatrix} \mathcal{W}_{task}
$$

3. **Integrate**: advance the end-effector pose with RK4 (see [RK4 Integration](#runge-kutta-4-rk4-integration)):

$$
x_{next} = \mathrm{RK4}(x_{curr},\, \mathcal{V}_{task},\, \Delta t)
$$

#### Runge-Kutta 4 (RK4) Integration

Rather than a simple Euler step, the planner uses a constrained RK4 scheme for better accuracy and stability. Each of the four stages evaluates a constrained twist at an intermediate pose, applies velocity and acceleration limits, then the four stages are averaged for the final update.

Given current pose $q_i = (\mathbf{x}_i, \mathbf{R}_i)$, previous applied twist $T_{i-1}$, and step size $\Delta t$:

**Stage 1**: evaluate at the current pose:
$$
\mathbf{k}_1 = T_c\big(q_i,\, T_{i-1},\, \Delta t\big), \qquad
q_{i}^{(1/2)} = \Big(\mathbf{x}_i + \tfrac{\Delta t}{2}\mathbf{v}_1,\; \mathbf{R}_i\exp\!\big([\boldsymbol{\omega}_1]_\times \tfrac{\Delta t}{2}\big)\Big)
$$

**Stage 2**: evaluate at the half-step pose from stage 1:
$$
\mathbf{k}_2 = T_c\big(q_{i}^{(1/2)},\, \mathbf{k}_1,\, \tfrac{\Delta t}{2}\big), \qquad
q_{i}^{(1/2)'} = \Big(\mathbf{x}_i + \tfrac{\Delta t}{2}\mathbf{v}_2,\; \mathbf{R}_i\exp\!\big([\boldsymbol{\omega}_2]_\times \tfrac{\Delta t}{2}\big)\Big)
$$

**Stage 3**: evaluate at the half-step pose from stage 2:
$$
\mathbf{k}_3 = T_c\big(q_{i}^{(1/2)'},\, \mathbf{k}_2,\, \tfrac{\Delta t}{2}\big), \qquad
q_{i}^{(1)} = \Big(\mathbf{x}_i + \Delta t\,\mathbf{v}_3,\; \mathbf{R}_i\exp\!\big([\boldsymbol{\omega}_3]_\times \Delta t\big)\Big)
$$

**Stage 4**: evaluate at the full-step pose from stage 3:
$$
\mathbf{k}_4 = T_c\big(q_{i}^{(1)},\, \mathbf{k}_3,\, \Delta t\big)
$$

**Weighted average** of the four stage twists:
$$
\bar{\mathbf{v}} = \frac{\mathbf{v}_1 + 2\mathbf{v}_2 + 2\mathbf{v}_3 + \mathbf{v}_4}{6}, \qquad
\bar{\boldsymbol{\omega}} = \frac{\boldsymbol{\omega}_1 + 2\boldsymbol{\omega}_2 + 2\boldsymbol{\omega}_3 + \boldsymbol{\omega}_4}{6}
$$

The averaged twist is re-limited (soft saturation and rate limits) before the final pose update:
$$
\mathbf{x}_{i+1} = \mathbf{x}_i + \bar{\mathbf{v}}\,\Delta t, \qquad
\mathbf{R}_{i+1} = \mathbf{R}_i\;\exp\!\big([\bar{\boldsymbol{\omega}}]_\times\,\Delta t\big)
$$

where $\exp([\boldsymbol{\omega}]_\times\,\Delta t)$ is the angle–axis exponential map $\mathrm{AngleAxis}(|\boldsymbol{\omega}|\Delta t,\, \boldsymbol{\omega}/|\boldsymbol{\omega}|)$, with the resulting quaternion normalized.

| Symbol | Description | Units |
|--------|-------------|-------|
| $T_c(\cdot)$ | Constrained twist function: applies local-minima filtering, maps wrench to twist, enforces velocity and acceleration limits |  |
| $[\boldsymbol{\omega}]_\times$ | Skew-symmetric matrix of $\boldsymbol{\omega}$ (so(3) element for the exponential map) |  |
| $\Delta t$ | Integration timestep | s |

> **Note:** Per-stage rate limits use the stage's effective step size ($\Delta t$ for stages 1 and 4, $\Delta t/2$ for stages 2–3). After averaging, constraints are applied once more before the final integration step.

#### Velocity and Acceleration Limits

Before integrating each twist, linear and angular velocities are constrained in two passes:

**Soft saturation**: smoothly caps a vector's norm to $v_{max}$ while preserving its direction, using the $\tanh$ function to avoid hard clipping:

$$
\mathbf{v}_{sat} = \mathbf{v}\cdot\frac{v_{max}\,\tanh\!\left(\beta\,\lVert\mathbf{v}\rVert / v_{max}\right)}{\lVert\mathbf{v}\rVert}
$$

When $\lVert\mathbf{v}\rVert \ll v_{max}$ the scaling is nearly linear; as $\lVert\mathbf{v}\rVert \to \infty$ the saturated norm asymptotes to $v_{max}$. Higher $\beta$ gives a sharper transition near the cap.

**Rate limiting**: bounds the change from the previous vector $\mathbf{v}_{prev}$ to the current target by a maximum step $\Delta_{max}$ (derived from acceleration limits over $\Delta t$):

$$
\mathbf{d} = \mathbf{v}_{curr} - \mathbf{v}_{prev}, \qquad
\mathbf{v}_{rl} = \begin{cases}
\mathbf{v}_{curr} & \lVert\mathbf{d}\rVert \le \Delta_{max} \\[4pt]
\mathbf{v}_{prev} + \mathbf{d}\,\dfrac{\Delta_{max}}{\lVert\mathbf{d}\rVert} & \lVert\mathbf{d}\rVert > \Delta_{max}
\end{cases}
$$

Soft saturation is re-applied after rate limiting to ensure the final vector satisfies both constraints.

| Symbol | Description | Units |
|--------|-------------|-------|
| $v_{max}$ | Maximum linear speed | m/s |
| $\omega_{max}$ | Maximum angular speed | rad/s |
| $\beta$ | Soft-saturation steepness parameter |  |
| $\Delta_{max}$ | Maximum allowed velocity change per step (acceleration limit $\times\,\Delta t$) | m/s |

---

### Whole-Body Velocity Path Planning

This method extends potential field planning to the full robot body by placing repulsive control points on every link, not just the end-effector. The approach follows §4.7.2–4.7.3 of Choset et al. [2].

**Best suited for:** cluttered environments where links and elbows risk colliding with obstacles. Every link of the arm contributes a repulsive force, so the robot naturally folds or repositions itself while driving the end-effector toward the goal.

#### Workspace Forces → Configuration Space Forces (§4.7.1)

Potential functions are easiest to define in the workspace ($\mathbb{R}^3$), but planning requires forces in **configuration space** (joint torques). We bridge these using the **principle of virtual work**: power is coordinate-independent, so $f^T\dot{x} = u^T\dot{q}$. With $\dot{x} = J(q)\dot{q}$, this gives:

$$
u = J^T(q)\,f
$$

A workspace force $f$ acting at any point $x = \phi(q)$ maps to a generalized joint-space force $u$ via the transpose of the Jacobian at that point.

#### Control Points (§4.7.2)

Rather than integrating forces over the robot's volume, a finite set of **control points** $\{r_j\}$ is selected on the robot body. Each control point has its own workspace potential; the resulting workspace forces are mapped to joint torques via $J^T$ and summed in configuration space.

**Floating control point:** Fixed points at link origins or vertices may miss the true closest approach between a link and an obstacle. For example, the midpoint of a long link edge can be far closer to an obstacle than either endpoint. The textbook therefore introduces a *floating* control point $r_\text{float}$: the point on the robot boundary closest to any obstacle at the current configuration $q$. Its repulsive force is computed identically to any other control point.

**In this implementation**, each link is approximated as a **capsule** fitted to the URDF mesh via PCA. The capsule's two endcap centers and shaft midpoint are the fixed candidate control points per link. For each link–obstacle pair, the candidate with the smallest clearance is selected as the floating control point, giving a smooth and geometrically tight clearance estimate as the configuration changes.

#### Repulsive Potential at a Control Point (§4.7.2, eq. 4.24–4.25)

For control point $r_j$ and obstacle $\mathcal{WO}_i$, the repulsive potential and its gradient (the workspace repulsive force) are:

$$
U_{\text{rep},i,j}(q) =
\begin{cases}
\dfrac{1}{2}\eta_j \left(\dfrac{1}{d_i(r_j(q))} - \dfrac{1}{Q^*_i}\right)^2 & d_i(r_j(q)) \le Q^*_i \\[8pt]
0 & d_i(r_j(q)) > Q^*_i
\end{cases}
$$

$$
\nabla U_{\text{rep},i,j}(q) =
\begin{cases}
\eta_j \left(\dfrac{1}{Q^*_i} - \dfrac{1}{d_i(r_j(q))}\right)\dfrac{1}{d_i^2(r_j(q))}\,\nabla d_i(r_j(q)) & d_i(r_j(q)) \le Q^*_i \\[8pt]
0 & d_i(r_j(q)) > Q^*_i
\end{cases}
$$

$\nabla d_i(r_j(q))$ is the outward unit normal of the obstacle surface at the point closest to $r_j$.

| Symbol | Description | Units |
|--------|-------------|-------|
| $r_j$ | Control point $j$ on the robot body |  |
| $d_i(r_j(q))$ | Shortest distance from control point $r_j$ to obstacle $\mathcal{WO}_i$ | m |
| $Q^*_i$ | Influence distance for obstacle $i$ | m |
| $\eta_j$ | Repulsive gain for control point $r_j$ | Ns/m |
| $\nabla d_i(r_j(q))$ | Outward unit normal of obstacle $i$'s surface at the closest point to $r_j$ |  |

#### Total Configuration Space Force (§4.7.2, eq. 4.26)

The total generalized force is the sum of end-effector attraction and whole-body repulsion, accumulated over all links and all obstacles. Crucially, this summation happens in **configuration space**, summing workspace forces directly would be incorrect because the same workspace force has different joint-torque implications depending on which link it acts on.

$$
u(q) = \underbrace{J_{ee}^T(q)\,\mathbf{F}_{att}}_{\text{EE attraction}} \;+\; \underbrace{\sum_{j \in \text{links}}\;\sum_{i \in \text{obstacles}} J_j^T(q)\,\nabla U_{\text{rep},i,j}(q)}_{\text{whole-body repulsion}}
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $J_{ee}(q)$ | Jacobian at the end-effector |  |
| $\mathbf{F}_{att}$ | Attractive force toward the goal, evaluated at the end-effector | N |
| $J_j(q)$ | Jacobian evaluated at the floating control point on link $j$ |  |

#### Joint Velocity Integration

The total configuration space force $u(q)$ is treated as a joint velocity command via an admittance gain, then integrated with an Euler step:

$$
\dot{\mathbf{q}} = k_{\text{adm}}\,u(q), \qquad \mathbf{q}_{next} = \mathbf{q}_{curr} + \dot{\mathbf{q}}\,\Delta t
$$

| Symbol | Description | Units |
|--------|-------------|-------|
| $k_{\text{adm}}$ | Admittance gain (scales configuration-space force to joint velocity) |  |
| $\dot{\mathbf{q}}$ | Joint velocity vector | rad/s |

> **Note:** The admittance mapping $\dot{\mathbf{q}} = k_{\text{adm}}\,u(q)$ can be replaced by the full robot dynamics equation (using the mass matrix and Coriolis terms) for more physically accurate joint velocity commands (see [Issue #23](https://github.com/argallab/potential_fields/issues/23)).

---

# References

[1] [Real-Time Obstacle Avoidance for Manipulators and Mobile Robots](https://ieeexplore.ieee.org/document/1087247)

```
O. Khatib, "Real-time obstacle avoidance for manipulators and mobile robots," in Proc. 1985 IEEE Int. Conf. Robotics and Automation, vol. 2, pp. 500–505, 1985, doi: 10.1109/ROBOT.1985.1087247.
```

[2] [Principles of Robot Motion: Theory, Algorithms, and Implementations](https://ieeexplore.ieee.org/book/6267238)

```
H. Choset, K. M. Lynch, S. Hutchinson, G. A. Kantor, W. Burgard, L. E. Kavraki, and S. Thrun, Principles of Robot Motion: Theory, Algorithms, and Implementations. Cambridge, MA: MIT Press, 2005.
```
