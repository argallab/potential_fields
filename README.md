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