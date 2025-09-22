# Project Notes
This file contains several notes about various components of the project in no particular order. Follow this [Weekly Journal](https://docs.google.com/document/d/1IJ0fK89f-ZZ52c72lqh1HTT0OAE9BSruDTp8FuKGbpw/edit?usp=sharing) for weekly updates, meeting notes, online references, and more information relevant to the project.

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

# Robots to test
Robot Arms the Argallab uses:
- Ufactory X Arm 7-DOF
- Kinova Gen2 Jaco 7-DOF
- Kinova Mico 6-DOF

# Project Notes and TODO items
- Verify Units for all equations (demonstrate dimensional analysis for entire potential function)


# Spring 2025 Quarter Remarks
The Demo Node currently publishes a static transform that is intended to be the transform between the fixed frame and the robot’s base link and this should be handled in the launch file of the PF package instead of in the user node. The arguments for the frame and the transformation can be accepted as user args so the user can define it easily without writing code, but configuring the launch arguments.

The objects being parsed directly into PF obstacles are of type `urdf::Collision`, and the publisher for the obstacles should be adjusted to use a standard ROS type for obstacles instead of the custom type. This might require parameters for default PF obstacles to be defined elsewhere instead of through the ROS message. This will be helpful for _any_ ROS node to be able to publish obstacles to the potential field.