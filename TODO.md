# Fall 2025 Project Goals
- Connect to planning with robot (Franka, Kinova) directly for users to avoid MoveIt
- Accept inverse kinematics function or library (IKFast) to create trajectories in the joint space, to then be sent to the Joint Trajectory Action Server
- Connect PF ROS Package to MoveIt Joint Trajectory Action Server to perform planning (and execution) to a goal pose and enable the user to call the ROS package with a simple interface to plan and execute a path
- Re-define obstacles and planning msg types to use standard ROS msg types and/or MoveIt types for interoperability and to reduce overhead and dependence on custom types
- Extract PF C++ library out of the ROS package so it can be used independently without installing the entire package
- For open sourcing, we may even want to consider installing the PF C++ library as a sub-module git repo to be maintained separately (not necessary but could be relevant if we identify a strong need for it)
- Configure CPPLint CI/CD Github Actions for PRs
---

# Immediate TODOs
- Finish PlanPath Service Implementation with FrankaIKPlugin to obtain a valid JointTrajectory
  - Create 3 different demos for testing:
    - No Obstacles and easy reachable goal (forwards or single motion)
    - With Obstacles surrounding a easily reachable goal
    - With obstacles obstructing the path to the goal
- Implement sending the JointTrajectory to the MoveIt JointTrajectory Action Server (Franka moveit demo)
- Use Pinnochio/KDL library on the given URDF to compute FK and update robot obstacles instead of listening to TF frames
  - Previous data-flow for URDF to PF Obstacles: URDF -> RSP (updates from JSP) -> TF -> PF Obstacles
  - New data-flow for URDF to PF Obstacles: URDF -> Pinnochio/KDL -> FK Definition -> Joint Angles -> PF Obstacles
- Condense ROS Nodes into a single node and condense PF library code together.
  - The `pfield` library should contain its own path planning function
  - `pfield` should also contain its own IKSolver and MotionPlugin instances with a factory method to create them
  - ROS Wrapper node should just handle ROS communication and call the `pfield` library functions

# Future TODOs and Refactors
- Think about smoothness while still enforcing velocity and acceleration limits
  - Improve interpolation and numerical integration methods
  - User should define motion constraints (velocity, acceleration, jerk) and planned path should respect those constraints
  - Somehow derive PF parameters from motion constraints (Repulsive Gain, Attractive Gain, Influence Zone Size, etc)
- Refactor pfield_lib with better naming and clearer definitions for maintaining a potential field
  - Update the wording/naming for gain parameters, influence zones, and function/variables
  - Update the README and in-code documentation to reflect these changes
  - Update plots and visualizations to reflect the correct units/etc.
- Include installation instructions for fcl, pinnochio, libfranka and other external dependencies
