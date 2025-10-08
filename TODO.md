# Fall 2025 Project Goals
- Connect to planning with robot (Franka, Kinova) directly for users to avoid MoveIt
- Accept inverse kinematics function or library (IKFast) to create trajectories in the joint space, to then be sent to the Joint Trajectory Action Server
- Connect PF ROS Package to MoveIt Joint Trajectory Action Server to perform planning (and execution) to a goal pose and enable the user to call the ROS package with a simple interface to plan and execute a path
- Re-define obstacles and planning msg types to use standard ROS msg types and/or MoveIt types for interoperability and to reduce overhead and dependence on custom types
- Extract PF C++ library out of the ROS package so it can be used independently without installing the entire package
- For open sourcing, we may even want to consider installing the PF C++ library as a sub-module git repo to be maintained separately (not necessary but could be relevant if we identify a strong need for it)
- Configure CPPLint CI/CD Github Actions for PRs
---

- Finish "Planning World" pipeline
  - JointTrajectory (and Pose) trajectory planner interface using `trajectory_msgs/JointTrajectory`
  - Interface to hand `JointTrajectory` to a robot controller (libfranka for example) to execute the planned path
- Include installation instructions for fcl and libfranka and other external dependencies

- Condense ROS Nodes into a single node and condense PF library code together.
  - The `pfield` library should contain its own path planning function with all the necessary components (IK Solver, obstacle manager)
