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
- Create walkthrough video about files and what they do
- Go through ROS package release process for open sourcing on ROS index
- Fix Build Warnings

- Include installation instructions for fcl, pinnochio, libfranka and other external dependencies in README
- Create a minimum Docker Image for PF and robot dependencies
- Investigate Configuration Space planninewewfeeg with the PF
  - Principles of Robot Motion, Choset et al.
  - Modern Robotics, Lynch and Park
- Allow editing pfield parameters dynamically during runtime via ROS2 parameters
- Plot Net force vs time as well, maybe force on each joint. Would help debug Attractive and Repulsive forces.
- Use RK4 to integrate joint velocities into joint positions and for integrating joint positions into end-effector poses and test if this improves accuracy/stability of the planner.
- User node should be required to pass in mesh/urdf directory so that they can just type "object.obj" and have the path be resolved by the potential_fields_library

# Future TODOs and Refactors
- Create U-shaped obstacle test case for classic local minima problem
- Improve Extent Estimator ("Extentimator") to be more accurate and be realistic for robot arms
  - Add unit tests for Extent Estimator
- Define IKSolver and MotionPlugin more concretely. IKSolver should be an ABC for a user to implement an IK implementation.
  MotionPlugin should be an ABC for a user to implement how to talk to real robot and should be capable of both ROS-agnostic and ROS-aware implementations.
  - Refactor where solvers and robot_plugins are stored across the potential_fields_library and potential_fields ROS package.
- Document process for creating plots and how to save data during runtime for plotting later
- Review papers for improvements to potential field algorithms. Demiana is helping with researching relevant papers.
