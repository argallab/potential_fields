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
- Include installation instructions for fcl, pinnochio, libfranka and other external dependencies in README
- Allow PF with no goal, only obstacles (for avoidance behavior)

# Future TODOs and Refactors
- Investigate Configuration Space planning with the PF
- namespace pfield c++ library and externalize from ROS package
- Dynamic `switchToQuadraticPotential` based on current PF state (distance to goal, obstacles, current velocity, etc.)
- Create U-shaped obstacle test case for classic local minima problem
- Improve Extent Estimator ("Extentimator") to be more accurate and be realistic for robot arms
  - Add unit tests for Extent Estimator
