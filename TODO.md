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
- Work on Sphere robot demo with simple path planning through several obstacles
- Update documentation and README files to reflect new changes in PF Equations
- Include installation instructions for fcl, pinnochio, libfranka and other external dependencies
- Refactor pfield_lib with better naming and clearer definitions for maintaining a potential field
  - Update the wording/naming/labeling for gain parameters, influence zones, and function/variables
  - Update the README and in-code documentation to reflect PF wrench and twist definitions
  - Update plots, docs, and visualizations to reflect the correct units/etc.


# Future TODOs and Refactors
- Investigate Configuration Space planning with the PF
- Read Modern Robotics book about potential fields
- Read stephen lavalle planning book's section on following constraints with potential fields
- Think about smoothness while still enforcing velocity and acceleration limits
  - Improve interpolation and numerical integration methods
  - User should define motion constraints (velocity, acceleration, jerk) and planned path should respect those constraints
- namespace pfield c++ library and externalize from ROS package
- Allow PF with no goal, only obstacles (for avoidance behavior)
