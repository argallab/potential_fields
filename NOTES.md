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
- Franka Emika Panda 7-DOF (ROS2 MoveIt)
- Ufactory X Arm 7-DOF (ROS2)
- Kinova Gen2 Jaco 7-DOF (ROS1)
- Kinova Mico 6-DOF (ROS1)

# Project Notes and TODO items
- Verify Units for all equations (demonstrate dimensional analysis for entire potential function)

# References
- Pinocchio
  - [Pinocchio](https://github.com/stack-of-tasks/pinocchio.git)
- Xarm
  - [custom singularities avoidance](https://github.com/argallab/argallab_jparse)
  - [python api](https://github.com/xArm-Developer/xArm-Python-SDK.git)

- Jaco
  - [kinova ros](https://github.com/argallab/kinova-ros/tree/3008ff1b7b4014bb20a36beda49555817062d919)

# Spring 2025 Quarter Remarks
The Demo Node currently publishes a static transform that is intended to be the transform between the fixed frame and the robot’s base link and this should be handled in the launch file of the PF package instead of in the user node. The arguments for the frame and the transformation can be accepted as user args so the user can define it easily without writing code, but configuring the launch arguments.

The objects being parsed directly into PF obstacles are of type `urdf::Collision`, and the publisher for the obstacles should be adjusted to use a standard ROS type for obstacles instead of the custom type. This might require parameters for default PF obstacles to be defined elsewhere instead of through the ROS message. This will be helpful for _any_ ROS node to be able to publish obstacles to the potential field.

# Motion Interface
The `MotionInterface` will bridge the computed PF velocity/path to a robot's motion controller. This should be universal enough so users can adapt it to their robot and can be done in a few ways:

1. Building the entire path by providing PF parameters (goal, obstacles, start pose) and obtaining a path to follow
2. "Real-time" velocity request by providing a setup PF (goal, obstacles) with a current pose to produce the computed velocity vector
3. Discretizing the path with velocity vectors with respective time stamps, requesting the current velocity vector by providing a current time stamp (and a start time).


# Franka Emika Panda Notes
[`libfranka`](https://github.com/frankarobotics/libfranka) is a C++ library to control the robot. The output from the potential fields will be an end-effector velocity, so we will need an analagous control method within `libfranka` to send velocity commands to the robot.

Another library is [`franky`](https://github.com/TimSchneider42/franky), which can be used to control the robot in Python and C++ using a high-level interface.

## Generate Cartesian Velocity Commands Example
The [Cartesian Velocity Motion Example](https://github.com/frankarobotics/libfranka/blob/main/examples/generate_cartesian_velocity_motion_external_control_loop.cpp) demonstrates how to apply cartesian velocity commands to the robot with an external control loop.


Stick to Moveit compatible msgs for planning path using position control. Trajectory_msgs/JointTrajectory
Update PF when robot moves to build path.
Think about replan timer.
Build-path should be able to do moving+planning or planning and then moving later.
Convert to JointTrajectory from plan.
Parrallize planning.
Send JT to JTAS when running MoveIt Franka demo. Doesn't matter if MoveIt.
Generic programming (Policy based design) for different IK solvers.

## IK Libraries
[Andrew Dornbush's Trac-IK](https://github.com/aurone/trac_ik), [Traclabs Track-IK](https://bitbucket.org/traclabs/trac_ik/src/rolling/)
[KDL](https://github.com/nbfigueroa/robot_kinematics_kdl) (ROS1 XArm)
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (ROS2 XArm)
[GeoFIK](https://arxiv.org/abs/2503.03992) (Franka Emika Panda)
[BioIK](https://github.com/TAMS-Group/bio_ik) (Kinova Jaco)

# What the User needs to provide to use PFields with their robot
1. Robot Model (URDF, raw or xacro)
   - Needs Collision elements for all links
   - Needs Joint limits for all joints
   - In addition to the URDF, include any mesh files used in the URDF
   - Static Transform from chosen fixed frame to robot base link (typically identity transform)
2. `MotionPlugin` implementation for their robot
   - `IKSolver` implementation with convergence criteria to pick "best" solution from multiple solutions if applicable
   - Robot interface to obtain robot state (joint states, end-effector pose)
   - Robot interface to send end-effector velocity commands or joint trajectory commands to the robot
3. (Optional) Obstacle publisher node to publish obstacles in the environment as standard ROS messages
   - Note that these are separate from the robot's collision objects, which are automatically parsed into PF obstacles
4. (Optional) Goal publisher node to publish goal positions for the robot to plan
5. (Optional) Human input node to publish human velocity commands for the robot to fuse with the autonomy commands
   - Joystick, keyboard, or other teleoperation interface
6. Service Client for Path Planning
    - Provide start pose (typically current end-effector pose)
    - Service internally requires a `MotionPlugin` for the robot (includes `IKSolver`)
    - Receive planned path as `trajectory_msgs/JointTrajectory` message

# Path Planning Notes
```cpp
// interpolatePath will need to account for robot motion influencing the PF
// during the motion so the function will need to be re-written.
// Start is not guaranteed to be at the robot's current EE position since
// planning should be supported for any arbitrary starting Pose
// 1. Given the current EE Pose (starting at Start), compute the robot's joint angles with an IKSolver attached to PFM
//    - For picking one of many IK solutions, use solution most similar to current robot state OR neutral/home robot state
//    - Also initialize/reset our planning PField if not initialized already.
// 2. Now that we have joint angles from the IK solver, publish these joint angles to the planning-copy of the JointStates
//    - Once planning JS are published, the planning copy of the robot's TF frames will update and RobotParser
//      will handle them and publish planning obstacles.
// 3. Take a look at the planning obstacles that were published by RobotParser and update the planning PField with them
//    - The planning PField should now have new obstacles but the same goal as the normal PField
// 4. Compute a new velocity from the planning PField
// 5. Project the new velocity onto the current EE pose using the deltaTime to obtain a new EE Pose
// 6. The new EE Pose is used for the loop and we repeat steps 1-5 until the EE Pose and the goal Pose are within the tolerance
// 7. Return the EE Path (nav_msgs/Path) and the Robot's joint positions (trajectory_msgs/JointTrajectory)
// 8. Compute the joint velocities and put the joint velocity path into the msg (trajectory_msgs/JointTrajectory)
//    - In order to do this, the IKSolver must offer the Jacobian to convert EE Velocites into Joint Velocities
```


# Notes for 10/08 Meeting
KDL and Pinnochio: input a URDF and get FK from that
Use IK -> Obstacles more directly
Push to get it working.
If I didn't have ROS, how would it work?
No need for MotionInterface, PFieldManager should be the only node.
1. Finishing Planning Service (PlanPath) Set Goal, Obstacles, Franka MoveIt Simulation moves to that goal
2. Start designing out diagram (ROS interaction, PF)
3. Lower dimensional interface with Franka

## Progress Notes
Paths are able to be planned from the service and the demo node shows how to call the service and visualize it.
A couple of notes:
- Need to clean up RSP/JSP confusion with a simple Pinnochio-powered RobotParser to convert Joint Positions to Obstacle Positions
- The new `RobotParser` should simply just call the c++ library functions to add obstacles after using Pinnochio to compute FK
- Need to use a single ROS node (PFM) that wraps around the PField class, absorb `RobotParser` into PFM
- pfield c++ library needs to contain `MotionPlugin`. and support a planPath function that the ros service just calls
- FollowJointTrajectory action client needs debugging since it is returning a PATH_TOLERANCE_VIOLATED error
- Path smoothing of some sort needs to be implemented

```log
[ros2_control_node-4] [INFO] [1761144377.856630911] [fer_arm_controller]: Received new action goal
[ros2_control_node-4] [ERROR] [1761144377.857129721] [fer_arm_controller]: Received trajectory with non-zero start time (1761144340.282935) that ends in the past (1761144346.197935)
```


# Franka Emika Panda Notes
- [Kris' Joystick Repo](https://github.com/wengmister/franka_joystick_teleop/blob/main/README.md)
- [Franka User Setup (Matt Elwin's Notes)](https://nu-msr.github.io/ros_notes/ros2/franka.html)

# Notes from 10/15 Meeting
- Revisit clamping and enforcing velocity/acceleration limits
  - Can smooth out entire trajectory after integrating a path
  - Think about how to adjust PF parameters so forces always give valid EE wrenches
- Refactor /goal_pose since it will get confused with navigation goal pose topic
- Keep MotionPlugin separate but as a member of PotentialField
- Implement the Pinocchio FK for obstacles and remove JSP/RSP


Maintain a distancemap of the robot's geometry to obstacles for thresholding
Sphere robot demo with some extent, obstacles influence should scale appropriately.
Single integrator
Simple joint trajectory
Look into Configuration space planning instead
Handle self collision externally

# RK4
https://lpsa.swarthmore.edu/NumInt/NumIntFourth.html

# 11/5 Meeting Notes
- Mesh objects to primitive estimation
- Test on Franka with some simple obstacles
  - MoveIt Demo
- Solid Euclidean Planner demo video on Robot
- Give Demiana an example for the code to use with her robots
- Test on XArm 7-DOF this week
- IK and URDF/Xacro

## Sequence for testing UFactory Xarm7

Start the Docker Container:

```bash
docker start -i argallab_pfields
```

If edits are made to the docker file, delete the image using `docker rmi <image_id>`, using the image id shown from `docker images`.
Then recreate the image by building the docker image:

```bash
# In the Docker/ directory
docker build -t pfields .
# After the image is built, the container can be created and started by running:
sudo docker run -it --privileged \
-v /dev:/dev \
-v /home/$USER/workspaces/pfield_ws:/home/workspace/src \
-e DISPLAY \
-e QT_X11_NO_MITSHM=1 \
--name argallab_pfields \
--net=host \
pfields:latest
```

Once the docker container is started, run `colcon build` and `source install/setup.bash` to initialize the ROS workspace, and then launch the appropriate nodes:

```bash
# Robot Launch
ros2 launch xarm_moveit_config xarm7_moveit_realmove.launch.py robot_ip:=192.168.1.199 add_gripper:=true
# Launch JParse
ros2 launch manipulator_control xarm_main_vel.launch use_teleop_control:=true use_teleop_control_jparse:=true
# Launch PF Node that hosts Path Planning service (and demo service)
ros2 launch potential_fields_demo pf_demo.launch.xml use_rviz:=false motion_plugin_type:="xarm" end_effector_frame:="link_tcp" urdf_file_path:="xarm7.urdf"
```

### Demo Obstacles
Obstacle definitions for demo. Positions are relative to robot base

```yaml
wooden_box:
  frame_id: "Wooden_Box"
  type: "Box"
  group: "Static"
  pose:
    position:
      x: 0.94 # [m]
      y: -0.31 # [m]
      z: 0.19 # [m]
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  radius: 0.0
  length: 0.38
  width: 0.38
  height: 0.38
  mesh_resource: ""
  scale_x: 0.0
  scale_y: 0.0
  scale_z: 0.0
```

# 11/12 Meeting Notes
robot modularity documentation (other robots)
rest of arm hitting obstacles using mesh links
real obstacles demo, maybe foam so it's safer
teleop demo

# Planning Paths while avoiding collisions between robot geometry (links) and obstacles
- The geometry of the robot should be able to be represented with primitives and meshes
- Given a robot state (joint angles), the robot geometry should be able to be transformed to the world frame
- We need a function to compute the minimum distance between all robot links and all obstacles in the environment
- The minimum distance can be used to threshold paths that are too close to obstacles and mark for replanning

## Replanning Strategy
While planning a path, the joint angles are recorded via the IKSolver. We can also record the minimum clearance between the
robot geometry and the obstacles in the environment at each step of the path planning. With a threshold distance, we can mark
sections of the path that are too close to obstacles and replan accordingly.

# COAL Integration Notes

```
@misc{coalweb,
   author = {Jia Pan and Sachin Chitta and Dinesh Manocha and Florent Lamiraux and Joseph Mirabel and Justin Carpentier and Louis Montaut and others},
   title = {Coal: an extension of the Flexible Collision Library},
   howpublished = {https://github.com/coal-library/coal},
   year = {2015--2024}
}
```

# 12/3 Meeting Notes
- Attempted to use dynamics equation but ran into problems
  - Damping and tuning gain parameters correctly is difficult
  - Probably better to just use a gain parameter to convert torques to velocities
- Termination bug was due to small forces so increasing gains helped resolve that
- Next Step is to revert to gain parameter and test with obstacles at SRA

## Post-Meeting
Try enforcing maximum force before it becomes a Torque.
Try using pinocchio to get next joint velocities instead of integrating
Try implementing dynamics equation with some other library
Get some videos of a working demo with the dynamics reverted.
Start Focusing on release-specific tasks:
  - Documentation
  - README + Code Documentation
  - Prep for ROS Index + Release

## Potential (pun intended) Infrastructure Edits
- If we're using an example that already has a ROS environment, we shouldn't need to pass in a raw URDF file, we should be able to just point to a robot description topic and propogate that throughout the ROS library and C++ library for use. If we're not using ROS, we will need to provide one.
- PFM can listen for the TF of the given eeLinkName and if found, use it as the "current" position for visualization of the query pose

# December 09 2025 Notes
Sending Joint Velocity trajectory from the demo node produces decent path.
However, looks like joint velocity trajectory is asympytotic near the end and has very small joint vels for a while near the goal
Despite capping joint velocity trajectory with a zero command
Using `ros2 topic hz` confirmed robot control at required frequency (50 Hz)

When trying TASK_SPACE, IKSolver seems to fail. This might be because of repulsive forces.
Planning with repulsive forces around the goal results in some pretty spiky behavior,
pushing the planning frame far away, the attraction bringing it back.

Bug in the repulsive force code??

## Demos
Screen Capture of RViz (Disable Robot Obstacles, Only Goal Pose)
GoPro Video
Save the CSV file/Plots and name them

### Demo 1: No Obstacles, WBV Planning to goal
Done

### Demo 2: WBV Planning to a goal near a obstacle's influence zone
Has Spiky repulsive behavior

### Demo 3: WBV Planning with several obstacles between start EE pose and goal EE Pose

### Demo 4: Teleop Demo combined with Task-Space Query
Done

### Demo 5 (Reach Goal): Simple Translation and Rotation to achieve goal pose
