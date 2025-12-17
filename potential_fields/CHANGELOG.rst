^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package potential_fields
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Copyright notices in code files
* Updating exclusion for CMakeLists
* Licenses in all packages
* Updating package.xml and CMakeLists with copyright testing
* Changing header doc for syntax hightlighting
* Updating XML files with author and license
* pfield_library -> potential_fields_library
* pfields_demo -> potential_fields_demos
* Enabling Github Workflow for building and testing repo (`#33 <https://github.com/argallab/pfields_2025/issues/33>`_)
* Successfully migrated all unit tests to `pfield_library` (`#32 <https://github.com/argallab/pfields_2025/issues/32>`_)
* December Final Demos (`#26 <https://github.com/argallab/pfields_2025/issues/26>`_)
* Use Robot Dynamics to convert Joint Torques to Joint Velocities (`#24 <https://github.com/argallab/pfields_2025/issues/24>`_)
* Independent pfield C++ Library (`#22 <https://github.com/argallab/pfields_2025/issues/22>`_)
* Updating ComputeAutonomyVector
* Collision Avoidance between robot links and environment obstacles (`#21 <https://github.com/argallab/pfields_2025/issues/21>`_)
* XArm 7-DoF Arm Demo (`#19 <https://github.com/argallab/pfields_2025/issues/19>`_)
* Dynamic D* Threshold (`#20 <https://github.com/argallab/pfields_2025/issues/20>`_)
* Aligned AttractiveMoment Doc with Implementation
* Fixing Rotational Attraction (`#18 <https://github.com/argallab/pfields_2025/issues/18>`_)
* Editing gains
* Fixing goal and query pose visualizations
* Fixing plotting functions and organizing code (`#16 <https://github.com/argallab/pfields_2025/issues/16>`_)
  Co-authored-by: Copilot <175728472+Copilot@users.noreply.github.com>
* Adding Reference to Principles of Robot Motion and Updating PF Equations (`#15 <https://github.com/argallab/pfields_2025/issues/15>`_)
* Updating repulsive force computation from Choset text
* Repulsive force piecewise equation includes equals to QStar
* Merged in extra fixes
* Comments cleanup
* Using RK4 for integrating position from PF Twist (`#13 <https://github.com/argallab/pfields_2025/issues/13>`_)
* Tested another demo with plots
* Docstrings for new functions
* First pass at RK4
* Fixing up tests
* Prototyping path planning (`#11 <https://github.com/argallab/pfields_2025/issues/11>`_)
* Refactored influence scale and mesh collision (`#12 <https://github.com/argallab/pfields_2025/issues/12>`_)
* Merge branch 'main' of github.com:argallab/pfields_2025
* Merged RobotParser into PFieldManager (`#10 <https://github.com/argallab/pfields_2025/issues/10>`_)
* Current TODO
* Pinocchio for Obstacles and Motion Constraints (`#9 <https://github.com/argallab/pfields_2025/issues/9>`_)
* PlanPath Service Demo (`#7 <https://github.com/argallab/pfields_2025/issues/7>`_)
* Planning "World" for path-planning with robot (`#6 <https://github.com/argallab/pfields_2025/issues/6>`_)
* Motion Interface (`#5 <https://github.com/argallab/pfields_2025/issues/5>`_)
* Update license to Apache 2.0
* Updating email to northwestern email for packages
* Updating functions to write to folders
* Documentation
* Updating robot parser targets
* typo
* Custom service to plan path in PF
* Merge pull request `#4 <https://github.com/argallab/pfields_2025/issues/4>`_ from argallab/demo-package
  Example Package
* Changing error to warn
* Launch arguments and parsing raw URDF
* Created robot parser node to listen for the transforms and parse the URDF collision objects
* Typo
* RViz sets fixed frame from user
* Moving static transforms to demo repository
* Updating build info
* Removing extra forward slash
* Another delta robot typo
* Fixing typo with delta robot include dir
* Created interpolate and integration functions for PField linear velocity and angular velocity
* A bit of hacking to get obstacles where the URDF defines them
* Created TF from world
* Attempting to fix TF listener and collision objects
* Fixed ID incrementing
* Fixed parsing problem but collision objects aren't showing up
* Created URDF testing file
* Adding urdf parser
* Updating the transforms of every obstacle
* Added path trajectory and tf at world frame
* Merge pull request `#2 <https://github.com/argallab/pfields_2025/issues/2>`_ from argallab/more-obstacles
  More Obstacle Support
* Moved rotated object to new position
* Fixed frequency in timer
* Removed the Lerp
* Updated Obstacle class, tests, and visualizer
* Added default case to withinInfluenceZone
* Converted SphereObstacle to PFObstacle class
* Cleaned up file comment
* documentation and normalizePosition
* Deleted default constructor for sphereObstacle to force data entry
* Followed NL.16 and removed extra public: statements
* Adjusting units and adding notes
* Deleted influence_scale_radius param since it's unused
* Changing query point size
* Visualize a query point
* Visualize a query point
* Visualizing the orientation of the goal and setting the goal pose with subscriber to /goal_pose
* Created an interfaces package
* Merge pull request `#1 <https://github.com/argallab/pfields_2025/issues/1>`_ from argallab/eigenify
  Implementing Vector operations with Eigen3
* Few more tests for angular distance
* Removed geodesic distance function and replaced with Eigen angularDistance function
* euler functions are now YPR and comments accordingly
* Added more tests and verified euler to quaternion method
* Collapsed implementation into header
* Fixed euler conversion
* All tests pass but need to write more
* Added normalizePosition function
* Fixed manager with new syntax and Eigen vectors
* Edited attraction and repulsion functions using Eigen and new SpatialVector formatting
* Changed tests to match new syntax
* Reimplemented position and orientation with eigen and removed operator overloads and changed floats to doubles
* Changed position to an Eigen::Vector3d and changed floats to doubles
* Adjusted language and changed floats to doubles
* Built PKG with Eigen dependency and with include statement
* Added rotational attractive gain and equation to README
* Wrote geodesic formulation in README and function as commented code
* Replaced geodesic distance with raw implementation inside Quaternion difference
* Wrote tests but orientation methods are failing
* Removing uncrustify tests
* Reverted formatting of uncrustify
* Reformated with uncrustify and created tests
* Attraction now includes orientation
* Vector3D -> SpatialVector
* Meeting notes
* Cleaning up docs in pfield vector markers
* Drew arrows in RViz
* Saved rviz config
* Included RViz and added config and launch folders/files
* Started writing node
* Edited math to match the python version that created graphs
* Updating force computation
* Docstrings
* Started working on implementation of C++ Potential Fields in a ROS2 pkg
* Contributors: Sharwin Patil, Sharwin24
