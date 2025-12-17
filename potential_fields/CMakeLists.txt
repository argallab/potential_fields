cmake_minimum_required(VERSION 3.8)
project(potential_fields)

# Ensure installed executables can find external libraries in non-standard locations
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

enable_testing()

# set(FRANKA_INSTALL_PATH "$ENV{HOME}/ws/franka/install" CACHE PATH "Path to libfranka installation directory")
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${FRANKA_INSTALL_PATH}")
# set(USING_FRANKA) # Toggle to include Franka code

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(potential_fields_interfaces REQUIRED)
find_package(Doxygen)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(hpp-fcl REQUIRED)
find_package(pinocchio REQUIRED)
# find_package(Franka REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Look in the default user install location if not found in system paths
find_package(potential_fields_library REQUIRED)


# Setup robot_plugins library
add_library(robot_plugins SHARED
  src/robot_plugins/examples_common.cpp
  src/robot_plugins/franka_plugin.cpp
  src/robot_plugins/xarm_plugin.cpp
)
ament_target_dependencies(robot_plugins
  Eigen3
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
  urdf
  # Franka
)
target_link_libraries(robot_plugins potential_fields_library::potential_fields_library)
target_include_directories(robot_plugins PUBLIC
  include/potential_fields/robot_plugins
)

if(Franka_FOUND)
  target_link_libraries(robot_plugins
    Franka::Franka
  )
else()
  message(WARNING "Franka not found, building without it.")
endif()

# Use precompiled headers to speed up builds (CMake >= 3.16)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  target_precompile_headers(robot_plugins PRIVATE
    <Eigen/Dense>
    <Eigen/Core>
    <Eigen/Geometry>
    <vector>
    <string>
    <cmath>
    <memory>
  )
endif()

include_directories(
  include/potential_fields
  ${potential_fields_interfaces_INCLUDE_DIRS}
)

add_executable(pfield_manager src/ros/pfield_manager.cpp)

ament_target_dependencies(pfield_manager
  Eigen3
  geometry_msgs
  nav_msgs
  potential_fields_interfaces
  rclcpp
  sensor_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
  urdf
  visualization_msgs
)

target_link_libraries(pfield_manager
  ${potential_fields_interfaces_LIBRARIES}
  potential_fields_library::potential_fields_library
  robot_plugins
)

# Install the robot_plugins shared library so it is available at runtime
install(
  TARGETS robot_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS pfield_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

## Test targets are installed only when BUILD_TESTING is enabled

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  # Add gtests only when explicitly building tests
  ament_add_gtest(test_visualization_profiling
    test/test_visualization_profiling.cpp
  )
  target_link_libraries(test_visualization_profiling potential_fields_library::potential_fields_library)
  ament_target_dependencies(test_visualization_profiling rclcpp)
  install(
    TARGETS test_visualization_profiling
    DESTINATION lib/${PROJECT_NAME}
  )

  # Runs cpplint over C/C++ sources as a CTest
  ament_cpplint(
    EXCLUDE
    include/potential_fields/robot_plugins/examples_common.h
    include/potential_fields/robot_plugins/geofik.h
    include/potential_fields/robot_plugins/weighted_ik.h
    src/robot_plugins/geofik.cpp
    src/robot_plugins/examples_common.cpp
    src/robot_plugins/weighted_ik.cpp
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)

  # ament_lint_auto_find_test_dependencies()

  # Run Copyright checks
  ament_copyright()
endif()

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(DOXYGEN_FOUND AND BUILD_DOCS)
  # Turn the README.md into the homepage of the doxygen docs
  set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

  # Tell Doxygen where to find the documentation
  doxygen_add_docs(doxygen include/ src/ README.md ALL)

  # The documentation will be in the build/html directory
  # The main page is build/html/index.html
endif()

ament_package()
