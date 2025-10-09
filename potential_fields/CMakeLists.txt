cmake_minimum_required(VERSION 3.8)
project(potential_fields)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

enable_testing()

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
find_package(fcl REQUIRED)


# Setup pfield_lib library
add_library(pfield_lib SHARED
  src/pfield/mesh_collision.cpp
  src/pfield/pf_obstacle.cpp
  src/pfield/pfield.cpp
  src/pfield/spatial_vector.cpp
)
target_link_libraries(pfield_lib
  fcl
)
ament_target_dependencies(pfield_lib
  Eigen3
  geometric_shapes
)
target_include_directories(pfield_lib PUBLIC
  include/potential_fields
)

# Use precompiled headers to speed up builds (CMake >= 3.16)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  target_precompile_headers(pfield_lib PRIVATE
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
add_executable(robot_parser src/ros/robot_parser.cpp)
add_executable(motion_interface src/ros/motion_interface.cpp)

# Library version of robot_parser (without main) for unit testing
add_library(robot_parser_lib STATIC src/ros/robot_parser.cpp)
target_compile_definitions(robot_parser_lib PRIVATE COMPILE_ROBOT_PARSER_NO_MAIN)
ament_target_dependencies(robot_parser_lib
  Eigen3
  geometry_msgs
  potential_fields_interfaces
  rclcpp
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)
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

ament_target_dependencies(robot_parser
  Eigen3
  geometry_msgs
  potential_fields_interfaces
  rclcpp
  tf2_eigen
  tf2_ros
  urdf
  visualization_msgs
)

ament_target_dependencies(motion_interface
  Eigen3
  geometry_msgs
  nav_msgs
  sensor_msgs
  potential_fields_interfaces
  rclcpp
  tf2_eigen
  tf2_ros
  trajectory_msgs
  urdf
  visualization_msgs
)

target_link_libraries(pfield_manager
  ${potential_fields_interfaces_LIBRARIES}
  pfield_lib
)

target_link_libraries(robot_parser
  ${potential_fields_interfaces_LIBRARIES}
)

target_link_libraries(motion_interface
  ${potential_fields_interfaces_LIBRARIES}
  pfield_lib
)

# Install the shared library so runtime linker can find it
install(
  TARGETS pfield_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS pfield_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS robot_parser
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS motion_interface
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

  # Add gtests only when explicitly building tests
  ament_add_gtest(test_spatial_vector
    test/test_spatial_vector.cpp
  )
  target_link_libraries(test_spatial_vector pfield_lib)

  ament_add_gtest(test_pfields
    test/test_pfields.cpp
  )
  target_link_libraries(test_pfields pfield_lib)

  ament_add_gtest(test_mesh_obstacle
    test/test_mesh_obstacle.cpp
  )
  target_link_libraries(test_mesh_obstacle pfield_lib)

  ament_add_gtest(test_robot_parser
    test/test_robot_parser.cpp
  )
  target_link_libraries(test_robot_parser pfield_lib robot_parser_lib)
  ament_target_dependencies(test_robot_parser
    Eigen3
    geometry_msgs
    potential_fields_interfaces
    rclcpp
    tf2_eigen
    tf2_ros
    urdf
    visualization_msgs
  )

  install(
    TARGETS test_spatial_vector
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS test_pfields
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS test_mesh_obstacle
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS test_robot_parser
    DESTINATION lib/${PROJECT_NAME}
  )

  find_package(ament_cmake_cpplint REQUIRED)
  # Runs cpplint over your C/C++ sources as a CTest
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
