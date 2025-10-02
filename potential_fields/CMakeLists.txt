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

# Setup testing files (moved under BUILD_TESTING below)

# Setup pfield_lib library
add_library(pfield_lib SHARED
  src/pfield/pf_obstacle.cpp
  src/pfield/pfield.cpp
  src/pfield/spatial_vector.cpp
)
target_include_directories(pfield_lib PUBLIC
  include/potential_fields
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(pfield_lib
  Eigen3
)

# Optional: use precompiled headers to speed up builds (CMake >= 3.16)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  target_precompile_headers(pfield_lib PRIVATE
    <Eigen/Core>
    <Eigen/Geometry>
    <vector>
    <string>
    <cmath>
  )
endif()

## Tests link against pfield_lib (see BUILD_TESTING section)

include_directories(
  include/potential_fields
  ${potential_fields_interfaces_INCLUDE_DIRS}
)

add_executable(pfield_manager src/ros/pfield_manager.cpp)
add_executable(robot_parser src/ros/robot_parser.cpp)
ament_target_dependencies(pfield_manager
  rclcpp
  Eigen3
  potential_fields_interfaces
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  trajectory_msgs
  tf2_ros
  tf2_eigen
  urdf
)

ament_target_dependencies(robot_parser
  rclcpp
  Eigen3
  potential_fields_interfaces
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  urdf
)

target_link_libraries(pfield_manager
  ${potential_fields_interfaces_LIBRARIES}
  pfield_lib
)

target_link_libraries(robot_parser
  ${potential_fields_interfaces_LIBRARIES}
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

  install(
    TARGETS test_spatial_vector
    DESTINATION lib/${PROJECT_NAME}
  )

  install(
    TARGETS test_pfields
    DESTINATION lib/${PROJECT_NAME}
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
