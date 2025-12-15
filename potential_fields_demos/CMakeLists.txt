cmake_minimum_required(VERSION 3.8)
project(potential_fields_demo)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pfield_library REQUIRED)
find_package(potential_fields_interfaces REQUIRED)
find_package(Doxygen)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include/potential_fields_demo
  ${potential_fields_interfaces_INCLUDE_DIRS}
)

# Create an executable for the C++ demo node
add_executable(pfield_demo_node src/pfield_demo.cpp)
ament_target_dependencies(pfield_demo_node
  rclcpp
  potential_fields_interfaces
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  urdf
  Eigen3
  std_srvs
  std_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
)

target_link_libraries(pfield_demo_node pfield_library::pfield_library)

add_executable(pfield_teleop_demo_node src/pfield_teleop_demo.cpp)
ament_target_dependencies(pfield_teleop_demo_node
  rclcpp
  potential_fields_interfaces
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  urdf
  Eigen3
  std_srvs
  std_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  ament_index_cpp
)

target_link_libraries(pfield_teleop_demo_node pfield_library::pfield_library)

# Install the Cpp demo node
install(
  TARGETS pfield_demo_node pfield_teleop_demo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python demo node
install(
  PROGRAMS
  potential_fields_demo/pf_demo.py
  potential_fields_demo/sphere_robot_demo.py
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
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()
