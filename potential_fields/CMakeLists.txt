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
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(potential_fields_interfaces REQUIRED)
find_package(Doxygen)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

# Setup testing files
ament_add_gtest(test_spatial_vector
  test/test_spatial_vector.cpp
)
ament_add_gtest(test_pfields
  test/test_pfields.cpp
)

include_directories(
  include/potential_fields
  ${potential_fields_interfaces_INCLUDE_DIRS}
)

add_executable(pfield_manager src/pfield_manager.cpp)
add_executable(robot_parser src/robot_parser.cpp)
ament_target_dependencies(pfield_manager
  rclcpp
  Eigen3
  potential_fields_interfaces
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  urdf
)

ament_target_dependencies(robot_parser
  rclcpp
  Eigen3
  potential_fields_interfaces
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_ros
  tf2_eigen
  urdf
)

target_link_libraries(pfield_manager
  ${potential_fields_interfaces_LIBRARIES}
)

target_link_libraries(robot_parser
  ${potential_fields_interfaces_LIBRARIES}
)

install(
  TARGETS pfield_manager robot_parser
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
  TARGETS test_spatial_vector
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS test_pfields
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)

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
