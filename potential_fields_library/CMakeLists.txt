cmake_minimum_required(VERSION 3.8)
project(potential_fields_library)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED) # HPP-FCL renamed to coal

# Try to find urdfdom using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM REQUIRED urdfdom)
pkg_check_modules(URDFDOM_HEADERS REQUIRED urdfdom_headers)
pkg_check_modules(ASSIMP REQUIRED assimp)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/pfield/pfield.cpp
  src/pfield/pf_kinematics.cpp
  src/pfield/pf_obstacle.cpp
  src/pfield/mesh_collision.cpp
  src/pfield/spatial_vector.cpp
  src/solvers/weighted_ik.cpp
  src/solvers/geofik.cpp
  src/solvers/xarm_ik_solver.cpp
  # src/solvers/examples_common.cpp # Requires libfranka
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${URDFDOM_INCLUDE_DIRS}
  ${URDFDOM_HEADERS_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio
  coal::coal
  ${URDFDOM_LIBRARIES}
  ${ASSIMP_LIBRARIES}
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Create Config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION 1.0.0
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)

  # Helper function to add tests
  function(add_pfield_test test_name source_file)
    add_executable(${test_name} ${source_file})
    target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
    # Add include directory for mocks
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    # Set C++ standard to match library
    set_target_properties(${test_name} PROPERTIES CXX_STANDARD 20)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endfunction()

  add_pfield_test(test_pfields test/test_pfields.cpp)
  add_pfield_test(test_pf_obstacles test/test_pf_obstacles.cpp)
  add_pfield_test(test_spatial_vector test/test_spatial_vector.cpp)
  add_pfield_test(test_mesh_obstacle test/test_mesh_obstacle.cpp)

  # Add cpplint test using ament_cmake_cpplint
  # Excluding code that was written by external authors
  ament_cpplint(
    EXCLUDE
    cmake
    include/solvers/**
    src/solvers/**
  )

  # Add copyright test using ament_cmake_copyright
  ament_copyright(
    EXCLUDE
    cmake
    include/solvers/**
    src/solvers/**
  )

endif()
