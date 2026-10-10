cmake_minimum_required(VERSION 3.18)
project(autoware_trajectory_controller LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(AUTOWARE_TC_BUILD_PYTHON "Build the Python extension module" ON)

include(cmake/Dependencies.cmake)

# -----------------------------------------------------------------------------
# Autoware sources: only the needed files, at pinned commits (scripts/fetch_upstream.py).
# -----------------------------------------------------------------------------
find_package(Python 3.9 REQUIRED COMPONENTS Interpreter)
set(TP ${CMAKE_BINARY_DIR}/third_party)
execute_process(
  COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fetch_upstream.py ${TP}
  RESULT_VARIABLE fetch_result
)
if(NOT fetch_result EQUAL 0)
  message(FATAL_ERROR "Failed to fetch the Autoware sources (scripts/fetch_upstream.py)")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/fetch_upstream.py)

set(UNIVERSE ${TP}/universe)
set(CORE ${TP}/core)
set(UTILS ${TP}/utils)

# Upstream sources are compiled unchanged.
# First-party sources: ROS 2 replacements and the ROS-free trajectory follower.
set(FIRST_PARTY_SOURCES
  shim/src/rclcpp_shim.cpp
  shim/src/autoware_utils_shim.cpp
  src/trajectory_follower.cpp
)

set(VENDORED_SOURCES
  # autoware_utils
  ${UTILS}/autoware_utils_geometry/src/geometry/geometry.cpp
  ${UTILS}/autoware_utils_geometry/src/geometry/pose_deviation.cpp
  ${UTILS}/autoware_utils_system/src/backtrace.cpp
  ${UTILS}/autoware_utils_visualization/src/marker_helper.cpp
  # autoware_core
  ${CORE}/autoware_interpolation/src/linear_interpolation.cpp
  ${CORE}/autoware_interpolation/src/spherical_linear_interpolation.cpp
  ${CORE}/autoware_interpolation/src/spline_interpolation.cpp
  ${CORE}/autoware_interpolation/src/spline_interpolation_points_2d.cpp
  ${CORE}/autoware_motion_utils/src/distance/distance.cpp
  ${CORE}/autoware_motion_utils/src/marker/marker_helper.cpp
  ${CORE}/autoware_motion_utils/src/marker/virtual_wall_marker_creator.cpp
  ${CORE}/autoware_motion_utils/src/resample/resample.cpp
  ${CORE}/autoware_motion_utils/src/trajectory/conversion.cpp
  ${CORE}/autoware_motion_utils/src/trajectory/interpolation.cpp
  ${CORE}/autoware_motion_utils/src/trajectory/path_with_lane_id.cpp
  ${CORE}/autoware_motion_utils/src/trajectory/trajectory.cpp
  ${CORE}/autoware_vehicle_info_utils/src/vehicle_info.cpp
  ${CORE}/autoware_vehicle_info_utils/src/vehicle_info_utils.cpp
  # autoware_universe: control
  ${UNIVERSE}/autoware_trajectory_follower_base/src/lateral_controller_base.cpp
  ${UNIVERSE}/autoware_trajectory_follower_base/src/longitudinal_controller_base.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/lowpass_filter.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/mpc.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/mpc_lateral_controller.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/mpc_trajectory.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/mpc_utils.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/qp_solver/qp_solver_unconstraint_fast.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/steering_predictor.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_interface.cpp
  ${UNIVERSE}/autoware_pid_longitudinal_controller/src/longitudinal_controller_utils.cpp
  ${UNIVERSE}/autoware_pid_longitudinal_controller/src/pid.cpp
  ${UNIVERSE}/autoware_pid_longitudinal_controller/src/pid_longitudinal_controller.cpp
  ${UNIVERSE}/autoware_pid_longitudinal_controller/src/pid_longitudinal_controller_node.cpp
  ${UNIVERSE}/autoware_pid_longitudinal_controller/src/smooth_stop.cpp
  ${UNIVERSE}/autoware_pure_pursuit/src/autoware_pure_pursuit_core/autoware_pure_pursuit.cpp
  ${UNIVERSE}/autoware_pure_pursuit/src/autoware_pure_pursuit_core/interpolate.cpp
  ${UNIVERSE}/autoware_pure_pursuit/src/autoware_pure_pursuit_core/planning_utils.cpp
  ${UNIVERSE}/autoware_pure_pursuit/src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp
  ${UNIVERSE}/autoware_pure_pursuit/src/autoware_pure_pursuit/autoware_pure_pursuit_viz.cpp
  # OSQP is always required: mpc_lateral_controller.cpp includes the OSQP solver header.
  ${CORE}/autoware_osqp_interface/src/csc_matrix_conv.cpp
  ${CORE}/autoware_osqp_interface/src/osqp_interface.cpp
  ${UNIVERSE}/autoware_mpc_lateral_controller/src/qp_solver/qp_solver_osqp.cpp
)

# Upstream code compiles unchanged; its warnings are not ours to fix.
set_source_files_properties(${VENDORED_SOURCES} PROPERTIES COMPILE_OPTIONS -w)

add_library(autoware_control_core STATIC ${FIRST_PARTY_SOURCES} ${VENDORED_SOURCES})
# The shim must come first (non-system dirs are searched before SYSTEM ones) so that it
# overrides ROS 2 / boost headers.
target_include_directories(autoware_control_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/shim/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_include_directories(autoware_control_core SYSTEM
  PUBLIC
    ${UTILS}/autoware_utils/include
    ${UTILS}/autoware_utils_geometry/include
    ${UTILS}/autoware_utils_math/include
    ${UTILS}/autoware_utils_rclcpp/include
    ${UTILS}/autoware_utils_system/include
    ${UTILS}/autoware_utils_visualization/include
    ${CORE}/autoware_interpolation/include
    ${CORE}/autoware_motion_utils/include
    ${CORE}/autoware_osqp_interface/include
    ${CORE}/autoware_vehicle_info_utils/include
    ${UNIVERSE}/autoware_trajectory_follower_base/include
    ${UNIVERSE}/autoware_mpc_lateral_controller/include
    ${UNIVERSE}/autoware_pid_longitudinal_controller/include
    ${UNIVERSE}/autoware_pure_pursuit/include
)
# Std headers the upstream headers expect to get transitively from ROS 2 (PUBLIC: every
# TU including them needs it, see shim/include/shim_prelude.hpp).
target_compile_options(autoware_control_core
  PUBLIC "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/shim/include/shim_prelude.hpp")
target_link_libraries(autoware_control_core PUBLIC Eigen3::Eigen)
target_include_directories(autoware_control_core SYSTEM PUBLIC ${OSQP_PREFIXED_INCLUDE_DIR})
target_link_libraries(autoware_control_core PUBLIC osqpstatic)

# -----------------------------------------------------------------------------
# Python module
# -----------------------------------------------------------------------------
if(AUTOWARE_TC_BUILD_PYTHON)
  find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)

  # Thin LTO: full LTO only adds link time here (the controllers live in a static library).
  pybind11_add_module(_core MODULE THIN_LTO src/bindings/module.cpp)
  target_link_libraries(_core PRIVATE autoware_control_core)

  set(PKG autoware_trajectory_controller)
  install(TARGETS _core LIBRARY DESTINATION ${PKG} COMPONENT python)
  # Default parameter files, taken from the upstream packages.
  install(
    FILES
      ${UNIVERSE}/autoware_trajectory_follower_node/param/trajectory_follower_node.param.yaml
      ${UNIVERSE}/autoware_trajectory_follower_node/param/lateral/mpc.param.yaml
      ${UNIVERSE}/autoware_trajectory_follower_node/param/lateral/pure_pursuit.param.yaml
      ${UNIVERSE}/autoware_trajectory_follower_node/param/longitudinal/pid.param.yaml
      ${CORE}/autoware_vehicle_info_utils/config/vehicle_info.param.yaml
    DESTINATION ${PKG}/config
    COMPONENT python
  )
  # The module contains Autoware code (Apache License 2.0): ship its license files.
  foreach(repo universe core utils)
    install(FILES ${TP}/${repo}/LICENSE DESTINATION ${PKG}/licenses
      RENAME LICENSE.autoware_${repo} COMPONENT python)
  endforeach()
endif()

# -----------------------------------------------------------------------------
# Upstream gtest suites, compiled unchanged against the shim (validation of the
# ROS-free build). Enable with -DAUTOWARE_TC_BUILD_TESTS=ON and run with ctest.
# -----------------------------------------------------------------------------
option(AUTOWARE_TC_BUILD_TESTS "Build the upstream C++ unit tests" OFF)
if(AUTOWARE_TC_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  include(GoogleTest)

  set(MPC_TEST_DIR ${UNIVERSE}/autoware_mpc_lateral_controller/test)
  set(PID_TEST_DIR ${UNIVERSE}/autoware_pid_longitudinal_controller/test)

  function(add_upstream_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE autoware_control_core GTest::gtest_main)
    target_compile_options(${name} PRIVATE -w)
    gtest_discover_tests(${name}
      PROPERTIES ENVIRONMENT "AUTOWARE_TC_SHARE_DIRS=${UNIVERSE}:${CORE}"
      DISCOVERY_TIMEOUT 60)
  endfunction()

  add_upstream_test(test_mpc_lateral_controller_core
    ${MPC_TEST_DIR}/test_mpc.cpp
    ${MPC_TEST_DIR}/test_mpc_utils.cpp
    ${MPC_TEST_DIR}/test_lowpass_filter.cpp)
  add_upstream_test(test_mpc_lateral_controller_node
    ${MPC_TEST_DIR}/test_mpc_lateral_controller.cpp)
  add_upstream_test(test_pid_longitudinal_controller
    ${PID_TEST_DIR}/test_pid.cpp
    ${PID_TEST_DIR}/test_smooth_stop.cpp
    ${PID_TEST_DIR}/test_longitudinal_controller_utils.cpp
    ${PID_TEST_DIR}/test_pid_longitudinal_controller.cpp)
endif()
