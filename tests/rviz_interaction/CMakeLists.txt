# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.10)
project(far_rviz_interaction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  set(CMAKE_CXX_STANDARD 14)
  find_package(catkin REQUIRED COMPONENTS
    roscpp rviz pluginlib tf2_ros nav_msgs sensor_msgs std_msgs geometry_msgs)
  add_executable(rviz_interaction test_rviz_interaction.cpp)
  target_compile_definitions(rviz_interaction PRIVATE FAR_UI_ROS1)
  target_include_directories(rviz_interaction PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(rviz_interaction ${catkin_LIBRARIES})
else()
  find_package(ament_cmake REQUIRED)
  set(ros_dependencies rclcpp pluginlib rviz_common rviz_rendering
    rviz_default_plugins tf2_ros nav_msgs sensor_msgs std_msgs geometry_msgs)
  foreach(dependency IN LISTS ros_dependencies)
    find_package(${dependency} REQUIRED)
  endforeach()
  add_executable(rviz_interaction test_rviz_interaction.cpp)
  if(COMMAND ament_target_dependencies)
    ament_target_dependencies(rviz_interaction ${ros_dependencies})
  else()
    # Lyrical exports imported targets instead of ament_target_dependencies.
    foreach(dependency IN LISTS ros_dependencies)
      target_link_libraries(rviz_interaction ${dependency}::${dependency})
    endforeach()
  endif()
endif()

if(TARGET Qt6::Widgets)
  set_property(TARGET rviz_interaction PROPERTY CXX_STANDARD 20)
  find_package(Qt6 REQUIRED COMPONENTS Widgets Test)
  target_link_libraries(rviz_interaction Qt6::Widgets Qt6::Test)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets Test)
  target_link_libraries(rviz_interaction Qt5::Widgets Qt5::Test)
endif()

find_program(FAR_UI_PYTHON NAMES python3)
if(NOT FAR_UI_PYTHON)
  message(FATAL_ERROR "Python 3 is required for the isolated test runner")
endif()
enable_testing()
add_test(NAME rviz_interaction COMMAND "${FAR_UI_PYTHON}"
  "${CMAKE_CURRENT_SOURCE_DIR}/run.py" "$<TARGET_FILE:rviz_interaction>"
  --output "${CMAKE_CURRENT_BINARY_DIR}/results")
set_tests_properties(rviz_interaction PROPERTIES TIMEOUT 150)
