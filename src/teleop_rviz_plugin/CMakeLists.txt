cmake_minimum_required(VERSION 3.16)
project(teleop_rviz_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(HDR_FILES
  include/teleop_rviz_plugin/drive_widget.hpp
  include/teleop_rviz_plugin/teleop_panel.hpp
)

set(SRC_FILES
  src/drive_widget.cpp
  src/teleop_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${HDR_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  "pluginlib"
  "rclcpp"
  "rviz_common"
  "rviz_rendering"
  "sensor_msgs"
  "rviz_default_plugins"
  "rviz2"
  "std_msgs"
)

target_link_libraries(${PROJECT_NAME} PUBLIC Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  sensor_msgs
  rviz_default_plugins
  rviz2
  std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
