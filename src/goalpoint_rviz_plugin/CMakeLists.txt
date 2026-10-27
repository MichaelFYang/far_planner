cmake_minimum_required(VERSION 3.0.2)
project(goalpoint_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  sensor_msgs
  geometry_msgs
  nav_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

catkin_package(LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp rviz pluginlib sensor_msgs geometry_msgs nav_msgs)

add_library(${PROJECT_NAME} SHARED
  src/goalpoint_tool.cpp
  include/goalpoint_rviz_plugin/goalpoint_tool.hpp
)
target_compile_definitions(${PROJECT_NAME} PRIVATE QT_NO_KEYWORDS)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
