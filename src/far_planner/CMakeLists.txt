cmake_minimum_required(VERSION 3.0.2)
project(far_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  visibility_graph_msg
  pcl_ros
  pcl_conversions
  tf
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  visibility_graph_msg
  pcl_ros
  pcl_conversions
  tf
)

add_library(far_planner_core STATIC
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/dynamic_graph.cpp
  src/far_planner.cpp
  src/graph_msger.cpp
  src/graph_planner.cpp
  src/map_handler.cpp
  src/planner_visualizer.cpp
  src/scan_handler.cpp
  src/terrain_planner.cpp
  src/utility.cpp
)
add_dependencies(far_planner_core ${catkin_EXPORTED_TARGETS})
target_include_directories(far_planner_core PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(far_planner_core ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} far_planner_core)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frame_transforms test/test_frame_transforms.cpp test/test_sensor_timing.cpp)
  if(TARGET test_frame_transforms)
    target_compile_definitions(test_frame_transforms PRIVATE FAR_PLANNER_ROS1)
    target_link_libraries(test_frame_transforms far_planner_core)
  endif()
endif()
