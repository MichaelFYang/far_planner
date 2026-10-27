cmake_minimum_required(VERSION 3.16)
project(far_planner)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui imgcodecs)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)

# Build
set(SOURCES
  src/dynamic_graph.cpp
  src/planner_visualizer.cpp
  src/utility.cpp
  src/graph_planner.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/graph_msger.cpp
  src/terrain_planner.cpp
)

add_library(far_planner_core STATIC src/far_planner.cpp ${SOURCES})
target_include_directories(far_planner_core PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(far_planner_core PUBLIC ${PCL_DEFINITIONS})

target_link_libraries(far_planner_core
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  visualization_msgs::visualization_msgs
  pcl_conversions::pcl_conversions
  visibility_graph_msg::visibility_graph_msg
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_link_libraries(far_planner_core Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} far_planner_core)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(terrain_search_benchmark EXCLUDE_FROM_ALL ../../benchmarks/terrain_search.cpp)
  target_link_libraries(terrain_search_benchmark far_planner_core)
  add_executable(terrain_neighbors_benchmark EXCLUDE_FROM_ALL ../../benchmarks/terrain_neighbors.cpp)
  target_link_libraries(terrain_neighbors_benchmark far_planner_core)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp test/test_sensor_timing.cpp test/test_local_terrain_callback.cpp test/test_viewpoint_extension.cpp test/test_overlap_cloud.cpp test/test_search.cpp test/test_graph_lifecycle.cpp test/test_graph_node_index.cpp test/test_graph_message.cpp test/test_map_handler.cpp test/test_contour_detector.cpp test/test_ros_context_cleanup.cpp)
  target_link_libraries(test_frame_transforms far_planner_core)
  ament_add_pytest_test(test_startup_shutdown test/test_startup_shutdown.py
    TIMEOUT 30
    ENV FAR_PLANNER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
  )
  ament_add_pytest_test(test_terrain_tf test/test_terrain_tf.py
    TIMEOUT 40
    ENV FAR_PLANNER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
  )
  ament_add_pytest_test(test_trajectory_sampling test/test_trajectory_sampling.py
    TIMEOUT 20
    ENV FAR_PLANNER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
  )
  ament_add_pytest_test(test_terrain_input_topics test/test_terrain_input_topics.py TIMEOUT 40)
  ament_add_pytest_test(test_dynamic_terrain_retention test/test_dynamic_terrain_retention.py
    TIMEOUT 25
    ENV FAR_PLANNER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
  )
endif()

ament_package()
