cmake_minimum_required(VERSION 3.16)
project(graph_decoder)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/decoder_node.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  geometry_msgs::geometry_msgs
  visibility_graph_msg::visibility_graph_msg
  visualization_msgs::visualization_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_graph_file_parser test/test_graph_file_parser.cpp)
  target_include_directories(test_graph_file_parser PRIVATE include)
  target_compile_definitions(test_graph_file_parser PRIVATE
    FAR_GRAPH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../far_planner/data")
endif()

ament_package()
