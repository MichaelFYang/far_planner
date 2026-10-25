cmake_minimum_required(VERSION 3.5)
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
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/decoder_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  std_msgs::std_msgs
  std_srvs::std_srvs
  geometry_msgs::geometry_msgs
  visibility_graph_msg::visibility_graph_msg
  pcl_conversions::pcl_conversions
  visualization_msgs::visualization_msgs
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
