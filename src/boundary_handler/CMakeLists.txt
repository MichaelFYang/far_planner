cmake_minimum_required(VERSION 3.5)
project(boundary_handler)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/include
  /usr/include
)

link_directories(
  /usr/local/lib
  /usr/lib
)

set(SOURCES
  # Add your source files here if any
)

add_executable(${PROJECT_NAME} src/graph_extractor.cpp ${SOURCES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  visibility_graph_msg
)

# Make sure to link against the PCL libraries and any other libraries you need
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()