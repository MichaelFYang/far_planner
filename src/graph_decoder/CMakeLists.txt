cmake_minimum_required(VERSION 3.16)
project(graph_decoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  visibility_graph_msg
  visualization_msgs
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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_graph_file_parser test/test_graph_file_parser.cpp)
  ament_target_dependencies(test_graph_file_parser visibility_graph_msg)
  target_include_directories(test_graph_file_parser PRIVATE include)
  target_compile_definitions(test_graph_file_parser PRIVATE
    FAR_GRAPH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../far_planner/data")

  ament_add_gtest(test_graph_message_validation test/test_graph_message_validation.cpp)
  ament_target_dependencies(test_graph_message_validation visibility_graph_msg)

  ament_add_pytest_test(test_decoder_callback test/test_decoder_callback.py
    TIMEOUT 35
    ENV GRAPH_DECODER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
  )
endif()

ament_package()
