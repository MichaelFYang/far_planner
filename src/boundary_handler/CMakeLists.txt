cmake_minimum_required(VERSION 3.16)
project(boundary_handler)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/graph_extractor.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  std_msgs::std_msgs
  geometry_msgs::geometry_msgs
  visualization_msgs::visualization_msgs
  visibility_graph_msg::visibility_graph_msg
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_startup_shutdown test/test_startup_shutdown.py
    TIMEOUT 30
    ENV BOUNDARY_HANDLER_EXECUTABLE=$<TARGET_FILE:${PROJECT_NAME}>
        BOUNDARY_SAMPLE_DATA=${CMAKE_CURRENT_SOURCE_DIR}/../far_planner/data
  )
  ament_add_gtest(test_input_parser test/test_input_parser.cpp test/test_intersection.cpp)
  target_include_directories(test_input_parser PRIVATE include)
  target_link_libraries(test_input_parser visibility_graph_msg::visibility_graph_msg)
endif()

ament_package()
