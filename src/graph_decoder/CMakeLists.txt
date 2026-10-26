cmake_minimum_required(VERSION 3.0.2)
project(graph_decoder)

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
  geometry_msgs
  visualization_msgs
  visibility_graph_msg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  visibility_graph_msg
)

add_executable(${PROJECT_NAME}
  src/decoder_node.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_graph_file_parser test/test_graph_file_parser.cpp)
  if(TARGET test_graph_file_parser)
    target_compile_definitions(test_graph_file_parser PRIVATE FAR_PLANNER_ROS1)
    target_include_directories(test_graph_file_parser PRIVATE include ${catkin_INCLUDE_DIRS})
    target_compile_definitions(test_graph_file_parser PRIVATE
      FAR_GRAPH_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../far_planner/data")
  endif()

  catkin_add_gtest(test_graph_message_validation test/test_graph_message_validation.cpp)
  if(TARGET test_graph_message_validation)
    target_compile_definitions(test_graph_message_validation PRIVATE FAR_PLANNER_ROS1)
    add_dependencies(test_graph_message_validation ${catkin_EXPORTED_TARGETS})
    target_include_directories(test_graph_message_validation PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(test_graph_message_validation ${catkin_LIBRARIES})
  endif()

  catkin_install_python(PROGRAMS test/test_decoder_callback_ros1.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/test_decoder_callback_ros1.test)
endif()
