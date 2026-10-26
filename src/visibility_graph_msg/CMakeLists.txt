cmake_minimum_required(VERSION 3.0.2)
project(visibility_graph_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Node.msg
  Graph.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
