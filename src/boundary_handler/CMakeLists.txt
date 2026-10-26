cmake_minimum_required(VERSION 3.0.2)
project(boundary_handler)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  visibility_graph_msg
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  visibility_graph_msg
)

add_executable(${PROJECT_NAME}
  src/graph_extractor.cpp
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
  catkin_add_gtest(test_input_parser test/test_input_parser.cpp test/test_intersection.cpp)
  if(TARGET test_input_parser)
    target_compile_definitions(test_input_parser PRIVATE FAR_PLANNER_ROS1)
    target_include_directories(test_input_parser PRIVATE include)
  endif()
endif()
