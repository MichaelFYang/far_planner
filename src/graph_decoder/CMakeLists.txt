cmake_minimum_required(VERSION 3.5)
project(graph_decoder)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Other CMake configurations
set(CMAKE_BUILD_TYPE Release)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visibility_graph_msg REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

# Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

# Dependent cpp files
set(SOURCES ${SOURCES}
  src/decoder_node.cpp
)

# Declare executables
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  visibility_graph_msg
  pcl_conversions
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

##########
## Test ##
##########
# If you have tests
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

# This macro ensures modules and global scripts declared therein get installed
ament_package()