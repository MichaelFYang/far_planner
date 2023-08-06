cmake_minimum_required(VERSION 3.5)
project(teleop_rviz_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

set(CMAKE_AUTOMOC ON)

set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES
  src/drive_widget.cpp
  src/teleop_panel.cpp
)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rviz2
  rviz_common
  Qt5
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
