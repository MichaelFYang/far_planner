cmake_minimum_required(VERSION 2.8.3)
project(goalpoint_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
)

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
 message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
 include(${QT_USE_FILE})
else()
 message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
 find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
 set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz
)

include_directories(
 include
  ${QT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(HEADER_FILES
        include/goalpoint_tool.h
)
set(SRC_FILES
    src/goalpoint_tool.cpp
)

add_library(${PROJECT_NAME}
            ${SRC_FILES} ${HEADER_FILES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OGRE_OV_LIBRARIES_ABS})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
