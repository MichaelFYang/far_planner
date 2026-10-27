# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.10)
project(far_planner_benchmarks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED NO_MODULE)

# Keep the optional build compatible with Melodic's CMake and PCL 1.8.
link_directories(${PCL_LIBRARY_DIRS})
add_executable(cloud_decay cloud_decay.cpp)
target_include_directories(cloud_decay PRIVATE ${PCL_INCLUDE_DIRS})
foreach(pcl_definition IN LISTS PCL_DEFINITIONS)
  string(STRIP "${pcl_definition}" pcl_definition)
  if(pcl_definition)
    target_compile_options(cloud_decay PRIVATE "${pcl_definition}")
  endif()
endforeach()
target_link_libraries(cloud_decay PRIVATE ${PCL_LIBRARIES})

add_executable(grid_position grid_position.cpp)
target_include_directories(grid_position PRIVATE ../src/far_planner/include)
target_link_libraries(grid_position PRIVATE Eigen3::Eigen)

add_executable(contour_vote_rank contour_vote_rank.cpp)
target_include_directories(contour_vote_rank PRIVATE ../src/far_planner/include)

add_executable(contour_vote_fill contour_vote_fill.cpp)

add_executable(scan_grid_reset scan_grid_reset.cpp)

add_executable(goal_grid_search goal_grid_search.cpp)
target_include_directories(goal_grid_search PRIVATE ../src/far_planner/include)
target_link_libraries(goal_grid_search PRIVATE Eigen3::Eigen)

add_executable(grid_inflation grid_inflation.cpp)
target_include_directories(grid_inflation PRIVATE ../src/far_planner/include)
target_link_libraries(grid_inflation PRIVATE Eigen3::Eigen)

add_executable(contour_compaction contour_compaction.cpp)
