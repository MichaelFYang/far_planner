# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(far_planner_benchmarks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(cloud_decay cloud_decay.cpp)
target_include_directories(cloud_decay PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(cloud_decay PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(cloud_decay PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cloud_decay PRIVATE ${PCL_LIBRARIES})

add_executable(grid_position grid_position.cpp)
target_include_directories(grid_position PRIVATE ../src/far_planner/include)
target_link_libraries(grid_position PRIVATE Eigen3::Eigen)

add_executable(contour_vote_rank contour_vote_rank.cpp)
target_include_directories(contour_vote_rank PRIVATE ../src/far_planner/include)

add_executable(contour_vote_fill contour_vote_fill.cpp)

add_executable(scan_grid_reset scan_grid_reset.cpp)
