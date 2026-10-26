# Copyright (c) 2026 Fan Yang and contributors
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.16)
project(far_cloud_decay_benchmark LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL REQUIRED COMPONENTS common filters)

add_executable(cloud_decay cloud_decay.cpp)
target_include_directories(cloud_decay PRIVATE ${PCL_INCLUDE_DIRS})
target_link_directories(cloud_decay PRIVATE ${PCL_LIBRARY_DIRS})
target_compile_options(cloud_decay PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(cloud_decay PRIVATE ${PCL_LIBRARIES})
