// Copyright (c) 2021-2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include "far_planner/far_planner.h"

#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "far_planner_node");
  try {
    FARMaster dp_node;
    dp_node.Init();
    dp_node.Loop();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAR Planner startup failed: " << error.what() << '\n';
    ROS_FATAL("FAR Planner startup failed: %s", error.what());
    return 1;
  }
}
