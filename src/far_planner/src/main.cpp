// Copyright (c) 2021-2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include "far_planner/far_planner.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "far_planner_node");
  FARMaster dp_node;
  dp_node.Init();
  dp_node.Loop();
}
