// Copyright (c) 2021-2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include "far_planner/far_planner.h"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  auto far_planner_node = std::make_shared<FARMaster>();
  far_planner_node->Init();
  rclcpp::spin(far_planner_node->GetNodeHandle());

  rclcpp::shutdown();

  return 0;
}
