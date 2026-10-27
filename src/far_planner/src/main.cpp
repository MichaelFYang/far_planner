// Copyright (c) 2021-2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include "far_planner/far_planner.h"

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  try {
    auto far_planner_node = std::make_shared<FARMaster>();
    far_planner_node->Init();
    // A signal can shut down the context while Init() is still running.
    if (rclcpp::ok()) rclcpp::spin(far_planner_node->GetNodeHandle());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    if (!rclcpp::ok()) return 0;
    RCLCPP_FATAL(rclcpp::get_logger("far_planner"), "Startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
