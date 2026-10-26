// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace {

class RosContextEnvironment final : public ::testing::Environment {
public:
    void TearDown() override {
        // Stop discovery threads before process-level ROS objects are destroyed.
        if (rclcpp::ok()) rclcpp::shutdown();
        EXPECT_FALSE(rclcpp::ok());
    }
};

struct RegisterRosContextEnvironment {
    RegisterRosContextEnvironment() {
        ::testing::AddGlobalTestEnvironment(new RosContextEnvironment);
    }
};

RegisterRosContextEnvironment registered_ros_context_environment;

}  // namespace
