// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "goalpoint_rviz_plugin/goalpoint_tool.hpp"

#include <string>
#include <cmath>
#include <exception>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/qos_profile_property.hpp>

namespace goalpoint_rviz_plugin
{
GoalpointTool::GoalpointTool()
: rviz_default_plugins::tools::PoseTool(), qos_profile_(5)
{
  shortcut_key_ = 'w';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/goal_point",
    "The topic on which to publish navigation waypoints.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

GoalpointTool::~GoalpointTool() = default;

void GoalpointTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {
      this->qos_profile_ = profile;
      this->updateTopic();
    });
  setName("Goalpoint");
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  sub_ = raw_node->template create_subscription<nav_msgs::msg::Odometry>(
    "/state_estimation", 5,
    std::bind(&GoalpointTool::odomHandler, this, std::placeholders::_1));
  pub_joy_ = raw_node->template create_publisher<sensor_msgs::msg::Joy>("/joy", qos_profile_);
  clock_ = raw_node->get_clock();
  updateTopic();
}

void GoalpointTool::updateTopic()
{
  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) {
    pub_.reset();
    RCLCPP_WARN(rclcpp::get_logger("goalpoint_rviz_plugin"), "Goal topic is empty");
    return;
  }
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  try {
    pub_ =
      raw_node->template create_publisher<geometry_msgs::msg::PointStamped>(topic, qos_profile_);
  } catch (const std::exception & error) {
    pub_.reset();
    RCLCPP_ERROR(
      rclcpp::get_logger("goalpoint_rviz_plugin"),
      "Cannot publish goal on '%s': %s", topic.c_str(), error.what());
  }
}

void GoalpointTool::odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = odom;
}

void GoalpointTool::onPoseSet(double x, double y, double /* theta */)
{
  if (!pub_) {
    return;
  }
  const std::string fixed_frame = context_->getFixedFrame().toStdString();
  if (fixed_frame.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("goalpoint_rviz_plugin"),
      "Cannot set a goal without an RViz fixed frame");
    return;
  }
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = latest_odom_;
  }
  double goal_height = 0.0;
  if (odom) {
    if (odom->header.frame_id == fixed_frame) {
      goal_height = odom->pose.pose.position.z;
    } else {
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if (!context_->getFrameManager()->transform(
          odom->header, odom->pose.pose, position, orientation))
      {
        RCLCPP_WARN(
          rclcpp::get_logger("goalpoint_rviz_plugin"),
          "Cannot transform odometry into the RViz fixed frame for goal height");
        return;
      }
      goal_height = position.z;
    }
    if (!std::isfinite(goal_height)) {
      RCLCPP_WARN(
        rclcpp::get_logger("goalpoint_rviz_plugin"),
        "Cannot set a goal with invalid odometry height");
      return;
    }
  }
  sensor_msgs::msg::Joy joy;

  joy.axes.push_back(0);
  joy.axes.push_back(0);
  joy.axes.push_back(-1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(1.0);
  joy.axes.push_back(0);
  joy.axes.push_back(0);

  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(1);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);
  joy.buttons.push_back(0);

  joy.header.stamp = clock_->now();
  joy.header.frame_id = "goalpoint_tool";
  pub_joy_->publish(joy);

  geometry_msgs::msg::PointStamped goalpoint;
  goalpoint.header.frame_id = fixed_frame;
  goalpoint.header.stamp = joy.header.stamp;
  goalpoint.point.x = x;
  goalpoint.point.y = y;
  goalpoint.point.z = goal_height;

  pub_->publish(goalpoint);
  usleep(10000);
  pub_->publish(goalpoint);
}
}  // namespace goalpoint_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(goalpoint_rviz_plugin::GoalpointTool, rviz_common::Tool)
