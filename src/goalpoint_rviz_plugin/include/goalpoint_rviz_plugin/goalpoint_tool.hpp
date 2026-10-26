// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_
#define GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_


#include <QObject>

#include <sstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/tool.hpp>

namespace rviz_common
{
class DisplayContext;
namespace properties
{
class StringProperty;
class QosProfileProperty;
}  // namespace properties
}  // namespace rviz_common

namespace goalpoint_rviz_plugin
{
class GoalpointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalpointTool();

  ~GoalpointTool() override;

  void onInitialize() override;

protected:
  void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_joy_;

  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::QosProfileProperty * qos_profile_property_;

  rclcpp::QoS qos_profile_;
};
}  // namespace goalpoint_rviz_plugin


#endif  // GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_
