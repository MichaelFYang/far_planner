// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "goalpoint_rviz_plugin/goalpoint_tool.hpp"
#include <rviz/frame_manager.h>
#include <cmath>
#include <exception>

namespace rviz
{
GoalPointTool::GoalPointTool()
{
  shortcut_key_ = 'w';

  topic_property_ = new StringProperty("Topic", "/goal_point",
      "The topic on which to publish goal waypoints for the dynamic route planner.",
                                       getPropertyContainer(), SLOT(updateTopic()), this);
}

void GoalPointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Goalpoint");
  sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &GoalPointTool::odomHandler,
      this);
  pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
  updateTopic();
}

void GoalPointTool::updateTopic()
{
  const std::string topic = topic_property_->getStdString();
  if (topic.empty()) {
    pub_.shutdown();
    ROS_WARN("Goal topic is empty");
    return;
  }
  try {
    pub_ = nh_.advertise<geometry_msgs::PointStamped>(topic, 5);
  } catch (const std::exception & error) {
    pub_.shutdown();
    ROS_ERROR("Cannot publish goal on '%s': %s", topic.c_str(), error.what());
  }
}

void GoalPointTool::odomHandler(const nav_msgs::Odometry::ConstPtr & odom)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  latest_odom_ = odom;
}

void GoalPointTool::onPoseSet(double x, double y, double /* theta */)
{
  if (!pub_) {
    return;
  }
  const std::string fixed_frame = context_->getFixedFrame().toStdString();
  if (fixed_frame.empty()) {
    ROS_WARN("Cannot set a goal without an RViz fixed frame");
    return;
  }
  nav_msgs::Odometry::ConstPtr odom;
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
          odom->header, odom->pose.pose, position, orientation)) {
        ROS_WARN("Cannot transform odometry into the RViz fixed frame for goal height");
        return;
      }
      goal_height = position.z;
    }
    if (!std::isfinite(goal_height)) {
      ROS_WARN("Cannot set a goal with invalid odometry height");
      return;
    }
  }
  sensor_msgs::Joy joy;

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

  const ros::Time click_stamp = ros::Time::now();
  joy.header.stamp = click_stamp;
  joy.header.frame_id = "goalpoint_tool";

  geometry_msgs::PointStamped goal_point;
  goal_point.header.frame_id = fixed_frame;
  goal_point.header.stamp = click_stamp;
  goal_point.point.x = x;
  goal_point.point.y = y;
  goal_point.point.z = goal_height;

  pub_.publish(goal_point);
  usleep(10000);
  pub_joy_.publish(joy);
}
}  // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::GoalPointTool, rviz::Tool)
