// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#ifndef GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_
#define GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_

#include <sstream>
#include <mutex>
#include <ros/ros.h>
#include <QObject>

#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/default_plugin/tools/pose_tool.h"

namespace rviz
{
class StringProperty;

class GoalPointTool : public PoseTool
{
  Q_OBJECT

public:
  GoalPointTool();
  virtual ~GoalPointTool()
  {
  }
  virtual void onInitialize();

protected:
  virtual void odomHandler(const nav_msgs::Odometry::ConstPtr & odom);
  virtual void onPoseSet(double x, double y, double theta);

private Q_SLOTS:
  void updateTopic();

private:
  std::mutex odom_mutex_;
  nav_msgs::Odometry::ConstPtr latest_odom_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher pub_joy_;

  StringProperty * topic_property_;
};
}  // namespace rviz

#endif  // GOALPOINT_RVIZ_PLUGIN__GOALPOINT_TOOL_HPP_
