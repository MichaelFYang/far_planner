// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "teleop_rviz_plugin/teleop_panel.hpp"

#include <string>
#include <rclcpp/time.hpp>
#include <rviz_common/display_context.hpp>

#include "teleop_rviz_plugin/drive_widget.hpp"

namespace teleop_rviz_plugin
{

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
  , linear_velocity_(0)
  , angular_velocity_(0)
  , mouse_pressed_(false)
  , mouse_pressed_sent_(false)
{
  node_ = rclcpp::Node::make_shared("teleop_panel_node");
  clock_ = node_->get_clock();

  QVBoxLayout * layout = new QVBoxLayout;
  check_box_1_ = new QCheckBox("Planning Attemptable", this);
  check_box_1_->setCheckState(Qt::Checked);
  layout->addWidget(check_box_1_);
  check_box_2_ = new QCheckBox("Update Visibility Graph", this);
  check_box_2_->setCheckState(Qt::Checked);
  layout->addWidget(check_box_2_);
  push_button_1_ = new QPushButton("Reset Visibility Graph", this);
  layout->addWidget(push_button_1_);
  push_button_2_ = new QPushButton("Resume Navigation to Goal", this);
  layout->addWidget(push_button_2_);
  push_button_3_ = new QPushButton("Read", this);
  layout->addWidget(push_button_3_);
  push_button_4_ = new QPushButton("Save", this);
  layout->addWidget(push_button_4_);
  drive_widget_ = new DriveWidget;
  layout->addWidget(drive_widget_);
  setLayout(layout);

  QTimer * output_timer = new QTimer(this);

  connect(push_button_1_, &QPushButton::pressed, this, &TeleopPanel::pressButton1);
  connect(push_button_2_, &QPushButton::pressed, this, &TeleopPanel::pressButton2);
  connect(push_button_3_, &QPushButton::pressed, this, &TeleopPanel::pressButton3);
  connect(push_button_4_, &QPushButton::pressed, this, &TeleopPanel::pressButton4);
  connect(check_box_1_, &QCheckBox::toggled, this, &TeleopPanel::clickBox1);
  connect(check_box_2_, &QCheckBox::toggled, this, &TeleopPanel::clickBox2);
  connect(drive_widget_, &DriveWidget::outputVelocity, this, &TeleopPanel::setVel);
  connect(output_timer, &QTimer::timeout, this, &TeleopPanel::sendVel);

  output_timer->start(100);

  velocity_publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);
  attemptable_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/planning_attemptable", 5);
  update_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/update_visibility_graph", 5);
  reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("/reset_visibility_graph", 5);
  read_publisher_ = node_->create_publisher<std_msgs::msg::String>("/read_file_dir", 5);
  save_publisher_ = node_->create_publisher<std_msgs::msg::String>("/save_file_dir", 5);


  drive_widget_->setEnabled(true);
}

void TeleopPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  clock_ = getDisplayContext()->getClock();
}

void TeleopPanel::pressButton1()
{
  if (rclcpp::ok() && reset_publisher_->get_subscription_count() > 0) {
    std_msgs::msg::Empty msg;
    reset_publisher_->publish(msg);
  }
}

void TeleopPanel::pressButton2()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0) {
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
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_->publish(joy);
  }
}

void TeleopPanel::pressButton3()
{
  if (rclcpp::ok() && read_publisher_->get_subscription_count() > 0) {
    QString qFilename = QFileDialog::getOpenFileName(this, tr("Read File"), "/",
        tr("VGH - Visibility Graph Files (*.vgh)"));
    std::string filename = qFilename.toStdString();
    if (filename.empty()) {return;}
    std_msgs::msg::String msg;
    msg.data = filename;
    read_publisher_->publish(msg);
  }
}

void TeleopPanel::pressButton4()
{
  if (rclcpp::ok() && save_publisher_->get_subscription_count() > 0) {
    QString qFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "/",
        tr("VGH - Visibility Graph Files (*.vgh)"));

    std::string filename = qFilename.toStdString();
    if (filename.empty()) {return;}
    if (filename.size() < 4 || filename.compare(filename.size() - 4, 4, ".vgh") != 0) {
      filename += ".vgh";
    }
    std_msgs::msg::String msg;
    msg.data = filename;
    save_publisher_->publish(msg);
  }
}

void TeleopPanel::clickBox1(int val)
{
  if (rclcpp::ok() && attemptable_publisher_->get_subscription_count() > 0) {
    std_msgs::msg::Bool msg;
    msg.data = static_cast<bool>(val);
    attemptable_publisher_->publish(msg);
  }
}

void TeleopPanel::clickBox2(int val)
{
  if (rclcpp::ok() && update_publisher_->get_subscription_count() > 0) {
    std_msgs::msg::Bool msg;
    msg.data = static_cast<bool>(val);
    update_publisher_->publish(msg);
  }
}

void TeleopPanel::setVel(float lin, float ang, bool pre)
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
  mouse_pressed_ = pre;
}

void TeleopPanel::sendVel()
{
  if (rclcpp::ok() && velocity_publisher_->get_subscription_count() > 0 &&
    (mouse_pressed_ || mouse_pressed_sent_))
  {
    sensor_msgs::msg::Joy joy;

    joy.axes.push_back(0);
    joy.axes.push_back(0);
    joy.axes.push_back(1.0);
    joy.axes.push_back(angular_velocity_);
    joy.axes.push_back(linear_velocity_);
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
    joy.header.frame_id = "teleop_panel";
    velocity_publisher_->publish(joy);

    mouse_pressed_sent_ = mouse_pressed_;
  }
}

void TeleopPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void TeleopPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

}  // namespace teleop_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(teleop_rviz_plugin::TeleopPanel, rviz_common::Panel)
