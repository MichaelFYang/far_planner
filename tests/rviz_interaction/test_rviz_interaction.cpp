// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMainWindow>
#include <QLineEdit>
#include <QPushButton>
#include <QScreen>
#include <QStatusBar>
#include <QTest>
#include <QTimer>
#include <QToolBar>
#include <pluginlib/class_loader.hpp>
#ifdef FAR_UI_ROS1
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <rviz/display.h>
#include <rviz/panel.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/render_panel.h>
#include <rviz/tool.h>
#include <rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/string_property.h>
namespace rviz_common = rviz;
using StringProperty = rviz::StringProperty;
namespace geom = geometry_msgs;
namespace nav = nav_msgs;
namespace sensor = sensor_msgs;
namespace stdmsg = std_msgs;
constexpr auto rviz_package = "rviz";
constexpr auto panel_base = "rviz::Panel";
constexpr auto goal_class = "rviz/GoalPointTool";
constexpr auto panel_class = "rviz/ControlPanel";
constexpr auto grid_class = "rviz/Grid";
constexpr auto view_class = "rviz/TopDownOrtho";
constexpr auto camera_class = "rviz/MoveCamera";
using Node = ros::NodeHandle;
template<class Msg> using MessagePtr = typename Msg::ConstPtr;
template<class Msg, class Callback> auto subscribe(Node & node, const std::string & topic, int depth, Callback callback) {
  boost::function<void(const typename Msg::ConstPtr &)> wrapped = callback;
  return node.subscribe<Msg>(topic, depth, wrapped);
}
template<class Msg> auto publisher(Node & node, const std::string & topic, int depth) { return node.advertise<Msg>(topic, depth); }
template<class Msg> void publish(const ros::Publisher & pub, const Msg & message) { pub.publish(message); }
unsigned int subscribers(const ros::Publisher & pub) { return pub.getNumSubscribers(); }
unsigned int publishers(const ros::Subscriber & sub) { return sub.getNumPublishers(); }
auto clock_now() { return ros::Time::now(); }
auto duration(double seconds) { return ros::Duration(seconds); }
#else
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/panel_dock_widget.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_rendering/render_window.hpp>
using StringProperty = rviz_common::properties::StringProperty;
namespace geom = geometry_msgs::msg;
namespace nav = nav_msgs::msg;
namespace sensor = sensor_msgs::msg;
namespace stdmsg = std_msgs::msg;
constexpr auto rviz_package = "rviz_common";
constexpr auto panel_base = "rviz_common::Panel";
constexpr auto goal_class = "goalpoint_rviz_plugin/GoalpointTool";
constexpr auto panel_class = "teleop_rviz_plugin/TeleopPanel";
constexpr auto grid_class = "rviz_default_plugins/Grid";
constexpr auto view_class = "rviz_default_plugins/TopDownOrtho";
constexpr auto camera_class = "rviz_default_plugins/MoveCamera";
using Node = rclcpp::Node::SharedPtr;
template<class Msg> using MessagePtr = typename Msg::ConstSharedPtr;
template<class Msg, class Callback> auto subscribe(Node & node, const std::string & topic, int depth, Callback callback) {
  return node->create_subscription<Msg>(topic, depth, callback);
}
template<class Msg> auto publisher(Node & node, const std::string & topic, int depth) { return node->create_publisher<Msg>(topic, depth); }
template<class Publisher, class Msg> void publish(const Publisher & pub, const Msg & message) { pub->publish(message); }
template<class Publisher> auto subscribers(const Publisher & pub) { return pub->get_subscription_count(); }
template<class Subscriber> auto publishers(const Subscriber & sub) { return sub->get_publisher_count(); }
auto clock_now() { return rclcpp::Clock(RCL_ROS_TIME).now(); }
auto duration(double seconds) { return rclcpp::Duration::from_seconds(seconds); }
#endif
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#endif

using namespace std::chrono_literals;
void require(bool condition, const std::string & message) {
  if (!condition) throw std::runtime_error(message);
}
void write_report(const QDir & output, const QJsonObject & report) {
  QFile file(output.filePath("results.json"));
  require(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "Cannot open results.json");
  const auto bytes = QJsonDocument(report).toJson();
  require(file.write(bytes) == bytes.size(), "Cannot write results.json");
}

// Use the real RViz rendering/plugin managers without persisting a user's layout.
class Window : public QMainWindow, public rviz_common::WindowManagerInterface {
public:
  QWidget * getParentWindow() override { return this; }
  rviz_common::PanelDockWidget * addPane(const QString & name, QWidget * pane,
      Qt::DockWidgetArea area = Qt::LeftDockWidgetArea, bool floating = true) override {
    auto * dock = new rviz_common::PanelDockWidget(name);
    dock->setContentWidget(pane);
    addDockWidget(area, dock);
    dock->setFloating(floating);
    return dock;
  }
  void setStatus(const QString & message) override { statusBar()->showMessage(message); }
};

int main(int argc, char ** argv) {
  QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);
  QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
  QApplication app(argc, argv);
  if (argc != 2) {
    std::cerr << "Usage: rviz_interaction OUTPUT_DIRECTORY" << std::endl;
    return 2;
  }
  const QDir output(QDir(argv[1]).absolutePath());
  if (!output.mkpath(".")) {
    std::cerr << "Cannot create output directory" << std::endl;
    return 2;
  }
#ifdef FAR_UI_ROS1
  ros::init(argc, argv, "far_rviz_ui");
#else
  rclcpp::init(argc, argv);
#endif
  QJsonObject report;
  report["success"] = false;
  report["status"] = "running";
  report["ros_distro"] = QString::fromLocal8Bit(qgetenv("ROS_DISTRO"));
  report["qt_version"] = QString::fromLatin1(qVersion());
  try {
    write_report(output, report);
#ifdef FAR_UI_ROS1
    Node observer;
    auto spin = [] { ros::spinOnce(); };
#else
    auto observer = rclcpp::Node::make_shared("far_rviz_ui_observer");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(observer);
    auto spin = [&] { executor.spin_some(); };
#endif
    std::vector<geom::PointStamped> goals;
    std::vector<sensor::Joy> joys;
    std::vector<bool> attempts, updates;
    int resets = 0;
    std::vector<std::string> reads, saves;
    auto goal_sub = subscribe<geom::PointStamped>(observer, "/goal_point", 100,
      [&](MessagePtr<geom::PointStamped> m) {
        goals.push_back(*m);
        std::cout << "goal_event " << m->header.frame_id << ' ' << m->point.x << ' ' << m->point.y << ' ' << m->point.z << std::endl;
      });
    auto joy_sub = subscribe<sensor::Joy>(observer, "/joy", 100,
      [&](MessagePtr<sensor::Joy> m) { joys.push_back(*m); });
    auto attempt_sub = subscribe<stdmsg::Bool>(observer, "/planning_attemptable", 10,
      [&](MessagePtr<stdmsg::Bool> m) { attempts.push_back(m->data); });
    auto update_sub = subscribe<stdmsg::Bool>(observer, "/update_visibility_graph", 10,
      [&](MessagePtr<stdmsg::Bool> m) { updates.push_back(m->data); });
    auto reset_sub = subscribe<stdmsg::Empty>(observer, "/reset_visibility_graph", 10,
      [&](MessagePtr<stdmsg::Empty>) { ++resets; });
    auto read_sub = subscribe<stdmsg::String>(observer, "/read_file_dir", 10,
      [&](MessagePtr<stdmsg::String> m) { reads.push_back(m->data); });
    auto save_sub = subscribe<stdmsg::String>(observer, "/save_file_dir", 10,
      [&](MessagePtr<stdmsg::String> m) { saves.push_back(m->data); });
    auto odom_pub = publisher<nav::Odometry>(observer, "/state_estimation", 5);
#ifdef FAR_UI_ROS1
    tf2_ros::StaticTransformBroadcaster tf_pub;
    tf2_ros::TransformBroadcaster moving_tf_pub;
#else
    tf2_ros::StaticTransformBroadcaster tf_pub(observer);
    tf2_ros::TransformBroadcaster moving_tf_pub(observer);
#endif
    geom::TransformStamped tf;
    tf.header.frame_id = "map";
    tf.header.stamp = clock_now();
    tf.child_frame_id = "odom";
    tf.transform.translation.z = 2.0;
    tf.transform.rotation.w = 1.0;
    tf_pub.sendTransform(tf);
    auto pump = [&](int milliseconds) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
      do {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        spin();
        std::this_thread::sleep_for(5ms);
      } while (std::chrono::steady_clock::now() < deadline);
    };
    auto wait = [&](auto predicate, const std::string & message, int timeout = 10000) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
      while (!predicate() && std::chrono::steady_clock::now() < deadline) pump(10);
      require(predicate(), message);
    };
#ifndef FAR_UI_ROS1
    auto rviz_node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("far_rviz_ui");
#endif
    pluginlib::ClassLoader<rviz_common::Panel> panels(rviz_package, panel_base);
    Window window;
    window.setWindowTitle("FAR Planner — RViz interaction validation");
    window.resize(1280, 850);
    auto * render = new rviz_common::RenderPanel;
    window.setCentralWidget(render);
    window.show();
    pump(100);
#ifdef FAR_UI_ROS1
    auto manager = std::make_unique<rviz_common::VisualizationManager>(render, &window);
    render->initialize(manager->getSceneManager(), manager.get());
#else
    render->getRenderWindow()->initialize();
    auto manager = std::make_unique<rviz_common::VisualizationManager>(render, rviz_node, &window,
      rviz_node->get_raw_node()->get_clock());
    render->initialize(manager.get());
#endif
    manager->initialize();
    manager->setFixedFrame("map");
    manager->createDisplay(grid_class, "Grid", true);
    manager->getViewManager()->setCurrentViewControllerType(view_class);
    auto * tools = manager->getToolManager();
    tools->addTool(camera_class);
    auto * goal_tool = tools->addTool(goal_class);
    require(goal_tool != nullptr, "Goal tool did not load");
    auto * panel = panels.createUnmanagedInstance(panel_class);
    panel->initialize(manager.get());
    window.addPane("TeleopPanel", panel, Qt::RightDockWidgetArea, false);
    auto * toolbar = window.addToolBar("Tools");
    auto * goal_action = toolbar->addAction("Goalpoint");
    QObject::connect(goal_action, &QAction::triggered, [&] { tools->setCurrentTool(goal_tool); });
    manager->startUpdate();
    pump(1000);
    wait([&] {
      return subscribers(odom_pub) > 0 && publishers(goal_sub) > 0 &&
        publishers(joy_sub) > 0 && publishers(attempt_sub) > 0 && publishers(update_sub) > 0 &&
        publishers(reset_sub) > 0 && publishers(read_sub) > 0 && publishers(save_sub) > 0;
    }, "Plugin endpoints did not connect");
#ifdef FAR_UI_ROS1
    auto * canvas = render;
    wait([&] { return canvas->isVisible() && canvas->width() > 100; }, "Render panel is not visible");
#else
    auto * canvas = render->getRenderWindow();
    wait([&] { return canvas->isExposed() && canvas->width() > 100; }, "Render window is not exposed");
#endif
    auto click_goal = [&] {
      auto * button = toolbar->widgetForAction(goal_action);
      require(button != nullptr && button->isVisible(), "Goal toolbar button is not visible");
      QTest::mouseClick(button, Qt::LeftButton);
      pump(100);
      const QPoint center(canvas->width()/2, canvas->height()/2);
      QTest::mousePress(canvas, Qt::LeftButton, Qt::NoModifier, center);
      pump(50);
      QTest::mouseMove(canvas, center + QPoint(35, 0), 50);
      QTest::mouseRelease(canvas, Qt::LeftButton, Qt::NoModifier, center + QPoint(35, 0));
      pump(250);
    };
    click_goal();
    wait([&] { return goals.size() >= 1; }, "No goal from the rendered viewport");
    pump(500);
    require(goals.size() == 1, "A single click emitted duplicate goals");
    require(goals.back().header.frame_id == "map" && std::abs(goals.back().point.z) < 1e-6,
      "Initial goal frame/height is wrong");
    std::cout << "PASS initial goal and single publication" << std::endl;
    auto publish_odom = [&](const std::string & frame) {
      nav::Odometry odom;
      odom.header.frame_id = frame;
      odom.header.stamp = clock_now();
      odom.pose.pose.position.z = 1.25;
      odom.pose.pose.orientation.w = 1.0;
      for (int i = 0; i < 5; ++i) { publish(odom_pub, odom); pump(80); }
    };
    publish_odom("odom");
    click_goal();
    wait([&] { return goals.size() >= 2; }, "No transformed-height goal");
    require(goals.size() == 2 && goals.back().header.frame_id == "map" && std::abs(goals.back().point.z - 3.25) < 1e-5,
      "Goal height was not transformed into map");
    manager->setFixedFrame("odom");
    pump(300);
    click_goal();
    wait([&] { return goals.size() >= 3; }, "No odom-frame goal");
    require(goals.size() == 3 && goals.back().header.frame_id == "odom" && std::abs(goals.back().point.z - 1.25) < 1e-5,
      "Fixed-frame goal failed");
    publish_odom("missing_frame");
    click_goal();
    pump(500);
    require(goals.size() == 3, "Missing TF still published a goal");
    publish_odom("odom");
    click_goal();
    wait([&] { return goals.size() == 4; }, "Goal did not recover after missing TF");
    std::cout << "PASS transformed height, fixed frame, missing TF and recovery" << std::endl;
    auto * topic = qobject_cast<StringProperty *>(
      goal_tool->getPropertyContainer()->subProp("Topic"));
    require(topic != nullptr, "Goal topic property missing");
    topic->setStdString("/invalid topic");
    wait([&] { return publishers(goal_sub) == 0; }, "Invalid goal topic retained its publisher");
    click_goal();
    require(goals.size() == 4, "Invalid topic still published a goal");
    topic->setStdString("/goal_point");
    wait([&] { return publishers(goal_sub) > 0; }, "Restored goal publisher did not connect");
    click_goal();
    wait([&] { return goals.size() == 5; }, "Goal topic did not recover");
    manager->setFixedFrame("map");
    const auto first_time = clock_now() - duration(0.1);
    auto moving = tf;
    moving.child_frame_id = "moving_odom";
    moving.header.stamp = first_time;
    moving.transform.translation.z = 1.0;
    moving_tf_pub.sendTransform(moving);
    moving.header.stamp = first_time + duration(0.09);
    moving.transform.translation.z = 5.0;
    moving_tf_pub.sendTransform(moving);
    nav::Odometry measured;
    measured.header.frame_id = "moving_odom";
    measured.header.stamp = first_time;
    measured.pose.pose.position.z = 1.25;
    measured.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 5; ++i) { publish(odom_pub, measured); pump(80); }
    click_goal();
    wait([&] { return goals.size() == 6; }, "No measurement-time goal");
    require(std::abs(goals.back().point.z - 2.25) < 1e-5, "Goal height used latest TF instead of odometry-time TF");
    std::cout << "PASS measurement-time TF (2.25 m, latest would be 6.25 m)" << std::endl;
#ifndef FAR_UI_ROS1
    // RViz can switch to a paused simulation clock after its plugins load.
    auto clock_pub = observer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    auto rviz_raw_node = rviz_node->get_raw_node();
    require(rviz_raw_node->set_parameter(rclcpp::Parameter("use_sim_time", true)).successful,
      "RViz rejected use_sim_time");
    rosgraph_msgs::msg::Clock simulated_clock;
    simulated_clock.clock.sec = 200;
    for (int i = 0; i < 20; ++i) { clock_pub->publish(simulated_clock); pump(30); }
    wait([&] { return rviz_raw_node->now().nanoseconds() == 200000000000LL; },
      "RViz did not receive the simulation clock");
    nav::Odometry simulated_odom;
    simulated_odom.header.frame_id = "map";
    simulated_odom.header.stamp = simulated_clock.clock;
    simulated_odom.pose.pose.position.z = 1.25;
    simulated_odom.pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 5; ++i) { publish(odom_pub, simulated_odom); pump(80); }
    click_goal();
    wait([&] { return goals.size() == 7; }, "No goal after switching RViz to simulated time");
    require(goals.back().header.stamp == simulated_clock.clock, "Goal ignored RViz simulated time");
#endif
    std::vector<sensor::Joy> goal_joys;
    for (const auto & joy : joys) if (joy.header.frame_id == "goalpoint_tool") goal_joys.push_back(joy);
    require(goal_joys.size() == goals.size(), "Goal/resume command count differs");
    for (std::size_t i = 0; i < goals.size(); ++i) {
      const auto & goal = goals[i];
      require(std::isfinite(goal.point.x) && std::isfinite(goal.point.y) && std::isfinite(goal.point.z), "Nonfinite goal");
      require(goal.header.stamp == goal_joys[i].header.stamp, "Goal and resume stamps differ");
      require(goal_joys[i].axes.size() == 8 && goal_joys[i].buttons.size() == 11 &&
        goal_joys[i].axes[2] == -1 && goal_joys[i].buttons[7] == 1, "Invalid goal resume payload");
    }
    auto find_button = [&](const QString & text) {
      for (auto * button : panel->findChildren<QPushButton *>()) if (button->text() == text) return button;
      throw std::runtime_error("Panel button missing: " + text.toStdString());
    };
    auto find_checkbox = [&](const QString & text) {
      for (auto * checkbox : panel->findChildren<QCheckBox *>()) if (checkbox->text() == text) return checkbox;
      throw std::runtime_error("Panel checkbox missing: " + text.toStdString());
    };
    QTest::mouseClick(find_checkbox("Planning Attemptable"), Qt::LeftButton);
    wait([&] { return attempts.size() == 1; }, "Planning toggle missing");
    QTest::mouseClick(find_checkbox("Planning Attemptable"), Qt::LeftButton);
    wait([&] { return attempts.size() == 2; }, "Planning toggle recovery missing");
    require(!attempts[0] && attempts[1], "Planning toggle values wrong");
    QTest::mouseClick(find_checkbox("Update Visibility Graph"), Qt::LeftButton);
    wait([&] { return updates.size() == 1; }, "Update toggle missing");
    QTest::mouseClick(find_checkbox("Update Visibility Graph"), Qt::LeftButton);
    wait([&] { return updates.size() == 2; }, "Update toggle recovery missing");
    require(!updates[0] && updates[1], "Update toggle values wrong");
    QTest::mouseClick(find_button("Reset Visibility Graph"), Qt::LeftButton);
    wait([&] { return resets == 1; }, "Reset button missing");
    auto panel_resumes = [&] {
      int result = 0;
      for (const auto & joy : joys) if (joy.header.frame_id == "teleop_panel" && joy.axes.size() == 8 && joy.axes[2] == -1) ++result;
      return result;
    };
    QTest::mouseClick(find_button("Resume Navigation to Goal"), Qt::LeftButton);
    wait([&] { return panel_resumes() == 1; }, "Resume button missing");
    std::cout << "PASS panel toggles, reset and resume" << std::endl;
    int dialogs_seen = 0;
    std::string dialog_error;
    auto arrange_dialog = [&](const QString & path, bool accept) {
      auto * timer = new QTimer(&window);
      timer->setInterval(40);
      QObject::connect(timer, &QTimer::timeout, &window, [&, timer, path, accept] {
        for (auto * widget : QApplication::topLevelWidgets()) {
          auto * dialog = qobject_cast<QFileDialog *>(widget);
          if (!dialog || !dialog->isVisible()) continue;
          ++dialogs_seen;
          timer->stop();
          timer->deleteLater();
          if (accept) {
            dialog->setDirectory(output.absolutePath());
            // QFileDialog loads its directory asynchronously before selecting a name.
            QTimer::singleShot(400, dialog, [&, dialog, path] {
              auto * edit = dialog->findChild<QLineEdit *>("fileNameEdit");
              if (!edit) {
                dialog_error = "File-name editor missing";
                dialog->reject();
                return;
              }
              edit->setFocus();
              edit->clear();
              edit->setText(path);
              QTimer::singleShot(100, dialog, [dialog] { QMetaObject::invokeMethod(dialog, "accept", Qt::DirectConnection); });
            });
          } else {
            dialog->reject();
          }
          return;
        }
      });
      timer->start();
    };
    const std::string fixture = output.filePath("existing.vgh").toStdString();
    std::ofstream(fixture) << "UI file selection fixture\n";
    arrange_dialog(QString::fromStdString(fixture), true);
    QTest::mouseClick(find_button("Read"), Qt::LeftButton);
    wait([&] { return reads.size() == 1; }, "Read selection missing");
    require(reads[0] == fixture, "Read path changed");
    arrange_dialog("", false);
    QTest::mouseClick(find_button("Read"), Qt::LeftButton);
    pump(250);
    require(reads.size() == 1, "Cancelled read published a path");
    arrange_dialog(output.filePath("saved_graph"), true);
    QTest::mouseClick(find_button("Save"), Qt::LeftButton);
    wait([&] { return saves.size() == 1; }, "Save selection missing");
    require(saves[0] == output.filePath("saved_graph.vgh").toStdString(), "Save suffix/path wrong");
    arrange_dialog(output.filePath("with_suffix.vgh"), true);
    QTest::mouseClick(find_button("Save"), Qt::LeftButton);
    wait([&] { return saves.size() == 2; }, "Save with extension missing");
    require(saves[1] == output.filePath("with_suffix.vgh").toStdString(), "Save duplicated existing suffix");
    arrange_dialog("", false);
    QTest::mouseClick(find_button("Save"), Qt::LeftButton);
    pump(250);
    require(dialog_error.empty(), dialog_error);
    require(saves.size() == 2 && dialogs_seen == 5, "Cancelled save published a path or dialog missing");
    std::cout << "PASS read/save dialogs, extension and cancellation" << std::endl;
    QWidget * drive = nullptr;
    for (auto * widget : panel->findChildren<QWidget *>()) {
      if (QString(widget->metaObject()->className()).endsWith("DriveWidget")) drive = widget;
    }
    require(drive != nullptr, "Drive widget missing");
    auto manual_joys = [&] {
      std::vector<sensor::Joy> result;
      for (const auto & joy : joys) if (joy.header.frame_id == "teleop_panel" && joy.axes.size() == 8 && joy.axes[2] == 1) result.push_back(joy);
      return result;
    };
    const QPoint drive_point(drive->width()/4, drive->height()/4);
    QTest::mouseMove(drive, drive_point);
    pump(100);
    QTest::mousePress(drive, Qt::LeftButton, Qt::NoModifier, drive_point);
    wait([&] { auto m = manual_joys(); return !m.empty() && m.back().axes[3] > .2f && m.back().axes[4] > .2f; }, "Joystick press did not command motion");
    pump(200);
    require(QGuiApplication::primaryScreen()->grabWindow(window.winId()).save(output.filePath("teleop-active.png")),
      "Active joystick screenshot failed");
    QTest::mouseRelease(drive, Qt::LeftButton, Qt::NoModifier, drive_point);
    wait([&] { auto m = manual_joys(); return !m.empty() && m.back().axes[3] == 0 && m.back().axes[4] == 0; }, "Joystick release did not stop");
    pump(250);
    const auto stopped_count = manual_joys().size();
    pump(350);
    require(manual_joys().size() == stopped_count, "Idle joystick kept publishing");
    QTest::mousePress(drive, Qt::LeftButton, Qt::NoModifier, drive_point);
    wait([&] { return manual_joys().size() > stopped_count; }, "Second joystick press missing");
    drive->setFixedSize(1, 1);
    pump(100);
    QTest::mousePress(drive, Qt::LeftButton, Qt::NoModifier, QPoint(0, 0));
    QTest::mouseRelease(drive, Qt::LeftButton, Qt::NoModifier, QPoint(0, 0));
    wait([&] { auto m = manual_joys(); return m.back().axes[3] == 0 && m.back().axes[4] == 0; }, "Undersized joystick did not stop");
    drive->setMinimumSize(0, 0);
    drive->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    pump(150);
    for (const auto & joy : joys) for (float value : joy.axes) require(std::isfinite(value), "Nonfinite joystick axis");
    std::cout << "PASS joystick motion, release, idle and undersized stop" << std::endl;
#ifndef FAR_UI_ROS1
    for (const auto & joy : joys) {
      if (joy.header.frame_id == "teleop_panel") {
        require(joy.header.stamp == simulated_clock.clock,
          "Panel command timestamp ignored RViz simulated time");
      }
    }
    report["sim_time_checked"] = true;
    std::cout << "PASS RViz simulation clock for goals, resume and joystick commands" << std::endl;
#else
    report["sim_time_checked"] = false;
#endif
    report["toggle_messages"] = static_cast<int>(attempts.size() + updates.size());
    report["reset_messages"] = resets;
    report["panel_resumes"] = panel_resumes();
    report["read_messages"] = static_cast<int>(reads.size());
    report["save_messages"] = static_cast<int>(saves.size());
    report["dialogs_seen"] = dialogs_seen;
    report["manual_joy_messages"] = static_cast<int>(manual_joys().size());
    report["goal_events"] = static_cast<int>(goals.size());
    report["success"] = true;
    report["status"] = "complete";
    require(QGuiApplication::primaryScreen()->grabWindow(window.winId()).save(output.filePath("rviz.png")), "Screenshot failed");
    manager->stopUpdate();
    manager.reset();
  } catch (const std::exception & error) {
    report["success"] = false;
    report["status"] = "failed";
    report["error"] = error.what();
    std::cerr << "FAIL " << error.what() << std::endl;
  }
  write_report(output, report);
#ifdef FAR_UI_ROS1
  ros::shutdown();
#else
  rclcpp::shutdown();
#endif
  return report["success"].toBool() ? 0 : 1;
}
