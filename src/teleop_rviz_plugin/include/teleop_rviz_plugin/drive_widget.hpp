// Copyright (c) 2021-2026 Fan Yang and contributors
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_HPP_
#define TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_HPP_

#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include <QWidget>

namespace teleop_rviz_plugin
{

class DriveWidget : public QWidget
{
  Q_OBJECT

public:
  explicit DriveWidget(QWidget * parent = nullptr);
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void leaveEvent(QEvent * event) override;
  QSize sizeHint() const override {return QSize(150, 150);}

Q_SIGNALS:
  void outputVelocity(float linear, float angular, bool pressed);

protected:
  void sendVelocitiesFromMouse(int x, int y, int width, int height);
  void stop();

  float linear_velocity_;
  float angular_velocity_;
  float linear_scale_;
  float angular_scale_;
  float x_mouse_, y_mouse_;
  bool mouse_pressed_;
};

}  // namespace teleop_rviz_plugin

#endif  // TELEOP_RVIZ_PLUGIN__DRIVE_WIDGET_HPP_
