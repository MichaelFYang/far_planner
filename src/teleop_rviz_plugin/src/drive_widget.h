#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H

#include <stdio.h>
#include <math.h>

#include <QPainter>
#include <QMouseEvent>

#include <QWidget>

namespace teleop_rviz_plugin
{

class DriveWidget: public QWidget
{
Q_OBJECT
public:
  DriveWidget( QWidget* parent = 0 );
  virtual void paintEvent( QPaintEvent* event );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void leaveEvent( QEvent* event );
  virtual QSize sizeHint() const { return QSize( 150, 150 ); }

Q_SIGNALS:
  void outputVelocity( float linear, float angular, bool pressed );

protected:
  void sendVelocitiesFromMouse( int x, int y, int width, int height );
  void stop();

  float linear_velocity_;
  float angular_velocity_;
  float linear_scale_;
  float angular_scale_;
  float x_mouse_, y_mouse_;
  bool mouse_pressed_;
};

}

#endif // DRIVE_WIDGET_H
