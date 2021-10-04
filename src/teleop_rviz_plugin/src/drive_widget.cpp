#include "drive_widget.h"

namespace teleop_rviz_plugin
{

DriveWidget::DriveWidget( QWidget* parent )
  : QWidget( parent )
  , linear_velocity_( 0 )
  , angular_velocity_( 0 )
  , linear_scale_( 1 )
  , angular_scale_( 1 )
  , x_mouse_( 0 )
  , y_mouse_( 0 )
  , mouse_pressed_(false)
{
}

void DriveWidget::paintEvent( QPaintEvent* event )
{
  QColor background;
  QColor crosshair;
  if( isEnabled() )
  {
    background = Qt::white;
    crosshair = Qt::black;
  }
  else
  {
    background = Qt::lightGray;
    crosshair = Qt::darkGray;
  }

  int w = width();
  int h = height();
  int size = (( w > h ) ? h : w) - 1;
  int hpad = ( w - size ) / 2;
  int vpad = ( h - size ) / 2;

  QPainter painter( this );
  painter.setBrush( background );
  painter.setPen( crosshair );

  painter.drawRect( QRect( hpad, vpad, size, size ));

  painter.drawLine( hpad, height() / 2, hpad + size, height() / 2 );
  painter.drawLine( width() / 2, vpad, width() / 2, vpad + size );

  if( isEnabled() && (angular_velocity_ != 0 || linear_velocity_ != 0 ))
  {
    QPen pen;
    pen.setWidth( size/150 );
    pen.setColor( Qt::darkRed );
    pen.setCapStyle( Qt::RoundCap );
    pen.setJoinStyle( Qt::RoundJoin );
    painter.setPen( pen );

    QPointF joystick[ 2 ];
    joystick[ 0 ].setX( w/2 );
    joystick[ 0 ].setY( h/2 );
    joystick[ 1 ].setX( x_mouse_ );
    joystick[ 1 ].setY( y_mouse_ );

    painter.drawPolyline( joystick, 2 );
    painter.drawEllipse( x_mouse_ - 10, y_mouse_ - 10, 20, 20 );
  }
}

void DriveWidget::mouseMoveEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::mousePressEvent( QMouseEvent* event )
{
  sendVelocitiesFromMouse( event->x(), event->y(), width(), height() );
}

void DriveWidget::leaveEvent( QEvent* event )
{
  stop();
}

void DriveWidget::mouseReleaseEvent( QMouseEvent* event )
{
  stop();
}

void DriveWidget::sendVelocitiesFromMouse( int x, int y, int width, int height )
{
  int size = (( width > height ) ? height : width );
  int hpad = ( width - size ) / 2;
  int vpad = ( height - size ) / 2;

  x_mouse_ = x;
  if ( x_mouse_ < width / 2 - size / 2 ) x_mouse_ = width / 2 - size / 2;
  else if ( x_mouse_ > width / 2 + size / 2 ) x_mouse_ = width / 2 + size / 2;
  y_mouse_ = y;
  if ( y_mouse_ < height / 2 - size / 2 ) y_mouse_ = height / 2 - size / 2;
  else if ( y_mouse_ > height / 2 + size / 2 ) y_mouse_ = height / 2 + size / 2;

  linear_velocity_ = (1.0 - float( y - vpad ) / float( size / 2 )) * linear_scale_;
  if ( linear_velocity_ < -1.0 ) linear_velocity_ = -1.0;
  else if ( linear_velocity_ > 1.0 ) linear_velocity_ = 1.0;
  if ( fabs( linear_velocity_ ) < 0.1 ) linear_velocity_ = 0;
  angular_velocity_ = ( 1.0 - float( x - hpad ) / float( size / 2 )) * angular_scale_;
  if ( angular_velocity_ < -1.0 ) angular_velocity_ = -1.0;
  else if ( angular_velocity_ > 1.0 ) angular_velocity_ = 1.0;

  mouse_pressed_ = true;

  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_, mouse_pressed_ );
  update();
}

void DriveWidget::stop()
{
  linear_velocity_ = 0;
  angular_velocity_ = 0;
  mouse_pressed_ = false;

  Q_EMIT outputVelocity( linear_velocity_, angular_velocity_, mouse_pressed_ );
  update();
}

}
