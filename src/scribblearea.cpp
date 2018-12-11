/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtWidgets>
#include <QPrinter>
#include <QPrintDialog>
#include <QTimer>

#include "scribblearea.h"

using namespace Eigen;
using namespace std;

const double ScribbleArea::room_height_ = 10.21;
const double ScribbleArea::room_width_ = 7.16;

ScribbleArea::ScribbleArea(int argc, char **argv, QWidget *parent)
  : QWidget(parent)
{
  setAttribute(Qt::WA_StaticContents);
  modified_ = false;
  scribbling_ = false;
  pen_width_ = 3;
  pen_color_ = Qt::cyan;
  empty_ = true;
  rough_trajectory_.clear();
  trajectory_altitude_ = -1.0;
  initROS(argc, argv);
}

ScribbleArea::~ScribbleArea()
{
  if(ros_node_)
    delete ros_node_;
  if (smoother_)
    delete smoother_;
}

void ScribbleArea::timerEvent(QTimerEvent * ev)
{
  if (ev->timerId() == ros_node_timer_id_)
    ros::spinOnce();
  else if (ev->timerId() == publish_command_timer_id_)
    updateCommand();
}

void ScribbleArea::initROS(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros_node_ = new TrajOptROS();
  ros_node_timer_id_ = startTimer(1);
}


void ScribbleArea::setPenColor(const QColor &newColor)
{
  pen_color_ = newColor;
}



void ScribbleArea::setPenWidth(int newWidth)

{
  pen_width_ = newWidth;
}

void ScribbleArea::drawBackground()
{
  QPainter qPainter(&image_);
  qPainter.setBrush(Qt::NoBrush);
  qPainter.setPen(QPen(QColor(45, 45, 45), 20.0, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
  qPainter.drawRect(10,10,room_width_*pixel_to_meters_-10, room_height_*pixel_to_meters_-10);
  midpixel_x_ = room_width_*pixel_to_meters_/2;
  midpixel_y_ = room_height_*pixel_to_meters_/2;
  qPainter.setPen(QPen(Qt::red, 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  qPainter.drawLine(QPoint{midpixel_x_, midpixel_y_}, QPoint{midpixel_x_+(int)pixel_to_meters_, midpixel_y_});
  qPainter.setPen(QPen(Qt::green, 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  qPainter.drawLine(QPoint{midpixel_x_, midpixel_y_}, QPoint{midpixel_x_, midpixel_y_+(int)pixel_to_meters_});
  update();
}



void ScribbleArea::clearImage()

{
  image_.fill(qRgb(0, 0, 0));
  rough_trajectory_.clear();
  modified_ = true;
  empty_ = true;
  drawBackground();
  update();
}

void ScribbleArea::setAltitude(double alt)
{
  trajectory_altitude_ = -alt;
}

void ScribbleArea::setMaxVelocity(double vel)
{
  max_vel_ = vel;
}

void ScribbleArea::setMaxAccel(double acc)
{
  max_acc_ = acc;
}

void ScribbleArea::addPoint(const Vector3d &point)
{
  rough_trajectory_.push_back(point);
}

void ScribbleArea::addPoint(const QPoint &point)
{
  rough_trajectory_.push_back(Vector3d{((double)point.x()-midpixel_x_)/pixel_to_meters_,
                                       ((double)point.y()-midpixel_y_)/pixel_to_meters_,
                                       trajectory_altitude_});
}


void ScribbleArea::mousePressEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton)
  {
    clearImage();
    start_position_ = ros_node_->getCurrentPosition();
    addPoint(event->pos());
    first_point_ = event->pos();
    last_point_ = event->pos();
    scribbling_ = true;
  }
}

void ScribbleArea::mouseMoveEvent(QMouseEvent *event)
{
  if ((event->buttons() & Qt::LeftButton) && scribbling_)
  {
    addPoint(event->pos());
    drawLineTo(event->pos(), pen_color_);
  }
}

void ScribbleArea::mouseReleaseEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton && scribbling_)
  {
    addPoint(event->pos());
    drawLineTo(event->pos(), pen_color_);
    scribbling_ = false;
    drawLineTo(first_point_, pen_color_);
    smoothTrajectory();
    plotSmoothTrajectory();
  }
}


void ScribbleArea::paintEvent(QPaintEvent *event)

{
  QPainter painter(this);
  QRect dirtyRect = event->rect();
  painter.drawImage(dirtyRect, image_, dirtyRect);
}



void ScribbleArea::resizeEvent(QResizeEvent *event)

{
  resizeImage(&image_, QSize(width(), height()));
  double x_scale = (width()-10) / room_width_;
  double y_scale = (height()-10) / room_height_;
  pixel_to_meters_ = qMin(x_scale, y_scale);
  clearImage();
  drawBackground();
  update();
  QWidget::resizeEvent(event);
}



void ScribbleArea::drawLineTo(const QPoint &endPoint, QColor &pen_color)

{
  QPainter painter(&image_);
  painter.setPen(QPen(pen_color, pen_width_, Qt::SolidLine, Qt::RoundCap,
                      Qt::RoundJoin));
  painter.drawLine(last_point_, endPoint);
  modified_ = true;

  int rad = (pen_width_ / 2) + 2;
  update(QRect(last_point_, endPoint).normalized()
         .adjusted(-rad, -rad, +rad, +rad));
  last_point_ = endPoint;
}



void ScribbleArea::resizeImage(QImage *image, const QSize &newSize)

{
  if (image->size() == newSize)
    return;

  QImage newImage(newSize, QImage::Format_RGB32);
  newImage.fill(qRgb(255, 255, 255));
  QPainter painter(&newImage);
  painter.drawImage(QPoint(0, 0), *image);
  *image = newImage;
}



void ScribbleArea::print()
{
  QPrinter printer(QPrinter::HighResolution);

  QPrintDialog printDialog(&printer, this);

  if (printDialog.exec() == QDialog::Accepted) {
    QPainter painter(&printer);
    QRect rect = painter.viewport();
    QSize size = image_.size();
    size.scale(rect.size(), Qt::KeepAspectRatio);
    painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
    painter.setWindow(image_.rect());
    painter.drawImage(0, 0, image_);
  }
}

void ScribbleArea::smoothTrajectory()
{
  if (smoother_ != nullptr)
    delete smoother_;

  smoother_ = new TrajectorySmoother(rough_trajectory_, waypoint_distance_, sample_dt_);
  double wall_buffer = 1.0;
  double max_x = room_width_ - (double)midpixel_x_ / pixel_to_meters_ - wall_buffer;
  double min_x = max_x - room_width_ + 2.0*wall_buffer;
  double max_y = room_height_ - (double)midpixel_y_ / pixel_to_meters_ - wall_buffer;
  double min_y = max_y - room_height_ + 2.0*wall_buffer;
  smoother_->setBounds(max_x, min_x, max_y, min_y, max_vel_, max_acc_);

  smoother_->optimize(optimized_states_, optimized_inputs_);

  smooth_traj_.resize(optimized_states_.cols());
  for (int i = 0; i < optimized_states_.cols(); i++)
  {
    smooth_traj_[i] = optimized_states_.block<3,1>(0, i);
  }
}

void ScribbleArea::handleFlyButton()
{
  if (state_ == UNCOMMANDED)
  {
      cmd_idx_ = 0;
      state_ = FLY_TO_ALTITUDE;
      cout << "Fly to altitude" << endl;
      publish_command_timer_id_ = startTimer(1000 * sample_dt_);
      start_position_ = ros_node_->getCurrentPosition();
  }
}

void ScribbleArea::handleRTHButton()
{
  cout << "fly home" << endl;
  state_ = FLY_TO_HOME;
}

void ScribbleArea::updateState()
{
    if ((x_r_.segment<3>(LQR::POS) - ros_node_->getCurrentPosition()).norm() < 0.1)
    {
        switch (state_)
        {
        case FLY_TO_ALTITUDE:
            state_ = FLY_TO_START_OF_TRAJECTORY;
            cout << "fly to start" << endl;
            break;
        case FLY_TO_START_OF_TRAJECTORY:
            state_ = FLY_TRAJECTORY;
            cout << "fly trajectory" << endl;
            break;
        case FLY_TO_HOME:
            cout << "land" << endl;
            state_ = LAND;
            break;
        case LAND:
            cout << "done" << endl;
            state_ = UNCOMMANDED;
            break;
        case FLY_TRAJECTORY:
        case UNCOMMANDED:
        default:
            break;
        }
    }

    if (state_ == FLY_TRAJECTORY)
    {
        cmd_idx_++;
        if (cmd_idx_ == optimized_states_.cols())
          cmd_idx_ = 0;
    }
}

void ScribbleArea::updateCommand()
{
    updateState();
    x_r_ << 0, 0, 0,
            1, 0, 0, 0,
            0, 0, 0;
    u_r_ << 0, 0, 0, hover_throttle_;


    switch (state_)
    {
    case FLY_TO_ALTITUDE:
        x_r_.segment<3>(LQR::POS) << start_position_.x(),
                start_position_.y(),
                trajectory_altitude_;
        x_r_.segment<4>(LQR::ATT) = quat::Quatd::Identity().elements();
        x_r_.segment<3>(LQR::VEL) << 0, 0, 0;
        break;
    case FLY_TO_START_OF_TRAJECTORY:
        x_r_.segment<3>(LQR::POS) << optimized_states_(0,0),
                optimized_states_(1,0),
                optimized_states_(2,0);
        x_r_.segment<4>(LQR::ATT) = quat::Quatd::Identity().elements();
        x_r_.segment<3>(LQR::VEL) << 0, 0, 0;
        break;
    case FLY_TRAJECTORY:
        x_r_ = optimized_states_.col(cmd_idx_);
        u_r_ = optimized_inputs_.col(cmd_idx_);
        break;
    case FLY_TO_HOME:
        x_r_.segment<3>(LQR::POS) << start_position_.x(),
                start_position_.y(),
                trajectory_altitude_;
        x_r_.segment<4>(LQR::ATT) = quat::Quatd::Identity().elements();
        x_r_.segment<3>(LQR::VEL) << 0, 0, 0;
        break;
    case LAND:
        x_r_.segment<3>(LQR::POS) << start_position_;
        x_r_.segment<4>(LQR::ATT) = quat::Quatd::Identity().elements();
        x_r_.segment<3>(LQR::VEL) << 0, 0, 0;
        break;
    case UNCOMMANDED:
        x_r_.setConstant(NAN);
        u_r_.setConstant(NAN);
    default:
        break;
    }
    ros_node_->publishCommand(x_r_, u_r_);
}


void ScribbleArea::plotSmoothTrajectory()
{
  QColor downsampled_color(Qt::yellow);
  last_point_ = QPoint(smooth_traj_[0](0)*pixel_to_meters_ + midpixel_x_,
      smooth_traj_[0](1)*pixel_to_meters_ + midpixel_y_);

  for (int i = 1; i < smooth_traj_.size(); i++)
  {
    QPoint new_point(smooth_traj_[i](0)*pixel_to_meters_ + midpixel_x_,
                     smooth_traj_[i](1)*pixel_to_meters_ + midpixel_y_);
        drawLineTo(new_point, downsampled_color);
    if (i == 1);
    first_point_ = last_point_;
  }
  drawLineTo(first_point_, downsampled_color);
  update();
}
