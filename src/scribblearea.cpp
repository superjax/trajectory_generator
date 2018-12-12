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

ScribbleArea::ScribbleArea(QWidget *parent)
  : QWidget(parent)
{
  setAttribute(Qt::WA_StaticContents);
  pen_width_ = 3;
  pen_color_ = Qt::cyan;
  trajectory_altitude_ = -1.0;
  clearImage();
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
  last_point_.setX(5e8);
  last_point_.setY(5e8);
  locked_screen_ = false;
  drawBackground();
  update();
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
  if (event->button() == Qt::LeftButton && !locked_screen_)
  {
    addPoint(event->pos());
    drawLineTo(event->pos(), pen_color_, true);
  }
}

void ScribbleArea::mouseMoveEvent(QMouseEvent *event)
{
//  if ((event->buttons() & Qt::LeftButton) && scribbling_)
//  {
//    addPoint(event->pos());
//    drawLineTo(event->pos(), pen_color_);
//  }
}

void ScribbleArea::mouseReleaseEvent(QMouseEvent *event)
{
//  if (event->button() == Qt::LeftButton && scribbling_)
//  {
//    addPoint(event->pos());
//    drawLineTo(event->pos(), pen_color_);
//    scribbling_ = false;
//    drawLineTo(first_point_, pen_color_);
//    smoothTrajectory();
//    plotSmoothTrajectory();
//  }
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



void ScribbleArea::drawLineTo(const QPoint &endPoint, QColor &pen_color, bool draw_point)

{
  QPainter painter(&image_);
  painter.setPen(QPen(pen_color, pen_width_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  if (last_point_.x() < 1e8 && last_point_.y() < 1e8)
  {
      painter.drawLine(last_point_, endPoint);
  }
  int rad = 5;
  if (draw_point)
  {
    painter.setBrush(pen_color);
    painter.drawEllipse(endPoint, rad, rad);
  }
  update();
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

void ScribbleArea::plotSmoothTrajectory(const trajVec& smooth_traj)
{
  QColor downsampled_color(Qt::yellow);
  last_point_ = QPoint(smooth_traj[0](0)*pixel_to_meters_ + midpixel_x_,
                       smooth_traj[0](1)*pixel_to_meters_ + midpixel_y_);
  QPoint first_point = last_point_;
  for (int i = 1; i < smooth_traj.size(); i++)
  {
    QPoint new_point(smooth_traj[i](0)*pixel_to_meters_ + midpixel_x_,
                     smooth_traj[i](1)*pixel_to_meters_ + midpixel_y_);
        drawLineTo(new_point, downsampled_color);
  }
  drawLineTo(first_point, downsampled_color);
  update();
  locked_screen_ = true;
}


