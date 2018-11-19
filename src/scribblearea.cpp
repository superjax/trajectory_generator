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

#include "scribblearea.h"

using namespace Eigen;
using namespace std;

const double ScribbleArea::room_height_ = 10.21;
const double ScribbleArea::room_width_ = 7.16;

ScribbleArea::ScribbleArea(QWidget *parent)
  : QWidget(parent)
{
  setAttribute(Qt::WA_StaticContents);
  modified_ = false;
  scribbling_ = false;
  pen_width_ = 3;
  pen_color_ = Qt::cyan;
  empty_ = true;
  rough_trajectory_.clear();
  trajectory_altitude_ = 1.0;
}


//bool ScribbleArea::openImage(const QString &fileName)
//{
//    QImage loadedImage;
//    if (!loadedImage.load(fileName))
//        return false;

//    QSize newSize = loadedImage.size().expandedTo(size());
//    resizeImage(&loadedImage, newSize);
//    image_ = loadedImage;
//    modified_ = false;
//    update();
//    return true;
//}



//bool ScribbleArea::saveImage(const QString &fileName, const char *fileFormat)

//{
//    QImage visibleImage = image_;
//    resizeImage(&visibleImage, size());

//    if (visibleImage.save(fileName, fileFormat)) {
//        modified_ = false;
//        return true;
//    } else {
//        return false;
//    }
//}



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
  trajectory_altitude_ = alt;
}

void ScribbleArea::setVelocity(double vel)
{
  velocity_ = vel;
}

void ScribbleArea::addPoint(const QPoint &point)
{
  rough_trajectory_.push_back(Vector4d{((double)point.x()-midpixel_x_)/pixel_to_meters_,
                                       ((double)point.y()-midpixel_y_)/pixel_to_meters_,
                                       trajectory_altitude_,
                                       velocity_});
}


void ScribbleArea::mousePressEvent(QMouseEvent *event)
{
  if (event->button() == Qt::LeftButton)
  {
    clearImage();
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
    for (int i = 0; i < rough_trajectory_.size(); i++)
    {
      cout << rough_trajectory_[i].transpose() << endl;
    }

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
  cout << width() << ", " << height() << endl;
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
  smoother_ = new TrajectorySmoother(rough_trajectory_, 0.5, 100);

  MatrixXd optimized_full_traj_ = smoother_->optimize();

  smooth_traj_.resize(optimized_full_traj_.cols());
  for (int i = 0; i < optimized_full_traj_.cols(); i++)
  {
    smooth_traj_[i].segment<3>(0) = optimized_full_traj_.block<3,1>(0, i);
    smooth_traj_[i](3) = optimized_full_traj_.block<3,1>(7, i).norm();
  }

  for (int i = 0; i < smooth_traj_.size(); i++)
  {
    cout << smooth_traj_[i].transpose() << endl;
  }
}


void ScribbleArea::plotSmoothTrajectory()
{
  QColor downsampled_color(Qt::yellow);
  last_point_ = QPoint(smooth_traj_[0](0)*pixel_to_meters_ + midpixel_x_,
      smooth_traj_[0](1)*pixel_to_meters_ + midpixel_y_);
  first_point_ = last_point_;

  for (int i = 1; i < smooth_traj_.size(); i++)
  {
      QPoint new_point(smooth_traj_[i](0)*pixel_to_meters_ + midpixel_x_,
          smooth_traj_[i](1)*pixel_to_meters_ + midpixel_y_);
      drawLineTo(new_point, downsampled_color);
  }
  drawLineTo(first_point_, downsampled_color);
  update();
}
