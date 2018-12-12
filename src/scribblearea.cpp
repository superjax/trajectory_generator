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
#include "mainwindow.h"

using namespace Eigen;
using namespace std;

const double ScribbleArea::room_height_ = 10.21;
const double ScribbleArea::room_width_ = 7.16;

ScribbleArea::ScribbleArea(MainWindow *parent)
    : QWidget((QWidget*)parent)
{
    setAttribute(Qt::WA_StaticContents);
    pen_width_ = 3;
    trajectory_altitude_ = -1;
    clearImage();
    setMouseTracking(true);
    parent_ = parent;
}


void ScribbleArea::setPenWidth(int newWidth)

{
    pen_width_ = newWidth;
}

void ScribbleArea::drawBackground()
{
    QPainter qPainter(&image_);
    qPainter.setBrush(Qt::NoBrush);
    int buffer_width_pixel = 1.0*pixel_to_meters_;
    qPainter.setPen(QPen(QColor(80, 45, 45), buffer_width_pixel, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
    qPainter.drawRect(buffer_width_pixel/2.0,buffer_width_pixel/2.0,room_width_*pixel_to_meters_-buffer_width_pixel, room_height_*pixel_to_meters_-buffer_width_pixel);
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

void ScribbleArea::clearTrajectory()
{
    rough_trajectory_.clear();
    clearImage();
}

void ScribbleArea::clearImage()
{
    image_.fill(qRgb(0, 0, 0));
    last_point_.setX(5e8);
    last_point_.setY(5e8);
    locked_screen_ = false;
    drawBackground();
    update();
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
        if (hovered_point_idx_ == -1)
        {
            addPoint(event->pos());
            drawPoint(last_point_, pen_color_);
            drawLineTo(event->pos(), pen_color_, true);
            drawPoint(event->pos(), selected_color_);
            selected_point_idx_ = rough_trajectory_.size() - 1;
        }
        else
        {
            selected_point_idx_ = hovered_point_idx_;
            movePoint(event->pos(), hovered_point_idx_);
            parent_->setAltValue(rough_trajectory_[selected_point_idx_](2));
        }
    }
}

void ScribbleArea::drawPosition(const Vector3d &position, const Vector3d &desired_position)
{
    QPoint pixel(position.x() * pixel_to_meters_ + midpixel_x_,
                 position.y() * pixel_to_meters_ + midpixel_y_);
    QPoint pixel_d(desired_position.x() * pixel_to_meters_ + midpixel_x_,
                   desired_position.y() * pixel_to_meters_ + midpixel_y_);
    clearImage();
    drawSmoothTraj();
    drawRoughTraj();
    drawPoint(pixel, position_color_);
    drawPoint(pixel_d, smooth_traj_color_);
}

void ScribbleArea::deletePoint()
{
    rough_trajectory_.erase(rough_trajectory_.begin() +selected_point_idx_);
    if (selected_point_idx_ == rough_trajectory_.size())
        selected_point_idx_--;
    clearImage();
    drawRoughTraj();
}

void ScribbleArea::setAltitude(double alt)
{
    trajectory_altitude_ = -alt;
    if (selected_point_idx_ >= 0)
        rough_trajectory_[selected_point_idx_](2) = -alt;
}

void ScribbleArea::movePoint(const QPoint &point, int id)
{
    rough_trajectory_[id] = Vector3d{((double)point.x()-midpixel_x_)/pixel_to_meters_,
            ((double)point.y()-midpixel_y_)/pixel_to_meters_,
            rough_trajectory_[id].z()};
    clearImage();
    drawRoughTraj();
    point_clicked_ = true;
}

void ScribbleArea::mouseMoveEvent(QMouseEvent *event)
{
    if (point_clicked_)
    {
        movePoint(event->pos(), hovered_point_idx_);
    }
    else
    {
        Vector2d event_m = Vector2d{(event->pos().x() - midpixel_x_)/pixel_to_meters_,
                (event->pos().y() - midpixel_y_)/pixel_to_meters_};
        const double dist_m = 5/pixel_to_meters_;
        int prev_selected_point = hovered_point_idx_;
        hovered_point_idx_ = -1;
        for (int i = 0; i < rough_trajectory_.size(); i++)
        {
            if ((event_m - rough_trajectory_[i].segment<2>(0)).norm() < dist_m)
            {
                drawPoint(rough_trajectory_[i].segment<2>(0), selected_color_);
                hovered_point_idx_ = i;
            }
        }
        if (hovered_point_idx_ != prev_selected_point && hovered_point_idx_ == -1)
        {
            if ( selected_point_idx_ != prev_selected_point)
            {
                drawPoint(rough_trajectory_[prev_selected_point].segment<2>(0), pen_color_);
            }
        }
    }
}

void ScribbleArea::drawPoint(const Vector2d &pt, const QColor& color)
{
    QPoint pixel(pt.x() * pixel_to_meters_ + midpixel_x_,
                 pt.y() * pixel_to_meters_ + midpixel_y_);
    drawPoint(pixel, color);
}

void ScribbleArea::drawPoint(const QPoint &pix, const QColor& color)
{
    QPainter painter(&image_);
    painter.setPen(QPen(color, pen_width_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(color);
    painter.drawEllipse(pix, point_radius_, point_radius_);
    update(QRect(pix.x() - point_radius_-1, pix.y() - point_radius_-1, pix.x() + point_radius_+1, pix.y() + point_radius_+1));
}

void ScribbleArea::mouseReleaseEvent(QMouseEvent *event)
{
    point_clicked_ = false;
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
    drawSmoothTraj();
    drawRoughTraj();
    update();
    QWidget::resizeEvent(event);
}



void ScribbleArea::drawLineTo(const QPoint &endPoint, const QColor &pen_color, bool draw_point)
{
    QPainter painter(&image_);
    painter.setPen(QPen(pen_color, pen_width_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    if (last_point_.x() < 1e8 && last_point_.y() < 1e8)
    {
        painter.drawLine(last_point_, endPoint);
    }
    int max_x = std::max(endPoint.x(), last_point_.x());
    int min_x = std::min(endPoint.x(), last_point_.x());
    int max_y = std::max(endPoint.y(), last_point_.y());
    int min_y = std::min(endPoint.y(), last_point_.y());
    update(QRect(min_x - 1, min_y -1, max_x+1, max_y+1));
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

void ScribbleArea::drawRoughTraj()
{
    last_point_.setX(5e8);
    last_point_.setY(5e8);

    for (int i = 0; i < rough_trajectory_.size(); i++)
    {
        QPoint pixel(rough_trajectory_[i].x() * pixel_to_meters_ + midpixel_x_,
                     rough_trajectory_[i].y() * pixel_to_meters_ + midpixel_y_);
        if (i == selected_point_idx_)
            drawPoint(pixel, selected_color_);
        else
            drawPoint(pixel, pen_color_);
        drawLineTo(pixel, pen_color_, true);
    }
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

void ScribbleArea::updateSmoothTraj(const MatrixXd &optimized_states)
{
    smooth_traj_.resize(optimized_states.cols());
    for (int i = 0; i < optimized_states.cols(); i++)
    {
        smooth_traj_[i] = optimized_states.block<3,1>(0, i);
    }
    drawSmoothTraj();
}

void ScribbleArea::drawSmoothTraj()
{
    if (smooth_traj_.size() == 0)
        return;
    QPoint prev_last_point = last_point_;
    last_point_ = QPoint(smooth_traj_[0](0)*pixel_to_meters_ + midpixel_x_,
                         smooth_traj_[0](1)*pixel_to_meters_ + midpixel_y_);
    QPoint first_point = last_point_;
    for (int i = 1; i < smooth_traj_.size(); i++)
    {
        QPoint new_point(smooth_traj_[i](0)*pixel_to_meters_ + midpixel_x_,
                         smooth_traj_[i](1)*pixel_to_meters_ + midpixel_y_);
        drawLineTo(new_point, smooth_traj_color_);
    }
    drawLineTo(first_point, smooth_traj_color_);
    last_point_ = prev_last_point;
}


