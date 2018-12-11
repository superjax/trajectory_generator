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

#ifndef SCRIBBLEAREA_H
#define SCRIBBLEAREA_H

#include <iostream>

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QWidget>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "trajectory.h"
#include "trajopt_ros.h"
#include "lqr.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector3d, aligned_allocator<Vector3d>> trajVec;
class TrajectorySmoother;

class ScribbleArea : public QWidget
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ScribbleArea(int argc, char** argv, QWidget *parent = 0);
    ~ScribbleArea();
    void initROS(int argc, char** argv);

//    bool openImage(const QString &fileName);
//    bool saveImage(const QString &fileName, const char *fileFormat);
    void setPenColor(const QColor &newColor);
    void setPenWidth(int newWidth);

    QColor penColor() const { return pen_color_; }
    int penWidth() const { return pen_width_; }

public slots:
    void setAltitude(double alt);
    void setMaxVelocity(double vel);
    void setMaxAccel(double acc);
    void clearImage();
    void print();
    void handleFlyButton();
    void handleRTHButton();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void timerEvent(QTimerEvent * ev) override;

private:
    void drawBackground();
    void addPoint(const Vector3d &point);
    void addPoint(const QPoint &point);
    void drawLineTo(const QPoint &endPoint, QColor& pen_color);
    void resizeImage(QImage *image_, const QSize &newSize);
    void smoothTrajectory();
    void plotSmoothTrajectory();

    void updateCommand();
    void updateState();
    void publishCommand();

    bool modified_;
    bool scribbling_;
    bool empty_;
    int pen_width_;
    QColor pen_color_;
    QImage image_;
    QPoint last_point_;
    QPoint first_point_;
    double trajectory_altitude_;
    double max_vel_;
    double max_acc_;

    double pixel_to_meters_;
    int midpixel_x_;
    int midpixel_y_;
    static const double room_width_;
    static const double room_height_;
    double hover_throttle_ = 0.5;

    double waypoint_distance_ = 0.3;
    double sample_dt_ = 0.02;

    trajVec rough_trajectory_;
    trajVec smooth_traj_;
    MatrixXd optimized_states_;
    MatrixXd optimized_inputs_;

    Matrix<double, 10, 1> x_r_;
    Matrix<double, 4, 1> u_r_;
    Vector3d start_position_;

    TrajectorySmoother* smoother_ = nullptr;
    TrajOptROS* ros_node_;
    int ros_node_timer_id_;

    int publish_command_timer_id_;
    int cmd_idx_;

    enum
    {
        UNCOMMANDED,
        FLY_TO_ALTITUDE,
        FLY_TO_START_OF_TRAJECTORY,
        FLY_TRAJECTORY,
        FLY_TO_HOME,
        LAND
    };

    int state_ = UNCOMMANDED;
};

#endif
