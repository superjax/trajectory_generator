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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QList>
#include <QMainWindow>
#include <QWidget>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include "trajectory.h"

class ScribbleArea;
class QDoubleSpinBox;
class QVBoxLayout;
class QPushButton;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv);
    void setAltValue(double alt);

public slots:
    void about();
    void createTrajectory();
    void handleFlyButton();
    void handleRTHButton();

protected:
    void closeEvent(QCloseEvent *event) override;
    void timerEvent(QTimerEvent * ev) override;

    void initROS(int argc, char** argv);
    void createActions();
    void createMenus();

    void updateCommand();
    void updateState();
    void publishCommand();

    ScribbleArea *scribble_area_;

    QMenu *save_as_menu_;
    QMenu *file_menu_;
    QMenu *option_menu_;
    QMenu *help_menu_;
    QHBoxLayout *main_layout_;

    QVBoxLayout *control_layout_;
    QDoubleSpinBox *altitude_spin_box_;
    QLabel *altitude_spin_box_label_;
    QDoubleSpinBox *velocity_spin_box_;
    QLabel *velocity_spin_box_label_;
    QDoubleSpinBox *acc_spin_box_;
    QLabel *acc_spin_box_label_;
    QPushButton *fly_button_;
    QPushButton *delete_button_;
    QPushButton *return_to_home_button_;
    QPushButton *clear_screen_button_;
    QPushButton *create_trajectory_button_;

    QAction *open_act_;
    QList<QAction *> save_as_acts_;
    QAction *exit_act_;
    QAction *pen_color_act_;
    QAction *pen_width_act_;
    QAction *print_act_;
    QAction *clear_screen_act_;
    QAction *about_act_;

    TrajectorySmoother* smoother_ = nullptr;
    MatrixXd optimized_states_;
    MatrixXd optimized_inputs_;
    const double hover_throttle_ = 0.5;
    const double sample_dt_ = 0.02;

    Matrix<double, 10, 1> x_r_;
    Matrix<double, 4, 1> u_r_;
    Vector3d start_position_;
    Vector3d landing_commanded_position_;

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
