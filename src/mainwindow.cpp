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

#include "mainwindow.h"
#include "scribblearea.h"

MainWindow::MainWindow()
{
    scribble_area_ = new ScribbleArea(this);
    main_layout_ = new QVBoxLayout();
    main_layout_->addWidget(scribble_area_);

    control_layout_ = new QHBoxLayout();
    altitude_spin_box_ = new QDoubleSpinBox();
    connect(altitude_spin_box_, SIGNAL(valueChanged(double)), scribble_area_, SLOT(setAltitude(double)));
    altitude_spin_box_->setSingleStep(0.1);
    altitude_spin_box_->setMaximum(3.0);
    altitude_spin_box_->setMinimum(0.1);
    altitude_spin_box_->setValue(1.0);
    altitude_spin_box_label_ = new QLabel();
    altitude_spin_box_label_->setText("altitude (m):");
    control_layout_->addWidget(altitude_spin_box_label_);
    control_layout_->addWidget(altitude_spin_box_);

    velocity_spin_box_ = new QDoubleSpinBox();
    connect(velocity_spin_box_, SIGNAL(valueChanged(double)), scribble_area_, SLOT(setMaxVelocity(double)));
    velocity_spin_box_->setSingleStep(0.1);
    velocity_spin_box_->setMaximum(20.0);
    velocity_spin_box_->setMinimum(0.1);
    velocity_spin_box_->setValue(1.0);
    velocity_spin_box_label_ = new QLabel();
    velocity_spin_box_label_->setText("max velocity (m/s):");
    control_layout_->addWidget(velocity_spin_box_label_);
    control_layout_->addWidget(velocity_spin_box_);

    acc_spin_box_ = new QDoubleSpinBox();
    connect(acc_spin_box_, SIGNAL(valueChanged(double)), scribble_area_, SLOT(setMaxAccel(double)));
    acc_spin_box_->setSingleStep(0.1);
    acc_spin_box_->setMaximum(20.0);
    acc_spin_box_->setMinimum(0.1);
    acc_spin_box_->setValue(1.0);
    acc_spin_box_label_ = new QLabel();
    acc_spin_box_label_->setText("max accel (m/s^2):");
    control_layout_->addWidget(acc_spin_box_label_);
    control_layout_->addWidget(acc_spin_box_);

    main_layout_->addLayout(control_layout_);

    QWidget* central_widget = new QWidget();
    setCentralWidget(central_widget);
    centralWidget()->setLayout(main_layout_);

    createActions();
    createMenus();

    setWindowTitle(tr("TrajectoryGenerator"));
    resize(650, 918);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    (void)event;
}

//void MainWindow::penColor()
//{
//    QColor newColor = QColorDialog::getColor(scribble_area_->penColor());
//    if (newColor.isValid())
//        scribble_area_->setPenColor(newColor);
//}

//void MainWindow::penWidth()
//{
//    bool ok;
//    int newWidth = QInputDialog::getInt(this, tr("TrajectoryGenerator"),
//                                        tr("Select pen width:"),
//                                        scribble_area_->penWidth(),
//                                        1, 50, 1, &ok);
//    if (ok)
//        scribble_area_->setPenWidth(newWidth);
//}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About TrajectoryGenerator"),
            tr("<p></p>"));
}


void MainWindow::createActions()
{
//    exit_act_ = new QAction(tr("E&xit"), this);
//    exit_act_->setShortcuts(QKeySequence::Quit);
//    connect(exit_act_, SIGNAL(triggered()), this, SLOT(close()));

//    pen_color_act_ = new QAction(tr("&Pen Color..."), this);
//    connect(pen_color_act_, SIGNAL(triggered()), this, SLOT(penColor()));

//    pen_width_act_ = new QAction(tr("Pen &Width..."), this);
//    connect(pen_width_act_, SIGNAL(triggered()), this, SLOT(penWidth()));

    clear_screen_act_ = new QAction(tr("&Clear Screen"), this);
    clear_screen_act_->setShortcut(tr("Ctrl+L"));
    connect(clear_screen_act_, SIGNAL(triggered()),
            scribble_area_, SLOT(clearImage()));

    about_act_ = new QAction(tr("&About"), this);
    connect(about_act_, SIGNAL(triggered()), this, SLOT(about()));
}

void MainWindow::createMenus()
{
    save_as_menu_ = new QMenu(tr("&Save As"), this);
    foreach (QAction *action, save_as_acts_)
        save_as_menu_->addAction(action);

    file_menu_ = new QMenu(tr("&File"), this);

    option_menu_ = new QMenu(tr("&Options"), this);
//    option_menu_->addAction(pen_color_act_);
//    option_menu_->addAction(pen_width_act_);
//    option_menu_->addSeparator();
    option_menu_->addAction(clear_screen_act_);

    help_menu_ = new QMenu(tr("&Help"), this);
    help_menu_->addAction(about_act_);

    menuBar()->addMenu(file_menu_);
    menuBar()->addMenu(option_menu_);
    menuBar()->addMenu(help_menu_);
}


