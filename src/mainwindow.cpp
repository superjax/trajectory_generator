#include <QtWidgets>

#include "mainwindow.h"
#include "scribblearea.h"

MainWindow::MainWindow(int argc, char **argv)
{
    scribble_area_ = new ScribbleArea(this);

    altitude_spin_box_ = new QDoubleSpinBox();
    connect(altitude_spin_box_, SIGNAL(valueChanged(double)), scribble_area_, SLOT(setAltitude(double)));
    altitude_spin_box_->setSingleStep(0.1);
    altitude_spin_box_->setMaximum(3.0);
    altitude_spin_box_->setMaximum(100.0);
    altitude_spin_box_->setMinimum(0.1);
    altitude_spin_box_->setValue(1.5);
    altitude_spin_box_label_ = new QLabel();
    altitude_spin_box_label_->setText("altitude (m):");

    velocity_spin_box_ = new QDoubleSpinBox();
    velocity_spin_box_->setSingleStep(0.1);
    velocity_spin_box_->setMaximum(20.0);
    velocity_spin_box_->setMinimum(0.1);
    velocity_spin_box_->setValue(2.0);
    velocity_spin_box_label_ = new QLabel();
    velocity_spin_box_label_->setText("max vel (m/s):");

    acc_spin_box_ = new QDoubleSpinBox();
    acc_spin_box_->setSingleStep(0.1);
    acc_spin_box_->setMaximum(20.0);
    acc_spin_box_->setMinimum(0.1);
    acc_spin_box_->setValue(5.0);
    acc_spin_box_label_ = new QLabel();
    acc_spin_box_label_->setText("max accel (m/s^2):");


    main_layout_ = new QHBoxLayout();
    main_layout_->addWidget(scribble_area_);
    main_layout_->setStretchFactor(scribble_area_, 1);
    control_layout_ = new QVBoxLayout();

    QHBoxLayout* alt = new QHBoxLayout();
    alt->addWidget(altitude_spin_box_label_);
    alt->addWidget(altitude_spin_box_);
    control_layout_->addLayout(alt);

    QHBoxLayout* vel = new QHBoxLayout();
    vel->addWidget(velocity_spin_box_label_);
    vel->addWidget(velocity_spin_box_);
    control_layout_->addLayout(vel);

    QHBoxLayout* acc = new QHBoxLayout();
    acc->addWidget(acc_spin_box_label_);
    acc->addWidget(acc_spin_box_);
    control_layout_->addLayout(acc);

    fly_button_ = new QPushButton();
    fly_button_->setText("Fly");
    connect(fly_button_, SIGNAL(released()), this, SLOT(handleFlyButton()));
    control_layout_->addWidget(fly_button_);

    delete_button_ = new QPushButton();
    delete_button_->setText("Delete Point");
    connect(delete_button_, SIGNAL(released()), scribble_area_, SLOT(deletePoint()));
    control_layout_->addWidget(delete_button_);

    return_to_home_button_ = new QPushButton();
    return_to_home_button_->setText("RTH");
    connect(return_to_home_button_, SIGNAL(released()), this, SLOT(handleRTHButton()));
    control_layout_->addWidget(return_to_home_button_);

    clear_screen_button_ = new QPushButton();
    clear_screen_button_->setText("Clear Trajectory");
    connect(clear_screen_button_, SIGNAL(released()), scribble_area_, SLOT(clearTrajectory()));
    control_layout_->addWidget(clear_screen_button_);

    create_trajectory_button_ = new QPushButton();
    create_trajectory_button_->setText("Create Trajectory");
    connect(create_trajectory_button_, SIGNAL(released()), this, SLOT(createTrajectory()));
    control_layout_->addWidget(create_trajectory_button_);

    control_layout_->addStretch(1);

    main_layout_->addLayout(control_layout_);

    QWidget* central_widget = new QWidget();
    setCentralWidget(central_widget);
    centralWidget()->setLayout(main_layout_);

    createActions();
    createMenus();

    initROS(argc, argv);

    setWindowTitle(tr("TrajectoryGenerator"));
    resize(835, 918);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    (void)event;

    if (smoother_)
        delete smoother_;

    if(ros_node_)
        delete ros_node_;
}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About TrajectoryGenerator"),
                       tr("<p></p>"));
}


void MainWindow::createActions()
{
    clear_screen_act_ = new QAction(tr("&Clear Screen"), this);
    clear_screen_act_->setShortcut(tr("Ctrl+L"));
    connect(clear_screen_act_, SIGNAL(triggered()), scribble_area_, SLOT(clearImage()));
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
    option_menu_->addAction(clear_screen_act_);

    help_menu_ = new QMenu(tr("&Help"), this);
    help_menu_->addAction(about_act_);

    menuBar()->addMenu(file_menu_);
    menuBar()->addMenu(option_menu_);
    menuBar()->addMenu(help_menu_);
}

void MainWindow::setAltValue(double alt)
{
    altitude_spin_box_->blockSignals(true);
    altitude_spin_box_->setValue(-alt);
    altitude_spin_box_->blockSignals(false);
}

void MainWindow::createTrajectory()
{
    if (smoother_ != nullptr)
        delete smoother_;

    const trajVec& rough_trajectory_(scribble_area_->getRoughTrajectory());

    smoother_ = new TrajectorySmoother(rough_trajectory_, sample_dt_);
    smoother_->setBounds(velocity_spin_box_->value(), acc_spin_box_->value());
    smoother_->optimize(optimized_states_, optimized_inputs_);

    scribble_area_->updateSmoothTraj(optimized_states_);
}

void MainWindow::updateState()
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
            landing_commanded_position_ << start_position_.x(), start_position_.y(), optimized_states_(2,0);
            state_ = LAND;
            break;
        case LAND:
            if (std::abs(x_r_(LQR::POS+2) - start_position_.z()) < 0.05)
            {
                cout << "done" << endl;
                state_ = UNCOMMANDED;
            }
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

void MainWindow::updateCommand()
{
    x_r_ << 0, 0, 0,
            1, 0, 0, 0,
            0, 0, 0;
    u_r_ << 0, 0, 0, hover_throttle_;


    switch (state_)
    {
    case FLY_TO_ALTITUDE:
        x_r_.segment<3>(LQR::POS) << start_position_.x(), start_position_.y(), optimized_states_(2,0);
        break;
    case FLY_TO_START_OF_TRAJECTORY:
        x_r_.segment<3>(LQR::POS) << optimized_states_(0,0),
                optimized_states_(1,0),
                optimized_states_(2,0);
        break;
    case FLY_TRAJECTORY:
        x_r_ = optimized_states_.col(cmd_idx_);
        u_r_ = optimized_inputs_.col(cmd_idx_);
        break;
    case FLY_TO_HOME:
        x_r_.segment<3>(LQR::POS) << start_position_.x(),
                start_position_.y(),
                optimized_states_(2,0);
        break;
    case LAND:
        landing_commanded_position_.z() += 0.3*sample_dt_;
        landing_commanded_position_.z() = std::min(start_position_.z(), landing_commanded_position_.z());
        x_r_.segment<3>(LQR::POS) = landing_commanded_position_;
        break;
    case UNCOMMANDED:
        x_r_.setConstant(NAN);
        u_r_.setConstant(NAN);
        scribble_area_->lockScreen(true);
        killTimer(publish_command_timer_id_);
    default:
        break;
    }
    ros_node_->publishCommand(x_r_, u_r_);
}

void MainWindow::initROS(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros_node_ = new TrajOptROS();
    ros_node_timer_id_ = startTimer(1);
}

void MainWindow::timerEvent(QTimerEvent * ev)
{
    if (ev->timerId() == ros_node_timer_id_)
    {
        ros::spinOnce();
    }
    else if (ev->timerId() == publish_command_timer_id_)
    {
        scribble_area_->drawPosition(ros_node_->getCurrentPosition(), x_r_.segment<3>(0));
        updateState();
        updateCommand();
    }
}


void MainWindow::handleRTHButton()
{
    cout << "fly home" << endl;
    state_ = FLY_TO_HOME;
    updateCommand();
}

void MainWindow::handleFlyButton()
{
    if (optimized_states_.cols() == 0)
    {
        createTrajectory();
    }
    if (state_ == UNCOMMANDED)
    {
        cmd_idx_ = 0;
        state_ = FLY_TO_ALTITUDE;
        cout << "Fly to altitude" << endl;
        publish_command_timer_id_ = startTimer(1000 * sample_dt_);
        start_position_ = ros_node_->getCurrentPosition();
        scribble_area_->lockScreen(true);
        updateCommand();
    }
}


