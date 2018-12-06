#include "trajopt_ros.h"

#include "trajectory_generator/TrajectoryCommand.h"

TrajOptROS::TrajOptROS() :
  nh_(),
  private_nh_("~")
{
  command_pub_ = nh_.advertise<trajectory_generator::TrajectoryCommand>("trajectory_command", 10, true);

  status_sub_ = nh_.subscribe("status", 10, &TrajOptROS::statusCallback, this);
  odom_sub_ = nh_.subscribe("odom", 10, &TrajOptROS::odomCallback, this);
}

void TrajOptROS::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  odom_stamp_ = odom->header.stamp;
  current_position_.x() = odom->pose.pose.position.x;
  current_position_.y() = odom->pose.pose.position.y;
  current_position_.z() = odom->pose.pose.position.z;
}

void TrajOptROS::statusCallback(const rosflight_msgs::StatusConstPtr &status)
{
  armed_ = status->armed;
}

void TrajOptROS::publishCommand(const Matrix<double, 10, 1> &x_c, Matrix<double, 4, 1> &u_c)
{
  if (!armed_)
    return;

  if (fabs((ros::Time::now() - odom_stamp_).toSec()) > 0.1)
  {
    ROS_WARN("TrajOptROS: Stale Odometry Data");
  }

  trajectory_generator::TrajectoryCommand traj_msg;
  traj_msg.header.stamp = odom_stamp_;
  traj_msg.pos.x = x_c(0);
  traj_msg.pos.y = x_c(1);
  traj_msg.pos.z = x_c(2);
  traj_msg.q.w = x_c(3);
  traj_msg.q.x = x_c(4);
  traj_msg.q.y = x_c(5);
  traj_msg.q.z = x_c(6);
  traj_msg.vel.x = x_c(7);
  traj_msg.vel.y = x_c(8);
  traj_msg.vel.z = x_c(9);
  traj_msg.omega.x = u_c(0);
  traj_msg.omega.y = u_c(1);
  traj_msg.omega.z = u_c(2);
  traj_msg.F = u_c(3);
  command_pub_.publish(traj_msg);
}


