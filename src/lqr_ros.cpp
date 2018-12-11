#include "lqr_ros.h"


LQR_ROS::LQR_ROS() :
  nh_(),
  private_nh_("~")
{
  hover_throttle_ = 0.5;
  drag_term_ = 0.1;

  odom_sub_ = nh_.subscribe("state", 1, &LQR_ROS::odomCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &LQR_ROS::statusCallback, this);
  cmd_sub_ = nh_.subscribe("trajectory_command", 1, &LQR_ROS::trajCommandCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);

  Vector9d Qdiag;
  Vector4d Rdiag;
  importMatrixFromParamServer(private_nh_, Qdiag, "Qdiag");
  importMatrixFromParamServer(private_nh_, Rdiag, "Rdiag");
  Matrix9d Q = Qdiag.asDiagonal();
  Matrix4d R = Rdiag.asDiagonal();
  lqr_.setQ(Q);
  lqr_.setR(R);
  lqr_.setDragTerm(drag_term_);
}


void LQR_ROS::odomCallback(const nav_msgs::OdometryConstPtr& odom)
{
  odom_stamp_ = odom->header.stamp;
  x_(LQR::POS)   = odom->pose.pose.position.x;
  x_(LQR::POS+1) = odom->pose.pose.position.y;
  x_(LQR::POS+2) = odom->pose.pose.position.z;
  x_(LQR::ATT)   = odom->pose.pose.orientation.w;
  x_(LQR::ATT+1) = odom->pose.pose.orientation.x;
  x_(LQR::ATT+2) = odom->pose.pose.orientation.y;
  x_(LQR::ATT+3) = odom->pose.pose.orientation.z;
  x_(LQR::VEL)   = odom->twist.twist.linear.x;
  x_(LQR::VEL+1) = odom->twist.twist.linear.y;
  x_(LQR::VEL+2) = odom->twist.twist.linear.z;
  got_state_ = true;
  computeCommand();
}

void LQR_ROS::trajCommandCallback(const trajectory_generator::TrajectoryCommandConstPtr& cmd)
{
  x_r_(LQR::POS)   = cmd->pos.x;
  x_r_(LQR::POS+1) = cmd->pos.y;
  x_r_(LQR::POS+2) = cmd->pos.z;
  x_r_(LQR::ATT)   = cmd->q.w;
  x_r_(LQR::ATT+1) = cmd->q.x;
  x_r_(LQR::ATT+2) = cmd->q.y;
  x_r_(LQR::ATT+3) = cmd->q.z;
  x_r_(LQR::VEL)   = cmd->vel.x;
  x_r_(LQR::VEL+1) = cmd->vel.y;
  x_r_(LQR::VEL+2) = cmd->vel.z;

  u_r_(LQR::OMEGA)   = cmd->omega.x;
  u_r_(LQR::OMEGA+1) = cmd->omega.y;
  u_r_(LQR::OMEGA+2) = cmd->omega.z;
  u_r_(LQR::F) = cmd->F;

  if  ((x_r_.array() != x_r_.array()).any()
       || (u_r_.array() != u_r_.array()).any())
  {
      got_reference_ = false;
  }
  else
  {
      got_reference_ = true;
  }
  computeCommand();
}

void LQR_ROS::computeCommand()
{
  rosflight_msgs::Command cmd;
  if (!got_reference_ || !got_state_)
  {
    u_.setZero();
  }
  else
  {
    lqr_.computeControl(x_, x_r_, u_tilde_);
    u_ = u_r_ + u_tilde_;
  }

  cmd.header.stamp = odom_stamp_;
  cmd.mode = rosflight_msgs::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
  cmd.x = u_(LQR::OMEGA);
  cmd.y = u_(LQR::OMEGA+1);
  cmd.z = u_(LQR::OMEGA+2);
  cmd.F = u_(LQR::F) > 0.9 ? 0.9 : u_(LQR::F) < 0.0 ? 0.0 : u_(LQR::F);
  command_pub_.publish(cmd);
}

void LQR_ROS::statusCallback(const rosflight_msgs::StatusConstPtr& status)
{
  armed_ = status->armed;
}


void LQR_ROS::setHoverThrottle(double hover_throttle)
{
  hover_throttle_ = hover_throttle;
  lqr_.setHoverThrottle(hover_throttle);
}


void LQR_ROS::setDragTerm(double drag_term)
{
  drag_term_ = drag_term;
  lqr_.setDragTerm(drag_term);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lqr_controller");
  LQR_ROS thing;
  ros::spin();
}
