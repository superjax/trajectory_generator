#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/Command.h>
#include <trajectory_generator/TrajectoryCommand.h>


#include "lqr.h"

class LQR_ROS
{
public:
  LQR_ROS();

  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void trajCommandCallback(const trajectory_generator::TrajectoryCommandConstPtr& cmd);
  void statusCallback(const rosflight_msgs::StatusConstPtr& status);
  void computeCommand();

  void setHoverThrottle(double hover_throttle);
  void setDragTerm(double drag_term);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber cmd_sub_;
  ros::Publisher command_pub_;

  LQR lqr_;
  double hover_throttle_;
  double drag_term_;
  bool armed_;
  bool got_state_ = false;
  bool got_reference_ = false;

  ros::Time odom_stamp_;

  Vector10d x_;
  Vector10d x_r_;
  Vector4d u_;
  Vector4d u_r_;
  Vector4d u_tilde_;
};
