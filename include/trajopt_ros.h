#include <ros/ros.h>
#include <Eigen/Core>

#include <nav_msgs/Odometry.h>
#include <rosflight_msgs/Status.h>

using namespace Eigen;

class TrajOptROS
{
public:
  TrajOptROS();

  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void statusCallback(const rosflight_msgs::StatusConstPtr& status);
  const Vector3d& getCurrentPosition() const { return current_position_; }
  void pubishCommand(const Matrix<double, 10, 1>& x_c, Matrix<double, 4,1>& u_c);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber status_sub_;
  ros::Time odom_stamp_;

  Vector3d current_position_{0, 0, 0};

  bool armed_;
};
