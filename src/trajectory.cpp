#include "trajectory.h"

TrajectorySmoother::TrajectorySmoother(const trajVec& vec) :
  rough_traj_(vec)
{}


void TrajectorySmoother::downSample(double t)
{
  int i = 0;
  int j = 1;
  smooth_traj_.clear();
  smooth_traj_.push_back(rough_traj_[0]);
  while (j < rough_traj_.size())
  {
    double dist = t*rough_traj_[i](3);
    if ((rough_traj_[i].topRows(3) - rough_traj_[j].topRows(3)).norm() >= dist)
    {
      smooth_traj_.push_back(rough_traj_[j]);
      i = j;
    }
    j++;
  }

  Vector4d v_to_start = smooth_traj_[0] - *(smooth_traj_.end()-1);
  double dist =  t*rough_traj_[i](3);
  while (v_to_start.topRows(3).norm() > dist)
  {
    Vector4d new_point = *(smooth_traj_.end()-1) + v_to_start / v_to_start.norm() * dist;
    smooth_traj_.push_back(new_point);
    v_to_start = smooth_traj_[0] - new_point;
    dist =  t*new_point(3);
  }
}

const trajVec& TrajectorySmoother::smooth()
{
  downSample(0.5);
  return smooth_traj_;
}
