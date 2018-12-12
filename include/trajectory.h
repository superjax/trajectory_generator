#pragma once

#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "mav_trajectory_generation/trajectory.h"

#include "scribblearea.h"


using namespace Eigen;
using namespace std;

typedef vector<Vector3d, aligned_allocator<Vector3d>> trajVec;
class TrajectorySmoother
{
public:
  TrajectorySmoother(const trajVec& rough_traj, double sample_dt);
  void setBounds(double max_v, double max_a);
  const void optimize(MatrixXd &states, MatrixXd &inputs);

private:
  bool solveTrajectoryOpt();
  void calcStatesAndInputsFromTrajectory();
  void log() const;
  Vector3d sat(const Vector3d &v);


  const trajVec& rough_traj_;
  mav_trajectory_generation::Trajectory trajectory_;
  MatrixXd optimized_traj_states_;
  MatrixXd optimized_traj_inputs_;
  std::vector<double> optimized_traj_t_;

  ceres::Problem* problem_ = nullptr;

  double sample_dt_;
  double hover_throttle_ = 0.5;
  double drag_term_ = 0.2;
  double max_v_ = 8.0;
  double max_a_ = 0.5;
};
