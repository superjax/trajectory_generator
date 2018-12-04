#pragma once

#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "scribblearea.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector4d, aligned_allocator<Vector4d>> trajVec;
class TrajectorySmoother
{
public:
  TrajectorySmoother(const trajVec& rough_traj, double dt, int steps_per_node);
  const MatrixXd &optimize();
  static void SinglePoly(Vector4d d0, Vector4d df, Vector4d c);


private:
  void downSample();
  void log() const;


  const trajVec& rough_traj_;
  trajVec downsampled_traj_;
  MatrixXd optimized_traj_states_;
  MatrixXd optimized_traj_inputs_;

  ceres::Problem* problem_ = nullptr;

  double dt_node_;
  int steps_per_node_;

  double hover_throttle_ = 0.5;
  double drag_term_ = 0.2;
};
