#pragma once

#include <vector>

#include <Eigen/Core>

#include "scribblearea.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector4d, aligned_allocator<Vector4d>> trajVec;
class TrajectorySmoother
{
public:
  TrajectorySmoother(const trajVec& rough_traj);
  void downSample(double t);
  const trajVec& smooth();

  const trajVec& rough_traj_;
  trajVec smooth_traj_;


};
