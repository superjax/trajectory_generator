#pragma once

#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "multirotor.h"
#include "factor.h"
#include "scribblearea.h"

using namespace Eigen;
using namespace std;

typedef vector<Vector4d, aligned_allocator<Vector4d>> trajVec;
class TrajectorySmoother
{
public:
  TrajectorySmoother(const trajVec& rough_traj, double dt, int steps_per_node);
  const MatrixXd &optimize();


private:
  void downSample();
  void initializeTrajectory();
  void createParameterBlocks();
  void addDynamicsCost(int from_id, int to_id);
  void addPositionCost(int downsampled_id, int optimized_id);
  void buildOptimizationGraph();
  void runOptimization();
  void log() const;


  const trajVec& rough_traj_;
  trajVec downsampled_traj_;
  MatrixXd optimized_traj_states_;
  MatrixXd optimized_traj_inputs_;

  vector<DynamicsCostFunction*> dynamics_constraints_;
  vector<PositionConstraintCostFunction*> position_constraints_;

  ceres::Problem* problem_ = nullptr;

  double dt_node_;
  int steps_per_node_;

  double hover_throttle_ = 0.5;
  double drag_term_ = 0.2;

  Vector3d pos_weight_;
  double vel_weight_;
  double tau_wxy_;
  double tau_wz_;
  double tau_F_;
  double max_wxy_;
  double max_wz_;
  double max_F_;
  Vector4d input_weight_;
  Vector9d state_weight_;
};
