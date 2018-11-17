#include "trajectory.h"
#include "multirotor.h"
#include "factor.h"

TrajectorySmoother::TrajectorySmoother(const trajVec& vec, double dt_node, int steps_per_node) :
  rough_traj_(vec),
  dt_node_(dt_node),
  steps_per_node_(steps_per_node)
{
  hover_throttle_ = 0.5;
  pos_weight_.setConstant(1.0);
  vel_weight_ = 1.0;
  input_weight_ << 10000.0, 10000.0, 10000.0, 500.0;
}


void TrajectorySmoother::downSample()
{
  int i = 0;
  int j = 1;
  downsampled_traj_.clear();
  downsampled_traj_.push_back(rough_traj_[0]);
  while (j < rough_traj_.size())
  {
    double dist = dt_node_*rough_traj_[i](3);
    if ((rough_traj_[i].topRows(3) - rough_traj_[j].topRows(3)).norm() >= dist)
    {
      downsampled_traj_.push_back(rough_traj_[j]);
      i = j;
    }
    j++;
  }

  Vector4d v_to_start = downsampled_traj_[0] - *(downsampled_traj_.end()-1);
  double dist =  dt_node_*rough_traj_[i](3);
  while (v_to_start.topRows(3).norm() > dist)
  {
    Vector4d new_point = *(downsampled_traj_.end()-1) + v_to_start / v_to_start.norm() * dist;
    downsampled_traj_.push_back(new_point);
    v_to_start = downsampled_traj_[0] - new_point;
    dist =  dt_node_*new_point(3);
  }
}

const MatrixXd& TrajectorySmoother::optimize()
{
  downSample();

  if (problem_)
    delete problem_;
  problem_ = new ceres::Problem;

  initializeTrajectory();
  buildOptimizationGraph();
  runOptimization();
  return optimized_traj_states_;
}

void TrajectorySmoother::runOptimization()
{
  ceres::Solver::Options options;
  options.max_num_iterations = 1000;
  options.num_threads = 4;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;

  ceres::Solve(options, problem_, &summary);
  std::cout << summary.FullReport();
}

void TrajectorySmoother::buildOptimizationGraph()
{
  for (int n = 0; n < downsampled_traj_.size(); n++)
  {
    addPositionCost(n, n*steps_per_node_);
    for (int i = 1; i < steps_per_node_; i++)
    {
      int from_id = n*steps_per_node_ + i - 1;
      int to_id = n*steps_per_node_ + i;
      addDynamicsCost(from_id, to_id);
    }
  }
//  addDynamicsCost(steps_per_node_*downsampled_traj_.size()-1, 0);
}

void TrajectorySmoother::initializeTrajectory()
{
  int num_states = downsampled_traj_.size() * steps_per_node_;
  optimized_traj_states_.setZero(10, num_states);
  optimized_traj_inputs_.setZero(4, num_states);

  optimized_traj_states_.row(3).setConstant(1.0);
  optimized_traj_inputs_.row(3).setConstant(hover_throttle_);
}


void TrajectorySmoother::createParameterBlocks()
{
  for (int n = 0; n < downsampled_traj_.size(); n++)
  {
    for (int i = 0; i < steps_per_node_; i++)
    {
      int id = i + n*steps_per_node_;
      problem_->AddParameterBlock(optimized_traj_states_.data() + (id*10), 10, new NodeLocalParam);
      problem_->AddParameterBlock(optimized_traj_inputs_.data() + (id*4), 4);
    }
  }
}


void TrajectorySmoother::addDynamicsCost(int from_id, int to_id)
{
  double dynamics_dt = dt_node_ / (double)(steps_per_node_);
  problem_->AddResidualBlock(new DynamicsFactor(new DynamicsCostFunction(dynamics_dt, drag_term_, hover_throttle_, input_weight_)),
                             NULL,
                             optimized_traj_states_.data() + (from_id * 10),
                             optimized_traj_states_.data() + (to_id * 10),
                             optimized_traj_inputs_.data() + (from_id * 4));
}


void TrajectorySmoother::addPositionCost(int downsampled_id, int optimized_id)
{
  Vector3d desired_pos = downsampled_traj_[downsampled_id].segment<3>(0);
  double desired_vel = downsampled_traj_[downsampled_id](3);

  problem_->AddResidualBlock(new ConstraintFactor(new PositionConstraintCostFunction(desired_pos, desired_vel, pos_weight_, vel_weight_)),
                             NULL,
                             optimized_traj_states_.data() + (optimized_id*10));
}
