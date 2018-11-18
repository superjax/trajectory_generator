#include <fstream>

#include "trajectory.h"

TrajectorySmoother::TrajectorySmoother(const trajVec& vec, double dt_node, int steps_per_node) :
  rough_traj_(vec),
  dt_node_(dt_node),
  steps_per_node_(steps_per_node)
{
  hover_throttle_ = 0.5;
  pos_weight_.setConstant(1.0);
  vel_weight_ = 1.0;
  input_weight_ << 1.0, 1.0, 1.0, 1.0;
  state_weight_ << 100.0, 100.0, 100.0,
                   100.0, 100.0, 100.0,
                   100.0, 100.0, 100.0;
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
  log();

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
  std::cout << summary.FullReport() << std::endl;
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

void TrajectorySmoother::log() const
{
  ofstream original_file("../logs/original.bin");
  ofstream downsampled_file("../logs/downsampled.bin");
  ofstream optimized_states_file("../logs/optimized_states.bin");
  ofstream optimized_inputs_file("../logs/optimized_inputs.bin");
  ofstream dynamics_costs_file("../logs/dynamics_costs.bin");

  MatrixXd dynamic_costs(4, optimized_traj_states_.size());
  for (int i = 0; i < dynamics_constraints_.size(); i++)
  {
    (*dynamics_constraints_[i])(optimized_traj_states_.data() + (i*10),
                                optimized_traj_states_.data() + ((i+1)*10),
                                optimized_traj_states_.data() + (i*4),
                                dynamic_costs.data() + (i*4));
  }

  dynamics_costs_file.write((char*)dynamic_costs.data(), sizeof(double)*dynamic_costs.rows()*dynamic_costs.cols());
  original_file.write((char*)rough_traj_.data(), sizeof(double) * 4 * rough_traj_.size());
  downsampled_file.write((char*)downsampled_traj_.data(), sizeof(double)*4*downsampled_traj_.size());
  optimized_states_file.write((char*)optimized_traj_states_.data(), sizeof(double)*optimized_traj_states_.rows()*optimized_traj_states_.cols());
  optimized_inputs_file.write((char*)optimized_traj_inputs_.data(), sizeof(double)*optimized_traj_inputs_.rows()*optimized_traj_inputs_.cols());
}


void TrajectorySmoother::addDynamicsCost(int from_id, int to_id)
{
  double dynamics_dt = dt_node_ / (double)(steps_per_node_);
  dynamics_constraints_.push_back(new DynamicsCostFunction(dynamics_dt, drag_term_, hover_throttle_, state_weight_, input_weight_));
  problem_->AddResidualBlock(new DynamicsFactor(*(dynamics_constraints_.end()-1)),
                             NULL,
                             optimized_traj_states_.data() + (from_id * 10),
                             optimized_traj_states_.data() + (to_id * 10),
                             optimized_traj_inputs_.data() + (from_id * 4));
}


void TrajectorySmoother::addPositionCost(int downsampled_id, int optimized_id)
{
  Vector3d desired_pos = downsampled_traj_[downsampled_id].segment<3>(0);
  double desired_vel = downsampled_traj_[downsampled_id](3);

  Vector3d pos = downsampled_traj_[downsampled_id].segment<3>(0);
  Vector3d vel;
  if (downsampled_id < downsampled_traj_.size() - 1)
     vel = downsampled_traj_[downsampled_id+1].segment<3>(0) - pos;
  else
    vel = downsampled_traj_[0].segment<3>(0) - pos;
  vel = vel / vel.norm() * desired_vel;

  Quatd q = Quatd::from_two_unit_vectors(vel/vel.norm(), e_z);
  q = q + 0.05*(Quatd::Identity() - q);

  for (int i = 0; i < steps_per_node_; i++)
  {
    optimized_traj_states_.block<3,1>(0, optimized_id+i) = pos;
    optimized_traj_states_.block<4,1>(3, optimized_id+i) = q.arr_;
    optimized_traj_states_.block<3,1>(7, optimized_id+i) = vel;
  }
  position_constraints_.push_back(new PositionConstraintCostFunction(desired_pos, desired_vel, pos_weight_, vel_weight_));
  problem_->AddResidualBlock(new ConstraintFactor(*(position_constraints_.end()-1)),
                             NULL,
                             optimized_traj_states_.data() + (optimized_id*10));
}
