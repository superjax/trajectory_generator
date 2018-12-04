#include <fstream>

#include "trajectory.h"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>

TrajectorySmoother::TrajectorySmoother(const trajVec& vec, double dt_node, int steps_per_node) :
  rough_traj_(vec),
  dt_node_(dt_node),
  steps_per_node_(steps_per_node)
{}


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

  mav_trajectory_generation::Vertex::Vector vertices;
  const int dim = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  for (int i = 0; i < downsampled_traj_.size(); i++)
  {
    mav_trajectory_generation::Vertex vertex(dim);
    Vector3d desired_pos = downsampled_traj_[i].topRows<3>();
    if (i == 0)
      vertex.makeStartOrEnd(desired_pos, derivative_to_optimize);
    else
      vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, desired_pos);

    vertices.push_back(vertex);
  }
  mav_trajectory_generation::Vertex vertex(dim);
  vertex.makeStartOrEnd(downsampled_traj_[0].topRows<3>(), derivative_to_optimize);
  vertices.push_back(vertex);

  std::vector<double> segment_times;
  const double v_max = 2.0;
  const double a_max = 2.0;
  segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max);

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 500.0;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;

  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dim, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
  opt.optimize();

  mav_trajectory_generation::Segment::Vector segments;
  opt.getPolynomialOptimizationRef().getSegments(&segments);


  MatrixXd optimized_traj_states_;
  return optimized_traj_states_;
}

void TrajectorySmoother::log() const
{
//  ofstream original_file("../logs/original.bin");
//  ofstream downsampled_file("../logs/downsampled.bin");
//  ofstream optimized_states_file("../logs/optimized_states.bin");
//  ofstream optimized_inputs_file("../logs/optimized_inputs.bin");
//  ofstream dynamics_costs_file("../logs/dynamics_costs.bin");

//  MatrixXd dynamic_costs(17, optimized_traj_states_.size());
//  for (int i = 0; i < dynamics_constraints_.size(); i++)
//  {
//    (*dynamics_constraints_[i])(optimized_traj_states_.data() + (i*14),
//                                optimized_traj_states_.data() + ((i+1)*14),
//                                optimized_traj_states_.data() + (i*4),
//                                dynamic_costs.data() + (i*4));
//  }

//  dynamics_costs_file.write((char*)dynamic_costs.data(), sizeof(double)*dynamic_costs.rows()*dynamic_costs.cols());
//  original_file.write((char*)rough_traj_.data(), sizeof(double) * 4 * rough_traj_.size());
//  downsampled_file.write((char*)downsampled_traj_.data(), sizeof(double)*4*downsampled_traj_.size());
//  optimized_states_file.write((char*)optimized_traj_states_.data(), sizeof(double)*optimized_traj_states_.rows()*optimized_traj_states_.cols());
//  optimized_inputs_file.write((char*)optimized_traj_inputs_.data(), sizeof(double)*optimized_traj_inputs_.rows()*optimized_traj_inputs_.cols());
}
