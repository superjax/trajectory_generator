#include <fstream>

#include "trajectory.h"

#include "geometry/quat.h"
#include "geometry/support.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/trajectory.h"


TrajectorySmoother::TrajectorySmoother(const trajVec& vec, double sample_dt) :
    rough_traj_(vec),
    sample_dt_(sample_dt)
{}


void TrajectorySmoother::setBounds(double max_v, double max_a)
{
    max_v_ = max_v;
    max_a_ = max_a;
}

bool TrajectorySmoother::solveTrajectoryOpt()
{
    mav_trajectory_generation::Vertex::Vector vertices;
    const int dim = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    for (int i = 0; i < rough_traj_.size(); i++)
    {
        mav_trajectory_generation::Vertex vertex(dim);
        Vector3d desired_pos = rough_traj_[i].topRows<3>();
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, desired_pos);
        vertices.push_back(vertex);
    }
    mav_trajectory_generation::Vertex vertex(dim);
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, rough_traj_[0].topRows<3>());
    vertices.push_back(vertex);

    std::vector<double> segment_times;
    try
    {
        segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, max_v_, max_a_);
    }
    catch (const std::exception& e)
    {
        cout << "unable to optimize trajectory: " << e.what() << endl;
        return false;
    }

    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    //  parameters.max_iterations = 1000;
    //  parameters.f_rel = 0.05;
    //  parameters.x_rel = 0.1;
    parameters.time_penalty = 10000.0;
    //  parameters.initial_stepsize_rel = 0.1;
    //  parameters.inequality_constraint_tolerance = 0.001;
    //  parameters.equality_constraint_tolerance = 0.0001;

    mav_trajectory_generation::PolynomialOptimizationNonLinear<10> opt(dim, parameters);
    try
    {
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
        opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
        opt.optimize();
    }
    catch (const std::exception& e)
    {
        cout << "unable to optimize trajectory: " << e.what() << endl;
        return false;
    }

    mav_trajectory_generation::Segment::Vector segments;
    opt.getPolynomialOptimizationRef().getSegments(&segments);

    trajectory_.clear();
    opt.getTrajectory(&trajectory_);
    return true;
}

void TrajectorySmoother::calcStatesAndInputsFromTrajectory()
{
    double t_start = trajectory_.getMinTime();
    double t_end = trajectory_.getMaxTime();
    std::vector<Eigen::VectorXd> p;
    std::vector<Eigen::VectorXd> pdot;
    std::vector<Eigen::VectorXd> pddot;
    trajectory_.evaluateRange(t_start, t_end, sample_dt_, mav_trajectory_generation::derivative_order::POSITION, &p, &optimized_traj_t_);
    trajectory_.evaluateRange(t_start, t_end, sample_dt_, mav_trajectory_generation::derivative_order::VELOCITY, &pdot, &optimized_traj_t_);
    trajectory_.evaluateRange(t_start, t_end, sample_dt_, mav_trajectory_generation::derivative_order::ACCELERATION, &pddot, &optimized_traj_t_);


    optimized_traj_states_.resize(10, optimized_traj_t_.size()); // [ P Q V ]
    optimized_traj_inputs_.resize(4, optimized_traj_t_.size()); // [ W, F ]

    static const double g = 9.80665;
    static const Vector3d gravity{0, 0, g};
    for (int i = 0; i < optimized_traj_t_.size(); i++)
    {
        optimized_traj_states_.block<3,1>(0, i) = p[i];

        Vector3d acc_desired_I = pddot[i] + gravity + pdot[i]*drag_term_;
        double acc_mag = acc_desired_I.norm();
        Vector3d acc_desired_direction = acc_desired_I / acc_mag;
        quat::Quatd q_I_b = quat::Quatd::from_two_unit_vectors(acc_desired_direction, e_z);
        optimized_traj_states_.block<4,1>(3, i) = q_I_b.elements();
        optimized_traj_states_.block<3,1>(7, i) = q_I_b.rota(pdot[i]);
        optimized_traj_inputs_(3,i) = acc_mag/g * hover_throttle_;
    }

    for (int i = 0; i < optimized_traj_t_.size(); i++)
    {
        Vector3d omega_d;
        if (i == 0)
        {
            quat::Quatd q_next(optimized_traj_states_.block<4,1>(3, i+1));
            quat::Quatd q_i(optimized_traj_states_.block<4,1>(3, i));
            omega_d = (q_next - q_i) / sample_dt_;
        }
        else if (i == optimized_traj_t_.size() - 1)
        {
            quat::Quatd q_i(optimized_traj_states_.block<4,1>(3, i));
            quat::Quatd q_prev(optimized_traj_states_.block<4,1>(3, i-1));
            omega_d = (q_i - q_prev) / sample_dt_;
        }
        else
        {
            quat::Quatd q_next(optimized_traj_states_.block<4,1>(3, i+1));
            quat::Quatd q_prev(optimized_traj_states_.block<4,1>(3, i-1));
            omega_d = (q_next - q_prev) / (2.0*sample_dt_);
        }
        optimized_traj_inputs_.block<3,1>(0, i) = omega_d;
    }
}

const void TrajectorySmoother::optimize(MatrixXd& states, MatrixXd& inputs)
{
    if (solveTrajectoryOpt())
    {
        calcStatesAndInputsFromTrajectory();
        log();
    }
    else
    {
        optimized_traj_states_.resize(0,0);
    }
    states = optimized_traj_states_;
    inputs = optimized_traj_inputs_;
}

void TrajectorySmoother::log() const
{
    ofstream time_file("../logs/time.bin");
    ofstream original_file("../logs/original.bin");
    ofstream optimized_states_file("../logs/optimized_states.bin");
    ofstream optimized_inputs_file("../logs/optimized_inputs.bin");

    original_file.write((char*)rough_traj_.data(), sizeof(double) * 4 * rough_traj_.size());
    time_file.write((char*)optimized_traj_t_.data(), sizeof(double) *optimized_traj_t_.size());
    optimized_states_file.write((char*)optimized_traj_states_.data(), sizeof(double)*optimized_traj_states_.rows()*optimized_traj_states_.cols());
    optimized_inputs_file.write((char*)optimized_traj_inputs_.data(), sizeof(double)*optimized_traj_inputs_.rows()*optimized_traj_inputs_.cols());

    time_file.close();
    original_file.close();
    optimized_states_file.close();
    optimized_inputs_file.close();
}
