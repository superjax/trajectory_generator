#include <ceres/ceres.h>
#include <Eigen/Core>

#include "geometry/xform.h"
#include "multirotor.h"

using namespace Eigen;
using namespace xform;


class DynamicsCostFunction
{
public:
  DynamicsCostFunction(double dt, double drag_term, double hover_throttle, const Vector9d& state_weight, const Vector4d& input_weight) :
    dt_{dt},
    drag_term_{drag_term},
    hover_throttle_{hover_throttle}
  {
    input_weight_ = input_weight;
    state_weight_ = state_weight;
  }

  template <typename T>
  bool operator() (const T* _x0, const T* _x1, const T* _u, T* _r) const
  {
    Multirotor mr(drag_term_, hover_throttle_);
    Matrix<T,10,1> x1hatbuf;
    Multirotor::State<T> x1hat(x1hatbuf.data());

    Multirotor::State<T> x0(_x0);
    Multirotor::State<T> x1(_x1);
    Multirotor::Input<T> u(_u);

    mr.F(x0, u, dt_, x1hat);

    Map<Matrix<T,13,1>> res(_r);
    res.template segment<6>(0) = x1.T - x1hat.T;
    res.template segment<3>(6) = x1.v - x1hat.v;
    Matrix<T,4,1> eq_input_{(T)0, (T)0, (T)0, (T)hover_throttle_};
    res.template segment<4>(9) = u.arr - eq_input_;
    return true;
  }

private:
  double dt_;
  double drag_term_;
  double hover_throttle_;
  Vector4d input_weight_;
  Vector9d state_weight_;
};

typedef ceres::AutoDiffCostFunction<DynamicsCostFunction, 13, 10, 10, 4> DynamicsFactor;


class PositionConstraintCostFunction
{
public:
  PositionConstraintCostFunction(const Vector3d& pos, double vel, const Vector3d& pos_weight, double vel_cost)
  {
    pos_ = pos;
    vel_ = vel;
    pos_weight_ = pos_weight.asDiagonal();
    vel_weight_ = vel_cost;
  }

  template <typename T>
  bool operator() (const T* _x, T* _r) const
  {
    Multirotor::State<T> x(_x);
    Map<Matrix<T,4,1>> res(_r);
    res.template segment<3>(0) = pos_weight_*(x.p - pos_);

    if (x.v.norm() > 1e-8)
//      res(3,0) = (T)vel_weight_ * (x.v.norm() - (T)vel_);
      res(3,0) = x.v.norm() - (T)vel_;
    else
      res(3,0) = (T)0.0;
    return true;
  }

private:
  Vector3d pos_;
  Matrix3d pos_weight_;
  double vel_;
  double vel_weight_;
};
typedef ceres::AutoDiffCostFunction<PositionConstraintCostFunction, 4, 10> ConstraintFactor;

struct NodePlus {
  template<typename T>
  bool operator()(const T* _x, const T* _d, T* _xp) const
  {
    Multirotor::State<T> x(_x);
    Multirotor::dState<T> d(_d);
    Multirotor::State<T> xp(_xp);
    xp.T = x.T + d.T;
    xp.v = x.v + d.v;
    return true;
  }
};
typedef ceres::AutoDiffLocalParameterization<NodePlus, 10, 9> NodeLocalParam;
