#include <ceres/ceres.h>
#include <Eigen/Core>

#include "geometry/xform.h"
#include "multirotor.h"

using namespace Eigen;
using namespace xform;


class DynamicsCostFunction
{
public:
  DynamicsCostFunction(const double& dt, const double& drag_term, const double& hover_throttle,
                       const Vector9d& state_weight, const Vector4d& input_weight,
                       const double& tau_wxy, const double& tau_wz, const double& tau_F) :
    dt_{dt},
    cd_{drag_term},
    hover_throttle_{hover_throttle},
    tau_wxy_{tau_wxy},
    tau_wz_{tau_wz},
    tau_F_{tau_F},
    input_weight_{const_cast<double*>(input_weight.data())},
    state_weight_{const_cast<double*>(state_weight.data())}
  {}

  template <typename T>
  inline T sat(const T& a, const double& max, const double& min) const
  {
    return (a > max) ? (T)max : (a < min) ? (T)min : a;
  }

  template<typename T>
  void F(State<T>& x, Input<T>& u, double dt, State<T>& xp) const
  {
    typedef Matrix<T,3,1> Vec3;
    typedef Matrix<T,3,3> Mat3;
    typedef Matrix<T,2,1> Vec2;
    typedef Matrix<T,1,1> Vec1;

    static const Vec3 ez = (Vec3() << (T)0, (T)0, (T)1.0).finished();
    static const Mat3 Ones_UL =(Mat3() << (T)1.0, (T)0, (T)0,  (T)0, (T)1.0, (T)0,  (T)0, (T)0, (T)0).finished();

    Matrix<T,9,1> dxbuf;
    dState<T> dx(dxbuf.data());

    volatile Matrix<T,3,1> vdebug =ez * (u.F/(T)hover_throttle_) + x.q.rotp(ez) - ((T)cd_ * Ones_UL * x.v - u.w.cross(x.v)) * (T)(1.0/9.80665);

    dx.p = x.q.rotp(x.v);
    dx.q = x.w;
    dx.v = ez * (x.F/(T)hover_throttle_) + x.q.rotp(ez) - ((T)cd_ * Ones_UL * x.v - x.w.cross(x.v)) * (T)(1.0/9.80665);

    dx.T *= (T)dt;
    dx.v *= (T)dt;

    xp.T = x.T + dx.T;
    xp.v = x.v + dx.v;

    T alpha_wxy = (T)(dt/(tau_wxy_ + dt));
    T alpha_wz = (T)(dt/(tau_wz_ + dt));
    T alpha_F = (T)(dt/(tau_F_ + dt));

    xp.w(0) = ((T)1.0 - alpha_wxy)* x.w(0) + alpha_wxy*u.w(0);
    xp.w(1) = ((T)1.0 - alpha_wxy)* x.w(1) + alpha_wxy*u.w(1);
    xp.w(2) = ((T)1.0 - alpha_wz)* x.w(2) + alpha_wz*u.w(2);
    xp.F = ((T)1.0 - alpha_F)* x.F + alpha_F*u.F;
  }

  template <typename T>
  bool operator() (const T* _x0, const T* _x1, const T* _u, T* _r) const
  {
    Matrix<T,14,1> x1hatbuf;
    State<T> x1hat(x1hatbuf.data());


    State<T> x0(_x0);
    State<T> x1(_x1);
    Input<T> u(_u);
    F(x0, u, dt_, x1hat);

    Map<Matrix<T,17,1>> res(_r);
    res.template segment<6>(0) = x1.T - x1hat.T;
    res.template segment<3>(6) = x1.v - x1hat.v;
    res.template segment<3>(9) = x1.w - x1hat.w;
    res(12) = x1.F - x1hat.F;
    Matrix<T,4,1> eq_input_{(T)0, (T)0, (T)0, (T)hover_throttle_};
    res.template segment<4>(13) = u.arr - eq_input_;

    Matrix<T,14,1> x0debug = x0.arr;
    Matrix<T,14,1> x1debug = x1.arr;
    Matrix<T,4,1> udebug = u.arr;
    Matrix<T,17,1> resdebug = res;
    return true;
  }

private:
  const double& dt_;
  const double& cd_;
  const double& hover_throttle_;
  const double& tau_wxy_;
  const double& tau_wz_;
  const double& tau_F_;
  const Map<Vector4d> input_weight_;
  const Map<Vector9d> state_weight_;
};
typedef ceres::AutoDiffCostFunction<DynamicsCostFunction, 17, 14, 14, 4> DynamicsFactor;


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
    State<T> x(_x);
    Map<Matrix<T,4,1>> res(_r);
    res.template segment<3>(0) = pos_weight_*(x.p - pos_);

    if (x.v.norm() > 1e-8)
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
typedef ceres::AutoDiffCostFunction<PositionConstraintCostFunction, 4, 14> ConstraintFactor;

struct NodePlus {
  template<typename T>
  bool operator()(const T* _x, const T* _d, T* _xp) const
  {
    State<T> x(_x);
    dState<T> d(_d);
    State<T> xp(_xp);
    xp.T = x.T + d.T;
    xp.v = x.v + d.v;
    xp.w = x.w + d.w;
    xp.F = x.F + d.F;
    return true;
  }
};
typedef ceres::AutoDiffLocalParameterization<NodePlus, 14, 13> NodeLocalParam;
