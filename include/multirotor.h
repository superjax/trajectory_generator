#pragma once

#include <Eigen/Core>

#include "geometry/xform.h"

using namespace Eigen;
using namespace xform;


class Multirotor
{
public:
  Multirotor(double drag_term, double hover_throttle)
  {
    cd_ = drag_term;
    hover_throttle_ = hover_throttle;
  }

  double cd_;
  double hover_throttle_;

  template<typename Type>
  struct dState
  {
    Map<Matrix<Type, 3, 1>> p;
    Map<Matrix<Type, 6, 1>> T;
    Map<Matrix<Type, 3, 1>> q;
    Map<Matrix<Type, 3, 1>> v;

    dState(const Type* buf) :
      p(const_cast<Type*>(buf)),
      T(const_cast<Type*>(buf)),
      q(const_cast<Type*>(buf+3)),
      v(const_cast<Type*>(buf+6))
    {}
  };

  template<typename Type>
  struct State
  {
    Map<Matrix<Type, 3, 1>> p;
    Xform<Type> T;
    Quat<Type> q;
    Map<Matrix<Type, 3, 1>> v;

    State(const Type* buf) :
      p(const_cast<Type*>(buf)),
      T(const_cast<Type*>(buf)),
      q(const_cast<Type*>(buf+3)),
      v(const_cast<Type*>(buf+7))
    {}
  };

  template<typename T>
  struct Input
  {
    Map<Matrix<T, 4, 1>> arr;
    Map<Matrix<T, 3, 1>> w;
    T& F;
    Input(const T* buf) :
      arr(const_cast<T*>(buf)),
      w(const_cast<T*>(buf)),
      F(*const_cast<T*>(buf+3))
    {}
  };

  template<typename T>
  void f(const State<T>& x, const Input<T>& u, dState<T>& dx) const
  {
    static const Matrix<T,3,1> ez = (Matrix<T,3,1>() << (T)0, (T)0, (T)1.0).finished();
    static const Matrix<T,3,3> Ones_UL =(Matrix<T,3,3>() << (T)1.0, (T)0, (T)0,  (T)0, (T)1.0, (T)0,  (T)0, (T)0, (T)0).finished();

    dx.p = x.q.rotp(x.v);
    dx.q = u.w;
    dx.v = ez * (u.F/(T)hover_throttle_) + x.q.rotp(ez) - ((T)cd_ * Ones_UL * x.v - u.w.cross(x.v)) * (T)(1.0/9.80665);
  }

  template<typename T>
  void F(State<T>& x, Input<T>& u, double dt, State<T>& xp) const
  {
    Matrix<T,9,1> dxbuf;
    dState<T> dx(dxbuf.data());
    f(x, u, dx);
    dx.T *= (T)dt;
    dx.v *= (T)dt;
    xp.T = x.T + dx.T;
    xp.v = x.v + dx.v;
  }
};
