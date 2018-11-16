#include <Eigen/Core>


#include "geometry/xform.h"

using namespace Eigen;
using namespace xform;


class Multirotor
{
public:
  Multirotor();

  double cd_;

  template<typename T>
  struct State
  {
    Matrix<T, 10, 1> arr;
    Map<Matrix<T, 3, 1>> p;
    Xform T;
    Quat q;
    Map<Matrix<T, 3, 1>> v;

    State() :
      p(arr.data()),
      T(arr.data()),
      q(arr.data()+3),
      v(arr.data()+7)
    {}
  };

  template<typename T>
  struct dState
  {
    Matrix<T, 9, 1> arr;
    Map<Matrix<T, 3, 1>> p;
    Map<Matrix<T, 6, 1>> T;
    Map<Matrix<T, 3, 1>> q;
    Map<Matrix<T, 3, 1>> v;

    State() :
      p(arr.data()),
      T(arr.data()),
      q(arr.data()+3),
      v(arr.data()+6)
    {}
  };

  template<typename T>
  struct Input
  {
    Matrix<T, 4, 1> arr;
    Map<Matrix, 3, 1> w;
    Map<Matrix, 1, 1> F;
    Input() :
      w(arr.data()),
      F(arr.data()+3)
    {}
  };

  template<typename T>
  void f(const State<T>& x, const Input<T>& u, dState<T>& dx)
  {
    static const Matrix<T,3,1> ez = (Matrix<T,3,1>() << 0, 0, (T)1.0).finished();
    static const Matrix<T,3,3> Ones_UL =(Matrix<T,3,3>() << (T)1.0, 0, 0,  0, (T)1.0, 0,  0, 0, 0).finished();
    static const Matrix<T,3,1> gravity = (Matrix<T,3,1>() << 0, 0, (T)9.80665).finished();

    dx.p = x.q.rotp(x.v);
    dx.q = u.w;
    dx.v = ez * u.F * x.q.rotp(gravity) - (T)cd_ * Ones_UL * x.v - u.w.cross(x.v);
  }

};
