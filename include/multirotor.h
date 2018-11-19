#pragma once

#include <Eigen/Core>

#include "geometry/xform.h"

using namespace Eigen;
using namespace xform;


template<typename Type>
struct dState
{
  Map<Matrix<Type, 13, 1>> arr;
  Map<Matrix<Type, 3, 1>> p;
  Map<Matrix<Type, 6, 1>> T;
  Map<Matrix<Type, 3, 1>> q;
  Map<Matrix<Type, 3, 1>> v;
  Map<Matrix<Type, 3, 1>> w;
  Type& F;

  dState(const Type* buf) :
    arr(const_cast<Type*>(buf)),
    p(const_cast<Type*>(buf)),
    T(const_cast<Type*>(buf)),
    q(const_cast<Type*>(buf+3)),
    v(const_cast<Type*>(buf+6)),
    w(const_cast<Type*>(buf+9)),
    F(*const_cast<Type*>(buf+12))
  {}
};

template<typename Type>
struct State
{
  Map<Matrix<Type, 14, 1>> arr;
  Map<Matrix<Type, 3, 1>> p;
  Xform<Type> T;
  Quat<Type> q;
  Map<Matrix<Type, 3, 1>> v;
  Map<Matrix<Type, 3, 1>> w;
  Type& F;

  State(const Type* buf) :
    arr(const_cast<Type*>(buf)),
    p(const_cast<Type*>(buf)),
    T(const_cast<Type*>(buf)),
    q(const_cast<Type*>(buf+3)),
    v(const_cast<Type*>(buf+7)),
    w(const_cast<Type*>(buf+10)),
    F(*const_cast<Type*>(buf+13))
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
