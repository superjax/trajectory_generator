#include <gtest/gtest.h>
#include <Eigen/Core>

#include "trajectory.h"

using namespace Eigen;

TEST (TrajOpt, SinglePolynomial_SingleSegment)
{
  Vector4d c_poly = Vector4d::Ones() * 0.0;
  Vector4d d0 = Vector4d::Ones() * NAN;
  Vector4d df = Vector4d::Ones() * NAN;

  d0(0) = 0;
  df(0) = 1;

  d0(0) = 0;
  df(0) = 0;
}
