#include "lqr.h"


LQR::LQR()
{
  hover_throttle_ = 0.5;
  drag_term_ = 0.2;
  Q_.setIdentity();
  R_.setIdentity();
  Rinv_ = R_.inverse();
}

void LQR::boxminus(const Vector10d &x1, const Vector10d &x2, Vector9d &dx) const
{
  dx.segment<3>(dPOS) = x1.segment<3>(POS) - x2.segment<3>(POS);
  dx.segment<3>(dATT) = Quatd(x1.segment<4>(ATT)) - Quatd(x2.segment<4>(ATT));
  dx.segment<3>(dVEL) = x1.segment<3>(VEL) - x2.segment<3>(VEL);
}

bool LQR::computeControl(const Vector10d &x, const Vector10d &x_c, Vector4d &u)
{
  boxminus(x_c, x, dx_);
  calcJacobian(x);
  care_solver_.solve(X_, A_, B_, Q_, R_);
  K_ = Rinv_ * B_.transpose() * X_;
  u = K_ * dx_;
  return true;
}

void LQR::calcJacobian(const Vector10d &x)
{
  Quatd q_I_b(x.data() + ATT);
  Map<const Vector3d> vel(x.data() + VEL);
  Matrix3d R_I_B = q_I_b.R();
  static const Vector3d gravity{0.0, 0.0, 9.80665};


  A_.block<3,3>(dPOS, dPOS).setZero();
  A_.block<3,3>(dPOS, dATT) = -R_I_B.transpose() * skew(vel);
  A_.block<3,3>(dPOS, dVEL) = R_I_B.transpose();

  A_.block<3,3>(dATT, dPOS).setZero();
  A_.block<3,3>(dATT, dATT).setZero();
  A_.block<3,3>(dATT, dVEL).setZero();

  A_.block<3,3>(dVEL, dPOS).setZero();
  A_.block<3,3>(dVEL, dATT) = skew(R_I_B * gravity);
  A_.block<3,3>(dVEL, dVEL) = -I_3x3 * drag_term_;



  B_.block<3,3>(dPOS, OMEGA).setZero();
  B_.block<3,3>(dATT, OMEGA) = I_3x3;
  B_.block<3,3>(dVEL, OMEGA).setZero();

  B_.block<3,1>(dPOS, F).setZero();
  B_.block<3,1>(dVEL, F) = -e_z; // hover throttle?
  B_.block<3,1>(dATT, F).setZero();
}
