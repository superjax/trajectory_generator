#include "Eigen/Core"
#include "lin_alg_tools/care.h"
#include "geometry/xform.h"

using namespace Eigen;

class LQR
{
  typedef Matrix<double, 10, 1> Vector10d;
  typedef Matrix<double, 9, 9> Matrix9d;
  typedef Matrix<double, 9, 4> Matrix94d;
  typedef Matrix<double, 4, 9> Matrix49d;

public:
  enum
  {
    dPOS = 0,
    dATT = 3,
    dVEL = 6,

    POS = 0,
    ATT = 3,
    VEL = 7,

    OMEGA = 0,
    F = 3
  };

  LQR();
  bool computeControl(const Vector10d& x, const Vector10d& x_c, Vector4d& u);
  void setDragTerm(double drag_term) { drag_term_ = drag_term; }
  void setHoverThrottle(double hover_throttle) { hover_throttle_ = hover_throttle; }

  inline void setQ(Matrix9d& Q) { Q_ = Q; }
  inline void setR(Matrix4d& R) { R_ = R; Rinv_ = R.inverse();}

  void boxminus(const Vector10d& x1, const Vector10d& x2, Vector9d& dx) const;
  void inputSat(Vector9d& dx) const;
  void calcJacobian(const Vector10d& x);
private:

  double sat(double x, double max, double min) const;


  Matrix9d A_;
  Matrix94d B_;
  Matrix9d Q_;
  Matrix9d X_;
  Matrix4d R_;
  Matrix4d Rinv_;
  Vector9d dx_;
  Matrix49d K_;
  double drag_term_;
  double hover_throttle_;
  double max_pos_err_ = 1.0;
  double max_att_err_ = 0.3;
  double max_vel_err_ = 1.0;

  CareSolver<9, 4> care_solver_;
};
