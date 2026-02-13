/*
 * Adapted from nhfc to my_first_genom
 */

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

/* --- my_first_genom_gtmrp_allocmatrix ---------------------------------- */

genom_event
my_first_genom_gtmrp_allocmatrix(int rotors,
                                 double cx, double cy, double cz,
                                 double armlen, double rx, double ry, double rz,
                                 double cf, double ct,
                                 double G[6 * or_rotorcraft_max_rotors],
                                 const genom_context self)
{
  using namespace Eigen;

  Map< Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Vector3d z, p;
  int i, sign;

  if (rotors < 0 || rotors > or_rotorcraft_max_rotors) {
    my_first_genom_e_inval_detail d = { "bad number of rotors" };
    return my_first_genom_e_inval(&d, self);
  }

  for(i = 0, sign = 1; i < rotors; i++, sign = -sign) {

    z =
      (AngleAxisd(2 * i * M_PI / rotors, Vector3d::UnitZ())
       * AngleAxisd(sign * rx, Vector3d::UnitX())
       * AngleAxisd(ry, Vector3d::UnitY())).matrix().col(2);

    p =
      armlen *
      AngleAxisd(2 * i * M_PI / rotors, Vector3d::UnitZ()).matrix().col(0)
      + Vector3d(cx, cy, cz);

    G_.col(i) <<
      cf * z,
      cf * p.cross(z) - sign * rz * ct * z;
  }

  for(; i < or_rotorcraft_max_rotors; i++)
    G_.col(i) << Vector3d::Zero(), Vector3d::Zero();

  return genom_ok;
}


/* --- my_first_genom_invert_G ------------------------------------------ */

void
my_first_genom_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                        double iG[or_rotorcraft_max_rotors * 6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor> > iG_(iG);

  iG_ = G_.
    jacobiSvd(ComputeFullU | ComputeFullV).
    solve(Matrix<double, 6, 6>::Identity());
}


/* --- my_first_genom_Gw2 ------------------------------------------------ */

void
my_first_genom_Gw2(const double G[6 * or_rotorcraft_max_rotors],
                   const double w,
                   double f[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  Map< Matrix<double, 6, 1> > f_(f);

  f_ = G_ * Matrix<double, or_rotorcraft_max_rotors, 1>::Constant(w * w);
}


/* --- my_first_genom_wrench_bounds ------------------------------------- */

void
my_first_genom_wrench_bounds(const double G[6 * or_rotorcraft_max_rotors],
                             const double wmin, const double wmax,
                             double fmin[6], double fmax[6])
{
  using namespace Eigen;

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G_(G);
  const double w2min = std::copysign(wmin * wmin, wmin);
  const double w2max = std::copysign(wmax * wmax, wmax);
  int i;

  for(i = 0; i < 6; i++) {
    fmin[i] = G_.row(i) * G_.row(i).unaryExpr([=](double g) {
      return g >= 0. ? w2min : w2max;
    }).transpose();

    fmax[i] = G_.row(i) * G_.row(i).unaryExpr([=](double g) {
      return g >= 0. ? w2max : w2min;
    }).transpose();
  }
}


/* --- my_first_genom_inertia ------------------------------------------- */

genom_event
my_first_genom_inertia(int rotors, double armlen,
                       double mass, double mbodyw, double mbodyh, double mmotor,
                       double J[3 * 3], const genom_context self)
{
  using namespace Eigen;

  Map<Matrix3d> J_(J);
  double bmass, izz;

  bmass = mass - rotors * mmotor;
  if (bmass <= 0.) {
    my_first_genom_e_inval_detail d = {
      "total motor mass greater than body mass"
    };
    return my_first_genom_e_inval(&d, self);
  }

  J_ = Vector3d(
    1/12. * bmass * (mbodyw * mbodyw + mbodyh * mbodyh),
    1/12. * bmass * (mbodyw * mbodyw + mbodyh * mbodyh),
    1/6. * bmass * mbodyw * mbodyw).asDiagonal();

  izz = rotors * mmotor * armlen * armlen;

  J_ += Vector3d(
    izz / 2.,
    izz / 2.,
    izz).asDiagonal();

  return genom_ok;
}


/* --- my_first_genom_scale_inertia ------------------------------------- */

genom_event
my_first_genom_scale_inertia(double s,
                             double J[3 * 3],
                             const genom_context self)
{
  using namespace Eigen;

  Map<Matrix3d> J_(J);
  J_ *= s;

  return genom_ok;
}
