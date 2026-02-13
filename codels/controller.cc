#include "acmy_first_genom.h"

#include <aio.h>
#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "proxsuite/proxqp/dense/dense.hpp"

#include "codels.h"

/*
 * --- my_first_genom_controller_init -------------------------------------
 */

static proxsuite::proxqp::dense::QP<double>
  my_first_genom_wrenchsat(4, 0, 0);

void
my_first_genom_controller_init(const my_first_genom_ids_body_s *body,
                               const my_first_genom_ids_servo_s *servo)
{
  using namespace proxsuite::proxqp;
  using namespace Eigen;

  my_first_genom_wrenchsat =
    dense::QP<double>(4, 0, body->rotors, HessianType::Diagonal);

  my_first_genom_wrenchsat.settings.eps_abs = 1e-3;
  my_first_genom_wrenchsat.settings.initial_guess =
    InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
  my_first_genom_wrenchsat.settings.verbose = false;

  const Vector4d g(
    - servo->satweight.thrust,
    - servo->satweight.tilt,
    - servo->satweight.tilt,
    - servo->satweight.head);

  const Matrix4d H = (-g).asDiagonal();

  VectorXd l(body->rotors), u(body->rotors);
  l.array() = body->wmin * std::fabs(body->wmin);
  u.array() = body->wmax * std::fabs(body->wmax);

  const Map<
    const Matrix<double, Dynamic, 6, RowMajor>
  > iG(body->iG, body->rotors, 6);

  MatrixXd C(body->rotors, 4);
  C.setZero();
  C.col(0).noalias() =
    iG.col(2) * (body->wrench_min[2] + body->wrench_max[2])/2.;

  my_first_genom_wrenchsat.init(
    H, g,
    proxsuite::nullopt, proxsuite::nullopt,
    C, l, u);
}

/*
 * --- my_first_genom_controller ------------------------------------------
 */

int
my_first_genom_controller(
    const my_first_genom_ids_body_s *body,
    const my_first_genom_ids_servo_s *servo,
    const or_pose_estimator_state *state,
    const or_rigid_body_state *reference,
    const or_wrench_estimator_state *mwrench,
    my_first_genom_log_s *log,
    or_rotorcraft_rotor_control *wprop)
{
  using namespace Eigen;

  Matrix3d Rd;
  Quaternion<double> qd;
  Vector3d xd, vd, wd, ad, jd;

  Matrix3d R;
  Quaternion<double> q;
  Vector3d x, v, w;

  Vector3d ex, ev, eR, ew;
  static Vector3d Iex;

  double c;
  Vector3d f;
  Matrix<double, 6, 1> wrench;

  Map< Matrix<double, or_rotorcraft_max_rotors, 1> >
    wprop_(wprop->_buffer);

  size_t i;

  const Map<
    const Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor>
  > iG_(body->iG);

  const Array3d Kp(
    servo->gain.Kpxy, servo->gain.Kpxy, servo->gain.Kpz);
  const Array3d Ki(
    servo->gain.Kixy, servo->gain.Kixy, servo->gain.Kiz);
  const Array3d Kv(
    servo->gain.Kvxy, servo->gain.Kvxy, servo->gain.Kvz);
  const Array3d Kq(
    servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Array3d Kw(
    servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);

  xd << reference->pos._value.x,
        reference->pos._value.y,
        reference->pos._value.z;

  qd.coeffs() <<
    reference->att._value.qx,
    reference->att._value.qy,
    reference->att._value.qz,
    reference->att._value.qw;

  vd << reference->vel._value.vx,
        reference->vel._value.vy,
        reference->vel._value.vz;

  wd << reference->avel._value.wx,
        reference->avel._value.wy,
        reference->avel._value.wz;

  ad << reference->acc._value.ax,
        reference->acc._value.ay,
        reference->acc._value.az;

  jd << reference->jerk._value.jx,
        reference->jerk._value.jy,
        reference->jerk._value.jz;

  if (reference->intrinsic) {
    vd = qd * vd;
    wd = qd * wd;
    ad = qd * ad;
    jd = qd * jd;
  }

  if (state->pos._present)
    x << state->pos._value.x,
         state->pos._value.y,
         state->pos._value.z;
  else {
    x = xd;
    ad = Vector3d(0, 0, - servo->emerg.descent);
    Iex.setZero();
  }

  if (state->att._present)
    q.coeffs() <<
      state->att._value.qx,
      state->att._value.qy,
      state->att._value.qz,
      state->att._value.qw;
  else
    q = qd;

  R = q.matrix();

  if (state->vel._present)
    v << state->vel._value.vx,
         state->vel._value.vy,
         state->vel._value.vz;
  else
    v = vd;

  if (state->avel._present)
    w << state->avel._value.wx,
         state->avel._value.wy,
         state->avel._value.wz;
  else
    w = wd;

  /* Gravity compensation */
  ad = Vector3d(0, 0, 9.81) + ad;
  c = ad.norm();

  c > 1e-4 ? Rd.col(2) = ad / c : Rd.col(2) = R.col(2);
  Rd.col(1) =
    Rd.col(2).cross(qd.matrix().col(0)).normalized();
  Rd.col(0) =
    Rd.col(1).cross(Rd.col(2));

  ex = x - xd;
  ev = v - vd;

  f = - Kp * ex.array()
      - Kv * ev.array()
      - Ki * Iex.array()
      + body->mass * ad.array();

  Rd.col(2) = f.normalized();
  Rd.col(1) =
    Rd.col(2).cross(Rd.col(0)).normalized();
  Rd.col(0) =
    Rd.col(1).cross(Rd.col(2));

  Matrix3d E(0.5 * (Rd.transpose()*R - R.transpose()*Rd));
  eR <<
    (E(2,1) - E(1,2))/2.,
    (E(0,2) - E(2,0))/2.,
    (E(1,0) - E(0,1))/2.;

  ew.noalias() = R.transpose() * (w - wd);

  wrench.block<3,1>(0,0) << 0., 0., f.dot(R.col(2));
  wrench.block<3,1>(3,0) =
    - Kq * eR.array()
    - Kw * ew.array();

  wprop->_length = body->rotors;
  wprop_.noalias() = iG_ * wrench;

  wprop_.head(body->rotors) =
    wprop_.head(body->rotors).unaryExpr(
      [body](double w2) {
        double w =
          std::copysign(
            std::sqrt(std::fabs(w2)), w2);
        return w < body->wmin ?
          body->wmin :
          w > body->wmax ?
          body->wmax : w;
      });

  return 0;
}
