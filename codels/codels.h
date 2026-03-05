/*
 * Copyright (c) 2016-2018,2021-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Tue Mar 22 2016
 */
#ifndef H_my_first_genom_CODELS
#define H_my_first_genom_CODELS

#include <aio.h>
#include <sys/time.h>

#include "my_first_genom_c_types.h"

/* ---- Constants --------------------------------------------------------- */

/* Maximum number of joints supported by wholebody controller */
#define MY_FIRST_GENOM_MAX_JOINTS        8

/* Physical constants */
#define MY_FIRST_GENOM_GRAVITY           9.81

/* State timeout for emergency detection (seconds) */
#define MY_FIRST_GENOM_STATE_TIMEOUT_S   0.5

/* Propeller velocity squared limits for wholebody controller */
#define MY_FIRST_GENOM_PROP_VEL_MIN_SQ   400.0    /* 20^2 rad/s */
#define MY_FIRST_GENOM_PROP_VEL_MAX_SQ   12100.0  /* 110^2 rad/s */

/* ---- Error codes ------------------------------------------------------- */

enum my_first_genome {
  my_first_genom_EOK =	0,
  my_first_genom_ETS =	1 << 0,
  my_first_genom_EPOS =	1 << 1,
  my_first_genom_EATT =	1 << 2,
  my_first_genom_EVEL =	1 << 3,
  my_first_genom_EAVEL =	1 << 4,
};

#ifdef __cplusplus
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Core>

struct my_first_genom_pinocchio_s {
  pinocchio::Model *model;
  pinocchio::Data *data;
  bool loaded;

  /* Pre-allocated vectors to avoid allocation in hot path */
  Eigen::VectorXd qd_cache;      /* desired configuration */
  Eigen::VectorXd error_cache;   /* configuration error */
  Eigen::VectorXd Kp_cache;      /* gain vectors */
  Eigen::VectorXd Kd_cache;
  Eigen::VectorXd tau_cache;     /* torque output */
};
#else
struct my_first_genom_pinocchio_s;
#endif

#ifdef __cplusplus
extern "C" {
#endif

  void	my_first_genom_controller_init(const my_first_genom_ids_body_s *body,
                const my_first_genom_ids_servo_s *servo);
  int	my_first_genom_controller(const my_first_genom_ids_body_s *body,
                const my_first_genom_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_rigid_body_state *desired,
                const or_wrench_estimator_state *exwrench,
                my_first_genom_log_s *log,
                or_rotorcraft_rotor_control *wprop);
  int	my_first_genom_state_check(const struct timeval now,
                const my_first_genom_ids_servo_s *servo,
                or_pose_estimator_state *state);
  void	my_first_genom_reference_check(const struct timeval now,
                or_rigid_body_state *reference);
  int	my_first_genom_wrench(const my_first_genom_ids_body_s *body,
                const or_pose_estimator_state *state,
                const double wprop[or_rotorcraft_max_rotors],
                double wrench[6]);

  genom_event
	my_first_genom_gtmrp_allocmatrix(int rotors, double cx, double cy, double cz,
                double armlen, double rx, double ry, double rz, double cf,
                double ct, double G[6 * or_rotorcraft_max_rotors],
                const genom_context self);
  genom_event
	my_first_genom_inertia(int rotors, double armlen, double mass,
                double mbodyw, double mbodyh, double mmotor, double J[3 * 3],
                const genom_context self);
  genom_event
	my_first_genom_scale_inertia(double s, double J[3 * 3], const genom_context self);

  void	my_first_genom_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                double iG[or_rotorcraft_max_rotors * 6]);
  void	my_first_genom_wrench_bounds(const double G[6 * or_rotorcraft_max_rotors],
                const double wmin, const double wmax, double fmin[6],
                double fmax[6]);

  int	my_first_genom_wholebody_controller(
                my_first_genom_pinocchio_s *pinocchio,
                const my_first_genom_ids_wholebody_s *wholebody,
                const double q[],
                const double v[],
                double tau[]);

  /* Check if pinocchio model is loaded (C-safe accessor) */
  int	my_first_genom_pinocchio_is_loaded(const my_first_genom_pinocchio_s *pinocchio);

  /* Reset position controller integrator (call on mode switch) */
  void	my_first_genom_controller_reset_integrator(void);

  /* Helper to write joint efforts to output port */
  void	my_first_genom_write_joint_efforts(or_joint_input *joint_data,
                const double *efforts, size_t nj,
                const struct timeval *tv);

  /* Disable wholebody controller and revert to legacy */
  genom_event	my_first_genom_disable_wholebody(my_first_genom_ids_wholebody_s *wholebody,
                const genom_context self);

#ifdef __cplusplus
}
#endif

static inline genom_event
my_first_genom_e_sys_error(const char *s, genom_context self)
{
  my_first_genom_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) { /* ignore error*/; }
  return my_first_genom_e_sys(&d, self);
}

struct my_first_genom_log_s {
  int fd;
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define my_first_genom_logfmt	" %g "
# define my_first_genom_log_header_fmt                                            \
  "ts delay "                                                           \
  "fx fy fz tx ty tz "                                                  \
  "meas_fx meas_fy meas_fz meas_tx meas_ty meas_tz "                    \
  "xd yd zd rolld pitchd yawd vxd vyd vzd wxd wyd wzd axd ayd azd "     \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz "           \
  "sat_fz sat_tx sat_ty sat_tz "                                        \
  "wb_active wb_tau_fx wb_tau_fy wb_tau_fz wb_tau_tx wb_tau_ty wb_tau_tz"
# define my_first_genom_log_fmt                                                   \
  "%d.%09d %g "                                                           \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt                                   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt
};

#endif /* H_my_first_genom_CODELS */
