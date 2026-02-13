#ifndef H_MY_FIRST_GENOM_CODELS
#define H_MY_FIRST_GENOM_CODELS

#include <aio.h>
#include "my_first_genom_c_types.h"

enum my_first_genome {
  MY_FIRST_GENOM_EOK   = 0,
  MY_FIRST_GENOM_ETS   = 1 << 0,
  MY_FIRST_GENOM_EPOS  = 1 << 1,
  MY_FIRST_GENOM_EATT  = 1 << 2,
  MY_FIRST_GENOM_EVEL  = 1 << 3,
  MY_FIRST_GENOM_EAVEL = 1 << 4,
};

#ifdef __cplusplus
extern "C" {
#endif

/* Controller core */
void my_first_genom_controller_init(
    const my_first_genom_ids_body_s *body,
    const my_first_genom_ids_servo_s *servo);

int my_first_genom_controller(
    const my_first_genom_ids_body_s *body,
    const my_first_genom_ids_servo_s *servo,
    const or_pose_estimator_state *state,
    const or_rigid_body_state *desired,
    const or_wrench_estimator_state *exwrench,
    my_first_genom_log_s *log,
    or_rotorcraft_rotor_control *wprop);

int my_first_genom_state_check(
    const struct timeval now,
    const my_first_genom_ids_servo_s *servo,
    or_pose_estimator_state *state);

void my_first_genom_reference_check(
    const struct timeval now,
    or_rigid_body_state *reference);

int my_first_genom_wrench(
    const my_first_genom_ids_body_s *body,
    const or_pose_estimator_state *state,
    const double wprop[or_rotorcraft_max_rotors],
    double wrench[6]);

/* Geometry helpers */
genom_event
my_first_genom_gtmrp_allocmatrix(
    int rotors, double cx, double cy, double cz,
    double armlen, double rx, double ry, double rz,
    double cf, double ct,
    double G[6 * or_rotorcraft_max_rotors],
    const genom_context self);

genom_event
my_first_genom_inertia(
    int rotors, double armlen, double mass,
    double mbodyw, double mbodyh, double mmotor,
    double J[3 * 3],
    const genom_context self);

genom_event
my_first_genom_scale_inertia(
    double s,
    double J[3 * 3],
    const genom_context self);

void my_first_genom_invert_G(
    const double G[6 * or_rotorcraft_max_rotors],
    double iG[or_rotorcraft_max_rotors * 6]);

void my_first_genom_wrench_bounds(
    const double G[6 * or_rotorcraft_max_rotors],
    const double wmin, const double wmax,
    double fmin[6], double fmax[6]);

#ifdef __cplusplus
}
#endif


/* ----------------------------------------------------------- */
/* System error helper                                         */
/* ----------------------------------------------------------- */

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
  strerror_r(d.code, d.what + l, sizeof(d.what) - l);

  return my_first_genom_e_sys(&d, self);
}


/* ----------------------------------------------------------- */
/* Logging structure                                           */
/* ----------------------------------------------------------- */

struct my_first_genom_log_s {
  int fd;
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define my_first_genom_logfmt " %g "

# define my_first_genom_log_header_fmt                               \
  "ts delay "                                                         \
  "fx fy fz tx ty tz "                                                \
  "meas_fx meas_fy meas_fz meas_tx meas_ty meas_tz "                  \
  "xd yd zd rolld pitchd yawd vxd vyd vzd wxd wyd wzd axd ayd azd "   \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz "         \
  "sat_fz sat_tx sat_ty sat_tz"

# define my_first_genom_log_fmt                                       \
  "%d.%09d %g "                                                        \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt   \
  my_first_genom_logfmt my_first_genom_logfmt my_first_genom_logfmt
};

#endif /* H_MY_FIRST_GENOM_CODELS */
