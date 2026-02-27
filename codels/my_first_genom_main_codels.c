/*
 * Copyright (c) 2015-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *					Anthony Mallet on Tue Aug 11 2015
 */
#include "acmy_first_genom.h"

#include <sys/time.h>
#include <err.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "my_first_genom_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */

/** Codel my_first_genom_main_start of task main.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_init.
 */
genom_event
my_first_genom_main_start(my_first_genom_ids *ids, const my_first_genom_rotor_input *rotor_input,
                const genom_context self)
{
  ids->body = (my_first_genom_ids_body_s){
    .wmax = 100., .wmin = 16.,
    .init = false
  };

  ids->servo = (my_first_genom_ids_servo_s){
    .sat = { .x = 0.10, .v = 0.1, .ix = 0.10 },
    .satweight = {
      .enable = true,
      .thrust = 10.0, .tilt = 100.0, .head = 1.0
    },
    .gain = {
      .Kpxy = 14., .Kvxy = 7., .Kpz = 20., .Kvz = 10.,
      .Kqxy = 2.3, .Kwxy = .23, .Kqz = .2, .Kwz = .02,
      .Kixy = 0., .Kiz = 0.
    },

    .att_mode = my_first_genom_tilt_prioritized,

    .ramp = 3,
    .scale = 0.,

    .emerg = {
      .descent = .1,
      .dx = 0.05 * 0.05 /9.,
      .dq = 5. * 5. * M_PI*M_PI/180./180./9.,
      .dv = 0.2 * 0.2 /9.,
      .dw = 20. * 20. * M_PI*M_PI/180./180./9.
    }
  };

  ids->reference = (or_rigid_body_state){
    .pos._present = false,
    .att._present = false, .att._value = { .qw = 1. },
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false
  };


  /* init controller */
  my_first_genom_controller_init(&ids->body, &ids->servo);

  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (my_first_genom_log_s){
    .fd = -1,
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  return my_first_genom_init;
}


/** Codel my_first_genom_main_init of task main.
 *
 * Triggered by my_first_genom_init.
 * Yields to my_first_genom_pause_init, my_first_genom_pause_control.
 */
genom_event
my_first_genom_main_init(or_rigid_body_state *reference,
               const my_first_genom_ids_body_s *body, const my_first_genom_state *state,
               const my_first_genom_rotor_measure *rotor_measure,
               const my_first_genom_rotor_input *rotor_input,
               const my_first_genom_wrench_measure *wrench_measure,
               const genom_context self)
{
  or_pose_estimator_state *state_data;
  or_rotorcraft_input *input_data;
  struct timeval tv;
  int i;

  state_data = state->data(self);
  
  if (!state_data) {
    warnx("[init] state: NULL");
  } else {
    warnx("[init] state ts=%d.%09d pos=%d att=%d vel=%d avel=%d",
          state_data->ts.sec, state_data->ts.nsec,
          state_data->pos._present, state_data->att._present,
          state_data->vel._present, state_data->avel._present);

    if (state_data->pos._present) {
      warnx("[init] pos: x=%g y=%g z=%g",
            state_data->pos._value.x,
            state_data->pos._value.y,
            state_data->pos._value.z);
    }

    if (state_data->att._present) {
      warnx("[init] att: qw=%g qx=%g qy=%g qz=%g",
            state_data->att._value.qw,
            state_data->att._value.qx,
            state_data->att._value.qy,
            state_data->att._value.qz);
    }
  }
  
  /* output zero (minimal) velocity */
  input_data = rotor_input->data(self);
  if (!input_data) return my_first_genom_pause_init;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  input_data->desired._length = body->rotors;
  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self);

  /* wait for geometry */
  if (!body->init) return my_first_genom_pause_init;

  /* update measured wrench */
  if (!state->read(self))
    my_first_genom_main_measure(body, state, rotor_measure, wrench_measure, self);

  /* switch to servo mode upon reception of the first valid position or
   * velocity. Ensure that state has valid pos/att and initialize any empty
   * pos/att reference to it. */
  if (!reference->pos._present && !reference->vel._present)
    return my_first_genom_pause_init;

  if (state_data && state_data->pos._present && state_data->att._present) {
    if (!reference->pos._present)
      reference->pos._value = state_data->pos._value;
    if (!reference->att._present)
      reference->att._value = state_data->att._value;

    return my_first_genom_pause_control;
  }

  return my_first_genom_pause_init;
}


/** Codel my_first_genom_main_control of task main.
 *
 * Triggered by my_first_genom_control.
 * Yields to my_first_genom_measure, my_first_genom_emergency.
 */
genom_event
my_first_genom_main_control(const my_first_genom_ids_body_s *body, my_first_genom_ids_servo_s *servo,
                  const my_first_genom_state *state,
                  const my_first_genom_wrench_measure *wrench_measure,
                  or_rigid_body_state *reference, my_first_genom_log_s **log,
                  const my_first_genom_rotor_input *rotor_input,
                  const genom_context self)
{
  or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *wrench_data = wrench_measure->data(self);
  or_rotorcraft_input *input_data = rotor_input->data(self);
  struct timeval tv;
  size_t i;
  int e;

  gettimeofday(&tv, NULL);

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    warnx("state unavailable");
    warnx("emergency descent");
    return my_first_genom_emergency;
  }

  /* check state */
  e = my_first_genom_state_check(tv, servo, state_data);
  if (e) {
    if (e & my_first_genom_ETS) warnx("obsolete state");
    if (e & my_first_genom_EPOS) warnx("uncertain position");
    if (e & my_first_genom_EATT) warnx("uncertain orientation");
    if (e & my_first_genom_EVEL) warnx("uncertain velocity");
    if (e & my_first_genom_EAVEL) warnx("uncertain angular velocity");
    warnx("emergency descent");
    return my_first_genom_emergency;
  }

  /* check reference */
  my_first_genom_reference_check(tv, reference);

  /* position controller */
  my_first_genom_controller(body, servo, state_data, reference, wrench_data,
                  *log, &input_data->desired);

  /* output */
  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * my_first_genom_control_period_ms / servo->ramp;
  }

  rotor_input->write(self);
  return my_first_genom_measure;
}


/** Codel my_first_genom_main_measure of task main.
 *
 * Triggered by my_first_genom_measure.
 * Yields to my_first_genom_pause_control.
 */
genom_event
my_first_genom_main_measure(const my_first_genom_ids_body_s *body, const my_first_genom_state *state,
                  const my_first_genom_rotor_measure *rotor_measure,
                  const my_first_genom_wrench_measure *wrench_measure,
                  const genom_context self)
{
  const or_pose_estimator_state *state_data;
  const or_rotorcraft_output *rotor_data;
  or_wrench_estimator_state *wrench_data;
  double wprop[or_rotorcraft_max_rotors];
  double wrench[6];
  struct timeval tv;
  double now;
  size_t i;

  wrench_data = wrench_measure->data(self);
  if (!wrench_data) return my_first_genom_pause_control;

  gettimeofday(&tv, NULL);
  now = tv.tv_sec + 1e-6 * tv.tv_usec;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = { ._present = false },
    .force_cov = { ._present = false },
    .torque = { ._present = false },
    .torque_cov = { ._present = false }
  };

  /* current state (already read by control codel) */
  if (!(state_data = state->data(self)))
    goto output;

  /* current propeller speed */
  if (rotor_measure->read(self) || !(rotor_data = rotor_measure->data(self)))
    goto output;

  for(i = 0; i < rotor_data->rotor._length; i++) {
    if (now > 0.1 +
        rotor_data->rotor._buffer[i].ts.sec +
        1e-9 * rotor_data->rotor._buffer[i].ts.nsec)
      goto output;

    if (rotor_data->rotor._buffer[i].spinning)
      wprop[i] = rotor_data->rotor._buffer[i].velocity;
    else
      wprop[i] = 0.;
  }
  for(;i < or_rotorcraft_max_rotors; i++)
    wprop[i] = 0.;

  /* wrench */
  if (my_first_genom_wrench(body, state_data, wprop, wrench))
    goto output;

  *wrench_data = (or_wrench_estimator_state){
    .ts = { .sec = tv.tv_sec, .nsec = 1000 * tv.tv_usec },
    .intrinsic = false,
    .force = {
      ._present = true,
      ._value = { .x = wrench[0], .y = wrench[1], .z = wrench[2] }
    },
    .force_cov = { ._present = false },
    .torque = {
      ._present = true,
      ._value = { .x = wrench[3], .y = wrench[4], .z = wrench[5] }
    },
    .torque_cov = { ._present = false }
  };

output:
  wrench_measure->write(self);

  return my_first_genom_pause_control;
}


/** Codel my_first_genom_main_emergency of task main.
 *
 * Triggered by my_first_genom_emergency.
 * Yields to my_first_genom_pause_emergency, my_first_genom_control.
 */
genom_event
my_first_genom_main_emergency(const my_first_genom_ids_body_s *body,
                    my_first_genom_ids_servo_s *servo, const my_first_genom_state *state,
                    or_rigid_body_state *reference, my_first_genom_log_s **log,
                    const my_first_genom_rotor_input *rotor_input,
                    const genom_context self)
{
  static const or_wrench_estimator_state wrench_data = {
    .force._present = false, .torque._present = false
  };

  or_pose_estimator_state *state_data = NULL;
  or_rotorcraft_input *input_data = rotor_input->data(self);
  struct timeval tv;
  size_t i;
  int e;

  gettimeofday(&tv, NULL);

  /* read current state */
  if (state->read(self) || !(state_data = state->data(self))) {
    state_data = &(or_pose_estimator_state){
      /* data without any 'present' field except current timestamp for logs */
      .ts.sec = tv.tv_sec, .ts.nsec = 1000 * tv.tv_usec
    };
  }

  /* check state */
  e = my_first_genom_state_check(tv, servo, state_data);
  if (!e) {
    warnx("recovered from emergency");
    return my_first_genom_control;
  }

  /* check reference */
  my_first_genom_reference_check(tv, reference);

  /* position controller */
  my_first_genom_controller(body, servo, state_data, reference, &wrench_data,
                  *log, &input_data->desired);

  /* output */
  input_data->ts = state_data->ts;
  if (servo->scale < 1.) {
    for(i = 0; i < input_data->desired._length; i++)
      input_data->desired._buffer[i] *= servo->scale;

    servo->scale += 1e-3 * my_first_genom_control_period_ms / servo->ramp;
  }

  rotor_input->write(self);
  return my_first_genom_pause_emergency;
}


/** Codel my_first_genom_main_stop of task main.
 *
 * Triggered by my_first_genom_stop.
 * Yields to my_first_genom_ether.
 */
genom_event
my_first_genom_main_stop(const my_first_genom_rotor_input *rotor_input,
               const genom_context self)
{
  or_rotorcraft_input *input_data;
  struct timeval tv;
  int i;

  input_data = rotor_input->data(self);
  if (!input_data) return my_first_genom_ether;

  gettimeofday(&tv, NULL);
  input_data->ts.sec = tv.tv_sec;
  input_data->ts.nsec = tv.tv_usec * 1000;
  input_data->control = or_rotorcraft_velocity;

  for(i = 0; i < input_data->desired._length; i++)
    input_data->desired._buffer[i] = 0.;

  rotor_input->write(self);
  return my_first_genom_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel my_first_genom_servo_main of activity servo.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_pause_start, my_first_genom_ether.
 * Throws my_first_genom_e_input, my_first_genom_e_geom.
 */
genom_event
my_first_genom_servo_main(const my_first_genom_reference *in,
                or_rigid_body_state *reference,
                const genom_context self)
{
  const or_rigid_body_state *ref_data;

  if (in->read(self)) return my_first_genom_e_input(self);
  ref_data = in->data(self);
  if (!ref_data) return my_first_genom_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec == ref_data->ts.nsec &&
      reference->ts.sec == ref_data->ts.sec) return my_first_genom_pause_start;

  /* keep old value for missing fields for my_first_genom_reference_check */
#define copy_if_present(dst, src, f)                                    \
  do {                                                                  \
    dst->f._present = src->f._present;                                  \
    if (src->f._present) dst->f._value = src->f._value;                 \
  } while(0)

  copy_if_present(reference, ref_data, pos);
  copy_if_present(reference, ref_data, att);
  copy_if_present(reference, ref_data, vel);
  copy_if_present(reference, ref_data, avel);
  copy_if_present(reference, ref_data, acc);
  copy_if_present(reference, ref_data, aacc);
  copy_if_present(reference, ref_data, jerk);
  copy_if_present(reference, ref_data, snap);
#undef copy_if_present

  reference->ts = ref_data->ts;
  reference->intrinsic = ref_data->intrinsic;
  return my_first_genom_pause_start;
}


/* --- Activity set_current_position ------------------------------------ */

/** Codel my_first_genom_set_current_position of activity set_current_position.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_ether.
 * Throws my_first_genom_e_input, my_first_genom_e_geom.
 */
genom_event
my_first_genom_set_current_position(const my_first_genom_state *state,
                          or_rigid_body_state *reference,
                          const genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return my_first_genom_e_input(self);
  state_data = state->data(self);
  if (!state_data) return my_first_genom_e_input(self);
  if (!state_data->pos._present) return my_first_genom_e_input(self);
  if (!state_data->att._present) return my_first_genom_e_input(self);

  qw = state_data->att._value.qw;
  qx = state_data->att._value.qx;
  qy = state_data->att._value.qy;
  qz = state_data->att._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  reference->ts = state_data->ts;
  reference->intrinsic = state_data->intrinsic;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return my_first_genom_ether;
}


/* --- Activity log ----------------------------------------------------- */

/** Codel my_first_genom_log_header of activity log.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_ether.
 * Throws my_first_genom_e_sys.
 */
genom_event
my_first_genom_log_header(const my_first_genom_ids_servo_s *servo, my_first_genom_log_s **log,
                const genom_context self)
{
  int s;

  /* log header with some config info */
  s = dprintf(
    (*log)->fd, "# logged on %s#\n" /* note that ctime(3) has a \n */,
    ctime(&(time_t){ time(NULL) }));
  if (s < 0) goto err;

  s = dprintf((*log)->fd, my_first_genom_log_header_fmt "\n");
  if (s < 0) goto err;

  /* enable async writes */
  (*log)->req.aio_fildes = (*log)->fd;

  return my_first_genom_ether;
err:
  my_first_genom_log_stop(log, self);
  return my_first_genom_e_sys_error("log", self);
}
