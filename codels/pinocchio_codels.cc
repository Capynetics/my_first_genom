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
 */
#include "acmy_first_genom.h"

#include <err.h>
#include <cstdlib>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/Dense>

#include "codels.h"

extern "C" genom_event
my_first_genom_load_urdf(const char *urdf_path,
                         my_first_genom_pinocchio_s **pinocchio,
                         my_first_genom_ids_wholebody_s *wholebody,
                         const genom_context self)
{
  namespace pin = pinocchio;

  try {
    /* Allocate struct if needed */
    if (*pinocchio == NULL) {
      *pinocchio = new my_first_genom_pinocchio_s;
      (*pinocchio)->model = NULL;
      (*pinocchio)->data = NULL;
      (*pinocchio)->loaded = false;
    }

    /* Clean up previous model */
    if ((*pinocchio)->model) delete (*pinocchio)->model;
    if ((*pinocchio)->data) delete (*pinocchio)->data;

    /* Load new model with free-flyer base */
    (*pinocchio)->model = new pin::Model();
    pin::urdf::buildModel(urdf_path,
                          pin::JointModelFreeFlyer(),
                          *(*pinocchio)->model);
    (*pinocchio)->data = new pin::Data(*(*pinocchio)->model);
    (*pinocchio)->loaded = true;

    /* Set number of joints (nv - 6 base DOFs) */
    int nj = (*pinocchio)->model->nv - 6;
    if (nj < 0) nj = 0;
    if (nj > 8) nj = 8;
    wholebody->nj = nj;
    wholebody->init = true;

    warnx("Loaded URDF: %s (nq=%d, nv=%d, nj=%d)",
          urdf_path,
          (int)(*pinocchio)->model->nq,
          (int)(*pinocchio)->model->nv,
          nj);
    return genom_ok;

  } catch (const std::exception &e) {
    warnx("Failed to load URDF: %s", e.what());
    if (*pinocchio) (*pinocchio)->loaded = false;
    wholebody->init = false;
    return my_first_genom_e_sys_error("load_urdf", self);
  }
}

extern "C" genom_event
my_first_genom_set_wholebody_gains(const double Kp_base[6],
                                   const double Kd_base[6],
                                   const double Kp_joint[8],
                                   const double Kd_joint[8],
                                   my_first_genom_ids_wholebody_s *wholebody,
                                   const genom_context self)
{
  (void)self;

  for (int i = 0; i < 6; i++) {
    wholebody->Kp_base[i] = Kp_base[i];
    wholebody->Kd_base[i] = Kd_base[i];
  }
  for (int i = 0; i < 8; i++) {
    wholebody->Kp_joint[i] = Kp_joint[i];
    wholebody->Kd_joint[i] = Kd_joint[i];
  }

  warnx("Wholebody gains set: Kp_base=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f], Kd_base=[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]",
        wholebody->Kp_base[0], wholebody->Kp_base[1], wholebody->Kp_base[2],
        wholebody->Kp_base[3], wholebody->Kp_base[4], wholebody->Kp_base[5],
        wholebody->Kd_base[0], wholebody->Kd_base[1], wholebody->Kd_base[2],
        wholebody->Kd_base[3], wholebody->Kd_base[4], wholebody->Kd_base[5]);

  return genom_ok;
}

/*
 * wholebody_controller - Compute whole-body PD + gravity compensation torques
 *
 * Control law: τ = g(q) + Kp·e - Kd·q̇
 *
 * Inputs:
 *   q_base[7]      - current base pose [x,y,z,qx,qy,qz,qw]
 *   v_base[6]      - current base velocity [vx,vy,vz,wx,wy,wz]
 *   q_joint[nj]    - current joint positions
 *   v_joint[nj]    - current joint velocities
 *   wholebody      - gains and desired config
 *   pinocchio      - model/data
 *
 * Outputs:
 *   tau_base[6]    - base wrench (force/torque)
 *   tau_joint[nj]  - joint torques
 *
 * Returns 0 on success, -1 if not initialized
 */
int wholebody_controller(const double q_base[7],
                         const double v_base[6],
                         const double *q_joint,
                         const double *v_joint,
                         const my_first_genom_ids_wholebody_s *wholebody,
                         const my_first_genom_pinocchio_s *pinocchio,
                         double tau_base[6],
                         double *tau_joint)
{
  namespace pin = pinocchio;

  if (!pinocchio || !pinocchio->loaded || !wholebody->init)
    return -1;

  const pin::Model &model = *pinocchio->model;
  pin::Data &data = *pinocchio->data;
  const int nj = wholebody->nj;
  const int nv = model.nv;
  const int nq = model.nq;

  /* Build configuration vector q (nq = 7 + nj for free-flyer) */
  Eigen::VectorXd q(nq);
  /* Base: [x,y,z,qx,qy,qz,qw] */
  for (int i = 0; i < 7; i++)
    q(i) = q_base[i];
  /* Joints */
  for (int i = 0; i < nj; i++)
    q(7 + i) = q_joint[i];

  /* Build velocity vector v (nv = 6 + nj) */
  Eigen::VectorXd v(nv);
  /* Base velocity: [vx,vy,vz,wx,wy,wz] */
  for (int i = 0; i < 6; i++)
    v(i) = v_base[i];
  /* Joint velocities */
  for (int i = 0; i < nj; i++)
    v(6 + i) = v_joint[i];

  /* Compute gravity compensation: g(q) */
  Eigen::VectorXd g = pin::computeGeneralizedGravity(model, data, q);

  /* Build desired configuration qd */
  Eigen::VectorXd qd(nq);
  for (int i = 0; i < 7; i++)
    qd(i) = wholebody->qd_base[i];
  for (int i = 0; i < nj; i++)
    qd(7 + i) = wholebody->qd_joint[i];

  /*
   * Compute configuration error e = qd ⊖ q
   * For SE(3) base, use Pinocchio's difference operator
   * For revolute joints, it's simply qd - q
   */
  Eigen::VectorXd e = pin::difference(model, q, qd);

  /* Build gain vectors */
  Eigen::VectorXd Kp(nv), Kd(nv);
  for (int i = 0; i < 6; i++) {
    Kp(i) = wholebody->Kp_base[i];
    Kd(i) = wholebody->Kd_base[i];
  }
  for (int i = 0; i < nj; i++) {
    Kp(6 + i) = wholebody->Kp_joint[i];
    Kd(6 + i) = wholebody->Kd_joint[i];
  }

  /* Control law: τ = g(q) + Kp·e - Kd·q̇ */
  Eigen::VectorXd tau = g + Kp.cwiseProduct(e) - Kd.cwiseProduct(v);

  /* Extract outputs */
  for (int i = 0; i < 6; i++)
    tau_base[i] = tau(i);
  for (int i = 0; i < nj; i++)
    tau_joint[i] = tau(6 + i);

  return 0;
}
