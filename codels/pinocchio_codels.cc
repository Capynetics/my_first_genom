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
