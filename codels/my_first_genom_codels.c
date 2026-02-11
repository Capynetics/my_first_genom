#include "acmy_first_genom.h"

#include "my_first_genom_c_types.h"


/* --- Attribute set_saturation_weights --------------------------------- */

/** Validation codel my_first_genom_set_satweights of attribute set_saturation_weights.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_inval.
 */
genom_event
my_first_genom_set_satweights(const my_first_genom_ids_servo_s_satweight_s *satweight,
                              const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute set_mass ----------------------------------------------- */

/** Validation codel my_first_genom_change_mass of attribute set_mass.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
my_first_genom_change_mass(double mass,
                           my_first_genom_ids_body_s *body,
                           const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel my_first_genom_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
my_first_genom_set_emerg(my_first_genom_ids_servo_s_emerg_s *emerg,
                         const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Activity servo --------------------------------------------------- */

/** Validation codel my_first_genom_check_geom of activity servo.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_input, my_first_genom_e_geom.
 */
genom_event
my_first_genom_check_geom(bool init, const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Activity log ----------------------------------------------------- */

/** Validation codel my_first_genom_log_open of activity log.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_sys.
 */
genom_event
my_first_genom_log_open(const char path[64], uint32_t decimation,
                        my_first_genom_log_s **log,
                        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute set_saturation_weights --------------------------------- */

/** Codel my_first_genom_reset_controller of attribute set_saturation_weights.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_inval.
 */
genom_event
my_first_genom_reset_controller(const my_first_genom_ids_body_s *body,
                                const my_first_genom_ids_servo_s *servo,
                                const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Attribute set_wlimit --------------------------------------------- */

/** Codel my_first_genom_set_wlimit of attribute set_wlimit.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_set_wlimit(my_first_genom_ids_body_s *body,
                          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}

/** Codel my_first_genom_reset_controller of attribute set_wlimit.
 *
 * Returns genom_ok.
 */
/* already defined in service set_saturation_weights */



/* --- Attribute set_geom ----------------------------------------------- */

/** Codel my_first_genom_set_geom of attribute set_geom.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_set_geom(my_first_genom_ids_body_s *body,
                        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}

/** Codel my_first_genom_reset_controller of attribute set_geom.
 *
 * Returns genom_ok.
 */
/* already defined in service set_saturation_weights */



/* --- Function set_gtmrp_geom ------------------------------------------ */

/** Codel my_first_genom_set_gtmrp_geom of function set_gtmrp_geom.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_inval.
 */
genom_event
my_first_genom_set_gtmrp_geom(uint16_t rotors, double cx, double cy,
                              double cz, double armlen, double mass,
                              double mbodyw, double mbodyh,
                              double mmotor, double rx, double ry,
                              int16_t rz, double cf, double ct,
                              my_first_genom_ids_body_s *body,
                              const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}

/** Codel my_first_genom_reset_controller of function set_gtmrp_geom.
 *
 * Returns genom_ok.
 * Throws my_first_genom_e_inval.
 */
/* already defined in service set_saturation_weights */



/* --- Function get_reference ------------------------------------------- */

/** Codel my_first_genom_get_reference of function get_reference.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_get_reference(const or_rigid_body_state *internal,
                             or_rigid_body_state *reference,
                             const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel my_first_genom_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_servo_stop(or_rigid_body_state *reference,
                          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel my_first_genom_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_log_stop(my_first_genom_log_s **log,
                        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel my_first_genom_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
my_first_genom_log_info(const my_first_genom_log_s *log,
                        uint32_t *miss, uint32_t *total,
                        const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}
