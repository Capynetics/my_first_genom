#include "acmy_first_genom.h"

#include "my_first_genom_c_types.h"


/* --- Task main -------------------------------------------------------- */


/** Codel my_first_genom_main_start of task main.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_init.
 */
genom_event
my_first_genom_main_start(my_first_genom_ids *ids,
                          const my_first_genom_rotor_input *rotor_input,
                          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_init;
}


/** Codel my_first_genom_main_init of task main.
 *
 * Triggered by my_first_genom_init.
 * Yields to my_first_genom_pause_init, my_first_genom_pause_control.
 */
genom_event
my_first_genom_main_init(or_rigid_body_state *reference,
                         const my_first_genom_ids_body_s *body,
                         const my_first_genom_state *state,
                         const my_first_genom_rotor_measure *rotor_measure,
                         const my_first_genom_rotor_input *rotor_input,
                         const my_first_genom_wrench_measure *wrench_measure,
                         const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_pause_init;
}


/** Codel my_first_genom_main_control of task main.
 *
 * Triggered by my_first_genom_control.
 * Yields to my_first_genom_measure, my_first_genom_emergency.
 */
genom_event
my_first_genom_main_control(const my_first_genom_ids_body_s *body,
                            my_first_genom_ids_servo_s *servo,
                            const my_first_genom_state *state,
                            const my_first_genom_wrench_measure *wrench_measure,
                            or_rigid_body_state *reference,
                            my_first_genom_log_s **log,
                            const my_first_genom_rotor_input *rotor_input,
                            const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_measure;
}


/** Codel my_first_genom_main_measure of task main.
 *
 * Triggered by my_first_genom_measure.
 * Yields to my_first_genom_pause_control.
 */
genom_event
my_first_genom_main_measure(const my_first_genom_ids_body_s *body,
                            const my_first_genom_state *state,
                            const my_first_genom_rotor_measure *rotor_measure,
                            const my_first_genom_wrench_measure *wrench_measure,
                            const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_pause_control;
}


/** Codel my_first_genom_main_emergency of task main.
 *
 * Triggered by my_first_genom_emergency.
 * Yields to my_first_genom_pause_emergency, my_first_genom_control.
 */
genom_event
my_first_genom_main_emergency(const my_first_genom_ids_body_s *body,
                              my_first_genom_ids_servo_s *servo,
                              const my_first_genom_state *state,
                              or_rigid_body_state *reference,
                              my_first_genom_log_s **log,
                              const my_first_genom_rotor_input *rotor_input,
                              const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_pause_emergency;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_ether;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_pause_start;
}


/* --- Activity log ----------------------------------------------------- */

/** Codel my_first_genom_log_header of activity log.
 *
 * Triggered by my_first_genom_start.
 * Yields to my_first_genom_ether.
 * Throws my_first_genom_e_sys.
 */
genom_event
my_first_genom_log_header(const my_first_genom_ids_servo_s *servo,
                          my_first_genom_log_s **log,
                          const genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return my_first_genom_ether;
}
