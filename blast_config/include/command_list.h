/* command_list.h: BLAST command specification file definitions
 *
 * This software is copyright (C) 2002-2012 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_COMMAND_LIST_H
#define INCLUDE_COMMAND_LIST_H

#include <limits.h>
#include <sys/types.h>

#include "netcmd.h"  /* common parts of command defintions moved here */


#define DATA_Q_SIZE (2 * MAX_N_PARAMS)  /* maximum size of the data queue */

#define MAX_15BIT (32767.)    // deprecated. Probably want CMD_I_MAX instead
#define CMD_I_MAX USHRT_MAX
#define CMD_L_MAX UINT_MAX

#define N_GROUPS 31

#define GRPOS_POINT 0
#define GRPOS_BAL   1
#define GRPOS_HWPR  2
#define GRPOS_TRIM  3
#define GRPOS_ELECT 4 // empty, remove after 2018 flight? -PAW
#define GRPOS_BIAS  5
#define GRPOS_VETO  6
#define GRPOS_ACT   7
#define GRPOS_XSC_HOUSE 8
#define GRPOS_XSC_MODE  9
#define GRPOS_XSC_PARAM 10
#define GRPOS_MOTOR  11
#define GRPOS_CRYO  12
#define GRPOS_POWER 13
#define GRPOS_LOCK  14
#define GRPOS_TELEM 15
#define GRPOS_MISC  16
#define GRPOS_FOCUS 17
#define GRPOS_ROACH 18
#define GRPOS_PSS   19

#define GR_POINT        (1 << GRPOS_POINT)
#define GR_BAL          (1 << GRPOS_BAL)
#define GR_HWPR         (1 << GRPOS_HWPR)
#define GR_TRIM         (1 << GRPOS_TRIM)
#define GR_ELECT        (1 << GRPOS_ELECT)
#define GR_BIAS         (1 << GRPOS_BIAS)
#define GR_VETO         (1 << GRPOS_VETO)
#define GR_ACT          (1 << GRPOS_ACT)
#define GR_XSC_HOUSE    (1 << GRPOS_XSC_HOUSE)
#define GR_XSC_MODE     (1 << GRPOS_XSC_MODE)
#define GR_XSC_PARAM    (1 << GRPOS_XSC_PARAM)
#define GR_MOTOR        (1 << GRPOS_MOTOR)
#define GR_CRYO         (1 << GRPOS_CRYO)
#define GR_POWER        (1 << GRPOS_POWER)
#define GR_LOCK         (1 << GRPOS_LOCK)
#define GR_TELEM        (1 << GRPOS_TELEM)
#define GR_MISC         (1 << GRPOS_MISC)
#define GR_FOCUS        (1 << GRPOS_FOCUS)
#define GR_ROACH        (1 << GRPOS_ROACH)
#define GR_PSS          (1 << GRPOS_PSS)

// reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];
extern const char *linklist_names[];
extern const char *downlink_names[];
extern const char *pilot_target_names[];
extern const char *stream_types[];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
  az_auto_gyro,     az_off,             az_on,
  balance_auto,     balance_off, balance_terminate,
  el_off,             el_on,
  elclin_allow,     elclin_veto,
  elmotenc_allow,   elmotenc_veto,
  xsc0_veto,        xsc0_allow,
  xsc1_veto,        xsc1_allow,         dgps_veto,        dgps_allow,
  mag_allow_fc1,      mag_veto_fc1,           mag_allow_fc2,        mag_veto_fc2,
  pin_in,                 reset_trims,
  stop,             pss_veto,		    trim_xsc0_to_xsc1,
  pss_allow,        trim_xsc1_to_xsc0,    autotrim_off,
  trim_to_xsc0,      unlock,             lock_off,
  force_el_on,
  actbus_cycle,
  charge_off,	    charge_on,		charge_cycle,

  ifroll_1_gy_allow, ifroll_1_gy_veto,   ifroll_2_gy_allow, ifroll_2_gy_veto,
  ifyaw_1_gy_allow, ifyaw_1_gy_veto,    ifyaw_2_gy_allow, ifyaw_2_gy_veto,
  ifel_1_gy_allow,  ifel_1_gy_veto,	ifel_2_gy_allow,  ifel_2_gy_veto,
  ifroll_1_gy_off,  ifroll_1_gy_on,	ifroll_2_gy_off,  ifroll_2_gy_on,
  ifyaw_1_gy_off,   ifyaw_1_gy_on,	ifyaw_2_gy_off,	  ifyaw_2_gy_on,
  ifel_1_gy_off,    ifel_1_gy_on,	ifel_2_gy_off,	  ifel_2_gy_on,
  gybox_cycle,
            reap_fc1,       reap_fc2,
  xy_panic,
  trim_to_xsc1,      antisun,            blast_rocks,      blast_sucks,
  at_float,           not_at_float,     el_auto_gyro,
  repoll,           autofocus_allow,
  autofocus_veto,   halt_fc1,         halt_fc2,       actbus_on,
  actbus_off,       actuator_stop,      restore_piv,
  reset_rw,         reset_piv,
  reset_elev,       reset_ethercat, rw_wake_and_wiggle,
  vtx1_xsc0,	    vtx1_xsc1,		vtx2_xsc0,
  vtx2_xsc1,
      shutter_init,     shutter_close,
  shutter_reset,    shutter_open,       shutter_off,      shutter_open_close,
  shutter_keepopen, shutter_keepclosed,
  lock45,           shutter_close_slow,
  // level_sensor_on,  level_sensor_off,
  bias_reset_rox,
    mag_reset, gps_sw_reset, gps_stats,
	 vtx_xsc0,
  vtx_xsc1,
  trigger_retune_check,
  reset_log,
  xyzzy
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
  az_el_goto,        az_gain,           az_scan,          balance_gain,
  balance_manual,    balance_vel,       balance_i,
  // fridge_cycle_params,
  box,
  // cal_repeat,
  cap,              cur_mode,
  az_el_trim,        drift,             el_gain,
  autotrim_to_sc,
  lock,              phase,             act_offset,
  pivot_gain,        ra_dec_goto,      ra_dec_set,
  pos_set,
  az_scan_accel,
  // t_gyro_set,
  highrate_bw,       pilot_bw,         biphase_bw,
  biphase_clk_speed, highrate_through_tdrss,   set_linklists,
  request_file,
  set_queue_execute, reconnect_lj,
  request_stream_file, set_pilot_oth,

  // t_gyro_gain,
  timeout,           vcap,
  vbox,              slot_sched,        az_gyro_offset,
  // jfet_set,
  gyro_off,	         quad,
  el_gyro_offset,    general,           slew_veto,        set_secondary,
  thermo_gain,       actuator_servo,    xy_goto,          actuator_vel,
  xy_jump,           xy_xscan,          xy_yscan,         xy_raster,
  actuator_i,        lock_vel,          lock_i,           actuator_delta,
  delta_secondary,   thermo_param,      focus_offset,
  motors_verbose,    fix_ethercat,
  // phase_step,
         params_test,
      act_enc_trim,     actuator_tol,
  el_scan,           el_box,            shutter_step,     shutter_step_slow,
  shutter_i, 	    shutter_vel,
  set_scan_params,   mag_cal_fc1,	mag_cal_fc2,         pss_cal, pss_cal_n,
  pss_set_noise,
  pss_cal_d, pss_cal_el, pss_cal_az, pss_cal_roll, pss_cal_array_az, pss_set_imin,

  xsc_is_new_window_period,
  xsc_offset,
  xsc_heaters_off,
  xsc_heaters_on,
  xsc_heaters_auto,
  xsc_exposure_timing,
  xsc_multi_trigger,
  xsc_trigger_threshold,
  xsc_scan_force_trigger,
  xsc_quit,
  xsc_shutdown,
  xsc_network_reset,
  xsc_main_settings,
  xsc_display_zoom,
  xsc_image_client,
  xsc_init_focus,
  xsc_get_focus,
  xsc_set_focus,
  xsc_stop_focus,
  xsc_define_focus,
  xsc_set_focus_incremental,
  xsc_run_autofocus,
  xsc_set_autofocus_range,
  xsc_abort_autofocus,
  xsc_autofocus_display_mode,
  xsc_init_aperture,
  xsc_get_aperture,
  xsc_set_aperture,
  xsc_stop_aperture,
  xsc_define_aperture,
  xsc_get_gain,
  xsc_set_gain,
  xsc_fake_sky_brightness,
  xsc_solver_general,
  xsc_solver_abort,
  xsc_selective_mask,
  xsc_blob_finding,
  xsc_blob_cells,
  xsc_pattern_matching,
  xsc_filter_hor_location,
  xsc_filter_hor_roll,
  xsc_filter_el,
  xsc_filter_eq_location,
  xsc_filter_matching,
  plugh,                // plugh should be at the end of the list
  sched_packet = 0xffe   // not really a command, more of a placeholder
};

#define N_SCOMMANDS (xyzzy + 1)
#define N_MCOMMANDS (plugh + 2)

extern struct scom scommands[N_SCOMMANDS];

/* parameter type:
 * i :  parameter is 15 bit unnormalised integer
 * f :  parameter is 15 bit renormalised floating point
 * l :  parameter is 30 bit renormalised floating point
 */
extern struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
