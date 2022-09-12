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
    /* HOUSEKEEPING */

    /* DETECTORS */

    /* POINTING */
    antisun,
    stop,
    // Vetoes & Allows
    elclin_allow,      elclin_veto,
    elmotenc_allow,    elmotenc_veto,
    xsc0_allow,        xsc0_veto,
    xsc1_allow,        xsc1_veto,
    dgps_allow,        dgps_veto,
    ifroll_1_gy_allow, ifroll_1_gy_veto,
    ifyaw_1_gy_allow,  ifyaw_1_gy_veto,
    ifel_1_gy_allow,   ifel_1_gy_veto,
    ifroll_2_gy_allow, ifroll_2_gy_veto,
    ifyaw_2_gy_allow,  ifyaw_2_gy_veto,
    ifel_2_gy_allow,   ifel_2_gy_veto,
    mag_allow_fc1,     mag_veto_fc1,
    mag_allow_fc2,     mag_veto_fc2,
    pss_allow,         pss_veto,
    // Sensors
    az_auto_gyro,
    el_auto_gyro,
    mag_reset,
    // Trims
    trim_to_xsc0,
    trim_to_xsc1,
    trim_xsc0_to_xsc1,
    trim_xsc1_to_xsc0,
    autotrim_off,
    reset_trims,

    /* MOTORS */
    rw_wake_and_wiggle,
    // Motor gains
    az_off,             az_on,
    el_off,             el_on,
    force_el_on,
    // Resets
    reset_rw,
    reset_piv,
    reset_elev,
    reset_ethercat,
    restore_piv,

    /* ACTUATORS */
    actbus_on,
    actbus_off,
    actbus_cycle,
    repoll,
    // Shutter
    shutter_off,
    shutter_init,
    shutter_reset,
    shutter_close,
    shutter_open,
    shutter_keepopen,
    shutter_keepclosed,
    // Lock pin
    pin_in,
    unlock,
    lock_off,
    lock45,
    // Secondary Mirror
    autofocus_allow,
    autofocus_veto,
    actuator_stop,
    // Balance system
    balance_auto,
    balance_off,
    balance_terminate,

    /* STAR CAMERAS */

    /* MISC */
    // Video transmitters
    vtx_xsc0,
    vtx_xsc1,
    // XY stage
    xy_panic,
    // BLAST(TIM) Misc
    reap_fc1,       reap_fc2,
    halt_fc1,       halt_fc2,
    blast_rocks,    blast_sucks,
    at_float,           not_at_float,
    gps_sw_reset,
    gps_stats,
    reset_log,
    xyzzy
};

/* multiCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum multiCommand {
    /* HOUSEKEEPING */

    /* DETECTORS */

    /* POINTING */
    slew_veto,  // No slewing
    // Scans
    az_scan_accel,
    set_scan_params,
    ra_dec_goto,
    az_el_goto,
    box,    // Rectangular scan
    quad,   // Quadrilateral with 4 defined corners
    cap,    // Circular scan
    el_scan,
    el_box,
    az_scan,
    cur_mode, // constant current??
    drift,    // constant speed
    // Magnetometer
    mag_cal_fc1,
    mag_cal_fc2,
    // PSS
    pss_cal_n,
    pss_set_noise,
    pss_cal_d,
    pss_cal_el,
    pss_cal_az,
    pss_cal_roll,
    pss_cal_array_az,
    pss_set_imin,
    // Gyros
    az_gyro_offset,
    el_gyro_offset,
    // Trims
    ra_dec_set, // RA DEC
    pos_set,    // LAT LON
    az_el_trim,
    autotrim_to_sc,

    /* MOTORS */
    fix_ethercat,
    az_gain,
    pivot_gain,
    el_gain,
    motors_verbose,
    /* ACTUATORS */
    actuator_i,
    actuator_vel,
    actuator_tol,
    actuator_servo,
    general,
    // Lock Pin
    lock_vel,
    lock_i,
    lock,
    // Shutter
    shutter_i,
    shutter_vel,
    // Balance
    balance_gain,
    balance_manual,
    balance_vel,
    balance_i,
    // Secondary mirror
    set_secondary,
    thermo_param,
    focus_offset,
    thermo_gain,
    actuator_delta,
    delta_secondary,
    act_enc_trim,

    /* TELEMETRY */
    highrate_bw,
    pilot_bw,
    biphase_bw,
    biphase_clk_speed,
    highrate_through_tdrss,
    set_linklists,
    request_file,
    request_stream_file,
    set_pilot_oth,

    /* STAR CAMERAS */
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

    /* MISC */
    // XY stage
    xy_goto,
    xy_jump,
    xy_xscan,
    xy_yscan,
    xy_raster,
    // Labjacks
    set_queue_execute, // Who sends the modbus commands
    reconnect_lj,      // Force a reconnect attempt to labjack_i
    // BLAST Misc
    params_test, // literally just a test function
    timeout,    // timer until schedule file mode takes over
    slot_sched, // upload new schedule file?
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
