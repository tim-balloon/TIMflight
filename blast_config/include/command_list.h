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

#define CMD_I_MAX USHRT_MAX
#define CMD_L_MAX UINT_MAX

// Number of different command groups we can have
#define N_GROUPS 31

// Current group:number mapping, plenty of space here
#define GRPOS_POINT 0
#define GRPOS_BAL   1
#define GRPOS_TRIM  2
#define GRPOS_VETO  3
#define GRPOS_ACT   4
#define GRPOS_XSC_HOUSE 5
#define GRPOS_XSC_MODE  6
#define GRPOS_XSC_PARAM 7
#define GRPOS_MOTOR  8
#define GRPOS_CRYO  9
#define GRPOS_POWER 10
#define GRPOS_LOCK  11
#define GRPOS_TELEM 12
#define GRPOS_MISC  13
#define GRPOS_FOCUS 14
#define GRPOS_PSS   15

// Groups saved as a 32 bit bitmap and shifted by their number
#define GR_POINT        (1 << GRPOS_POINT)
#define GR_BAL          (1 << GRPOS_BAL)
#define GR_TRIM         (1 << GRPOS_TRIM)
#define GR_VETO         (1 << GRPOS_VETO)
#define GR_ACT          (1 << GRPOS_ACT)
#define GR_XSC_HOUSE    (1 << GRPOS_XSC_HOUSE)
#define GR_XSC_MODE     (1 << GRPOS_XSC_MODE) // deprecated but effort, in the xsc TODO
#define GR_XSC_PARAM    (1 << GRPOS_XSC_PARAM)
#define GR_MOTOR        (1 << GRPOS_MOTOR)
#define GR_CRYO         (1 << GRPOS_CRYO)
#define GR_POWER        (1 << GRPOS_POWER)
#define GR_LOCK         (1 << GRPOS_LOCK)
#define GR_TELEM        (1 << GRPOS_TELEM)
#define GR_MISC         (1 << GRPOS_MISC)
#define GR_FOCUS        (1 << GRPOS_FOCUS)
#define GR_PSS          (1 << GRPOS_PSS)

// reserved for CONFIRM  0x80000000

extern const char *command_list_serial;
extern const char *GroupNames[N_GROUPS];
extern const char *linklist_names[];
extern const char *downlink_names[];
extern const char *pilot_target_names[];

/* singleCommand enumeration.  The command list here does NOT have to be in
 * order relative to the command definitions in command_list.c */
enum singleCommand {
    /* POWER SYSTEMS */
    // OF PBOB
    fc1_on, fc1_off, fc2_on, fc2_off, motor_lj_on, motor_lj_off,
    of_relay_5_on, of_relay_5_off, of_inc_on, of_inc_off, mag_on, mag_off,
    therm_on, therm_off, gps_on, gps_off, pss_on, pss_off,
    // IF PBOB
    sc1_on, sc1_off, cryo_digital_on, cryo_digital_off, gyros_on, gyros_off,
    rfsoc_on, rfsoc_off, steppers_on, steppers_off, if_inc_on, if_inc_off,
    sc2_on, sc2_off, cryo_analog_on, cryo_analog_off, if_relay_10_on, if_relay_10_off,
    /* HOUSEKEEPING */

    /* DETECTORS */

    /* POINTING */
    antisun,
    stop,
    // Vetoes & Allows
    elclin_allow_fc1,      elclin_veto_fc1,
    elclin_allow_fc2,      elclin_veto_fc2,
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
    sc_gps_updates,
    sc_stop_gps_updates,
    sc1_interrupt_command,
    sc2_interrupt_command,
    sc1_interrupt_image,
    sc2_interrupt_image,
    sc1_interrupt_param,
    sc2_interrupt_param,
    sc1_reset_command,
    sc2_reset_command,
    sc1_reset_image,
    sc2_reset_image,
    sc1_reset_param,
    sc2_reset_param,
    force_starcam_trigger,
    enable_sc_trigger,
    disable_sc_trigger,
    reset_sc_timeout,
    sc1_trigger_on,
    sc1_trigger_off,
    sc2_trigger_on,
    sc2_trigger_off,

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

    // EVTM telemetry
    enable_evtm_los, enable_evtm_tdrss, disable_evtm_los, disable_evtm_tdrss,
    enable_evtm_all, disable_evtm_all,

    xyzzy // xyzzy must come last in this list!
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
    set_pilot_oth,

    /* NEW STAR CAMERAS */
    // SC1
    sc1_trim_lat,
    sc1_trim_lon,
    sc1_trim_height,
    sc1_trim_pos,
    sc1_set_exposure_time,
    sc1_set_logodds,
    sc1_set_time_limit,
    sc1_set_af_parameters,
    sc1_set_af_photos,
    sc1_set_focus_mode,
    sc1_focus_move,
    sc1_set_focus_inf,
    sc1_change_aperture,
    sc1_max_aperture,
    sc1_make_static_hp,
    sc1_use_static_hp,
    sc1_set_spike_limit,
    sc1_search_dynamic_hp,
    sc1_set_lpf_radius,
    sc1_use_hpf,
    sc1_set_hpf_radius,
    sc1_set_border,
    sc1_set_unique_spacing,
    sc1_set_n_sigma,
    // SC2
    sc2_trim_lat,
    sc2_trim_lon,
    sc2_trim_height,
    sc2_trim_pos,
    sc2_set_exposure_time,
    sc2_set_logodds,
    sc2_set_time_limit,
    sc2_set_af_parameters,
    sc2_set_af_photos,
    sc2_set_focus_mode,
    sc2_focus_move,
    sc2_set_focus_inf,
    sc2_change_aperture,
    sc2_max_aperture,
    sc2_make_static_hp,
    sc2_use_static_hp,
    sc2_set_spike_limit,
    sc2_search_dynamic_hp,
    sc2_set_lpf_radius,
    sc2_use_hpf,
    sc2_set_hpf_radius,
    sc2_set_border,
    sc2_set_unique_spacing,
    sc2_set_n_sigma,
    // sc trigger timeout
    set_sc_timeout,
    sc1_set_trigger_timeout,
    sc2_set_trigger_timeout,

    /* OLD STAR CAMERAS */
    // TODO(ianlowe13): Remove these old XSC commands
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

/**
 * @brief How many single commands are there? This used to be mapped
 * by a magic number, but now we required that xyzzy is the last one 
 * so any commands added automatically increment the number as long as
 * we leave xyzzy last
 * 
 */
#define N_SCOMMANDS (xyzzy + 1)
/**
 * @brief How many multi commands are there? This used to be mapped
 * by a magic number, but now we required that plugh is the second to
 * last one so any commands added automatically increment the number
 * as long as we leave plugh second to last
 * 
 */
#define N_MCOMMANDS (plugh + 2)

// Make a structure out of these single commands available throughout mcp
extern struct scom scommands[N_SCOMMANDS];

// 7/24/23 updated parameter comment ILOWE
/* parameter type:
 * i :  parameter is 16 bit unnormalised integer. Max is CMD_I_MAX
 * l :  parameter is 32 bit unnormalised integer. Max is CMD_L_MAX
 * f :  parameter is 16 bit renormalised floating point
 * d :  parameter is 32 bit renormalised floating point
 * s :  parameter is 7-bit character string JOY: actually 32 char long
 */
// make a structure out of these multi commands available throughout mcp
extern struct mcom mcommands[N_MCOMMANDS];

/* validator function for mcommands */
extern int mcom_validate(enum multiCommand cmd, const int *ivalues,
    const double *rvalues, char svalues[][CMD_STRING_LEN], size_t buflen,
    char *err_buffer);

#endif /* COMMAND_LIST_H */
