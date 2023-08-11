/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_COMMAND_STRUCT_H
#define INCLUDE_COMMAND_STRUCT_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <xsc_protocol.h>
#include "command_list.h"
#include "channels_tng.h"
#include "mcp_sched.h"
#include "pointing_struct.h"

// motor state values
#define AXIS_VEL      0
#define AXIS_POSITION 1
#define AXIS_LOCK     2

// pointing state values
#define P_AZEL_GOTO  1
#define P_AZ_SCAN    2
#define P_DRIFT      3
#define P_RADEC_GOTO 4
#define P_VCAP       5
#define P_CAP        6
#define P_BOX        7
#define P_LOCK       8
#define P_VBOX       9
#define P_QUAD      10
#define P_EL_SCAN   11
#define P_EL_BOX    12
#define P_CURRENT   13

#define MAX_ISC_SLOW_PULSE_SPEED 0.015

// XY stage state values
#define XYSTAGE_PANIC  0
#define XYSTAGE_GOTO   1
#define XYSTAGE_JUMP   2
#define XYSTAGE_SCAN   3
#define XYSTAGE_RASTER 4

/* latching relay pulse length in 200ms slow frames */
#define LATCH_PULSE_LEN	 2
/* time (slow frames) to keep power off when power cycling devices */
#define PCYCLE_HOLD_LEN	 20

#define PREV_STATUS_FILE "/data/etc/blast/mcp.prev_status"

/* Need to undef I here in for source files that utilize complex.h */
#undef I
/**
 * @brief PID control gains structure
 * 
 */
struct GainStruct {
  float P;
  float I;
  float D;
  float SP;
  float DB; // Deadband the integral term
  float PT; // position velocity gain
  float F; // Current offset to overcome static friction.
};


/**
 * @brief pivot-specific gain structure that holds reaction wheel info as well
 * 
 */
struct PivGainStruct {
    float PV; // prop to RW velocity
    float IV; // prop to RW velocity
    float PE; // prop to velocity error
    float IE; // prop to velocity error
    double SP; // RW velocity Set Point
    double F; // Current offset to overcome static friction.
};


// lock pin state values
#define LS_OPEN        0x0001  // Set => Lock is OPEN
#define LS_CLOSED      0x0002  // Set => Lock is CLOSED
#define LS_DRIVE_OFF   0x0004  // Set => Drive is OFF or NOT MOVING
#define LS_POT_RAIL    0x0008  // now defunct
#define LS_DRIVE_EXT   0x0010  // Set => Drive is extending
#define LS_DRIVE_RET   0x0020  // Set => Drive is retracting
#define LS_DRIVE_STP   0x0040  // This is never set. No idea what this is.
#define LS_DRIVE_JIG   0x0080  // now defunct
#define LS_DRIVE_UNK   0x0100
#define LS_EL_OK       0x0200
#define LS_IGNORE_EL   0x0400
#define LS_DRIVE_FORCE 0x0800
#define LS_DRIVE_MASK  0x09F4


// shutter state values
#define SHUTTER_OPEN    0x0001
#define SHUTTER_CLOSED  0x0002
#define SHUTTER_INIT    0x0004
#define SHUTTER_OFF     0x0008
#define SHUTTER_RESET   0x0010
#define SHUTTER_NOP     0x0020
#define SHUTTER_CLOSED2 0x0040
#define SHUTTER_CLOSED_SLOW 0x0080
#define SHUTTER_UNK     0x0100
#define SHUTTER_KEEPCLOSED 0x0200
#define SHUTTER_KEEPOPEN 0X0400


// actuator bus state values
#define ACTBUS_FM_SLEEP  0
#define ACTBUS_FM_SERVO  1
#define ACTBUS_FM_FOCUS  2
#define ACTBUS_FM_OFFSET 3
#define ACTBUS_FM_THERMO 4
#define ACTBUS_FM_NOW    5  // unused
#define ACTBUS_FM_DELTA  6
#define ACTBUS_FM_PANIC  7
#define ACTBUS_FM_DELFOC 8
#define ACTBUS_FM_TRIM   9


// actuator thermal compensation state values
#define TC_MODE_ENABLED  0
#define TC_MODE_AUTOVETO 1
#define TC_MODE_VETOED   2

// mode        X     Y    vaz   del    w    h
// LOCK              el
// AZEL_GOTO   az    el
// AZ_SCAN     az    el   vaz
// DRIFT                  vaz   vel
// RADEC_GOTO  ra    dec
// VCAP        ra    dec  vaz   vel    r
// CAP         ra    dec  vaz   elstep r
// BOX         ra    dec  vaz   elstep w    h
/**
 * @brief pointing mode state structure
 * 
 */
struct PointingModeStruct {
  int nw; /* used for gy-offset veto during slews */
  int mode;
  double X;
  double Y;
  double vaz;
  double del;
  double w;
  double h;
  time_t t;
  double ra[4]; // the RAs for radbox (ie, quad)
  double dec[4]; // the decs for radbox (ie, quad)
  uint32_t n_dith; // Elevation dither step
  int next_i_dith; // Dither starting index for next scan
  int next_i_hwpr; // HWPR pos for next scan
  double vel; // Elevation scan velocity
  double daz; // Azimuth step size (for el scans)
};

// TODO(ianlowe13): remove old XSC stuff
typedef enum
{
    xsc_heater_off, xsc_heater_on, xsc_heater_auto
} xsc_heater_modes_t;

// TODO(ianlowe13): remove old XSC stuff
typedef struct XSCHeaters
{
    xsc_heater_modes_t mode;
    double setpoint;
} XSCHeaters;

// TODO(ianlowe13): remove old XSC stuff
typedef struct XSCTriggerThresholds
{
    bool enabled;
    double blob_streaking_px;
}
XSCTriggerThreshold;

// TODO(ianlowe13): remove old XSC stuff
typedef struct XSCTrigger
{
    int exposure_time_cs;
    int grace_period_cs;
    int post_trigger_counter_mcp_share_delay_cs; // should probably be less than grace period

    int num_triggers;
    int multi_trigger_time_between_triggers_cs;

    XSCTriggerThreshold threshold;
    bool scan_force_trigger_enabled;
} XSCTrigger;

// TODO(ianlowe13): remove old XSC stuff
typedef struct XSCCommandStruct
{
    int is_new_window_period_cs;
    XSCHeaters heaters;
    XSCTrigger trigger;
    XSCClientData net;
    double cross_el_trim;
    double el_trim;
} XSCCommandStruct;


/**
 * @brief labjack command queue information structure
 * 
 */
typedef struct {
    uint16_t lj_q_on;
    uint16_t which_q[11];
    uint16_t set_q;
} labjack_queue_t;


/**
 * @brief highrate and biphase packet slinger - probably deprecated but need to extract when we do EVTM
 * 
 */
typedef struct slinger_commanding
{
    unsigned int downlink_rate_bps;
    bool highrate_active;
    bool biphase_active;
} slinger_commanding_t;


/**
 * @brief Ethercat controller/device commands
 * 
 */
typedef struct {
    bool reset;
    bool fix_rw;
    bool fix_el;
    bool fix_piv;
    bool fix_hwpr;
    bool have_commutated_rw;
    bool rw_commutate_next_ec_reset;
} ec_devices_struct_t;


/**
 * @brief balance system structure
 * 
 */
typedef struct {
    enum {bal_rest = 0, bal_manual, bal_auto} mode;
    enum {neg = 0, no_bal, pos} bal_move_type;
    uint32_t pos;
    uint32_t vel;
    uint16_t hold_i;
    uint16_t move_i;
    uint16_t acc;

    // servo parameters
    double i_el_on_bal;
    double i_el_off_bal;
    double gain_bal;
} cmd_balance_t;


/**
 * @brief labjack pbob command structure
 * 
 */
typedef struct {
  int relay_1_on, relay_1_off, relay_2_on, relay_2_off;
  int relay_3_on, relay_3_off, relay_4_on, relay_4_off;
  int relay_5_on, relay_5_off, relay_6_on, relay_6_off;
  int relay_7_on, relay_7_off, relay_8_on, relay_8_off;
  int relay_9_on, relay_9_off, relay_10_on, relay_10_off;
  int update_pbob;
} lj_pbob_t;

// See file star_camera_struct.h for extended definitions of these parameters
/**
 * @brief star camera command parameters structure
 * 
 */
typedef struct {
  int send_commands;
  double logOdds; // significance of point sources
  int update_logOdds; // is this a new commanded value?
  double latitude; // payload lat
  int update_lat; // is this a new commanded value?
  double longitude; // payload long
  int update_lon; // is this a new commanded value?
  double heightWGS84; // payload alt above reference surface
  int update_height; // is this a new commanded value?
  double exposureTime; // milliseconds
  int update_exposureTime; // is this a new commanded value?
  double solveTimeLimit; // time allowed to solve an image
  int update_solveTimeLimit; // is this a new commanded value?
  float focusPos; // desired focus position, encoder units
  int update_focusPos; // is this a new commanded value?
  int focusMode; // autofocus or manual?
  int update_focusMode; // is this a new commanded value?
  int startPos; // start of autofocus range
  int update_startPos; // is this a new commanded value?
  int endPos; // end of autofocus range
  int update_endPos; // is this a new commanded value?
  int focusStep; // step size in encoder units
  int update_focusStep; // is this a new commanded value?
  int photosPerStep; // ...
  int update_photosPerStep; // is this a new commanded value?
  int setFocusInf; // move focus position to infinity
  int update_setFocusInf; // is this a new commanded value?
  int apertureSteps; // number of positions +/- to move the aperture
  int update_apertureSteps; // is this a new commanded value?
  int maxAperture; // open aperture fully
  int update_maxAperture; // is this a new commanded value?
  int makeHP; // set to 20 to do it, makes a new static hot pixel map
  int update_makeHP; // is this a new commanded value?
  int useHP; // use the hot pixel map to mask bad pixels
  int update_useHP; // is this a new commanded value?
  float blobParams[9]; // blobfinding parameters...
  int update_blobParams[9]; // is this a new commanded value?
} sc_commands_t;


/**
 * @brief star camera thread state boolean structure
 * 
 */
typedef struct {
  int sc1_command_bool;
  int sc2_command_bool;
  int sc1_image_bool;
  int sc2_image_bool;
  int sc1_param_bool;
  int sc2_param_bool;
} sc_thread_bools_t;


/**
 * @brief star camera thread reset data structure
 * 
 */
typedef struct {
  int reset_sc1_comm;
  int reset_sc2_comm;
  int reset_sc1_image;
  int reset_sc2_image;
  int reset_sc1_param;
  int reset_sc2_param;
} sc_resets_t;


/**
 * @brief full command data structure containing relevant cross-mcp information for commanding
 * 
 */
struct CommandDataStruct {
  uint16_t command_count;
  uint16_t last_command;

  // TODO(seth): Insert these into "Scheduler struct"
  uint16_t timeout;
  uint16_t slot_sched; // what slot to use
  uint16_t upslot_sched; // slot being uplinked
  uint32_t parts_sched; // bitfield up pulinked parts
  uint16_t uplink_sched; // use uplink sched
  uint16_t sucks;
  uint16_t lat_range;
  uint16_t at_float;

  uint32_t highrate_bw;
  uint32_t pilot_bw;
  uint32_t biphase_bw;

  float highrate_allframe_fraction;
  float pilot_allframe_fraction;
  float biphase_allframe_fraction;

  uint32_t biphase_clk_speed;
  bool biphase_rnrz;
  bool highrate_through_tdrss;
  char pilot_linklist_name[32];
  char bi0_linklist_name[32];
  char highrate_linklist_name[32];
  char sbd_linklist_name[32];
  uint32_t pilot_oth;

  enum {VTX_XSC0, VTX_XSC1} vtx_sel[2];

  struct GainStruct ele_gain;
  struct GainStruct azi_gain;
  struct PivGainStruct pivot_gain;

  uint16_t disable_az;
  uint16_t disable_el;
  uint16_t force_el;

  uint16_t reset_rw;
  uint16_t reset_piv;
  uint16_t reset_elev;
  uint16_t restore_piv;

  uint16_t verbose_rw;
  uint16_t verbose_el;
  uint16_t verbose_piv;
  int az_autogyro;
  int el_autogyro;
  double offset_ifel_gy;
  double offset_ifroll_gy;
  double offset_ifyaw_gy;
  uint32_t gymask;

  unsigned char use_elmotenc;
  unsigned char use_elclin;
  unsigned char use_pss;
  unsigned char use_xsc0;
  unsigned char use_xsc1;
  unsigned char use_mag1;
  unsigned char use_mag2;
  unsigned char use_dgps;

  uint16_t fast_offset_gy;
  uint32_t slew_veto;

  double az_accel;

  double clin_el_trim;
  double enc_motor_el_trim;
  double null_az_trim;
  double null_el_trim;
  double mag_az_trim[2];
  double pss_az_trim;
  double dgps_az_trim;

  int autotrim_enable;
  double autotrim_thresh;    // in sc sigma
  double autotrim_rate;      // degrees/s
  time_t autotrim_time;      // in seconds
  time_t autotrim_xsc0_last_bad;
  time_t autotrim_xsc1_last_bad;

  double cal_xmax_mag[2];
  double cal_xmin_mag[2];
  double cal_ymax_mag[2];
  double cal_ymin_mag[2];
  double cal_mag_align[2];

  double cal_d_pss[NUM_PSS];
  double cal_az_pss[NUM_PSS];
  double cal_az_pss_array;
  double cal_el_pss[NUM_PSS];
  double cal_roll_pss[NUM_PSS];
  double pss_noise;


  double cal_imin_pss;

  sc_commands_t sc1_commands;
  sc_commands_t sc2_commands;
  sc_thread_bools_t sc_bools;
  sc_resets_t sc_resets;

  lj_pbob_t if_power;

  lj_pbob_t of_power;


  labjack_queue_t Labjack_Queue;

  cmd_balance_t balance;

  ec_devices_struct_t ec_devices;

  struct {
    int off;
    int force_repoll;

    /* arbitrary command */
    int cindex;
    int caddr[3];
    char command[3][CMD_STRING_LEN];

    /* thermal control */
    double g_primary;
    double g_secondary;
    int tc_step;
    int tc_wait;
    int tc_mode;
    int tc_prefp;
    int tc_prefs;
    double tc_spread;
    int sf_offset;
    int sf_time;

    /* actuator control */
    int act_vel;
    int act_acc;
    int act_hold_i;
    int act_move_i;
    uint16_t act_tol;

    /* low-level actuator servo */
    int focus_mode;
    int goal[3];
    int delta[3];
    int offset[3];
    int trim[3];
    int focus;

    /* lock control */
    int lock_vel;
    int lock_acc;
    int lock_hold_i;
    int lock_move_i;

    uint32_t lock_goal;

    /* shutter control */
    int shutter_step;
    int shutter_step_slow;
    int shutter_out;
    int shutter_move_i;
    int shutter_hold_i;
    int shutter_vel;
    int shutter_acc;

    uint32_t  shutter_goal;
  } actbus;

  int pin_is_in;
  int mag_reset;
  int inc_reset;

  struct {
    int x1, y1, x2, y2, step, xvel, yvel, is_new, mode;
    int force_repoll;
  } xystage;

  /* sensors output: read in mcp:SensorReader() */
  uint16_t temp1, temp2, temp3;
  uint16_t df;

  uint16_t plover;

  struct PointingModeStruct pointing_mode; // meta mode (map, scan, etc)
  double lat;
  double lon;

  struct {
    int pulse_width;
    int fast_pulse_width;
    int reconnect;
    int autofocus;
    int save_period;
    int auto_save;
    int max_age;    // maximum allowed time between trigger and solution
    int age;	    // last measured time between trigger and solution
  } ISCControl[2];

  struct XSCCommandStruct XSC[2];

  slinger_commanding_t packet_slinger;

  uint32_t checksum;
};

void InitCommandData();
double LockPosition(double);
int SIndex(enum singleCommand);
int MIndex(enum multiCommand);
void WritePrevStatus();

extern struct CommandDataStruct CommandData;

#endif   // COMMAND_STRUCT_H

