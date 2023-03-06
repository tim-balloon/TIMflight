/* mcp: the BLAST master control program
 *
 * commands.c: functions for listening to and processing commands
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include <conversions.h>
#include <crc.h>
#include <pointing.h>
#include <ec_motors.h>

#include <linklist.h>
#include <linklist_compress.h>

#include "command_list.h"
#include "command_struct.h"
#include "framing.h"
#include "mcp.h"
#include "tx.h"
#include "pointing_struct.h"
#include "channels_tng.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "sip.h"
#include "watchdog.h"
#include "comparison.h"

/* Lock positions are nominally at 5, 15, 25, 35, 45, 55, 65, 75
 * 90 degrees.  This is the offset to the true lock positions.
 * This number is relative to the elevation encoder reading, NOT
 * true elevation */
// #define LOCK_OFFSET (-0.77) /* Updated by LMF on July 12th, 2012 */
#define LOCK_OFFSET (0.0)
#define NUM_LOCK_POS 10
static const double lock_positions[NUM_LOCK_POS] = {0.03, 5.01, 14.95, 24.92, 34.88, 44.86, 54.83, 64.81, 74.80, 89.78};

/* based on xsc0.h */
#define ISC_SHUTDOWN_NONE     0
#define ISC_SHUTDOWN_HALT     1
#define ISC_SHUTDOWN_REBOOT   2
#define ISC_SHUTDOWN_CAMCYCLE 3

#define ISC_TRIGGER_INT  0
#define ISC_TRIGGER_EDGE 1
#define ISC_TRIGGER_POS  2
#define ISC_TRIGGER_NEG  3

// Penn highbay
#define PSN_EAST_BAY_LAT 31.779300
#define PSN_EAST_BAY_LON 264.283000
// MCM-LDB
#define MCM_LDB_LAT -77.8616
#define MCM_LDB_LON 167.0592

/*
 * The distanace (in ULPS) between two floating-point numbers above which they
 * will be considered different.
 */
#define MAXULPS (DEFAULT_MAXULPS)

void RecalcOffset(double, double);  /* actuators.c */

/* defined in pointing.c */
void SetTrimToSC(int);
void ClearTrim();
void AzElTrim(double, double);
void NormalizeAngle(double*);

int LoadUplinkFile(int slot); /*sched.c */

extern int doing_schedule; /* sched.c */

extern linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES];
extern linklist_t * telemetries_linklist[NUM_TELEMETRIES];
extern char * ROACH_TYPES[NUM_RTYPES];
extern int ResetLog;

extern int16_t SouthIAm;
pthread_mutex_t mutex;

struct SIPDataStruct SIPData = {.GPSpos = {.lat = MCM_LDB_LAT, .lon = MCM_LDB_LON}};
struct CommandDataStruct CommandData;

const char* SName(enum singleCommand command); // share/sip.c
char * linklist_nt[64] = {NULL};

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus()
{
  int fp, n;

  CommandData.checksum = 0;
  CommandData.checksum = crc32_le(0, (uint8_t*) &CommandData, sizeof(CommandData));
  /** write the default file */
  fp = open(PREV_STATUS_FILE, O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "mcp.prev_status close()");
    return;
  }

  // framing_publish_command_data(&CommandData);
}

/* calculate the nearest lockable elevation */
double LockPosition(double elevation)
{
  int i_pos;
  double min_err = 360.0;
  double err;
  int i_min_err = 0;

  for (i_pos = 0; i_pos <NUM_LOCK_POS; i_pos++) {
    err = fabs(elevation - lock_positions[i_pos]);
    if (err < min_err) {
      i_min_err = i_pos;
      min_err = err;
    }
  }
  return (lock_positions[i_min_err] + LOCK_OFFSET);
}

static bool xsc_command_applies_to(int which_to_check, int which)
{
    if (which_to_check == 0 || which_to_check == 1) {
        if (which == 2 || which == which_to_check) {
            return true;
        }
    }
    return false;
}

void xsc_activate_command(int which, int command_index)
{
    if (command_index < xC_num_command_admins) {
        CommandData.XSC[which].net.command_admins[command_index].is_new_countdown =
                CommandData.XSC[which].is_new_window_period_cs;
        CommandData.XSC[which].net.command_admins[command_index].counter++;
    } else {
        blast_warn("Warning: xsc_activate_command called with invalid index");
    }
}

void SingleCommand(enum singleCommand command, int scheduled)
{
#ifndef BOLOTEST
    int i_point = GETREADINDEX(point_index);
    double sun_az;
#endif

    if (!scheduled)
    blast_info("Commands: Single command: %d (%s)\n", command, SName(command));

//   Update CommandData structure with new info

    switch (command) {
        /* HOUSEKEEPING */

        /* DETECTORS */

        /* POINTING */
        case antisun:  // turn antisolar (az-only)
            sun_az = PointingData[i_point].sun_az + 250;  // point solar panels to sun
            NormalizeAngle(&sun_az);
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_AZEL_GOTO;
            CommandData.pointing_mode.X = sun_az;   // az
            CommandData.pointing_mode.Y = PointingData[i_point].el;   // el
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            break;
        case stop:  // Pointing abort
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_DRIFT;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = 0;
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            break;
        // Vetoes & Allows
        case elclin_allow:
            CommandData.use_elclin = 1;
            break;
        case elclin_veto:
            CommandData.use_elclin = 0;
            break;
        case elmotenc_allow:
            CommandData.use_elmotenc = 1;
            break;
        case elmotenc_veto:
            CommandData.use_elmotenc = 0;
            break;
        case xsc0_allow:
            CommandData.use_xsc0 = 1;
            break;
        case xsc0_veto:
            CommandData.use_xsc0 = 0;
            break;
        case xsc1_allow:
            CommandData.use_xsc1 = 1;
            break;
        case xsc1_veto:
            CommandData.use_xsc1 = 0;
            break;
        case dgps_allow:
            CommandData.use_dgps = 1;
            break;
        case dgps_veto:
            CommandData.use_dgps = 0;
            break;
        case ifroll_1_gy_allow:
            CommandData.gymask |= 0x01;
            break;
        case ifroll_1_gy_veto:
            CommandData.gymask &= ~0x01;
            break;
        case ifyaw_1_gy_allow:
            CommandData.gymask |= 0x04;
            break;
        case ifyaw_1_gy_veto:
            CommandData.gymask &= ~0x04;
            break;
        case ifel_1_gy_allow:
            CommandData.gymask |= 0x10;
            break;
        case ifel_1_gy_veto:
            CommandData.gymask &= ~0x10;
            break;
        case ifroll_2_gy_allow:
            CommandData.gymask |= 0x02;
            break;
        case ifroll_2_gy_veto:
            CommandData.gymask &= ~0x02;
            break;
        case ifyaw_2_gy_allow:
            CommandData.gymask |= 0x08;
            break;
        case ifyaw_2_gy_veto:
            CommandData.gymask &= ~0x08;
            break;
        case ifel_2_gy_allow:
            CommandData.gymask |= 0x20;
            break;
        case ifel_2_gy_veto:
            CommandData.gymask &= ~0x20;
            break;
        case mag_allow_fc1:
            CommandData.use_mag1 = 1;
            break;
        case mag_veto_fc1:
            CommandData.use_mag1 = 0;
            break;
        case mag_allow_fc2:
            CommandData.use_mag2 = 1;
            break;
        case mag_veto_fc2:
            CommandData.use_mag2 = 0;
            break;
        case pss_allow:
            CommandData.use_pss = 1;
            break;
        case pss_veto:
            CommandData.use_pss = 0;
            break;
        // Sensors
        case az_auto_gyro:
            CommandData.az_autogyro = 1;
            break;
        case el_auto_gyro:
            CommandData.el_autogyro = 1;
            break;
        case mag_reset:
            CommandData.mag_reset = 1;
            break;
        // Trims
        case trim_to_xsc0:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            SetTrimToSC(0);
            break;
        case trim_to_xsc1:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            SetTrimToSC(1);
            break;
        case trim_xsc0_to_xsc1:
            trim_xsc(1);
            break;
        case trim_xsc1_to_xsc0:
            trim_xsc(0);
            break;
        case autotrim_off:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            break;
        case reset_trims:
            CommandData.autotrim_enable = 0;
            CommandData.autotrim_rate = 0.0;
            ClearTrim();
            break;

        /* MOTORS */
        case rw_wake_and_wiggle:
            CommandData.ec_devices.have_commutated_rw = 0;
            break;
        // Motor gains
        case az_off: // disable az motors
            CommandData.disable_az = 1;
            break;
        case az_on: // enable az motors
            CommandData.disable_az = 0;
            break;
        case el_off:  // disable el motors
            CommandData.disable_el = 1;
            CommandData.force_el = 0;
            break;
        case el_on: // enable el motors
            CommandData.disable_el = 0;
            CommandData.force_el = 0;
            break;
        case force_el_on:  // force enabling of el motors
            CommandData.disable_el = 0;
            CommandData.force_el = 1;
            break;
        // Resets
        case reset_rw:
            rw_reset_fault();
            break;
        case reset_piv:
            piv_reset_fault();
            break;
        case reset_elev:
            el_reset_fault();
            break;
        case reset_ethercat:
            CommandData.ec_devices.reset = 1;
            break;
        case restore_piv:
            CommandData.restore_piv = 1;
            break;
        /* ACTUATORS */
        case actbus_on:
            CommandData.actbus.off = 0;
            CommandData.actbus.force_repoll = 1;
            break;
        case actbus_off:
            CommandData.actbus.off = -1;
            break;
        case actbus_cycle:
            CommandData.actbus.off = PCYCLE_HOLD_LEN;
            CommandData.actbus.force_repoll = 1;
            break;
        case repoll:
            CommandData.actbus.force_repoll = 1;
        #ifdef USE_XY_THREAD
            CommandData.xystage.force_repoll = 1;
        #endif
            break;
        // Shutter
        case shutter_off:
            CommandData.actbus.shutter_goal = SHUTTER_OFF;
            break;
        case shutter_init:
            CommandData.actbus.shutter_goal = SHUTTER_INIT;
            break;
        case shutter_reset:
            CommandData.actbus.shutter_goal = SHUTTER_RESET;
            break;
        case shutter_close:
            CommandData.actbus.shutter_goal = SHUTTER_CLOSED;
            break;
        case shutter_open:
            CommandData.actbus.shutter_goal = SHUTTER_OPEN;
            break;
        case shutter_keepopen:
            CommandData.actbus.shutter_goal = SHUTTER_KEEPOPEN;
            break;
        case shutter_keepclosed:
            CommandData.actbus.shutter_goal = SHUTTER_KEEPCLOSED;
            break;
        // Lock pin
        case pin_in:
            CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF | LS_IGNORE_EL;
            break;
        case unlock:
            CommandData.actbus.lock_goal = LS_OPEN | LS_DRIVE_OFF;
            if (CommandData.pointing_mode.mode == P_LOCK) {
                CommandData.pointing_mode.nw = CommandData.slew_veto;
                CommandData.pointing_mode.mode = P_DRIFT;
                CommandData.pointing_mode.X = 0;
                CommandData.pointing_mode.Y = 0;
                CommandData.pointing_mode.vaz = 0.0;
                CommandData.pointing_mode.del = 0.0;
                CommandData.pointing_mode.w = 0;
                CommandData.pointing_mode.h = 0;
            }
            break;
        case lock_off:
            CommandData.actbus.lock_goal = LS_DRIVE_OFF | LS_DRIVE_FORCE;
            break;
        case lock45:   // Lock Inner Frame at 45 (to be sent by CSBF pre-termination)
            if (CommandData.pointing_mode.nw >= 0) {
                CommandData.pointing_mode.nw = VETO_MAX;
            }
            CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_LOCK;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = LockPosition(45.0);
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            CommandData.pointing_mode.vaz = 0;
            CommandData.pointing_mode.del = 0;
            blast_info("Commands: Lock at : %g\n", CommandData.pointing_mode.Y);
            break;
        // Secondary Mirror
        case autofocus_allow:
            CommandData.actbus.tc_mode = TC_MODE_ENABLED;
            break;
        case autofocus_veto:
            CommandData.actbus.tc_mode = TC_MODE_VETOED;
            break;
        case actuator_stop:
            CommandData.actbus.focus_mode = ACTBUS_FM_PANIC;
            CommandData.actbus.tc_mode = TC_MODE_VETOED;
            break;
        // Balance system
        case balance_auto:
            CommandData.balance.mode = bal_auto;
            break;
        case balance_off:
            CommandData.balance.mode = bal_rest;
            break;
        case balance_terminate:
            // after lock45, before termination, drive balance system to lower limit
            CommandData.balance.vel = 200000;
            CommandData.balance.mode = bal_manual;
            CommandData.balance.bal_move_type = 2;
            break;

        /* STAR CAMERAS */

        /* MISC */
        // Video transmitters
        case vtx_xsc0:
            CommandData.vtx_sel[0] = VTX_XSC0;
            // CommandData.Relays.video_trans = 0;
            // CommandData.Relays.update_video = 1;
            break;
        case vtx_xsc1:
            CommandData.vtx_sel[0] = VTX_XSC1;
            // CommandData.Relays.video_trans = 1;
            // CommandData.Relays.update_video = 1;
            break;
        // XY stage
        #ifdef USE_XY_THREAD
        case xy_panic:
            CommandData.xystage.mode = XYSTAGE_PANIC;
            CommandData.xystage.is_new = 1;
            break;
        #endif
        // BLAST(TIM) Misc
        case reap_fc1:  // Miscellaneous commands
            if (command == reap_fc1 && !SouthIAm) {
                blast_warn("Commands: Reaping the watchdog tickle due to command.\n");
                watchdog_stop();
            }
            break;
        case reap_fc2:
            if (command == reap_fc2 && SouthIAm) {
                blast_warn("Commands: Reaping the watchdog tickle due to command.\n");
                watchdog_stop();
            }
            break;
        case halt_fc1:
            if (command == halt_fc1 && !SouthIAm) {
                blast_warn("Commands: Halting the MCC\n");
                if (system("/sbin/reboot") < 0) berror(fatal, "Commands: failed to reboot, dying\n");
            }
            break;
        case halt_fc2:
            if (command == halt_fc2 && SouthIAm) {
                blast_warn("Commands: Halting the MCC\n");
                if (system("/sbin/reboot") < 0) berror(fatal, "Commands: failed to reboot, dying\n");
            }
            break;
        case blast_rocks:
            CommandData.sucks = 0;
            CommandData.uplink_sched = 0;
            break;
        case blast_sucks:
            CommandData.sucks = 1;
            CommandData.uplink_sched = 0;
            break;
        case at_float:
            CommandData.at_float = 1;
            break;
        case not_at_float:
            CommandData.at_float = 0;
            break;
        case gps_sw_reset:
            if (system("/usr/local/bin/gps_sw_reset") != 0) {
                blast_err("Commands: failed to reboot gps software\n");
            }
            break;
        case gps_stats:
            if (system("/usr/local/bin/gps_stats") != 0) {
                blast_err("Commands: failed to check gps stats\n");
            } else {
                setenv("JLTGPS", "/data/etc/blast/gps/stats.txt", 1);
            }
            break;
        case reset_log:
            ResetLog = 1;
            break;
        case xyzzy:
            break;
        default:
            bputs(warning, "Commands: ***Invalid Single Word Command***\n");
            return;  // invalid command - no write or update
    }

    CommandData.command_count++;
    CommandData.last_command = (uint16_t) command;

#ifndef BOLOTEST
    if (!scheduled) {
        // TODO(seth): RE-enable doing_schedule
        if (doing_schedule)
             blast_info("Scheduler: *** Out of schedule file mode ***");
        CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
    } else {
        CommandData.pointing_mode.t = PointingData[i_point].t;
    }
#endif

    WritePrevStatus();
}

static inline void copysvalue(char* dest, const char* src)
{
  strncpy(dest, src, CMD_STRING_LEN - 1);
  dest[CMD_STRING_LEN - 1] = '\0';
}

void MultiCommand(enum multiCommand command, double *rvalues,
    int *ivalues, char svalues[][CMD_STRING_LEN], int scheduled)
{
  int i;
  int is_new;
  char * filename;

//   Update CommandData struct with new info
//   * If the parameter is type 'i'/'l' set CommandData using ivalues[i]
//   * If the parameter is type 'f'/'d' set CommandData using rvalues[i]


//   Pointing Modes
  switch (command) {
        /* HOUSEKEEPING */

        /* DETECTORS */

        /* POINTING */
        case slew_veto:
            CommandData.slew_veto = rvalues[0] * SR;
                blast_info("CommandData.slew_veto = %i, CommandData.pointing_mode.nw = %i",
                            CommandData.slew_veto, CommandData.pointing_mode.nw);
            if (CommandData.pointing_mode.nw > CommandData.slew_veto) {
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            break;
        // Scans
        case az_scan_accel:
            CommandData.az_accel = rvalues[0];
            if (CommandData.az_accel < 0.005) {
                blast_warn("Attempt to set az_accel to %f, that is too low! Setting %f instead",
                           rvalues[0], CommandData.az_accel);
            }
            break;
        case set_scan_params:
            CommandData.pointing_mode.next_i_dith = ivalues[0];
            break;
        case ra_dec_goto:
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_RADEC_GOTO;
            CommandData.pointing_mode.X = rvalues[0];  // ra
            CommandData.pointing_mode.Y = rvalues[1];  // dec
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.vaz = 0;
            CommandData.pointing_mode.del = 0;
            CommandData.pointing_mode.h = 0;
            break;
        case az_el_goto:
            if ((CommandData.pointing_mode.mode != P_AZEL_GOTO) ||
                !is_almost_equal(CommandData.pointing_mode.X, rvalues[0], MAXULPS) ||
                !is_almost_equal(CommandData.pointing_mode.Y, rvalues[1], MAXULPS)) {
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            // zero unused parameters
            for (i = 0; i < 4; i++) {
                CommandData.pointing_mode.ra[i] = 0;
                CommandData.pointing_mode.dec[i] = 0;
            }
            CommandData.pointing_mode.mode = P_AZEL_GOTO;
            CommandData.pointing_mode.X = rvalues[0];   // az
            CommandData.pointing_mode.Y = rvalues[1];   // el
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            break;
        case box:
            if ((CommandData.pointing_mode.mode != P_BOX) ||
                !is_almost_equal(CommandData.pointing_mode.X, rvalues[0], MAXULPS) ||  // ra
                !is_almost_equal(CommandData.pointing_mode.Y, rvalues[1], MAXULPS) ||  // dec
                !is_almost_equal(CommandData.pointing_mode.w, rvalues[2], MAXULPS) ||  // width
                !is_almost_equal(CommandData.pointing_mode.h, rvalues[3], MAXULPS) ||  // height
                !is_almost_equal(CommandData.pointing_mode.vaz, rvalues[4], MAXULPS) ||  // az scan speed
                !is_almost_equal(CommandData.pointing_mode.del, rvalues[5], MAXULPS)) {  // el step size
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            // zero unused parameters
            for (i = 0; i < 4; i++) {
                CommandData.pointing_mode.ra[i] = 0;
                CommandData.pointing_mode.dec[i] = 0;
            }
            CommandData.pointing_mode.mode = P_BOX;
            CommandData.pointing_mode.X = rvalues[0];  // ra
            CommandData.pointing_mode.Y = rvalues[1];  // dec
            CommandData.pointing_mode.w = rvalues[2];  // width
            CommandData.pointing_mode.h = rvalues[3];  // height
            CommandData.pointing_mode.vaz = rvalues[4];  // az scan speed
            CommandData.pointing_mode.del = rvalues[5];  // el step size
            CommandData.pointing_mode.n_dith = ivalues[6];  // number of el dither steps
            break;
        case quad:
            is_new = 0;
            if ((CommandData.pointing_mode.mode != P_QUAD) ||
                !is_almost_equal(CommandData.pointing_mode.vaz, rvalues[8], MAXULPS) ||  // az scan speed
                !is_almost_equal(CommandData.pointing_mode.del, rvalues[9], MAXULPS)) { // el step size
                is_new = 1;
            }
            for (i = 0; i < 4; i++) {
                if (!is_almost_equal(CommandData.pointing_mode.ra[i], rvalues[i * 2], MAXULPS) ||
                    !is_almost_equal(CommandData.pointing_mode.dec[i], rvalues[i * 2 + 1], MAXULPS)) {
                    is_new = 1;
                }
            }
            if (is_new) {
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            CommandData.pointing_mode.X = 0;  // ra
            CommandData.pointing_mode.Y = 0;  // dec
            CommandData.pointing_mode.w = 0;  // width
            CommandData.pointing_mode.h = 0;  // height
            CommandData.pointing_mode.mode = P_QUAD;
            for (i = 0; i < 4; i++) {
                CommandData.pointing_mode.ra[i] = rvalues[i * 2];
                CommandData.pointing_mode.dec[i] = rvalues[i * 2 + 1];
            }
            CommandData.pointing_mode.vaz = rvalues[8];  // az scan speed
            CommandData.pointing_mode.del = rvalues[9];  // el step size
            CommandData.pointing_mode.n_dith = ivalues[10];  // N dither steps
            break;
        case cap:
            if ((CommandData.pointing_mode.mode != P_CAP) ||
                !is_almost_equal(CommandData.pointing_mode.X, rvalues[0], MAXULPS) ||   // ra
                !is_almost_equal(CommandData.pointing_mode.Y, rvalues[1], MAXULPS) ||   // dec
                !is_almost_equal(CommandData.pointing_mode.w, rvalues[2], MAXULPS) ||   // radius
                !is_almost_equal(CommandData.pointing_mode.vaz, rvalues[3], MAXULPS) ||   // az scan speed
                !is_almost_equal(CommandData.pointing_mode.del, rvalues[4], MAXULPS) ||   // el step size
                !is_almost_equal(CommandData.pointing_mode.h, 0, MAXULPS)) {  // N dither steps
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            // zero unused parameters
            for (i = 0; i < 4; i++) {
                CommandData.pointing_mode.ra[i] = 0;
                CommandData.pointing_mode.dec[i] = 0;
            }
            CommandData.pointing_mode.mode = P_CAP;
            CommandData.pointing_mode.X = rvalues[0];  // ra
            CommandData.pointing_mode.Y = rvalues[1];  // dec
            CommandData.pointing_mode.w = rvalues[2];  // radius
            CommandData.pointing_mode.vaz = rvalues[3];  // az scan speed
            CommandData.pointing_mode.del = rvalues[4];  // el step size
            CommandData.pointing_mode.h = 0;
            CommandData.pointing_mode.n_dith = ivalues[5];  // No of dither steps
            break;
        case el_scan:
            //      blast_info("Commands: El scan not enabled yet!");
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_EL_SCAN;
            CommandData.pointing_mode.X = rvalues[0];   // az
            CommandData.pointing_mode.Y = rvalues[1];   // el
            //      blast_info("Scan center: %f, %f", CommandData.pointing_mode.X, CommandData.pointing_mode.Y);
            CommandData.pointing_mode.h = rvalues[2];   // height
            CommandData.pointing_mode.vel = rvalues[3];  // az scan speed
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            break;
        case el_box:
            if ((CommandData.pointing_mode.mode != P_EL_BOX) ||
                !is_almost_equal(CommandData.pointing_mode.X, rvalues[0], MAXULPS) ||  // ra
                !is_almost_equal(CommandData.pointing_mode.Y, rvalues[1], MAXULPS) ||  // dec
                !is_almost_equal(CommandData.pointing_mode.w, rvalues[2], MAXULPS) ||  // width
                !is_almost_equal(CommandData.pointing_mode.h, rvalues[3], MAXULPS) ||  // height
                !is_almost_equal(CommandData.pointing_mode.vel, rvalues[4], MAXULPS) ||  // az scan speed
                !is_almost_equal(CommandData.pointing_mode.daz, rvalues[5], MAXULPS)) {  // el step size
                CommandData.pointing_mode.nw = CommandData.slew_veto;
            }
            // zero unused parameters
            for (i = 0; i < 4; i++) {
                CommandData.pointing_mode.ra[i] = 0;
                CommandData.pointing_mode.dec[i] = 0;
            }
            CommandData.pointing_mode.mode = P_EL_BOX;
            CommandData.pointing_mode.X = rvalues[0];  // ra
            CommandData.pointing_mode.Y = rvalues[1];  // dec
            CommandData.pointing_mode.w = rvalues[2];  // width
            CommandData.pointing_mode.h = rvalues[3];  // height
            CommandData.pointing_mode.vel = rvalues[4];  // az scan speed
            CommandData.pointing_mode.daz = rvalues[5];  // el step size
            CommandData.pointing_mode.n_dith = ivalues[6];  // number of el dither steps
            break;
        case az_scan:
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_AZ_SCAN;
            CommandData.pointing_mode.X = rvalues[0];   // az
            CommandData.pointing_mode.Y = rvalues[1];   // el
            blast_info("Scan center: %f, %f", CommandData.pointing_mode.X, CommandData.pointing_mode.Y);
            CommandData.pointing_mode.w = rvalues[2];   // width
            CommandData.pointing_mode.vaz = rvalues[3];  // az scan speed
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.h = 0;
            break;
        case cur_mode:
            CommandData.pointing_mode.mode = P_CURRENT;
            CommandData.pointing_mode.X = rvalues[0];  // pivot current
            CommandData.pointing_mode.Y = rvalues[1];  // rw current
            CommandData.pointing_mode.w = rvalues[2];  // el current
            break;
        case drift:
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_DRIFT;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = 0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.vaz = rvalues[0];  // az speed
            CommandData.pointing_mode.del = rvalues[1];  // el speed
            CommandData.pointing_mode.h = 0;
            break;
        // Magnetometer
        case mag_cal_fc1:
            CommandData.cal_xmax_mag[0] = rvalues[0];
            CommandData.cal_xmin_mag[0] = rvalues[1];
            CommandData.cal_ymax_mag[0] = rvalues[2];
            CommandData.cal_ymin_mag[0] = rvalues[3];
            CommandData.cal_mag_align[0] = rvalues[4];
            blast_info("Updating mag1 cal coeffs: xmax = %f, xmin = %f, ymin = %f, ymax = %f, align = %f",
                        CommandData.cal_xmax_mag[0], CommandData.cal_xmin_mag[0],
                        CommandData.cal_ymax_mag[0], CommandData.cal_ymin_mag[0], CommandData.cal_mag_align[0]);
            break;
        case mag_cal_fc2:
            CommandData.cal_xmax_mag[1] = rvalues[0];
            CommandData.cal_xmin_mag[1] = rvalues[1];
            CommandData.cal_ymax_mag[1] = rvalues[2];
            CommandData.cal_ymin_mag[1] = rvalues[3];
            CommandData.cal_mag_align[1] = rvalues[4];
            blast_info("Updating mag1 cal coeffs: xmax = %f, xmin = %f, ymin = %f, ymax = %f, align = %f",
                        CommandData.cal_xmax_mag[1], CommandData.cal_xmin_mag[1],
                        CommandData.cal_ymax_mag[1], CommandData.cal_ymin_mag[1], CommandData.cal_mag_align[1]);
            break;
        // PSS
        case pss_cal_n:
            i = ivalues[0]-1;
            CommandData.cal_d_pss[i] = rvalues[1];
            CommandData.cal_az_pss[i] = rvalues[2];
            CommandData.cal_el_pss[i] = rvalues[3];
            CommandData.cal_roll_pss[i] = rvalues[4];
            break;
        case pss_set_noise:
            CommandData.pss_noise = rvalues[0];
            break;
        case pss_cal_d:
            for (i = 0; i < NUM_PSS; i++) {
                CommandData.cal_d_pss[i] = rvalues[i];
            }
            break;
        case pss_cal_el:
            for (i = 0; i < NUM_PSS; i++) {
                CommandData.cal_el_pss[i] = rvalues[i];
            }
            break;
        case pss_cal_az:
            for (i = 0; i < NUM_PSS; i++) {
                CommandData.cal_az_pss[i] = rvalues[i];
            }
            break;
        case pss_cal_roll:
            for (i = 0; i < NUM_PSS; i++) {
                CommandData.cal_roll_pss[i] = rvalues[i];
            }
            break;
        case pss_cal_array_az:
            CommandData.cal_az_pss_array = rvalues[0];
            break;
        case pss_set_imin:
            CommandData.cal_imin_pss = rvalues[0];
            // blast_info("Changed PSS min current to: %f", CommandData.cal_imin_pss);
            break;
        // Gyros
        case az_gyro_offset:
            CommandData.offset_ifroll_gy = rvalues[0];
            CommandData.offset_ifyaw_gy = rvalues[1];
            CommandData.az_autogyro = 0;
            break;
        case el_gyro_offset:
            CommandData.offset_ifel_gy = rvalues[0];
            CommandData.el_autogyro = 0;
            break;
        // Trims
        case ra_dec_set:
            SetRaDec(rvalues[0], rvalues[1]);
            break;
        case pos_set:
            set_position(rvalues[0], rvalues[1]);
            break;
        case az_el_trim:
            AzElTrim(rvalues[0], rvalues[1]);
            break;
        case autotrim_to_sc:
            CommandData.autotrim_thresh = rvalues[0];
            CommandData.autotrim_time = ivalues[1];
            CommandData.autotrim_rate = rvalues[2];
            CommandData.autotrim_xsc0_last_bad = mcp_systime(NULL);
            CommandData.autotrim_xsc1_last_bad = CommandData.autotrim_xsc0_last_bad;
            CommandData.autotrim_enable = 1;
            break;

        /* MOTORS */
        case fix_ethercat:
            CommandData.ec_devices.fix_rw = ivalues[0];
            CommandData.ec_devices.fix_el = ivalues[1];
            CommandData.ec_devices.fix_piv = ivalues[2];
            break;
        case az_gain:   // az gains
            CommandData.azi_gain.P = rvalues[0];
            if (rvalues[1] <= 0.0005) {
                blast_err("You tried to set the Azimuth Motor time constant to less than 0.5ms!"
                          "  This is invalid, so we will assume you wanted a really long time.");
                CommandData.azi_gain.I = 1000.0;
            } else {
                CommandData.azi_gain.I = rvalues[1];
            }
            CommandData.azi_gain.D = rvalues[2];
            CommandData.azi_gain.PT = rvalues[3];
            break;
        case pivot_gain:   // pivot gains
            CommandData.pivot_gain.SP = rvalues[0];
            CommandData.pivot_gain.PE = rvalues[1];
            CommandData.pivot_gain.IE = rvalues[2];
            CommandData.pivot_gain.PV = rvalues[3];
            CommandData.pivot_gain.IV = rvalues[4];
            CommandData.pivot_gain.F = rvalues[5];
            break;
        case el_gain:   // ele gains
            CommandData.ele_gain.P = rvalues[0];
            if (rvalues[1] <= 0.0005) {
                blast_err("You tried to set the Elevation Motor time constant to less than 0.5ms!"
                          "  This is invalid, so we will assume you wanted a really long time.");
                CommandData.ele_gain.I = 1000.0;
            } else {
                CommandData.ele_gain.I = rvalues[1];
            }
            CommandData.ele_gain.D = rvalues[2];
            CommandData.ele_gain.PT = rvalues[3];
            CommandData.ele_gain.DB = rvalues[4];
            CommandData.ele_gain.F = rvalues[5];
            break;
        case motors_verbose:
            CommandData.verbose_rw = ivalues[0];
            CommandData.verbose_el = ivalues[1];
            CommandData.verbose_piv = ivalues[2];
            break;

        /* ACTUATORS */
        case actuator_i:
            CommandData.actbus.act_move_i = ivalues[0];
            CommandData.actbus.act_hold_i = ivalues[1];
            break;
        case actuator_vel:
            CommandData.actbus.act_vel = ivalues[0];
            CommandData.actbus.act_acc = ivalues[1];
            break;
        case actuator_tol:
            CommandData.actbus.act_tol = ivalues[0];
            break;
        case actuator_servo:
            CommandData.actbus.goal[0] = ivalues[0] + CommandData.actbus.offset[0];
            CommandData.actbus.goal[1] = ivalues[1] + CommandData.actbus.offset[1];
            CommandData.actbus.goal[2] = ivalues[2] + CommandData.actbus.offset[2];
            CommandData.actbus.focus_mode = ACTBUS_FM_SERVO;
            break;
        case general:  // General actuator bus command
            CommandData.actbus.caddr[CommandData.actbus.cindex] = ivalues[0] + 0x30;
            copysvalue(CommandData.actbus.command[CommandData.actbus.cindex],
                       svalues[1]);
            CommandData.actbus.cindex = INC_INDEX(CommandData.actbus.cindex);
            break;
        // Lock Pin
        case lock_vel:
            CommandData.actbus.lock_vel = ivalues[0];
            CommandData.actbus.lock_acc = ivalues[1];
            break;
        case lock_i:
            CommandData.actbus.lock_move_i = ivalues[0];
            CommandData.actbus.lock_hold_i = ivalues[1];
            break;
        case lock:   // Lock Inner Frame
            if (CommandData.pointing_mode.nw >= 0) {
                CommandData.pointing_mode.nw = VETO_MAX;
            }
            CommandData.actbus.lock_goal = LS_CLOSED | LS_DRIVE_OFF;
            CommandData.pointing_mode.nw = CommandData.slew_veto;
            CommandData.pointing_mode.mode = P_LOCK;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = LockPosition(rvalues[0]);
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            CommandData.pointing_mode.vaz = 0;
            CommandData.pointing_mode.del = 0;
            blast_info("Commands: Lock Mode: %g\n", CommandData.pointing_mode.Y);
            break;
        // Shutter
        case shutter_i:
            CommandData.actbus.shutter_move_i = ivalues[0];
            CommandData.actbus.shutter_hold_i = ivalues[1];
            break;
        case shutter_vel:
            CommandData.actbus.shutter_vel = ivalues[0];
            CommandData.actbus.shutter_acc = ivalues[1];
            break;
        // Balance
        case balance_gain:
            CommandData.balance.i_el_on_bal = rvalues[0];
            CommandData.balance.i_el_off_bal = rvalues[1];
            // CommandData.balance.i_el_target_bal = rvalues[2];
            // CommandData.balance.gain_bal = rvalues[3];
            break;
        case balance_manual:
            CommandData.balance.bal_move_type = ((int)(0 < ivalues[0]) - (int)(ivalues[0] < 0)) + 1;
            CommandData.balance.mode = bal_manual;
            break;
        case balance_vel:
            CommandData.balance.vel = ivalues[0];
            CommandData.balance.acc = ivalues[1];
            break;
        case balance_i:
            CommandData.balance.move_i = ivalues[0];
            CommandData.balance.hold_i = ivalues[1];
            break;
        // Secondary mirror
        case set_secondary:
            CommandData.actbus.focus = ivalues[0] + POSITION_FOCUS
                                    + CommandData.actbus.sf_offset;
            CommandData.actbus.focus_mode = ACTBUS_FM_FOCUS;
            break;
        case thermo_param:
            CommandData.actbus.tc_spread = rvalues[0];
            CommandData.actbus.tc_prefp = ivalues[1];
            CommandData.actbus.tc_prefs = ivalues[2];
            break;
        case focus_offset:
            CommandData.actbus.sf_offset = ivalues[0];
            CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
            break;
        case thermo_gain:
            CommandData.actbus.tc_step = ivalues[2];
            CommandData.actbus.tc_wait = ivalues[3] * 300;  // convert min->5Hz
            CommandData.actbus.sf_time = CommandData.actbus.tc_wait - 5;
            RecalcOffset(rvalues[0], rvalues[1]);
            CommandData.actbus.g_primary = rvalues[0];
            CommandData.actbus.g_secondary = rvalues[1];
            break;
        case actuator_delta:
            CommandData.actbus.delta[0] = ivalues[0];
            CommandData.actbus.delta[1] = ivalues[1];
            CommandData.actbus.delta[2] = ivalues[2];
            CommandData.actbus.focus_mode = ACTBUS_FM_DELTA;
            break;
        case delta_secondary:
            CommandData.actbus.focus = ivalues[0];
            CommandData.actbus.focus_mode = ACTBUS_FM_DELFOC;
            break;
        case act_enc_trim:
            CommandData.actbus.trim[0] = rvalues[0];
            CommandData.actbus.trim[1] = rvalues[1];
            CommandData.actbus.trim[2] = rvalues[2];
            CommandData.actbus.focus_mode = ACTBUS_FM_TRIM;
            break;

        /* TELEMETRY */
        case highrate_bw:
            // Value entered by user in kbps but stored in Bps
            CommandData.highrate_bw = rvalues[0]*1000.0/8.0;
            CommandData.highrate_allframe_fraction = rvalues[1];
            blast_info("Changed highrate bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
            break;
        case pilot_bw:
            // Value entered by user in kbps but stored in Bps
            CommandData.pilot_bw = rvalues[0]*1000.0/8.0;
            CommandData.pilot_allframe_fraction = rvalues[1];
            blast_info("Changed pilot bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
            break;
        case biphase_bw:
            // Value entered by user in kbps but stored in Bps
            CommandData.biphase_bw = rvalues[0]*1000.0/8.0;
            CommandData.biphase_allframe_fraction = rvalues[1];
            blast_info("Changed biphase bw to %f kbps (%f percent allframe)", rvalues[0], rvalues[1]*100.0);
            break;
        case biphase_clk_speed:
            // Value entered by user in kbps but stored in bps
            if (ivalues[0] == 100) {
                CommandData.biphase_clk_speed = 100000;
            } else if (ivalues[0] == 500) {
                CommandData.biphase_clk_speed = 500000;
            } else if (ivalues[0] == 1000) {
                CommandData.biphase_clk_speed = 1000000;
            } else {
                char *str;
                char *str2;
                char str3[1000];
                asprintf(&str, "Biphase clk_speed : %d kbps is not allowed (try 100, 500 or 1000).\n", ivalues[0]);
                asprintf(&str2, "Biphase clk_speed has not been changed, it\'s %d bps", CommandData.biphase_clk_speed);
                snprintf(str3, sizeof(str3), "%s %s", str, str2);
                blast_warn("%s", str3);
            }
            break;
        case highrate_through_tdrss:
            // route through tdrss or otherwise
            if (ivalues[0]) {
                CommandData.highrate_through_tdrss = true;
            } else {
                CommandData.highrate_through_tdrss = false;
            }
            break;
        case set_linklists:
            if (ivalues[0] == 0) {
                copysvalue(CommandData.pilot_linklist_name, linklist_nt[ivalues[1]]);
                telemetries_linklist[PILOT_TELEMETRY_INDEX] =
                        linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
            } else if (ivalues[0] == 1) {
                copysvalue(CommandData.bi0_linklist_name, linklist_nt[ivalues[1]]);
                telemetries_linklist[BI0_TELEMETRY_INDEX] =
                        linklist_find_by_name(CommandData.bi0_linklist_name, linklist_array);
            } else if (ivalues[0] == 2) {
                copysvalue(CommandData.highrate_linklist_name, linklist_nt[ivalues[1]]);
                telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
                        linklist_find_by_name(CommandData.highrate_linklist_name, linklist_array);
            } else if (ivalues[0] == 3) {
                copysvalue(CommandData.sbd_linklist_name, linklist_nt[ivalues[1]]);
                telemetries_linklist[SBD_TELEMETRY_INDEX] =
                        linklist_find_by_name(CommandData.sbd_linklist_name, linklist_array);
            } else {
                blast_err("Unknown downlink index %d", ivalues[0]);
            }
            break;
        case request_file:
            filename = svalues[3];
            if (svalues[3][0] == '$') {
                filename = getenv(svalues[3]+1); // hook for environment variable
            }
            if (filename && linklist_send_file_by_block_ind(
                                           linklist_find_by_name(FILE_LINKLIST, linklist_array),
                                           "file_block",
                                           filename,
                                           ivalues[1],
                                           BLOCK_OVERRIDE_CURRENT,
                                           (ivalues[2] > 0) ? ivalues[2]-1 : 0,
                                           (ivalues[2] > 0) ? ivalues[2]   : 0)) {
                if (ivalues[0] == 0) { // pilot
                    CommandData.pilot_bw = MIN(1000.0*1000.0/8.0, CommandData.pilot_bw); // max out bw
                    telemetries_linklist[PILOT_TELEMETRY_INDEX] =
                            linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else if (ivalues[0] == 1) { // BI0
                    CommandData.biphase_bw = MIN(1000.0*1000.0/8.0, CommandData.biphase_bw); // max out bw
                    telemetries_linklist[BI0_TELEMETRY_INDEX] =
                            linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else if (ivalues[0] == 2) { // highrate
                    telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
                            linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else {
                    blast_err("Cannot send files over link index %d", ivalues[0]);
                    break;
                }
            } else { // set the indices to 0 so that file transfers are stopped
                blast_err("Could not resolve filename \"%s\"", svalues[3]);
            }
            break;
        case request_stream_file:
            filename = (char *) stream_types[ivalues[3]];
            if (filename[0] == '$') {
                filename = getenv(filename+1); // hook for environment variable
            }
            if (filename && linklist_send_file_by_block_ind(
                                          linklist_find_by_name(FILE_LINKLIST, linklist_array),
                                          "file_block",
                                           filename,
                                           ivalues[1],
                                           BLOCK_OVERRIDE_CURRENT,
                                           (ivalues[2] > 0) ? ivalues[2]-1 : 0,
                                           (ivalues[2] > 0) ? ivalues[2]   : 0)) {
                if (ivalues[0] == 0) { // pilot
                    CommandData.pilot_bw = MIN(1000.0*1000.0/8.0, CommandData.pilot_bw); // max out bw
                    telemetries_linklist[PILOT_TELEMETRY_INDEX] =
                            linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else if (ivalues[0] == 1) { // BI0
                    CommandData.biphase_bw = MIN(1000.0*1000.0/8.0, CommandData.biphase_bw); // max out bw
                    telemetries_linklist[BI0_TELEMETRY_INDEX] =
                            linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else if (ivalues[0] == 2) { // highrate
                    telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
                    linklist_find_by_name(FILE_LINKLIST, linklist_array);
                } else {
                    blast_err("Cannot send files over link index %d", ivalues[0]);
                    break;
                }
            } else { // set the indices to 0 so that file transfers are stopped
                blast_err("Could not resolve filename \"%s\"", filename);
            }
            break;
        case set_pilot_oth:
            CommandData.pilot_oth = ivalues[0];
            blast_info("Switched to Pilot to stream to \"%s\"\n", pilot_target_names[CommandData.pilot_oth]);
            break;

        /* STAR CAMERAS */
        case xsc_is_new_window_period:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].is_new_window_period_cs = ivalues[1];
                    }
                }
                break;
            }
        case xsc_offset:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].cross_el_trim = from_degrees(rvalues[1]);
                        CommandData.XSC[which].el_trim = from_degrees(rvalues[2]);
                    }
                }
                break;
            }
        case xsc_heaters_off:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].heaters.mode = xsc_heater_off;
                    }
                }
                break;
            }
        case xsc_heaters_on:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].heaters.mode = xsc_heater_on;
                    }
                }
                break;
            }
        case xsc_heaters_auto:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].heaters.mode = xsc_heater_auto;
                        CommandData.XSC[which].heaters.setpoint = rvalues[1];
                    }
                }
                break;
            }
        case xsc_exposure_timing:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].trigger.exposure_time_cs = ivalues[1];
                    }
                    // Commanded value is in seconds!
                    CommandData.XSC[which].trigger.grace_period_cs = rvalues[2] * 100.0;
                    CommandData.XSC[which].trigger.post_trigger_counter_mcp_share_delay_cs = ivalues[3];
                }
                break;
            }
        case xsc_multi_trigger:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    CommandData.XSC[which].trigger.num_triggers = ivalues[1];
                    CommandData.XSC[which].trigger.multi_trigger_time_between_triggers_cs = ivalues[2];
                    xsc_activate_command(which, xC_multi_triggering);
                }
                break;
            }
        case xsc_trigger_threshold:
            {
                int which = 0;
                for (which = 0; which < 2; which++) {
                    CommandData.XSC[which].trigger.threshold.enabled = (ivalues[1] != 0);
                    CommandData.XSC[which].trigger.threshold.blob_streaking_px = rvalues[2];
                }
                break;
            }
        case xsc_scan_force_trigger:
            {
                int which = 0;
                for (which = 0; which < 2; which++) {
                    CommandData.XSC[which].trigger.scan_force_trigger_enabled = (ivalues[1] != 0);
                }
                break;
            }
        case xsc_quit:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_quit);
                    }
                }
                break;
            }
        case xsc_shutdown:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.shutdown.shutdown_now = true;
                        CommandData.XSC[which].net.shutdown.include_restart = (ivalues[1] != 0);
                        xsc_activate_command(which, xC_shutdown);
                    }
                }
                break;
            }
        case xsc_main_settings:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.main_settings.display_frequency = rvalues[1];
                        CommandData.XSC[which].net.main_settings.display_fullscreen = (ivalues[2] != 0);
                        CommandData.XSC[which].net.main_settings.display_image_only = (ivalues[3] != 0);
                        CommandData.XSC[which].net.main_settings.display_solving_filters = (ivalues[4] != 0);
                        CommandData.XSC[which].net.main_settings.display_image_brightness = rvalues[5];
                        xsc_activate_command(which, xC_main_settings);
                    }
                }
                break;
            }
        case xsc_display_zoom:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.main_settings.display_zoom_x = ivalues[1];
                        CommandData.XSC[which].net.main_settings.display_zoom_y = ivalues[2];
                        CommandData.XSC[which].net.main_settings.display_zoom_magnitude = rvalues[3];
                        xsc_activate_command(which, xC_display_zoom);
                    }
                }
                break;
            }
        case xsc_image_client:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.image_client_enabled = (ivalues[1] != 0);
                        xsc_activate_command(which, xC_image_client);
                    }
                }
                break;
            }
        case xsc_init_focus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_init_focus);
                    }
                }
                break;
            }
        case xsc_get_focus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_get_focus);
                    }
                }
                break;
            }
        case xsc_set_focus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.set_focus_value = ivalues[1];
                        xsc_activate_command(which, xC_set_focus);
                    }
                }
                break;
            }
        case xsc_stop_focus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_stop_focus);
                    }
                }
                break;
            }
        case xsc_define_focus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.define_focus_value = ivalues[1];
                        xsc_activate_command(which, xC_define_focus);
                    }
                }
                break;
            }
        case xsc_set_focus_incremental:
            {
                if (ivalues[0]) {
                    for (unsigned int which = 0; which < 2; which++) {
                        if (xsc_command_applies_to(which, ivalues[0])) {
                            CommandData.XSC[which].net.set_focus_incremental_value = ivalues[1];
                            xsc_activate_command(which, xC_set_focus_incremental);
                        }
                    }
                } else {
                    blast_err("Commands: must provide non-zero incremental value\n");
                }
                break;
            }
        case xsc_run_autofocus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_run_autofocus);
                    }
                }
                break;
            }
        case xsc_set_autofocus_range:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.autofocus_search_min = ivalues[1];
                        CommandData.XSC[which].net.autofocus_search_max = ivalues[2];
                        CommandData.XSC[which].net.autofocus_search_step = ivalues[3];
                        xsc_activate_command(which, xC_set_autofocus_range);
                    }
                }
                break;
            }
        case xsc_abort_autofocus:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.abort_autofocus_still_use_solution = (ivalues[1] != 0);
                        xsc_activate_command(which, xC_abort_autofocus);
                    }
                }
                break;
            }
        case xsc_autofocus_display_mode:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        if (ivalues[1] >= xC_autofocus_display_mode_auto && ivalues[1]
                                        <= xC_autofocus_display_mode_off) {
                            CommandData.XSC[which].net.autofocus_display_mode = ivalues[1];
                            xsc_activate_command(which, xC_autofocus_display_mode);
                        } else {
                            blast_warn("warning: command xsc_autofocus_display_mode: display mode out of range");
                        }
                    }
                }
                break;
            }
        case xsc_init_aperture:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_init_aperture);
                    }
                }
                break;
            }
        case xsc_get_aperture:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_get_aperture);
                    }
                }
                break;
            }
        case xsc_set_aperture:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.set_aperture_value = ivalues[1];
                        xsc_activate_command(which, xC_set_aperture);
                    }
                }
                break;
            }
        case xsc_stop_aperture:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_stop_aperture);
                    }
                }
                break;
            }
        case xsc_define_aperture:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.define_aperture_value = ivalues[1];
                        xsc_activate_command(which, xC_define_aperture);
                    }
                }
                break;
            }
        case xsc_get_gain:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_get_gain);
                    }
                }
                break;
            }
        case xsc_set_gain:
            {
                for (unsigned int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.set_gain_value = rvalues[1];
                        xsc_activate_command(which, xC_set_gain);
                    }
                }
                break;
            }
        case xsc_fake_sky_brightness:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.brightness.counter++;
                        CommandData.XSC[which].net.brightness.enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.brightness.level_kepsa = rvalues[2];
                        CommandData.XSC[which].net.brightness.gain_db = rvalues[3];
                        CommandData.XSC[which].net.brightness.actual_exposure = rvalues[4];
                        CommandData.XSC[which].net.brightness.simulated_exposure = rvalues[5];
                        xsc_activate_command(which, xC_brightness);
                    }
                }
                break;
            }
        case xsc_solver_general:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.timeout = rvalues[2];
                        xsc_activate_command(which, xC_solver_general);
                    }
                }
                break;
            }
        case xsc_solver_abort:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        xsc_activate_command(which, xC_solver_abort);
                    }
                }
                break;
            }
        case xsc_selective_mask:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.mask.enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.mask.field0 = (unsigned int) ivalues[2];
                        CommandData.XSC[which].net.solver.mask.field1 = (unsigned int) ivalues[3];
                        CommandData.XSC[which].net.solver.mask.field2 = (unsigned int) ivalues[4];
                        xsc_activate_command(which, xC_solver_mask);
                    }
                }
                break;
            }
        case xsc_blob_finding:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.snr_threshold = rvalues[1];
                        CommandData.XSC[which].net.solver.max_num_blobs = ivalues[2];
                        CommandData.XSC[which].net.solver.robust_mode_enabled = (ivalues[3] != 0);
                        if (ivalues[4] >= xC_solver_fitting_method_none && ivalues[4]
                                        <= xC_solver_fitting_method_double_gaussian) {
                            CommandData.XSC[which].net.solver.fitting_method = ivalues[4];
                        } else {
                            blast_warn(
                            "warning: command xsc_blob_finder: fitting_method out of range.  Defaulting to 'none'");
                            CommandData.XSC[which].net.solver.fitting_method = xC_solver_fitting_method_none;
                        }
                        xsc_activate_command(which, xC_solver_blob_finder);
                    }
                }
                break;
            }
        case xsc_blob_cells:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.cell_size = pow(2, ivalues[1]);
                        CommandData.XSC[which].net.solver.max_num_blobs_per_cell = ivalues[2];
                        xsc_activate_command(which, xC_solver_blob_cells);
                    }
                }
                break;
            }
        case xsc_pattern_matching:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.pattern_matcher_enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.display_star_names = (ivalues[2] != 0);
                        CommandData.XSC[which].net.solver.match_tolerance_px = rvalues[3];
                        CommandData.XSC[which].net.solver.iplatescale_min = from_arcsec(rvalues[4]);
                        CommandData.XSC[which].net.solver.iplatescale_max = from_arcsec(rvalues[5]);
                        CommandData.XSC[which].net.solver.platescale_always_fixed = (ivalues[6] != 0);
                        CommandData.XSC[which].net.solver.iplatescale_fixed = from_arcsec(rvalues[7]);
                        xsc_activate_command(which, xC_solver_pattern_matcher);
                    }
                }
                break;
            }
        case xsc_filter_hor_location:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.filters.hor_location_limit_enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.filters.hor_location_limit_radius = from_degrees(rvalues[2]);
                        xsc_activate_command(which, xC_solver_filter_hor_location);
                    }
                }
                break;
            }
        case xsc_filter_hor_roll:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.filters.hor_roll_limit_enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.filters.hor_roll_limit_min = from_degrees(rvalues[2]);
                        CommandData.XSC[which].net.solver.filters.hor_roll_limit_max = from_degrees(rvalues[3]);
                        xsc_activate_command(which, xC_solver_filter_hor_roll);
                    }
                }
                break;
            }
        case xsc_filter_el:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.filters.hor_el_limit_enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.filters.hor_el_limit_min = from_degrees(rvalues[2]);
                        CommandData.XSC[which].net.solver.filters.hor_el_limit_max = from_degrees(rvalues[3]);
                        xsc_activate_command(which, xC_solver_filter_hor_el);
                    }
                }
                break;
            }
        case xsc_filter_eq_location:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.filters.eq_location_limit_enabled = (ivalues[1] != 0);
                        CommandData.XSC[which].net.solver.filters.eq_location_limit_radius = from_degrees(rvalues[2]);
                        xsc_activate_command(which, xC_solver_filter_eq_location);
                    }
                }
                break;
            }
        case xsc_filter_matching:
            {
                for (int which = 0; which < 2; which++) {
                    if (xsc_command_applies_to(which, ivalues[0])) {
                        CommandData.XSC[which].net.solver.filters.matching_pointing_error_threshold = from_arcsec(
                                                                                                    rvalues[1]);
                        CommandData.XSC[which].net.solver.filters.matching_fit_error_threshold_px = rvalues[2];
                        CommandData.XSC[which].net.solver.filters.matching_num_matched = (unsigned int) ivalues[3];
                        xsc_activate_command(which, xC_solver_filter_matching);
                    }
                }
                break;
            }

        /* MISC */
        // XY stage
        case xy_goto:
            CommandData.xystage.x1 = ivalues[0];
            CommandData.xystage.y1 = ivalues[1];
            CommandData.xystage.xvel = ivalues[2];
            CommandData.xystage.yvel = ivalues[3];
            CommandData.xystage.mode = XYSTAGE_GOTO;
            CommandData.xystage.is_new = 1;
            break;
        case xy_jump:
            CommandData.xystage.x1 = ivalues[0];
            CommandData.xystage.y1 = ivalues[1];
            CommandData.xystage.xvel = ivalues[2];
            CommandData.xystage.yvel = ivalues[3];
            CommandData.xystage.mode = XYSTAGE_JUMP;
            CommandData.xystage.is_new = 1;
            break;
        case xy_xscan:
            CommandData.xystage.x1 = ivalues[0];
            CommandData.xystage.x2 = ivalues[1];
            CommandData.xystage.xvel = ivalues[2];
            CommandData.xystage.yvel = 0;
            CommandData.xystage.mode = XYSTAGE_SCAN;
            CommandData.xystage.is_new = 1;
            break;
        case xy_yscan:
            CommandData.xystage.y1 = ivalues[0];
            CommandData.xystage.y2 = ivalues[1];
            CommandData.xystage.yvel = ivalues[2];
            CommandData.xystage.xvel = 0;
            CommandData.xystage.mode = XYSTAGE_SCAN;
            CommandData.xystage.is_new = 1;
            break;
        case xy_raster:
            CommandData.xystage.x1 = ivalues[0];
            CommandData.xystage.x2 = ivalues[1];
            CommandData.xystage.y1 = ivalues[2];
            CommandData.xystage.y2 = ivalues[3];
            CommandData.xystage.xvel = ivalues[4];
            CommandData.xystage.yvel = ivalues[5];
            CommandData.xystage.step = ivalues[6];
            CommandData.xystage.mode = XYSTAGE_RASTER;
            CommandData.xystage.is_new = 1;
            break;
        // Labjacks
        case set_queue_execute:
            set_execute(ivalues[0]);
            break;
        case reconnect_lj:
            set_reconnect(ivalues[0]-1);
            break;
        // BLAST Misc
        case params_test: // Do nothing, with lots of parameters
            blast_info("Integer params 'i': %d 'l' %d", ivalues[0], ivalues[1]);
            blast_info("Float params 'f': %g 'd' %g", rvalues[2], rvalues[3]);
            blast_info("String param 's': %s", svalues[4]);
            CommandData.plover = ivalues[0];
            break;
        case timeout:        // Set timeout
            CommandData.timeout = rvalues[0];
            break;
        case slot_sched:  // change uplinked schedule file
            if (LoadUplinkFile(ivalues[0])) {
                CommandData.uplink_sched = 1;
                CommandData.slot_sched = ivalues[0];
            }
            break;
        case plugh: // A hollow voice says "Plugh".
            CommandData.plover = ivalues[0];
            break;
        default:
            bputs(warning, "Commands: ***Invalid Multi Word Command***\n");
            return;  // invalid command - don't update
  }

  CommandData.command_count++;
  // set high bit to differentiate multi-commands from single
  CommandData.last_command = (uint16_t)command | 0x8000;

#ifndef BOLOTEST
  int i_point = GETREADINDEX(point_index);

  if (!scheduled)
    CommandData.pointing_mode.t = PointingData[i_point].t + CommandData.timeout;
  else
    CommandData.pointing_mode.t = PointingData[i_point].t;
#endif

  WritePrevStatus();
}



/************************************************************/
/*                                                          */
/*  Initialize CommandData: read last valid state: if there */
/*   is no previous state file, set to default              */
/*                                                          */
/************************************************************/
static int one(const struct dirent *unused) {
  return 1;
}

void InitCommandData()
{
		/* --- Start of Convenience hack for linklist --- */
		struct dirent **dir;
		int n = scandir("/data/etc/linklists/", &dir, one, alphasort);
		int num_ll = 0;
    int i = 0;

		// get the list of linklists in the directory
		for (i = 0; i < n; i++) {
			if (num_ll >= 63) {
				printf("Reached maximum linklists for dropdown\n");
				break;
			}
			int len = strlen(dir[i]->d_name);
			if ((len >=3) && strcmp(&dir[i]->d_name[len-3], ".ll") == 0) {
				linklist_nt[num_ll] = calloc(1, 80);
				strncpy(linklist_nt[num_ll], dir[i]->d_name, 64);
				num_ll++;
			}
		}
		// assign the list of linklists to the parameters that have linklist dropdowns
		if (num_ll > 0) {
			for (i = 0; i < N_MCOMMANDS; i++) {
				int p;
				for (p = 0; p < mcommands[i].numparams; p++) {
					if (strcmp(mcommands[i].params[p].name, "Linklist") == 0) {
						mcommands[i].params[p].nt = (const char **) linklist_nt;
					}
				}
			}
			linklist_nt[num_ll] = calloc(1, 80);
			strncpy(linklist_nt[num_ll],  ALL_TELEMETRY_NAME, 79);
      num_ll++;
			linklist_nt[num_ll] = calloc(1, 80);
			strncpy(linklist_nt[num_ll], "no_linklist", 79);
      num_ll++;
		}

		/* --- End of Convenience hack for linklists --- */

    int fp, n_read = 0, junk, extra = 0;
    int is_valid = 0;
    uint32_t prev_crc;

    if ((fp = open(PREV_STATUS_FILE, O_RDONLY)) < 0) {
        berror(err, "Commands: Unable to open prev_status file for reading");
    } else {
        if ((n_read = read(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) berror(
                err, "Commands: prev_status read()");
        if ((extra = read(fp, &junk, sizeof(junk))) < 0) berror(err, "Commands: extra prev_status read()");
        if (close(fp) < 0) berror(err, "Commands: prev_status close()");
    }
    prev_crc = CommandData.checksum;
    CommandData.checksum = 0;
    is_valid = (prev_crc == crc32_le(0, (uint8_t*)&CommandData, sizeof(CommandData)));

    /** this overrides prev_status **/
    CommandData.force_el = 0;

    CommandData.actbus.off = 0;
    CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
    CommandData.actbus.lock_goal = LS_DRIVE_OFF;
    CommandData.actbus.force_repoll = 0;
    CommandData.actbus.cindex = 0;
    CommandData.actbus.caddr[0] = 0;
    CommandData.actbus.caddr[1] = 0;
    CommandData.actbus.caddr[2] = 0;

    CommandData.mag_reset = 0;
    CommandData.inc_reset = 0;

    /* don't use the fast gy offset calculator */
    CommandData.fast_offset_gy = 0;

    /* force autotrim to reset its wait time on restart */
    CommandData.autotrim_xsc0_last_bad = mcp_systime(NULL);
    CommandData.autotrim_xsc1_last_bad = CommandData.autotrim_xsc0_last_bad;

    CommandData.reset_rw = 0;
    CommandData.reset_piv = 0;
    CommandData.reset_elev = 0;
    CommandData.restore_piv = 0;

    CommandData.slot_sched = 0x100;
    CommandData.parts_sched = 0x0;

    CommandData.Labjack_Queue.lj_q_on = 0;
    CommandData.Labjack_Queue.set_q = 1;
    CommandData.Labjack_Queue.which_q[0] = 0;
    CommandData.Labjack_Queue.which_q[1] = 0;
    CommandData.Labjack_Queue.which_q[2] = 0;
    CommandData.Labjack_Queue.which_q[3] = 0;
    CommandData.Labjack_Queue.which_q[4] = 0;
    CommandData.Labjack_Queue.which_q[5] = 0;
    CommandData.Labjack_Queue.which_q[6] = 0;
    CommandData.Labjack_Queue.which_q[7] = 0;
    CommandData.Labjack_Queue.which_q[8] = 0;
    CommandData.Labjack_Queue.which_q[9] = 0;
    CommandData.Labjack_Queue.which_q[10] = 0;
    // CommandData.Relays.update_video = 0;

    /* return if we successfully read the previous status */
    if (n_read != sizeof(struct CommandDataStruct))
        blast_warn("Commands: prev_status: Wanted %i bytes but got %i.\n",
                   (int) sizeof(struct CommandDataStruct), n_read);
    else if (extra > 0)
        bputs(warning, "Commands: prev_status: Extra bytes found.\n");
    else if (!is_valid)
        blast_warn("Invalid Checksum on saved data.  Reverting to defaults!");
    else
        return;

    bputs(warning, "Commands: Regenerating Command Data and prev_status\n");

    /* prev_status overrides this stuff */
    CommandData.command_count = 0;
    CommandData.last_command = 0xffff;

    CommandData.at_float = 0;
    CommandData.timeout = 3600;
    CommandData.slot_sched = 0;

    CommandData.highrate_bw = 6000/8.0; /* Bps */
    CommandData.pilot_bw = 8000000/8.0; /* Bps */
    CommandData.biphase_bw = 1000000/8.0; /* Bps */

    CommandData.highrate_allframe_fraction = 0.1;
    CommandData.pilot_allframe_fraction = 0.1;
    CommandData.biphase_allframe_fraction = 0.1;

    CommandData.biphase_clk_speed = 1000000; /* bps */
    CommandData.biphase_rnrz = false;
    CommandData.highrate_through_tdrss = true;
    copysvalue(CommandData.pilot_linklist_name, ALL_TELEMETRY_NAME);
    copysvalue(CommandData.bi0_linklist_name, "roach_noise_psd.ll");
    copysvalue(CommandData.highrate_linklist_name, "test3.ll");
    copysvalue(CommandData.sbd_linklist_name, "sbd.ll");
    CommandData.vtx_sel[0] = vtx_xsc0;
    CommandData.vtx_sel[1] = vtx_xsc1;
    CommandData.pilot_oth = 0;

    CommandData.slew_veto = VETO_MAX; /* 5 minutes */

    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_DRIFT;
    CommandData.pointing_mode.X = 0;
    CommandData.pointing_mode.Y = 0;
    CommandData.pointing_mode.vaz = 0.0;
    CommandData.pointing_mode.del = 0.0;
    CommandData.pointing_mode.w = 0;
    CommandData.pointing_mode.h = 0;
    CommandData.pointing_mode.t = mcp_systime(NULL) + CommandData.timeout;
    CommandData.pointing_mode.n_dith = 0;
    CommandData.pointing_mode.next_i_dith = 0;
    CommandData.pointing_mode.next_i_hwpr = 0;
    CommandData.pointing_mode.n_dith = 0;
    CommandData.pointing_mode.vel = 0.0;
    CommandData.pointing_mode.daz = 0.0;

    CommandData.az_accel = 0.4;

    CommandData.ele_gain.I = 2.0;
    CommandData.ele_gain.P = 1500;
    CommandData.ele_gain.D = 0;
    CommandData.ele_gain.PT = 40;
    CommandData.ele_gain.DB = 0;
    CommandData.ele_gain.F = 0;

    CommandData.azi_gain.P = 4000;
    CommandData.azi_gain.I = 1.0;
    CommandData.azi_gain.PT = 75;
    CommandData.azi_gain.D = 0;

    CommandData.pivot_gain.SP = 30; // dps
    CommandData.pivot_gain.PV = 2;
    CommandData.pivot_gain.IV = 100;
    CommandData.pivot_gain.PE = 0;
    CommandData.pivot_gain.IE = 100;
    CommandData.pivot_gain.F = 0.0;

    CommandData.ec_devices.reset = 0;
    // By default don't try to fix the Ethercat devices to an operational state.
    CommandData.ec_devices.fix_rw = 0;
    CommandData.ec_devices.fix_el = 0;
    CommandData.ec_devices.fix_piv = 0;
    // By default trigger a write of all of the RW motor set-up parameters on mcp startup
    // including the encoder defaults.  This will trigger a wake-and-wiggle recommutation
    // of the reaction wheel which will make it unable to generate torque for many seconds.
    CommandData.ec_devices.have_commutated_rw = 0;
    CommandData.ec_devices.rw_commutate_next_ec_reset = 0;
    // /TODO: Re-enable El prior to flight
    CommandData.disable_az = 1;
    CommandData.disable_el = 0;

    CommandData.verbose_rw = 0;
    CommandData.verbose_el = 0;
    CommandData.verbose_piv = 0;

    CommandData.use_elmotenc = 1;
    CommandData.use_elclin = 1;
    CommandData.use_pss = 1;
    CommandData.use_dgps = 0;
    CommandData.use_xsc0 = 1;
    CommandData.use_xsc1 = 1;
    CommandData.use_mag1 = 1;
    CommandData.use_mag2 = 1;
    CommandData.lat_range = 1;
    CommandData.sucks = 1;
    CommandData.uplink_sched = 0;

    CommandData.clin_el_trim = 0;
    CommandData.enc_motor_el_trim = 25.16;
    CommandData.null_az_trim = 0;
    CommandData.null_el_trim = 0;
    CommandData.mag_az_trim[0] = 0;
    CommandData.mag_az_trim[1] = 0;
    CommandData.pss_az_trim = 0;
    CommandData.dgps_az_trim = -90.0;

    CommandData.autotrim_enable = 0;
    CommandData.autotrim_thresh = 0.05;
    CommandData.autotrim_rate = 1.0;
    CommandData.autotrim_time = 60;

    CommandData.cal_xmax_mag[0] = 0.1;
    CommandData.cal_ymax_mag[0] = 0.095;
    CommandData.cal_xmin_mag[0] = -0.105;
    CommandData.cal_ymin_mag[0] = -0.1076;
    CommandData.cal_mag_align[0] = 0.0;

    CommandData.cal_xmax_mag[1] = 0.103;
    CommandData.cal_ymax_mag[1] = 0.098;
    CommandData.cal_xmin_mag[1] = -0.108;
    CommandData.cal_ymin_mag[1] = -0.111;
    CommandData.cal_mag_align[1] = 0.0;

    CommandData.cal_az_pss[0] = 0.0;
    CommandData.cal_az_pss[1] = 0.0;
    CommandData.cal_az_pss[2] = 0.0;
    CommandData.cal_az_pss[3] = 0.0;
    CommandData.cal_az_pss[4] = 0.0;
    CommandData.cal_az_pss[5] = 0.0;

    CommandData.cal_d_pss[0] = 0.0;
    CommandData.cal_d_pss[1] = 0.0;
    CommandData.cal_d_pss[2] = 0.0;
    CommandData.cal_d_pss[3] = 0.0;
    CommandData.cal_d_pss[4] = 0.0;
    CommandData.cal_d_pss[5] = 0.0;

    CommandData.cal_imin_pss = 4.5;
	CommandData.pss_noise = 0.2;

    CommandData.az_autogyro = 1;
    CommandData.el_autogyro = 1;
    CommandData.offset_ifel_gy = 0;
    CommandData.offset_ifroll_gy = 0;
    CommandData.offset_ifyaw_gy = 0;
    CommandData.gymask = 0x3f;

    CommandData.actbus.tc_mode = TC_MODE_VETOED;
    CommandData.actbus.tc_step = 100; /* microns */
    CommandData.actbus.tc_wait = 3000; /* = 10 minutes in 5-Hz frames */
    CommandData.actbus.tc_spread = 5; /* centigrade degrees */
    CommandData.actbus.tc_prefp = 1;
    CommandData.actbus.tc_prefs = 1;

    CommandData.actbus.offset[0] = 40000;
    CommandData.actbus.offset[1] = 40000;
    CommandData.actbus.offset[2] = 40000;

    /* The first is due to change in radius of curvature, the second due to
     * displacement of the secondary due to the rigid struts */

    /* Don sez:   50.23 + 9.9 and 13.85 - 2.2 */
    /* Marco sez: 56          and 10          */

    CommandData.actbus.g_primary = 56; /* um/deg */
    CommandData.actbus.g_secondary = 10; /* um/deg */
    CommandData.actbus.focus = 0;
    CommandData.actbus.sf_time = 0;
    CommandData.actbus.sf_offset = 6667;

    CommandData.actbus.act_vel = 200;
    CommandData.actbus.act_acc = 1000;
    CommandData.actbus.act_move_i = 75;
    CommandData.actbus.act_hold_i = 10;
    CommandData.actbus.act_tol = 5;

    CommandData.actbus.lock_vel = 110000;
    CommandData.actbus.lock_acc = 100;
    CommandData.actbus.lock_move_i = 50;
    CommandData.actbus.lock_hold_i = 0;

    CommandData.balance.vel = 6400;
    CommandData.balance.acc = 1000;
    CommandData.balance.move_i = 20;
    CommandData.balance.hold_i = 0;

    CommandData.balance.i_el_on_bal = 3.5;
    CommandData.balance.i_el_off_bal = 1.5;
    CommandData.balance.mode = bal_rest;

    CommandData.actbus.shutter_step = 4224;
    CommandData.actbus.shutter_step_slow = 300;
    CommandData.actbus.shutter_move_i = 40;
    CommandData.actbus.shutter_hold_i = 40;
    CommandData.actbus.shutter_vel = 3000;
    CommandData.actbus.shutter_acc = 1;

    CommandData.pin_is_in = 1;

    // XY STAGE
    CommandData.xystage.x1 = 0;
    CommandData.xystage.y1 = 0;
    CommandData.xystage.x2 = 0;
    CommandData.xystage.y2 = 0;
    CommandData.xystage.step = 0;
    CommandData.xystage.xvel = 0;
    CommandData.xystage.yvel = 0;
    CommandData.xystage.is_new = 1;
    CommandData.xystage.mode = XYSTAGE_GOTO;
    CommandData.xystage.force_repoll = 0;

    CommandData.ISCControl[0].max_age = 200; /* 2000 ms*/

    CommandData.ISCControl[0].autofocus = 0;
    CommandData.ISCControl[0].save_period = 12000; /* 120 sec */
    CommandData.ISCControl[0].pulse_width = 18; /* 180.00 msec */
    CommandData.ISCControl[0].fast_pulse_width = 8; /* 80.00 msec */

    CommandData.ISCControl[1].max_age = 200; /* 2000 ms*/

    CommandData.ISCControl[1].autofocus = 0;
    CommandData.ISCControl[1].save_period = 12000; /* 120 sec */
    CommandData.ISCControl[1].pulse_width = 18; /* 180.00 msec */
    CommandData.ISCControl[1].fast_pulse_width = 8; /* 80.00 msec */

    for (int which = 0; which < 2; which++) {
        CommandData.XSC[which].is_new_window_period_cs = 1500;

        // CommandData.XSC[which].heaters.mode = xsc_heater_auto;
        CommandData.XSC[which].heaters.mode = xsc_heater_off;
        CommandData.XSC[which].heaters.setpoint = 10.0;

        CommandData.XSC[which].trigger.exposure_time_cs = 12;
        CommandData.XSC[which].trigger.grace_period_cs = 4500;
        CommandData.XSC[which].trigger.post_trigger_counter_mcp_share_delay_cs = 200;

        CommandData.XSC[which].trigger.num_triggers = 1;
        CommandData.XSC[which].trigger.multi_trigger_time_between_triggers_cs = 18;

        CommandData.XSC[which].trigger.threshold.enabled = true;
        CommandData.XSC[which].trigger.threshold.blob_streaking_px = 2.0;

        CommandData.XSC[which].trigger.scan_force_trigger_enabled = true;
        CommandData.XSC[which].el_trim = 0.0;
        CommandData.XSC[which].cross_el_trim = 0.0;

        xsc_clear_client_data(&CommandData.XSC[which].net);
    }

    CommandData.temp1 = 0;
    CommandData.temp2 = 0;
    CommandData.temp3 = 0;
    CommandData.df = 0;

    CommandData.lat = -77.86;  // McMurdo Building 096
    CommandData.lon = -167.04; // Willy Field Dec 2010

    WritePrevStatus();
}
