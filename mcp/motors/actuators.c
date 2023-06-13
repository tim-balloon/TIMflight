/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2006 University of Toronto
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <limits.h>

#include "therm_heater.h"
#include "ezstep.h"
#include "mcp.h"
#include "command_struct.h"
#include "pointing_struct.h"
#include "balance.h"
#include "tx.h"
#include "actuators.h"
#include "ec_motors.h"


void nameThread(const char*); // mcp.c
double LockPosition(double elevation); // commands.c
extern int16_t InCharge; // tx.c

// temp buffer for querying channels for various actuators
static char name_buffer[100];

// ============================================================================
// EZStepper Bus Parameters
// ============================================================================
// Index for each stepper for structures, name, id
#define LOCKNUM 4
#define SHUTTERNUM 6
// actuator bus setup paramters
#define ACT_BUS "/dev/ttyACT"
#define NACT 10
#define POLL_TIMEOUT 5 // 1s @ 5Hz
#define	MAX_SERIAL_ERRORS 20 // after this many, repoll bus
// FREE_N are addresses on the EZ bus that are open for use
static const char *name[NACT] = {
    "Actuator #0",
    "Actuator #1",
    "Actuator #2",
    "Balance Motor",
    "Lock Motor",
    "FREE_1",
    "Shutter",
    "FREE_2",
    "FREE_3",
    "FREE_4"
};
static const int id[NACT] = {
    EZ_WHO_S1,
    EZ_WHO_S2,
    EZ_WHO_S3,
    EZ_WHO_S4,
    EZ_WHO_S5,
    EZ_WHO_S6,
    EZ_WHO_S7,
    EZ_WHO_S8,
    EZ_WHO_S9,
    EZ_WHO_S10
};
static char preamble_buf[EZ_BUS_BUF_LEN];
static struct ezbus bus;
static int poll_timeout = POLL_TIMEOUT; /* track time since last repoll */
static int actbus_reset = 1; /* 1 means actbus is on */
static unsigned int actuators_init = 0;	/* bitfield for when actuators usable */
static unsigned int valve_check = 0;

// ============================================================================
// Elevation lock pin motor parameters
// ============================================================================
#define LOCK_MOTOR_DATA_TIMER 100 // 1 second
#define DRIVE_TIMEOUT 3000 // 30 seconds
#define LOCK_MIN_POT 3000 // actual min stop: ~2500 (fully extended)
#define LOCK_MAX_POT 15000 // max stop at saturation: 16368 (fully retracted)
#define LOCK_POT_RANGE 500

#define SEND_SLEEP 100000 // 100 milliseconds
#define WAIT_SLEEP 100000 // 100 millisecond
#define LA_EXIT    0
#define LA_STOP    1
#define LA_WAIT    2
#define LA_EXTEND  3
#define LA_RETRACT 4

int lock_timeout = -1;

static struct lock_struct {
    int pos; // raw step count
    uint16_t adc[4]; // ADC readout (including pot)
    unsigned int state;
} lock_data = { .state = LS_DRIVE_UNK };

// ============================================================================
// Shutter motor parameters
// ============================================================================
#define SHUTTER_MOTOR_DATA_TIMER 100 // 1 second
#define SHUTTER_TIMEOUT 3000 // 30 seconds
#define SHUTTER_CLOSE_TIMEOUT 20000000
#define SHUTTER_CLOSE_SLOW_TIMEOUT 50000000
// /7?4 returns 15 when shutter is closed and returns 11 when shutter is not
// closed
#define SHUTTER_CLOSED_BIT 0x04
#define SHUTTER_OPEN_BIT 0x08
// #define  SHUTTER_OPEN 7
#define SHUTTER_SLEEP 100000 // 100 milliseconds
// #define  SHUTTER_SLEEP 50000
#define SHUTTER_IS_CLOSED 2
#define SHUTTER_IS_UNK 1
#define SHUTTER_IS_OPEN 0
// If polarity on one pair of windings is reversed then motor will turn in
// opposite direction
#define SHUTTER_POLARITY 0

#define SHUTTER_EXIT 0
#define SHUTTER_DO_OFF 1
#define SHUTTER_DO_CLOSE 2
#define SHUTTER_DO_CLOSE_SLOW 3
#define SHUTTER_DO_OPEN 4
#define SHUTTER_DO_OPEN_CLOSE 6
#define SHUTTER_DO_RESET 7
#define SHUTTER_DO_INIT 8
#define SHUTTER_DO_NOP 9
#define SHUTTER_DO_KEEPCLOSED 10
#define SHUTTER_DO_KEEPOPEN 11

int shutter_timeout = -1;

static struct shutter_struct {
    int32_t pos;
    // lims
    // shutter not out (presumed in) = 0
    // shutter out (limit switch depressed) = 1
    // shutter in = 1 (opto-switch blocked)
    // shutter not in = 0 (opto-switch not blocked)
    int lims;
    unsigned int state;
    int move_commanded;
} shutter_data = { .state = SHUTTER_UNK };

// ============================================================================
// Secondary mirror actuator parameters
// ============================================================================
#define DEFAULT_DR 32768 // value to use if reading file fails
#define MIN_ENC 1000 // minimum acceptable encoder value, load dr below
// wait between trims, and after moves
#define ACTBUS_TRIM_WAIT 3*25 // thrice LVDT_FILT_LEN

static struct act_struct {
    int pos; // raw step count
    int enc; // encoder reading
    int dr; // dead reckoning (best-guess absolute position)
} act_data[3];

static unsigned int actbus_flags = 0;
static int act_trim_wait = 0;
static int act_trim_flag_wait = 0;

#define ACT_FL_TRIM_WAIT 0x001
#define ACT_FL_TRIMMED 0x002
#define ACT_FL_BUSY0 0x004
#define ACT_FL_BUSY1 0x008
#define ACT_FL_BUSY2 0x010
#define ACT_FL_BUSY(i) (ACT_FL_BUSY0 << i)
#define ACT_FL_BUSY_MASK (ACT_FL_BUSY0 | ACT_FL_BUSY1 | ACT_FL_BUSY2)
#define ACT_FL_BAD_MOVE 0x020

// Secondary mirror focus things
static double t_primary = -1;
static double t_secondary = -1;
static double focus = -1; // set in ab thread, read in fc thread
static double correction = 0; // set in fc thread, read in ab thread

// ============================================================================
// Actuator Logic: servo focus based on thermal model/commands
// ============================================================================
/**
 * @brief Return the appropriate EZStepper bus address to send commands to.
 * Used by balance.c, hwpr.c, cryovalves.c
 * @param ind
 */
int GetActAddr(int ind)
{
    return id[ind];
}

/**
 * @brief write dead reckoning estimate of actuator positions to disk
 */
static void WriteDR(void)
{
    int fp, n, i;

    // write the default file
    fp = open("/data/etc/blast/act.dr", O_WRONLY | O_CREAT | O_TRUNC, 00666);
    if (fp < 0) {
        berror(err, "act.dr open()");
        return;
    }
    for (i = 0; i < 3; i++) {
        if ((n = write(fp, &act_data[i].dr, sizeof(int))) < 0) {
            berror(err, "act.dr write()");
            return;
        }
    }
    if ((n = close(fp)) < 0) {
        berror(err, "act.dr close()");
        return;
    }
}

/**
 * @brief on initialization, or bad enc detection, read DR from disk
 */
void ReadDR(void)
{
    int fp;
    int i;
    int n_read = 0;
    int read_fail = 0;

    if ((fp = open("/data/etc/blast/act.dr", O_RDONLY)) < 0) {
        read_fail = 1;
        berror(err, "Unable to open act.dr file for reading");
    } else {
        for (i = 0; i < 3; i++) {
            if ((n_read = read(fp, &act_data[i].dr, sizeof(int))) < 0) {
                // read failed
                read_fail = 1;
                berror(err, "act.dr read()");
                break;
            } else if (n_read != sizeof(int)) {
                // short read
                read_fail = 1;
                blast_err("act.dr read(): wrong number of bytes");
                break;
            }
        }
        if (close(fp) < 0) {
            berror(err, "act.dr close()");
        }
    }

    if (read_fail) {
        blast_info("Read of act.dr failed. Using default value %d", DEFAULT_DR);
        for (i = 0; i < 3; i++) {
            act_data[i].dr = DEFAULT_DR;
        }
    }
}

/**
 * @brief Helper to make string for setting closed loop stepper correction
 * tolerance
 */
static inline char* actPreamble(uint16_t tol)
{
    snprintf(preamble_buf, sizeof(preamble_buf), ACT_PREAMBLE, tol);
    return preamble_buf;
}

/**
 * @brief Helper to read and put actuator position and encoder data in act_data
 */
static void ReadActuator(int num)
{
    if (!EZBus_IsUsable(&bus, id[num])) {
        return;
    }
    EZBus_ReadInt(&bus, id[num], "?0", &act_data[num].pos);
    EZBus_ReadInt(&bus, id[num], "?8", &act_data[num].enc);
}

/**
 * @brief Set both dead reckoning and encoder to trim value, dump dead
 * reckoning of actuator position to disk
 */
void actEncTrim(int *trim)
{
    int i;
    char buffer[EZ_BUS_BUF_LEN];

    blast_info("trim enc and dr to (%d, %d, %d)", trim[0], trim[1], trim[2]);

    for (i = 0; i < 3; i++) {
        /* set the dr */
        act_data[i].dr = trim[i];
        /* Set the encoder */
        EZBus_Comm(&bus, id[i], EZBus_StrComm(&bus, id[i], sizeof(buffer), buffer, "z%iR", act_data[i].dr));
    }

    WriteDR();
}

/**
 * @brief Helper to check encoders for sane values, and update if not good
 */
void CheckEncoders(void)
{
    int i;
    int trim[3];
    int do_trim = 0;

    // check busy state of actuators, update flag
    for (i = 0; i < 3; i++) {
        if (EZBus_IsBusy(&bus, id[i])) {
            act_trim_wait = ACTBUS_TRIM_WAIT;
            actbus_flags |= ACT_FL_BUSY(i);
            actbus_flags |= ACT_FL_TRIM_WAIT;
        } else {
            actbus_flags &= ~ACT_FL_BUSY(i);
        }
    }

    // read encoders, and check if values need updating
    for (i = 0; i < 3; ++i) {
        ReadActuator(i);
        trim[i] = act_data[i].dr;
    }

    // if busy, or waiting, do not trim
    if (actbus_flags & (ACT_FL_BUSY_MASK | ACT_FL_TRIM_WAIT)) {
        return;
    } else if (do_trim) {
        actEncTrim(trim);
        act_trim_flag_wait = act_trim_wait = ACTBUS_TRIM_WAIT;
        actbus_flags |= ACT_FL_TRIMMED | ACT_FL_TRIM_WAIT;
    }
}

/**
 * @brief before moving, update dead reckoning to new goal
 */
static void UpdateDR(int* goal)
{
    int i;
    for (i = 0; i < 3; i++) {
        act_data[i].dr = goal[i];
    }
    WriteDR();
}

/**
 * @brief Actually send move commands to secondary mirror actuators
 * @param goal Pointer to array of desired positions for secondary actuators
 */
static void ServoActuators(int* goal)
{
    // NB has less checks than servoing used to, but I don't see how they were useful
    int i;
    char buf[EZ_BUS_BUF_LEN];

    if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC) {
        return;
    }

    for (i = 0; i < 3; ++i) {
        EZBus_Take(&bus, id[i]);
    }
    UpdateDR(goal);

    // stop any current action
    for (i = 0; i < 3; ++i) {
        EZBus_Stop(&bus, id[i]);
    }
    for (i = 0; i < 3; ++i) {
        // send command to each actuator, but don't run yet
        EZBus_Comm(&bus, id[i], EZBus_StrComm(&bus, id[i], sizeof(buf), buf, "A%d", goal[i]));
    }
    for (i = 0; i < 3; ++i) {
        // kick off all move commands as close together as possible
        EZBus_Comm(&bus, id[i], "R");
    }
    for (i = 0; i < 3; ++i) {
        EZBus_Release(&bus, id[i]);
    }
}

/**
 * @brief Compute desired position of secondary mirror actuators
 */
static void DeltaActuators(void)
{
    int i;
    int goal[3];

    for (i = 0; i < 3; ++i) {
        goal[i] = CommandData.actbus.delta[i] + act_data[i].enc;
    }
    ServoActuators(goal);
}

/**
 * @brief Computer thermal compensation offset to secondary focus
 * @return (int) Thermal compensation focus mode
 */
static int ThermalCompensation(void)
{
    // If panicking, continue to do so
    if (CommandData.actbus.focus_mode == ACTBUS_FM_PANIC) {
        return ACTBUS_FM_PANIC;
    }
    // Do nothing if vetoed or autovetoed
    if (CommandData.actbus.tc_mode != TC_MODE_ENABLED) {
        return ACTBUS_FM_SLEEP;
    }
    // Do nothing if we haven't timed out
    if (CommandData.actbus.sf_time < CommandData.actbus.tc_wait) {
        return ACTBUS_FM_SLEEP;
    }
    // Do nothing if the offset is below the threshold
    if (correction < CommandData.actbus.tc_step &&
        -correction < CommandData.actbus.tc_step) {
        return ACTBUS_FM_SLEEP;
    }
    // Do something!
    CommandData.actbus.focus = focus - correction;
    CommandData.actbus.sf_time = 0;

    return ACTBUS_FM_THERMO;
}

/**
 * @brief State machine for secondary mirror actuator focus mode
 */
static void DoActuators(void)
{
    int delta;
    int i;

    // Send secondary mirror EZStepper settings
    for (i = 0; i < 3; ++i) {
        EZBus_SetVel(&bus, id[i], CommandData.actbus.act_vel);
        EZBus_SetAccel(&bus, id[i], CommandData.actbus.act_acc);
        EZBus_SetIMove(&bus, id[i], CommandData.actbus.act_move_i);
        EZBus_SetIHold(&bus, id[i], CommandData.actbus.act_hold_i);
        EZBus_SetPreamble(&bus, id[i], actPreamble(CommandData.actbus.act_tol));
    }

    switch (CommandData.actbus.focus_mode) {
        case ACTBUS_FM_PANIC:
            blast_info("M2 Actuators in Panic");
            for (i = 0; i < 3; ++i) {
                EZBus_Stop(&bus, id[i]); // terminate all running commands
                EZBus_Comm(&bus, id[i], "n0R");	// also stop fine correction
            }
            CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
            break;
        case ACTBUS_FM_DELTA:
            DeltaActuators();
            break;
        case ACTBUS_FM_DELFOC:
            CommandData.actbus.focus += focus;
            // fallthough
        case ACTBUS_FM_THERMO:
        case ACTBUS_FM_FOCUS:
            blast_info("changing focus %g to %d", focus, CommandData.actbus.focus);
            delta = CommandData.actbus.focus - focus;
            CommandData.actbus.goal[0] = act_data[0].enc + delta;
            CommandData.actbus.goal[1] = act_data[1].enc + delta;
            CommandData.actbus.goal[2] = act_data[2].enc + delta;
            // fallthrough
        case ACTBUS_FM_SERVO:
            ServoActuators(CommandData.actbus.goal);
            break;
        case ACTBUS_FM_TRIM:
            actEncTrim(CommandData.actbus.trim);
            break;
        case ACTBUS_FM_SLEEP:
            break;
        default:
            blast_err("Unknown Focus Mode (%i), sleeping", CommandData.actbus.focus_mode);
            CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP;
    }

    CommandData.actbus.focus_mode = ThermalCompensation();

    CheckEncoders();

    focus = (act_data[0].enc + act_data[1].enc + act_data[2].enc) / 3.0;
}

/**
 * @brief adjust focus offset so that new gains don't change focus thermal
 * correction
 * @param new_gp primary gain
 * @param new_gs secondary gain
 */
void RecalcOffset(double new_gp, double new_gs)
{
    if (t_primary < 0 || t_secondary < 0) {
        return;
    }

    CommandData.actbus.sf_offset += (
        (new_gp - CommandData.actbus.g_primary) * (t_primary - T_PRIMARY_FOCUS) -
        (new_gs - CommandData.actbus.g_secondary) * (t_secondary - T_SECONDARY_FOCUS)) / ACTENC_TO_UM;
}

/**
 * @brief Initialize secondary mirror actuators
 * @param thebus pointer to EZStepper Bus
 * @param who address of actuator to initialize
 * @return (int) 1 if successful, 0 if failed
 */
static int InitializeActuator(struct ezbus* thebus, char who)
{
    char buffer[EZ_BUS_BUF_LEN];
    int i;

    for (i = 0; i < 3; i++) {
        if (id[i] == who) { // only operate on actautors
            blast_info("Inside InitializeActuator, initializing %s...", name[i]);
            ReadDR(); // inefficient,
            // Set the encoder
            if (EZBus_Comm(thebus, who,
                    EZBus_StrComm(thebus, who, sizeof(buffer), buffer, "z%iR", act_data[i].dr)) == EZ_ERR_OK) {
                return 1;
            } else {
                blast_warn("Initialising %s failed...", name[i]);
                return 0;
            }
        }
    }
    return 1;
}


// ============================================================================
// Do shutter logic: check status, determine where shutter is
// ============================================================================
/**
 * @brief Set shutter stepper hold current to 0
 */
static void TurnOffShutter(void)
{
    // Set hold current to zero
    blast_info("TurnOffShutter... Setting hold current to 0");
    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0R") != EZ_ERR_OK)
    if (EZBus_SetIHold(&bus, id[SHUTTERNUM], 0) != EZ_ERR_OK) {
        bputs(warning, "TurnOffShutter: Error turning off shutter");
    } else {
        CommandData.actbus.shutter_hold_i = 0;
    }
}

/**
 * @brief Set move and hold currents, velocity, and accel for shutter stepper
 */
static void InitializeShutter(void)
{
    // Set move current and speed
    // blast_info("InitializeShutter:...");
    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64m100l100h50R") != EZ_ERR_OK) // removing because has old settings
    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64m100l100v10h50R") != EZ_ERR_OK) // removing because has old settings
    // bputs(info, "InitializeShutter: Error initializing shutter");
    // CommandData.actbus.shutter_step = 4224;
    // CommandData.actbus.shutter_step_slow = 300;
    // CommandData.actbus.shutter_move_i = 100;
    // CommandData.actbus.shutter_move_i = 50;
    // CommandData.actbus.shutter_vel = 20;
    // CommandData.actbus.shutter_acc = 1;
    // Set microstepping on j64N1
    // Wait 2 seconds M2000
    // Set hold current to 50 h50
    // Set position to 5000 z5000
    // Move to activate limit switch D424
    // Set position to 0 z0
    EZBus_SetIMove(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_move_i);
    EZBus_SetIHold(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_hold_i);
    EZBus_SetVel(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_vel);
    EZBus_SetAccel(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_acc);
}

/**
 * @brief If shutter failed to close completely, this causes shutter to close by
 * setting hold current to 0 and then moving to limit switch.
 */
static void ResetShutter()
{
    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0M2000h50z5000D424z0R") != EZ_ERR_OK)
    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0M2000h50z5000P424z0R") != EZ_ERR_OK)
    //   bputs(info, "ResetShutter: Error resetting shutter");
    blast_info("Resetting shutter -- set hold current to zero, wait, and re-close");
    EZBus_SetIHold(&bus, id[SHUTTERNUM], 0);
    usleep(1000000); // wait for shutter to fall open
    EZBus_SetIHold(&bus, id[SHUTTERNUM], CommandData.actbus.shutter_hold_i);
    if (EZBus_MoveComm(&bus, id[SHUTTERNUM], "D0") != EZ_ERR_OK) {
        bputs(info, "ResetShutter: Error moving after reset");
    }
}

/**
 * @brief Command an endless negative move to close the shutter.
 * @param cancel If nonzero, veto keeping shutter closed
 */
static void KeepClosedShutter(int* cancel)
{
    if (*cancel == 0) {
        if (((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) && (shutter_data.move_commanded == 0)) {
            blast_info("KeepClosed mode sees shutter not closed! Commanding close...");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]);

            if (EZBus_MoveComm(&bus, id[SHUTTERNUM], "D0") != EZ_ERR_OK) {
                blast_info("KeepClosedShutter: error commanding close move");
            }
            EZBus_Release(&bus, id[SHUTTERNUM]);
            shutter_data.move_commanded = 1;
        } else {
            if ((shutter_data.lims & SHUTTER_CLOSED_BIT) == SHUTTER_CLOSED_BIT) {
                shutter_data.move_commanded = 0;
            }
        }
    }
}

/**
 * @brief Command an endless positive move to keep the shutter open.
 * @param cancel If nonzero, veto keeping shutter open
 */
static void KeepOpenShutter(int* cancel)
{
    if (*cancel == 0) {
        if (((shutter_data.lims & SHUTTER_OPEN_BIT) != SHUTTER_OPEN_BIT) && (shutter_data.move_commanded == 0)) {
            blast_info("KeepOpen mode sees shutter not open! Commanding open...");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]);

            if (EZBus_MoveComm(&bus, id[SHUTTERNUM], "P0") != EZ_ERR_OK) {
                blast_info("KeepOpenShutter: error commanding open move");
            }
            EZBus_Release(&bus, id[SHUTTERNUM]);
            shutter_data.move_commanded = 1;
        } else {
            if ((shutter_data.lims & SHUTTER_OPEN_BIT) == SHUTTER_OPEN_BIT) {
                shutter_data.move_commanded = 0;
            }
        }
    }
}

/**
 * @brief Jog shutter positive 424, then negative by a set amount, shutter_step
 */
static void OpenCloseShutter(void)
{
    char cmd[80];
    bputs(info, "OpenCloseShutter...");
    // if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.lims) != EZ_ERR_OK)
    // bputs(info, "OpenCloseShutter: Error polling opto switch");
    // usleep(SHUTTER_SLEEP);
    EZBus_Stop(&bus, id[SHUTTERNUM]);
    if ((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
        bputs(info, "OpenCloseShutter: doing action");
        // if (EZBus_Comm(&bus, id[SHUTTERNUM], "h0z5000h50V10000D424P4224R") != EZ_ERR_OK)
        snprintf(cmd, sizeof(cmd), "h0z5000h50V10000P424D%dR", CommandData.actbus.shutter_step);
        if (EZBus_Comm(&bus, id[SHUTTERNUM], cmd) != EZ_ERR_OK) {
            bputs(warning, "OpenCloseShutter: EZ Bus error");
            usleep(20*SHUTTER_SLEEP);
        }
    } else {
        bputs(info, "OpenCloseShutter: Shutter is already closed");
    }
}

/**
 * @brief Command an endless negative move to close the shutter
 */
static void CloseShutter(void)
{
    if ((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
        blast_info("CloseShutter: closing shutter...");
        if (EZBus_MoveComm(&bus, id[SHUTTERNUM], "D0") != EZ_ERR_OK) {
            blast_info("CloseShutter: EZ Bus error");
        }
    } else {  // Shutter is closed according to opto switch
        shutter_data.state = SHUTTER_CLOSED;
    }
}

/**
 * @brief This code does the old style closing of the shutter: close shutter a
 * little, check the opto, close the shutter a little, check the opto... until
 * shutter is closed
 */
static void CloseSlowShutter(void)
{
    int shutter_timeout = 0;
    char cmd[80];

    // if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.lims) != EZ_ERR_OK) {
    //     bputs(info, "CloseShutter: 1. Error polling opto switch");
    // } else {
    //     ;
    // }

    // blast_info("%d %d %d", shutter_data.lims, shutter_data.lims & SHUTTER_CLOSED_BIT,
    //        SHUTTER_CLOSED_BIT);

    if ((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
        bputs(info, "CloseSlowShutter: Closing shutter...");
        while (((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) &
                (shutter_timeout < SHUTTER_CLOSE_TIMEOUT)) {
            if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
                // if (EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", &shutter_data.lims) != EZ_ERR_OK)
                //  bputs(warning, "CloseShutter: 2. Error polling opto switch");
                usleep(SHUTTER_SLEEP);
                if ((shutter_data.lims & SHUTTER_CLOSED_BIT) != SHUTTER_CLOSED_BIT) {
                    // if (EZBus_Comm(&bus, id[SHUTTERNUM], "j64z0h50V1000P300R") != EZ_ERR_OK)
                    snprintf(cmd, sizeof(cmd), "j64z5000h50V1000D%dR", CommandData.actbus.shutter_step_slow);
                    if (EZBus_Comm(&bus, id[SHUTTERNUM], cmd) != EZ_ERR_OK) {
                        bputs(warning, "CloseShutter: EZ Bus error");
                    }
                }
            }
            shutter_timeout += SHUTTER_SLEEP;
        }
    } else {  // Shutter is closed according to opto switch
        shutter_data.state = SHUTTER_CLOSED;
        // bputs(info, "CloseShutter: shutter is closed");
    }

    if (shutter_timeout >= SHUTTER_CLOSE_SLOW_TIMEOUT) {
        bputs(warning, "CloseSlowShutter: Closing shutter timed out");
    }
}

/**
 * @brief Command an endless positive move to open the shutter
 */
static void OpenShutter(void)
{
    if ((shutter_data.lims & SHUTTER_OPEN_BIT) != SHUTTER_OPEN_BIT) {
        blast_info("OpenShutter: opening shutter...");
        if (EZBus_MoveComm(&bus, id[SHUTTERNUM], "P0") != EZ_ERR_OK) {
            blast_info("OpenShutter: EZ Bus error");
        }
    } else {  // Shutter is open according to opto switch
        shutter_data.state = SHUTTER_OPEN;
    }
}

/**
 * @brief Ask the shutter EZStepper about the commanded position and the limit
 * switches.
 * This position is only where the step controller thinks the shutter is. There
 * is no direct feedback on the shutter position other than the limit switch.
 * @param[out] lims Contains status of all digital outputs
 * @param[out] pos Contains commanded motor position
 * @return -1 if either read failed, 0 if both succeeded.
 */
static int GetShutterData(int* lims, int* pos)
{
    int retval = 0;
    int errorCodeLim = EZ_ERR_OK;
    int errorCodePos = EZ_ERR_OK;

    if ((errorCodeLim = EZBus_ReadInt(&bus, id[SHUTTERNUM], "?4", lims) != EZ_ERR_OK)) {
        blast_info("GetShutterData: EZBus_ReadInt error -- lims, error code = %d", errorCodeLim);
    }
    if ((errorCodePos = EZBus_ReadInt(&bus, id[SHUTTERNUM], "?0", pos) != EZ_ERR_OK)) {
        blast_info("GetShutterData: EZBus_ReadInt error -- pos, error code = %d", errorCodePos);
    }

    if ((errorCodeLim != EZ_ERR_OK) || (errorCodePos != EZ_ERR_OK)) {
        retval = -1;
    }
    return retval;
}

/**
 * @brief Switch cases to call shutter move functions based on CommandData
 */
static void DoShutter(void)
{
    int action = SHUTTER_EXIT;
    int cancel;

    if (shutter_data.state == SHUTTER_UNK) {
        // bputs(info, "Initializing shutter...");
        EZBus_Take(&bus, id[SHUTTERNUM]);
        EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
        // InitializeShutter();
        EZBus_Release(&bus, id[SHUTTERNUM]);
        shutter_data.state = SHUTTER_OPEN;
        CommandData.actbus.shutter_goal = SHUTTER_NOP;
    }

    EZBus_Take(&bus, id[SHUTTERNUM]);
    InitializeShutter();
    if (GetShutterData(&shutter_data.lims, &shutter_data.pos)) {
        blast_info("DoShutter: One or more queries of the shutter stepper failed!");
    }
    EZBus_Release(&bus, id[SHUTTERNUM]);

    switch (CommandData.actbus.shutter_goal) {
        case SHUTTER_OPEN:
            action = SHUTTER_DO_OPEN;
            shutter_data.state = SHUTTER_OPEN;
            CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_CLOSED:
            action = SHUTTER_DO_CLOSE;
            // shutter_data.state = SHUTTER_CLOSED;
            // CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_CLOSED_SLOW:
            action = SHUTTER_DO_CLOSE_SLOW;
            break;
        case SHUTTER_CLOSED2:
            action = SHUTTER_DO_OPEN_CLOSE;
            // CommandData.actbus.shutter_goal = SHUTTER_CLOSED;
            CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_INIT:
            action = SHUTTER_DO_INIT;
            shutter_data.state = SHUTTER_OPEN;
            CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_RESET:
            action = SHUTTER_DO_RESET;
            shutter_data.state = SHUTTER_OPEN;
            CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_OFF:
            action = SHUTTER_DO_OFF;
            shutter_data.state = SHUTTER_OPEN;
            CommandData.actbus.shutter_goal = SHUTTER_NOP;
            break;
        case SHUTTER_KEEPCLOSED:
            action = SHUTTER_DO_KEEPCLOSED;
            shutter_data.state = SHUTTER_CLOSED;
            CommandData.actbus.shutter_goal = SHUTTER_KEEPCLOSED; // don't clear goal in CommandData
            break;
        case SHUTTER_KEEPOPEN:
            action = SHUTTER_DO_KEEPOPEN;
            shutter_data.state = SHUTTER_OPEN;
            CommandData.actbus.shutter_goal = SHUTTER_KEEPOPEN;
            break;
        // case SHUTTER_NOP:
            // action = SHUTTER_DO_NOP;
            // break;
    }

    // Figure out what to do...
    if (action != SHUTTER_DO_KEEPCLOSED && action != SHUTTER_DO_KEEPOPEN) {
        cancel = 1;
        shutter_data.move_commanded = 0;
    } else {
        cancel = 0;
    }
    switch (action) {
        case SHUTTER_DO_OFF:
            bputs(warning, "Turning off shutter.  Shutter will open.");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
            TurnOffShutter();
            EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_CLOSE:
            // shutter_timeout = DRIVE_TIMEOUT;
            // bputs(warning, "1. Closing shutter.");
            if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
                EZBus_Take(&bus, id[SHUTTERNUM]);
                // EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
                CloseShutter();
                EZBus_Release(&bus, id[SHUTTERNUM]);
            }
            break;
        case SHUTTER_DO_CLOSE_SLOW:
            if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
                EZBus_Take(&bus, id[SHUTTERNUM]);
                // EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
                CloseSlowShutter();
                EZBus_Release(&bus, id[SHUTTERNUM]);
            }
            break;
        case SHUTTER_DO_OPEN_CLOSE:
        // if (!EZBus_IsBusy(&bus, id[SHUTTERNUM])) {
            EZBus_Take(&bus, id[SHUTTERNUM]);
            // EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
            OpenCloseShutter();
            EZBus_Release(&bus, id[SHUTTERNUM]);
        // }
        // else {
            // bputs(warning, "EZBus busy --- not calling OpenCloseShutter");
        // }
            break;
        case SHUTTER_DO_OPEN:
            // shutter_timeout = DRIVE_TIMEOUT;
            bputs(warning, "Opening shutter.");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]); // stop current action first
            OpenShutter();
            EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_INIT:
            // shutter_timeout = DRIVE_TIMEOUT;
            bputs(warning, "Intializing shutter.  Shutter will open.");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]);  // stop current action first
            InitializeShutter();
            EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_RESET:
            // shutter_timeout = DRIVE_TIMEOUT;
            bputs(warning, "Resetting shutter.  Shutter will open.");
            EZBus_Take(&bus, id[SHUTTERNUM]);
            EZBus_Stop(&bus, id[SHUTTERNUM]);  // stop current action first
            ResetShutter();
            EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_KEEPCLOSED:
            // EZBus_Take(&bus, id[SHUTTERNUM]);
            cancel = 0;
            KeepClosedShutter(&cancel);
            // EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_KEEPOPEN:
            // EZBus_Take(&bus, id[SHUTTERNUM]);
            cancel = 0;
            KeepOpenShutter(&cancel);
            // EZBus_Release(&bus, id[SHUTTERNUM]);
            break;
        case SHUTTER_DO_NOP:
            break;
    }
    action = SHUTTER_EXIT;
}

// ============================================================================
// Do elevation axis lock logic: check status, determine if we are locked, etc.
// ============================================================================
/**
 * @brief Query elevation lock ADCs
 */
static void GetLockADCs(void)
{
    static int counter = 0;
    // when lock motor not active, take data more slowly
    if (EZBus_IsTaken(&bus, id[LOCKNUM]) != EZ_ERR_OK && counter++ < LOCK_MOTOR_DATA_TIMER) {
        return;
    }
    counter = 0;

    // EZBus_ReadInt(&bus, id[LOCKNUM], "?0", &lock_data.pos);
    EZBus_Comm(&bus, id[LOCKNUM], "?aa");
    sscanf(bus.buffer, "%hi,%hi,%hi,%hi", &lock_data.adc[0], &lock_data.adc[1], &lock_data.adc[2], &lock_data.adc[3]);
}

/**
 * @brief Update structs related to elevation lock.
 * The NiC MCC does this via the BlastBus to give it a chance to know what's
 * going on. The ICC reads it directly to get more promptly the answer (since
 * all these fields are slow).
 * @param nic flag to determine where the state data is queried from. 1 =
 * other computer, 0 = in-charge computer
 */
static void SetLockState(int nic)
{
    static int firsttime = 1;
    int pot;
    unsigned int state;
    int i_point;

    static channel_t* potLockAddr;
    static channel_t* stateLockAddr;

    if (firsttime) {
        firsttime = 0;
        potLockAddr = channels_find_by_name("pot_lock");
        stateLockAddr = channels_find_by_name("state_lock");
    }

    // get lock data
    if (nic) {
        // use bbus when nic
        pot = GET_UINT16(potLockAddr);
        state = GET_UINT16(stateLockAddr);
        lock_data.adc[1] = pot;
    } else {
        // otherwise (in charge) use lock_data
        pot = lock_data.adc[1];
        state = lock_data.state;
    }

    // update the NIC on pot state

    // set the EZBus move parameters
    EZBus_SetVel(&bus, id[LOCKNUM], CommandData.actbus.lock_vel);
    EZBus_SetAccel(&bus, id[LOCKNUM], CommandData.actbus.lock_acc);
    EZBus_SetIMove(&bus, id[LOCKNUM], CommandData.actbus.lock_move_i);
    EZBus_SetIHold(&bus, id[LOCKNUM], CommandData.actbus.lock_hold_i);

    state &= LS_DRIVE_MASK; // zero everything but drive info

    if (pot <= LOCK_MIN_POT) {
        state |= LS_CLOSED;
    } else if (pot >= LOCK_MAX_POT) {
        state |= LS_OPEN;
    } else if ((pot < LOCK_MIN_POT + LOCK_POT_RANGE) || (pot > LOCK_MAX_POT - LOCK_POT_RANGE)) {
        // Incorporate previous state data
        state |= lock_data.state & (LS_OPEN | LS_CLOSED);
    }

    i_point = GETREADINDEX(point_index);
    if (fabs(ACSData.enc_motor_elev - LockPosition(CommandData.pointing_mode.Y)) <= 0.5) {
        state |= LS_EL_OK;
    }
    // Assume the pin is out unless we're all the way closed
    if (state & LS_CLOSED) {
        CommandData.pin_is_in = 1;
    } else {
        CommandData.pin_is_in = 0;
    }

    lock_data.state = state;
}

/**
 * @brief Decides the lock actuator action to take based on the state, the
 * goal, and the lock timeout value.
 * @param lock_state (uint32_t) lock_data.state
 * @param lock_timeout (int)
 * @param lock_goal (uint32_t*) pointer to CommandData.actbus.lock_goal, since 
 * it may need to be updated
 * @return action (int)
 */
static int GetLockAction(uint32_t lock_state, int lock_timeout, int* lock_goal)
{
    int action = LA_EXIT;
    // compare goal to current state -- only 3 goals are supported
    // open + off, closed + off and off
    if ((*lock_goal & 0x7) == (LS_OPEN | LS_DRIVE_OFF)) {
        /*                                       ORe -.
         * cUe -+-(stp)- cFe -(ext)- cXe -(---)- OXe -+-(stp)- OFe ->
         * cRe -'                                OUe -'
         */
        // Lock is OPEN and Drive is OFF, so done
        if ((lock_state & (LS_OPEN | LS_DRIVE_OFF)) == (LS_OPEN | LS_DRIVE_OFF)) {
            action = LA_EXIT;
        // Lock is OPEN and Drive is NOT OFF, so stop it
        } else if (lock_state & LS_OPEN) {
            action = LA_STOP;
        // Lock is not OPEN, but is retracting (drive not OFF), so wait.
        } else if (lock_state & (LS_DRIVE_RET)) {
            action = LA_WAIT;
        // Lock is NOT OPEN and Drive is OFF, so retract
        } else if (lock_state & LS_DRIVE_OFF) {
            action = LA_RETRACT;
        // Lock is NOT OPEN and Drive is NOT OFF, so assume stop (not retracting)
        } else {
            action = LA_STOP;
        }
    } else if ((*lock_goal & 0x7) == (LS_CLOSED | LS_DRIVE_OFF)) {
        /* oX -.         oUE -(stp)-.              CRe -(stp)-+
         * oR -+-(stp) - oF  -(---)-+- oFE -(ret)- oRE -(---)-+- CFe ->
         * oU -'         oXE -(stp)-'              CUe -(stp)-+
         *                                         CXe -(stp)-'
         */
        // Lock is CLOSED and Drive is OFF, so done
        if ((lock_state & (LS_CLOSED | LS_DRIVE_OFF)) == (LS_CLOSED | LS_DRIVE_OFF)) {
            action = LA_EXIT;
        // Lock is CLOSED and Drive is NOT OFF, so stop it
        } else if (lock_state & LS_CLOSED) {
            action = LA_STOP;
        // Elevation is in a lock position or we are ignoring elevation
        // el in range
        } else if ((lock_state & LS_EL_OK) || (*lock_goal & LS_IGNORE_EL)) {
            // Doesn't happen since LS_DRIVE_STP is never set
            if ((lock_state & (LS_OPEN | LS_DRIVE_STP)) == (LS_OPEN | LS_DRIVE_STP)) {
                action = LA_WAIT;
            // Lock is not CLOSED, but is extending (drive not OFF), so wait.
            } else if (lock_state & LS_DRIVE_EXT) {
                action = LA_WAIT;
            // Doesn't happen since LS_DRIVE_STP is never set
            } else if (lock_state & LS_DRIVE_STP) {
                action = LA_STOP;
            // Lock is not CLOSED, and Drive is OFF, so extend
            } else if (lock_state & LS_DRIVE_OFF) {
                action = LA_EXTEND;
            // Lock is not CLOSED, and Drive is NOT OFF, so assume stop (not extending)
            } else {
                action = LA_STOP;
            }
        // Elevation is not in range while we are not ignoring elevation
        } else { // el out of range
            action = (lock_state & LS_DRIVE_OFF) ? LA_WAIT : LA_STOP;
        }
    // Just stop the drive.
    } else if ((*lock_goal & 0x7) == LS_DRIVE_OFF) {
        /* ocXe -.
         * ocRe -+-(stp)- ocFe ->
         * ocUe -+
         * ocSe -'
         */
        action = (lock_state & LS_DRIVE_OFF) ? LA_EXIT : LA_STOP;
    } else {
        blast_warn("Unhandled lock goal (%x) ignored.", *lock_goal);
        *lock_goal = LS_DRIVE_OFF;
    }

    // Timeout check
    if (lock_timeout == 0) {
        bputs(warning, "Lock Motor drive timeout.");
        action = LA_STOP;
    }

    return action;
}

/**
 * @brief Performs the lock actuator action based on the result of
 * GetLockAction.
 * @param action (int) Any of LA_EXIT, LA_STOP, LA_WAIT, LA_EXTEND, LA_RETRACT
 * @param lock_timeout (int*) Used to decide when to exit the DoLock loop; 0
 * will exit
 * @param lock_state (uint32_t*) address of lock_data.state
 */
static void DoLockAction(int action, int* lock_timeout, uint32_t* lock_state)
{
    // Seize the bus
    if (action == LA_EXIT)
        EZBus_Release(&bus, id[LOCKNUM]);
    else
        EZBus_Take(&bus, id[LOCKNUM]);
    // Figure out what to do...
    switch (action) {
        case LA_STOP:
            *lock_timeout = -1;
            bputs(info, "Stopping lock motor.");
            EZBus_Stop(&bus, id[LOCKNUM]); // terminate all strings
            usleep(SEND_SLEEP); // wait for a bit
            *lock_state &= ~LS_DRIVE_MASK;
            *lock_state |= LS_DRIVE_OFF;
            break;
        case LA_EXTEND:
            *lock_timeout = DRIVE_TIMEOUT;
            bputs(info, "Extending lock motor.");
            EZBus_Stop(&bus, id[LOCKNUM]); // stop current action first
            EZBus_RelMove(&bus, id[LOCKNUM], INT_MIN);
            usleep(SEND_SLEEP); // wait for a bit
            *lock_state &= ~LS_DRIVE_MASK;
            *lock_state |= LS_DRIVE_EXT;
            break;
        case LA_RETRACT:
            *lock_timeout = DRIVE_TIMEOUT;
            bputs(info, "Retracting lock motor.");
            EZBus_Stop(&bus, id[LOCKNUM]); // stop current action first
            EZBus_RelMove(&bus, id[LOCKNUM], INT_MAX);
            usleep(SEND_SLEEP); // wait for a bit
            *lock_state &= ~LS_DRIVE_MASK;
            *lock_state |= LS_DRIVE_RET;
            break;
        case LA_WAIT:
            usleep(WAIT_SLEEP); // wait for a bit
            break;
        }
}

/**
 * @brief Loop to query elevation lock logic based on command and lock state
 * data
 */
static void DoLock(void)
{
    int action = LA_EXIT;

    do {
        GetLockADCs();

        // Fix weird states
        if (((lock_data.state & (LS_DRIVE_EXT | LS_DRIVE_RET | LS_DRIVE_UNK)) &&
                (lock_data.state & LS_DRIVE_OFF)) ||
                (CommandData.actbus.lock_goal & LS_DRIVE_FORCE)) {
            lock_data.state &= ~LS_DRIVE_MASK | LS_DRIVE_UNK;
            CommandData.actbus.lock_goal &= ~LS_DRIVE_FORCE;
            blast_warn("Reset lock motor state.");
        }

        SetLockState(0);

        action = GetLockAction(lock_data.state, lock_timeout, &CommandData.actbus.lock_goal);
        DoLockAction(action, &lock_timeout, &lock_data.state);

        // quit if timeout
        if (lock_timeout == 0) {
            lock_timeout = -1;
            action = LA_EXIT;
        }
    } while (action != LA_EXIT);
}

// ============================================================================
// Data logging functions, called from main thread
// ============================================================================
/**
 * @brief Get pointer to a given field of a given actuator via
 * channels_find_by_name
 * @param i (int) index into N actuators
 * @param field (char*) field to query for a given actuator
 */
static inline channel_t* GetActNiosAddr(int i, const char* field)
{
    snprintf(name_buffer, sizeof(name_buffer), "%s_%i_act", field, i);
    return channels_find_by_name(name_buffer);
}

/**
 * @brief handle counters in a well-timed frame synchronous manner
 */
void UpdateActFlags(void)
{
    // count down timeout on ACT_FL_TRIMMED indicator flag
    if (act_trim_flag_wait > 0) {
        act_trim_flag_wait--;
        actbus_flags |= ACT_FL_TRIMMED;
    } else {
        actbus_flags &= ~ACT_FL_TRIMMED;
    }
    // Check if waiting before trimming again
    if (act_trim_wait > 0) {
        act_trim_wait--;
        actbus_flags |= ACT_FL_TRIM_WAIT;
    } else {
        actbus_flags &= ~ACT_FL_TRIM_WAIT;
    }
    if (poll_timeout > 0) {
        poll_timeout--;
    }
    if (lock_timeout > 0) {
        lock_timeout--;
    }
}

/**
 * @brief Log data
 */
void StoreActBus(void)
{
    int j;
    static int firsttime = 1;

    static channel_t* busResetActAddr;
    static channel_t* posLockAddr;
    static channel_t* stateLockAddr;
    static channel_t* goalLockAddr;
    static channel_t* seizedActAddr;
    static channel_t* potLockAddr;
    static channel_t* pinInLockAddr;

    static channel_t* velLockAddr;
    static channel_t* accLockAddr;
    static channel_t* iMoveLockAddr;
    static channel_t* iLockHoldAddr;

    static channel_t* posShutterAddr;
    static channel_t* limsShutterAddr;
    static channel_t* iMoveShutterAddr;
    static channel_t* iHoldShutterAddr;
    static channel_t* velShutterAddr;
    static channel_t* accShutterAddr;
    static channel_t* stepShutterAddr;
    static channel_t* stepSlowShutterAddr;

    static channel_t* velActAddr;
    static channel_t* accActAddr;
    static channel_t* iMoveActAddr;
    static channel_t* iHoldActAddr;
    static channel_t* tolActAddr;
    static channel_t* flagsActAddr;
    static channel_t* modeActAddr;

    static channel_t* posActAddr[3];
    static channel_t* encActAddr[3];
    static channel_t* offsetActAddr[3];
    static channel_t* goalActAddr[3];
    static channel_t* drActAddr[3];

    static channel_t* statusActbusAddr;

    if (firsttime) {
        firsttime = 0;

        busResetActAddr = channels_find_by_name("bus_reset_act");
        pinInLockAddr = channels_find_by_name("pin_in_lock");
        posLockAddr = channels_find_by_name("pos_lock");
        stateLockAddr = channels_find_by_name("state_lock");
        seizedActAddr = channels_find_by_name("seized_act");
        goalLockAddr = channels_find_by_name("goal_lock");
        potLockAddr = channels_find_by_name("pot_lock");

        for (j = 0; j < 3; ++j) {
            posActAddr[j] = GetActNiosAddr(j, "pos");
            encActAddr[j] = GetActNiosAddr(j, "enc");
            offsetActAddr[j] = GetActNiosAddr(j, "offset");
            goalActAddr[j] = GetActNiosAddr(j, "goal");
            drActAddr[j] = GetActNiosAddr(j, "dr");
        }

        velActAddr = channels_find_by_name("vel_act");
        accActAddr = channels_find_by_name("acc_act");
        iMoveActAddr = channels_find_by_name("i_move_act");
        iHoldActAddr = channels_find_by_name("i_hold_act");
        tolActAddr = channels_find_by_name("tol_act");
        flagsActAddr = channels_find_by_name("flags_act");
        modeActAddr = channels_find_by_name("mode_act");

        velLockAddr = channels_find_by_name("vel_lock");
        accLockAddr = channels_find_by_name("acc_lock");
        iMoveLockAddr = channels_find_by_name("i_move_lock");
        iLockHoldAddr = channels_find_by_name("i_hold_lock");

        posShutterAddr = channels_find_by_name("pos_shutter");
        limsShutterAddr = channels_find_by_name("lims_shutter");
        iMoveShutterAddr = channels_find_by_name("i_move_shutter");
        iHoldShutterAddr = channels_find_by_name("i_hold_shutter");
        velShutterAddr = channels_find_by_name("vel_shutter");
        accShutterAddr = channels_find_by_name("acc_shutter");
        stepShutterAddr = channels_find_by_name("steps_shutter");
        stepSlowShutterAddr = channels_find_by_name("steps_slow_shutter");

        statusActbusAddr = channels_find_by_name("status_actbus");
    }

    UpdateActFlags();

    if (CommandData.actbus.off) {
        if (CommandData.actbus.off > 0) {
            CommandData.actbus.off--;
        }
        actbus_reset = 0; // turn actbus off
    } else {
        actbus_reset = 1;
    }

    SET_UINT16(busResetActAddr, actbus_reset);

    SET_UINT16(pinInLockAddr, CommandData.pin_is_in);

    for (j = 0; j < 3; ++j) {
        SET_UINT16(posActAddr[j], act_data[j].pos - CommandData.actbus.offset[j]);
        SET_UINT16(encActAddr[j], act_data[j].enc - CommandData.actbus.offset[j]);
        SET_UINT16(offsetActAddr[j], CommandData.actbus.offset[j]);
        SET_UINT16(goalActAddr[j], CommandData.actbus.goal[j] - CommandData.actbus.offset[j]);
        SET_UINT16(drActAddr[j], act_data[j].dr - CommandData.actbus.offset[j]);
    }

    SET_UINT16(potLockAddr, lock_data.adc[1]);
    SET_UINT16(stateLockAddr, lock_data.state);
    SET_UINT16(seizedActAddr, bus.seized);
    SET_UINT16(goalLockAddr, CommandData.actbus.lock_goal);
    SET_UINT16(posLockAddr, lock_data.pos);

    SET_UINT16(velActAddr, CommandData.actbus.act_vel);
    SET_UINT16(accActAddr, CommandData.actbus.act_acc);
    SET_UINT16(iMoveActAddr, CommandData.actbus.act_move_i);
    SET_UINT16(iHoldActAddr, CommandData.actbus.act_hold_i);
    SET_UINT16(tolActAddr, CommandData.actbus.act_tol);
    SET_UINT16(flagsActAddr, actbus_flags);
    SET_UINT16(modeActAddr, CommandData.actbus.focus_mode);

    SET_UINT16(velLockAddr, CommandData.actbus.lock_vel / 100);
    SET_UINT16(accLockAddr, CommandData.actbus.lock_acc);
    SET_UINT16(iMoveLockAddr, CommandData.actbus.lock_move_i);
    SET_UINT16(iLockHoldAddr, CommandData.actbus.lock_hold_i);

    // Shutter data
    SET_UINT16(stepShutterAddr, CommandData.actbus.shutter_step);
    SET_UINT16(stepSlowShutterAddr, CommandData.actbus.shutter_step_slow);
    SET_INT32(posShutterAddr, shutter_data.pos);
    SET_UINT16(limsShutterAddr, shutter_data.lims);
    SET_UINT16(iMoveShutterAddr, CommandData.actbus.shutter_move_i);
    SET_UINT16(iHoldShutterAddr, CommandData.actbus.shutter_hold_i);
    SET_UINT16(velShutterAddr, CommandData.actbus.shutter_vel);
    SET_UINT16(accShutterAddr, CommandData.actbus.shutter_acc);

    SET_UINT16(statusActbusAddr, actuators_init);
}

/**
 * @brief for not in charge computer to get dead reckoning actuator positions
 * from bus and save to disk
 */
void SyncDR(void)
{
    int i;
    static int firsttime = 1;
    static channel_t* drActAddr[3];
    static channel_t* offsetActAddr[3];
    int32_t dr;
    uint16_t  offset;

    if (firsttime) {
        firsttime = 0;
        drActAddr[0] = channels_find_by_name("dr_0_act");
        drActAddr[1] = channels_find_by_name("dr_1_act");
        drActAddr[2] = channels_find_by_name("dr_2_act");
        offsetActAddr[0] = channels_find_by_name("offset_0_act");
        offsetActAddr[1] = channels_find_by_name("offset_1_act");
        offsetActAddr[2] = channels_find_by_name("offset_2_act");
    }

    // get dead reckoning data
    for (i = 0; i < 3; i++) {
        dr = GET_UINT16(drActAddr[i]);
        offset = GET_UINT16(offsetActAddr[i]);
        act_data[i].dr = dr + offset;
    }

    WriteDR();
}

/**
 * @brief Actuator Thread: initialize bus and command lock/secondary steppers
 * @param param (void*)
 */
void *ActuatorBus(void *param)
{
    int all_ok = 0;
    int i;
    int j = 0; // Used for debugging print statements.  Delete later.
    int my_cindex = 0;
    int caddr_match = 0;
    int is_init = 0;
    int first_time = 1;
    int sf_ok;
    // TODO(IAN): see what we need here in the future
    // I think we don't need this
    // int valve_arr[N_PUMP_VALVES + 1] = {POTVALVE_NUM, PUMP1_VALVE_NUM, PUMP2_VALVE_NUM};

    nameThread("ActBus");
    bputs(startup, "ActuatorBus startup.");

    while (!InCharge) {
        if (first_time) {
            blast_info("Not in charge.  Waiting.");
            first_time = 0;
        }
        usleep(1000000);
        CommandData.actbus.force_repoll = 1; // repoll bus as soon as gaining control

        SetLockState(1); // to ensure the NiC MCC knows the pin state
        SyncDR(); // get encoder absolute state from the ICC

        CommandData.actbus.focus_mode = ACTBUS_FM_SLEEP; // ignore all commands
        CommandData.actbus.caddr[my_cindex] = 0; // prevent commands from executing twice if we switch to ICC
    }

    first_time = 1;
    while (!is_init) {
        if (first_time) {
            blast_info("In Charge! Attempting to initalize.");
            first_time = 0;
        }
        if (EZBus_Init(&bus, ACT_BUS, "", ACTBUS_CHATTER) == EZ_ERR_OK) is_init = 1;
        usleep(10000);
        if (is_init) {
            blast_info("Bus initialized on %ith attempt", j);
        }
        j++;
    }

    for (i = 0; i < NACT; i++) {
        blast_info("Actuator %i, id[i] =%i", i, id[i]);
        blast_info("name[i] = %s", name[i]);
        EZBus_Add(&bus, id[i], name[i]);
        if (i == BALANCENUM) {
            EZBus_SetPreamble(&bus, id[i], BALANCE_PREAMBLE);
        } else if (i == LOCKNUM) {
            EZBus_SetPreamble(&bus, id[i], LOCK_PREAMBLE);
        } else if (i == SHUTTERNUM) {
            EZBus_SetPreamble(&bus, id[i], SHUTTER_PREAMBLE);
        } else {
            EZBus_SetPreamble(&bus, id[i], actPreamble(CommandData.actbus.act_tol));
        }
        /*
         note IAN: This is removed from above for the potvalves and pump valves deprecated
         else if (i == POTVALVE_NUM) {
         EZBus_SetPreamble(&bus, id[i], POTVALVE_PREAMBLE);
         } else if ((i == PUMP1_VALVE_NUM) || (i == PUMP2_VALVE_NUM)) {
         EZBus_SetPreamble(&bus, id[i], PUMP_VALVES_PREAMBLE);
         }
         */
    }

    // I don't think this is necessary, it will always be called in the for loop --PAW 2018/06/20
    // using now because the loop will poll based on which steppers are commanded to be used
    all_ok = !(EZBus_PollInit(&bus, InitializeActuator) & EZ_ERR_POLL);

    for (;;) {
        // Repoll bus if necessary
        if (CommandData.actbus.force_repoll || bus.err_count > MAX_SERIAL_ERRORS) {
            // blast_info("forcing repoll of entire actuator bus (or polling first time)"); // DEBUG PAW
            for (i = 0; i < NACT; i++) {
                EZBus_ForceRepoll(&bus, id[i]);
            }
            poll_timeout = 0;
            all_ok = 0;
            CommandData.actbus.force_repoll = 0;
        }

        if (poll_timeout <= 0 && !all_ok && actbus_reset) {
            // suppress non-error messages during repoll
            // blast_info("supressing non-errors during repoll"); // DEBUG PAW
            // bus.chatter = EZ_CHAT_ERR;
            // for now, not changing chatter during repoll
            // blast_info("about to call EZBus_PollInit (repolling steppers that were flagged)"); // DEBUG PAW
            all_ok = !(EZBus_PollInit(&bus, InitializeActuator) & EZ_ERR_POLL);
            // blast_info("done repolling"); // DEBUG PAW
            bus.chatter = ACTBUS_CHATTER;
            poll_timeout = POLL_TIMEOUT;
        }

        // Send the uplinked command, if any
        my_cindex = GETREADINDEX(CommandData.actbus.cindex);
        caddr_match = 0;
        for (i = 0; i < NACT; i++) {
           if (CommandData.actbus.caddr[my_cindex] == id[i]) {
                caddr_match = 1;
           }
        }

        if (caddr_match) {
            // blast_info("Sending command %s to Act %c\n", CommandData.actbus.command[my_cindex],
            //            CommandData.actbus.caddr[my_cindex]);
            // increase print level for uplinked manual commands
            bus.chatter = EZ_CHAT_BUS;
            EZBus_Comm(&bus, CommandData.actbus.caddr[my_cindex], CommandData.actbus.command[my_cindex]);
            CommandData.actbus.caddr[my_cindex] = 0;
            bus.chatter = ACTBUS_CHATTER;
        }

        if (EZBus_IsUsable(&bus, id[LOCKNUM])) {
            // blast_info("calling DoLock"); // DEBUG PAW
            DoLock();
            actuators_init |= 0x1 << LOCKNUM;
        } else {
            // blast_info("forcing repoll of lockpin"); // DEBUG PAW
            EZBus_ForceRepoll(&bus, id[LOCKNUM]);
            all_ok = 0;
            actuators_init &= ~(0x1 << LOCKNUM);
        }

        if (EZBus_IsUsable(&bus, id[SHUTTERNUM])) {
            // blast_info("calling DoShutter"); // DEBUG PAW
            DoShutter();
            actuators_init |= 0x1 << SHUTTERNUM;
        } else {
            // blast_info("forcing repoll of shutter"); // DEBUG PAW
            EZBus_ForceRepoll(&bus, id[SHUTTERNUM]);
            all_ok = 0;
            actuators_init &= ~(0x1 << SHUTTERNUM);
        }

        sf_ok = 1;
        for (i = 0; i < 3; i++) {
            if (EZBus_IsUsable(&bus, id[i])) {
                actuators_init |= 0x1 << i;
            } else {
                EZBus_ForceRepoll(&bus, id[i]);
                all_ok = 0;
                sf_ok = 0;
                actuators_init &= ~(0x1 << i);
            }
        }

        if (sf_ok) {
            DoActuators();
        }

        if (EZBus_IsUsable(&bus, id[BALANCENUM])) {
            // blast_info("calling DoBalance"); // DEBUG PAW
            DoBalance(&bus);
            actuators_init |= 0x1 << BALANCENUM;
        } else {
            // blast_info("forcing repoll of balance"); // DEBUG PAW
            EZBus_ForceRepoll(&bus, id[BALANCENUM]);
            all_ok = 0;
            actuators_init &= ~(0x1 << BALANCENUM);
        }

        usleep(10000);
    }
}
