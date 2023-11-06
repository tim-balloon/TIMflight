/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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

#include "motors.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include "channels_tng.h"
#include "tx.h"
#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"
#include "motordefs.h"

#include "angles.h"
#include "conversions.h"
#include "mputs.h"
#include "pointing.h"
#include "radbox.h"
#include "ec_motors.h"

extern int16_t InCharge;

// motor info storage for the RW motor
motor_data_t RWMotorData[3] = {{0}};
// motor info storage for the elevation motor
motor_data_t ElevMotorData[3] = {{0}};
// motor info storage for the pivot motor
motor_data_t PivotMotorData[3] = {{0}};
int motor_index = 0;

// some inits for the axes movement modes
struct AxesModeStruct axes_mode = {
  .el_dir = 1,
  .az_dir = 0,
  .i_dith = 0
}; /* low level velocity mode */

// motor control parameters
#define MOTORSR 200.0 // sample rate of the motors (Hz)
#define MAX_DI_EL 20.0 // Maximum integral accumulation per step in milliamps for El motor
#define MAX_I_EL  800.0 // Maximum accumulated integral in milliamps for El motor
#define MAX_DI 20.0 // Maximum integral accumulation per step in milliamps for Az motors
#define MAX_I  200.0 // Maximum accumulated integral in milliamps for Az motors

#define INTEGRAL_LENGTH  5.0  // length of the integral time constant in seconds
#define INTEGRAL_CUTOFF (1.0/(INTEGRAL_LENGTH*MOTORSR))

#define LPFILTER_POLES 5
#define LPFILTER_GAIN  1.099763251e+09
static const double lpfilter_coefs[LPFILTER_POLES] = { 0.903328285, -4.60847636,
                                                       9.40530799, -9.59849709, 4.89833715 };

// border for elevation moves, for turnaround
#define EL_BORDER 1.0
// border for azimuth moves, for turnaround
#define AZ_BORDER 1.0
// minimum scan size in degrees
#define MIN_SCAN 0.2
// TODO(evanmayer, ianlowe13): test this with 0.1 as min size for the TIM fields

// azimuth acceleration in degrees/s/s
static double az_accel = 0.1;
// last pointing mode
static int last_mode = -1;

extern bool scan_entered_snap_mode;
extern bool scan_leaving_snap_mode;


/**
 * @brief Writes the axes mode data to the frame
 */
void store_axes_mode_data(void)
{
    /* low level scan mode diagnostics */
    static channel_t *modeAzMcAddr;
    static channel_t *modeElMcAddr;
    static channel_t *dirAzMcAddr;
    static channel_t *dirElMcAddr;
    static channel_t *destAzMcAddr;
    static channel_t *destElMcAddr;
    static channel_t *dithElAddr;
    static channel_t *velAzMcAddr;
    static channel_t *velElMcAddr;
    static channel_t *iDithMcAddr;

    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;

        modeAzMcAddr = channels_find_by_name("mode_az_mc");
        modeElMcAddr = channels_find_by_name("mode_el_mc");
        destAzMcAddr = channels_find_by_name("dest_az_mc");
        destElMcAddr = channels_find_by_name("dest_el_mc");
        velAzMcAddr = channels_find_by_name("vel_az_mc");
        velElMcAddr = channels_find_by_name("vel_el_mc");
        dirAzMcAddr = channels_find_by_name("dir_az_mc");
        dirElMcAddr = channels_find_by_name("dir_el_mc");
        dithElAddr = channels_find_by_name("dith_el");
        iDithMcAddr = channels_find_by_name("i_dith_el");
    }

    /* scan modes */
    SET_SCALED_VALUE(modeAzMcAddr, axes_mode.az_mode);
    SET_SCALED_VALUE(modeElMcAddr, axes_mode.el_mode);
    SET_SCALED_VALUE(dirAzMcAddr, axes_mode.az_dir);
    SET_SCALED_VALUE(dirElMcAddr, axes_mode.el_dir);
    SET_SCALED_VALUE(dithElAddr, axes_mode.el_dith);
    SET_SCALED_VALUE(destAzMcAddr, axes_mode.az_dest);
    SET_SCALED_VALUE(destElMcAddr, axes_mode.el_dest);
    SET_SCALED_VALUE(velAzMcAddr, axes_mode.az_vel);
    SET_SCALED_VALUE(velElMcAddr, axes_mode.el_vel);
    SET_SCALED_VALUE(iDithMcAddr, axes_mode.i_dith);
}


/**
 * @brief GetVElev: get the current elevation velocity, given current
 * pointing mode, etc..
 * 
 * @return double elevation velocity in units of 0.1*gyro unit
 */
static double get_elev_vel(void)
{
    double vel = 0;
    static double last_vel = 0;
    double dvel;
    int i_point, i_elev;
    double max_dv = 0.1;
    double el_for_limit, el, el_dest;
    double dy;

    i_point = GETREADINDEX(point_index);
    i_elev = GETREADINDEX(motor_index);

    if (axes_mode.el_mode == AXIS_VEL) {
        vel = axes_mode.el_vel;
    } else if (axes_mode.el_mode == AXIS_POSITION) {
        el = PointingData[i_point].el;
        el_dest = axes_mode.el_dest;
        dy = el_dest - el;
        if (dy < 0) {
            vel = -sqrt(-dy);
        } else {
            vel = sqrt(dy);
        }
        vel *= (double) CommandData.ele_gain.PT / MOTORSR;
        //    vel = (axes_mode.el_dest - PointingData[i_point].el) * 0.36;
    } else if (axes_mode.el_mode == AXIS_LOCK) {
        /* for the lock, only use the elevation encoder */
        vel = (axes_mode.el_dest - ElevMotorData[i_elev].motor_position * EL_MOTOR_ENCODER_SCALING) * 0.64;
    }

    /* correct offset and convert to Gyro Units */
    vel -= (PointingData[i_point].offset_ifel_gy - PointingData[i_point].ifel_earth_gy);

    if (CommandData.use_elmotenc) {
        el_for_limit = wrap_to(el_get_motor_position_degrees() + CommandData.enc_motor_el_trim, 360.0);
    } else {
        el_for_limit = PointingData[i_point].el;
    }

    if (el_for_limit < MIN_EL) {
        if (vel <= 0) { // if we are going down
            vel = (MIN_EL - el_for_limit) * 0.36; // go to the stop
        }
    }
    if (el_for_limit > MAX_EL) {
        if (vel >= 0) { // if we are going up
            vel = (MAX_EL - el_for_limit) * 0.36; // go to the stop
        }
    }

    /* Limit Maximim speed to 0.5 dps*/
    if (vel > MAX_V_EL)
        vel = MAX_V_EL;
    if (vel < (-1.0) * MAX_V_EL)
        vel = (-1.0) * MAX_V_EL;

    /* limit Maximum acceleration */
    dvel = vel - last_vel;
    if (dvel > max_dv)
        vel = last_vel + max_dv;
    if (dvel < -max_dv)
        vel = last_vel - max_dv;
    last_vel = vel;

    // blast_info("GetVEl: vel=%f", vel);
    return vel;
}


/**
 * @brief GetVAz: get the current az velocity, given current
 * pointing mode, etc..
 * @return double azimuth velocity in degrees per second
 */
static double get_az_vel(void)
{
    double vel = 0.0;
    double vel_offset = 0.0;
    static double last_vel = 0;
    double dvel;
    int i_point;

    double az, az_dest;
    double max_dv = 0.1; // TODO(seth): Put max acceleration into commanddata
    double dx;

    i_point = GETREADINDEX(point_index);

    if (axes_mode.az_mode == AXIS_VEL) {
        vel = axes_mode.az_vel;
    } else if (axes_mode.az_mode == AXIS_POSITION) {
        az = PointingData[i_point].az;
        az_dest = axes_mode.az_dest;
        SetSafeDAz(az, &az_dest);
        dx = az_dest - az;
        if (dx < 0) {
            vel = -sqrt(-dx);
        } else {
            vel = sqrt(dx);
        }
        vel *= (double) CommandData.azi_gain.PT / MOTORSR;
    }

    vel_offset =
            -(PointingData[i_point].offset_ifroll_gy - PointingData[i_point].ifroll_earth_gy)
                * sin(from_degrees(PointingData[i_point].el))
            -(PointingData[i_point].offset_ifyaw_gy - PointingData[i_point].ifyaw_earth_gy)
                * cos(from_degrees(PointingData[i_point].el));

    vel -= vel_offset;
    /* Limit Maximum speed */
    if (vel > MAX_V_AZ)
        vel = MAX_V_AZ;
    if (vel < -MAX_V_AZ)
        vel = -MAX_V_AZ;

    /* limit Maximum acceleration */
    dvel = vel - last_vel;
    if (dvel > max_dv) vel = last_vel + max_dv;
    if (dvel < -max_dv) vel = last_vel - max_dv;
    last_vel = vel;

    return vel;
}


/**
 * @brief stores the 100hz motor data to the channels/frame
 * seems to just be the reaction wheel.
 * 
 */
void write_motor_channels_100hz(void)
{
    int i_motors;
    /* Motor data read out over motor thread in ec_motors.c */
    static channel_t *statusRWAddr;
    static channel_t *networkStatusRWAddr;
    static channel_t *networkProblemRWAddr;
    static channel_t *stateRWAddr;
    static channel_t *ctl_word_read_rw_addr;
    static channel_t *ctl_word_write_rw_addr;
    static channel_t *latched_fault_rw_addr;

    // Timing data is here to get higher resolution
    static channel_t* timeAddr;
    static channel_t* timeUSecAddr;
    struct timeval tv;
    struct timezone tz;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;
        statusRWAddr = channels_find_by_name("status_rw");
        networkStatusRWAddr = channels_find_by_name("network_status_rw");
        networkProblemRWAddr = channels_find_by_name("network_problem_rw");
        stateRWAddr = channels_find_by_name("state_rw");
        ctl_word_read_rw_addr = channels_find_by_name("control_word_read_rw");
        ctl_word_write_rw_addr = channels_find_by_name("control_word_write_rw");
        latched_fault_rw_addr = channels_find_by_name("latched_fault_rw");

        timeAddr = channels_find_by_name("time");
        timeUSecAddr = channels_find_by_name("time_usec");
    }
    i_motors = GETREADINDEX(motor_index);
    SET_UINT32(statusRWAddr, RWMotorData[i_motors].status);
    SET_UINT16(networkStatusRWAddr, RWMotorData[i_motors].network_status);
    SET_UINT16(networkProblemRWAddr, RWMotorData[i_motors].network_problem);
    SET_UINT16(stateRWAddr, RWMotorData[i_motors].drive_info);
    SET_UINT16(ctl_word_read_rw_addr, RWMotorData[i_motors].control_word_read);
    SET_UINT16(ctl_word_write_rw_addr, RWMotorData[i_motors].control_word_write);
    SET_UINT32(latched_fault_rw_addr, RWMotorData[i_motors].fault_reg);

    gettimeofday(&tv, &tz);
    SET_VALUE(timeAddr, tv.tv_sec + TEMPORAL_OFFSET);
    SET_VALUE(timeUSecAddr, tv.tv_usec);
}


/**
 * @brief writes the 5hz motor data to the channels/frame
 * 
 */
void write_motor_channels_5hz(void)
{
    uint8_t ec_cmd_status_field = 0;
    static channel_t* gPElAddr;
    static channel_t* gIElAddr;
    static channel_t* gDElAddr;
    static channel_t* gPtElAddr;
    static channel_t* gDBElAddr;

    static channel_t* gPAzAddr;
    static channel_t* gIAzAddr;
    static channel_t* gDAzAddr;

    static channel_t* gPtAzAddr;
    static channel_t* gPVPivAddr;
    static channel_t* gIVPivAddr;
    static channel_t* gIEPivAddr;
    static channel_t* gPEPivAddr;
    static channel_t* setRWAddr;
    static channel_t* frictOffPivAddr;
    static channel_t* frictOffElAddr;
    static channel_t* accelAzAddr;

    static channel_t *tMCRWAddr;
    static channel_t *phase_angle_rw_addr;
    static channel_t *phase_mode_rw_addr;

    static channel_t *tMCElAddr;
    static channel_t *statusElAddr;
    static channel_t *networkStatusElAddr;
    static channel_t *networkProblemElAddr;
    static channel_t *stateElAddr;
    static channel_t *ctl_word_read_el_addr;
    static channel_t *ctl_word_write_el_addr;
    static channel_t *latched_fault_el_addr;
    static channel_t *phase_angle_el_addr;
    static channel_t *phase_mode_el_addr;

    static channel_t *tMCPivAddr;
    static channel_t *statusPivAddr;
    static channel_t *networkStatusPivAddr;
    static channel_t *networkProblemPivAddr;
    static channel_t *statePivAddr;
    static channel_t *ctl_word_read_piv_addr;
    static channel_t *ctl_word_write_piv_addr;
    static channel_t *latched_fault_piv_addr;
    static channel_t *phase_angle_piv_addr;
    static channel_t *phase_mode_piv_addr;

    int i_motors;

    static channel_t *ethercat_cmds_addr;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;

        gPElAddr = channels_find_by_name("g_p_el");
        gIElAddr = channels_find_by_name("g_i_el");
        gDElAddr = channels_find_by_name("g_d_el");
        gPtElAddr = channels_find_by_name("g_pt_el");
        gDBElAddr = channels_find_by_name("g_db_el");

        gPAzAddr = channels_find_by_name("g_p_az");
        gIAzAddr = channels_find_by_name("g_i_az");
        gDAzAddr = channels_find_by_name("g_d_az");
        gPtAzAddr = channels_find_by_name("g_pt_az");

        gPVPivAddr = channels_find_by_name("g_pv_piv");
        gIVPivAddr = channels_find_by_name("g_iv_piv");
        gIEPivAddr = channels_find_by_name("g_ie_piv");
        gPEPivAddr = channels_find_by_name("g_pe_piv");

        setRWAddr = channels_find_by_name("set_rw");
        frictOffPivAddr = channels_find_by_name("frict_off_piv");
        frictOffElAddr = channels_find_by_name("frict_off_el");
        accelAzAddr = channels_find_by_name("accel_az");

        tMCRWAddr = channels_find_by_name("t_mc_rw");
        phase_angle_rw_addr = channels_find_by_name("mc_phase_rw");
        phase_mode_rw_addr = channels_find_by_name("mc_phase_mode_rw");

        tMCElAddr = channels_find_by_name("t_mc_el");
        statusElAddr = channels_find_by_name("status_el");
        networkStatusElAddr = channels_find_by_name("network_status_el");
        networkProblemElAddr = channels_find_by_name("network_problem_el");
        stateElAddr = channels_find_by_name("state_el");
        ctl_word_read_el_addr = channels_find_by_name("control_word_read_el");
        ctl_word_write_el_addr = channels_find_by_name("control_word_write_el");
        latched_fault_el_addr = channels_find_by_name("latched_fault_el");
        phase_angle_el_addr = channels_find_by_name("mc_phase_el");
        phase_mode_el_addr = channels_find_by_name("mc_phase_mode_el");

        tMCPivAddr = channels_find_by_name("t_mc_piv");
        statusPivAddr = channels_find_by_name("status_piv");
        networkStatusPivAddr = channels_find_by_name("network_status_piv");
        networkProblemPivAddr = channels_find_by_name("network_problem_piv");
        statePivAddr = channels_find_by_name("state_piv");
        ctl_word_read_piv_addr = channels_find_by_name("control_word_read_piv");
        ctl_word_write_piv_addr = channels_find_by_name("control_word_write_piv");
        latched_fault_piv_addr = channels_find_by_name("latched_fault_piv");
        phase_angle_piv_addr = channels_find_by_name("mc_phase_piv");
        phase_mode_piv_addr = channels_find_by_name("mc_phase_mode_piv");

        ethercat_cmds_addr = channels_find_by_name("mc_cmd_status");
    }

    /***************************************************/
    /**           Elevation Drive Motors              **/

    /* proportional term for el motor */
    SET_FLOAT(gPElAddr, CommandData.ele_gain.P);
    /* integral term for el_motor */
    SET_FLOAT(gIElAddr, CommandData.ele_gain.I);
    /* derivative term for el_motor */
    SET_FLOAT(gDElAddr, CommandData.ele_gain.D);
    /* pointing gain term for elevation drive */
    SET_FLOAT(gPtElAddr, CommandData.ele_gain.PT);
    /* deadband for the el_motor integral term*/
    SET_FLOAT(gDBElAddr, CommandData.ele_gain.DB);
    /* Elevation current offset to compensate for static friction. */
    SET_FLOAT(frictOffElAddr, CommandData.ele_gain.F);

    /***************************************************/
    /**            Azimuth Drive Motors              **/

    //  blast_info("Motors: pivFrictOff= %f, CommandData.pivot_gain.F = %f",pivFrictOff,CommandData.pivot_gain.F);
    /* p term for az motor */
    SET_FLOAT(gPAzAddr, CommandData.azi_gain.P);
    /* I term for az motor */
    SET_FLOAT(gIAzAddr, CommandData.azi_gain.I);
    /* D term for az motor */
    SET_FLOAT(gDAzAddr, CommandData.azi_gain.D);
    /* pointing gain term for az drive */
    SET_FLOAT(gPtAzAddr, CommandData.azi_gain.PT);

    /* p term to rw vel for pivot motor */
    SET_FLOAT(gPVPivAddr, CommandData.pivot_gain.PV);
    /* I term to rw vel for pivot motor */
    SET_FLOAT(gIVPivAddr, CommandData.pivot_gain.IV);
    /* p term to vel error for pivot motor */
    SET_FLOAT(gPEPivAddr, CommandData.pivot_gain.PE);
    /* setpoint for reaction wheel */
    SET_FLOAT(setRWAddr, CommandData.pivot_gain.SP);
    /* I term to vel error for pivot motor */
    SET_FLOAT(gIEPivAddr, CommandData.pivot_gain.IE);
    /* Pivot current offset to compensate for static friction. */
    SET_FLOAT(frictOffPivAddr, CommandData.pivot_gain.F);
    /* Azimuth Scan Acceleration */
    SET_VALUE(accelAzAddr, (CommandData.az_accel / 2.0 * 65536.0));

    /**
     * Motor Controller Fields
     */
    i_motors = GETREADINDEX(motor_index);
    SET_INT16(tMCRWAddr, RWMotorData[i_motors].temp);
    SET_INT16(phase_angle_rw_addr, RWMotorData[i_motors].phase_angle);
    SET_INT16(phase_mode_rw_addr, RWMotorData[i_motors].phase_mode);

    SET_INT16(tMCElAddr, ElevMotorData[i_motors].temp);
    SET_UINT32(statusElAddr, ElevMotorData[i_motors].status);
    SET_UINT16(networkStatusElAddr, ElevMotorData[i_motors].network_status);
    SET_UINT16(networkProblemElAddr, ElevMotorData[i_motors].network_problem);
    SET_UINT16(stateElAddr, ElevMotorData[i_motors].drive_info);
    SET_UINT16(ctl_word_read_el_addr, ElevMotorData[i_motors].control_word_read);
    SET_UINT16(ctl_word_write_el_addr, ElevMotorData[i_motors].control_word_write);
    SET_UINT32(latched_fault_el_addr, ElevMotorData[i_motors].fault_reg);
    SET_INT16(phase_angle_el_addr, ElevMotorData[i_motors].phase_angle);
    SET_UINT16(phase_mode_el_addr, ElevMotorData[i_motors].phase_mode);

    SET_INT16(tMCPivAddr, PivotMotorData[i_motors].temp);
    SET_UINT32(statusPivAddr, PivotMotorData[i_motors].status);
    SET_UINT16(networkStatusPivAddr, PivotMotorData[i_motors].network_status);
    SET_UINT16(networkProblemPivAddr, PivotMotorData[i_motors].network_problem);
    SET_UINT16(statePivAddr, PivotMotorData[i_motors].drive_info);
    SET_UINT16(ctl_word_read_piv_addr, PivotMotorData[i_motors].control_word_read);
    SET_UINT16(ctl_word_write_piv_addr, PivotMotorData[i_motors].control_word_write);
    SET_UINT32(latched_fault_piv_addr, PivotMotorData[i_motors].fault_reg);
    SET_INT16(phase_angle_piv_addr, PivotMotorData[i_motors].phase_angle);
    SET_INT16(phase_mode_piv_addr, PivotMotorData[i_motors].phase_mode);

    ec_cmd_status_field = ((uint8_t)CommandData.ec_devices.reset) +
                          ((uint8_t)CommandData.ec_devices.fix_rw << 1) +
                          ((uint8_t)CommandData.ec_devices.fix_el << 2) +
                          ((uint8_t)CommandData.ec_devices.fix_piv << 3);
    SET_UINT8(ethercat_cmds_addr, ec_cmd_status_field);
}


/**
 * @brief writes the 200Hz motor data to the channels/frame
 * 
 */
void write_motor_channels_200hz(void)
{
    static channel_t* el_current_read;
    static channel_t* rw_current_read;
    static channel_t* piv_current_read;

    static channel_t* az_req_velocity;
    static channel_t* el_req_velocity;

    float v_elev, v_az;
    int i_motors;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;
        el_req_velocity = channels_find_by_name("vel_req_el");
        az_req_velocity = channels_find_by_name("vel_req_az");
        el_current_read = channels_find_by_name("mc_el_i_read");
        rw_current_read = channels_find_by_name("mc_rw_i_read");
        piv_current_read = channels_find_by_name("mc_piv_i_read");
    }

    /***************************************************/
    /**           Elevation Drive Motors              **/
    /* elevation speed */
    v_elev = get_elev_vel();
    SET_FLOAT(el_req_velocity, v_elev);

    /***************************************************/
    /**            Azimuth Drive Motors              **/
    v_az = get_az_vel();
    SET_FLOAT(az_req_velocity, v_az);

    /**
     * Motor Controller Fields
     */
    i_motors = GETREADINDEX(motor_index);
    SET_INT16(el_current_read, ElevMotorData[i_motors].current * 100.0);
    SET_INT16(rw_current_read, RWMotorData[i_motors].current * 100.0);
    SET_INT16(piv_current_read, PivotMotorData[i_motors].current * 100.0);
}

/***************************************************************/
/*                                                             */
/* GetElDither: set the current elevation dither offset.       */
/*                                                             */
/***************************************************************/


/**
 * @brief calculates and sets the elevation dither terms N, I, and El
 * so that we get some slightly offset scans of the same portion of the sky
 * for better coverage with the detector arrays.
 * 
 * @param m_inc boolean, do we increment the dither or not
 */
static void calculate_el_dither(unsigned int m_inc)
{
    // Set up the random variable.

    if (m_inc) {
        (axes_mode.i_dith)++;
        blast_info("GetElDither: Incrementing axes_mode.i_dith to %i", axes_mode.i_dith);
    }
    if (CommandData.pointing_mode.n_dith <= 0) {
        axes_mode.el_dith = 0.0;
        if (m_inc) blast_info("No dither: axes_mode.el_dith = %f", axes_mode.el_dith);
    } else {
        axes_mode.i_dith %= (CommandData.pointing_mode.n_dith);
        axes_mode.el_dith = 2.0 * (CommandData.pointing_mode.del) *
                ((double) axes_mode.i_dith) / ((double) (CommandData.pointing_mode.n_dith));
    }

    if (m_inc) blast_info("***Dither Time!!!***  El Dither = %f", axes_mode.el_dith);

    if (axes_mode.el_dith > CommandData.pointing_mode.del) {
        axes_mode.el_dith += (-2.0) * CommandData.pointing_mode.del;
        if (m_inc) blast_info("GetElDither: Wrapping dither... axes_mode.el_dith=%f", axes_mode.el_dith);
    }
}


/**
 * @brief Initializes the el dithering with a call to
 * calculate dither with no increment. This is called
 * in all pointing modes/scan types the first time
 * through the scan algorithm.
 * 
 */
static void initialize_el_dither()
{
    if (CommandData.pointing_mode.next_i_dith >= 0) {
        axes_mode.i_dith = CommandData.pointing_mode.next_i_dith;
        CommandData.pointing_mode.next_i_dith = -1;
    } else {
        CommandData.pointing_mode.next_i_dith = -1;
    }

    calculate_el_dither(NO_DITH_INC);
}

/****************************************************************/
/*                                                              */
/*   Do scan modes                                              */
/*                                                              */
/****************************************************************/


/**
 * @brief Calculates the azimuth velocity. This can be in terms of the 
 * allowed limits, turnarounds, or simply setting the vel += accel on a call
 * for the PID targets. We also set booleans that help with triggers set here.
 * 
 * @param m_az current azimuth
 * @param m_leftbound the left border in az coordinates
 * @param m_rightbound the right border in az coordinates
 * @param m_vel scan speed in azimuth direction (commanded or scheduled)
 * @param m_az_drift_vel Sky drift velocity of the patch in terms of telescope azimuth
 */
static void calculate_az_mode_vel(double m_az, double m_leftbound, double m_rightbound, double m_vel,
                                  double m_az_drift_vel)
{
    static bool from_scan_to_turnaround = false;
    static bool from_turnaround_to_scan = true;
    if (axes_mode.az_vel < -m_vel + m_az_drift_vel) axes_mode.az_vel = -m_vel + m_az_drift_vel;
    if (axes_mode.az_vel > m_vel + m_az_drift_vel) axes_mode.az_vel = m_vel + m_az_drift_vel;

    if (m_az < m_leftbound) {
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel < m_vel + m_az_drift_vel) axes_mode.az_vel += az_accel;
        // Forced Triggering
        if (from_scan_to_turnaround) {
            scan_entered_snap_mode = true;
            from_scan_to_turnaround = false;
        }
        from_turnaround_to_scan = true;
    } else if (m_az > m_rightbound) {
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel > -m_vel + m_az_drift_vel) axes_mode.az_vel -= az_accel;
        // Forced Triggering
        if (from_scan_to_turnaround) {
            scan_entered_snap_mode = true;
            from_scan_to_turnaround = false;
        }
        from_turnaround_to_scan = true;
    } else {
        axes_mode.az_mode = AXIS_VEL;
        if (axes_mode.az_vel > 0) {
            axes_mode.az_vel = m_vel + m_az_drift_vel;
        } else {
            axes_mode.az_vel = -m_vel + m_az_drift_vel;
        }
        // Forced Triggering
        from_scan_to_turnaround = true;
        if (from_turnaround_to_scan) {
            scan_leaving_snap_mode = true;
            from_turnaround_to_scan = false;
        }
    }
}


/**
 * @brief Calculates the elevation velocity goal. Checks if we are out of range
 * +/- of the allowed elevation velocities and put it within the limits. Then we 
 * check to see if we are "out of bounds" of the scan and do an el turnaround if we 
 * are. Otherwise we just try and accelerate vel to the goal.
 * 
 * @param m_el current elevation
 * @param m_botbound lowest elevation in the scan
 * @param m_topbound highest elevation in the scan
 * @param m_el_vel commanded elevation velocity
 * @param m_el_drift sky drift speed in terms of the elevation coordinate
 */
static void calculate_el_mode_vel(double m_el, double m_botbound, double m_topbound, double m_el_vel, double m_el_drift)
{
    double el_accel = EL_ACCEL / SR;
    if (axes_mode.el_vel < -m_el_vel + m_el_drift)
        axes_mode.el_vel = -m_el_vel + m_el_drift;
    if (axes_mode.el_vel > m_el_vel + m_el_drift)
        axes_mode.el_vel = m_el_vel + m_el_drift;

    if (m_el < m_botbound) {
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel < m_el_vel + m_el_drift)
            axes_mode.el_vel += el_accel;
    } else if (m_el > m_topbound) {
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel > -m_el_vel + m_el_drift)
            axes_mode.el_vel -= el_accel;
    } else {
        axes_mode.el_mode = AXIS_VEL;
        if (axes_mode.el_vel > 0) {
            axes_mode.el_vel = m_el_vel + m_el_drift;
        } else {
            axes_mode.el_vel = -m_el_vel + m_el_drift;
        }
    }
}


/**
 * @brief Executes the commanded "azimuth only" scan
 * 
 */
static void do_az_scan_mode(void)
{
    static double last_x = 0, last_w = 0;
    double az, left, right, v, w;
    int i_point;

    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = CommandData.pointing_mode.Y;
    axes_mode.el_vel = 0.0;

    i_point = GETREADINDEX(point_index);
    az = PointingData[i_point].az;

    w = CommandData.pointing_mode.w;
    right = CommandData.pointing_mode.X + w / 2;
    left = CommandData.pointing_mode.X - w / 2;

    SetSafeDAz(left, &az);

    v = CommandData.pointing_mode.vaz;

    if (last_x != CommandData.pointing_mode.X || last_w != w) {
        if (az < left) {
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = left;
            axes_mode.az_vel = 0.0;
        } else if (az > right) {
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = right;
            axes_mode.az_vel = 0.0;
        } else {
            // once we are within the new az/w range, we can mark this as 'last'.
            last_x = CommandData.pointing_mode.X;
            last_w = w;
            calculate_az_mode_vel(az, left, right, v, 0);
        }
    } else {
        calculate_az_mode_vel(az, left, right, v, 0);
    }
}


/**
 * @brief Executes the commanded "elevation only" scan
 * 
 */
static void do_el_scan_mode(void)
{
    static double last_y = 0, last_h = 0;
    double el, top, bottom, v, h;
    //  double az, left, right, v,w;
    int i_point;
    static int first_time = 1;

    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = CommandData.pointing_mode.X;
    axes_mode.az_vel = 0.0;

    i_point = GETREADINDEX(point_index);
    el = PointingData[i_point].el;

    h = CommandData.pointing_mode.h;
    top = CommandData.pointing_mode.Y + h / 2;
    bottom = CommandData.pointing_mode.Y - h / 2;

    if (first_time)
        blast_info("Starting an elevation scan! h = %f, top=%f , bottom=%f", h, top, bottom);
    first_time = 0;

    v = CommandData.pointing_mode.vel;

    if (last_y != CommandData.pointing_mode.Y || last_h != h) {
        if (el < bottom) {
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = bottom;
            axes_mode.el_vel = 0.0;
        } else if (el > top) {
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = top;
            axes_mode.el_vel = 0.0;
        } else {
            // once we are within the new az/w range, we can mark this as 'last'.
            last_y = CommandData.pointing_mode.Y;
            last_h = h;
            calculate_el_mode_vel(el, bottom, top, v, 0);
        }
    } else {
        calculate_el_mode_vel(el, bottom, top, v, 0);
    }
}


// Deprecated (INL, 2023, still deprecated)
static void do_mode_vcap(void)
{
    double caz, cel;
    double az, az2, el, el1, el2;
    double daz_dt, del_dt, v_el;
    double lst;
    int i_point;
    double y, r, v;
    double x2, xw;
    double left, right;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    if (el > 80)
        el = 80; /* very bad situation - don't know how this can happen */
    if (el < -10)
        el = -10; /* very bad situation - don't know how this can happen */

    /* get raster center and sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst, PointingData[i_point].lat, &caz, &cel);
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst + 1.0, PointingData[i_point].lat, &az2, &el2);
    daz_dt = drem(az2 - caz, 360.0);
    del_dt = el2 - cel;
    SetSafeDAz(az, &caz);

    /* get elevation limits */
    if (cel < MIN_EL)
        cel = MIN_EL;
    if (cel > MAX_EL)
        cel = MAX_EL;
    r = CommandData.pointing_mode.w;
    el1 = cel + r;
    el2 = cel - r;
    if (el1 > MAX_EL)
        el1 = MAX_EL;
    if (el2 < MIN_EL)
        el2 = MIN_EL;

    /* check for out of range in el */
    if (el > el1 + EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el1;
        axes_mode.el_dir = -1;
        return;
    } else if (el < el2 - EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el2;
        axes_mode.el_dir = 1;
        return;
    } else if (el > el1) { /* turn around */
        axes_mode.el_dir = -1;
    } else if (el < el2) { /* turn around */
        axes_mode.el_dir = 1;
    }
    v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

    /* we must be in range for elevation - go to el-vel mode */
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = v_el + del_dt;

    /** Get x limits **/
    y = el - cel;
    x2 = r * r - y * y;
    if (x2 < 0) {
        xw = 0.0;
    } else {
        xw = sqrt(x2);
    }
    if (xw < MIN_SCAN)
        xw = MIN_SCAN;
    xw /= cos(el * M_PI / 180.0);
    left = caz - xw;
    right = caz + xw;

    /* set az v */
    v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
    calculate_az_mode_vel(az, left, right, v, daz_dt);
}


// Deprecated (INL, 2023, still deprecated)
static void do_mode_velocity_box(void)
{
    double caz, cel;
    double az, az2, el, el1, el2;
    double daz_dt, del_dt, v_el;
    double lst;
    int i_point;
    double y, x, v;
    double left, right;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    if (el > 80)
        el = 80; /* very bad situation - dont know how this can happen */
    if (el < -10)
        el = -10; /* very bad situation - dont know how this can happen */

    /* get raster center and sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst, PointingData[i_point].lat, &caz, &cel);
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst + 1.0, PointingData[i_point].lat, &az2, &el2);
    daz_dt = drem(az2 - caz, 360.0);
    del_dt = el2 - cel;
    SetSafeDAz(az, &caz);

    /* get elevation limits */
    if (cel < MIN_EL)
        cel = MIN_EL;
    if (cel > MAX_EL)
        cel = MAX_EL;
    y = CommandData.pointing_mode.h / 2.0;
    el1 = cel + y;
    el2 = cel - y;
    if (el1 > MAX_EL)
        el1 = MAX_EL;
    if (el2 < MIN_EL)
        el2 = MIN_EL;

    /* check for out of range in el */
    if (el > el1 + EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el1;
        axes_mode.el_dir = -1;
        return;
    } else if (el < el2 - EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el2;
        axes_mode.el_dir = 1;
        return;
    } else if (el > el1) { /* turn around */
        axes_mode.el_dir = -1;
    } else if (el < el2) { /* turn around */
        axes_mode.el_dir = 1;
    }
    v_el = CommandData.pointing_mode.del * axes_mode.el_dir;

    /* we must be in range for elevation - go to el-vel mode */
    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = v_el + del_dt;

    /** Get x limits **/
    x = CommandData.pointing_mode.w / 2.0;
    x = x / cos(el * M_PI / 180.0);

    left = caz - x;
    right = caz + x;

    /* set az v */
    v = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
    calculate_az_mode_vel(az, left, right, v, daz_dt);
}


/**
 * @brief executes the move to a specified RA-DEC coordinate
 * 
 */
static void do_mode_RAdec_goto(void)
{
    double caz, cel;
    double lst, az;
    int i_point;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;

    az = PointingData[i_point].az;

    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst, PointingData[i_point].lat, &caz, &cel);
    SetSafeDAz(az, &caz);

    axes_mode.az_mode = AXIS_POSITION;
    axes_mode.az_dest = caz;
    axes_mode.az_vel = 0.0;
    axes_mode.el_mode = AXIS_POSITION;
    axes_mode.el_dest = cel;
    axes_mode.el_vel = 0.0;
}


/**
 * @brief Executes the commanded "cap" scan. This is a circle on the sky.
 * 
 */
static void do_mode_new_cap(void)
{
    double caz, cel, r, x2, y, xw;
    double bottom, left, right;
    double next_left, next_right, az_distance;
    double az, az2, el, el1, el2;
    double daz_dt, del_dt;
    double lst;
    double v_az, t = 1;
    int i_point;
    int new_step = 0;

    static double last_X = 0, last_Y = 0, last_w = 0;
    static double v_el = 0;
    static double targ_el = 0.0;

    // Stuff for the elevation offset
    static int el_dir_last = 0;
    static int n_scan = 0;
    static int el_next_dir = 0.0;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

    /* get raster center and sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst, PointingData[i_point].lat, &caz, &cel);
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y,
                             lst + 1.0, PointingData[i_point].lat, &az2, &el2);

    /* add in elevation dither */
    cel += axes_mode.el_dith;
    el2 += axes_mode.el_dith;

    daz_dt = drem(az2 - caz, 360.0);
    del_dt = el2 - cel;

    SetSafeDAz(az, &caz);

    r = CommandData.pointing_mode.w;
    bottom = cel - r;

    /* If a new command, reset to bottom row */
    if ((CommandData.pointing_mode.X != last_X)
            || (CommandData.pointing_mode.Y != last_Y)
            || (CommandData.pointing_mode.w != last_w)
            || (last_mode != P_CAP)) {
        initialize_el_dither();
        if ((fabs(az - (caz)) < 0.1) && (fabs(el - (bottom)) < 0.05)) {
            last_X = CommandData.pointing_mode.X;
            last_Y = CommandData.pointing_mode.Y;
            last_w = CommandData.pointing_mode.w;
            n_scan = 0;
        } else {
            last_w = 0; // remember we are moving...
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = caz;
            axes_mode.az_vel = 0.0;
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = bottom;
            axes_mode.el_vel = 0.0;
            v_el = 0.0;
            targ_el = -r;
            el_next_dir = 1;
            return;
        }
    }
    /** Get x limits at the next elevation row **/
    y = targ_el; // el - cel + CommandData.pointing_mode.del*el_dir;
    x2 = r * r - y * y;
    if (x2 < 0) {
        xw = 0.0;
    } else {
        xw = sqrt(x2);
    }
    if (xw < MIN_SCAN * 0.5)
        xw = MIN_SCAN * 0.5;
    xw /= cos(el * M_PI / 180.0);
    next_left = caz - xw;
    next_right = caz + xw;

    /** Get x limits at the current elevation **/
    y = el - cel;
    x2 = r * r - y * y;
    if (x2 < 0) {
        xw = 0.0;
    } else {
        xw = sqrt(x2);
    }
    if (xw < MIN_SCAN * 0.5)
        xw = MIN_SCAN * 0.5;
    xw /= cos(el * M_PI / 180.0);
    left = caz - xw;
    right = caz + xw;

    /* set az v */
    v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
    calculate_az_mode_vel(az, left, right, v_az, daz_dt);

    /** set El V **/
    new_step = 0;
    if (az < left) {
        if (axes_mode.az_dir < 0) {
            az_distance = next_right - left;
            t = az_distance / v_az + 2.0 * v_az / (az_accel * SR);
            new_step = 1;
        }
        axes_mode.az_dir = 1;
    } else if (az > right) {
        if (axes_mode.az_dir > 0) {
            az_distance = right - next_left;
            t = az_distance / v_az + 2.0 * v_az / (az_accel * SR);
            new_step = 1;
        }
        axes_mode.az_dir = -1;
    }

    if (new_step) {
        // CommandData.trigger_lo_offset_check = ROACH_TRIGGER_LO_OFFSET_MASK; // Flag to offset the LO
        // set v for this step
        v_el = (targ_el - (el - cel)) / t;
        // set targ_el for the next step
        targ_el += CommandData.pointing_mode.del * el_next_dir;
        axes_mode.el_dir = el_next_dir;
        //    blast_info("Az Step: Next Step targ_el = %f",targ_el);
        if (targ_el >= r) {
            targ_el = r;
            el_next_dir = -1;
            blast_info("Approaching the top: next targ_el = %f, r = %f,"
                        "el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",
                       targ_el, r, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_top = ROACH_TRIGGER_EL_TARG_MASK;
        } else if (targ_el <= -r) {
            targ_el = -r;
            el_next_dir = 1;
            blast_info("Approaching the bottom: next targ_el = %f, -r = %f,"
                        "el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",
                    targ_el, (-1.0) * r, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_bottom = ROACH_TRIGGER_EL_TARG_MASK;
            n_scan += 1;
            if (n_scan != 0) {
                calculate_el_dither(DITH_INC);
                blast_info("We're dithering! El Dither = %f", axes_mode.el_dith);
            }
        }
    }

    el1 = cel + r;
    el2 = cel - r;
    if (el1 > MAX_EL)
        el1 = MAX_EL;
    if (el2 < MIN_EL)
        el2 = MIN_EL;

    /* check for out of range in el */
    if (el > el1 + EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el1;
        // axes_mode.el_dir = -1;
        if (v_el > 0) {
            v_el = -v_el;
        }
        return;
    } else if (el < el2 - EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = el2;
        // axes_mode.el_dir = 1;
        if (v_el < 0) {
            v_el = -v_el;
        }
        return;
    }
    /* else if (el > el1) { */
    /*     axes_mode.el_dir = -1; */
    /*   } else if (el < el2) {  */
    /*     axes_mode.el_dir = 1; */
    /*   }     */

    el_dir_last = axes_mode.el_dir;

    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = v_el + del_dt;
}


/**
 * @brief Executes a "box" scan with stepped azimuth and scanned elevation
 * 
 */
static void do_mode_el_box(void)
{
    double caz, cel, w, h;
    double bottom, top, left, right;
    double az, az2, el, el2;
    double daz_dt, del_dt;
    double lst;
    double v_el, t = 1;
    int i_point;
    int new_step = 0;
    int new = 0;
    int turn_az = 0;
    static int j = 0;

    static double last_X = 0, last_Y = 0, last_w = 0, last_h = 0;
    static double v_az = 0;
    static double targ_az = 0.0;

    // Stuff for dither triggering (right now no dither is implemented)
    static int az_dir_last = 0;
    static int n_scan = 0;
    static int az_next_dir = 0.0;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    v_el = fabs(CommandData.pointing_mode.vel);

    /* get raster center and sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y, lst, PointingData[i_point].lat,
                             &caz, &cel);
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y, lst + 1.0,
                             PointingData[i_point].lat, &az2, &el2);
    /* sky drift terms */
    daz_dt = drem(az2 - caz, 360.0);
    del_dt = el2 - cel;

    SetSafeDAz(az, &caz);

    w = CommandData.pointing_mode.w / cos(el * M_PI / 180.0);
    h = CommandData.pointing_mode.h;
    bottom = cel - h * 0.5;
    top = cel + h * 0.5;
    left = caz - w * 0.5;
    right = caz + w * 0.5;

    if (top > MAX_EL) top = MAX_EL;
    if (bottom < MIN_EL) bottom = MIN_EL;
    new = 0;

    /* If a new command, reset to bottom row */
    if ((CommandData.pointing_mode.X != last_X) || (CommandData.pointing_mode.Y != last_Y)
        || (CommandData.pointing_mode.w != last_w) || (CommandData.pointing_mode.h != last_h)
        || (last_mode != P_EL_BOX)) {
        initialize_el_dither();
        new = 1;
    }
    if (el < bottom - 0.5) new = 1;
    if (el > top + 0.5) new = 1;
    if (az < left - 2.0) new = 1;
    if (az > right + 2.0) new = 1;

    /* If a new command, reset to bottom row */
    if (new) {
        n_scan = 0;
        if ((fabs(az - left) < 0.1) && (fabs(el - bottom) < 0.05)) {
            last_X = CommandData.pointing_mode.X;
            last_Y = CommandData.pointing_mode.Y;
            last_w = CommandData.pointing_mode.w;
            last_h = CommandData.pointing_mode.h;
        } else {
            last_w = 0; // remember we are moving...
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = left;
            axes_mode.az_vel = 0.0;
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = bottom;
            axes_mode.el_vel = 0.0;
            v_az = 0.0;
            targ_az = -w * 0.5;
            az_next_dir = 1;
            return;
        }
    }
    /* set az v */

    v_el = CommandData.pointing_mode.vel;
    calculate_el_mode_vel(el, bottom, top, v_el, del_dt);

    /** set Az V **/
    new_step = 0;
    if (el < bottom) {
        if (axes_mode.el_dir < 0) {
            t = h / v_el + 2.0 * v_el / (EL_ACCEL * SR);
            new_step = 1;
        }
        axes_mode.el_dir = 1;
    } else if (el > top) {
        if (axes_mode.el_dir > 0) {
            t = h / v_el + 2.0 * v_el / (EL_ACCEL * SR);
            new_step = 1;
        }
        axes_mode.el_dir = -1;
    }

    if (new_step) {
        // CommandData.trigger_lo_offset_check = ROACH_TRIGGER_LO_OFFSET_MASK; // Flag to offset the LO
        // set v for this step
        v_az = (targ_az - (az - caz)) / t;
        // set targ_az for the next step
        targ_az += CommandData.pointing_mode.daz * az_next_dir; // This is actually the next target az....
        axes_mode.az_dir = az_next_dir;
        if (targ_az > w * 0.5) { // If the target az for the next step is outside the az box range
            targ_az = w * 0.5;
            az_next_dir = -1;
            blast_info(
                    "Approaching the top %i: next targ_az = %f, h*0.5 = %f,"
                    "az_next_dir = %i,axes_mode.az_dir=%i,  v_az = %f",
                    j, targ_az, h * 0.5, az_next_dir, axes_mode.az_dir, v_az);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_top= ROACH_TRIGGER_EL_TARG_MASK;
        } else if (targ_az < -w * 0.5) {
            targ_az = -w * 0.5;
            az_next_dir = 1;
            blast_info(
                    "Approaching the bottom %i: next targ_az = %f, h*0.5 = %f,"
                    "az_next_dir = %i,axes_mode.az_dir=%i, v_az = %f",
                    j, targ_az, h * 0.5, az_next_dir, axes_mode.az_dir, v_az);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_bottom= ROACH_TRIGGER_EL_TARG_MASK;
        }
    }
    /* check for out of range in az */
    if (az > right + AZ_BORDER) {
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_dest = cel;
        axes_mode.el_vel = 0.0;
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_vel = 0.0;
        blast_info("%i: az_vel=%f", j, axes_mode.az_vel);

        axes_mode.az_dest = left;
        axes_mode.az_dir = -1;
        if (v_az > 0) {
            v_az = -v_az;
        }
        return;
    } else if (az < left - AZ_BORDER) {
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_dest = cel;
        axes_mode.el_vel = 0.0;
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_vel = 0.0;
        axes_mode.az_dest = bottom;
        axes_mode.az_dir = 1;
        if (v_az < 0) {
            v_az = -v_az;
        }
        return;
    }

    az_dir_last = axes_mode.az_dir;

    if (!turn_az) {
        axes_mode.az_mode = AXIS_VEL;
        axes_mode.az_vel = v_az + daz_dt;
    }

    j++;
}


/**
 * @brief Executes a "box" scan with stepped elevation and scanned azimuth
 * 
 */
static void do_mode_new_box(void)
{
    double caz, cel, w, h;
    double bottom, top, left, right;
    double az, az2, el, el2;
    double daz_dt, del_dt;
    double lst;
    double v_az, t = 1;
    int i_point;
    int new_step = 0;
    int new = 0;
    int turn_el = 0;
    static int j = 0;
    static double last_X = 0, last_Y = 0, last_w = 0, last_h = 0;
    static double v_el = 0;
    static double targ_el = 0.0;

    // Stuff for the elevation offset
    static int el_dir_last = 0;
    static int n_scan = 0;
    static int el_next_dir = 0.0;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    v_az = fabs(CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0));

    /* get raster center and sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y, lst, PointingData[i_point].lat,
                             &caz, &cel);
    equatorial_to_horizontal(CommandData.pointing_mode.X, CommandData.pointing_mode.Y, lst + 1.0,
                             PointingData[i_point].lat, &az2, &el2);

    /* add the elevation dither term */
    cel += axes_mode.el_dith;
    el2 += axes_mode.el_dith;

    /* sky drift terms */
    daz_dt = drem(az2 - caz, 360.0);
    del_dt = el2 - cel;

    SetSafeDAz(az, &caz);

    w = CommandData.pointing_mode.w / cos(el * M_PI / 180.0);
    h = CommandData.pointing_mode.h;
    bottom = cel - h * 0.5;
    top = cel + h * 0.5;
    left = caz - w * 0.5;
    right = caz + w * 0.5;
    j++;

    if (top > MAX_EL) top = MAX_EL;
    if (bottom < MIN_EL) bottom = MIN_EL;
    new = 0;

    /* If a new command, reset to bottom row */
    if ((CommandData.pointing_mode.X != last_X) || (CommandData.pointing_mode.Y != last_Y)
        || (CommandData.pointing_mode.w != last_w) || (CommandData.pointing_mode.h != last_h) || (last_mode != P_BOX)) {
        new = 1;
        initialize_el_dither();
    }
    if (el < bottom - 0.5) new = 1;
    if (el > top + 0.5) new = 1;
    if (az < left - 2.0) new = 1;
    if (az > right + 2.0) new = 1;

    /* If a new command, reset to bottom row */
    if (new) {
        n_scan = 0;
        if ((fabs(az - left) < 0.1) && (fabs(el - bottom) < 0.05)) {
            last_X = CommandData.pointing_mode.X;
            last_Y = CommandData.pointing_mode.Y;
            last_w = CommandData.pointing_mode.w;
            last_h = CommandData.pointing_mode.h;
        } else {
            last_w = 0; // remember we are moving...
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = left;
            axes_mode.az_vel = 0.0;
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = bottom;
            axes_mode.el_vel = 0.0;
            v_el = 0.0;
            targ_el = -h * 0.5;
            el_next_dir = 1;
            return;
        }
    }
    /* set az v */
    v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
    calculate_az_mode_vel(az, left, right, v_az, daz_dt);

    /** set El V **/
    new_step = 0;
    if (az < left) {
        if (axes_mode.az_dir < 0) {
            t = w / v_az + 2.0 * v_az / (az_accel * SR);
            new_step = 1;
        }
        axes_mode.az_dir = 1;
    } else if (az > right) {
        if (axes_mode.az_dir > 0) {
            t = w / v_az + 2.0 * v_az / (az_accel * SR);
            new_step = 1;
        }
        axes_mode.az_dir = -1;
    }

    if (new_step) {
        // CommandData.trigger_lo_offset_check = ROACH_TRIGGER_LO_OFFSET_MASK;
//        blast_dbg("Scan Entered snap mode!");
        // set v for this step
        v_el = (targ_el - (el - cel)) / t;
//         set targ_el for the next step
//            blast_info("Az Step:targ_el = %f, el = %f, cel = %f,el-cel = %f, "
//                "el_next_dir = %i,axes_mode.el_dir=%i,  v_el (target)= %f",
//                targ_el,el,cel,el-cel,el_next_dir,axes_mode.el_dir,v_el);
        targ_el += CommandData.pointing_mode.del * el_next_dir; // This is actually the next target el....
        //    blast_info("Az Step: Next Step targ_el = %f",targ_el);
        axes_mode.el_dir = el_next_dir;
        if (targ_el > h * 0.5) { // If the target el for the next step is outside the el box range
            targ_el = h * 0.5;
            el_next_dir = -1;
            blast_info(
                    "Approaching the top: next targ_el = %f, h*0.5 = %f, "
                    "el_next_dir = %i,axes_mode.el_dir=%i,  v_el = %f",
                    targ_el, h * 0.5, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_top = ROACH_TRIGGER_EL_TARG_MASK;
        } else if (targ_el < -h * 0.5) {
            targ_el = -h * 0.5;
            el_next_dir = 1;
            blast_info(
                    "Approaching the bottom: next targ_el = %f, h*0.5 = %f,"
                    "el_next_dir = %i,axes_mode.el_dir=%i, v_el = %f",
                    targ_el, h * 0.5, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_bottom = ROACH_TRIGGER_EL_TARG_MASK;
            n_scan += 1;
            if (n_scan != 0) {
                calculate_el_dither(DITH_INC);
                blast_info("We're dithering! El Dither = %f", axes_mode.el_dith);
            }
        }
    }
    /* check for out of range in el */
    if (el > top + EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = top;
        axes_mode.el_dir = -1;
        if (v_el > 0) {
            v_el = -v_el;
        }
        return;
    } else if (el < bottom - EL_BORDER) {
        axes_mode.az_mode = AXIS_POSITION;
        axes_mode.az_dest = caz;
        axes_mode.az_vel = 0.0;
        axes_mode.el_mode = AXIS_POSITION;
        axes_mode.el_vel = 0.0;
        axes_mode.el_dest = bottom;
        axes_mode.el_dir = 1;
        if (v_el < 0) {
            v_el = -v_el;
        }
        return;
    }

//     if (((axes_mode.el_dir - el_dir_last) == 2) && (CommandData.pointing_mode.nw == 0)) {
//         n_scan += 1;
//         if (n_scan != 0) {
//             calculate_el_dither(DITH_INC);
//             blast_info("We're dithering! El Dither = %f", axes_mode.el_dith);
//         }
//     }

    el_dir_last = axes_mode.el_dir;

    if (!turn_el) {
        axes_mode.el_mode = AXIS_VEL;
        axes_mode.el_vel = v_el + del_dt;
    }
    j++;
}


/**
 * @brief Executes a "quadrilateral" scan based on 4 corners given to MCP
 * 
 */
void do_mode_quad(void) // aka radbox
{
    double bottom, top, left, right, next_left, next_right, az_distance;
    double az, az2, el, el2;
    double daz_dt, del_dt;
    double lst, lat;
    double v_az, t = 1;
    int i, i_point;
    int new_step = 0;
    double c_az[4], c_el[4]; // corner az and corner el
    double az_of_bot;
    int new;

    static double last_ra[4] = { 0, 0, 0, 0 }, last_dec[4] = { 0, 0, 0, 0 };
    static double v_el = 0;
    static double targ_el = 0.0; // targ_el is in degrees from bottom

    // Stuff for the elevation offset
    static int el_dir_last = 0;
    static int n_scan = 0;
    static int el_next_dir = 0.0;

    i_point = GETREADINDEX(point_index);
    lst = PointingData[i_point].lst;
    lat = PointingData[i_point].lat;
    az = PointingData[i_point].az;
    el = PointingData[i_point].el;

    /* convert ra/decs to az/el */
    for (i = 0; i < 4; i++) {
        equatorial_to_horizontal(CommandData.pointing_mode.ra[i], CommandData.pointing_mode.dec[i], lst, lat, c_az + i,
                                 c_el + i);
        *(c_el + i) += axes_mode.el_dith;
    }

    /* get sky drift speed */
    equatorial_to_horizontal(CommandData.pointing_mode.ra[0], CommandData.pointing_mode.dec[0], lst + 1.0, lat, &az2,
                             &el2);

    el2 += axes_mode.el_dith;

    UnwindDiff(az, &az2);

    daz_dt = drem(az2 - c_az[0], 360.0);
    del_dt = el2 - c_el[0];

    radbox_endpoints(c_az, c_el, el, &left, &right, &bottom, &top, &az_of_bot);

    SetSafeDAz(az, &left); // don't cross the sun
    UnwindDiff(left, &right);

    SetSafeDAz(az, &az_of_bot); // correct left

    if (right - left < MIN_SCAN) {
        left = (left + right) / 2.0 - MIN_SCAN / 2.0;
        right = left + MIN_SCAN;
    }

    new = 0;
    if (last_mode != P_QUAD) {
        new = 1;
        v_el = 0.0;
        targ_el = 0.0;
        el_next_dir = 1;
    }

    if (el < bottom - 1.0) new = 1;
    if (el > top + 1.0) new = 1;
    if (az < left - 6.0) new = 1;
    if (az > right + 6.0) new = 1;

    for (i = 0; i < 4; i++) {
        if (CommandData.pointing_mode.ra[i] != last_ra[i]) new = 1;
        if (CommandData.pointing_mode.dec[i] != last_dec[i]) new = 1;
    }

    if (new) {
        initialize_el_dither();
        n_scan = 0;
        if ((fabs(az - az_of_bot) < 0.1) && (fabs(el - bottom) < 0.05)) {
            for (i = 0; i < 4; i++) {
                last_ra[i] = CommandData.pointing_mode.ra[i];
                last_dec[i] = CommandData.pointing_mode.dec[i];
            }
        } else {
            last_dec[0] = -99.9745; // remember it is new....
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = az_of_bot;
            axes_mode.az_vel = 0.0;
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = bottom;
            axes_mode.el_vel = 0.0;
            v_el = 0.0;
            targ_el = 0.0;
            el_next_dir = 1; // First we go up in El then down.
            return;
        }
    }

    if (targ_el < 0) {
        targ_el = 0;
    }
    if (targ_el > top - bottom) {
        targ_el = top - bottom;
    }

    radbox_endpoints(c_az, c_el, targ_el + bottom, &next_left, &next_right, &bottom, &top, &az_of_bot);

    // make next close to this...
    UnwindDiff(left, &next_left);
    UnwindDiff(left, &next_right);

    if (next_right - next_left < MIN_SCAN) {
        next_left = (next_left + next_right) / 2.0 - MIN_SCAN / 2.0;
        next_right = next_left + MIN_SCAN;
    }

    /* set az v */
    v_az = CommandData.pointing_mode.vaz / cos(el * M_PI / 180.0);
    calculate_az_mode_vel(az, left, right, v_az, daz_dt);

    /** set El V **/
    new_step = 0;
    if ((axes_mode.az_vel < 0) && (axes_mode.az_dir >= 0)) { // turn around
        axes_mode.az_dir = -1;
        az_distance = az - next_left;
        if (az_distance < MIN_SCAN) az_distance = MIN_SCAN;
        t = az_distance / v_az + 2.0 * v_az / (az_accel * SR);
        new_step = 1;
    } else if ((axes_mode.az_vel > 0) && (axes_mode.az_dir <= 0)) { // turn around
        axes_mode.az_dir = 1;
        az_distance = next_right - az;
        if (az_distance < MIN_SCAN) az_distance = MIN_SCAN;
        t = az_distance / v_az + 2.0 * v_az / (az_accel * SR);
        new_step = 1;
    }

    if (new_step) {
        // CommandData.trigger_lo_offset_check = ROACH_TRIGGER_LO_OFFSET_MASK; // Flag to offset the LO
        // set v for this step
        v_el = (targ_el + bottom - el) / t;
        // set targ_el for the next step
        targ_el += CommandData.pointing_mode.del * el_next_dir;
        axes_mode.el_dir = el_next_dir;
        if (targ_el > top - bottom) {
            targ_el = top - bottom;
            el_next_dir = -1;
            blast_info(
                    "Approaching the top: next targ_el = %f, top-bottom = %f, "
                    "el_next_dir = %i,axes_mo8de.el_dir=%i,  v_el = %f",
                    targ_el, top - bottom, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_top = ROACH_TRIGGER_EL_TARG_MASK;
        } else if (targ_el < 0) {
            targ_el = 0;
            el_next_dir = 1;
            blast_info(
                    "Approaching the bottom: next targ_el = %f, top-bottom = %f, "
                    "el_next_dir = %i,axes_mode.el_dir=%i,  v_el = %f",
                    targ_el, top - bottom, el_next_dir, axes_mode.el_dir, v_el);
            blast_info("Setting trigger_roach_tuning_check");
            // CommandData.trigger_roach_tuning_check_bottom = ROACH_TRIGGER_EL_TARG_MASK;
            // Calc El Dither for the next raster scan
            n_scan += 1;
            if (n_scan != 0) {
                calculate_el_dither(DITH_INC);
                blast_info("We're dithering! El Dither = %f", axes_mode.el_dith);
            }
        }
    }

// LMF shouldn't be necessary now that I have moved the dither to new_step
//     if (((axes_mode.el_dir - el_dir_last) == 2) && (CommandData.pointing_mode.nw == 0)) {
//         n_scan += 1;
//         if (n_scan != 0) {
//             calculate_el_dither(DITH_INC);
//             blast_info("We're dithering! El Dither = %f", axes_mode.el_dith);
//         }
//     }

    el_dir_last = axes_mode.el_dir;

    axes_mode.el_mode = AXIS_VEL;
    axes_mode.el_vel = v_el + del_dt;
}

/******************************************************************
 *                                                                *
 * Update Axis Modes: Set axes_mode based on                      *
 *    CommandData.pointing_mode                                   *
 *                                                                *
 ******************************************************************/


/**
 * @brief Set axes_mode based on CommandData.pointing_mode structure
 * which is updated by user commands or scheduled commands.
 * 
 */
void update_axes_mode(void)
{
    az_accel = CommandData.az_accel / SR;
    switch (CommandData.pointing_mode.mode) {
        case P_DRIFT:
            axes_mode.el_mode = AXIS_VEL;
            axes_mode.el_vel = CommandData.pointing_mode.del;
            axes_mode.az_mode = AXIS_VEL;
            axes_mode.az_vel = CommandData.pointing_mode.vaz;
            break;
        case P_AZEL_GOTO:
            axes_mode.el_mode = AXIS_POSITION;
            axes_mode.el_dest = CommandData.pointing_mode.Y;
            axes_mode.el_vel = 0.0;
            axes_mode.az_mode = AXIS_POSITION;
            axes_mode.az_dest = CommandData.pointing_mode.X;
            axes_mode.az_vel = 0.0;
            break;
        case P_AZ_SCAN:
            do_az_scan_mode();
            break;
        case P_EL_SCAN:
            do_el_scan_mode();
            break;
        case P_VCAP:
            do_mode_vcap();
            break;
        case P_VBOX:
            do_mode_velocity_box();
            break;
        case P_BOX:
            do_mode_new_box();
            break;
        case P_EL_BOX:
            do_mode_el_box();
            break;
        case P_CAP:
            do_mode_new_cap();
            break;
        case P_RADEC_GOTO:
            do_mode_RAdec_goto();
            break;
        case P_QUAD: // aka radbox
            do_mode_quad();
            break;
        case P_LOCK:
            axes_mode.el_mode = AXIS_LOCK;
            axes_mode.el_dest = CommandData.pointing_mode.Y;
            axes_mode.el_vel = 0.0;
            axes_mode.az_mode = AXIS_VEL;
            axes_mode.az_vel = 0.0;
            break;
        case P_CURRENT:
            axes_mode.el_mode = AXIS_VEL;
            axes_mode.el_vel = 0.0;
            axes_mode.az_mode = AXIS_VEL;
            axes_mode.az_vel = 0.0;
            break;
        default:
            blast_warn("Pointing: Unknown Elevation Pointing Mode %d: " "stopping\n", CommandData.pointing_mode.mode);
            CommandData.pointing_mode.mode = P_DRIFT;
            CommandData.pointing_mode.X = 0;
            CommandData.pointing_mode.Y = 0;
            CommandData.pointing_mode.vaz = 0.0;
            CommandData.pointing_mode.del = 0.0;
            CommandData.pointing_mode.w = 0;
            CommandData.pointing_mode.h = 0;
            axes_mode.el_mode = AXIS_VEL;
            axes_mode.el_vel = 0.0;
            axes_mode.az_mode = AXIS_VEL;
            axes_mode.az_vel = 0.0;
//      isc_pulses[0].is_fast = isc_pulses[1].is_fast = 0;
            break;
    }
    last_mode = CommandData.pointing_mode.mode;
}


/**
 * @brief Calculates the required current to request
 * from the el motor in order to move at the requested
 * velocity in elevation.
 * 
 * @param m_vreq_el required elevation velocity in degrees/s
 * @param m_disabled have we disabled the elevation axis or not
 * @return int16_t current in milliamps
 */
static int16_t calculate_el_current(float m_vreq_el, int m_disabled)
{
    static int first_time = 1;

    static channel_t *error_el_ch = NULL;
    static channel_t *p_el_ch = NULL;
    static channel_t *i_el_ch = NULL;
    static channel_t *d_el_ch = NULL;
    static channel_t *el_integral_ch = NULL;
    static channel_t* frictTermElAddr;
    static channel_t* frictTermUnfiltElAddr;

    float K_p = 0.0;        //!< Proportional gain
    float T_i = 0.0;        //!< Integral time constant
    float T_d = 0.0;        //!< Derivative time constant
    float I_db = 0.0;       //!< I_step deadband current

    float error_pv = 0.0;
    float P_term = 0.0;
    float I_step = 0.0; // intermediate control loop results
    float D_term = 0.0;

    double friction = 0.0;
    static double friction_in[2] = { 0.0 };
    static double friction_out[2] = { 0.0 };

    static float I_term = 0.0;

    static double lpfilter_in[LPFILTER_POLES + 1] = { 0.0 };
    static double lpfilter_out[LPFILTER_POLES + 1] = { 0.0 };
    static float last_pv = 0.0;

    int i_point_read = GETREADINDEX(point_index);

    float pv = ACSData.ifel_gy - PointingData[i_point_read].ifel_earth_gy;

    int16_t milliamp_return;
    static int16_t last_milliamp = 0;

    static const int16_t max_delta_mA = 5; /* Limit current increase to 5 mA/cycle or 10 A/s */

    if (first_time) {
        first_time = 0;

        error_el_ch = channels_find_by_name("error_el");
        p_el_ch = channels_find_by_name("p_term_el");
        i_el_ch = channels_find_by_name("i_term_el");
        d_el_ch = channels_find_by_name("d_term_el");
        el_integral_ch = channels_find_by_name("el_integral_step");
        frictTermElAddr = channels_find_by_name("frict_term_el");
        frictTermUnfiltElAddr = channels_find_by_name("frict_term_uf_el");
    }

    K_p = CommandData.ele_gain.P;
    T_i = CommandData.ele_gain.I;
    T_d = CommandData.ele_gain.D;
    I_db = CommandData.ele_gain.DB;

    /** 
     * The elevation error is the difference between the requested El velocity and
     * that measured by the ACS
     */
    error_pv = m_vreq_el - pv;

    /**
     * The P term is the P gain times the error
     */
    P_term = K_p * error_pv;

    /**
     * The I gain K_i = K_p / T_i where T_i is measured in seconds and therefore is
     * multiplied by the sample rate of the motors.  We implement a "bump-less"
     * transfer here by accumulating the I_term_el.  
     */
    I_step = error_pv * K_p / (T_i * MOTORSR);

    /**
     * Check whether I_step is within the commanded deadband range.
     */

    if (fabsf(I_step) < I_db) I_step = 0;

    if (fabsf(I_step) > MAX_DI_EL) {
        I_step = copysignf(MAX_DI_EL, I_step);
    }

    /**
     * Our integral term exists to remove residual DC offset from the Proportional response,
     * thus we want to exclude the "integral wind-up" phenomenon where the overshoot in current
     * is a result of accumulating excessively large I terms.
     */
     // LMF the NICC should not calculate an integral term because its pointing solution is
     // shared from the ICC and laggy.
    if (InCharge) {
        I_term += I_step;
        if (fabsf(I_term) > MAX_I_EL) {
            I_term = copysignf(MAX_I_EL, I_term);
        }
    } else {
        I_term = 0.0;
    }
    /**
     * The derivative term is calculated based on the change in the process value (our measured speed).
     * This can be excessively noisy, so we implement an IIR lowpass with a corner frequency at 10Hz
     */
    lpfilter_in[0] = lpfilter_in[1];
    lpfilter_in[1] = lpfilter_in[2];
    lpfilter_in[2] = lpfilter_in[3];
    lpfilter_in[3] = lpfilter_in[4];
    lpfilter_in[4] = lpfilter_in[5];
    lpfilter_in[5] = (pv - last_pv) / LPFILTER_GAIN;
    last_pv = pv;

    lpfilter_out[0] = lpfilter_out[1];
    lpfilter_out[1] = lpfilter_out[2];
    lpfilter_out[2] = lpfilter_out[3];
    lpfilter_out[3] = lpfilter_out[4];
    lpfilter_out[4] = lpfilter_out[5];
    lpfilter_out[5] = (lpfilter_in[0] + lpfilter_in[5]) + 5 * (lpfilter_in[1] + lpfilter_in[4])
                      + 10 * (lpfilter_in[2] + lpfilter_in[3]) + (lpfilter_coefs[0] * lpfilter_out[0])
                      + (lpfilter_coefs[1] * lpfilter_out[1]) + (lpfilter_coefs[2] * lpfilter_out[2])
                      + (lpfilter_coefs[3] * lpfilter_out[3]) + (lpfilter_coefs[4] * lpfilter_out[4]);

    D_term = K_p * T_d * MOTORSR * lpfilter_out[LPFILTER_POLES];

    milliamp_return = P_term + I_term + D_term;

    // Calculate static friction offset term
    if (fabs(milliamp_return) < 3) {
        friction = 0.0;
    } else {
        friction = copysign(CommandData.ele_gain.F, milliamp_return);
    }
    /**
     * This is a simple Butterworth low-pass filter with 200Hz input and -3dB frequency
     * at 0.1Hz (637.62, 0.996863).  Additional terms (if we want to adjust:
     * 0.2 Hz (319.31, 0.993737)
     * 0.5 Hz (128.32, 0.984414)
     * 1 Hz (64.6567, 0.969067)
     * 1.5 Hz (43.4335, 0.95395)
     */
    friction_in[0] = friction_in[1];
    friction_in[1] = friction / 637.62;
    friction_out[0] = friction_out[1];
    friction_out[1] = friction_in[0] + friction_in[1] + 0.996863 * friction_out[0];

    milliamp_return += friction_out[1];

    if (milliamp_return > MAX_EL_CURRENT) milliamp_return = MAX_EL_CURRENT;
    if (milliamp_return < MIN_EL_CURRENT) milliamp_return = MIN_EL_CURRENT;

    /**
     * Limit our change in current output to be 5 mA/cycle or 10A/s
     * NB: This needs to be disabled for feedback tuning using standard oscillation
     * methods
     */
    if (milliamp_return > last_milliamp + max_delta_mA) {
        milliamp_return = last_milliamp + max_delta_mA;
    } else if (milliamp_return < last_milliamp - max_delta_mA) {
        milliamp_return = last_milliamp - max_delta_mA;
    }

    if (m_disabled || (CommandData.pin_is_in && !CommandData.force_el)) {
        last_pv = pv;
        I_term = 0.0;
        milliamp_return = 0;
    } else if (CommandData.pointing_mode.mode == P_CURRENT) {
        last_pv = pv;
        I_term = 0.0;
        milliamp_return = CommandData.pointing_mode.w * 100.0;
    }

    SET_FLOAT(error_el_ch, error_pv);
    SET_FLOAT(p_el_ch, P_term);
    SET_FLOAT(i_el_ch, I_term);
    SET_FLOAT(d_el_ch, D_term);
    SET_FLOAT(el_integral_ch, I_step);
    SET_FLOAT(frictTermElAddr, friction_out[1]);
    SET_FLOAT(frictTermUnfiltElAddr, friction);
    last_milliamp = milliamp_return;
    return milliamp_return;
}


/**
 * @brief Calculates the required current to request
 * from the RW motor controller to move at the desired 
 * velocity in azimuth.
 * 
 * @param v_req_az requested azimuth velocity in degrees/s
 * @param m_disabled is the RW disabled or not
 * @return int16_t required current in milliamps
 */
static int16_t calculate_rw_current(float v_req_az, int m_disabled)
{
    static int first_time = 1;

    static channel_t *error_az_ch = NULL;
    static channel_t *p_az_ch = NULL;
    static channel_t *i_az_ch = NULL;
    static channel_t *d_az_ch = NULL;

    int i_point;
    float K_p = 0.0;        //!< Proportional gain
    float T_i = 0.0;        //!< Integral time constant
    float T_d = 0.0;        //!< Derivative time constant

    float error_pv = 0.0;
    float P_term = 0.0;
    float I_step = 0.0; // intermediate control loop results
    float D_term = 0.0;

    static float I_term = 0.0;
    static float lpfilter_in[LPFILTER_POLES+1] = { 0.0 };
    static float lpfilter_out[LPFILTER_POLES+1] = { 0.0 };
    static float last_pv = 0.0;
    float pv;

    int16_t milliamp_return;
    static int16_t last_milliamp = 0;
    static const int16_t max_delta_mA = 5;

    if (first_time) {
        first_time = 0;

        error_az_ch = channels_find_by_name("error_az");
        p_az_ch = channels_find_by_name("p_term_az");
        i_az_ch = channels_find_by_name("i_term_az");
        d_az_ch = channels_find_by_name("d_term_az");
    }

    K_p = CommandData.azi_gain.P;
    T_i = CommandData.azi_gain.I;
    T_d = CommandData.azi_gain.D;

    i_point = GETREADINDEX(point_index);
    pv = PointingData[i_point].gy_az;
    error_pv = pv - v_req_az;

    /**
     * The P term is the P gain times the error
     */
    P_term = K_p*error_pv;

    /**
     * The I gain K_i = K_p / T_i where T_i is measured in seconds and therefore is
     * multiplied by the sample rate of the motors.  We implement a "bump-less"
     * transfer here by accumulating the I_term_el
     */
    I_step = error_pv * K_p / (T_i * MOTORSR);
    if (fabsf(I_step) > MAX_DI) {
        I_step = copysignf(MAX_DI, I_step);
    }
    if (T_i == 0) {
        I_step = 0;
    }

    /**
     * Our integral term exists to remove residual DC offset from the Proportional response,
     * thus we want to exclude the "integral wind-up" phenomenon where the overshoot in current
     * is a result of accumulating excessively large I terms.
     */
    if (InCharge) {
        I_term += I_step;
        if (fabsf(I_term) > MAX_I) {
            I_term = copysignf(MAX_I, I_term);
        }
        if (K_p < 0.1) {
            // reset the integral
            I_term = 0;
        }
    } else {
        I_term = 0;
    }
    /**
     * The derivative term is calculated based on the change in the process value (our measured speed).
     * This can be excessively noisy, so we implement an IIR lowpass with a corner frequency at 10Hz
     */
    lpfilter_in[0] = lpfilter_in[1];
    lpfilter_in[1] = lpfilter_in[2];
    lpfilter_in[2] = lpfilter_in[3];
    lpfilter_in[3] = lpfilter_in[4];
    lpfilter_in[4] = lpfilter_in[5];
    lpfilter_in[5] = (pv - last_pv) / LPFILTER_GAIN;
    last_pv = pv;

    lpfilter_out[0] = lpfilter_out[1];
    lpfilter_out[1] = lpfilter_out[2];
    lpfilter_out[2] = lpfilter_out[3];
    lpfilter_out[3] = lpfilter_out[4];
    lpfilter_out[4] = lpfilter_out[5];
    lpfilter_out[5] =    (lpfilter_in[0] + lpfilter_in[5]) +
                     5 * (lpfilter_in[1] + lpfilter_in[4]) +
                    10 * (lpfilter_in[2] + lpfilter_in[3]) +
                         (lpfilter_coefs[0] * lpfilter_out[0]) +
                         (lpfilter_coefs[1] * lpfilter_out[1]) +
                         (lpfilter_coefs[2] * lpfilter_out[2]) +
                         (lpfilter_coefs[3] * lpfilter_out[3]) +
                         (lpfilter_coefs[4] * lpfilter_out[4]);

    D_term = K_p * T_d * MOTORSR * lpfilter_out[LPFILTER_POLES];

    milliamp_return = P_term + I_term + D_term;

    if (milliamp_return > MAX_RW_CURRENT) milliamp_return = MAX_RW_CURRENT;
    if (milliamp_return < MIN_RW_CURRENT) milliamp_return = MIN_RW_CURRENT;

    /**
     * Limit our change in current output to be 5 mA/cycle or 10A/s
     * NB: This needs to be disabled for feedback tuning using standard oscillation
     * methods
     */
    if (milliamp_return > last_milliamp + max_delta_mA) {
        milliamp_return = last_milliamp + max_delta_mA;
    } else if (milliamp_return < last_milliamp - max_delta_mA) {
        milliamp_return = last_milliamp - max_delta_mA;
    }

    if (m_disabled) {
        last_pv = pv;
        I_term = 0.0;
        milliamp_return = 0;
    } else if (CommandData.pointing_mode.mode == P_CURRENT) {
        last_pv = pv;
        I_term = 0.0;
        milliamp_return = CommandData.pointing_mode.Y * 100.0;
    }
    last_milliamp = milliamp_return;

    SET_FLOAT(error_az_ch, error_pv);
    SET_FLOAT(p_az_ch, P_term);
    SET_FLOAT(i_az_ch, I_term);
    SET_FLOAT(d_az_ch, D_term);
    return milliamp_return;
}


/**
 * @brief Calculate the current required by the pivot given a requested azimuthal
 * velocity.
 * @param m_az_req_vel Requested azimuth scan speed in degrees/second
 * @param m_disabled TRUE/FALSE.  If TRUE, set requested current to 0
 * @return Returns the desired pivot current in 0.01A (units of motor controller)
 */
static double calculate_piv_current(float m_az_req_vel, unsigned int m_disabled)
{
    static channel_t* pRWTermPivAddr;
    static channel_t* IRWTermPivAddr;
    static channel_t* pErrTermPivAddr;
    static channel_t* IErrTermPivAddr;
    static channel_t* frictTermPivAddr;
    static channel_t* frictTermUnfiltPivAddr; // For debugging only.  Remove later!

    double friction = 0.0;
    static double friction_in[2] = {0.0};
    static double friction_out[2] = {0.0};

    double milliamp_return = 0.0;
    static int16_t last_milliamp = 0;
    static const int16_t max_delta_mA = 5;
    int i_point;

    double err_rw;
    double err_vel;
    double I_step, I_step_err;
    double P_rw_term, P_vel_term;
    static double I_term, I_term_err;
    static unsigned int firsttime = 1;

    if (firsttime) {
        pRWTermPivAddr = channels_find_by_name("p_rw_term_piv");
        IRWTermPivAddr = channels_find_by_name("i_rw_term_piv");
        pErrTermPivAddr = channels_find_by_name("p_err_term_piv");
        IErrTermPivAddr = channels_find_by_name("i_err_term_piv");
        frictTermPivAddr = channels_find_by_name("frict_term_piv");
        frictTermUnfiltPivAddr = channels_find_by_name("frict_term_uf_piv");
        firsttime = 0;
    }

    i_point = GETREADINDEX(point_index);

    err_rw = rw_get_velocity_dps() - CommandData.pivot_gain.SP;
    err_vel = m_az_req_vel - PointingData[i_point].gy_az;
    P_rw_term = CommandData.pivot_gain.PV * err_rw;
    P_vel_term = (-1.0)*CommandData.pivot_gain.PE * err_vel;

    /**
     * The I gain K_i = K_p / T_i where T_i is measured in seconds and therefore is
     * multiplied by the sample rate of the motors.  We implement a "bump-less"
     * transfer here by accumulating the I_term_el
     */
    I_step = P_rw_term / (CommandData.pivot_gain.IV * MOTORSR);
    if (fabsf(I_step) > MAX_DI) {
        I_step = copysignf(MAX_DI, I_step);
    }
    if (CommandData.pivot_gain.IV == 0) {
        I_step = 0;
    }
    I_step_err = P_vel_term / (CommandData.pivot_gain.IE * MOTORSR);
    if (fabsf(I_step_err) > MAX_DI) {
        I_step_err = copysignf(MAX_DI, I_step);
    }
    if (CommandData.pivot_gain.IE == 0) {
        I_step_err = 0;
    }

    /**
     * Our integral term exists to remove residual DC offset from the Proportional response,
     * thus we want to exclude the "integral wind-up" phenomenon where the overshoot in current
     * is a result of accumulating excessively large I terms.
     */
    I_term += I_step;
    if (fabsf(I_term) > MAX_I) {
        I_term = copysignf(MAX_I, I_term);
    }
    if (CommandData.pivot_gain.PV < 0.1) {
        // reset the integral
        I_term = 0;
    }

    I_term_err += I_step_err;
    if (fabsf(I_term_err) > MAX_I) {
        I_term_err = copysignf(MAX_I, I_term);
    }
    if (CommandData.pivot_gain.PE < 0.1) {
        // reset the integral
        I_term_err = 0;
    }
    // LMF: If we are not in charge don't calc the I term because our pointing solution comes
    // from the ICC, and is laggy.
    if (!InCharge) {
        I_term_err = 0;
        I_term = 0;
    }
    milliamp_return = P_rw_term + P_vel_term + I_term + I_term_err;

    // Calculate static friction offset term
    if (fabs(milliamp_return) < 3) {
        friction = 0.0;
    } else {
        friction = copysign(CommandData.pivot_gain.F, milliamp_return);
    }
    /**
     * This is a simple Butterworth low-pass filter with 200Hz input and -3dB frequency
     * at 0.1Hz (637.62, 0.996863).  Additional terms (if we want to adjust:
     * 0.2 Hz (319.31, 0.993737)
     * 0.5 Hz (128.32, 0.984414)
     * 1 Hz (64.6567, 0.969067)
     * 1.5 Hz (43.4335, 0.95395)
     */
    friction_in[0] = friction_in[1];
    friction_in[1] = friction / 637.62;
    friction_out[0] = friction_out[1];
    friction_out[1] = friction_in[0] + friction_in[1] + 0.996863 * friction_out[0];

    milliamp_return += friction_out[1];

    if (milliamp_return > MAX_PIV_CURRENT) milliamp_return = MAX_PIV_CURRENT;
    if (milliamp_return < MIN_PIV_CURRENT) milliamp_return = MIN_PIV_CURRENT;

    /**
     * Limit our change in current output to be 5 mA/cycle or 10A/s
     * NB: This needs to be disabled for feedback tuning using standard oscillation
     * methods
     */
    if (milliamp_return > last_milliamp + max_delta_mA) {
        milliamp_return = last_milliamp + max_delta_mA;
    } else if (milliamp_return < last_milliamp - max_delta_mA) {
        milliamp_return = last_milliamp - max_delta_mA;
    }

    if (m_disabled) { // Don't attempt to send current to the motors if we are m_disabled.
        milliamp_return = 0.0;
        friction_in[0] = 0.0;
        friction_in[1] = 0.0;
        friction_out[0] = 0.0;
        friction_out[1] = 0.0;
        I_term = 0;
    } else if (CommandData.pointing_mode.mode == P_CURRENT) {
        I_term = 0.0;
        friction_in[0] = 0.0;
        friction_in[1] = 0.0;
        friction_out[0] = 0.0;
        friction_out[1] = 0.0;
        milliamp_return = CommandData.pointing_mode.X * 100.0;
    }
    last_milliamp = milliamp_return;

    SET_FLOAT(pRWTermPivAddr, P_rw_term);
    SET_FLOAT(IRWTermPivAddr, I_term);
    SET_FLOAT(pErrTermPivAddr, P_vel_term);
    SET_FLOAT(IErrTermPivAddr, I_term_err);
    SET_FLOAT(frictTermPivAddr, friction_out[1]);
    SET_FLOAT(frictTermUnfiltPivAddr, friction);
    return milliamp_return;
}


/**
 * @brief Sets the current (amperes) values for each of the motor controllers based
 * on the requested velocities and current status of the system.  Values are
 * written to the frame and to the motor controller memory maps.
 */
void command_motors(void)
{
    static channel_t* velReqElAddr;
    static channel_t* velReqAzAddr;

    static channel_t* el_current_addr;
    static channel_t* piv_current_addr;
    static channel_t* rw_current_addr;

    float v_req_el = 0.0;
    float v_req_az = 0.0;

    int16_t el_current;
    int16_t rw_current;
    int16_t piv_current;

    /******** Obtain correct indexes the first time here ***********/
    static int firsttime = 1;
    if (firsttime) {
      firsttime = 0;
      velReqElAddr = channels_find_by_name("vel_req_el");
      velReqAzAddr = channels_find_by_name("vel_req_az");

      piv_current_addr = channels_find_by_name("mc_piv_i_cmd");
      el_current_addr = channels_find_by_name("mc_el_i_cmd");
      rw_current_addr = channels_find_by_name("mc_rw_i_cmd");
    }
    /*******************************************************************\
    * Drive the Elevation motor                                         *
    \*******************************************************************/

    v_req_el = GET_FLOAT(velReqElAddr);

    // TODO(seth): limits in dps: revisit these
    if ((v_req_el < -10.0) || (v_req_el > 10.0))
        v_req_el = 0; // no really really crazy values!

    el_current = calculate_el_current(v_req_el, CommandData.disable_el);

    SET_INT16(el_current_addr, el_current);
    el_set_current(el_current);

    /*******************************************************************\
    * Drive the Pivot Motor                                             *
    \*******************************************************************/
    v_req_az = GET_FLOAT(velReqAzAddr);
    piv_current = calculate_piv_current(v_req_az, CommandData.disable_az);

    SET_INT16(piv_current_addr, piv_current);
    piv_set_current(piv_current);

    /*******************************************************************\
    * Drive the Reaction Wheel                                          *
    \*******************************************************************/
    rw_current = calculate_rw_current(v_req_az, CommandData.disable_az);
    SET_INT16(rw_current_addr, rw_current);
    rw_set_current(rw_current);
}

// FUTURE HOME OF ERROR AND STATE MONITORING CHANNEL FUNCTIONS

/**
 * @brief sets up channel pointers to reaction wheel error
 * channels and then when called zeros all of them. This is used
 * in the error code switch statement to clear any previously written
 * errors. We also compare the current error code with the the old one
 * to save on processor cycles if nothing has changed.
 * 
 * @param error_code - current error code reported by the motor controller.
 */
static void zero_all_rw_errors(uint16_t error_code) {
    static int first_time = 1;
    static uint16_t ecode_old = 0;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("rw_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("rw_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("rw_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("rw_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("rw_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("rw_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("rw_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("rw_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("rw_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("rw_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("rw_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("rw_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("rw_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("rw_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("rw_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("rw_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("rw_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("rw_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("rw_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("rw_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("rw_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("rw_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("rw_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("rw_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("rw_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("rw_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("rw_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("rw_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("rw_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("rw_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("rw_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("rw_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("rw_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("rw_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("rw_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("rw_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("rw_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("rw_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("rw_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("rw_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("rw_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("rw_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("rw_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("rw_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("rw_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("rw_mc_e_unknown");
        first_time = 0;
    }
    if (error_code != ecode_old) {
        SET_SCALED_VALUE(ecode_none_Addr , 0);
        SET_SCALED_VALUE(ecode_unspec_Addr , 0);
        SET_SCALED_VALUE(ecode_no_mem_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_state_change_Addr , 0);
        SET_SCALED_VALUE(ecode_unk_req_state_Addr, 0);
        SET_SCALED_VALUE(ecode_no_bootstrap_Addr , 0);
        SET_SCALED_VALUE(ecode_no_firmware_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config2_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_config_Addr , 0);
        SET_SCALED_VALUE(ecode_no_inputs_Addr , 0);
        SET_SCALED_VALUE(ecode_no_outputs_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_error_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_types_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_init_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_preop_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_safeop_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inconsistent_settings_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_3buf_Addr , 0);
        SET_SCALED_VALUE(ecode_background_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_no_in_out_Addr , 0);
        SET_SCALED_VALUE(ecode_fatal_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_no_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_latch_Addr , 0);
        SET_SCALED_VALUE(ecode_pll_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_io_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_eoe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_coe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_foe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_soe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_voe_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_access_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_restart_Addr , 0);
        SET_SCALED_VALUE(ecode_device_id_update_Addr , 0);
        SET_SCALED_VALUE(ecode_app_controller_unavail_Addr , 0);
        SET_SCALED_VALUE(ecode_unknown_Addr , 0);
    }
    ecode_old = error_code;
}


/**
 * @brief sets up channel pointers to elevation drive error
 * channels and then when called zeros all of them. This is used
 * in the error code switch statement to clear any previously written
 * errors. We also compare the current error code with the the old one
 * to save on processor cycles if nothing has changed.
 * 
 * @param error_code - current error code reported by the motor controller.
 */
static void zero_all_el_errors(uint16_t error_code) {
    static int first_time = 1;
    static uint16_t ecode_old = 0;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("el_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("el_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("el_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("el_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("el_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("el_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("el_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("el_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("el_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("el_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("el_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("el_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("el_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("el_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("el_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("el_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("el_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("el_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("el_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("el_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("el_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("el_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("el_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("el_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("el_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("el_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("el_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("el_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("el_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("el_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("el_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("el_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("el_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("el_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("el_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("el_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("el_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("el_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("el_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("el_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("el_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("el_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("el_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("el_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("el_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("el_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("el_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("el_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("el_mc_e_unknown");
        first_time = 0;
    }
    if (error_code != ecode_old) {
        SET_SCALED_VALUE(ecode_none_Addr , 0);
        SET_SCALED_VALUE(ecode_unspec_Addr , 0);
        SET_SCALED_VALUE(ecode_no_mem_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_state_change_Addr , 0);
        SET_SCALED_VALUE(ecode_unk_req_state_Addr, 0);
        SET_SCALED_VALUE(ecode_no_bootstrap_Addr , 0);
        SET_SCALED_VALUE(ecode_no_firmware_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config2_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_config_Addr , 0);
        SET_SCALED_VALUE(ecode_no_inputs_Addr , 0);
        SET_SCALED_VALUE(ecode_no_outputs_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_error_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_types_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_init_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_preop_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_safeop_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inconsistent_settings_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_3buf_Addr , 0);
        SET_SCALED_VALUE(ecode_background_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_no_in_out_Addr , 0);
        SET_SCALED_VALUE(ecode_fatal_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_no_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_latch_Addr , 0);
        SET_SCALED_VALUE(ecode_pll_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_io_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_eoe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_coe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_foe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_soe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_voe_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_access_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_restart_Addr , 0);
        SET_SCALED_VALUE(ecode_device_id_update_Addr , 0);
        SET_SCALED_VALUE(ecode_app_controller_unavail_Addr , 0);
        SET_SCALED_VALUE(ecode_unknown_Addr , 0);
    }
    ecode_old = error_code;
}


/**
 * @brief sets up channel pointers to pivot motor error
 * channels and then when called zeros all of them. This is used
 * in the error code switch statement to clear any previously written
 * errors. We also compare the current error code with the the old one
 * to save on processor cycles if nothing has changed.
 * 
 * @param error_code - current error code reported by the motor controller.
 */
static void zero_all_piv_errors(uint16_t error_code) {
    static int first_time = 1;
    static uint16_t ecode_old = 0;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("piv_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("piv_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("piv_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("piv_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("piv_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("piv_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("piv_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("piv_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("piv_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("piv_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("piv_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("piv_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("piv_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("piv_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("piv_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("piv_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("piv_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("piv_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("piv_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("piv_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("piv_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("piv_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("piv_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("piv_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("piv_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("piv_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("piv_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("piv_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("piv_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("piv_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("piv_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("piv_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("piv_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("piv_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("piv_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("piv_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("piv_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("piv_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("piv_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("piv_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("piv_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("piv_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("piv_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("piv_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("piv_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("piv_mc_e_unknown");
        first_time = 0;
    }
    if (error_code != ecode_old) {
        SET_SCALED_VALUE(ecode_none_Addr , 0);
        SET_SCALED_VALUE(ecode_unspec_Addr , 0);
        SET_SCALED_VALUE(ecode_no_mem_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_state_change_Addr , 0);
        SET_SCALED_VALUE(ecode_unk_req_state_Addr, 0);
        SET_SCALED_VALUE(ecode_no_bootstrap_Addr , 0);
        SET_SCALED_VALUE(ecode_no_firmware_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_mail_config2_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_config_Addr , 0);
        SET_SCALED_VALUE(ecode_no_inputs_Addr , 0);
        SET_SCALED_VALUE(ecode_no_outputs_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_error_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_sync_types_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_config_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_init_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_preop_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_safeop_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_out_map_Addr , 0);
        SET_SCALED_VALUE(ecode_inconsistent_settings_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_sync_unsup_Addr , 0);
        SET_SCALED_VALUE(ecode_freerun_3buf_Addr , 0);
        SET_SCALED_VALUE(ecode_background_watchdog_Addr , 0);
        SET_SCALED_VALUE(ecode_no_in_out_Addr , 0);
        SET_SCALED_VALUE(ecode_fatal_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_no_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_sync_Addr , 0);
        SET_SCALED_VALUE(ecode_inv_dc_latch_Addr , 0);
        SET_SCALED_VALUE(ecode_pll_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_io_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr , 0);
        SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_eoe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_coe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_foe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_soe_Addr , 0);
        SET_SCALED_VALUE(ecode_mailbox_voe_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_access_Addr , 0);
        SET_SCALED_VALUE(ecode_eeprom_Addr , 0);
        SET_SCALED_VALUE(ecode_peripheral_restart_Addr , 0);
        SET_SCALED_VALUE(ecode_device_id_update_Addr , 0);
        SET_SCALED_VALUE(ecode_app_controller_unavail_Addr , 0);
        SET_SCALED_VALUE(ecode_unknown_Addr , 0);
    }
    ecode_old = error_code;
}

/**
 * @brief This function takes the error code reported by the
 * reaction wheel controller and uses a switch statement to
 * determine which error channel should be set high. We also report
 * the full uint16_t error code in a channel should that be useful for
 * debugging by knowledgable users.
 * 
 * @param error_code the error code reported by the motor controller
 * 
 */
static void rw_ecode_reporter(uint16_t error_code) {
    static int first_time = 1;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("rw_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("rw_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("rw_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("rw_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("rw_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("rw_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("rw_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("rw_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("rw_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("rw_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("rw_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("rw_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("rw_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("rw_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("rw_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("rw_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("rw_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("rw_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("rw_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("rw_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("rw_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("rw_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("rw_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("rw_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("rw_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("rw_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("rw_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("rw_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("rw_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("rw_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("rw_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("rw_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("rw_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("rw_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("rw_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("rw_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("rw_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("rw_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("rw_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("rw_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("rw_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("rw_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("rw_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("rw_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("rw_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("rw_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("rw_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("rw_mc_e_unknown");
        first_time = 0;
    }
    zero_all_rw_errors(error_code);
    switch (error_code) {
        case ECODE_NONE:
            SET_SCALED_VALUE(ecode_none_Addr, 1);
            break;
        case ECODE_UNSPEC:
            SET_SCALED_VALUE(ecode_unspec_Addr, 1);
            break;
        case ECODE_NO_MEM:
            SET_SCALED_VALUE(ecode_no_mem_Addr, 1);
            break;
        case ECODE_INVALID_STATE_CHANGE:
            SET_SCALED_VALUE(ecode_inv_state_change_Addr, 1);
            break;
        case ECODE_UNK_REQ_STATE:
            SET_SCALED_VALUE(ecode_unk_req_state_Addr, 1);
            break;
        case ECODE_NO_BOOTSTRAP:
            SET_SCALED_VALUE(ecode_no_bootstrap_Addr, 1);
            break;
        case ECODE_NO_FIRMWARE:
            SET_SCALED_VALUE(ecode_no_firmware_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG:
            SET_SCALED_VALUE(ecode_inv_mail_config_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG2:
            SET_SCALED_VALUE(ecode_inv_mail_config2_Addr, 1);
            break;
        case ECODE_INV_SYNC_CONFIG:
            SET_SCALED_VALUE(ecode_inv_sync_config_Addr, 1);
            break;
        case ECODE_NO_INPUTS:
            SET_SCALED_VALUE(ecode_no_inputs_Addr, 1);
            break;
        case ECODE_NO_OUTPUTS:
            SET_SCALED_VALUE(ecode_no_outputs_Addr, 1);
            break;
        case ECODE_SYNC_ERROR:
            SET_SCALED_VALUE(ecode_sync_error_Addr, 1);
            break;
        case ECODE_SYNC_WATCHDOG:
            SET_SCALED_VALUE(ecode_sync_watchdog_Addr, 1);
            break;
        case ECODE_INV_SYNC_TYPES:
            SET_SCALED_VALUE(ecode_inv_sync_types_Addr, 1);
            break;
        case ECODE_INV_OUT_CONFIG:
            SET_SCALED_VALUE(ecode_inv_out_config_Addr, 1);
            break;
        case ECODE_INV_IN_CONFIG:
            SET_SCALED_VALUE(ecode_inv_in_config_Addr, 1);
            break;
        case ECODE_INV_WATCHDOG_CONFIG:
            SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr, 1);
            break;
        case ECODE_PERIPHERAL_COLD_START:
            SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr, 1);
            break;
        case ECODE_PERIPHERAL_INIT:
            SET_SCALED_VALUE(ecode_peripheral_init_Addr, 1);
            break;
        case ECODE_PERIPHERAL_PREOP:
            SET_SCALED_VALUE(ecode_peripheral_preop_Addr, 1);
            break;
        case ECODE_PERIPHERAL_SAFEOP:
            SET_SCALED_VALUE(ecode_peripheral_safeop_Addr, 1);
            break;
        case ECODE_INV_IN_MAP:
            SET_SCALED_VALUE(ecode_inv_in_map_Addr, 1);
            break;
        case ECODE_INV_OUT_MAP:
            SET_SCALED_VALUE(ecode_inv_out_map_Addr, 1);
            break;
        case ECODE_INCONSISTENT_SETTINGS:
            SET_SCALED_VALUE(ecode_inconsistent_settings_Addr, 1);
            break;
        case ECODE_FREERUN_UNSUP:
            SET_SCALED_VALUE(ecode_freerun_unsup_Addr, 1);
            break;
        case ECODE_SYNC_UNSUP:
            SET_SCALED_VALUE(ecode_sync_unsup_Addr, 1);
            break;
        case ECODE_FREERUN_3BUF:
            SET_SCALED_VALUE(ecode_freerun_3buf_Addr, 1);
            break;
        case ECODE_BACKGROUND_WATCHDOG:
            SET_SCALED_VALUE(ecode_background_watchdog_Addr, 1);
            break;
        case ECODE_NO_IN_OUT:
            SET_SCALED_VALUE(ecode_no_in_out_Addr, 1);
            break;
        case ECODE_FATAL_SYNC:
            SET_SCALED_VALUE(ecode_fatal_sync_Addr, 1);
            break;
        case ECODE_NO_SYNC:
            SET_SCALED_VALUE(ecode_no_sync_Addr, 1);
            break;
        case ECODE_INV_IN_FMMU:
            SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr, 1);
            break;
        case ECODE_INV_DC_SYNC:
            SET_SCALED_VALUE(ecode_inv_dc_sync_Addr, 1);
            break;
        case ECODE_INV_DC_LATCH:
            SET_SCALED_VALUE(ecode_inv_dc_latch_Addr, 1);
            break;
        case ECODE_PLL:
            SET_SCALED_VALUE(ecode_pll_Addr, 1);
            break;
        case ECODE_DC_SYNC_IO:
            SET_SCALED_VALUE(ecode_dc_sync_io_Addr, 1);
            break;
        case ECODE_DC_SYNC_TIMEOUT:
            SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE0:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE1:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr, 1);
            break;
        case ECODE_MAILBOX_EOE:
            SET_SCALED_VALUE(ecode_mailbox_eoe_Addr, 1);
            break;
        case ECODE_MAILBOX_COE:
            SET_SCALED_VALUE(ecode_mailbox_coe_Addr, 1);
            break;
        case ECODE_MAILBOX_FOE:
            SET_SCALED_VALUE(ecode_mailbox_foe_Addr, 1);
            break;
        case ECODE_MAILBOX_SOE:
            SET_SCALED_VALUE(ecode_mailbox_soe_Addr, 1);
            break;
        case ECODE_MAILBOX_VOE:
            SET_SCALED_VALUE(ecode_mailbox_voe_Addr, 1);
            break;
        case ECODE_EEPROM_ACCESS:
            SET_SCALED_VALUE(ecode_eeprom_access_Addr, 1);
            break;
        case ECODE_EEPROM:
            SET_SCALED_VALUE(ecode_eeprom_Addr, 1);
            break;
        case ECODE_PERIPHERAL_RESTART:
            SET_SCALED_VALUE(ecode_peripheral_restart_Addr, 1);
            break;
        case ECODE_DEVICE_ID_UPDATE:
            SET_SCALED_VALUE(ecode_device_id_update_Addr, 1);
            break;
        case ECODE_APP_CONTROLLER_AVAIL:
            SET_SCALED_VALUE(ecode_app_controller_unavail_Addr, 1);
            break;
        case ECODE_UNK:
            SET_SCALED_VALUE(ecode_unknown_Addr, 1);
            break;
        default:
            blast_err("INVALID ECODE MATCH\n");
            break;
    }
}

/**
 * @brief This function takes the error code reported by the
 * elevation drive controller and uses a switch statement to
 * determine which error channel should be set high. We also report
 * the full uint16_t error code in a channel should that be useful for
 * debugging by knowledgable users.
 * 
 * @param error_code the error code reported by the motor controller
 * 
 */
static void el_ecode_reporter(uint16_t error_code) {
    static int first_time = 1;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("el_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("el_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("el_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("el_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("el_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("el_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("el_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("el_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("el_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("el_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("el_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("el_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("el_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("el_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("el_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("el_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("el_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("el_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("el_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("el_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("el_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("el_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("el_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("el_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("el_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("el_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("el_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("el_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("el_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("el_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("el_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("el_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("el_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("el_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("el_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("el_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("el_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("el_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("el_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("el_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("el_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("el_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("el_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("el_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("el_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("el_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("el_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("el_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("el_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("el_mc_e_unknown");
        first_time = 0;
    }
    zero_all_el_errors(error_code);
    switch (error_code) {
        case ECODE_NONE:
            SET_SCALED_VALUE(ecode_none_Addr, 1);
            break;
        case ECODE_UNSPEC:
            SET_SCALED_VALUE(ecode_unspec_Addr, 1);
            break;
        case ECODE_NO_MEM:
            SET_SCALED_VALUE(ecode_no_mem_Addr, 1);
            break;
        case ECODE_INVALID_STATE_CHANGE:
            SET_SCALED_VALUE(ecode_inv_state_change_Addr, 1);
            break;
        case ECODE_UNK_REQ_STATE:
            SET_SCALED_VALUE(ecode_unk_req_state_Addr, 1);
            break;
        case ECODE_NO_BOOTSTRAP:
            SET_SCALED_VALUE(ecode_no_bootstrap_Addr, 1);
            break;
        case ECODE_NO_FIRMWARE:
            SET_SCALED_VALUE(ecode_no_firmware_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG:
            SET_SCALED_VALUE(ecode_inv_mail_config_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG2:
            SET_SCALED_VALUE(ecode_inv_mail_config2_Addr, 1);
            break;
        case ECODE_INV_SYNC_CONFIG:
            SET_SCALED_VALUE(ecode_inv_sync_config_Addr, 1);
            break;
        case ECODE_NO_INPUTS:
            SET_SCALED_VALUE(ecode_no_inputs_Addr, 1);
            break;
        case ECODE_NO_OUTPUTS:
            SET_SCALED_VALUE(ecode_no_outputs_Addr, 1);
            break;
        case ECODE_SYNC_ERROR:
            SET_SCALED_VALUE(ecode_sync_error_Addr, 1);
            break;
        case ECODE_SYNC_WATCHDOG:
            SET_SCALED_VALUE(ecode_sync_watchdog_Addr, 1);
            break;
        case ECODE_INV_SYNC_TYPES:
            SET_SCALED_VALUE(ecode_inv_sync_types_Addr, 1);
            break;
        case ECODE_INV_OUT_CONFIG:
            SET_SCALED_VALUE(ecode_inv_out_config_Addr, 1);
            break;
        case ECODE_INV_IN_CONFIG:
            SET_SCALED_VALUE(ecode_inv_in_config_Addr, 1);
            break;
        case ECODE_INV_WATCHDOG_CONFIG:
            SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr, 1);
            break;
        case ECODE_PERIPHERAL_COLD_START:
            SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr, 1);
            break;
        case ECODE_PERIPHERAL_INIT:
            SET_SCALED_VALUE(ecode_peripheral_init_Addr, 1);
            break;
        case ECODE_PERIPHERAL_PREOP:
            SET_SCALED_VALUE(ecode_peripheral_preop_Addr, 1);
            break;
        case ECODE_PERIPHERAL_SAFEOP:
            SET_SCALED_VALUE(ecode_peripheral_safeop_Addr, 1);
            break;
        case ECODE_INV_IN_MAP:
            SET_SCALED_VALUE(ecode_inv_in_map_Addr, 1);
            break;
        case ECODE_INV_OUT_MAP:
            SET_SCALED_VALUE(ecode_inv_out_map_Addr, 1);
            break;
        case ECODE_INCONSISTENT_SETTINGS:
            SET_SCALED_VALUE(ecode_inconsistent_settings_Addr, 1);
            break;
        case ECODE_FREERUN_UNSUP:
            SET_SCALED_VALUE(ecode_freerun_unsup_Addr, 1);
            break;
        case ECODE_SYNC_UNSUP:
            SET_SCALED_VALUE(ecode_sync_unsup_Addr, 1);
            break;
        case ECODE_FREERUN_3BUF:
            SET_SCALED_VALUE(ecode_freerun_3buf_Addr, 1);
            break;
        case ECODE_BACKGROUND_WATCHDOG:
            SET_SCALED_VALUE(ecode_background_watchdog_Addr, 1);
            break;
        case ECODE_NO_IN_OUT:
            SET_SCALED_VALUE(ecode_no_in_out_Addr, 1);
            break;
        case ECODE_FATAL_SYNC:
            SET_SCALED_VALUE(ecode_fatal_sync_Addr, 1);
            break;
        case ECODE_NO_SYNC:
            SET_SCALED_VALUE(ecode_no_sync_Addr, 1);
            break;
        case ECODE_INV_IN_FMMU:
            SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr, 1);
            break;
        case ECODE_INV_DC_SYNC:
            SET_SCALED_VALUE(ecode_inv_dc_sync_Addr, 1);
            break;
        case ECODE_INV_DC_LATCH:
            SET_SCALED_VALUE(ecode_inv_dc_latch_Addr, 1);
            break;
        case ECODE_PLL:
            SET_SCALED_VALUE(ecode_pll_Addr, 1);
            break;
        case ECODE_DC_SYNC_IO:
            SET_SCALED_VALUE(ecode_dc_sync_io_Addr, 1);
            break;
        case ECODE_DC_SYNC_TIMEOUT:
            SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE0:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE1:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr, 1);
            break;
        case ECODE_MAILBOX_EOE:
            SET_SCALED_VALUE(ecode_mailbox_eoe_Addr, 1);
            break;
        case ECODE_MAILBOX_COE:
            SET_SCALED_VALUE(ecode_mailbox_coe_Addr, 1);
            break;
        case ECODE_MAILBOX_FOE:
            SET_SCALED_VALUE(ecode_mailbox_foe_Addr, 1);
            break;
        case ECODE_MAILBOX_SOE:
            SET_SCALED_VALUE(ecode_mailbox_soe_Addr, 1);
            break;
        case ECODE_MAILBOX_VOE:
            SET_SCALED_VALUE(ecode_mailbox_voe_Addr, 1);
            break;
        case ECODE_EEPROM_ACCESS:
            SET_SCALED_VALUE(ecode_eeprom_access_Addr, 1);
            break;
        case ECODE_EEPROM:
            SET_SCALED_VALUE(ecode_eeprom_Addr, 1);
            break;
        case ECODE_PERIPHERAL_RESTART:
            SET_SCALED_VALUE(ecode_peripheral_restart_Addr, 1);
            break;
        case ECODE_DEVICE_ID_UPDATE:
            SET_SCALED_VALUE(ecode_device_id_update_Addr, 1);
            break;
        case ECODE_APP_CONTROLLER_AVAIL:
            SET_SCALED_VALUE(ecode_app_controller_unavail_Addr, 1);
            break;
        case ECODE_UNK:
            SET_SCALED_VALUE(ecode_unknown_Addr, 1);
            break;
        default:
            blast_err("INVALID ECODE MATCH\n");
            break;
    }
}

/**
 * @brief This function takes the error code reported by the
 * reaction wheel controller and uses a switch statement to
 * determine which error channel should be set high. We also report
 * the full uint16_t error code in a channel should that be useful for
 * debugging by knowledgable users.
 * 
 * @param error_code the error code reported by the motor controller
 * 
 */
static void piv_ecode_reporter(uint16_t error_code) {
    static int first_time = 1;
    static channel_t * ecode_none_Addr;
    static channel_t * ecode_unspec_Addr;
    static channel_t * ecode_no_mem_Addr;
    static channel_t * ecode_inv_state_change_Addr;
    static channel_t * ecode_unk_req_state_Addr;
    static channel_t * ecode_no_bootstrap_Addr;
    static channel_t * ecode_no_firmware_Addr;
    static channel_t * ecode_inv_mail_config_Addr;
    static channel_t * ecode_inv_mail_config2_Addr;
    static channel_t * ecode_inv_sync_config_Addr;
    static channel_t * ecode_no_inputs_Addr;
    static channel_t * ecode_no_outputs_Addr;
    static channel_t * ecode_sync_error_Addr;
    static channel_t * ecode_sync_watchdog_Addr;
    static channel_t * ecode_inv_sync_types_Addr;
    static channel_t * ecode_inv_out_config_Addr;
    static channel_t * ecode_inv_in_config_Addr;
    static channel_t * ecode_inv_watchdog_config_Addr;
    static channel_t * ecode_peripheral_cold_start_Addr;
    static channel_t * ecode_peripheral_init_Addr;
    static channel_t * ecode_peripheral_preop_Addr;
    static channel_t * ecode_peripheral_safeop_Addr;
    static channel_t * ecode_inv_in_map_Addr;
    static channel_t * ecode_inv_out_map_Addr;
    static channel_t * ecode_inconsistent_settings_Addr;
    static channel_t * ecode_freerun_unsup_Addr;
    static channel_t * ecode_sync_unsup_Addr;
    static channel_t * ecode_freerun_3buf_Addr;
    static channel_t * ecode_background_watchdog_Addr;
    static channel_t * ecode_no_in_out_Addr;
    static channel_t * ecode_fatal_sync_Addr;
    static channel_t * ecode_no_sync_Addr;
    static channel_t * ecode_inv_in_fmmu_Addr;
    static channel_t * ecode_inv_dc_sync_Addr;
    static channel_t * ecode_inv_dc_latch_Addr;
    static channel_t * ecode_pll_Addr;
    static channel_t * ecode_dc_sync_io_Addr;
    static channel_t * ecode_dc_sync_timeout_Addr;
    static channel_t * ecode_dc_inv_sync_cycle_Addr;
    static channel_t * ecode_dc_inv_sync_cycle0_Addr;
    static channel_t * ecode_dc_inv_sync_cycle1_Addr;
    static channel_t * ecode_mailbox_eoe_Addr;
    static channel_t * ecode_mailbox_coe_Addr;
    static channel_t * ecode_mailbox_foe_Addr;
    static channel_t * ecode_mailbox_soe_Addr;
    static channel_t * ecode_mailbox_voe_Addr;
    static channel_t * ecode_eeprom_access_Addr;
    static channel_t * ecode_eeprom_Addr;
    static channel_t * ecode_peripheral_restart_Addr;
    static channel_t * ecode_device_id_update_Addr;
    static channel_t * ecode_app_controller_unavail_Addr;
    static channel_t * ecode_unknown_Addr;
    if (first_time) {
        ecode_none_Addr = channels_find_by_name("piv_mc_e_none");
        ecode_unspec_Addr = channels_find_by_name("piv_mc_e_unspecified");
        ecode_no_mem_Addr = channels_find_by_name("piv_mc_e_no_memory");
        ecode_inv_state_change_Addr = channels_find_by_name("piv_mc_e_invalid_state_change");
        ecode_unk_req_state_Addr = channels_find_by_name("piv_mc_e_unknown_requested_state");
        ecode_no_bootstrap_Addr = channels_find_by_name("piv_mc_e_no_bootstrap");
        ecode_no_firmware_Addr = channels_find_by_name("piv_mc_e_no_firmware");
        ecode_inv_mail_config_Addr = channels_find_by_name("piv_mc_e_invalid_mail_config");
        ecode_inv_mail_config2_Addr = channels_find_by_name("piv_mc_e_invalid_mail_config_2");
        ecode_inv_sync_config_Addr = channels_find_by_name("piv_mc_e_invalid_sync_config");
        ecode_no_inputs_Addr = channels_find_by_name("piv_mc_e_no_inputs");
        ecode_no_outputs_Addr = channels_find_by_name("piv_mc_e_no_outputs");
        ecode_sync_error_Addr = channels_find_by_name("piv_mc_e_sync_error");
        ecode_sync_watchdog_Addr = channels_find_by_name("piv_mc_e_sync_watchdog");
        ecode_inv_sync_types_Addr = channels_find_by_name("piv_mc_e_invalid_sync_types");
        ecode_inv_out_config_Addr = channels_find_by_name("piv_mc_e_invalid_out_config");
        ecode_inv_in_config_Addr = channels_find_by_name("piv_mc_e_invalid_in_config");
        ecode_inv_watchdog_config_Addr = channels_find_by_name("piv_mc_e_invalid_watchdog_config");
        ecode_peripheral_cold_start_Addr = channels_find_by_name("piv_mc_e_peripheral_cold_start");
        ecode_peripheral_init_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_init");
        ecode_peripheral_preop_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_preop");
        ecode_peripheral_safeop_Addr = channels_find_by_name("piv_mc_e_peripheral_needs_safeop");
        ecode_inv_in_map_Addr = channels_find_by_name("piv_mc_e_invalid_in_map");
        ecode_inv_out_map_Addr = channels_find_by_name("piv_mc_e_invalid_out_map");
        ecode_inconsistent_settings_Addr = channels_find_by_name("piv_mc_e_inconsistent_settings");
        ecode_freerun_unsup_Addr = channels_find_by_name("piv_mc_e_freerun_unsupported");
        ecode_sync_unsup_Addr = channels_find_by_name("piv_mc_e_sync_unsupported");
        ecode_freerun_3buf_Addr = channels_find_by_name("piv_mc_e_freerun_3buf");
        ecode_background_watchdog_Addr = channels_find_by_name("piv_mc_e_background_watchdog");
        ecode_no_in_out_Addr = channels_find_by_name("piv_mc_e_no_in_out");
        ecode_fatal_sync_Addr = channels_find_by_name("piv_mc_e_fatal_sync");
        ecode_no_sync_Addr = channels_find_by_name("piv_mc_e_no_sync");
        ecode_inv_in_fmmu_Addr = channels_find_by_name("piv_mc_e_invalid_in_fmmu");
        ecode_inv_dc_sync_Addr = channels_find_by_name("piv_mc_e_invalid_dc_sync");
        ecode_inv_dc_latch_Addr = channels_find_by_name("piv_mc_e_invalid_dc_latch");
        ecode_pll_Addr = channels_find_by_name("piv_mc_e_phase_locked_loop");
        ecode_dc_sync_io_Addr = channels_find_by_name("piv_mc_e_dc_sync_io");
        ecode_dc_sync_timeout_Addr = channels_find_by_name("piv_mc_e_dc_sync_timeout");
        ecode_dc_inv_sync_cycle_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle");
        ecode_dc_inv_sync_cycle0_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle0");
        ecode_dc_inv_sync_cycle1_Addr = channels_find_by_name("piv_mc_e_dc_sync_cycle1");
        ecode_mailbox_eoe_Addr = channels_find_by_name("piv_mc_e_mailbox_eoe");
        ecode_mailbox_coe_Addr = channels_find_by_name("piv_mc_e_mailbox_coe");
        ecode_mailbox_foe_Addr = channels_find_by_name("piv_mc_e_mailbox_foe");
        ecode_mailbox_soe_Addr = channels_find_by_name("piv_mc_e_mailbox_soe");
        ecode_mailbox_voe_Addr = channels_find_by_name("piv_mc_e_mailbox_voe");
        ecode_eeprom_access_Addr = channels_find_by_name("piv_mc_e_eeprom_access");
        ecode_eeprom_Addr = channels_find_by_name("piv_mc_e_eeprom");
        ecode_peripheral_restart_Addr = channels_find_by_name("piv_mc_e_peripheral_restart");
        ecode_device_id_update_Addr = channels_find_by_name("piv_mc_e_device_id_update");
        ecode_app_controller_unavail_Addr = channels_find_by_name("piv_mc_e_app_controller_avail");
        ecode_unknown_Addr = channels_find_by_name("piv_mc_e_unknown");
        first_time = 0;
    }
    zero_all_piv_errors(error_code);
    switch (error_code) {
        case ECODE_NONE:
            SET_SCALED_VALUE(ecode_none_Addr, 1);
            break;
        case ECODE_UNSPEC:
            SET_SCALED_VALUE(ecode_unspec_Addr, 1);
            break;
        case ECODE_NO_MEM:
            SET_SCALED_VALUE(ecode_no_mem_Addr, 1);
            break;
        case ECODE_INVALID_STATE_CHANGE:
            SET_SCALED_VALUE(ecode_inv_state_change_Addr, 1);
            break;
        case ECODE_UNK_REQ_STATE:
            SET_SCALED_VALUE(ecode_unk_req_state_Addr, 1);
            break;
        case ECODE_NO_BOOTSTRAP:
            SET_SCALED_VALUE(ecode_no_bootstrap_Addr, 1);
            break;
        case ECODE_NO_FIRMWARE:
            SET_SCALED_VALUE(ecode_no_firmware_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG:
            SET_SCALED_VALUE(ecode_inv_mail_config_Addr, 1);
            break;
        case ECODE_INV_MAIL_CONFIG2:
            SET_SCALED_VALUE(ecode_inv_mail_config2_Addr, 1);
            break;
        case ECODE_INV_SYNC_CONFIG:
            SET_SCALED_VALUE(ecode_inv_sync_config_Addr, 1);
            break;
        case ECODE_NO_INPUTS:
            SET_SCALED_VALUE(ecode_no_inputs_Addr, 1);
            break;
        case ECODE_NO_OUTPUTS:
            SET_SCALED_VALUE(ecode_no_outputs_Addr, 1);
            break;
        case ECODE_SYNC_ERROR:
            SET_SCALED_VALUE(ecode_sync_error_Addr, 1);
            break;
        case ECODE_SYNC_WATCHDOG:
            SET_SCALED_VALUE(ecode_sync_watchdog_Addr, 1);
            break;
        case ECODE_INV_SYNC_TYPES:
            SET_SCALED_VALUE(ecode_inv_sync_types_Addr, 1);
            break;
        case ECODE_INV_OUT_CONFIG:
            SET_SCALED_VALUE(ecode_inv_out_config_Addr, 1);
            break;
        case ECODE_INV_IN_CONFIG:
            SET_SCALED_VALUE(ecode_inv_in_config_Addr, 1);
            break;
        case ECODE_INV_WATCHDOG_CONFIG:
            SET_SCALED_VALUE(ecode_inv_watchdog_config_Addr, 1);
            break;
        case ECODE_PERIPHERAL_COLD_START:
            SET_SCALED_VALUE(ecode_peripheral_cold_start_Addr, 1);
            break;
        case ECODE_PERIPHERAL_INIT:
            SET_SCALED_VALUE(ecode_peripheral_init_Addr, 1);
            break;
        case ECODE_PERIPHERAL_PREOP:
            SET_SCALED_VALUE(ecode_peripheral_preop_Addr, 1);
            break;
        case ECODE_PERIPHERAL_SAFEOP:
            SET_SCALED_VALUE(ecode_peripheral_safeop_Addr, 1);
            break;
        case ECODE_INV_IN_MAP:
            SET_SCALED_VALUE(ecode_inv_in_map_Addr, 1);
            break;
        case ECODE_INV_OUT_MAP:
            SET_SCALED_VALUE(ecode_inv_out_map_Addr, 1);
            break;
        case ECODE_INCONSISTENT_SETTINGS:
            SET_SCALED_VALUE(ecode_inconsistent_settings_Addr, 1);
            break;
        case ECODE_FREERUN_UNSUP:
            SET_SCALED_VALUE(ecode_freerun_unsup_Addr, 1);
            break;
        case ECODE_SYNC_UNSUP:
            SET_SCALED_VALUE(ecode_sync_unsup_Addr, 1);
            break;
        case ECODE_FREERUN_3BUF:
            SET_SCALED_VALUE(ecode_freerun_3buf_Addr, 1);
            break;
        case ECODE_BACKGROUND_WATCHDOG:
            SET_SCALED_VALUE(ecode_background_watchdog_Addr, 1);
            break;
        case ECODE_NO_IN_OUT:
            SET_SCALED_VALUE(ecode_no_in_out_Addr, 1);
            break;
        case ECODE_FATAL_SYNC:
            SET_SCALED_VALUE(ecode_fatal_sync_Addr, 1);
            break;
        case ECODE_NO_SYNC:
            SET_SCALED_VALUE(ecode_no_sync_Addr, 1);
            break;
        case ECODE_INV_IN_FMMU:
            SET_SCALED_VALUE(ecode_inv_in_fmmu_Addr, 1);
            break;
        case ECODE_INV_DC_SYNC:
            SET_SCALED_VALUE(ecode_inv_dc_sync_Addr, 1);
            break;
        case ECODE_INV_DC_LATCH:
            SET_SCALED_VALUE(ecode_inv_dc_latch_Addr, 1);
            break;
        case ECODE_PLL:
            SET_SCALED_VALUE(ecode_pll_Addr, 1);
            break;
        case ECODE_DC_SYNC_IO:
            SET_SCALED_VALUE(ecode_dc_sync_io_Addr, 1);
            break;
        case ECODE_DC_SYNC_TIMEOUT:
            SET_SCALED_VALUE(ecode_dc_sync_timeout_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE0:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle0_Addr, 1);
            break;
        case ECODE_DC_INV_SYNC_CYCLE1:
            SET_SCALED_VALUE(ecode_dc_inv_sync_cycle1_Addr, 1);
            break;
        case ECODE_MAILBOX_EOE:
            SET_SCALED_VALUE(ecode_mailbox_eoe_Addr, 1);
            break;
        case ECODE_MAILBOX_COE:
            SET_SCALED_VALUE(ecode_mailbox_coe_Addr, 1);
            break;
        case ECODE_MAILBOX_FOE:
            SET_SCALED_VALUE(ecode_mailbox_foe_Addr, 1);
            break;
        case ECODE_MAILBOX_SOE:
            SET_SCALED_VALUE(ecode_mailbox_soe_Addr, 1);
            break;
        case ECODE_MAILBOX_VOE:
            SET_SCALED_VALUE(ecode_mailbox_voe_Addr, 1);
            break;
        case ECODE_EEPROM_ACCESS:
            SET_SCALED_VALUE(ecode_eeprom_access_Addr, 1);
            break;
        case ECODE_EEPROM:
            SET_SCALED_VALUE(ecode_eeprom_Addr, 1);
            break;
        case ECODE_PERIPHERAL_RESTART:
            SET_SCALED_VALUE(ecode_peripheral_restart_Addr, 1);
            break;
        case ECODE_DEVICE_ID_UPDATE:
            SET_SCALED_VALUE(ecode_device_id_update_Addr, 1);
            break;
        case ECODE_APP_CONTROLLER_AVAIL:
            SET_SCALED_VALUE(ecode_app_controller_unavail_Addr, 1);
            break;
        case ECODE_UNK:
            SET_SCALED_VALUE(ecode_unknown_Addr, 1);
            break;
        default:
            blast_err("INVALID ECODE MATCH\n");
            break;
    }
}

/**
 * @brief sets up the motor controller telemetry channels
 * for the states of the motors and updates them from the local structs.
 * 
 */
static void motor_controller_state_telemetry(void) {
    static int first_time = 1;
    static channel_t * rw_mc_state_Addr;
    static channel_t * el_mc_state_Addr;
    static channel_t * piv_mc_state_Addr;
    if (first_time) {
        rw_mc_state_Addr = channels_find_by_name("state_rw_mc");
        el_mc_state_Addr = channels_find_by_name("state_el_mc");
        piv_mc_state_Addr = channels_find_by_name("state_piv_mc");
    }
    // modulo 16 here gets rid of the first 12 bits of the 16 bit integer ALstate
    // the actual state is of type uint4 but reported as part of a uint16 quantity
    // we could do bitshifts as well but since this quantity is on the end
    // % 16 does the same thing in a more visible manner.
    uint16_t rw_val = RWMotorData[GETREADINDEX(motor_index)].ALstate % 16;
    uint16_t el_val = ElevMotorData[GETREADINDEX(motor_index)].ALstate % 16;
    uint16_t piv_val = PivotMotorData[GETREADINDEX(motor_index)].ALstate % 16;
    SET_UINT16(rw_mc_state_Addr, rw_val);
    SET_UINT16(el_mc_state_Addr, el_val);
    SET_UINT16(piv_mc_state_Addr, piv_val);
}

void record_motor_status_1hz(void) {
    if (InCharge) {
        motor_controller_state_telemetry();
        rw_ecode_reporter(RWMotorData[GETREADINDEX(motor_index)].ALstatuscode);
        el_ecode_reporter(ElevMotorData[GETREADINDEX(motor_index)].ALstatuscode);
        piv_ecode_reporter(PivotMotorData[GETREADINDEX(motor_index)].ALstatuscode);
    }
}
