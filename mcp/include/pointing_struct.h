/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2004 University of Toronto
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

/********************************************************************/
/*
                    Global data structures:
    Each of the structures below (except PointingDataStruct)
    is used by a thread to collect raw pointing data from some
    sensor or set of sensors.
    -The data are to be written to the downlinkframe (as needed)
     in tx.c::StoreData()
    -The data are to be read and collected into PointingData in pointing.c
     unless the raw data is actually what is wanted...
    -Where the data are written and read in different threads, a 3 way
     buffer is used.
    -r_index = GETREADINDEX(wr_index) can be used to get the readindex.
     it should be valid for atomic reads for at least 100ms.
    -index = INC_INDEX(index) can be used to increment the write index.
*/

#ifndef INCLUDE_POINTING_STRUCT_H
#define INCLUDE_POINTING_STRUCT_H

#include <math.h>
#include <time.h>
#include <stdbool.h>

// number of PSS modules we have
// Ian 8-3-24 we have 4 PSS for the test flight so we should disable some of them
#define NUM_PSS 4
// number of voltages on each PSS (4 sensors/chip)
#define NUM_PSS_V 4

// number of magnetometers
#define NUM_MAGS 2
// number of inclinometers
#define NUM_INCS 2

/**********************************************/
/** @brief  ACSDataStruct                     */
/*  Purpose: Store raw pointing info          */
/*   Source: main thread; GetACS()            */
/*     Used: Main thread;                     */
/*  Does not need to be a circular buffer...  */
struct ACSDataStruct {
  double mag_x[NUM_MAGS];     // counts;
  double mag_y[NUM_MAGS];     // counts;
  double mag_z[NUM_MAGS];     // counts;
  double inc_x[NUM_INCS];
  double inc_y[NUM_INCS];
  double inc_temp[NUM_INCS];
  double inc_ok[NUM_INCS]; // good status and inside valid range
  double pss_i[NUM_PSS][NUM_PSS_V]; // pss voltage
  double enc_motor_elev;  // degrees
  double clin_elev[NUM_INCS]; // degrees
  double ifel_gy;   // deg/s
  double ifyaw_gy;  // deg/s
  double ifroll_gy; // deg/s
  time_t t;
};

extern struct ACSDataStruct ACSData;




/**********************************************/
/** @brief  PointingDataStruct                */
/*  Purpose: Store derived pointing info      */
/*   Source: main thread; pointing.c          */
/*     Used: Main thread; VSC thread          */
struct PointingDataStruct {
  double az;        // degrees
  double el;        // degrees
  double az_sigma;  // degrees
  double el_sigma;  // degrees
  double el_noenc;  // degrees
  double ra;        // hours, apparent
  double dec;       // degrees, apparent
  double offset_ifel_gy;
  double offset_ifyaw_gy;
  double offset_ifroll_gy;
  double ifel_earth_gy;
  double ifyaw_earth_gy;
  double ifroll_earth_gy;
  double gy_roll_amp;
  double gy_az;
  double gy_el;
  double gy_total_vel;
  double gy_total_accel;

  double lat;       // degrees
  double lon;       // degrees

  int longitude_octave_since_launch;
  double alt;       // m
  int at_float;
  time_t t;
  time_t lst;
  time_t unix_lsd;  // local sidereal date in seconds

  bool mag_ok[NUM_MAGS];   // flag
  double mag_az[NUM_MAGS];   // degrees
  double mag_az_raw[NUM_MAGS];   // degrees
  double mag_el[NUM_MAGS];   // degrees
  double mag_el_raw[NUM_MAGS];   // degrees
  double mag_model_dec[NUM_MAGS]; // degrees
  double mag_model_dip[NUM_MAGS]; // degrees
  double mag_sigma[NUM_MAGS]; // degrees
  double mag_strength[NUM_MAGS]; // nanoTesla
  double offset_ifrollmag_gy[NUM_MAGS];
  double offset_ifyawmag_gy[NUM_MAGS];
  double offset_ifrolldgps_gy;
  double offset_ifyawdgps_gy;
  double offset_ifelmotenc_gy;
  double offset_ifelclin_gy[NUM_INCS];
  double dgps_az_raw;   // degrees
  double dgps_az;   // degrees
  double dgps_sigma;   // degrees
  double null_az; // degrees
  double null_el; // degrees

  double sun_az; // degrees current calculated az of sun
  double sun_el; // degrees current calculated el of sun

  int pss_ok;
  int dgps_ok;
  double pss_az;
  double pss_el; // not used, as far as I can tell -PAW 2018/12/23 //Makes sense - JSB 2022/2/24

  // solutions for individual sensors, from PSSConvert
  double pss_azraw[NUM_PSS]; // degrees
  double pss_elraw[NUM_PSS]; // degrees
  // weighted solution for the entire array, from PSSConvert
  double pss_array_azraw, pss_array_elraw;
  double pss_snr[NUM_PSS];
  double pss_sigma;
  double offset_ifrollpss_gy;
  double offset_ifyawpss_gy;

  double xsc_az[2];
  double xsc_el[2];
  double xsc_var[2];
  double xsc_sigma[2];
  double offset_ifel_gy_xsc[2];
  double offset_ifyaw_gy_xsc[2];
  double offset_ifroll_gy_xsc[2];
  double estimated_xsc_az_deg[2]; // these come from the full pointing solution, not the individual star camera solution
  double estimated_xsc_el_deg[2];
  double estimated_xsc_ra_hours[2];
  double estimated_xsc_dec_deg[2];

  bool enc_motor_ok;   // flag
  double enc_motor_el;
  double enc_motor_sigma;

  double clin_ok[NUM_INCS];
  double clin_el[NUM_INCS];
  double clin_el_lut[NUM_INCS];
  double clin_sigma[NUM_INCS];
  uint8_t recv_shared_data;   // flag

  bool requested_el_out_of_bounds;
  bool az_destination_capped;

// TODO(laura): These next fields are just for debugging.  Remove from mcp before flight!
  double new_offset_ifel_elmotenc_gy;
  double int_ifel_elmotenc;
  double new_offset_ifyaw_mag1_gy;
  double new_offset_ifroll_mag1_gy;
  double d_az_mag1;
  double int_ifroll_mag1;
  double int_ifyaw_mag1;
  double new_offset_ifyaw_mag2_gy;
  double new_offset_ifroll_mag2_gy;
  double d_az_mag2;
  double int_ifroll_mag2;
  double int_ifyaw_mag2;
  double new_offset_ifel_xsc0_gy;
  double new_offset_ifyaw_xsc0_gy;
  double new_offset_ifroll_xsc0_gy;
  double int_ifel_xsc0;
  double int_ifyaw_xsc0;
  double int_ifroll_xsc0;
  double d_az_xsc0;
  double prev_sol_az_xsc0;
  double prev_sol_el_xsc0;
  double new_offset_ifel_xsc1_gy;
  double new_offset_ifyaw_xsc1_gy;
  double new_offset_ifroll_xsc1_gy;
  double int_ifel_xsc1;
  double int_ifyaw_xsc1;
  double int_ifroll_xsc1;
  double d_az_xsc1;
  double prev_sol_az_xsc1;
  double prev_sol_el_xsc1;
  double autotrim_rate_xsc;
  uint8_t fresh;
  double new_az;
  double new_el;
  double weight_az;
  double weight_el;
};

extern struct PointingDataStruct PointingData[3];
extern int point_index;

/**********************************************/
/** @brief  DGPS Attittude struct             */
/*  Purpose: Store dgps attitude info         */
/*   Source: dgps thread: csbf_dgps.c         */
/*     Used: Main thread;                     */
struct DGPSAttStruct {
  double az;
  int att_ok;
};

/**********************************************/
/** @brief  DGPS Position  struct             */
/*  Purpose: Store dgps position info         */
/*   Source: dgps thread: dgps.c              */
/*     Used: Main thread;                     */
struct DGPSPosStruct {
  double lat; //
  double lon; //
  double alt; //
  double speed; //
  double direction; //
  double climb; //
  int n_sat;  //
};

/**
 * @brief structure that keeps track of the mode that each movement axis is in
 * 
 */
struct AxesModeStruct {
  int az_mode;
  int el_mode;
  int el_dir;
  int az_dir;
  double az_dest;
  double el_dest;
  double az_vel;
  double el_vel;
  double el_dith;
  unsigned int i_dith;
};

// extern time_t csbf_gps_time;

// deprecated potentially
typedef struct XSCLastTriggerState
{
    int counter_mcp;                        // mcp counter at the time of last trigger
    int counter_stars;                      // stars counter at the time of last trigger
    double lat;
    time_t lst;
    int trigger_time;                       // Time of the last trigger, measured in loops through xsc_control_triggers
    bool forced_grace_period;
    bool forced_trigger_threshold;
    uint32_t timestamp_s;
    uint32_t timestamp_us;
} xsc_last_trigger_state_t;


// deprecated potentially
typedef struct XSCPointingState {
    struct XSCLastTriggerState last_trigger;
    int counter_mcp;                        // the current counter_mcp, passed to the star camera after some delay
    int last_counter_mcp;                   // the previous counter_mcp passed to the star camera
    int last_solution_stars_counter;        // stars counter of last solution used in pointing solution
    unsigned int stars_response_counter;
    double az;                              // XSC Az
    double el;                              // XSC El
    int last_trigger_time;
    int exposure_time_cs;
    double predicted_streaking_px;
} xsc_pointing_state_t;

extern struct XSCPointingState xsc_pointing_state[2];

// enum to map elevation modes to integers
typedef enum
{
    EL_DRIVE,
    EL_INHIBIT
} elevation_pointing_state_enabled_t;

extern bool scan_entered_snap_mode;
extern bool scan_leaving_snap_mode;

#endif
