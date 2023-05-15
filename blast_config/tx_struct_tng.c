/* 
 * tx_struct_tng.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University,  Sacramento
 *  (C) 2014-2016 University of Pennsylvania
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 5, 2014 by seth
 */

#include <limits.h>

#include "channels_tng.h"
#include "calibrate.h"
#include "conversions.h"
/* Analog channel calibrations */
/* 16-bit channels with analog preamps. To Volts */
#define CAL16(m, b) ((m)*M_16PRE),               ((b) + B_16PRE*(m)*M_16PRE)
/* bare thermistor. To Volts. Use LUT for temperature conversion */
#define CAL16T(m, b) ((m)*M_16T), ((b) + B_16T*(m)*M_16T)
/* AD590 conversion. To Celsius */
#define CAL_AD590(m, b) ((m)*M_16_AD590),        ((b)+B_16_AD590*(m)*M_16_AD590-273.15)

#define U_NONE  "",           ""
#define U_T_C   "Temperature", "^oC"
#define U_T_K   "Temperature", "K"
#define U_P_PSI   "Pressure", "PSI"
#define U_V_DPS "Rate",       "^o/s"
#define U_V_MPS "Speed",      "m/s"
#define U_V_KPH "Speed",      "km/hr"
#define U_ALT_M "Altitude",   "m"
#define U_P_DEG "Position",   "^o"
#define U_LA_DEG "Latitude",  "^o"
#define U_LO_DEG "Longitude", "^o"
#define U_D_DEG "Direction",  "^o"
#define U_V_V "Voltage",      "V"
#define U_I_A   "Current",    "A"
#define U_T_MS  "Time",       "ms"
#define U_T_S  "Time",       "s"
#define U_T_MIN "Time",       "min"
#define U_R_O   "Resistance", "Ohms"
#define U_RATE "Rate",        "bps"
#define U_GB  "",             "GB"
#define U_TRIM_DEG "Trim",    "^o"
#define U_TRIM_MM "Trim",     "mm"

// #define NO_KIDS_TEST

#define SCALE(_type)  _type ## _M, _type ## _B
// TODO(seth): Unify the _M, _B scale factor offset terms in a single location

channel_t channel_list[] =
{
    // { "labjack_conn_status",  1, 0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    // below are the thermistor channels as well as the current loop channels
    // { "thermistor_1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    // { "current_eth_switch",      SCALE(CURLOOP_D), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    // if we ever use a chopped source this could be useful for ground testing.
    // { "chopper",              CAL16(1.0, 0.0),          TYPE_UINT16, RATE_100HZ, U_V_V, 0 },


    /* Easy but time-consuming to disentangle, comments are light but possible*/
    { "x_vel_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "x_stp_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "x_str_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "y_lim_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "y_stp_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "y_str_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "x_lim_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "y_vel_stage",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    /*---------------------------------------------------------------------------------------*/
    /*                                         Motors                                        */
    /*---------------------------------------------------------------------------------------*/

    // EtherCat Status Info
    { "n_found_ec",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "slave_count_ec",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "status_ec",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "status_ec_rw",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "status_ec_piv",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "status_ec_el",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },

    // Overall MC Az/El variables (mixed pivot+rw and el)
    { "mc_cmd_status", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "mode_az_mc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "mode_el_mc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dest_az_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dest_el_mc",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "vel_az_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "vel_el_mc",            1. / 6000,        0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "dir_az_mc",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "dir_el_mc",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "g_pt_az",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_pt_el",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "accel_az",             2.0 / 65536,      0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Reaction wheel
    /* Calculated P/I and Error (diff btw commanded/actual velocity) terms from control loop */
    { "p_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "i_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "d_term_az",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "error_az",             SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    /* Motor Controller Temperatures */
    { "t_mc_rw",              SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_T_C, 0 },
    /* Motor Controller State and Status Registers */
    { "status_rw",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "state_rw",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },
    { "set_rw",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_DPS, 0 },
    /* Velocity control loop commanded P/I terms */
    { "g_p_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_i_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_d_az",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "vel_req_az",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    // commanded and read currents
    { "mc_rw_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    { "mc_rw_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    // Reaction wheel reported motor parameters
    { "mc_rw_vel",           RW_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, U_V_DPS, 0 },
    { "mc_rw_pos",           RW_ENCODER_SCALING,        0.0, TYPE_INT32, RATE_100HZ, U_D_DEG, 0 },
    { "mc_phase_mode_rw",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "mc_phase_rw",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    // Motor controller global info Reaction wheel
    {"control_word_read_rw", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },
    {"control_word_write_rw", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },
    {"latched_fault_rw",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    {"network_status_rw",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },
    {"network_problem_rw",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },

    // Pivot
    /* Calculated P/I and Error (diff btw commanded/actual velocity) terms from control loop */
    { "p_rw_term_piv",        SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "i_rw_term_piv",        SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "p_err_term_piv",       SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "i_err_term_piv",       SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    /* Motor Controller Temperatures */
    { "t_mc_piv",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_T_C, 0 },
    /* Motor Controller State and Status Registers */
    { "status_piv",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    { "state_piv",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // rest of channels
    { "frict_off_piv",        SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "frict_term_piv",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "frict_term_uf_piv",    SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    /* Velocity control loop commanded P/I terms */
    { "g_pe_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_pv_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_iv_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_ie_piv",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    // commanded and read currents
    { "mc_piv_i_cmd",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    { "mc_piv_i_read",          1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    // Pivot reported motor parameters
    { "mc_piv_vel",          PIV_RESOLVER_SCALING * 0.1, 0.0, TYPE_INT32, RATE_100HZ, U_V_DPS, 0 },
    { "mc_piv_pos",          PIV_RESOLVER_SCALING,      0.0, TYPE_INT32, RATE_100HZ, U_D_DEG, 0 },
    { "mc_phase_piv",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "mc_phase_mode_piv",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // Motor controller global info Pivot
    {"control_word_read_piv", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"control_word_write_piv", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"latched_fault_piv",    SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"network_status_piv",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"network_problem_piv",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Elevation drive
    /* Calculated P/I and Error (diff btw commanded/actual velocity) terms from control loop */
    { "p_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "i_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "d_term_el",            0.01,             0.0,    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "error_el",             SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "el_integral_step",     SCALE(CONVERT_UNITY),    TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "i_dith_el",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "dith_el",              0.5 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "trim_motor_enc",       I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    /* Motor Controller Temperatures */
    { "t_mc_el",              SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_T_C, 0 },
    /* Motor Controller State and Status Registers */
    { "status_el",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    { "state_el",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // rest of channels
    { "frict_off_el",        SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "frict_term_el",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "frict_term_uf_el",    SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_200HZ, U_NONE, 0 },
    { "vel_el_p",             I2VEL,            0.0, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    { "ok_motor_enc",        SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    /* Velocity control loop commanded P/I terms */
    { "g_p_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_i_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_d_el",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "g_db_el",              SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "vel_req_el",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    // commanded and read currents
    { "mc_el_i_cmd",            1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    { "mc_el_i_read",           1.0/100.0,  0.0, TYPE_INT16, RATE_200HZ, U_NONE, 0 },
    // Elevation drive reported motor parameters
    { "mc_el_vel",           EL_MOTOR_ENCODER_SCALING * 0.1,  0.0, TYPE_INT32, RATE_100HZ, U_V_DPS, 0 },
    { "mc_el_pos",           EL_LOAD_ENCODER_SCALING,   0.0, TYPE_INT32, RATE_100HZ, U_D_DEG, 0 },
    { "mc_el_motor_pos",     EL_MOTOR_ENCODER_SCALING,  0.0, TYPE_INT32, RATE_100HZ, U_D_DEG, 0 },
    { "mc_phase_el",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "mc_phase_mode_el",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // Motor controller global info El
    {"control_word_read_el", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"control_word_write_el", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"latched_fault_el",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"network_status_el",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    {"network_problem_el",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // Elevation motor encoder
    { "el_raw_enc",           I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, U_P_DEG, 0 },
    { "el_motor_enc",         I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, U_P_DEG, 0 },
    { "sigma_motor_enc",      I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, U_NONE, 0 },


    /*---------------------------------------------------------------------------------------*/
    /*                                        Actuators                                      */
    /*---------------------------------------------------------------------------------------*/

    // ActBus generic info
    { "mode_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "flags_act",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_move_act",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_hold_act",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "vel_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "acc_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "bus_reset_act",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dr_0_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "dr_1_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "dr_2_act",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "tol_act",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "status_actbus",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Many of these are deprecated but need more careful extraction

    // DEPRECATED helium 4 cryogenic valve channels
    {"enc_potvalve",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0},
    {"pos_potvalve",            SCALE(CONVERT_UNITY), TYPE_INT32, RATE_5HZ, U_NONE, 0},
    {"state_potvalve",          SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"vel_potvalve",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0},
    {"i_open_potvalve",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"i_close_potvalve",        SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"i_hold_potvalve",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"thresh_clos_potvalve",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},
    {"threshlclos_potvalve",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},
    {"thresh_open_potvalve",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},
    {"tight_move_potvalve",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},
    {"enable_potvalve",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},

    // Let's remove this as well and use our own coding
    {"lims_vent_A_valve",        SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"pos_vent_A_valve",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_5HZ, U_NONE, 0},
    {"lims_vent_B_valve",        SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"pos_vent_B_valve",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_5HZ, U_NONE, 0},
    {"vel_valves",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},
    {"i_move_valves",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"i_hold_valves",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0},
    {"acc_valves",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0},

    // balance motor information channels
    // These are (mostly?) valid to keep and use
    { "vel_bal",             SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    { "acc_bal",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_move_bal",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_hold_bal",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "pos_bal",          SCALE(CONVERT_UNITY), TYPE_INT32, RATE_5HZ, U_NONE, 0 },
    { "lim_bal",         SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "i_level_on_bal",       CUR15_M,     CUR15_B, TYPE_UINT16, RATE_5HZ, U_I_A, 0 },
    { "i_level_off_bal",      CUR15_M,     CUR15_B, TYPE_UINT16, RATE_5HZ, U_I_A, 0 },
    { "i_el_req_avg_bal",     CUR15_M,     CUR15_B, TYPE_UINT16, RATE_5HZ, U_I_A, 0 },
    { "status_bal",           SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },

    // Lock pin
    { "pot_lock",             -100.0 / 16068.0, 1636800.0 / 16068.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "state_lock",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "goal_lock",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "seized_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "pos_lock",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "pin_in_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_move_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_hold_lock",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "vel_lock",             100.,             0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "acc_lock",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Secondary mirror focusing actuators (triplet)
    { "pos_0_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "pos_1_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "pos_2_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "enc_0_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "enc_1_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "enc_2_act",            SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "goal_0_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "goal_1_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "goal_2_act",           SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "offset_0_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "offset_1_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "offset_2_act",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Cryo shutter
    { "steps_shutter",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "steps_slow_shutter",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "pos_shutter",          SCALE(CONVERT_UNITY), TYPE_INT32, RATE_5HZ, U_NONE, 0 },
    { "lims_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_hold_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "i_move_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "vel_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "acc_shutter",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },


    /*---------------------------------------------------------------------------------------*/
    /*                                      Other subsystem                                  */
    /*---------------------------------------------------------------------------------------*/

    // Power LabJacks

    // Outer Frame PBOB
    { "current_fc1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_fc2",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_of_eth",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_motor_box_lj",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    // free at the moment
    { "current_unassigned",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_of_inclinometer",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_magnetometers",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_gondola_thermometry",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_gps_ntp",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_pss_box",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },

    // Voltage monitoring
    { "outer_frame_vm_1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "outer_frame_vm_2",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "outer_frame_vm_3",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },


    // Inner Frame PBOB
    { "current_sc1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_cryo_digital",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_gyros",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_rfsoc",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_if_eth",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_steppers",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_if_inclinometer",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_sc2",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "current_cryo_analog",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    // perhaps the last resort valve
    { "current_last_resort",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },

    // Voltage monitoring
    { "inner_frame_vm_1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "inner_frame_vm_2",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "inner_frame_vm_3",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },


    // Charge controllers

    //  Charge controller 1
    { "v_batt_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_arr_cc1",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "i_batt_cc1",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },
    { "i_arr_cc1",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },
    { "t_hs_cc1",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "fault_cc1",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "alarm_cc1",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "v_targ_cc1",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "state_cc1",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "led_cc1",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    //  Charge controller 2
    { "v_batt_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_arr_cc2",            1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "i_batt_cc2",           1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },
    { "i_arr_cc2",            1 / 400.0,        -32000.0 / 400.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },
    { "t_hs_cc2",             SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "fault_cc2",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "alarm_cc2",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "v_targ_cc2",           1 / 180.0,        -32400.0 / 180.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "state_cc2",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "led_cc2",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },


    /*---------------------------------------------------------------------------------------*/
    /*                                     Pointing Subsystems                               */
    /*---------------------------------------------------------------------------------------*/

    // Magnetometers
    // magnetometer 1
    { "ok_mag1",             SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "az_mag1",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_D_DEG, 0 },
    { "az_raw_mag1",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_D_DEG, 0 },
    { "pitch_mag1",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "sigma_mag1",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "declination_mag1",      I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "dip_mag1",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "cal_xmax_mag1",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_xmin_mag1",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_ymax_mag1",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_ymin_mag1",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_align_mag1",        M_16UNI*180.0,          0.0, TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    { "x_mag1_n",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "y_mag1_n",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "z_mag1_n",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "status_mag1_n",           SCALE(CONVERT_UNITY),   TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "err_count_mag1_n",        SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "timeout_count_mag1_n",    SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "reset_count_mag1_n",      SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "trim_mag1",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // magnetometer 2
    { "ok_mag2",             SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "az_mag2",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_D_DEG, 0 },
    { "az_raw_mag2",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_D_DEG, 0 },
    { "pitch_mag2",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "sigma_mag2",            I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "declination_mag2",      I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "dip_mag2",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_D_DEG, 0 },
    { "cal_xmax_mag2",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_xmin_mag2",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_ymax_mag2",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_ymin_mag2",         M_32UNI*20.0,          0.0, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    { "cal_align_mag2",        M_16UNI*180.0,          0.0, TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    { "x_mag2_s",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "y_mag2_s",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "z_mag2_s",                M_16MAG,              0,  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "status_mag2_s",           SCALE(CONVERT_UNITY),   TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "err_count_mag2_s",        SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "timeout_count_mag2_s",    SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "reset_count_mag2_s",      SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "trim_mag2",             I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // Inclinometers
    // Juzz will need to set an OK for the elevation clinometers here
    { "ok_elclin",            SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    // mandatory error term for pointing
    { "sigma_clin",           I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // Elevation solution
    { "el_clin",              I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "el_lut_clin",          I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // trim/offset
    { "trim_clin",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // new JUZZ inclinometer stuff
    { "x_inc1_n",                SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_100HZ, U_NONE, 0 },
    { "y_inc1_n",                SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "z_inc1_n",                SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "status_inc1_n",           SCALE(CONVERT_UNITY),   TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "err_count_inc1_n",        SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "timeout_count_inc1_n",    SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "reset_count_inc1_n",      SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "x_inc2_s",                SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "y_inc2_s",                SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "z_inc2_s",                SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "status_inc2_s",           SCALE(CONVERT_UNITY),   TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    { "err_count_inc2_s",        SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "timeout_count_inc2_s",    SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "reset_count_inc2_s",      SCALE(CONVERT_UNITY),   TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    // GPS channels
    // DGPS stuff (CSBF?)
    { "ok_dgps",             SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "lat_dgps",                I2DEG,              0,  TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    { "lon_dgps",                I2DEG,              0,  TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    { "alt_dgps",                I2DEG,              0,  TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    { "num_sat_dgps",            SCALE(CONVERT_UNITY),   TYPE_INT8, RATE_1HZ, U_NONE, 0 },
    { "quality_dgps",            SCALE(CONVERT_UNITY),   TYPE_INT8, RATE_1HZ, U_NONE, 0 },
    { "az_raw_dgps",             I2DEG,              0,  TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "az_dgps",                 I2DEG,              0,  TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "sigma_dgps",              I2DEG,              0,  TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "trim_dgps",               SCALE(CONVERT_UNITY),  TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // Gyroscopes
    // Status channels
    { "fault_gy",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    { "mask_gy",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },

    // Total solution from the gyros
    { "gy_az_vel",            SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_DPS, 0 },
    { "gy_el_vel",            SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_DPS, 0 },
    { "gy_total_vel",         SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_DPS, 0 },
    { "gy_total_accel",       SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_DPS, 0 },
    { "ifel_gy",              SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, U_V_DPS, 0 },
    { "ifroll_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, U_V_DPS, 0 },
    { "ifyaw_gy",             SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_100HZ, U_V_DPS, 0 },

    // Gyro Earth rotation
    { "ifel_earth_gy",        0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "ifroll_earth_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "ifyaw_earth_gy",       0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },

    // Gyro offsets
    { "offset_ifel_gy",       0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "offset_ifroll_gy",     0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "offset_ifyaw_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },

    // Gyro offsets for pointing subsystems
    // star cameras
    { "offset_ifelxsc0_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "offset_ifelxsc1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "offset_ifrollxsc0_gy",    0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "new_offset_ifelxsc0_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifyawxsc0_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifrollxsc0_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "offset_ifrollxsc1_gy",    0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "offset_ifyawxsc0_gy",     0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "offset_ifyawxsc1_gy",     0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "new_offset_ifelxsc1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifyawxsc1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifrollxsc1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    // El motor encoder
    { "new_offset_ifelmotorenc_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "offset_ifelmotorenc_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    // Inclinometer
    { "offset_ifelclin_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    // Magnetometer
    { "offset_ifrollmag1_gy",  0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "offset_ifyawmag1_gy",   0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "new_offset_ifrollmag1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifyawmag1_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "offset_ifrollmag2_gy",  0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "offset_ifyawmag2_gy",   0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "new_offset_ifrollmag2_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "new_offset_ifyawmag2_gy",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    // Differential GPS
    { "offset_ifrolldgps_gy",  0.1 / 32768.0,    0.0, TYPE_INT16, RATE_1HZ, U_V_DPS, 0 },
    { "offset_ifyawdgps_gy",   0.1 / 32768.0,    0.0, TYPE_INT16, RATE_1HZ, U_V_DPS, 0 },
    // Pinhole sun sensors
    { "offset_ifrollpss_gy",  0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },
    { "offset_ifyawpss_gy",   0.1 / 32768.0,    0.0, TYPE_INT16, RATE_5HZ, U_V_DPS, 0 },

    // Gyro 1
    { "ifyaw_1_gy",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_yaw_1_gy",  SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },
    { "ifel_1_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_el_1_gy",   SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },
    { "ifroll_1_gy",          SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_roll_1_gy", SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },

    // Gyro 2
    { "ifyaw_2_gy",           SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_yaw_2_gy",  SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },
    { "ifel_2_gy",            SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_el_2_gy",   SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },
    { "ifroll_2_gy",          SCALE(CONVERT_UNITY),            TYPE_FLOAT, RATE_200HZ, U_V_DPS, 0 },
    { "good_pktcnt_roll_2_gy", SCALE(CONVERT_UNITY),           TYPE_UINT32, RATE_200HZ, U_NONE, 0 },

    // Pinhole Sun Sensors
    // Misc PSS channels
    { "ok_pss",               SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "sigma_pss",            I2DEG,              0.0,   TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "cal_imin_pss",         SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "az_raw_pss",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_pss",               SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "trim_pss",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "noise_pss",            SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },

    // Spherical trig calibration
    { "cal_el_pss1",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_el_pss2",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_el_pss3",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_el_pss4",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_el_pss5",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_el_pss6",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss1",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss2",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss3",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss4",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss5",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_roll_pss6",        SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss_array",     SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss1",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss2",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss3",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss4",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss5",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_az_pss6",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_DEG, 0 },
    { "cal_d_pss1",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },
    { "cal_d_pss2",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },
    { "cal_d_pss3",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },
    { "cal_d_pss4",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },
    { "cal_d_pss5",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },
    { "cal_d_pss6",           SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_TRIM_MM, 0 },

    // Noise and attitude channels
    { "snr_pss1",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "snr_pss2",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "snr_pss3",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "snr_pss4",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "snr_pss5",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "snr_pss6",             SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_NONE, 0 },
    { "az_raw_pss1",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_raw_pss2",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_raw_pss3",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_raw_pss4",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_raw_pss5",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "az_raw_pss6",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss1",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss2",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss3",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss4",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss5",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },
    { "el_raw_pss6",          SCALE(CONVERT_UNITY),  TYPE_FLOAT, RATE_5HZ, U_P_DEG, 0 },

    // Raw input signals from all PSS, 7/8 died on TNG but we can use all 8
    // v1-4 for each PSS are the voltages from the PSD, v5 is the voltage from the thermistor
    { "v1_1_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_1_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_1_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_1_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_1_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_2_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_2_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_2_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_2_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_2_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_3_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_3_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_3_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_3_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_3_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_4_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_4_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_4_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_4_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_4_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_5_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_5_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_5_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_5_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_5_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_6_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_6_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_6_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_6_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_6_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    /* { "v1_7_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_7_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_7_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_7_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_7_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v1_8_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v2_8_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v3_8_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v4_8_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 },
    { "v5_8_pss",             SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_5HZ, U_V_V, 0 }, */

    /*---------------------------------------------------------------------------------------*/
    /*                                       Star Cameras                                    */
    /*---------------------------------------------------------------------------------------*/

    // New star camera channels for TIM MCP

    // Blob-finding parameters
    // SC1
    {"sc1_spike_limit",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_dynamic_hp",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_r_smooth",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_high_pass_filter",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_r_high_pass_filter",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_search_border",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_filter_return",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_n_sigma",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_1HZ, U_NONE, 0 },
    {"sc1_unique_star_spacing",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_make_static_hp_mask",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_use_static_hp_mask",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    // SC2
    {"sc2_spike_limit",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_dynamic_hp",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_r_smooth",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_high_pass_filter",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_r_high_pass_filter",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_search_border",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_filter_return",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_n_sigma",               SCALE(CONVERT_UNITY), TYPE_FLOAT, RATE_1HZ, U_NONE, 0 },
    {"sc2_unique_star_spacing",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_make_static_hp_mask",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_use_static_hp_mask",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },

    // Astrometry parameters
    // SC1
    // for solving
    {"sc1_time_limit",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_logodds",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    // for ALTAZ
    {"sc1_latitude",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_longitude",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_hm",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    // SC2
    // for solving
    {"sc2_time_limit",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_logodds",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    // for ALTAZ
    {"sc2_latitude",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_longitude",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_hm",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },

    // Camera Parameters
    // SC1
    {"sc1_focus_position",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_focus_inf",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_aperture_steps",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_max_aperture",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_min_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_max_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_current_aperture",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_exposure_time",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_focus_mode",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_start_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_end_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_focus_step",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc1_photos_per_focus",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    // SC1
    {"sc2_focus_position",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_focus_inf",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_aperture_steps",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_max_aperture",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_min_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_max_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_current_aperture",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_exposure_time",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_focus_mode",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_start_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_end_focus_pos",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_focus_step",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },
    {"sc2_photos_per_focus",               SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_NONE, 0 },


    // Image solution information
    // SC1
    {"sc1_rawtime",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_ra",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_dec",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_image_rms",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_fr",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_ps",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_ir",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_alt",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc1_az",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    // SC2
    {"sc2_rawtime",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_ra",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_dec",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_image_rms",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_fr",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_ps",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_ir",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_alt",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },
    {"sc2_az",               SCALE(CONVERT_UNITY), TYPE_DOUBLE, RATE_1HZ, U_NONE, 0 },


    // XSC CHANNELS BELOW
    // some of this is deprecated - worthy of it's own pass
    // note its probably ALL deprecated, but the "used" ones need to be replaced
    // with actual new code not just removed.

    // Ian is keeping these in small groups to show what have counterparts

    // did we ask to trigger the SCs
    { "trigger_xsc",          SCALE(CONVERT_UNITY),            TYPE_UINT8, RATE_100HZ, U_NONE, 0 },

    // points and CD robust
    {"x0_point_az_raw", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_point_az", SCALE(CONVERT_WIDE_ANGLE_DEG), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x0_point_el_raw", SCALE(CONVERT_ANGLE_DEG), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_point_el", SCALE(CONVERT_WIDE_ANGLE_DEG), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x0_point_var", SCALE(CONVERT_WIDE_ANGLE_DEG), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x0_point_sigma", SCALE(CONVERT_WIDE_ANGLE_DEG), TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x0_point_az_trim", SCALE(CONVERT_ANGLE), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_point_el_trim", SCALE(CONVERT_ANGLE), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_cd_robust_mode", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    {"prev_soln_az_xsc0",      360.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    {"prev_soln_el_xsc0",      360.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },

    {"x1_point_az_raw", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_point_az", CONVERT_WIDE_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x1_point_el_raw", CONVERT_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_point_el", CONVERT_WIDE_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x1_point_var", CONVERT_WIDE_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x1_point_sigma", CONVERT_WIDE_ANGLE_DEG_M, CONVERT_ANGLE_DEG_B, TYPE_UINT32, RATE_5HZ, U_NONE, 0 },
    {"x1_point_az_trim", CONVERT_ANGLE_M, CONVERT_ANGLE_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_point_el_trim", CONVERT_ANGLE_M, CONVERT_ANGLE_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_cd_robust_mode", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    {"prev_soln_az_xsc1",      360.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    {"prev_soln_el_xsc1",      360.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },

    // heaters
    {"x0_heater", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    {"x1_heater", SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },

    // MISC grouping 1 (streaking and blobs?)
    {"x0_predicted_streaking_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x0_last_trig_ctr_stars", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_image_blobn_x", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x0_image_blobn_y", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x0_image_blobn_flux", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x0_image_blobn_peak_to_flux", SCALE(CONVERT_0_TO_10), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x0_image_num_blobs_found",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_num_blobs_matched",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    {"x1_predicted_streaking_px", CONVERT_VEL_M, CONVERT_VEL_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x1_last_trig_ctr_stars", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x1_image_blobn_x", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x1_image_blobn_y", CONVERT_BLOB_POS_M, CONVERT_BLOB_POS_B, TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x1_image_blobn_flux", SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x1_image_blobn_peak_to_flux", SCALE(CONVERT_0_TO_10), TYPE_UINT16, RATE_200HZ, U_NONE, 0 },
    {"x1_image_num_blobs_found",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_num_blobs_matched",   SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    // Housekeeping
    {"x0_hk_temp_lens",            CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x0_hk_temp_comp",            CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x0_hk_temp_plate",           CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x0_hk_temp_flange",          CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x0_hk_pressure",             CONVERT_PRES_M,  CONVERT_PRES_B,  TYPE_UINT16, RATE_1HZ, U_P_PSI, 0 },
    {"x0_hk_disk",                 CONVERT_GB_M,    CONVERT_GB_B,    TYPE_UINT16, RATE_1HZ, U_GB, 0 },

    {"x1_hk_temp_lens",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x1_hk_temp_comp",    CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x1_hk_temp_plate",   CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x1_hk_temp_flange",  CONVERT_TEMP_M,  CONVERT_TEMP_B,  TYPE_UINT16, RATE_1HZ, U_T_C, 0 },
    {"x1_hk_pressure",     CONVERT_PRES_M,  CONVERT_PRES_B,  TYPE_UINT16, RATE_1HZ, U_P_PSI, 0 },
    {"x1_hk_disk",         CONVERT_GB_M,    CONVERT_GB_B,    TYPE_UINT16, RATE_1HZ, U_GB, 0 },

    // Image data (exposure and low level info)
    {"x0_image_num_exposures",     SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    {"x0_image_stats_mean",        CONVERT_STATS_DEPTH_M, CONVERT_STATS_DEPTH_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_stats_noise",       CONVERT_STATS_4000_M, CONVERT_STATS_4000_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_stats_gaindb",      CONVERT_STATS_4000_M, CONVERT_STATS_4000_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_stats_num_px_sat",  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_stats_frac_px_sat", 2.0/(NARROW_MAX-1.0), -1.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_afocus_metric",     CONVERT_STATS_DEPTH_M, CONVERT_STATS_DEPTH_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    {"x1_image_num_exposures",     SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_1HZ, U_NONE, 0 },
    {"x1_image_stats_mean",        CONVERT_STATS_DEPTH_M, CONVERT_STATS_DEPTH_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_stats_noise",       CONVERT_STATS_4000_M, CONVERT_STATS_4000_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_stats_gaindb",      CONVERT_STATS_4000_M, CONVERT_STATS_4000_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_stats_num_px_sat",  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_stats_frac_px_sat", 2.0/(NARROW_MAX-1.0), -1.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_afocus_metric",     CONVERT_STATS_DEPTH_M, CONVERT_STATS_DEPTH_B, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    // Image data (STARS program stuff and lens?)
    {"x0_ctr_stars",               SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_image_ctr_stars",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_image_ctr_mcp",           SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_stars_run_time",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_T_S, 0 },
    {"x0_cam_gain_db",             CONVERT_GAIN_M, CONVERT_GAIN_B, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    {"x0_lens_focus",              SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_lens_aperture",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },

    {"x1_ctr_stars",               SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x1_image_ctr_stars",         SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x1_image_ctr_mcp",           SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x1_stars_run_time",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_T_S, 0 },
    {"x1_cam_gain_db",             CONVERT_GAIN_M, CONVERT_GAIN_B, TYPE_INT32, RATE_1HZ, U_NONE, 0 },
    {"x1_lens_focus",              SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_lens_aperture",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },

    // validity and iplate metrics
    {"x0_image_eq_valid",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_cam_gain_valid",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_valid",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_afocus_metric_valid", SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_iplate",         9.7e-5/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_iplate",        9.7e-5/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    {"x1_image_eq_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_cam_gain_valid",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_valid",         SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_afocus_metric_valid", SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_iplate",         9.7e-5/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_iplate",        9.7e-5/NARROW_MAX, 0.0, TYPE_UINT16, RATE_1HZ, U_NONE, 0 },

    // Image positioning solution and variance(sigma)
    {"x0_image_eq_ra",             SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_dec",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_roll",           SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_sigma_ra",       SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_sigma_dec",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_sigma_roll",     SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_eq_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_az",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_el",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_roll",          SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_sigma_az",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_sigma_el",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_sigma_roll",    SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x0_image_hor_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },

    {"x1_image_eq_ra",             SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_dec",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_roll",           SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_sigma_ra",       SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_sigma_dec",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_sigma_roll",     SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_eq_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_az",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_el",            SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_roll",          SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_sigma_az",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_sigma_el",      SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_sigma_roll",    SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    {"x1_image_hor_sigma_pointing", SCALE(CONVERT_WIDE_ANGLE), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },

    // 1-off channels (no counterparts for XSC1 because we only need 1 set of these)
    // non-response items, set by MCP it seems
    {"x0_ctr_mcp", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_last_trig_age_cs", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_last_trig_ctr_mcp", SCALE(CONVERT_UNITY), TYPE_INT32, RATE_200HZ, U_NONE, 0 },
    {"x0_last_trig_lat", LI2DEG, 0.0, TYPE_UINT32, RATE_200HZ, U_NONE, 0 },
    {"x0_last_trig_lst", LI2SEC*SEC2HR, 0.0, TYPE_UINT32, RATE_200HZ, U_NONE, 0 },

    // end of XSC stuff

    /*---------------------------------------------------------------------------------------*/
    /*                                        Pointing                                       */
    /*---------------------------------------------------------------------------------------*/

    // pointing mode channels
    // SIP provided data channels
    { "alt_sip",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "lat_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_LA_DEG, 0 },
    { "lon_sip",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_LO_DEG, 0 },
    { "time_sip",             SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },

    // Gondola location and pointing channels
    { "alt",                  SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "lat",                  LI2DEG*2.0,       0.0, TYPE_INT32, RATE_100HZ, U_NONE, 0 },
    { "lon",                  LI2DEG*2.0,       0.0, TYPE_INT32, RATE_100HZ, U_NONE, 0 },
    { "ra",                   LI2H,             0.0,        TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "dec",                  LI2DEG,           0.0, TYPE_INT32, RATE_100HZ, U_NONE, 0 },
    { "az",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, U_P_DEG, 0 },
    { "el",                   LI2DEG,   0.0,            TYPE_UINT32, RATE_200HZ, U_P_DEG, 0 },
    { "el_null",              I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, U_D_DEG, 0 },
    { "az_null",              I2DEG,            0.0, TYPE_UINT16, RATE_100HZ, U_D_DEG, 0 },
    { "az_sun",               I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_D_DEG, 0 },
    { "el_sun",               I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "lst_sched",            LI2DEG,           0.0, TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "lst",                  LI2H,             0.0, TYPE_UINT32, RATE_100HZ, U_NONE, 0 },

    // ACS pointing data from sensors
    { "d_az_mag1",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "d_az_mag2",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "d_az_xsc0",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "d_az_xsc1",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifelmotorenc",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifyawmag1",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifrollmag1",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifyawmag2",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifrollmag2",      0.1 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifrollxsc0",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifyawxsc0",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifelxsc0",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifrollxsc1",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifyawxsc1",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },
    { "int_ifelxsc1",      100.0 / 32768.0,    0.0, TYPE_INT16, RATE_100HZ, U_NONE, 0 },

    // pointing solution generating channels
    { "thresh_cmd_atrim",         10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "time_cmd_atrim",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "rate_cmd_atrim",           30.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "rate_atrim",           30.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "thresh_atrim",           10.0 / 65536.0,   0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "fresh_trim",           SCALE(CONVERT_UNITY), TYPE_UINT8, RATE_5HZ, U_NONE, 0 },
    { "new_az",               720.0 / 65536.0,  0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "new_el",               720.0 / 65536.0,  0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "weight_az",               360.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "weight_el",               360.0 / 65536.0,  0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // null az and el trims from commands
    { "trim_null",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "trim_el_null",            I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // slew veto, unsure of meaning
    { "slew_veto",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "sveto_len",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Scan parameters
    // so many RA/DEC since we use 4 for quad boxes
    { "ra_1_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dec_1_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "ra_2_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dec_2_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "ra_3_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dec_3_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    { "ra_4_p",               I2H,              0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "dec_4_p",              I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    // More scan parameters
    // mode        X     Y    vaz   del    w    h
    // LOCK              el
    // AZEL_GOTO   az    el
    // AZ_SCAN     az    el   vaz
    // DRIFT                  vaz   vel
    // RADEC_GOTO  ra    dec
    // VCAP        ra    dec  vaz   vel    r
    // CAP         ra    dec  vaz   elstep r
    // BOX         ra    dec  vaz   elstep w    h

    // this is one of the above pointing modes in the left column
    { "mode_p",               1, 0.0,           TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // x and y are targets on the sky or in telescope coordinates
    { "x_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "y_p",                  I2DEG,            0.0, TYPE_INT16, RATE_5HZ, U_NONE, 0 },
    // w and h are scan parameters for box or cap scans (box uses width and height, cap uses a radius [w])
    { "w_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "h_p",                  I2DEG,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // our azimuthal velocity
    { "vel_az_p",             I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // we only use cap and box along with quad so this is an elevation step size
    { "del_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // may be deprecated?
    { "daz_p",                I2VEL,            0.0, TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    // ditheering parameters for scans
    { "n_dith_p",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "next_i_dith_p",        SCALE(CONVERT_UNITY), TYPE_INT16, RATE_5HZ, U_NONE, 0 },

    /*---------------------------------------------------------------------------------------*/
    /*                                        Detectors                                      */
    /*---------------------------------------------------------------------------------------*/

    // Fields to be addded here

    /*---------------------------------------------------------------------------------------*/
    /*                                      Housekeeping                                     */
    /*---------------------------------------------------------------------------------------*/

    // Gondola thermometry
    { "thermistor_1",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_2",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_3",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_4",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_5",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_6",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_7",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_8",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_9",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_10",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_11",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_12",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_13",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_14",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_15",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_16",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_17",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_18",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_19",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_20",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_21",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_22",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_23",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_24",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_25",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_26",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_27",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_28",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_29",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_30",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_31",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_32",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_33",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_34",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_35",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_36",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_37",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_38",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_39",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_40",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_41",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_42",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_43",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_44",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_45",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_46",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_47",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_48",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_49",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_50",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_51",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_52",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_53",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_54",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_55",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_56",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_57",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_58",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_59",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_60",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_61",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_62",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_63",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_64",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_65",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_66",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_67",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_68",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_69",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_70",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_71",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_72",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_73",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_74",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_75",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_76",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_77",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_78",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_79",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_80",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_81",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_82",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_83",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "thermistor_84",      SCALE(LABJACK), TYPE_UINT16, RATE_1HZ, U_V_V, 0 },

    /*---------------------------------------------------------------------------------------*/
    /*                                        MCP core                                       */
    /*---------------------------------------------------------------------------------------*/

    // MCP core information
    { "time",                 SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "time_usec",            SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "parts_sched",          SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_100HZ, U_NONE, 0 },
    { "status_mcc",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "upslot_sched",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "veto_sensor",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

    // Telemetry
    // Plover is the plugh multi command integer test channel
    { "plover",               SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "bits_vtx",             SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "rate_highrate",        SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_RATE, 0 },
    { "rate_biphase",         SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_RATE, 0 },
    { "mpsse_clock_speed",    SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_RATE, 0 },
    { "rate_pilot",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_RATE, 0 },
    { "chatter",              SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_100HZ, U_NONE, 0 },

    // frame counts
    { "mcp_1hz_framecount",     SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_1HZ,   U_NONE, 0 },
    { "mcp_1hz_framecount_dl",  SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_1HZ,   U_NONE, 0 },
    { "mcp_5hz_framecount",     SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_5HZ,   U_NONE, 0 },
    { "mcp_100hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_100HZ, U_NONE, 0 },
    { "mcp_200hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_200HZ, U_NONE, 0 },
    { "mcp_244hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_244HZ, U_NONE, 0 },
    { "mcp_488hz_framecount",   SCALE(CONVERT_UNITY),  TYPE_INT32,    RATE_488HZ, U_NONE, 0 },

    // Shared FC status variables (temperatures and voltages)
    { "t_cpu0_flc_s",          SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "t_cpu0_flc_n",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "t_cpu1_flc_s",          SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "t_cpu1_flc_n",         SCALE(CONVERT_UNITY), TYPE_INT16, RATE_1HZ, U_T_C, 0 },
    { "v_12v_flc_s",          0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_12v_flc_n",          0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_5v_flc_s",           0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_5v_flc_n",           0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_batt_flc_s",         0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "v_batt_flc_n",         0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_V_V, 0 },
    { "i_flc_s",              0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },
    { "i_flc_n",              0.01,             0.0, TYPE_UINT16, RATE_1HZ, U_I_A, 0 },

    // Shared MCP status variables (disk free, command counts, timeout, etc)
    { "last_cmd_s",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "last_cmd_n",           SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "count_cmd_s",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "count_cmd_n",          SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "df_flc_s",             1.0 / 250.0,      0.0, TYPE_UINT16, RATE_5HZ, U_GB, 0 },
    { "df_flc_n",             1.0 / 250.0,      0.0, TYPE_UINT16, RATE_5HZ, U_GB, 0 },
    { "time_flc_s",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    { "time_flc_n",           SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    { "hdd_disk_space_s",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    { "hdd_disk_space_n",     SCALE(CONVERT_UNITY), TYPE_UINT32, RATE_1HZ, U_NONE, 0 },
    { "hdd_disk_index_s",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "hdd_disk_index_n",     SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_1HZ, U_NONE, 0 },
    { "timeout_s",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },
    { "timeout_n",            SCALE(CONVERT_UNITY), TYPE_UINT16, RATE_5HZ, U_NONE, 0 },

     /* ----------------------- */
     /* NULL TERMINATE THE LIST */
     /* ----------------------- */
     { {0} }
  };

