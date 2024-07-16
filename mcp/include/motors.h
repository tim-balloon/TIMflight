/* 
 * motors.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
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
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 31, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_MOTORS_H_
#define INCLUDE_MOTORS_H_

#include <stdint.h>

// Keep mcp from attempting to point too high or too low.
// minimum allowed elevation angle of the inner frame
#define MIN_EL 19.0
// maximum allowed elevation angle of the inner frame
#define MAX_EL 55.0

// maximum azimuth velocity 
#define MAX_V_AZ 2.0 // was 2000 in gyro units
// maximum elevation velocity 
#define MAX_V_EL 0.5

// maximum reaction wheel current
// #define MAX_RW_CURRENT 2000 // 20 Amps in 0.01A units
#define MAX_RW_CURRENT 500 // 5 Amps in 0.01A units
// minimum (maximally negative) reaction wheel current
// #define MIN_RW_CURRENT (-2000) // 20 Amps in 0.01A units
#define MIN_RW_CURRENT (-500) // 5 Amps in 0.01A units

// maximum elevation drive current
// #define MAX_EL_CURRENT 2000 // 20 Amps in 0.01A units
#define MAX_EL_CURRENT 200 // 2 Amps in 0.01A units
// minimum (maximally negative) elevation drive current
// #define MIN_EL_CURRENT (-2000) // 20 Amps in 0.01A units
#define MIN_EL_CURRENT (-200) // 2 Amps in 0.01A units

// maximum pivot motor current
// #define MAX_PIV_CURRENT 3000 // 30 Amps in 0.01A units
#define MAX_PIV_CURRENT 500 // 5 Amps in 0.01A units
// minimum (maximally negative) pivot motor current
// #define MIN_PIV_CURRENT (-3000) // 30 Amps in 0.01A units
#define MIN_PIV_CURRENT (-500) // 5 Amps in 0.01A units

// unused
#define VPIV_FILTER_LEN 40
// unused
#define FPIV_FILTER_LEN 1000

// elevation drive acceleration 
#define EL_ACCEL 0.5

#define NO_DITH_INC 0
#define DITH_INC 1

// ETHERCAT ERROR CODES FOR CHANNELIZATION
#define ECODE_NONE   0x0000
#define ECODE_UNSPEC   0x0001
#define ECODE_NO_MEM   0x0002
#define ECODE_INVALID_STATE_CHANGE   0x0011
#define ECODE_UNK_REQ_STATE   0x0012
#define ECODE_NO_BOOTSTRAP   0x0013
#define ECODE_NO_FIRMWARE   0x0014
#define ECODE_INV_MAIL_CONFIG   0x0015
#define ECODE_INV_MAIL_CONFIG2   0x0016
#define ECODE_INV_SYNC_CONFIG   0x0017
#define ECODE_NO_INPUTS   0x0018
#define ECODE_NO_OUTPUTS   0x0019
#define ECODE_SYNC_ERROR   0x001A
#define ECODE_SYNC_WATCHDOG   0x001B
#define ECODE_INV_SYNC_TYPES   0x001C
#define ECODE_INV_OUT_CONFIG   0x001D
#define ECODE_INV_IN_CONFIG   0x001E
#define ECODE_INV_WATCHDOG_CONFIG   0x001F
#define ECODE_PERIPHERAL_COLD_START   0x0020
#define ECODE_PERIPHERAL_INIT   0x0021
#define ECODE_PERIPHERAL_PREOP   0x0022
#define ECODE_PERIPHERAL_SAFEOP   0x0023
#define ECODE_INV_IN_MAP   0x0024
#define ECODE_INV_OUT_MAP   0x0025
#define ECODE_INCONSISTENT_SETTINGS   0x0026
#define ECODE_FREERUN_UNSUP   0x0027
#define ECODE_SYNC_UNSUP   0x0028
#define ECODE_FREERUN_3BUF   0x0029
#define ECODE_BACKGROUND_WATCHDOG   0x002A
#define ECODE_NO_IN_OUT   0x002B
#define ECODE_FATAL_SYNC   0x002C
#define ECODE_NO_SYNC   0x002D
#define ECODE_INV_IN_FMMU   0x002E
#define ECODE_INV_DC_SYNC   0x0030
#define ECODE_INV_DC_LATCH   0x0031
#define ECODE_PLL   0x0032
#define ECODE_DC_SYNC_IO   0x0033
#define ECODE_DC_SYNC_TIMEOUT   0x0034
#define ECODE_DC_INV_SYNC_CYCLE   0x0035
#define ECODE_DC_INV_SYNC_CYCLE0   0x0036
#define ECODE_DC_INV_SYNC_CYCLE1   0x0037
#define ECODE_MAILBOX_EOE   0x0042
#define ECODE_MAILBOX_COE   0x0043
#define ECODE_MAILBOX_FOE   0x0044
#define ECODE_MAILBOX_SOE   0x0045
#define ECODE_MAILBOX_VOE   0x004F
#define ECODE_EEPROM_ACCESS   0x0050
#define ECODE_EEPROM   0x0051
#define ECODE_PERIPHERAL_RESTART   0x0060
#define ECODE_DEVICE_ID_UPDATE   0x0061
#define ECODE_APP_CONTROLLER_AVAIL   0x00f0
#define ECODE_UNK   0xffff

// TODO(seth): Add State/Desired State here
/**********************************************/
/** @brief  Motor Data Struct                 */
/*  - Stores encoder/velocity information     */
/*  from the motors                           */
/*  - Written to struct in ec_motors.c        */
/*  - Written to the frame in the main thread */
/*  USE A CIRCULAR BUFFER !!!                 */
typedef struct
{
    int32_t velocity;             // in 0.1 counts per second
    int16_t temp;                 // drive temperature in degrees Celsius
    double current;               // drive current read from controller
    int32_t position;             // Position used for calculations
    int32_t motor_position;       // Motor position
    uint32_t status;              // drive status
    uint16_t network_status;      // network status
    uint32_t fault_reg;           // drive fault register
    uint16_t ALstatuscode;        // EtherCAT application layer status code (see ETG.1000)
    uint16_t ALstate;             // EtherCAT state machine state (see ETG.1000)
    uint16_t drive_info;          // motorinfo struct
    uint16_t control_word_read;   // commanded state read back from controller
    uint16_t control_word_write;  // commanded state
    int16_t  phase_mode;          // motor phase mode
    uint16_t phase_angle;         // motor phase angle
    uint32_t err_count;           // count of serious serial errors
    uint8_t network_problem;      // check_for_network_problem returns 1
} motor_data_t;

extern motor_data_t RWMotorData[3];
extern motor_data_t ElevMotorData[3];
extern motor_data_t PivotMotorData[3];

extern int motor_index;

void command_motors(void);
void update_axes_mode(void);
void write_motor_channels_5hz(void);
void write_motor_channels_100hz(void);
void write_motor_channels_200hz(void);
void store_axes_mode_data(void);
void record_motor_status_1hz(void);

#endif /* INCLUDE_MOTORS_H_ */
