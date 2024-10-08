/* 
 * ec_motors.h: 
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
 * Created on: Mar 27, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_EC_MOTORS_H_
#define INCLUDE_EC_MOTORS_H_

#include <stdint.h>

/**
 * @brief takes and index and subindex with a map object and places them in the map
 * 
 */
#define MAP(_index, _subindex, _map) {\
    _map.index = _index;\
    _map.subindex = _subindex;\
}
// Current loop tuning coefficients are generally determined from Copley
// tuning software (CME2) or manual tuning of response curve of commanded
// current/achieved current vs. time. They will need to be updated for each new
// motor/drive pairing.
// reaction wheel P.I.(D.) coefficients
#define RW_DEFAULT_CURRENT_P    20000
#define RW_DEFAULT_CURRENT_I    200
#define RW_DEFAULT_CURRENT_OFF  (0)
// elevation drive P.I.(D.) coefficients
#define EL_DEFAULT_CURRENT_P    1500
#define EL_DEFAULT_CURRENT_I    45
#define EL_DEFAULT_CURRENT_OFF  (0)
// pivot motor P.I.(D.) coefficients
#define PIV_DEFAULT_CURRENT_P    6000
#define PIV_DEFAULT_CURRENT_I    200
#define PIV_DEFAULT_CURRENT_OFF  (0)

// Period of heartbeat sent by devices in milliseconds
#define HEARTBEAT_MS    0
// require a connection every second (10 x 100 ms) or trigger a heartbeat error.
#define LIFETIME_FACTOR_EC    10
// After this many network status errors we will attempt to reset the Ethercat connection.
#define NETWORK_ERR_RESET_THRESH (500 * 20)
/**
 * @brief N.B. Here, RX/TX are from the controller's perspective, so RX is
 * received by the controller and TX is transmitted by the controller
 */
#define ECAT_RXPDO_ASSIGNMENT 0x1c12
/**
 * @brief N.B. Here, RX/TX are from the controller's perspective, so RX is
 * received by the controller and TX is transmitted by the controller
 */
#define ECAT_TXPDO_ASSIGNMENT 0x1c13

#define ECAT_TXPDO_MAPPING 0x1a00
#define ECAT_RXPDO_MAPPING 0x1600

#define ECAT_DC_CYCLE_NS 1000000 /* Distributed Clock Cycle in nanoseconds */
#define EC_TIMEOUTMON 500 // Timeout for peripheral recovery and reconfig, microseconds
#ifndef EC_STATE_NONE
// Older versions of SOEM don't provide this, but it's useful for recovering
// lost drives
#define EC_STATE_NONE 0x00
#endif


/**
 * @brief Process Data Object mapping union as raw or split up data
 * 
 */
typedef union {
    uint32_t val;
    struct {
        uint8_t size;
        uint8_t subindex;
        uint16_t index;
    };
} pdo_mapping_t;


/**
 * Map a PDO mapping structure to its parts.
 * @param m_map PDO map
 * @param m_index The object index
 * @param m_subindex The object sub-index
 * @param m_bits The number of bits utilized by the object (look it up before assigning!)
 */
static inline void map_pdo(pdo_mapping_t *m_map, uint16_t m_index, uint8_t m_subindex, uint8_t m_bits)
{
    m_map->index = m_index;
    m_map->subindex = m_subindex;
    m_map->size = m_bits;
}


/**
 * Take a defined Object Index/Subindex and return just the index
 * @param m_index
 * @param m_subindex
 * @return uint16_t index, we get passed a pair of indices and return the top level index
 */
static inline uint16_t object_index(uint16_t m_index, uint8_t m_subindex)
{
    return m_index;
}


/**
 * Take a defined Object Index/Subindex and return just the subindex
 * @param m_index
 * @param m_subindex
 * @return uint16_t subindex, we get passed a pair of indices and return the low level subindex
 */
static inline uint8_t object_subindex(uint16_t m_index, uint8_t m_subindex)
{
    return m_subindex;
}


/**
 * @brief maps the Ethercat state symbols to integers
 * 
 */
typedef enum {
    ECAT_DEV_COLD,           //!< ECAT_MOTOR_COLD
    ECAT_DEV_FOUND,          //!< ECAT_MOTOR_FOUND
    ECAT_DEV_MAPPED,         //!< ECAT_MOTOR_MAPPED
    ECAT_DEV_RUNNING,        //!< ECAT_MOTOR_RUNNING
    ECAT_DEV_LOST,
} ec_motor_state_t;


/**
 * @brief maps the ethercat control status symbols to integers
 * 
 */
typedef enum {
    ECAT_MOTOR_COLD,           //!< ECAT_MOTOR_COLD
    ECAT_MOTOR_INIT,           //!< ECAT_MOTOR_INIT
    ECAT_MOTOR_FOUND_PARTIAL,  //!< ECAT_MOTOR_FOUND_PARTIAL
    ECAT_MOTOR_FOUND,          //!< ECAT_MOTOR_FOUND
    ECAT_MOTOR_RUNNING_PARTIAL,//!< ECAT_MOTOR_RUNNING_PARTIAL
    ECAT_MOTOR_RUNNING         //!< ECAT_MOTOR_RUNNING
} ec_control_status_t;


/**
 * @brief ethercat device state structure containing relevant information about the device
 * 
 */
typedef struct {
    uint8_t index;
    uint8_t ec_unknown;
    uint8_t is_mc;
    uint8_t comms_ok;
    uint8_t has_dc;
    uint8_t periph_error;
    uint16_t network_error_count;
    ec_control_status_t status;
} ec_device_state_t;


/**
 * @brief ethercat network state structure containing relevant information about the network
 * 
 */
typedef struct {
    int8_t n_found;
    int8_t periph_count;
    uint16_t network_error_count;
    ec_control_status_t status;
} ec_state_t;

#define COPLEY_ETHERCAT_VENDOR 0x000000ab
#define AEP_090_036_PRODCODE 0x00000380
#define BEL_090_030_PRODCODE 0x00001110

/*******************************************************************
 * This section encodes a number of CanBus/EtherCAT index/subindex values
 * for the Copley motor controller commands.  This are referenced from
 * http://www.copleycontrols.com/Motion/pdf/Parameter_Dictionary.pdf
 *
 * They are stored as index, subindex macros for use in registering
 * SDO calls.
 *
 */

#define ECAT_COUNTS_PER_REV 0x2383, 23 /* Encoder counts per revolution INT32 */
#define ECAT_RESOLVER_CYCLES_PER_REV 0x2383, 34 /* Resolver cycles per revolution UINT16 */

#define ECAT_ENCODER_WRAP 0x2220, 0 /* Encoder wrap position INT32 */
#define ECAT_LOAD_WRAP 0x2221, 0 /* Encoder Load wrap position INT32 */
#define ECAT_LOAD_DIR 0x2383, 31 /* Load Encoder direction (0 or 1) UINT16 */

#define ECAT_CURRENT_LOOP_CP 0x2380, 1 /* Proportional Gain UINT16 */
#define ECAT_CURRENT_LOOP_CI 0x2380, 2 /* Integral Gain UINT16 */
#define ECAT_CURRENT_LOOP_OFFSET 0x2380, 3 /* Current Offset INT16 */
#define ECAT_CURRENT_LOOP_CMD 0x2340, 0 /* Commanded current in 0.01A INT16 */
#define ECAT_CURRENT_ACTUAL 0x221C, 0 /* Measured current in 0.01A INT16 */

#define ECAT_VEL_CMD 0x2341, 0 /* Requested Velocity INT32 */
#define ECAT_VEL_LOOP_CP 0x2381, 1 /* Velocity Proportional Gain UINT16 */
#define ECAT_VEL_LOOP_CI 0x2381, 2 /* Velocity Integral Gain UINT16 */
#define ECAT_VEL_LOOP_CD 0x2381, 5 /* Velocity Vi Drain UINT16 */
#define ECAT_VEL_ACTUAL 0x6069, 0 /* Actual Velocity 0.1 counts/s INT32 */
#define ECAT_VEL_ENCODER 0x2231, 0 /* Velocity of the external encoder 0.1 counts/s INT32 */

#define ECAT_DRIVE_STATE 0x2300, 0 /* Desired state of the drive UINT16 */
# define ECAT_DRIVE_STATE_DISABLED 0
# define ECAT_DRIVE_STATE_PROG_CURRENT 1
# define ECAT_DRIVE_STATE_PROG_VELOCITY 11

/**
 * @brief Status bits for the load encoder
 * Bit 0 - CRC Error on data
 * Bit 1 - Encoder failed to transmit data to amp 
 * Bit 2 - Error bit on encoder stream active
 * Bit 3 - Warning bit on encoder stream active
 * Bit 4 - Encoder Transmission delay too long
 * 
 */
#define ECAT_LOAD_STATUS 0x2225, 0  

#define ECAT_MOTOR_POSITION 0x2240, 0 /* Encoder position in counts INT32 */
#define ECAT_LOAD_POSITION 0x2242, 0 /* Load Encoder position in counts INT32 */
#define ECAT_ACTUAL_POSITION 0x6063, 0  /* Encoder position used for loops in counts INT32 */
#define ECAT_DRIVE_TEMP 0x2202, 0 /* A/D Reading in degrees C INT16 */
#define ECAT_PHASE_ANGLE 0x2260, 0 /* Motor phasing mode (deg)*/
#define ECAT_PHASE_ANGLE_RW 0x2262, 0 /* Motor phasing mode (deg)*/
#define ECAT_PHASING_MODE 0x21C0, 0 /* Motor phasing mode */
#define ECAT_MOTOR_HALL_OFF 0x2383, 6 /* Motor Hall Sensor Offset (deg)*/
#define ECAT_PHASE_INIT_CONFIG 0x21C4, 0 /* ALGORITHMIC PHASE INITIALIZATION CONFIG  UINT16*/
#define ECAT_CONTROL_MODE 0x6060, 0 /* Motor Control Mode UINT8*/
#define ECAT_COMMUTATION_ANGLE 0x60EA, 0 /* Commutation Angle (2^16) UINT16 */

#define ECAT_FUCHS_POSITION  0x6004, 0 /* PEPERL+FUCHS encoder position value UINT32 */
#define ECAT_FUCHS_OP_STATUS 0x6500, 0 /* PEPERL+FUCHS encoder position value UINT16 */


#define ECAT_PHASE_INIT 0x04 /* This is the configuration that we will start with  UINT16*/

#define ECAT_DRIVE_STATUS 0x1002, 0 /* Drive status bitmap UINT32 */
#  define ECAT_STATUS_SHORTCIRCUIT          (1<<0)
#  define ECAT_STATUS_DRIVE_OVERTEMP        (1<<1)
#  define ECAT_STATUS_OVERVOLTAGE           (1<<2)
#  define ECAT_STATUS_UNDERVOLTAGE          (1<<3)
#  define ECAT_STATUS_TEMP_SENS_ACTIVE      (1<<4)
#  define ECAT_STATUS_ENCODER_FEEDBACK_ERR  (1<<5)
#  define ECAT_STATUS_PHASING_ERROR         (1<<6)
#  define ECAT_STATUS_CURRENT_LIMITED       (1<<7)
#  define ECAT_STATUS_VOLTAGE_LIMITED       (1<<8)
#  define ECAT_STATUS_POS_LIMIT_SW          (1<<9)
#  define ECAT_STATUS_NEG_LIMIT_SW          (1<<10)
#  define ECAT_STATUS_ENABLE_NOT_ACTIVE     (1<<11)
#  define ECAT_STATUS_SW_DISABLE            (1<<12)
#  define ECAT_STATUS_STOPPING              (1<<13)
#  define ECAT_STATUS_BRAKE_ON              (1<<14)
#  define ECAT_STATUS_PWM_DISABLED          (1<<15)
#  define ECAT_STATUS_POS_SW_LIMIT          (1<<16)
#  define ECAT_STATUS_NEG_SW_LIMIT          (1<<17)
#  define ECAT_STATUS_TRACKING_ERROR        (1<<18)
#  define ECAT_STATUS_TRACKING_WARNING      (1<<19)
#  define ECAT_STATUS_DRIVE_RESET           (1<<20)
#  define ECAT_STATUS_POS_WRAPPED           (1<<21)
#  define ECAT_STATUS_DRIVE_FAULT           (1<<22)
#  define ECAT_STATUS_VEL_LIMIT             (1<<23)
#  define ECAT_STATUS_ACCEL_LIMIT           (1<<24)
#  define ECAT_STATUS_TRACK_WINDOW          (1<<25)
#  define ECAT_STATUS_HOME_SWITCH_ACTIVE    (1<<26)
#  define ECAT_STATUS_IN_MOTION             (1<<27)
#  define ECAT_STATUS_VEL_WINDOW            (1<<28)
#  define ECAT_STATUS_PHASE_UNINIT          (1<<29)
#  define ECAT_STATUS_CMD_FAULT             (1<<30)

#define ECAT_STICKY_EVENT_STATUS 0x2180, 0 // memory register of amp events
#define ECAT_LATCHED_EVENT_STATUS 0x2181, 0 // memory of events causing latching fault
#define ECAT_LATCHED_FAULT_MASK 0x2182, 0 // unset bits to ignore latching faults
#define ECAT_LATCHED_FAULT 0x2183, 0 // Drive faults bitmap UINT32
#  define ECAT_FAULT_DATA_CRC               (1<<0)
#  define ECAT_FAULT_INT_ERR                (1<<1)
#  define ECAT_FAULT_SHORT_CIRCUIT          (1<<2)
#  define ECAT_FAULT_DRIVE_OVER_TEMP        (1<<3)
#  define ECAT_FAULT_MOTOR_OVER_TEMP        (1<<4)
#  define ECAT_FAULT_OVER_VOLT              (1<<5)
#  define ECAT_FAULT_UNDER_VOLT             (1<<6)
#  define ECAT_FAULT_FEEDBACK_FAULT         (1<<7)
#  define ECAT_FAULT_PHASING_ERR            (1<<8)
#  define ECAT_FAULT_TRACKING_ERR           (1<<9)
#  define ECAT_FAULT_CURRENT_LIMIT          (1<<10)
#  define ECAT_FAULT_FPGA_ERR1              (1<<11)
#  define ECAT_FAULT_CMD_LOST               (1<<12)
#  define ECAT_FAULT_FPGA_ERR2              (1<<13)
#  define ECAT_FAULT_SAFETY_CIRCUIT_FAULT   (1<<14)
#  define ECAT_FAULT_CANT_CONTROL_CURRENT   (1<<15)
#  define ECAT_FAULT_WIRING_DISCONNECT      (1<<16)

#define ECAT_CTL_STATUS 0x6041, 0 /* Status word for Motor controller */
#  define ECAT_CTL_STATUS_READY             (1<<0)
#  define ECAT_CTL_STATUS_ON                (1<<1)
#  define ECAT_CTL_STATUS_OP_ENABLED        (1<<2)
#  define ECAT_CTL_STATUS_FAULT             (1<<3)
#  define ECAT_CTL_STATUS_VOLTAGE_ENABLED   (1<<4)
#  define ECAT_CTL_STATUS_QUICK_STOP        (1<<5)
#  define ECAT_CTL_STATUS_DISABLED          (1<<6)
#  define ECAT_CTL_STATUS_WARNING           (1<<7)
#  define ECAT_CTL_STATUS_LAST_ABORTED      (1<<8)
#  define ECAT_CTL_STATUS_USING_CANOPEN     (1<<9)
#  define ECAT_CTL_STATUS_TARGET_REACHED    (1<<10)
#  define ECAT_CTL_STATUS_LIMIT             (1<<11)
#  define ECAT_CTL_STATUS_MOVING            (1<<14)
#  define ECAT_CTL_STATUS_HOME_CAP          (1<<15)

#define ECAT_CTL_WORD 0x6040, 0 /* Control word for motor controller */
#  define ECAT_CTL_ON                       (1<<0)
#  define ECAT_CTL_ENABLE_VOLTAGE           (1<<1)
#  define ECAT_CTL_QUICK_STOP               (1<<2) /* Clear to enable quick stop */
#  define ECAT_CTL_ENABLE                   (1<<3)
#  define ECAT_CTL_RESET_FAULT              (1<<7)
#  define ECAT_CTL_HALT                     (1<<8)

#define ECAT_NET_STATUS 0x21B4, 0 /* Motor controller network status */
                                  /* NOTE: this bit field can only be mapped to */
                                  /* a transmit PDO */
#  define ECAT_NET_NODE_STATUS      (1<<0) /* This is two bits */
#  define ECAT_NET_SYNC_MISSING     (1<<4)
#  define ECAT_NET_GUARD_ERROR      (1<<5)
#  define ECAT_NET_BUS_OFF          (1<<8)
#  define ECAT_NET_TRANSMIT_ERROR   (1<<9)
#  define ECAT_NET_RECEIVE_ERROR    (1<<10)
#  define ECAT_NET_TRANSMIT_WARNING (1<<11)
#  define ECAT_NET_RECEIVE_WARNING  (1<<12)

# define ECAT_NET_NODE_CHECK  0x0003
# define ECAT_NET_SYNC_CHECK  0x0010
# define ECAT_NET_COBUS_OFF_CHECK  0x0100

#define ECAT_HEARTBEAT_TIME 0x1017, 0 /* Heartbeat period (ms) */
#define ECAT_LIFETIME_FACTOR 0x100D, 0 /* */

double rw_get_position_degrees(void);
double el_get_position_degrees(void);
double el_get_motor_position_degrees(void);
double piv_get_position_degrees(void);
double rw_get_velocity_dps(void);
double el_get_velocity_dps(void);
double piv_get_velocity_dps(void);

uint32_t rw_get_latched(void);
uint32_t el_get_latched(void);
uint32_t piv_get_latched(void);
uint16_t rw_get_ctl_word(void);
uint16_t el_get_ctl_word(void);
uint16_t piv_get_ctl_word(void);
int32_t rw_get_position(void);
int32_t el_get_position(void);
int32_t el_get_motor_position(void);
int32_t piv_get_position(void);
int32_t rw_get_velocity(void);
int32_t el_get_velocity(void);
int32_t piv_get_velocity(void);
int16_t rw_get_current(void);
int16_t el_get_current(void);
int16_t piv_get_current(void);
uint16_t rw_get_status_word(void);
uint16_t el_get_status_word(void);
uint16_t piv_get_status_word(void);
uint32_t rw_get_status_register(void);
uint32_t el_get_status_register(void);
uint32_t piv_get_status_register(void);
int16_t rw_get_amp_temp(void);
int16_t el_get_amp_temp(void);
int16_t piv_get_amp_temp(void);
int16_t rw_get_phase_angle(void);
int16_t el_get_phase_angle(void);
int16_t piv_get_phase_angle(void);
uint16_t rw_get_phase_mode(void);
uint16_t el_get_phase_mode(void);
uint16_t piv_get_phase_mode(void);

void rw_set_current(int16_t m_cur);
void el_set_current(int16_t m_cur);
void piv_set_current(int16_t m_cur);
void rw_enable(void);
void el_enable(void);
void piv_enable(void);
void rw_disable(void);
void el_disable(void);
void piv_disable(void);
void rw_quick_stop(void);
void el_quick_stop(void);
void piv_quick_stop(void);
void rw_reset_fault(void);
void el_reset_fault(void);
void piv_reset_fault(void);
void rw_write_latched_fault_mask(int bit, int latching);
void el_write_latched_fault_mask(int bit, int latching);
void piv_write_latched_fault_mask(int bit, int latching);
void rw_reset_latched_fault(int bit);
void el_reset_latched_fault(int bit);
void piv_reset_latched_fault(int bit);

uint8_t is_el_motor_ready();
int initialize_motors(void);

#endif /* INCLUDE_EC_MOTORS_H_ */
