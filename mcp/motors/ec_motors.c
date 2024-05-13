/* 
 * ec_motors.c: 
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
 * Created on: Mar 26, 2015 by Seth Hillbrand
 */


#include <phenom/thread.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <glib.h>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>

#include <blast_time.h>
#include <calibrate.h>
#include <command_struct.h>
#include <ec_motors.h>
#include <motors.h>
#include <mcp.h>
#include <mputs.h>

static ph_thread_t *motor_ctl_id;
static ph_thread_t *ecmonitor_ctl_id;

extern int16_t InCharge;

// memory on this PC is not a concern yet - give much headroom by setting to
// 8 MB which is fairly standard on modern PCs
#define ECAT_SENDRECV_STACKSIZE (8 * 1024 * 1024)
int ecat_sendrecv_cadence_ns = 2000000;
pthread_t ecat_sendrecv_thread;
pthread_t ecat_check_thread;
// We require some globals here to keep track of state between threads for
// EtherCAT setup and monitoring. Exercise care editing usage of these,
// since we don't guard access to them.
// Toggle for operation data to/from peripherals
uint8_t ecat_do_sendrecv = 0;
// The Working Counter is incremented if an EtherCAT device was successfully
// addressed and a read operation, a write operation or a read/write operation
// was executed successfully. We check it to verify that.
uint16_t ecat_expected_wkc = 0;
volatile uint16_t ecat_current_wkc = 0;
// Keep track of whether all EtherCAT devices should be in the OP state. We use
// this to monitor for devices that have departed the OP state.
uint8_t ecat_in_op_state = 0;

// Intentional aliasing, ref. ecx_contextt struct in ethercatmain.h.
// ec_slavecount is declared extern, and the ecx_context has a pointer to it,
// which is used by SOEM routines to store updated values when we ask SOEM to
// find EtherCAT devices on the network. We access that memory using our new
// name.
int* p_ec_periphcount = &ec_slavecount;
// Typically, we would want to just keep the pointer, and dereference it when
// we need the value in memory. To keep changes minimal, and avoid adding
// potentially confusing dereferences everywhere, we use a #define.
#define ec_periphcount (*p_ec_periphcount)
// ecx_context struct holds a pointer to this array of ec_slavet, and the
// values pointed to are once again updated by SOEM. We access that memory
// using our new name.
ec_slavet* ec_periph = ec_slave;

/**
 * @brief Number of ethercat controllers
 * If you change this, also change EC_MAXSLAVE in ethercatmain.h
 */
#define N_MCs 4

// device node Serial Numbers
#define RW_SN  0x35f4cc0d
#define PIV_SN 0x36abd52a
#define EL_SN 0x35f4cc11
// addresses on the EC network
#define RW_ADDR 0x3
#define EL_ADDR 0x2
#define PIV_ADDR 0x1

// max number of characters in a PDO name
#define PDO_NAME_LEN 32

/**
 * @brief Structure for storing the PDO assignments and their offsets in the
 * memory map
 */
typedef struct {
    char        name[PDO_NAME_LEN];
    uint16_t    index;
    uint8_t     subindex;
    int         offset;
} pdo_channel_map_t;
static GSList *pdo_list[N_MCs];


/**
 * @brief Index numbers for the peripheral array.  0 is the root node (flight computer)
 * These are set by the find_controllers function that is run when we start up the network
 */
static int rw_index = 0;
static int piv_index = 0;
static int el_index = 0;


/**
 * @brief Memory mapping for the PDO variables, allocates 4 kb
 */
static char io_map[4096];

static int motors_exit = false;

/**
 * @brief Ethercat driver status
 */
static ec_device_state_t controller_state[N_MCs] = {{0}};

/*
static ec_motor_state_t controller_state[N_MCs] = {ECAT_MOTOR_COLD, ECAT_MOTOR_COLD, ECAT_MOTOR_COLD, ECAT_MOTOR_COLD};
*/
ec_state_t ec_mcp_state = {0};

/**
 * @brief The following pointers reference data points read from/written to
 * PDOs for each motor controller.  Modifying the data at the address for the write
 * word will cause that change to be transmitted to the motor controller at
 * the next PDO cycle (0.5ms)
 */
static int32_t dummy_var = 0;
static int32_t dummy_write_var = 0;

/// Read words
static int32_t *motor_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var };
static int32_t *motor_velocity[N_MCs] = { &dummy_var, &dummy_var, &dummy_var , &dummy_var };
static int32_t *actual_position[N_MCs] = { &dummy_var, &dummy_var, &dummy_var, &dummy_var };
static int16_t *motor_current[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                         (int16_t*) &dummy_var, (int16_t*) &dummy_var };
static uint32_t *status_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                            (uint32_t*) &dummy_var, (uint32_t*) &dummy_var };
static int16_t *amp_temp[N_MCs] = { (int16_t*) &dummy_var, (int16_t*) &dummy_var,
                                    (int16_t*) &dummy_var, (int16_t*) &dummy_var };
static uint16_t *status_word[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                        (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };
static uint16_t *network_status_word[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                                (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };
static uint32_t *latched_register[N_MCs] = { (uint32_t*) &dummy_var, (uint32_t*) &dummy_var,
                                             (uint32_t*) &dummy_var, (uint32_t*) &dummy_var };
static uint16_t *control_word_read[N_MCs] = { (uint16_t*) &dummy_var, (uint16_t*) &dummy_var,
                                              (uint16_t*) &dummy_var, (uint16_t*) &dummy_var };
static int16_t *phase_angle[N_MCs] = { (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var,
                                       (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var };
static uint16_t *phase_mode[N_MCs] = { (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var,
                                       (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var };
/// Write words
static uint16_t *control_word[N_MCs] = { (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var,
                                         (uint16_t*) &dummy_write_var, (uint16_t*) &dummy_write_var };
static int16_t *target_current[N_MCs] = { (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var,
                                          (int16_t*) &dummy_write_var, (int16_t*) &dummy_write_var };



/**
 * @brief take an index and checks if the the comm status is good to go
 * 
 * @param m_index motor controller to talk to
 * @return int 0 on failure or 1 on success
 */
int check_peripheral_comm_ready(int m_index) {
    if (m_index < 1) {
        return 0; // m_index must be > 0
    }
    if (!controller_state[m_index].comms_ok) {
        return 0; // no errors in mapping
    } else {
        return 1;
    }
}


/**
 * @brief This function returns the latched faults of the RW motor controller
 * 
 * @return uint32_t value latched bitmap
 */
uint32_t rw_get_latched(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *latched_register[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the latched faults of the elevation motor controller
 * 
 * @return uint32_t value latched bitmap
 */
uint32_t el_get_latched(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *latched_register[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the latched faults of the pivot motor controller
 * 
 * @return uint32_t value latched bitmap
 */
uint32_t piv_get_latched(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *latched_register[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word for the RW motor controller
 * 
 * @return uint16_t control word bitmap
 */
uint16_t rw_get_ctl_word(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *control_word_read[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word for the elevation motor controller
 * 
 * @return uint16_t control word bitmap
 */
uint16_t el_get_ctl_word(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *control_word_read[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word for the pivot motor controller
 * 
 * @return uint16_t control word bitmap
 */
uint16_t piv_get_ctl_word(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *control_word_read[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase angle for the pivot motor controller (read back from the controller)
 * 
 * @return int16_t phase angle of the motor
 */
int16_t piv_get_phase_angle(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *phase_angle[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase angle for the RW motor controller (read back from the controller)
 * 
 * @return int16_t phase angle of the motor
 */
int16_t rw_get_phase_angle(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *phase_angle[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase angle for the elevation motor controller (read back from the controller)
 * 
 * @return int16_t phase angle of the motor
 */
int16_t el_get_phase_angle(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *phase_angle[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase mode for the pivot motor controller (read back from the controller)
 * 
 * @return uint16_t phase mode
 */
uint16_t piv_get_phase_mode(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *phase_mode[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase mode for the RW motor controller (read back from the controller)
 * 
 * @return uint16_t phase mode
 */
uint16_t rw_get_phase_mode(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *phase_mode[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the phase mode for the elevation motor controller (read back from the controller)
 * 
 * @return uint16_t phase mode
 */
uint16_t el_get_phase_mode(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *phase_mode[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word of the RW motor controller (sent to the controller)
 * 
 * @return uint16_t control word bitmap
 */
uint16_t rw_get_ctl_word_write(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *control_word[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word of the elevation motor controller (sent to the controller)
 * 
 * @return uint16_t control word bitmap
 */
uint16_t el_get_ctl_word_write(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *control_word[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the control word of the pivot motor controller (sent to the controller)
 * 
 * @return uint16_t control word bitmap
 */
uint16_t piv_get_ctl_word_write(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *control_word[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the network status word of the RW motor controller 
 * 
 * @return uint16_t network status word bitmap
 */
uint16_t rw_get_network_status_word(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *network_status_word[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the network status word of the elevation motor controller 
 * 
 * @return uint16_t network status word bitmap
 */
uint16_t el_get_network_status_word(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *network_status_word[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the network status word of the pivot motor controller 
 * 
 * @return uint16_t network status word bitmap
 */
uint16_t piv_get_network_status_word(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *network_status_word[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief Returns the hex value of the EtherCAT application layer status code.
 * 
 * @details Ref. ETG.1000, ETG.1300, Beckhoff Hardware Data Sheet Section II,
 * Sec. 2.21. The EtherCAT application layer (AL) status code tells us the
 * state of peripheral device's register 0x0134, which is controlled by the
 * peripheral and contains the last error detected by the state control
 * instance on it. It can be used to diagnose failed or unauthorized state
 * transitions (e.g. OP -> SAFE_OP) that may cause a drive's ERR indicator
 * light to flash.
 * 
 * @param[in] m_index The motor controller to talk to
 * @return Value of peripheral's 0x0134 register
 */
uint16_t get_ALstatuscode(int m_index) {
    // Ensure peripherals' state struct is up to date
    ec_readstate();
    return ec_periph[m_index].ALstatuscode;
}


/**
 * @brief Wrapper of get_ALstatuscode() for reaction wheel.
 * 
 * @return Value of peripheral's 0x0134 register
 */
uint16_t rw_get_ALstatuscode(void) {
    return get_ALstatuscode(rw_index);
}


/**
 * @brief Wrapper of get_ALstatuscode() for elevation axis.
 * 
 * @return Value of peripheral's 0x0134 register
 */
uint16_t el_get_ALstatuscode(void) {
    return get_ALstatuscode(el_index);
}


/**
 * @brief Wrapper of get_ALstatuscode() for pivot.
 * 
 * @return Value of peripheral's 0x0134 register
 */
uint16_t piv_get_ALstatuscode(void) {
    return get_ALstatuscode(piv_index);
}


/**
 * @brief Returns the hex value of the EtherCAT state machine (ESM).
 * 
 * @details Ref. ETG.1000 Part 6, Sec. 6.4.1, ETG.1300 Table 8. The EtherCAT
 * application layer (AL) state machine coordinates communication and
 * synchronization of the controller and peripheral devices during start up
 * and operation. It is the peripheral device's register 0x0130.
 * 
 * @param[in] m_index The motor controller to talk to
 * @return Value of peripheral's 0x0130 register
 */
uint16_t get_ALstate(int m_index) {
    // Ensure peripherals' state struct is up to date
    ec_readstate();
    return ec_periph[m_index].state;
}


/**
 * @brief Wrapper of get_ALstate() for reaction wheel.
 * 
 * @return Value of peripheral's 0x0130 register, EtherCAT state
 */
uint16_t rw_get_ALstate(void) {
    return get_ALstate(rw_index);
}


/**
 * @brief Wrapper of get_ALstate() for elevation axis.
 * 
 * @return Value of peripheral's 0x0130 register, EtherCAT state
 */
uint16_t el_get_ALstate(void) {
    return get_ALstate(el_index);
}


/**
 * @brief Wrapper of get_ALstate() for pivot.
 * 
 * @return Value of peripheral's 0x0130 register, EtherCAT state
 */
uint16_t piv_get_ALstate(void) {
    return get_ALstate(piv_index);
}


/**
 * @brief This function returns the absolute position read by the RW motor controller
 * 
 * @return int32_t value of the position (modulo the wrap position value)
 */
int32_t rw_get_position(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *actual_position[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the absolute position read by the elevation motor controller
 * 
 * @return int32_t value of the position (modulo the wrap position value)
 */
int32_t el_get_position(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *actual_position[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the offset position read by the elevation motor controller.
 * Offsetting motor units to correspond with elevation.
 * 
 * @return int32_t value of the position (modulo the wrap position value)
 */
int32_t el_get_motor_position(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *motor_position[el_index] + ENC_RAW_EL_OFFSET/EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the absolute position read by the pivot motor controller
 * 
 * @return int32_t value of the position (modulo the wrap position value)
 */
int32_t piv_get_position(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *actual_position[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the absolute position read by the RW motor controller
 * 
 * @return double value of the position in degrees (no set zero point)
 */
double rw_get_position_degrees(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return rw_get_position() * RW_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the absolute position read by the elevation motor controller
 * Uses the LOAD ENCODER scaling
 * 
 * @return double value of the position in degrees (no set zero point)
 */
double el_get_position_degrees(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return el_get_position() * EL_LOAD_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the position read by the elevation motor controller.
 * Uses the ENCODER scaling
 * 
 * @return double value of the position in degrees (no set zero point)
 */
double el_get_motor_position_degrees(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return el_get_motor_position() * EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the absolute position read by the pivot motor controller
 * 
 * @return double value of the position in degrees (no set zero point)
 */
double piv_get_position_degrees(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return piv_get_position() * PIV_RESOLVER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the RW motor controller
 * 
 * @return double value of the angular velocity in degrees per second
 */
double rw_get_velocity_dps(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *motor_velocity[rw_index] * 0.1 * RW_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the elevation motor controller
 * 
 * @return double value of the angular velocity in degrees per second
 */
double el_get_velocity_dps(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *motor_velocity[el_index] * 0.1 * EL_MOTOR_ENCODER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the pivot motor controller
 * 
 * @return double value of the angular velocity in degrees per second
 */
double piv_get_velocity_dps(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *motor_velocity[piv_index] * 0.1 * PIV_RESOLVER_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the RW motor controller
 * 
 * @return int32_t value of the velocity in 0.1 counts per second
 */
int32_t rw_get_velocity(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *motor_velocity[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the elevation motor controller
 * 
 * @return int32_t value of the velocity in 0.1 counts per second
 */
int32_t el_get_velocity(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *motor_velocity[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the calculated motor velocity of the pivot motor controller
 * 
 * @return int32_t value of the velocity in 0.1 counts per second
 */
int32_t piv_get_velocity(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *motor_velocity[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the current from the RW motor controller
 * 
 * @return int16_t value of the current in 0.01A units
 */
int16_t rw_get_current(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *motor_current[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the current from the elevation motor controller
 * 
 * @return int16_t value of the current in 0.01A units
 */
int16_t el_get_current(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *motor_current[el_index]*EL_MOTOR_CURRENT_SCALING;
    } else {
        return 0;
    }
}


/**
 * @brief This function returns the current from the pivot motor controller
 * 
 * @return int16_t value of the current in 0.01A units
 */
int16_t piv_get_current(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *motor_current[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the RW motor current operating mode
 * 
 * @return uint16_t mode bitmap
 */
uint16_t rw_get_status_word(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *status_word[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the elevation motor current operating mode
 * 
 * @return uint16_t mode bitmap
 */
uint16_t el_get_status_word(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *status_word[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the pivot motor current operating mode
 * 
 * @return uint16_t mode bitmap
 */
uint16_t piv_get_status_word(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *status_word[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the state of the RW motor controller
 * 
 * @return uint32_t bitmap for the status register
 */
uint32_t rw_get_status_register(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *status_register[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the state of the elevation motor controller
 * 
 * @return uint32_t bitmap for the status register
 */
uint32_t el_get_status_register(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *status_register[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function returns a bitmap of the state of the pivot motor controller
 * 
 * @return uint32_t bitmap for the status register
 */
uint32_t piv_get_status_register(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *status_register[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function gets the current amplifier temperature for the RW in Celsius
 * 
 * @return int16_t amplifier temperature in degrees C
 */
int16_t rw_get_amp_temp(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        return *amp_temp[rw_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function gets the current amplifier temperature for the elevation motor in Celsius
 * 
 * @return int16_t amplifier temperature in degrees C
 */
int16_t el_get_amp_temp(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        return *amp_temp[el_index];
    } else {
        return 0;
    }
}


/**
 * @brief This function gets the current amplifier temperature for the pivot in Celsius
 * 
 * @return int16_t amplifier temperature in degrees C
 */
int16_t piv_get_amp_temp(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        return *amp_temp[piv_index];
    } else {
        return 0;
    }
}


/**
 * @brief Sets the requested current for the RW motor
 * 
 * @param m_cur int16_t requested current in units of 0.01 amps
 */
void rw_set_current(int16_t m_cur)
{
    if (check_peripheral_comm_ready(rw_index)) {
        *target_current[rw_index] = m_cur;
    }
}


/**
 * @brief Sets the requested current for the elevation motor
 * 
 * @param m_cur int16_t requested current in units of 0.01 amps
 */
void el_set_current(int16_t m_cur)
{
    if (check_peripheral_comm_ready(el_index)) {
        *target_current[el_index] = m_cur*EL_MOTOR_CURRENT_SCALING;
    }
}


/**
 * @brief Sets the requested current for the pivot motor
 * 
 * @param m_cur int16_t requested current in units of 0.01 amps
 */
void piv_set_current(int16_t m_cur)
{
    if (check_peripheral_comm_ready(piv_index)) {
        *target_current[piv_index] = m_cur;
    }
}


/**
 * @brief Enables the operation of the RW motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void rw_enable(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        *control_word[rw_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}


/**
 * @brief Enables the operation of the elevation motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void el_enable(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        *control_word[el_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}


/**
 * @brief Enables the operation of the pivot motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void piv_enable(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        *control_word[piv_index] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
    }
}


/**
 * @brief Disables the operation of the RW motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void rw_disable(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        *control_word[rw_index] &= (~ECAT_CTL_ENABLE);
    }
}


/**
 * @brief Disables the operation of the elevation motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void el_disable(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        *control_word[el_index] &= (~ECAT_CTL_ENABLE);
    }
}


/**
 * @brief Disables the operation of the pivot motor controller.  These are currently set using the
 * PDO interface
 * 
 */
void piv_disable(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        *control_word[piv_index] &= (~ECAT_CTL_ENABLE);
    }
}


/**
 * @brief Sets the emergency quick stop flag for the RW motor
 * 
 */
void rw_quick_stop(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        *control_word[rw_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}


/**
 * @brief Sets the emergency quick stop flag for the elevation motor
 * 
 */
void el_quick_stop(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        *control_word[el_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}


/**
 * @brief Sets the emergency quick stop flag for the pivot motor
 * 
 */
void piv_quick_stop(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        *control_word[piv_index] &= (~ECAT_CTL_QUICK_STOP);
    }
}


/**
 * @brief Sets 'Reset Fault' flag for the RW motor
 * 
 */
void rw_reset_fault(void)
{
    if (check_peripheral_comm_ready(rw_index)) {
        *control_word[rw_index] |= ECAT_CTL_RESET_FAULT;
    }
}


/**
 * @brief Sets 'Reset Fault' flag for the elevation motor
 * 
 */
void el_reset_fault(void)
{
    if (check_peripheral_comm_ready(el_index)) {
        *control_word[el_index] |= ECAT_CTL_RESET_FAULT;
    }
}


/**
 * @brief Sets 'Reset Fault' flag for the pivot motor
 * 
 */
void piv_reset_fault(void)
{
    if (check_peripheral_comm_ready(piv_index)) {
        *control_word[piv_index] |= ECAT_CTL_RESET_FAULT;
    }
}


/**
 * @brief reads the shared data object and initializes the RW phases (commutation?)
 * 
 */
void rw_init_phasing(void)
{
    uint16_t phase_mode = 0;
    uint8_t control_mode = 0;
    int size = 0;
    if (rw_index) {
        ec_SDOread(rw_index, ECAT_PHASE_INIT_CONFIG, false, &size, &phase_mode, EC_TIMEOUTRXM);
        blast_info("Algorithmic phasing initialization config = %d, writing %d, read %d bytes",
                    phase_mode, ECAT_PHASE_INIT, size);
        ec_SDOwrite16(rw_index, ECAT_PHASE_INIT_CONFIG, ECAT_PHASE_INIT);
        ec_SDOread(rw_index, ECAT_CONTROL_MODE, false, &size, &control_mode, EC_TIMEOUTRXM);
        blast_info("Control mode = %d, read %d bytes",
                    control_mode, size);
        ec_SDOread(rw_index, ECAT_PHASE_INIT_CONFIG, false, &size, &phase_mode, EC_TIMEOUTRXM);
        blast_info("After change Algorithmic phasing initialization config = %d, read %d bytes",
                    phase_mode, size);
    }
}


/**
 * Sets the current limits for the RW motor.
 */
void rw_init_current_limit(void)
{
    if (rw_index) {
        ec_SDOwrite16(rw_index, 0x2110, 0, 3000);   // 30 Amps peak current limit
        ec_SDOwrite16(rw_index, 0x2111, 0, 1500);   // 15 Amps continuous current limit
    }
}


/**
 * Sets the current limits for the elevation motor.
 */
void el_init_current_limit(void)
{
    if (el_index) {
        ec_SDOwrite16(el_index, 0x2110, 0, 3000);   // 30 Amps peak current limit
        ec_SDOwrite16(el_index, 0x2111, 0, 1500);   // 15 Amps continuous current limit
    }
}


/**
 * Sets the current limits for the pivot motor.
 */
void piv_init_current_limit(void)
{
    if (piv_index) {
        ec_SDOwrite16(piv_index, 0x2110, 0, 3000);   // 30 Amps peak current limit
        ec_SDOwrite16(piv_index, 0x2111, 0, 1500);   // 15 Amps continuous current limit
    }
}


/**
 * @brief Sets the current default PID values for the RW motor
 * TODO(evanmayer): Update the current PIDs for each motor after tuning
 */
void rw_init_current_pid(void)
{
    if (rw_index) {
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_CP, RW_DEFAULT_CURRENT_P);
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_CI, RW_DEFAULT_CURRENT_I);
        ec_SDOwrite16(rw_index, ECAT_CURRENT_LOOP_OFFSET, RW_DEFAULT_CURRENT_OFF);
    }
}


/**
 * @brief Sets the current default PID values for the elevation motor
 * TODO(evanmayer): Update the current PIDs for each motor after tuning
 */
void el_init_current_pid(void)
{
    if (el_index) {
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_CP, EL_DEFAULT_CURRENT_P);
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_CI, EL_DEFAULT_CURRENT_I);
        ec_SDOwrite16(el_index, ECAT_CURRENT_LOOP_OFFSET, EL_DEFAULT_CURRENT_OFF);
    }
}


/**
 * @brief Sets the current default PID values for the pivot motor
 * TODO(evanmayer): Update the current PIDs for each motor after tuning
 */
void piv_init_current_pid(void)
{
    if (piv_index) {
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_CP, PIV_DEFAULT_CURRENT_P);
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_CI, PIV_DEFAULT_CURRENT_I);
        ec_SDOwrite16(piv_index, ECAT_CURRENT_LOOP_OFFSET, PIV_DEFAULT_CURRENT_OFF);
    }
}


/**
 * @brief initializes the RW encoder in the shared data object 
 * 
 */
static void rw_init_encoder(void)
{
    if (rw_index) {
        ec_SDOwrite32(rw_index, ECAT_ENCODER_WRAP, RW_ENCODER_COUNTS);
        ec_SDOwrite32(rw_index, ECAT_COUNTS_PER_REV, RW_COUNTS_PER_REV);
    }
}


/**
 * @brief initializes the elevation encoder in the shared data object 
 * 
 */
static void el_init_encoder(void)
{
    if (el_index) {
        ec_SDOwrite32(el_index, ECAT_ENCODER_WRAP, EL_MOTOR_ENCODER_COUNTS);
        ec_SDOwrite32(el_index, ECAT_COUNTS_PER_REV, EL_MOTOR_COUNTS_PER_REV);
        // ec_SDOwrite32(el_index, ECAT_LOAD_WRAP, EL_LOAD_ENCODER_COUNTS);
        // ec_SDOwrite16(el_index, ECAT_LOAD_DIR, 1); // The load encoder runs in the opposite direction of the motor
    }
}


/**
 * @brief initializes the pivot resolver in the shared data object (similar to encoder but absolute)
 * 
 */
static void piv_init_resolver(void)
{
    if (piv_index) {
        ec_SDOwrite32(piv_index, ECAT_ENCODER_WRAP, PIV_RESOLVER_COUNTS);
        ec_SDOwrite32(piv_index, ECAT_COUNTS_PER_REV, PIV_RESOLVER_COUNTS);
        ec_SDOwrite16(piv_index, ECAT_RESOLVER_CYCLES_PER_REV, 1);
    }
}


/**
 * @brief initializes the ethercat heartbeat to make sure we remain connected to a motor controller
 * 
 * @param periph_index index of the particular motor to monitor
 */
static void ec_init_heartbeat(int periph_index)
{
    if (periph_index) {
        ec_SDOwrite16(periph_index, ECAT_HEARTBEAT_TIME, HEARTBEAT_MS);
        ec_SDOwrite8(periph_index, ECAT_LIFETIME_FACTOR, LIFETIME_FACTOR_EC);
    }
}


/**
 * @brief Find a distributed clock source on the bus and configure it to send
 * out sync pulses.
 * 
 * @details There must be only a single distributed clock source on the entire
 * bus. This should be one of the controllers, since the flight computer has
 * that pesky operating system messing with its timing. So we choose the first
 * peripheral that has dc enabled as the SYNC source. Everyone else on the bus
 * gets the same DC_CYCLE (in nanoseconds) but have their sync disabled. For
 * BEL-090-030 drives, it was found that this sync step works best if it
 * happens in PRE-OP state, just before requesting transition to SAFE-OP, but
 * this behavior may vary between drives, see
 * https://github.com/OpenEtherCATsociety/SOES/issues/151.
 */
static void motor_configure_timing(void)
{
    int found_dc_source = 0;
    ec_configdc();
    while (ec_iserror()) {
        blast_err("Error after ec_iserror(), %s", ec_elist2string());
    }
    for (int i = 1; i <= ec_periphcount; i++) {
        // TODO(evanmayer): This code is necessary for running with a
        // distributed clock, which is more accurate. But programming the
        // configuration is extremely tricky, so leave in machinery for looking
        // for a clock source, but run in free-run mode for now.
        // blast_info("Looking for distributed clock on peripheral %d", i);
        if (!found_dc_source && ec_periph[i].hasdc) {
            // ec_dcsync0(i, true, ECAT_DC_CYCLE_NS, ec_periph[i].pdelay);
            found_dc_source = 1;
            // blast_info("Distributed clock source is peripheral %d", i);
        } else {
            // ec_dcsync0(i, false, ECAT_DC_CYCLE_NS, ec_periph[i].pdelay);
        }
        // blast_info("Peripheral %d propagation delay is %d", i, ec_periph[i].pdelay);
        while (ec_iserror()) {
            blast_err("Peripheral %i, %s", i, ec_elist2string());
        }

        // Set drives to run in free-run mode
        // Ref.
        // https://infosys.beckhoff.com/english.php?content=../content/1033/ethercatsystem/2469122443.html&id=
        ec_SDOwrite16(i, 0x1C32, 1, 0);
        while (ec_iserror()) {
            blast_err("Peripheral %i, %s", i, ec_elist2string());
        }
        if (ec_periph[i].hasdc && found_dc_source) {
            controller_state[i].has_dc = 1;
        } else {
            controller_state[i].has_dc = 0;
        }
    }
}


/**
 * @brief Finds all motor controllers on the network and sets them to pre-operational state
 * @return -1 on error, number of controllers found otherwise
 */
static int find_controllers(void)
{
//    blast_info("initial io_map pointer %p pointer to pointer %p", io_map, &io_map);
    ec_mcp_state.n_found = ec_config(false, &io_map);
//    blast_info("after ec_config io_map pointer %p pointer to pointer %p", io_map, &io_map);
    if (ec_mcp_state.n_found <= 0) {
        berror(err, "No motor controller peripherals found on the network!");
        return -1;
    }
    blast_startup("ec_config returns %d peripherals found", ec_mcp_state.n_found);

    if (ec_mcp_state.n_found < (N_MCs -1)) {
        ec_mcp_state.status = ECAT_MOTOR_FOUND_PARTIAL;
    } else {
        ec_mcp_state.status = ECAT_MOTOR_FOUND;
    }

    // Start the Distributed Clock cycle.
    // Start sync0 pulses before transition to SAFE-OP state, but this may vary
    // between models and manufacturers. Ref.
    // https://github.com/OpenEtherCATsociety/SOES/issues/151
    motor_configure_timing();

    /* wait for all peripherals to reach SAFE_OP state */
    if (ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * (N_MCs - 1)) != EC_STATE_SAFE_OP) {
        ec_mcp_state.status = ECAT_MOTOR_RUNNING_PARTIAL;
        blast_err("Not all peripherals reached safe operational state.");
        ec_readstate();
        for (int i = 1; i <= ec_periphcount; i++) {
            if (ec_periph[i].state != EC_STATE_SAFE_OP) {
                blast_err("Peripheral %d State=0x%.1x StatusCode=0x%.4x : %s", i, ec_periph[i].state,
                        ec_periph[i].ALstatuscode, ec_ALstatuscode2string(ec_periph[i].ALstatuscode));
            }
        }
    } else {
        ec_mcp_state.status = ECAT_MOTOR_RUNNING;
    }
    ec_mcp_state.periph_count = ec_periphcount;
    for (int i = 1; i <= ec_periphcount; i++) {
        controller_state[i].index = i;
        /**
         * Configure the index values for later use.  These are mapped to the hard-set
         * addresses on the motor controllers (look for the dials on the side)
         */
        int32_t serial = 0;
        int size = 4;
        ec_SDOread(i, 0x2384, 1, false, &size, &serial, EC_TIMEOUTRXM);
        if (serial == RW_SN) {
            blast_startup("Reaction Wheel Motor Controller %d: %s: SN: 0x%.4x",
                          ec_periph[i].aliasadr, ec_periph[i].name, serial);
            rw_index = i;
            blast_info("Setting rw_index to %d", rw_index);
            blast_info("ec_periph[%d].outputs = %p", i, ec_periph[i].outputs);
           controller_state[i].is_mc = 1;
        } else if (serial == PIV_SN) {
            blast_startup("Pivot Motor Controller %d: %s: SN: 0x%.4x",
                          ec_periph[i].aliasadr, ec_periph[i].name, serial);
            piv_index = i;
            blast_info("Setting piv_index to %d", piv_index);
            blast_info("ec_periph[%d].outputs = %p", i, ec_periph[i].outputs);
            controller_state[i].is_mc = 1;
        } else if (serial == EL_SN) {
            blast_startup("Elevation Motor Controller %d: %s: SN: 0x%.4x",
                          ec_periph[i].aliasadr, ec_periph[i].name, serial);
            el_index = i;
            controller_state[i].is_mc = 1;
            blast_info("Setting el_index to %d", el_index);
            blast_info("ec_periph[%d].outputs = %p", i, ec_periph[i].outputs);
        } else {
            blast_warn("Got unknown MC %s at position %d with serial 0x%.4x and alias %d",
                       ec_periph[i].name, ec_periph[i].configadr, serial, ec_periph[i].aliasadr);
            controller_state[i].ec_unknown = 1;
        }
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    return ec_periphcount;
}


/**
 * @brief Configure the PDO assignments.  PDO (Process Data Objects) are sent on configured SYNC cycles
 * as a static block of up to 8 bytes each.  Locally, we map these objects to a chunk of memory
 * that can be read or modified outside of the loop that send these objects to the controllers.
 * 
 * @details PDO mapping happens in two halves: data received by the
 * peripherals, and data received by the flight computer(s). Tx/Rx are from the
 * perspective of the peripheral. In this function, first we configure Tx, 
 * making sure each PDO map is cleared before loading new objects into it.
 * Then we clear and configure Rx. After each PDO we map, we check for mapping
 * errors. Finally, we save the data to the controller.
 * For all the gory details, see the Copley EtherCAT User Guide (16-01450).
 * In general, when we run mcp, we have already saved these parameters to the
 * FLASH of the controller before flight, but this function acts to ensure a
 * correct mapping upon startup, should the flash be corrupted.
 *
 * @param m_periph Motor controller index number to configure.
 * @return -1 on failure otherwise 0
 */
static int motor_pdo_init(int m_periph)
{
    pdo_mapping_t map;
    int retval = 0;

    if (ec_periph[m_periph].state != EC_STATE_SAFE_OP && ec_periph[m_periph].state != EC_STATE_PRE_OP) {
        blast_err("Motor Controller %d (%s) is not in pre-operational state! State is %d, cannot configure.",
                  m_periph, ec_periph[m_periph].name, ec_periph[m_periph].state);
        return -1;
    }

    blast_startup("Configuring PDO Mappings for controller %d (%s)", m_periph, ec_periph[m_periph].name);

    // ========================================================================
    // Configure data transmitted by peripherals, received by flight computer.
    // Un-fixed (user editable) TxPDOs: choose the data you want the FC to get.
    // TxPDO objects {0x1a00, 0x1a01, 0x1a02, 0x1a03}
    // ========================================================================
    // To program the TxPDO mapping, we first must clear the old state
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_ASSIGNMENT, 0, 0)) {
        blast_err("Failed mapping!");
    }
    for (int i = 0; i < 4; i++) {
        if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_MAPPING + i, 0, 0)) {
            blast_err("Failed mapping!");
        }
    }

    // 0x1a00: encoder-reported position, actual velocity
    map_pdo(&map, ECAT_MOTOR_POSITION, 32);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING, 1, map.val)) {
        blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_VEL_ACTUAL, 32);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING, 2, map.val)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of elements we have stored
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_MAPPING, 0, 2)) {
        blast_err("Failed mapping!");
    }
    // 0x1a00 maps to the first PDO
    if (!ec_SDOwrite16(m_periph, ECAT_TXPDO_ASSIGNMENT, 1, ECAT_TXPDO_MAPPING)) {
        blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    // 0x1a01: actual position, control word, network status
    // Actual Position (for el, the position reported by an encoder not
    // directly connected to the motor shaft (the "load" encoder). Otherwise,
    // contains same data as ECAT_MOTOR_POSITION.
    map_pdo(&map, ECAT_ACTUAL_POSITION, 32);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING+1, 1, map.val)) {
       blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_CTL_WORD, 16);
    retval = ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 1, 2, map.val);
    if (!retval) {
        blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_NET_STATUS, 16); // (includes heartbeat monitor)
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 1, 3, map.val)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of elements we have stored
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_MAPPING + 1, 0, 3)) {
        blast_err("Failed mapping!");
    }
    // 0x1a01 maps to the second PDO
    if (!ec_SDOwrite16(m_periph, ECAT_TXPDO_ASSIGNMENT, 2, ECAT_TXPDO_MAPPING + 1)) {
        blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    // 0x1a02: event status register, status word, drive temp
    // Manufacturer Status Register
    // Some extra notes here because this register is important.
    // See Handling Faults in CANopen and EtherCAT (Copley AN111)
    // Related to Latching Fault Status Register (0x2183);
    // whenever that 16-bit register does not equal zero, a Latching Fault
    // occurs. Latching Faults force the drive to disable and set bit 3 of the
    // Status Word.
    map_pdo(&map, ECAT_DRIVE_STATUS, 32);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 2, 1, map.val)) {
        blast_err("Failed mapping!");
    }
    // Status Word
    // Some extra notes here because this register is important.
    // See e.g. Beckhoff Information System entry on 6041.
    // Bit 1 indicates switched on, bit 3 being set indicates a fault, etc.
    map_pdo(&map, ECAT_CTL_STATUS, 16);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 2, 2, map.val)) {
        blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_DRIVE_TEMP, 16); // (deg C)
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 2, 3, map.val)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of elements we have stored
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_MAPPING + 2, 0, 3)) {
        blast_err("Failed mapping!");
    }
    // 0x1a02 maps to the third PDO
    if (!ec_SDOwrite16(m_periph, ECAT_TXPDO_ASSIGNMENT, 3, ECAT_TXPDO_MAPPING + 2)) {
        blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    // 0x1a03: latching fault status, actual motor current, motor commutation
    // angle
    map_pdo(&map, ECAT_LATCHED_DRIVE_FAULT, 32);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 3, 1, map.val)) {
        blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_CURRENT_ACTUAL, 16);
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 3, 2, map.val)) {
        blast_err("Failed mapping!");
    }
    // map_pdo(&map, ECAT_PHASE_ANGLE, 16); // Motor phase angle
    map_pdo(&map, ECAT_COMMUTATION_ANGLE, 16); // Commutation angle
    if (!ec_SDOwrite32(m_periph, ECAT_TXPDO_MAPPING + 3, 3, map.val)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of elements we have stored
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_MAPPING + 3, 0, 3)) {
        blast_err("Failed mapping!");
    }
    // 0x1a03 maps to the fourth PDO
    if (!ec_SDOwrite16(m_periph, ECAT_TXPDO_ASSIGNMENT, 4, ECAT_TXPDO_MAPPING + 3)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of maps we have used in the TxPDO
    if (!ec_SDOwrite8(m_periph, ECAT_TXPDO_ASSIGNMENT, 0, 4)) {
        blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    // blast_info("ec_periph[%d].outputs = %p", m_periph, ec_periph[m_periph].outputs);

    // ========================================================================
    // Configure data transmitted by flight computer, received by peripherals.
    // Un-fixed (user editable) RxPDOs: choose the data you want the
    // peripherals to get.
    // RxPDO objects {0x1600, 0x1601, 0x1602, 0x1603}
    // ========================================================================
    // To program the PDO mapping, we first must clear the old state
    ec_SDOwrite8(m_periph, ECAT_RXPDO_ASSIGNMENT, 0, 0);
    for (int i = 0; i < 4; i++) {
        if (!ec_SDOwrite8(m_periph, ECAT_RXPDO_MAPPING + i, 0, 0)) {
            blast_err("Failed mapping!");
        }
    }

    // 0x1600: control word, target current, latched faults
    map_pdo(&map, ECAT_CTL_WORD, 16);
    if (!ec_SDOwrite32(m_periph, ECAT_RXPDO_MAPPING, 1, map.val)) {
        blast_err("Failed mapping!");
    }
    map_pdo(&map, ECAT_CURRENT_LOOP_CMD, 16);
    if (!ec_SDOwrite32(m_periph, ECAT_RXPDO_MAPPING, 2, map.val)) {
        blast_err("Failed mapping!");
    }
    // map_pdo(&map, ECAT_LATCHED_DRIVE_FAULT, 32);
    // if (!ec_SDOwrite32(m_periph, ECAT_RXPDO_MAPPING, 3, map.val)) {
    //     blast_err("Failed mapping!");
    // }
    // Uncomment this once the phasing angle readout has been debugged
    // map_pdo(&map, ECAT_PHASING_MODE, 16);    // Phasing Mode
    // if (!ec_SDOwrite32(m_periph, ECAT_RXPDO_MAPPING, 3, map.val)) {
    //     blast_err("Failed mapping!");
    // }
    // Convey the number of elements we have stored
    if (!ec_SDOwrite8(m_periph, ECAT_RXPDO_MAPPING, 0, 3)) {
        blast_err("Failed mapping!");
    }
    // Set the 0x1600 map to the first PDO
    if (!ec_SDOwrite16(m_periph, ECAT_RXPDO_ASSIGNMENT, 1, ECAT_RXPDO_MAPPING)) {
        blast_err("Failed mapping!");
    }
    // Convey the number of maps we have used in the RxPDO
    if (!ec_SDOwrite8(m_periph, ECAT_RXPDO_ASSIGNMENT, 0, 1)) {
            blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    // MISC settings for aborting trajectory, saving PDO map to onboard file
    if (!ec_SDOwrite32(m_periph, 0x2420, 0, 8)) {
        blast_err("Failed mapping!");
    }

    // ========================================================================
    // Save all objects (the 0x65766173 is hex for 'save')
    // ========================================================================
    if (!ec_SDOwrite32(m_periph, 0x1010, 1, 0x65766173)) {
        blast_err("Failed mapping!");
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }

    return 0;
}


/**
 * @brief Generic mapping function for ethercat peripheral PDO variables.  This function works with
 * motor_pdo_init to set up the PDO input/output memory mappings for the motor controllers
 * @param m_index position on the ethercat chain
 */
static void map_index_vars(int m_index)
{
    bool found;
    GSList *test;
    test = pdo_list[m_index];
    /**
     * Inputs.  Each is sequentially mapped to the IOMap memory space
     * for the motor controller
     */
    //  blast_info("Starting map_index_vars for index %d", m_index);
    //  blast_info("Initial pdolist pointer: %p", test);
#define PDO_SEARCH_LIST(_obj, _map) { \
    found = false; \
    for (GSList *el = pdo_list[m_index]; (el); el = g_slist_next(el)) { \
        pdo_channel_map_t *ch = (pdo_channel_map_t*)el->data; \
        if (ch->index == object_index(_obj) && \
                ch->subindex == object_subindex(_obj)) { \
            _map[m_index] = (typeof(_map[0])) (ec_periph[m_index].inputs + ch->offset); \
            found = true; \
        } \
    } \
    if (!found) { \
        blast_err("Could not find PDO map for index %d channel index = 0x%.2x, subindex = %d", \
                   m_index, object_index(_obj),  object_subindex(_obj)); \
    } \
    }
    if (controller_state[m_index].is_mc) {
        PDO_SEARCH_LIST(ECAT_MOTOR_POSITION, motor_position);
        PDO_SEARCH_LIST(ECAT_VEL_ACTUAL, motor_velocity);
        PDO_SEARCH_LIST(ECAT_ACTUAL_POSITION, actual_position);
        PDO_SEARCH_LIST(ECAT_DRIVE_STATUS, status_register);
        PDO_SEARCH_LIST(ECAT_CTL_STATUS, status_word);
        PDO_SEARCH_LIST(ECAT_DRIVE_TEMP, amp_temp);
        PDO_SEARCH_LIST(ECAT_LATCHED_DRIVE_FAULT, latched_register);
        PDO_SEARCH_LIST(ECAT_CURRENT_ACTUAL, motor_current);
//        PDO_SEARCH_LIST(ECAT_PHASE_ANGLE, phase_angle);
        PDO_SEARCH_LIST(ECAT_COMMUTATION_ANGLE, phase_angle);
        PDO_SEARCH_LIST(ECAT_NET_STATUS, network_status_word);
        PDO_SEARCH_LIST(ECAT_CTL_WORD, control_word_read);
        while (ec_iserror()) {
            blast_err("%s", ec_elist2string());
        }
    }
#undef PDO_SEARCH_LIST

    // TODO(seth): Add dynamic mapping to outputs
    /// Outputs
    if (controller_state[m_index].is_mc) {
        control_word[m_index] = (uint16_t*) (ec_periph[m_index].outputs);
        target_current[m_index] = (int16_t*) (control_word[m_index] + 1);
        if (!(ec_periph[m_index].outputs)) {
            blast_err("Error: IOmap was not configured correctly!"
                "Setting periph_error = 1 for peripheral %d...", m_index);
            controller_state[m_index].periph_error = 1;
        } else {
            controller_state[m_index].periph_error = 0;
        }
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}


/**
 * @brief Interface function to @map_index_vars.  Maps the variables for each of the motor
 * controllers found on the bus.  If the motor controller is not found, its map remains
 * attached to the @dummy_var position
 */
static void map_motor_vars(void)
{
    blast_info("Starting map_motor_vars.");
    if (el_index) {
        blast_info("mapping el_motors to index: %d", el_index);
        map_index_vars(el_index);
    }
    if (rw_index) {
        blast_info("mapping rw_motors to index: %d", rw_index);
        map_index_vars(rw_index);
    }
    if (piv_index) {
        blast_info("mapping piv_motors to index: %d", piv_index);
        map_index_vars(piv_index);
    }

    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}



/**
 * @brief sets the motor controller to operational state
 * 
 * @return int -1 on failure, 0 on success
 */
static int motor_set_operational(void)
{
    /* send one processdata cycle to init SM in peripherals */
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    ec_periph[0].state = EC_STATE_OPERATIONAL;

    /* send one valid process data to make outputs in peripherals happy*/
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* request OP state for all peripherals */
    ec_writestate(0);

    // Kick off thread for ec_send_processdata and ec_receive_processdata
    // to allow peripherals to sync
    ecat_do_sendrecv = 1;

    /* wait for all peripherals to reach OP state */
    ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);

    /**
     * If we've reached fully operational state, return
     */
    if (ec_periph[0].state == EC_STATE_OPERATIONAL) {
        ecat_in_op_state = 1;
        blast_info("We have reached a fully operational state.");
        ec_mcp_state.status = ECAT_MOTOR_RUNNING;
        return 0;
    }

    /**
     * Something has prevented a motor controller from entering operational mode (EtherCAT Ops)
     */
    for (int i = 1; i <= ec_periphcount; i++) {
        if (ec_periph[i].state != EC_STATE_OPERATIONAL) {
            blast_err("Peripheral %d State=0x%.1x StatusCode=0x%.4x : %s", i, ec_periph[i].state,
                    ec_periph[i].ALstatuscode, ec_ALstatuscode2string(ec_periph[i].ALstatuscode));
        }
    }
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
    return -1;
}


/**
 * @brief Checks to see if the ethercat network for a specified controller is ready.
 * 
 * @param index which MC to check
 * @return uint8_t 0 on failure or 1 on success
 */
static uint8_t check_ec_ready(int index)
{
    if ((index <= 0) || (index >= N_MCs)) {
        return(0);
    }
    if (!controller_state[index].comms_ok) {
        return(0);
    }
    if (!controller_state[index].is_mc) {
        return(0);
    }
    if (check_peripheral_comm_ready(index)) {
        if (!((*control_word_read[index]) == (*control_word[index])) ||
               (*status_register[el_index] & ECAT_STATUS_PHASE_UNINIT) ||
               !(*status_word[index] & ECAT_CTL_STATUS_READY) ) {
            return(0);
        }
    } else {
        return 0;
    }

    return(1);
}


/**
 * @brief LMF note: Deprecated this doesn't seem to work in the new ethercat configuration.
 * 
 * @param net_status bitmap of the ethercat network status
 * @param firsttime used to print failures only once
 * @return uint8_t 1 on failure, 0 on success
 */
static uint8_t check_for_network_problem(uint16_t net_status, bool firsttime)
{
    if (firsttime) {
        blast_info("net_status = 0x%.2x", net_status);
    }
    // Device is not in operational mode (bits 0 and 1 have value of 3)
    if (!(net_status & ECAT_NET_NODE_CHECK)) {
        if (firsttime) {
            blast_info("Device is not in operational mode.");
        }
        return(1);
    }
    if (!(net_status & ECAT_NET_SYNC_CHECK)) {
        if (firsttime) {
            blast_info("Device network error.");
        }
        return(1);
    }
    if (!(net_status & ECAT_NET_COBUS_OFF_CHECK)) {
        if (firsttime) {
            blast_info("Network CANOPEN Bus is off.");
        }
        return(1);
    }
    if (firsttime) {
        blast_info("No error in network status %u, returning 0.", net_status);
    }
    return(0);
}


/**
 * @brief Wrapper function around ethercat ready check for the elevation motor
 * 
 * @return uint8_t 1 on success, 0 on failure
 */
uint8_t is_el_motor_ready() {
    return(check_ec_ready(el_index));
}


/**
 * @brief Wrapper function around ethercat ready check for the RW motor
 * 
 * @return uint8_t 1 on success, 0 on failure
 */
uint8_t is_rw_motor_ready() {
    return(check_ec_ready(rw_index));
}


/**
 * @brief Wrapper function around ethercat ready check for the pivot motor
 * 
 * @return uint8_t 1 on success, 0 on failure
 */
uint8_t is_pivot_motor_ready() {
    return(check_ec_ready(piv_index));
}


/**
 * @brief logging function that checks all motor data and writes it to the local motor structures
 * 
 */
static void read_motor_data()
{
    int motor_i = motor_index;
    static bool firsttime = 1;
    RWMotorData[motor_i].current = rw_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    RWMotorData[motor_i].drive_info = rw_get_status_word();
    RWMotorData[motor_i].fault_reg = rw_get_latched();
    RWMotorData[motor_i].ALstatuscode = rw_get_ALstatuscode();
    RWMotorData[motor_i].ALstate = rw_get_ALstate();
    RWMotorData[motor_i].status = rw_get_status_register();
    RWMotorData[motor_i].network_status = rw_get_network_status_word();
    RWMotorData[motor_i].position = rw_get_position();
    RWMotorData[motor_i].motor_position = rw_get_position();
    RWMotorData[motor_i].temp = rw_get_amp_temp();
    RWMotorData[motor_i].velocity = rw_get_velocity();
    RWMotorData[motor_i].control_word_read = rw_get_ctl_word();
    RWMotorData[motor_i].control_word_write = rw_get_ctl_word_write();
    RWMotorData[motor_i].network_problem = !is_rw_motor_ready();
    RWMotorData[motor_i].phase_angle = rw_get_phase_angle()/65536.0*360.0;
    RWMotorData[motor_i].phase_mode = rw_get_phase_mode();

    ElevMotorData[motor_i].current = el_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    ElevMotorData[motor_i].drive_info = el_get_status_word();
    ElevMotorData[motor_i].fault_reg = el_get_latched();
    ElevMotorData[motor_i].ALstatuscode = el_get_ALstatuscode();
    ElevMotorData[motor_i].ALstate = el_get_ALstate();
    ElevMotorData[motor_i].status = el_get_status_register();
    ElevMotorData[motor_i].network_status = el_get_network_status_word();
    ElevMotorData[motor_i].position = el_get_position();
    ElevMotorData[motor_i].motor_position = el_get_motor_position();
    ElevMotorData[motor_i].temp = el_get_amp_temp();
    ElevMotorData[motor_i].velocity = el_get_velocity();
    ElevMotorData[motor_i].control_word_read = el_get_ctl_word();
    ElevMotorData[motor_i].control_word_write = el_get_ctl_word_write();
    ElevMotorData[motor_i].network_problem  = !is_el_motor_ready();
    ElevMotorData[motor_i].phase_angle = el_get_phase_angle();
    ElevMotorData[motor_i].phase_mode = el_get_phase_mode();

    PivotMotorData[motor_i].current = piv_get_current() / 100.0; /// Convert from 0.01A in register to Amps
    PivotMotorData[motor_i].drive_info = piv_get_status_word();
    PivotMotorData[motor_i].fault_reg = piv_get_latched();
    PivotMotorData[motor_i].ALstatuscode = piv_get_ALstatuscode();
    PivotMotorData[motor_i].ALstate = piv_get_ALstate();
    PivotMotorData[motor_i].status = piv_get_status_register();
    PivotMotorData[motor_i].network_status = piv_get_network_status_word();
    PivotMotorData[motor_i].position = piv_get_position();
    PivotMotorData[motor_i].temp = piv_get_amp_temp();
    PivotMotorData[motor_i].velocity = piv_get_velocity();
    PivotMotorData[motor_i].control_word_read = piv_get_ctl_word();
    PivotMotorData[motor_i].control_word_write = piv_get_ctl_word_write();
    PivotMotorData[motor_i].network_problem  = !is_pivot_motor_ready();
    PivotMotorData[motor_i].phase_angle = piv_get_phase_angle();
    PivotMotorData[motor_i].phase_mode = piv_get_phase_mode();

    firsttime = 0;
    motor_index = INC_INDEX(motor_index);
}


/**
 * @brief Takes the PDOs and susses out the mapping for later use
 * 
 * @param m_periph which controller index to talk to
 */
void mc_readPDOassign(int m_periph) {
    uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
    uint8 subcnt;
//    GSList *m_pdo_list;
    int wkc = 0;
    int len = 0;
    int offset = 0;

    len = sizeof(rdat);
    rdat = 0;

//    m_pdo_list = pdo_list[m_periph];
//    blast_info("Starting mc_readPDOassign for peripheral %i", m_periph);
    /* read PDO assign subindex 0 ( = number of PDO's) */
    wkc = ec_SDOread(m_periph, ECAT_TXPDO_ASSIGNMENT, 0x00, FALSE, &len, &rdat, EC_TIMEOUTRXM);
    rdat = etohs(rdat);
    /* positive result from peripheral ? */
    blast_info("Result from ec_SDOread at index 0x%.4x, wkc = %i, len = %i, rdat = 0x%.4x",
               ECAT_TXPDO_ASSIGNMENT, wkc, len, rdat);
    if ((wkc <= 0) || (rdat <= 0))  {
        blast_info("no data returned from ec_SDOread ... returning.");
        return;
    }

    /* number of available sub indexes */
    nidx = rdat;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
        len = sizeof(rdat);
        rdat = 0;
        /* read PDO assign */
        wkc = ec_SDOread(m_periph, ECAT_TXPDO_ASSIGNMENT, (uint8) idxloop, FALSE, &len, &rdat, EC_TIMEOUTRXM);
        /* result is index of PDO */
        idx = etohl(rdat);
        if (idx <= 0) {
            continue;
        } else {
//            blast_info("found idx = 0x%.2x at wkc = %i, idxloop = %i", idx, wkc, idxloop);
        }
        len = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ec_SDOread(m_periph, idx, 0x00, FALSE, &len, &subcnt, EC_TIMEOUTRXM);
        subidx = subcnt;
//        blast_info("Number of subindexes: %i", subidx);
        /* for each subindex */
//        blast_info("Reading out the SDOs");
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
            pdo_channel_map_t *channel = NULL;
            pdo_mapping_t pdo_map = { 0 };
            len = sizeof(pdo_map);
            /* read SDO that is mapped in PDO */
            wkc = ec_SDOread(m_periph, idx, (uint8) subidxloop, FALSE, &len, &pdo_map, EC_TIMEOUTRXM);
            channel = malloc(sizeof(pdo_channel_map_t));
            channel->index = pdo_map.index;
            channel->subindex = pdo_map.subindex;
            channel->offset = offset;
            pdo_list[m_periph] = g_slist_prepend(pdo_list[m_periph], channel);
//            blast_info("Read SDO subidxloop = %i, wkc = %i, idx = %i, len = %i", subidxloop, wkc, idx, len);
//            blast_info("Appending channel to m_pdo_list = %p: index = 0x%.2x, subindex = %i, offset = %i",
//                       pdo_list[m_periph], channel->index, channel->subindex, channel->offset);

            /// Offset is the number of bytes into the memory map this element is.  First element is 0 bytes in.
            offset += (pdo_map.size / 8);
        }
    }
}


/**
 * @brief Attempts to connect to the RW ethercat device and set default parameters.
 * 
 */
void set_rw_motor_defaults()
{
    /// Configure the reaction wheel phasing
//    rw_init_phasing();

    /// Set the default current limits
    if (!CommandData.ec_devices.have_commutated_rw) {
        rw_init_encoder();
    }
    rw_init_current_limit();
    rw_init_current_pid();
    ec_init_heartbeat(rw_index);
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}


/**
 * @brief Attempts to connect to the elevation ethercat device and set default parameters.
 * 
 */
void set_el_motor_defaults()
{
    el_init_current_limit();
    el_init_current_pid();
    el_init_encoder();
    ec_init_heartbeat(el_index);
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}


/**
 * @brief Attempts to connect to the pivot ethercat device and set default parameters.
 * 
 */
void set_pivot_motor_defaults()
{
    piv_init_current_limit();
    piv_init_current_pid();
    piv_init_resolver();
    ec_init_heartbeat(piv_index);
    while (ec_iserror()) {
        blast_err("%s", ec_elist2string());
    }
}


/**
 * @brief wrapper around the individual default calls, sets defaults for all motors
 * current limits, readout,  and PID values.
 * 
 */
void set_ec_motor_defaults()
{
    set_rw_motor_defaults();
    set_el_motor_defaults();
    set_pivot_motor_defaults();
}


/**
 * @brief shuts down the ethercat motors
 * 
 * @return int 1 on use.
 */
int close_ec_motors()
{
    ec_close(); // Attempt to close down the motors
    return(1);
}


/**
 * @brief Attempts to connect to the EtherCat devices.
 * 
 * @return int 1 on call
 */
int configure_ec_motors()
{
    find_controllers();
    if (CommandData.ec_devices.rw_commutate_next_ec_reset) {
        CommandData.ec_devices.have_commutated_rw = 0;
        CommandData.ec_devices.rw_commutate_next_ec_reset = 0;
    }
    for (int i = 1; i <= ec_periphcount; i++) {
        if (controller_state[i].is_mc) {
            motor_pdo_init(i);
            mc_readPDOassign(i);
        }
    }
    /// We re-configure the map now that we have assigned the PDOs
    if (ec_config_map(&io_map) <= 0) blast_warn("Warning ec_config_map(&io_map) return null map size.");
    map_motor_vars();

    /**
     * Set the initial values of all three commands to "safe" default values
     */
    for (int i = 1; i <= ec_periphcount; i++) {
        if ((controller_state[i].is_mc) && !(controller_state[i].periph_error)) {
            *target_current[i] = 0;
            *control_word[i] = ECAT_CTL_ON | ECAT_CTL_ENABLE_VOLTAGE | ECAT_CTL_QUICK_STOP| ECAT_CTL_ENABLE;
        }
    }

    if (CommandData.disable_az) {
        rw_disable();
        piv_disable();
    } else {
        rw_enable();
        piv_enable();
    }
    if (CommandData.disable_el) {
        el_disable();
    } else {
        el_enable();
    }

    set_ec_motor_defaults();


    /// Put the motors in Operational mode (EtherCAT Operation)
    blast_info("Setting the EtherCAT devices in operational mode.");
    motor_set_operational();

    blast_info("Writing the drive state to start current mode.");
    for (int i = 1; i <= ec_periphcount; i++) {
        if ((controller_state[i].periph_error == 0) && (controller_state[i].has_dc == 1)) {
            controller_state[i].comms_ok = 1;
        } else {
            controller_state[i].comms_ok = 0;
        }
        if (controller_state[i].is_mc) {
            ec_SDOwrite16(i, ECAT_DRIVE_STATE, ECAT_DRIVE_STATE_PROG_CURRENT);
        }
    }
    // If things seem to be working then we don't need to recommutate the RW.
    if (controller_state[rw_index].comms_ok) {
        CommandData.ec_devices.have_commutated_rw = 1;
    }
    return(1);
}


/**
 * @brief resets the ethercat motor structures and indices for re-initialization.
 * 
 * @return int 1 on call
 */
int reset_ec_motors()
{
    int i = 0;
    rw_index = 0;
    el_index = 0;
    piv_index = 0;
    for (i = 1; i < N_MCs; i++) {
        controller_state[i].index = 0;
        controller_state[i].ec_unknown = 0;
        controller_state[i].is_mc = 0;
        controller_state[i].has_dc = 0;
        controller_state[i].comms_ok = 0;
        controller_state[i].periph_error = 0;
    }
    ecat_do_sendrecv = 0;
    ecat_in_op_state = 0;
    configure_ec_motors();
    return(1);
}


/**
 * @brief Checks the ethercat network for any issues and reports what issue exists if there is one.
 * 
 * @return int 1 on success, 0 on any failure.
 */
static int check_ec_network_status()
{
    static bool has_warned = 0;
    int retval = 1;
    int motor_i = motor_index;
    if ((!check_ec_ready(rw_index) && CommandData.ec_devices.fix_rw) ||
        (!check_ec_ready(el_index) && CommandData.ec_devices.fix_el) ||
        (!check_ec_ready(piv_index) && CommandData.ec_devices.fix_piv)) {
            retval = 0;
            if (!has_warned) {
                blast_warn("check_ec_network_status shows a network problem. RW = %u, El = %u, Piv = %u",
                           RWMotorData[motor_i].network_problem, ElevMotorData[motor_i].network_problem,
                           PivotMotorData[motor_i].network_problem);
                has_warned = 1;
            }
    } else {
       has_warned = 0;
    }
    return(retval);
}


/**
 * @brief shuts down the ethercat motor network and reports it to the log
 * 
 */
void shutdown_motors(void)
{
    blast_info("Shutting down motors");
    close_ec_motors();
    blast_info("Finished shutting down motors");
}


/**
 * @brief This thread is solely responsible for sending/receiving process data
 * from motor controllers at the prescribed cadence.
 * 
 * @details This takes after the SOEM example red_test.c. The benefit of a
 * separate thread for this task is that it allows us to start
 * sending/receiving process data without interruption before we have completed
 * EtherCAT state machine setup, via the ecat_do_sendrecv global toggle.
 * 
 * @return OSAL_THREAD_FUNC_RT
 */
OSAL_THREAD_FUNC_RT motor_send_recv(void) {
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = ecat_sendrecv_cadence_ns}; /// 500HZ interval
    int ret = 0;
    bool firsttime = true;

    nameThread("Motors send/recv");

    while (!InCharge) {
        osal_usleep(100000);
        if (firsttime) {
            blast_info("Not in charge. Waiting for control.");
            firsttime = false;
        }
    }

    ec_send_processdata();

    clock_gettime(CLOCK_REALTIME, &ts);
    while (!shutdown_mcp) {
        // Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);
        // Danger: check a global, allowing another thread to control this one
        if (ecat_do_sendrecv > 0) {
            ec_send_processdata();
            ecat_current_wkc = ec_receive_processdata(EC_TIMEOUTRET);
            if (ecat_current_wkc < ecat_expected_wkc) {
                bprintf(none, "Possible missing data in communicating with Motor Controllers");
            }
        }
    }
}

/**
 * @brief This thread is solely responsible for querying the EtherCAT state
 * machine to pick up drives that have been lost or experienced a fault.
 * 
 * @details The function takes after the SOEM example red_test.c function.
 * It does not need to be real-time, because we only need to occasionally
 * look for and rehab drives.
 * 
 * @return OSAL_THREAD_FUNC 
 */
OSAL_THREAD_FUNC ecatcheck(void)
{
    int periph_idx;
    bool firsttime = true;

    while (!InCharge) {
        osal_usleep(100000);
        if (firsttime) {
            blast_info("Not in charge. Waiting for control.");
            firsttime = false;
        }
    }

    while (!shutdown_mcp) {
        if (ecat_in_op_state && ((ecat_current_wkc < ecat_expected_wkc) || ec_group[0].docheckstate)) {
            // one or more peripherals are not responding
            ec_group[0].docheckstate = FALSE;
            ec_readstate();
            for (periph_idx = 1; periph_idx <= ec_periphcount; periph_idx++) {
                if ((ec_periph[periph_idx].group == 0) && (ec_periph[periph_idx].state != EC_STATE_OPERATIONAL)) {
                    ec_group[0].docheckstate = TRUE;
                    if (ec_periph[periph_idx].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        blast_err("ERROR : periph %d is in SAFE_OP + ERROR, attempting ack.\n", periph_idx);
                        ec_periph[periph_idx].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(periph_idx);
                    } else if (ec_periph[periph_idx].state == EC_STATE_SAFE_OP) {
                        blast_info("WARNING : periph %d is in SAFE_OP, change to OPERATIONAL.\n", periph_idx);
                        ec_periph[periph_idx].state = EC_STATE_OPERATIONAL;
                        ec_writestate(periph_idx);
                    } else if (ec_periph[periph_idx].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(periph_idx, EC_TIMEOUTMON)) {
                            ec_periph[periph_idx].islost = FALSE;
                            blast_info("MESSAGE : periph %d reconfigured\n", periph_idx);
                        }
                    } else if (!ec_periph[periph_idx].islost) {
                        // re-check state
                        ec_statecheck(periph_idx, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_periph[periph_idx].state == EC_STATE_NONE) {
                            ec_periph[periph_idx].islost = TRUE;
                            blast_err("ERROR : periph %d lost\n", periph_idx);
                        }
                    }
                }
                if (ec_periph[periph_idx].islost) {
                    if (ec_periph[periph_idx].state == EC_STATE_NONE) {
                        if (ec_recover_slave(periph_idx, EC_TIMEOUTMON)) {
                            ec_periph[periph_idx].islost = FALSE;
                            blast_info("MESSAGE : periph %d recovered\n", periph_idx);
                        }
                    } else {
                        ec_periph[periph_idx].islost = FALSE;
                        blast_info("MESSAGE : periph %d found\n", periph_idx);
                    }
                }
            }
            if (!ec_group[0].docheckstate) {
               blast_info("OK : all peripherals resumed OPERATIONAL.\n");
            }
        }
        osal_usleep(10000);
    }
}


/**
 * @brief Motor control thread, handles all aspects of running the pointing motors.
 * 
 * @param arg unused 
 * @return void* not used
 */
static void* motor_control(void* arg)
{
    int ret;
    struct timespec ts;
    struct timespec interval_ts = { .tv_sec = 0,
                                    .tv_nsec = 2000000}; /// 500HZ interval
    char name[16] = "enp1s0";
    bool firsttime = 1;

    nameThread("Motors config");
    while (!InCharge) {
        usleep(100000);
        if (firsttime) {
            blast_info("Not in charge.  Waiting for control.");
            firsttime = 0;
        }
    }
    blast_startup("Starting Motor Control");

    ph_thread_set_name("Motors config");

    if (!(ret = ec_init(name))) {
        berror(err, "Could not initialize %s, exiting.", name);
        exit(1);
    } else {
        blast_info("Initialized %s", name);
    }

    blast_info("Attempting configure to EtherCat devices.");
    configure_ec_motors();

    // Our work counter (WKC) provides a count of the number of items to handle.
    // Update the value of this global here, so it can be checked by the
    // send/recv thread.
    ecat_expected_wkc = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    blast_info("ecat_expected_wkc = %i", ecat_expected_wkc);

    clock_gettime(CLOCK_REALTIME, &ts);
    while (!shutdown_mcp) {
        /// Set our wakeup time
        ts = timespec_add(ts, interval_ts);
        ret = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        if (!check_ec_network_status()) {
            ec_mcp_state.network_error_count++;
            if (((ec_mcp_state.network_error_count) % 1000) == 1) {
                blast_info("network_error_count = %u", ec_mcp_state.network_error_count);
            }
            if (ec_mcp_state.network_error_count >= NETWORK_ERR_RESET_THRESH) {
                ec_mcp_state.network_error_count = 0;
                if (!reset_ec_motors()) {
                    blast_err("Reset of EtherCat devices failed!");
                } else {
                    blast_info("Reset EtherCat connection.");
                }
            }
        }
        if (CommandData.ec_devices.reset) {
            if (!reset_ec_motors()) {
                blast_err("Reset of EtherCat devices failed!");
            }
            CommandData.ec_devices.reset = 0;
        }
        if (!CommandData.ec_devices.have_commutated_rw) {
            set_rw_motor_defaults();
            CommandData.ec_devices.have_commutated_rw = 1;
        }
        if (CommandData.disable_az) {
            rw_disable();
            piv_disable();
        } else {
            rw_enable();
            piv_enable();
        }
        if (CommandData.disable_el) {
            el_disable();
        } else {
            el_enable();
        }

        if (ret && ret != -EINTR) {
            blast_err("error while sleeping, code %d (%s)\n", ret, strerror(-ret));
            break;
        }

        // Useful for debugging ERR indicator on drives
        // ec_readstate();
        // for (int i = 1; i <= ec_periphcount; i++) {
        //     if (ec_periph[i].state != EC_STATE_OPERATIONAL) {
        //         blast_err("Peripheral %d State=0x%.1x StatusCode=0x%.4x : %s", i, ec_periph[i].state,
        //                 ec_periph[i].ALstatuscode, ec_ALstatuscode2string(ec_periph[i].ALstatuscode));
        //     }
        // }

        read_motor_data();

        while (ec_iserror()) {
            blast_err("%s", ec_elist2string());
        }
    }
    shutdown_motors();

    return 0;
}


/**
 * @brief initializes the motor control structures and then spawns the motor controlling thread.
 * 
 * @return int 0 on finish
 */
int initialize_motors(void)
{
    memset(ElevMotorData, 0, sizeof(ElevMotorData));
    memset(RWMotorData, 0, sizeof(RWMotorData));
    memset(PivotMotorData, 0, sizeof(PivotMotorData));

    motor_ctl_id =  ph_thread_spawn(motor_control, NULL);

    // set up another thread to handle sending/receiving process data at a
    // higher priority
    int sendrecv_ret = osal_thread_create_rt(&ecat_sendrecv_thread, ECAT_SENDRECV_STACKSIZE, &motor_send_recv, NULL);
    if (1 != sendrecv_ret) {
        berror(err, "Could not initialize motor communication thread, exiting.");
        exit(1);
    }

    // set up a third thread to handle peripheral errors in OP state
    int check_ret = osal_thread_create(&ecat_check_thread, ECAT_SENDRECV_STACKSIZE, &ecatcheck, NULL);
    if (1 != check_ret) {
        berror(err, "Could not initialize motor health monitoring thread.");
    }

    return 0;
}


/**
 * @brief Makes a bitmap out of the ethercat motor controller status fields
 * 
 * @param m_index which controller to refer to
 * @return uint8_t bitmap of the ethercat status fields
 */
uint8_t make_ec_status_field(int m_index)
{
    uint8_t m_stats = 0;
    if ((m_index < 1) || (m_index >= N_MCs)) {
        return m_stats;
    }
    m_stats |= (m_index & 0x07);
    m_stats |= ((controller_state[m_index].comms_ok & 0x01) << 3);
    m_stats |= ((controller_state[m_index].periph_error & 0x01) << 4);
    m_stats |= ((controller_state[m_index].has_dc & 0x01) << 5);
    m_stats |= ((controller_state[m_index].is_mc & 0x01) << 6);
    return m_stats;
}


/**
 * @brief Initializes the channel pointers and stores the 1Hz data from the ethercat network.
 * 
 */
void store_1hz_ethercat(void)
{
    static int firsttime = 1;
    static channel_t *NFoundECAddr;
    static channel_t *PeripheralCountECAddr;
    static channel_t *StatusECAddr;
    static channel_t *StatusECRWAddr;
    static channel_t *StatusECElAddr;
    static channel_t *StatusECPivAddr;

    if (firsttime) {
        NFoundECAddr = channels_find_by_name("n_found_ec");
        PeripheralCountECAddr = channels_find_by_name("periph_count_ec");
        StatusECAddr = channels_find_by_name("status_ec");
        StatusECRWAddr = channels_find_by_name("status_ec_rw");
        StatusECElAddr = channels_find_by_name("status_ec_el");
        StatusECPivAddr = channels_find_by_name("status_ec_piv");
        firsttime = 0;
    }
    SET_UINT8(NFoundECAddr, ec_mcp_state.n_found);
    SET_UINT8(PeripheralCountECAddr, ec_mcp_state.periph_count);
    SET_UINT8(StatusECAddr, ec_mcp_state.status);
    if (rw_index) SET_UINT8(StatusECRWAddr, make_ec_status_field(rw_index));
    if (el_index) SET_UINT8(StatusECElAddr, make_ec_status_field(el_index));
    if (piv_index) SET_UINT8(StatusECPivAddr, make_ec_status_field(piv_index));
}

