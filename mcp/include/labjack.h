/**
 * @file data_sharing.h
 *
 * @date Dec 25, 2012
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2016 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_LABJACK_H_
#define INCLUDE_LABJACK_H_

#include <stdint.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

// Breakdown of the numbering in state[m_labjack] for the non-multiplexed labjacks
#define LABJACK_OF_POWER 0
#define LABJACK_IF_POWER 1
#define LABJACK_MOTOR_POWER 2
#define LABJACK_OF_2 3
#define LABJACK_OF_3 4
#define LABJACK_HIGHBAY 7
#define LABJACK_MICROSCROLL 9
#define LABJACK_IR 10


#define LABJACK_CRYO_NCHAN 14 // Number of Channels to stream (14 = all analog input channels)
#define LABJACK_CRYO_SPP 1 // Number of scans to readout per streaming packet

// DACS
#define DAC0 1000
#define DAC1 1002

void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet);
float labjack_get_value(int m_labjack, int m_channel);
ph_thread_t* initialize_labjack_commands(int m_which);
void store_labjack_data(void);
void labjack_test_dac(float v_value, int m_labjack);
int labjack_dio(int m_labjack, int address, int command);
void heater_write(int m_labjack, int address, float command);
uint16_t labjack_read_dio(int m_labjack, int address);
void labjack_reboot(int m_labjack);
void labjack_queue_command(int, int, float);
void query_time(int m_labjack);
void initialize_labjack_queue(void);
void labjack_choose_execute(void);
void init_labjack_digital(void);
void set_execute(int which);
void init_labjacks(int set_1, int set_2, int set_3, int set_4, int set_5, int q_set);
void set_reconnect(int which);
#endif /* LABJACK_H_ */
