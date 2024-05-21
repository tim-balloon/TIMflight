/**
 * @file inclinometer.h
 *
 * @date Nov 23, 2015
 * @author James & Juzz
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
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

#ifndef INCLUDE_INCLINOMETER_H_
#define INCLUDE_INCLINOMETER_H_

// Protocol constants:

#define INC_ID_IDX 0U
#define INC_LENGTH_BYTE_IDX 1U
#define INC_ADDR_BYTE_IDX 2U
#define INC_CMD_BYTE_IDX 3U
#define INC_CMD_VAL_BYTE_IDX 4U
#define INC_MEMS_ID 0x68

// Device specifics:

#define INC_DEV_ADDR 0x01

// Message specifics:
#define INC_CMD_MSG_LEN 0x05 // All command messages we use happen to have the same len
#define INC_CMD_RESP_MSG_LEN 0x05

// set address

#define INC_CMD_SET_ADDR 0x0F // Set device address
#define INC_CMD_SET_ADDR_TGT 0xFF // All devices on RS-422 network
#define INC_CMD_SET_ADDR_VAL 0x01 // Adopt address 0x01

// command baud

#define INC_CMD_BAUD 0x0B // Set baud rate
#define INC_CMD_BAUD_VAL 0x05 // 115200

// command mode change

#define INC_CMD_DATA_MODE 0x0C // automatic reporting
#define INC_CMD_REPORT_MODE 0x04 // for some reason, the auto reporting mode replies with 0x84
#define INC_CMD_DATA_MODE_VAL_STOP 0x00 // Only when queried
#define INC_CMD_DATA_MODE_VAL_5HZ 0x01 // 5 Hz

// receive data
#define INC_DATA_RESP_MSG_LEN 0x0D
#define INC_DATA_RESP_BUF_LEN 14U
#define INC_DATA_HEADER_LEN 4U
#define INC_DATA_RESP_IDX_X 4U
#define INC_DATA_RESP_IDX_Y 7U
#define INC_DATA_RESP_IDX_T 10U

void initialize_inclinometer(void);
void *monitor_inclinometer(void *m_arg);

void store_1hz_inc(void);

#endif /* INCLUDE_inclinometer_H_ */
