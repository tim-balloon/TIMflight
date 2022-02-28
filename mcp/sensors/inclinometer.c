/**
 * @file inclinometer.c
 *
 * @date Feb 22 2022
 * @author Juzz
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

#include <stdint.h>
#include <endian.h>
#include <errno.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/serial.h"
#include "phenom/memory.h"

#include "blast.h"
#include "channels_tng.h"
#include "inclinometer.h"   // Doesn't exist yet.
#include "mcp.h"
#include "pointing_struct.h"
#include "command_struct.h"

#define INCCOM "/dev/ttyINC"

#define INC_ERR_THRESHOLD 1000
#define INC_TIMEOUT_THRESHOLD 10
#define INC_RESET_THRESHOLD 50

extern int16_t SouthIAm; // defined in mcp.c

int inc_verbose_level = 0;
ph_serial_t *inc_comm = NULL;

typedef enum {
    // INC_WE_BIN = 0,
	// INC_BIN,
	// INC_WE_RATE,
	INC_RATE,
	INC_CONT,
	INC_READ,
	INC_END
} e_inc_state;

typedef struct {
	char cmd[32];
	char resp[16];
} inc_state_cmd_t;          // Command struct does expect cmd and response.

typedef enum {
    INC_BOOT = 0,
    INC_INIT,
    INC_READING,
    INC_ERROR,
    INC_RESET,
    INC_POWERCYCLE
} e_inc_status_t;

typedef struct {
	e_inc_state cmd_state;
	e_inc_status_t status;
	uint16_t err_count;
	uint16_t error_warned;
	uint16_t timeout_count;
	uint16_t reset_count;
} inc_state_t;

inc_state_t inc_state = {0};

// static inc_state_cmd_t state_cmd[INC_END] = {
// 		[INC_WE_BIN] = { "*99WE", "OK" },
// 		[INC_BIN] = { "*99A", "ASCII ON" },
// 		[INC_WE_RATE] = { "*99WE", "OK" },
// 		[INC_RATE] = { "*99R=20", "OK" },
// 		[INC_CONT] = { "*99C" },
// };

    static inc_state_cmd_t state_cmd[INC_END] = {
        // DMH-2-60-422 commands follow {DMH ID, Msg Len [len-chksum],command,
        // command argument, checksum} // I'm unconfident that I'm using chksum correctly.
        [INC_RATE] = {{0x68, 0x05, 0xFF, 0x0B, 0x05, 0x04}, {0x68, 0x10, 0xFF, 0x8B, 0x05, 0x0F}},
        [INC_CONT] = {{0x68, 0x05, 0xFF, 0x0C, 0x05, 0x04}, {0x68, 0x05, 0xFF, 0x8B, 0x05, 0x04}},
    };

static void inc_set_framedata(int16_t m_incx, int16_t m_incy, int16_t m_incz)
{
    static channel_t *inc_x_channel = NULL;
    static channel_t *inc_y_channel = NULL;
    static channel_t *inc_temp_channel = NULL;

// Since each flight computer has its own inclinometer we only want to write to the channels
// corresponding to that computer's inclinometer.
    static uint8_t inc_index = 0;
    static int firsttime = 1;

    if (firsttime) {
        inc_index = SouthIAm;
        if (inc_index == 0) { // We are North (fc1)
            inc_x_channel = channels_find_by_name("x_inc1_n");
            inc_y_channel = channels_find_by_name("y_inc1_n");
            inc_temp_channel = channels_find_by_name("z_inc1_n");
        } else { // We are South (fc2)
            inc_x_channel = channels_find_by_name("x_inc2_s");
            inc_y_channel = channels_find_by_name("y_inc2_s");
            inc_temp_channel = channels_find_by_name("z_inc2_s");
        }
        firsttime = 0;
    }

    SET_SCALED_VALUE(inc_x_channel, ((double)m_incx)/1000.0);
    SET_SCALED_VALUE(inc_y_channel, ((double)m_incy)/1000.0);
    SET_SCALED_VALUE(inc_temp_channel, ((double)m_incz)/1000.0);
}

static void inc_get_data(char *inc_buf, size_t len_inc_buf)
{
    static int have_warned = 0;
    static int firsttime = 1;
    char x2[2], x3[2], x4[2], x5[2], x6[2];
    char y2[2], y3[2], y4[2], y5[2], y6[2];
    char z2[2], z3[2], z4[2], z5[2], z6[2];
    char xsn, ysn, zsn;
    x2[1] = x3[1] = x4[1] = x5[1] = x6[1] =
    y2[1] = y3[1] = y4[1] = y5[1] = y6[1] =
    z2[1] = z3[1] = z4[1] = z5[1] = z6[1] = 0;

    if (len_inc_buf != 28) {
        if (!have_warned) {
            blast_warn("We were only passed %d bytes of data instead of 28.", (uint16_t)len_inc_buf);
            have_warned = 1;
        }
        return;
    }
    xsn = inc_buf[8]; // sign
    x2[0]=inc_buf[9]; // number
    x3[0]=inc_buf[10]; // number
    x4[0]=inc_buf[11]; // number
    x5[0]=inc_buf[12]; // number
    x6[0]=inc_buf[13]; // number

    ysn = inc_buf[14]; // sign
    y2[0]=inc_buf[15]; // number
    y3[0]=inc_buf[16]; // number
    y4[0]=inc_buf[17]; // number
    y5[0]=inc_buf[18]; // number
    y6[0]=inc_buf[19]; // number

    zsn = inc_buf[20]; // sign
    z2[0]=inc_buf[21]; // number
    z3[0]=inc_buf[22]; // number
    z4[0]=inc_buf[23]; // number
    z5[0]=inc_buf[24]; // number
    z6[0]=inc_buf[25]; // number

// inc_buf[26] and [27] should be two characters denoting a hex expressed sum of all previous values.
// might be worth writing a quick check sum function.

    // int x = 1000*(atoi(x2))+100*(atoi(x3))+10*(atoi(x4))+atoi(x5); //atoi({x2,x3...x6})/1000.0   ...?
    // int y = 1000*(atoi(y2))+100*(atoi(y3))+10*(atoi(y4))+atoi(y5);
    // int z = 1000*(atoi(z2))+100*(atoi(z3))+10*(atoi(z4))+atoi(z5);

    // fix this mapping of the decimals TODO:JUZZ
//    int x = atoi({x2,x3,x4,x5,x6})/1000.0;
//    int y = atoi({y2,y3,y4,y5,y6})/1000.0;
//    int z = atoi({z2,z3,z4,z5,z6})/1000.0;
    int x = atoi(x6);
    int y = atoi(y6);
    int z = atoi(z6);
    blast_info("++++++++++++x6 is reading %d +++++++++++++", x);
    if (xsn == '1') x *= -1;
    if (ysn == '1') y *= -1;
    if (zsn == '1') z *= -1;
    inc_set_framedata(x, y, z);
}


/**
 * Inclinometer callback function handling events from the serial device.
 * @param serial
 * @param why
 * @param m_data
 */
static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);

    typedef struct {
    	int16_t inc_x;
    	int16_t inc_y;
    	int16_t inc_z;
    } inc_data_t;
    inc_data_t *inc_reading;

    static int has_warned = 0;

    ph_buf_t *buf;

#ifdef DEBUG_INCLINOMETER
    if (inc_verbose_level) blast_info("Inclinometer callback for reason %u, inc_state.cmd_state = %u, status = %u",
                 (uint8_t) why, (uint8_t) inc_state.cmd_state, (uint8_t) inc_state.status);
#endif

    // First check to see whether we have been asked to reset the inclinometer.
    if (CommandData.inc_reset) {
        inc_state.status = INC_RESET;
        CommandData.inc_reset = 0;
        return;
    }
    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * inclinometer stream
     */
    if ((why & PH_IOMASK_TIME)) {
        inc_state.cmd_state = 0;
        inc_state.timeout_count++;
        if (inc_verbose_level) blast_info("We timed out, count = %d , status = %d, Sending CMD '%s' to the INC",
                               inc_state.timeout_count, inc_state.status, state_cmd[inc_state.cmd_state].cmd);
        // Try again!
        inc_state.status = INC_ERROR;
        ph_stm_printf(serial->stream, "%s\r", state_cmd[inc_state.cmd_state].cmd);
        // I'm not sure if inc needs carriage return - Juzz
        ph_stm_flush(serial->stream);
        if (inc_state.timeout_count > INC_TIMEOUT_THRESHOLD) {
            inc_state.status = INC_RESET;
        }
        return;
    }

    if (why & PH_IOMASK_READ) {
        if (inc_verbose_level) blast_info("Reading inc data!");
//-------------------------HERE IS WHERE SERIAL IS ATUALLY READ-----------------------------------------------
  // Read until you find the header '680d0084', expect header to 8 bytes.
        buf = ph_serial_read_record(serial, "680d0084", 8);
        if (!buf) return; // we didn't get anything
        inc_state.status = INC_READING;
        /**
         * Handle the initial handshaking and setup with the inclinometer
         * If the inclinometer is not in continuous mode, send continuous command.
         */
        if (inc_state.cmd_state < INC_CONT) {
            if ((ph_buf_len(buf) - 1) == strlen(state_cmd[inc_state.cmd_state].resp)) {
                // I believe the -1 is considering carriage return? - juzz
                if (!memcmp(ph_buf_mem(buf), state_cmd[inc_state.cmd_state].resp,
                                      strlen(state_cmd[inc_state.cmd_state].resp))) {
                    inc_state.cmd_state++;
                    // blast_info("writing %s", state_cmd[inc_state.cmd_state].cmd);
                    ph_stm_printf(serial->stream, "%s\r", state_cmd[inc_state.cmd_state].cmd);
                    ph_stm_flush(serial->stream);

                    ph_buf_delref(buf);
                }
            } else {
                /**
                 * If we don't receive the length response that we expect, try to interrupt whatever
                 * is happening and reset our initialization to the beginning.
                 */
                if (!has_warned) blast_info("We didn't receive the appropriate response.  Resetting...");
                has_warned = 1;
                inc_state.status = INC_RESET;
                inc_state.cmd_state = 0;
                ph_stm_printf(serial->stream, "\e\r");
                // This may be meaningless to inclinometer check what it means to mag - juzz
                ph_stm_flush(serial->stream);
                do {
                    ph_buf_delref(buf);
                    buf = ph_serial_read_record(serial, "\r", 1);
                } while (buf);
            }
            return;
        }

        /**
         * We expect 28 bytes total per reading including the <CR>
         */
        if (ph_buf_len(buf) != 28) {
            ph_buf_delref(buf);
            return;
        }

        inc_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf));
        ph_buf_delref(buf);
        inc_state.error_warned = 0;
    }

    if (why & PH_IOMASK_ERR) {
    	if (inc_state.status != INC_RESET) {
    	    inc_state.status = INC_ERROR;
    	    inc_state.err_count++;
    	    // Try to restart the sequence.
    	    ph_stm_printf(serial->stream, "\e\r");
            ph_stm_flush(serial->stream);
        }
    	if (!(inc_state.error_warned)) {
    		blast_err("Error reading from the inclinometer! %s", strerror(errno));
    		inc_state.error_warned = 1;
    	}
    	if (inc_state.err_count > INC_ERR_THRESHOLD) {
    		// blast_err("Too many errors reading the inclinometer...attempting to reset.");
    		inc_state.status = INC_RESET;
    		inc_state.err_count = 0;
    		inc_state.error_warned = 0;
    		inc_state.cmd_state = 0;
    	}
    }
}

/**
 * This initialization function can be called at anytime to close, re-open and initialize the inclinometer.
 */
void initialize_inclinometer()
{
    static int has_warned = 0;
    static int firsttime = 1;
    if (inc_comm) ph_serial_free(inc_comm);
    inc_set_framedata(0, 0, 0);

    inc_comm = ph_serial_open(INCCOM, NULL, state_cmd);
    if (!inc_comm) {
    	if (!has_warned) blast_err("Could not open Inclinometer port %s", INCCOM);
      has_warned = 1;
    	return;
    } else {
    	// blast_info("Successfully opened Inclinometer port %s", INCCOM);
      has_warned = 0;
    }

    inc_comm->callback = inc_process_data;
    inc_comm->timeout_duration.tv_sec = 1;

    ph_serial_setspeed(inc_comm, B115200);
    // ph_stm_printf(inc_comm->stream, "*99\e\r"); This is the mag write enable command
    // I don't think we need a WE command for inc.
    // ph_stm_flush(inc_comm->stream);
    ph_stm_printf(inc_comm->stream, "%s\r", state_cmd[inc_state.cmd_state].cmd);
    ph_stm_flush(inc_comm->stream);
    ph_serial_enable(inc_comm, true);

    inc_state.err_count = 0;
    inc_state.timeout_count = 0;
    inc_state.status = INC_INIT;
    if (firsttime) {
        blast_startup("Initialized Inclinometer");
        firsttime = 0;
    }
}

void reset_inc()
{
    ph_stm_printf(inc_comm->stream, "\e");
    ph_stm_flush(inc_comm->stream);
    usleep(1000);
    initialize_inclinometer();
}

void *monitor_inclinometer(void *m_arg)
{
  static int has_warned = 0;
  while (!shutdown_mcp) {
    if (inc_state.status == INC_RESET) {
      if (inc_state.reset_count >= INC_RESET_THRESHOLD) {
          if (!has_warned) {
              blast_info("Still not able to connect to the inclinometers. reset_count = %d", inc_state.reset_count);
          }
          has_warned = 1;
          inc_state.reset_count = 0;
      }
      if (inc_verbose_level) blast_info("Received a request to reset the inclinometer communications.");
      reset_inc();
      inc_state.reset_count++;
      if (inc_verbose_level) blast_info("Inclinometer reset complete. reset counter = %d", inc_state.reset_count);
    }
    usleep(100000);
  }
  return NULL;
}

// Called in store_1hz_acs of acs.c
void store_1hz_inc(void)
{
    static int firsttime = 1;
    static channel_t *StatusIncAddr;
    static channel_t *ErrCountIncAddr;
    static channel_t *TimeoutCountIncAddr;
    static channel_t *ResetCountIncAddr;

    if (firsttime) {
        if (SouthIAm) {
            StatusIncAddr = channels_find_by_name("status_inc2_s");
            ErrCountIncAddr = channels_find_by_name("err_count_inc2_s");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_inc2_s");
            ResetCountIncAddr = channels_find_by_name("reset_count_inc2_s");
        } else {
            StatusIncAddr = channels_find_by_name("status_inc1_n");
            ErrCountIncAddr = channels_find_by_name("err_count_inc1_n");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_inc1_n");
            ResetCountIncAddr = channels_find_by_name("reset_count_inc1_n");
        }
        firsttime = 0;
    }
    SET_UINT8(StatusIncAddr, inc_state.status);
    SET_UINT16(ErrCountIncAddr, inc_state.err_count);
    SET_UINT16(TimeoutCountIncAddr, inc_state.timeout_count);
    SET_UINT16(ResetCountIncAddr, inc_state.reset_count);
}

