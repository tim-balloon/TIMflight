/**
 * @file inclinometer.c
 *
 * @date Feb 22 2022
 * @author Juzz
 *
 * @brief This file is part of MCP, created for the TIM project.
 * Specifically for use with Jewell model DMH-2-60-422 Inclinometers.
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
#include <termios.h>

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

#define INCCOMIF "/dev/ttyUSB0"
#define INCCOMOF "/dev/ttyINC2"
// #define INCCOM "/dev/ttyACM0"

#define INC_ERR_THRESHOLD 1000
#define INC_TIMEOUT_THRESHOLD 10
#define INC_RESET_THRESHOLD 50

int inc_verbose_level = 0;
static ph_serial_t *inc_comm[2] = {NULL, NULL};
static const char INCCOMM[2][13] = {"/dev/ttyUSB0", "/dev/ttyINC2"};
static const uint32_t min_backoff_sec = 1;
static const uint32_t max_backoff_sec = 60;

typedef enum {
    // INC_WE_BIN = 0,
	// INC_BIN,
	// INC_WE_RATE,
	INC_BAUD_RATE = 0,
	INC_CONT,
	INC_READ,
	INC_END
} e_inc_mode;

typedef struct {
	char cmd[6];
	char resp[6];
} cmd_resp_t;

typedef enum {
    INC_BOOT = 0,
    INC_INIT,
    INC_READING,
    INC_ERROR,
    INC_RESET,
    INC_POWERCYCLE
} e_inc_sub_state_t;

typedef struct {
    int             which;
    uint32_t        backoff_sec;
    ph_job_t        connect_job;
    bool            want_reset;

	e_inc_mode          cmd_mode;
	e_inc_sub_state_t   status;
	uint16_t            err_count;
	uint16_t            error_warned;
	uint16_t            timeout_count;
	uint16_t            reset_count;
} inc_super_state_t;

inc_super_state_t inc_frame[2] = {{0}};

char continuous[] = {0x68, 0x05, 0x01, 0x0C, 0x01, 0x13, 0x00};
char STOP[6] = {0x68, 0x05, 0x01, 0x0C, 0x00, 0x11}; // Stop won't work 0x00 is read as NULL

static cmd_resp_t commanding_state[INC_END] = {
    [INC_BAUD_RATE] = {"\x68\x05\x01\x0B\x02\x13", "\x68\x10\x00\x8B\x05\xA0"},
    [INC_CONT] = {"\x68\x05\x01\x0C\x01\x13", "\x68\x05\x00\x8C\x00\x91"},
};

static void inc_set_frame_data(int incID, float m_incx, float m_incy, float m_incTemp) {
    static channel_t *inc_x_channel = NULL;
    static channel_t *inc_y_channel = NULL;
    static channel_t *inc_temp_channel = NULL;

    static int firsttime = 1;

    if (firsttime) {
        if (incID == 0) {
            inc_x_channel = channels_find_by_name("inc_if_x");
            inc_y_channel = channels_find_by_name("inc_if_y");
            inc_temp_channel = channels_find_by_name("inc_if_temp");
        } else if (incID == 1) {
            inc_x_channel = channels_find_by_name("inc_of_x");
            inc_y_channel = channels_find_by_name("inc_of_y");
            inc_temp_channel = channels_find_by_name("inc_of_temp");
        } else {
            blast_err("Bad inclinometer ID");
            return;
        }
        firsttime = 0;
    }

    // blast_info("incx is %f\n", m_incx);
    // SET_SCALED_VALUE(inc_x_channel, m_incx);
    // SET_SCALED_VALUE(inc_y_channel, m_incy);
    // SET_SCALED_VALUE(inc_temp_channel, m_incTemp);
    // SET_VALUE(inc_x_channel, m_incx);
    // SET_VALUE(inc_y_channel, m_incy);
    // SET_VALUE(inc_temp_channel, m_incTemp);
    SET_FLOAT(inc_x_channel, m_incx);
    SET_FLOAT(inc_y_channel, m_incy);
    SET_FLOAT(inc_temp_channel, m_incTemp);
}
/**
 * Inclinometer callback function handling events from the serial device.
 * @param serial
 * @param why
 * @param m_data
 */
// static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data, int m_which)
static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    // ph_unused_parameter(why);
    // ph_unused_parameter(m_data);
    inc_super_state_t *meta = (inc_super_state_t*)m_data;
    int incID = meta->which;
    const char inc_header[] = {104, 13, 1, 132};
    ph_buf_t *buf;
    // char *incData;
    size_t incData_len;

    // First check to see whether we have been asked to reset the inclinometer.
    if (CommandData.inc_reset) {
        inc_frame[incID].status = INC_RESET;
        CommandData.inc_reset = 0;
        return;
    }
    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * inclinometer stream
     */
    if ((why & PH_IOMASK_TIME)) {
        blast_info("TIME");
        inc_frame[incID].cmd_mode = 0;
        inc_frame[incID].timeout_count++;
        blast_info("We timed out, count = %d , status = %d, Sending CMD '%s' to the INC",
        inc_frame[incID].timeout_count, inc_frame[incID].status, commanding_state[inc_frame[incID].cmd_mode].cmd);
        // Try again!
        inc_frame[incID].status = INC_ERROR;

        ph_stm_flush(serial->stream);
        if (inc_frame[incID].timeout_count > INC_TIMEOUT_THRESHOLD) {
            inc_frame[incID].status = INC_RESET;
        }
        return;
    }

    if (why & PH_IOMASK_READ) {
        if (inc_verbose_level) blast_info("Reading inc data!");
        inc_frame[incID].status = INC_READING;
        inc_frame[incID].error_warned = 0;

        buf = ph_serial_read_record(serial, inc_header, 4);
        if (!buf) {return;}
        // inc_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf), m_which);  // ***** Don't call, just set Vars *****
        char *incData = (char*)ph_buf_mem(buf);
        incData_len = ph_buf_len(buf);
        static int have_warned = 0;
    // INC GET DATA BEGINS HERE
    int x2, x3;
    int y2, y3;
    int z2, z3;
    int xsn, ysn, zsn;
    int chksm;
    int msg_sum = 0x0d + 0x01 + 0x84;// need to be included in check sum.;

    // blast_info("Called get_data\n");
    if (incData_len != 14) {
        // if (!have_warned) {
            blast_info("We were only passed %d bytes of data instead of 14.", (uint16_t)incData_len);
            // have_warned = 1;
        // }
        return;
    }
    for (int i = 0; i < 9; i++) {
        // blast_info("incData[%d]: %d", i, incData[i]);
        msg_sum += incData[i];
        }

        // trim to least significant 2 hex digits if > 2-digit hex necessary to represent chksm.
    if (msg_sum > 255 ) msg_sum -= (msg_sum/256)*256;
    if (msg_sum != chksm) {
        blast_info("CheckSum Error\n msg_sum = %d, chksm = %d", msg_sum, chksm);
    // return;
    }
    xsn = incData[0]; // sign nybble then data nybble
    x2 = incData[1]; // number byte
    x3 = incData[2]; // number byte

    ysn = incData[3]; // sign/number
    y2 = incData[4]; // number
    y3 = incData[5]; // number

    zsn = incData[6]; // sign/number
    z2 = incData[7]; // number
    z3 = incData[8]; // number

    chksm = incData[9]; // Check Sum: number

    ph_buf_delref(buf);
    float x, y, temp;

    if (xsn/16 != 0) {
        // interpret hex as if it's just decimal. e.g. 0x27 = 27 and != 39
        // and bit shift to represent 5-sig fig decimal in degrees.
        x = 10.0 * (xsn % 16) + (x2/16) + 0.1*(x2%16);
        x += 0.01*(x3/16) + 0.001*(x3%16);
        x *= -1.0;
        } else {
        x = 10.0 * (xsn) + (x2/16) + 0.1*(x2%16);
        x += 0.01*((int)x3/16) + 0.001*(x3%16);
        }

    if (ysn/16 != 0) {
        y = 10.0 * (ysn % 16) + (y2/16) + 0.1*(y2%16);
        y += 0.01*(y3/16) + 0.001*(y3%16);
        y *= -1.0;
        } else {
        y = 10.0*(ysn) + (y2/16) + 0.1*(y2%16);
        y += 0.01*((int)y3/16) + 0.001*(y3%16);
        }
    // same as above except celcius, not angle.
    if (zsn/16 != 0) {
        temp = 10.0 * (zsn % 16) + ((int)z2/16) + 0.1*(z2%16);
        temp += 0.01*(z3/16) + 0.001*(z3%16);
        temp *= -1.0;
        } else {
        temp = 10.0*(zsn) + ((int)z2/16) + 0.1*(z2%16);
        temp += 0.01 * ((int) z3 / 16) + 0.001 * (z3 % 16);
        }
    blast_info("\nX: %f, Y: %f, Temp: %f\n", x, y, temp);

    inc_set_frame_data(incID, x, y, temp);
}

    if (why & PH_IOMASK_ERR) {
        blast_info("ERROR");
    	if (inc_frame[incID].status != INC_RESET) {
    	    inc_frame[incID].status = INC_ERROR;
    	    inc_frame[incID].err_count++;
    	    // Try to restart the sequence.
    	    ph_stm_printf(serial->stream, STOP); // this won't work
            ph_stm_flush(serial->stream);
        }
    	if (!(inc_frame[incID].error_warned)) {
    		blast_err("Error reading from the inclinometer! %s", strerror(errno));
    		inc_frame[incID].error_warned = 1;
    	}
    	if (inc_frame[incID].err_count > INC_ERR_THRESHOLD) {
    		// blast_err("Too many errors reading the inclinometer...attempting to reset.");
    		inc_frame[incID].status = INC_RESET;
    		inc_frame[incID].err_count = 0;
    		inc_frame[incID].error_warned = 0;
    		inc_frame[incID].cmd_mode = 0;
    	}
    }
}

/**
 * This initialization function can be called at anytime to close, re-open and initialize the inclinometer.
 */
void connect_inclinometer(ph_job_t *m_job, ph_iomask_t m_why, void *m_data) {
    ph_unused_parameter(m_why);
    static bool firsttime = true;

    inc_super_state_t *data = (inc_super_state_t*)m_data;
    int incID = data->which;
    if (inc_comm[incID]) ph_serial_free(inc_comm[incID]);

    // inc_comm[incID] = ph_serial_open(gyro_port[incID], &term, data);
    inc_comm[incID] = ph_serial_open(INCCOMM[incID], NULL, data);

    if (!inc_comm[incID]) {
        // if (!has_warned) blast_err("Could not open Inclinometer port %d\n", incID);
        blast_err("Could not open Inclinometer port %d\n", incID);
        // has_warned = 1;
            return;
        } else {
            blast_info("Successfully opened Inclinometer port %d", incID);
        }

    ph_serial_setspeed(inc_comm[incID], B9600);

    inc_comm[incID]->callback = inc_process_data;
    inc_comm[incID]->timeout_duration.tv_sec = 1;
    inc_comm[incID]->timeout_duration.tv_usec = 0;

    ph_serial_enable(inc_comm[incID], true);
    ph_stm_printf(inc_comm[incID]->stream, continuous);
    ph_stm_flush(inc_comm[incID]->stream);

    inc_frame[incID].err_count = 0;
    inc_frame[incID].timeout_count = 0;
    inc_frame[incID].status = INC_INIT;
    if (firsttime) {
            blast_startup("Initialized Inclinometer %i\n", incID);
            firsttime = false;
        }
        inc_set_frame_data(incID, 0, 0, 0);
}
void initialize_inclinometers()
{
        /// Define a separate job pool for the gyro read.
//    gyro_pool = ph_thread_pool_define("gyro_read", 4, 1);

    for (int i = 0; i < 2; i++) {
        // BLAST_ZERO(inc_frame[i]);
        inc_frame[i].which = i;
        inc_frame[i].backoff_sec = min_backoff_sec;

        ph_job_init(&(inc_frame[i].connect_job));
        inc_frame[i].connect_job.callback = connect_inclinometer;
        inc_frame[i].connect_job.data = &(inc_frame[i]);
//        ph_job_set_pool(&(inc_frame[i].connect_job), gyro_pool);

        // Set the dispatch to 500ms and 1000ms respectively for the gyro connect
        ph_job_set_timer_in_ms(&(inc_frame[i].connect_job), 500 * (i+1));
    }
}

void reset_inc(int incID)
{
    // ph_stm_printf(inc_comm->stream, "\x68\x05\x00\x0C\x00\x11");
    ph_stm_printf(inc_comm[incID]->stream, STOP);
    ph_stm_flush(inc_comm[incID]->stream);
    usleep(1000);
    initialize_inclinometers();
}

void *monitor_inclinometer(int incID)
{
  static int has_warned = 0;
  while (!shutdown_mcp) {
    if (inc_frame[incID].status == INC_RESET) {
      if (inc_frame[incID].reset_count >= INC_RESET_THRESHOLD) {
          if (!has_warned) {
              blast_info("Still not able to connect to the inc. reset_count = %d",
              inc_frame[incID].reset_count);
          }
          has_warned = 1;
          inc_frame[incID].reset_count = 0;
      }
      if (inc_verbose_level) blast_info("Received a request to reset the inclinometer communications.");
      reset_inc(incID);
      inc_frame[incID].reset_count++;
      if (inc_verbose_level) blast_info("Inclinometer reset complete. reset counter = %d",
      inc_frame[incID].reset_count);
    }
    usleep(100000);
  }
  return NULL;
}

// Called in store_1hz_acs of acs.c
void store_1hz_inc(int incID)
{
    static int firsttime = 1;
    static channel_t *StatusIncAddr;
    static channel_t *ErrCountIncAddr;
    static channel_t *TimeoutCountIncAddr;
    static channel_t *ResetCountIncAddr;

    if (firsttime) {
        if (incID == 1) {
            StatusIncAddr = channels_find_by_name("status_if_inc");
            ErrCountIncAddr = channels_find_by_name("err_count_if_inc");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_if_inc");
            ResetCountIncAddr = channels_find_by_name("reset_count_if_inc");
        } else if (incID == 2) {
            StatusIncAddr = channels_find_by_name("status_of_inc");
            ErrCountIncAddr = channels_find_by_name("err_count_of_inc");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_of_inc");
            ResetCountIncAddr = channels_find_by_name("reset_count_of_inc");
        } else {blast_err("Bad inc ID in store_1hz_inc");}
        firsttime = 0;
    }
    SET_UINT8(StatusIncAddr, inc_frame[incID].status);
    SET_UINT16(ErrCountIncAddr, inc_frame[incID].err_count);
    SET_UINT16(TimeoutCountIncAddr, inc_frame[incID].timeout_count);
    SET_UINT16(ResetCountIncAddr, inc_frame[incID].reset_count);
}

