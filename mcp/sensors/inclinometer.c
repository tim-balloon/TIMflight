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

#define INCCOM "/dev/ttyUSB0"
// #define INCCOM "/dev/ttyACM0"

#define INC_ERR_THRESHOLD 1000
#define INC_TIMEOUT_THRESHOLD 10
#define INC_RESET_THRESHOLD 50

// extern int16_t SouthIAm; // defined in mcp.c

int inc_verbose_level = 0;
ph_serial_t *inc_comm = NULL;

/*typedef struct { // A struct of char arrays to toss data around
	unsigned char response[6];
	unsigned char data[10];
    unsigned char header[4];
    unsigned char single[1];
} inc_buffers_t; */

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
	e_inc_mode cmd_mode;
	e_inc_sub_state_t status;
	uint16_t err_count;
	uint16_t error_warned;
	uint16_t timeout_count;
	uint16_t reset_count;
} inc_super_state_t;

inc_super_state_t inc_frame = {0};
// char baud[6] = {0x68, 0x05, 0x00, 0x0B, 0x02, 0x12};
char continuous[] = {0x68, 0x05, 0x01, 0x0C, 0x01, 0x13, 0x00};
// char continuous[7] = {104, 5, 1, 12, 1, 19, 13};
// char continuous[] = "\x68\x05\x01\x0C\x01\x00";
char STOP[6] = {0x68, 0x05, 0x01, 0x0C, 0x00, 0x11}; // Stop won't work 0x00 is read as NULL

/**char baud[6];
char continuous[6];
char STOP[6];

baud[0] = 0x68;
baud[1] = 0x05;
baud[2] = 0x00;
baud[3] = 0x0B;
baud[4] = 0x02;
baud[5] = 0x12;

continuous[0] = 0x68;
continuous[1] = 0x05;
continuous[2] = 0x00;
continuous[3] = 0x0C;
continuous[4] = 0x01;
continuous[5] = 0x12;

STOP[0] = 0x68;
STOP[1] = 0x05;
STOP[2] = 0x00;
STOP[3] = 0x0C;
STOP[4] = 0x00;
STOP[5] = 0x11;
*/


// unsigned char STOP[] = "\x68\x05\x00\x0C\x00\x11";
static cmd_resp_t commanding_state[INC_END] = {
    [INC_BAUD_RATE] = {"\x68\x05\x01\x0B\x02\x13", "\x68\x10\x00\x8B\x05\xA0"},
    // [INC_BAUD_RATE] = {baud, "\x68\x10\x00\x8B\x05\xA0"},
    [INC_CONT] = {"\x68\x05\x01\x0C\x01\x13", "\x68\x05\x00\x8C\x00\x91"},
    // [INC_CONT] = {"%d%d%d%d%d%d",0x68,0x05,0x00,0x0C,0x01,0x12}
    // [INC_CONT] = {continuous, "\x68\x05\x00\x8C\x00\x91"},
};

static void inc_set_framedata(float m_incx, float m_incy, float m_incTemp)
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
    blast_info("incx is %f\n", m_incx);
    SET_SCALED_VALUE(inc_x_channel, m_incx);
    SET_SCALED_VALUE(inc_y_channel, m_incy);
    SET_SCALED_VALUE(inc_temp_channel, m_incTemp);
}

static void inc_get_data(char *inc_buf, size_t len_inc_buf)
{
    static int have_warned = 0;
    // static int firsttime = 1;

    // TEST STYLE
    int x2, x3;
    int y2, y3;
    int z2, z3;
    int xsn, ysn, zsn;
    int chksm;
    int msg_sum = 0x0d + 0x01 + 0x84;// need to be included in check sum.;

    // blast_info("Called get_data\n");
    if (len_inc_buf != 14) {
        if (!have_warned) {
            blast_info("We were only passed %d bytes of data instead of 14.", (uint16_t)len_inc_buf);
            have_warned = 1;
        }
        return;
    }

    // blast_info("Setting Data Vars");
    // TEST STYLE
    xsn = inc_buf[0]; // sign byte
    x2 = inc_buf[1]; // number byte
    x3 = inc_buf[2]; // number byte


    ysn = inc_buf[3]; // sign
    y2 = inc_buf[4]; // number
    y3 = inc_buf[5]; // number


    zsn = inc_buf[6]; // sign
    z2 = inc_buf[7]; // number
    z3 = inc_buf[8]; // number

    chksm = inc_buf[9]; // Check Sum: number

    for (int i = 0; i < 9; i++) {msg_sum += inc_buf[i];}

    // trim to least significant 2 hex digits if > 2-digit hex necessary to represent chksm.
    if (msg_sum > 255 ) msg_sum -= (msg_sum/256)*256;
    if (msg_sum != chksm) {
        blast_info("CheckSum Error");
        // return;
    }

/** Need to Add CHKSUM error Consequence here - Juzz Apr 2022*/

    float x, y, temp;
    int intX, intY, intTemp;

    if (xsn/16 != 0) {
        // interpret hex as if it's just decimal. e.g. 0x27 = 27 and != 39
        // and bit shift to represent 5-sig fig decimal in degrees.
        x = 10 * (xsn % 16) + (x2/16) + 0.1*(x2%16);
        x += 0.01*(x3/16) + 0.001*(x3%16);
        x *= -1.0;
        } else {
        x = 10 * (xsn) + (x2/16) + 0.1*(x2%16);
        x += 0.01*((int)x3/16) + 0.001*(x3%16);
        }

    if (ysn/16 != 0) {
        y = 10.0 * (ysn % 16) + (y2/16) + 0.1*(y2%16);
        y += 0.01*(y3/16) + 0.001*(y3%16);
        y *= -1.0;
        } else {
        y = 10*(ysn) + (y2/16) + 0.1*(y2%16);
        y += 0.01*((int)y3/16) + 0.001*(y3%16);
        }
    // same as above except celcius, not angle.
    if (zsn/16 != 0) {
        temp = 10.0 * (zsn % 16) + ((int)z2/16) + 0.1*(z2%16);
        temp += 0.01*(z3/16) + 0.001*(z3%16);
        temp *= -1.0;
        } else {
        temp = 10*(zsn) + ((int)z2/16) + 0.1*(z2%16);
        temp += 0.01 * ((int) z3 / 16) + 0.001 * (z3 % 16);
        }
    blast_info("\nX: %f, Y: %f, Temp: %f\n", x, y, temp);
    // intTemp = (int)(temp * 1000.0);
    // intX = (int)(x * 1000.0);
    // intY = (int)(y * 1000.0);
    // blast_info("\nINTX: %d, INTY: %d, INTTemp: %d\n", intX, intY, intTemp);
    inc_set_framedata(x, y, temp);
    // inc_set_framedata(intX, intY, intTemp);
}


/**
 * Inclinometer callback function handling events from the serial device.
 * @param serial
 * @param why
 * @param m_data
 */
static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    // blast_info("inc_process data has been called\n");
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);

    // static int has_warned = 0;
    // unsigned char inc_header[] = {0x68, 0x0d, 0x01, 0x84};
    const char inc_header[] = {104, 13, 1, 132};
    // char inc_header[] = "\x68\x0d\x01\x84";
    // ph_buf_t *buf;
    // inc_buffers_t buffers;
    ph_buf_t *buf;
    // ph_buf_t *bufHeader;
    ph_buf_t *singleBuf;
    int single;
    int *header[4];

#ifdef DEBUG_INCLINOMETER
    if (inc_verbose_level) blast_info("Inclinometer callback for reason %u, inc_frame.cmd_mode = %u, status = %u",
                 (uint8_t) why, (uint8_t) inc_frame.cmd_mode, (uint8_t) inc_frame.status);
#endif

    // First check to see whether we have been asked to reset the inclinometer.
    if (CommandData.inc_reset) {
        inc_frame.status = INC_RESET;
        CommandData.inc_reset = 0;
        return;
    }
    /**
     * If we timeout, then the assumption is that we need to re-initialize the
     * inclinometer stream
     */
    if ((why & PH_IOMASK_TIME)) {
        blast_info("TIME");
        inc_frame.cmd_mode = 0;
        inc_frame.timeout_count++;
        blast_info("We timed out, count = %d , status = %d, Sending CMD '%s' to the INC",
                               inc_frame.timeout_count, inc_frame.status, commanding_state[inc_frame.cmd_mode].cmd);
        // Try again!
        inc_frame.status = INC_ERROR;
        // ph_stm_printf(serial->stream, "%s", commanding_state[inc_frame.cmd_mode].cmd);
        // I'm not sure if inc needs carriage return replacing with below. - Juzz

        ph_stm_flush(serial->stream);
        if (inc_frame.timeout_count > INC_TIMEOUT_THRESHOLD) {
            inc_frame.status = INC_RESET;
        }
        return;
    }

    if (why & PH_IOMASK_READ) {
        if (inc_verbose_level) blast_info("Reading inc data!");
        // blast_info("READ");
        bool messageRead = false;
            inc_frame.status = INC_READING;
        // do {
           // blast_info("Ping");
            buf = ph_serial_read_record(serial, inc_header, 4);
            if (!buf) {return;}
                // blast_info("\n\nHeader read in\n\n");
                // buf = ph_serial_read_bytes_exact(serial, 10);
                inc_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf));  // ***** SEEMS LIKE WE'RE CRASHING HERE *****
                ph_buf_delref(buf);
                inc_frame.error_warned = 0;
                // messageRead = true;
            // } while (!messageRead);
            // return;
    }

    if (why & PH_IOMASK_ERR) {
        blast_info("ERROR");
    	if (inc_frame.status != INC_RESET) {
    	    inc_frame.status = INC_ERROR;
    	    inc_frame.err_count++;
    	    // Try to restart the sequence.
    	    ph_stm_printf(serial->stream, STOP); // this won't work
            ph_stm_flush(serial->stream);
        }
    	if (!(inc_frame.error_warned)) {
    		blast_err("Error reading from the inclinometer! %s", strerror(errno));
    		inc_frame.error_warned = 1;
    	}
    	if (inc_frame.err_count > INC_ERR_THRESHOLD) {
    		// blast_err("Too many errors reading the inclinometer...attempting to reset.");
    		inc_frame.status = INC_RESET;
    		inc_frame.err_count = 0;
    		inc_frame.error_warned = 0;
    		inc_frame.cmd_mode = 0;
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
    // struct termios term = {0}; // added this from dsp1760

    inc_comm = ph_serial_open(INCCOM, NULL, commanding_state);
    // term.c_cflag = CS8 | B38400 | CLOCAL | CREAD;
    // term.c_iflag = IGNPAR | IGNBRK;
    // inc_comm = ph_serial_open(INCCOM, &term, commanding_state);
    ph_serial_setspeed(inc_comm, B9600);

    if (!inc_comm) {
    	if (!has_warned) blast_err("\n\n\n\n\nCould not open Inclinometer port %s\n\n\n\n\n", INCCOM);
      has_warned = 1;
    	return;
    } else {
    	// blast_info("Successfully opened Inclinometer port %d", INCCOM);
      has_warned = 0;
    }

    inc_comm->callback = inc_process_data;
    inc_comm->timeout_duration.tv_sec = 1;

    // ph_stm_printf(inc_comm->stream, "*99\e\r"); This is the mag write enable command
    // I don't think we need a WE command for inc.
    // ph_stm_flush(inc_comm->stream);
    // ph_stm_printf(inc_comm->stream, "%s\r", commanding_state[inc_frame.cmd_mode].cmd);
    // ph_stm_printf(inc_comm->stream, commanding_state[inc_frame.cmd_mode].cmd);
    ph_serial_enable(inc_comm, true);
    ph_stm_printf(inc_comm->stream, continuous);
    ph_stm_flush(inc_comm->stream);
    // ph_stm_flush(inc_comm->stream);

    inc_frame.err_count = 0;
    inc_frame.timeout_count = 0;
    blast_info("inc cmd_mode = %d", inc_frame.cmd_mode);
    inc_frame.status = INC_INIT;
    if (firsttime) {
        blast_startup("\n\nInitialized Inclinometer\n\n");
        firsttime = 0;
    }
}

void reset_inc()
{
    // ph_stm_printf(inc_comm->stream, "\x68\x05\x00\x0C\x00\x11");
    ph_stm_printf(inc_comm->stream, STOP);
    ph_stm_flush(inc_comm->stream);
    usleep(1000);
    initialize_inclinometer();
}

void *monitor_inclinometer(void *m_arg)
{
  static int has_warned = 0;
  while (!shutdown_mcp) {
    if (inc_frame.status == INC_RESET) {
      if (inc_frame.reset_count >= INC_RESET_THRESHOLD) {
          if (!has_warned) {
              blast_info("Still not able to connect to the inc. reset_count = %d", inc_frame.reset_count);
          }
          has_warned = 1;
          inc_frame.reset_count = 0;
      }
      if (inc_verbose_level) blast_info("Received a request to reset the inclinometer communications.");
      reset_inc();
      inc_frame.reset_count++;
      if (inc_verbose_level) blast_info("Inclinometer reset complete. reset counter = %d", inc_frame.reset_count);
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
    SET_UINT8(StatusIncAddr, inc_frame.status);
    SET_UINT16(ErrCountIncAddr, inc_frame.err_count);
    SET_UINT16(TimeoutCountIncAddr, inc_frame.timeout_count);
    SET_UINT16(ResetCountIncAddr, inc_frame.reset_count);
}

