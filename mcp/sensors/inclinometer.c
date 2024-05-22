/**
 * @file inclinometer.c
 *
 * @date Feb 22 2022
 * @author Juzz
 *
 * @brief This file is part of MCP, created for the TIM project.
 * Specifically for use with Jewell model DMH-2-60-422 Inclinometers.
 *
 * This software is copyright (C) 2022-2023 University of Pennsylvania
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

// comm port for the inclinometers
#define INCCOM "/dev/ttyINC"

#define INC_ERR_THRESHOLD 1000
#define INC_TIMEOUT_THRESHOLD 10
#define INC_RESET_THRESHOLD 50

// north or south (FC1 or FC2)
extern int16_t SouthIAm; // defined in mcp.c

int inc_verbose_level = 0;
ph_serial_t *inc_comm = NULL;

/**
 * @brief Possible command modes
 */
typedef enum {
    INC_BAUD_RATE = 0,
    INC_CONT,
    INC_READ,
    INC_END
} e_inc_mode;


/**
 * @brief inclinometer state enum mapping states to integers for low level states
 * 
 */
typedef enum {
    INC_BOOT = 0,
    INC_INIT,
    INC_READING,
    INC_ERROR,
    INC_RESET,
    INC_POWERCYCLE
} e_inc_sub_state_t;


/**
 * @brief inclinometer state structure holding info about the whole device
 * 
 */
typedef struct {
    e_inc_mode cmd_mode;
    e_inc_sub_state_t status;
    uint16_t err_count;
    uint16_t error_warned;
    uint16_t timeout_count;
    uint16_t reset_count;
} inc_super_state_t;

inc_super_state_t inc_frame = {0};

static char inc_cmd_buf[INC_CMD_MSG_LEN + 1U] = {0}; // reserve a byte for MEMS ID
static char inc_cmd_resp_buf[INC_CMD_MSG_LEN + 1U] = {0};


// Logging and telemetry


/**
 * @brief logs the inclinometer data to the frame
 * 
 * @param m_incx inclinometer x angle to log
 * @param m_incy inclinometer y angle to log
 * @param m_incTemp inclinometer temp to log
 */
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
            inc_temp_channel = channels_find_by_name("temp_inc1_n");
        } else { // We are South (fc2)
            inc_x_channel = channels_find_by_name("x_inc2_s");
            inc_y_channel = channels_find_by_name("y_inc2_s");
            inc_temp_channel = channels_find_by_name("temp_inc2_s");
        }
        firsttime = 0;
    }
    // blast_info("incx is %f\n", m_incx);
    SET_SCALED_VALUE(inc_x_channel, m_incx);
    SET_SCALED_VALUE(inc_y_channel, m_incy);
    SET_SCALED_VALUE(inc_temp_channel, m_incTemp);
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


// Helper functions

/**
 * @brief Return the integer value representation of the byte in the message
 * length field
 *
 * @param inc_buf entire message from inclinometer
 * @return uint8_t message length 
 */
uint8_t inc_get_msg_len(char *inc_buf) {
    return (uint8_t) inc_buf[INC_LENGTH_BYTE_IDX];
}


// /**
//  * @brief Return the integer value representation of the byte in the message
//  * address field
//  *
//  * @param inc_buf entire message from inclinometer
//  * @return uint8_t message address
//  */
// uint8_t inc_get_msg_addr(char *inc_buf) {
//     return (uint8_t) inc_buf[ADDR_BYTE_IDX];
// }


// /**
//  * @brief Return the integer value representation of the byte in the command
//  * field
//  *
//  * @param inc_buf entire message from inclinometer
//  * @return uint8_t command
//  */
// uint8_t inc_get_msg_cmd(char *inc_buf) {
//     return (uint8_t) inc_buf[CMD_BYTE_IDX];
// }


/**
 * @brief Return the index of the checksum
 *
 * @param inc_buf entire message from inclinometer
 * @return uint8_t checksum index in message
 */
uint8_t inc_get_msg_checksum_idx(char *inc_buf) {
    // Since msg len byte is after the 0th byte, the total message length
    // is also the index of the last byte, the checksum.
    return inc_get_msg_len(inc_buf);
}

/**
 * @brief Return the integer value representation of the acknowledgment byte
 * in the message expected for the given command
 *
 * @param inc_buf entire message from inclinometer
 * @return uint8_t ack byte
 */
uint8_t inc_calc_ack_byte(uint8_t command_byte) {
    // Set most significant bit
    return command_byte | (1U << 7U);
}

/**
 * @brief Return the integer value representation of the checksum calculated
 * from the message
 *
 * @param inc_buf entire message from inclinometer
 * @return uint8_t checksum
 */
uint8_t inc_calc_checksum(char *inc_buf) {
    uint16_t checksum_full = 0;
    uint8_t checksum_idx = inc_get_msg_checksum_idx(inc_buf);
    for (uint8_t i = INC_LENGTH_BYTE_IDX; i < checksum_idx; ++i) {
        checksum_full += (uint16_t)inc_buf[i];
    }
    return (uint8_t)(checksum_full & 0xFF);
}

/**
 * @brief Populate buffers for the desired command to send to the inclinometer,
 * and expected response.
 * 
 * @param inc_cmd_buf command buffer to be filled: sent to device
 * @param inc_cmd_resp_buf command buffer to be filled: expected response from device
 * @param addr device address
 * @param mode command mode
 * @param value command value
 */
void inc_build_cmd_resp(char *inc_cmd_buf, char *inc_cmd_resp_buf, uint8_t addr, uint8_t mode, uint8_t value)
{
    inc_cmd_buf[INC_ID_IDX] = INC_MEMS_ID;
    inc_cmd_buf[INC_LENGTH_BYTE_IDX] = INC_CMD_MSG_LEN;
    inc_cmd_buf[INC_ADDR_BYTE_IDX] = (char)addr;
    inc_cmd_buf[INC_CMD_BYTE_IDX] = (char)mode;
    inc_cmd_buf[INC_CMD_VAL_BYTE_IDX] = (char)value;
    uint8_t checksum_idx = inc_get_msg_checksum_idx(inc_cmd_buf);
    inc_cmd_buf[checksum_idx] = inc_calc_checksum(inc_cmd_buf);

    inc_cmd_resp_buf[INC_ID_IDX] = INC_MEMS_ID;
    inc_cmd_resp_buf[INC_LENGTH_BYTE_IDX] = INC_CMD_MSG_LEN;
    inc_cmd_resp_buf[INC_ADDR_BYTE_IDX] = (char)addr;
    inc_cmd_resp_buf[INC_CMD_BYTE_IDX] = (char)inc_calc_ack_byte(mode);
    inc_cmd_resp_buf[INC_CMD_VAL_BYTE_IDX] = (char)value; // commanded value echoed back as response
    checksum_idx = inc_get_msg_checksum_idx(inc_cmd_resp_buf);
    inc_cmd_resp_buf[checksum_idx] = inc_calc_checksum(inc_cmd_resp_buf);
}

/**
 * @brief Send all-call message to set device address to INC_DEV_ADDR
 * 
 * @param serial 
 * @return true msg sent
 * @return false msg failed or not all bytes written
 */
bool inc_cmd_set_all_addr(ph_serial_t *serial)
{
    bool success = false;
    uint64_t nwrote = 0U;
    if (serial) {
        inc_build_cmd_resp(
            inc_cmd_buf,
            inc_cmd_resp_buf,
            INC_CMD_SET_ADDR_TGT,
            INC_CMD_SET_ADDR,
            INC_CMD_SET_ADDR_VAL);
        blast_info("Sending command: %s", inc_cmd_buf);
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_info("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Send all-call message to set device baud to INC_CMD_BAUD_VAL
 * 
 * @param serial 
 * @return true msg sent
 * @return false msg failed or not all bytes written
 */
bool inc_cmd_set_all_baud(ph_serial_t *serial)
{
    bool success = false;
    uint64_t nwrote = 0U;
    if (serial) {
        inc_build_cmd_resp(
            inc_cmd_buf,
            inc_cmd_resp_buf,
            INC_CMD_SET_ADDR_TGT,
            INC_CMD_BAUD,
            INC_CMD_BAUD_VAL);
        blast_info("Sending command: %s", inc_cmd_buf);
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_info("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Send message to INC_DEV_ADDR to stop auto reporting
 * 
 * @param serial 
 * @return true msg sent
 * @return false msg failed or not all bytes written
 */
bool inc_cmd_stop(ph_serial_t *serial)
{
    bool success = false;
    uint64_t nwrote = 0U;
    if (serial) {
        inc_build_cmd_resp(
            inc_cmd_buf,
            inc_cmd_resp_buf,
            INC_DEV_ADDR,
            INC_CMD_DATA_MODE,
            INC_CMD_DATA_MODE_VAL_STOP);
        blast_info("Sending command: %s", inc_cmd_buf);
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_info("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Send message to INC_DEV_ADDR to start auto reporting
 * 
 * @param serial 
 * @return true msg sent
 * @return false msg failed or not all bytes written
 */
bool inc_cmd_continuous(ph_serial_t *serial)
{
    bool success = false;
    uint64_t nwrote = 0U;
    if (serial) {
        inc_build_cmd_resp(
            inc_cmd_buf,
            inc_cmd_resp_buf,
            INC_DEV_ADDR,
            INC_CMD_DATA_MODE,
            INC_CMD_DATA_MODE_VAL_5HZ);
        blast_info("Sending command: %s", inc_cmd_buf);
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_info("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Extract a float from 3 bytes of inclinometer message data
 * 
 * @param inc_buf entire message from inclinometer
 * @param field_start_idx location in buffer to start reading from
 * @return float
 */
float inc_get_msg_value(char *inc_buf, uint8_t field_start_idx) {
    float val = 0.0;
    // Unpack the value: binary coded decimal
    val = ((inc_buf[field_start_idx] & 0x0F) * 10);
    val += ((inc_buf[field_start_idx + 1] & 0xF0) >> 4);
    val += ((inc_buf[field_start_idx + 1] & 0x0F) / 10.0);
    val += (((inc_buf[field_start_idx + 2] & 0xF0) >> 4) / 100.0);
    val += ((inc_buf[field_start_idx + 2] & 0x0F) / 1000.0);
    // Unpack the sign information
    if ((inc_buf[field_start_idx] & 0xF0)) {
        val *= -1.0;
    }
    return val;
}

/**
 * @brief phenom ph_serial_read_record returns the message in a weird order,
 * header last. aid swapping memory by clever indexing.
 * @param idx 
 * @return uint8_t equivalent index into header-last phenom buffer
 */
uint8_t inc_msg_idx_to_ph_idx(uint8_t msg_idx, uint32_t buf_len)
{
    return (msg_idx + (buf_len - INC_DATA_HEADER_LEN)) % buf_len;
}


/**
 * @brief polls the inclinometer data buffer and logs the info to the frame
 * @details Algorithm based on Jewell Instruments MEMS Communication Manual
 * Appendix B for DMH devices.
 * @param inc_buf entire message from inclinometer, from start byte to checksum
 * @param len_inc_buf size of the buffer
 */
static void inc_get_data(char *inc_buf, size_t len_inc_buf) {
    float x_deg = 0.0;
    float y_deg = 0.0;
    float celsius = 0.0;
    uint8_t msg_checksum = 0U;
    uint8_t our_checksum = 0U;
    static int have_warned = 0;
    static char protocol_ordered_buf[INC_DATA_RESP_BUF_LEN] = {0};

    if (len_inc_buf != INC_DATA_RESP_BUF_LEN) {
        if (!have_warned) {
            blast_info("We were only passed %d bytes of data instead of %d.",
                (uint16_t)len_inc_buf, INC_DATA_RESP_BUF_LEN);
            have_warned = 1;
        }
        return;
    }

    uint8_t ph_idx = 0U;
    for (uint8_t i = 0U; i < INC_DATA_RESP_BUF_LEN; ++i) {
        ph_idx = inc_msg_idx_to_ph_idx(i, INC_DATA_RESP_BUF_LEN);
        protocol_ordered_buf[i] = inc_buf[ph_idx];
    }

    uint8_t checksum_idx = inc_get_msg_checksum_idx(protocol_ordered_buf);
    msg_checksum = protocol_ordered_buf[checksum_idx];
    our_checksum = inc_calc_checksum(protocol_ordered_buf);
    if (our_checksum != msg_checksum) {
        blast_info("Checksum mismatch! Calc'd 0x%X instead of 0x%X", our_checksum, msg_checksum);
    }

    x_deg = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_X);
    y_deg = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_Y);
    celsius = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_T);

    // blast_info("x: %f y: %f T: %f\n", x_deg, y_deg, celsius);

    inc_set_framedata(x_deg, y_deg, celsius);
}


/**
 * @brief polls the inclinometer data buffer and logs the info to the frame
 * 
 * @param inc_buf data buffer to sort through
 * @param len_inc_buf size of the buffer
 */
static void inc_get_data_old(char *inc_buf, size_t len_inc_buf)
{
    static int have_warned = 0;

    int xsn;
    int x2;
    int x3;
    int ysn;
    int y2;
    int y3;
    int zsn;
    int z2;
    int z3;
    int chksm;
    int msg_sum = 0x0d + 0x01 + 0x84; // need to be included in checksum

    if (len_inc_buf != 14) {
        if (!have_warned) {
            blast_info("We were only passed %d bytes of data instead of 14.", (uint16_t)len_inc_buf);
            have_warned = 1;
        }
        return;
    }

    // blast_info("Setting Data Vars");
    xsn = inc_buf[0]; // sign byte
    x2 = inc_buf[1]; // number byte
    x3 = inc_buf[2]; // number byte

    ysn = inc_buf[3]; // sign
    y2 = inc_buf[4]; // number
    y3 = inc_buf[5]; // number

    zsn = inc_buf[6]; // sign
    z2 = inc_buf[7]; // number
    z3 = inc_buf[8]; // number

    chksm = inc_buf[9]; // Checksum: number

    for (int i = 0; i < 9; i++) {
        msg_sum += inc_buf[i];
    }

    // trim to least significant 2 hex digits if > 2-digit hex necessary to represent chksm.
    if (msg_sum > 255) {
        msg_sum -= (msg_sum / 256) * 256;
    }
    if (msg_sum != chksm) {
        blast_info("Checksum Error");
    }

    // Need to Add CHKSUM error Consequence here - Juzz Apr 2022

    float x;
    float y;
    float temp;

    if (xsn / 16 != 0) {
        // interpret hex as if it's just decimal. e.g. 0x27 = 27 and != 39
        // and bit shift to represent 5-sig fig decimal in degrees.
        x = 10 * (xsn % 16) + (x2 / 16) + 0.1 * (x2 % 16);
        x += 0.01 * (x3 / 16) + 0.001 * (x3 % 16);
        x *= -1.0;
    } else {
        x = 10 * (xsn) + (x2 / 16) + 0.1 * (x2 % 16);
        x += 0.01 * ((int)x3 / 16) + 0.001 * (x3 % 16);
        }

    if (ysn/16 != 0) {
        y = 10.0 * (ysn % 16) + (y2 / 16) + 0.1 * (y2 % 16);
        y += 0.01 * (y3 / 16) + 0.001 * (y3 % 16);
        y *= -1.0;
    } else {
        y = 10 * (ysn) + (y2 / 16) + 0.1 * (y2 % 16);
        y += 0.01 * ((int)y3 / 16) + 0.001 * (y3 % 16);
    }
    // same as above except celcius, not angle.
    if (zsn/16 != 0) {
        temp = 10.0 * (zsn % 16) + ((int)z2 / 16) + 0.1 * (z2 % 16);
        temp += 0.01 * (z3 / 16) + 0.001 * (z3 % 16);
        temp *= -1.0;
    } else {
        temp = 10 * (zsn) + ((int)z2 / 16) + 0.1 * (z2 % 16);
        temp += 0.01 * ((int) z3 / 16) + 0.001 * (z3 % 16);
    }

    // blast_info("OLD: x: %f, y: %f, T: %f\n", x, y, temp);
    inc_set_framedata(x, y, temp);
}


/**
 * @brief Inclinometer callback function handling events from the serial device.
 * @param serial part of the callback
 * @param why unused
 * @param m_data unused
 */
static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    // blast_info("inc_process data has been called\n");
    ph_unused_parameter(why);
    ph_unused_parameter(m_data);

    const char INC_DATA_HEADER[INC_DATA_HEADER_LEN] = {
        INC_MEMS_ID,
        INC_DATA_RESP_MSG_LEN,
        INC_DEV_ADDR,
        inc_calc_ack_byte(INC_CMD_REPORT_MODE)};
    ph_buf_t *buf;

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
        blast_info("We timed out, count = %d , status = %d\n",
                               inc_frame.timeout_count, inc_frame.status);
        inc_frame.timeout_count++;
        inc_frame.cmd_mode = 0;
        inc_frame.status = INC_ERROR;
        ph_stm_flush(serial->stream);
        if (inc_frame.timeout_count > INC_TIMEOUT_THRESHOLD) {
            inc_frame.status = INC_RESET;
        }
        return;
    }

    if (why & PH_IOMASK_READ) {
        if (inc_verbose_level) {
            blast_info("Reading inc data!");
        }
        inc_frame.status = INC_READING;
        buf = ph_serial_read_record(serial, INC_DATA_HEADER, 4);
        if (!buf) {
            return;
        }
        // inc_get_data_old((char*)ph_buf_mem(buf), ph_buf_len(buf));  // ***** SEEMS LIKE WE'RE CRASHING HERE *****
        inc_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf));
        ph_buf_delref(buf);
        inc_frame.error_warned = 0;
    }

    if (why & PH_IOMASK_ERR) {
        blast_info("ERROR");
        if (inc_frame.status != INC_RESET) {
            inc_frame.status = INC_ERROR;
            inc_frame.err_count++;
        }
        if (!(inc_frame.error_warned)) {
            blast_err("Error reading from the inclinometer! %s", strerror(errno));
            inc_frame.error_warned = 1;
        }
        if (inc_frame.err_count > INC_ERR_THRESHOLD) {
            inc_frame.status = INC_RESET;
            inc_frame.err_count = 0;
            inc_frame.error_warned = 0;
            inc_frame.cmd_mode = 0;
        }
    }
}


/**
 * @brief This initialization function can be called at any time to close,
 * re-open and re-initialize the inclinometer.
 */
void initialize_inclinometer()
{
    static int has_warned = 0;
    static int firsttime = 1;
    if (inc_comm) {
        ph_serial_free(inc_comm);
    }
    inc_set_framedata(0, 0, 0);

    inc_comm = ph_serial_open(INCCOM, NULL, NULL);

    if (!inc_comm) {
        if (!has_warned) {
            blast_err("\n\n\n\n\nCould not open Inclinometer port %s\n\n\n\n\n", INCCOM);
        }
        has_warned = 1;
        return;
    } else {
        // blast_info("Successfully opened Inclinometer port %d", INCCOM);
        has_warned = 0;
    }

    ph_serial_setspeed(inc_comm, B115200);

    inc_comm->callback = inc_process_data;
    inc_comm->timeout_duration.tv_sec = 1;

    inc_frame.err_count = 0;
    inc_frame.timeout_count = 0;
    // blast_info("inc cmd_mode = %d", inc_frame.cmd_mode);
    inc_frame.status = INC_INIT;

    bool success = false;
    success = inc_cmd_set_all_baud(inc_comm);
    if (!success) {
        inc_frame.err_count++;
    }
    success = inc_cmd_set_all_addr(inc_comm);
    if (!success) {
        inc_frame.err_count++;
    }
    success = inc_cmd_continuous(inc_comm);
    if (!success) {
        inc_frame.err_count++;
    }

    if (firsttime) {
        blast_startup("Initialized Inclinometer");
        firsttime = 0;
    }

    ph_serial_enable(inc_comm, true);
}


/**
 * @brief resets the inclinometer and purges the data stream
 * 
 */
void reset_inc(void)
{
    inc_cmd_stop(inc_comm);
    usleep(1000);
    initialize_inclinometer();
}


/**
 * @brief thread function for running an inclinometer in MCP
 * 
 * @param m_arg unused
 * @return void* unused for all intents and purposes
 */
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
            if (inc_verbose_level) {
                blast_info("Received a request to reset the inclinometer communications.");
            }
            reset_inc();
            inc_frame.reset_count++;
            if (inc_verbose_level) {
                blast_info("Inclinometer reset complete. reset counter = %d", inc_frame.reset_count);
            }
        }
        usleep(100000);
    }
    return NULL;
}
