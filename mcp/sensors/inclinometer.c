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
#include "inclinometer.h"
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
 * @brief Inclinometer state enum mapping states to integers for low level states
 * 
 */
typedef enum {
    INC_BOOT = 0,
    INC_INIT,
    INC_READING,
    INC_ERROR,
    INC_RESET,
    INC_POWERCYCLE
} e_inc_state_t;


/**
 * @brief Tracking for error statuses
 * 
 */
typedef struct {
    e_inc_state_t state;
    uint16_t err_count;
    uint16_t error_warned;
    uint16_t timeout_count;
    uint16_t reset_count;
} inc_status_t;

inc_status_t inc_status = {0};

static char inc_cmd_buf[INC_CMD_MSG_LEN + 1U] = {0}; // reserve a byte for MEMS ID
static char inc_cmd_resp_buf[INC_CMD_MSG_LEN + 1U] = {0};


// Logging and telemetry


/**
 * @brief Send high rate inclinometer data to memory map
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
    // Since each flight computer has its own inclinometer, we only want to
    // write to the channels corresponding to that computer's inclinometer.
    static int firsttime = 1;

    if (firsttime) {
        if (SouthIAm) { // We are South (fc2)
            inc_x_channel = channels_find_by_name("x_inc2_s");
            inc_y_channel = channels_find_by_name("y_inc2_s");
            inc_temp_channel = channels_find_by_name("temp_inc2_s");
        } else { // We are North (fc1)
            inc_x_channel = channels_find_by_name("x_inc1_n");
            inc_y_channel = channels_find_by_name("y_inc1_n");
            inc_temp_channel = channels_find_by_name("temp_inc1_n");
        }
        firsttime = 0;
    }

    if (inc_verbose_level) {
        blast_info("incx is %f\n", m_incx);
    }

    SET_SCALED_VALUE(inc_x_channel, m_incx);
    SET_SCALED_VALUE(inc_y_channel, m_incy);
    SET_SCALED_VALUE(inc_temp_channel, m_incTemp);
}


/**
 * @brief Send low rate inclinometer data to memory map
 * @details Called in store_1hz_acs of acs.c
 */
void store_1hz_inc(void)
{
    static int firsttime = 1;
    uint8_t inc_ok = 0;
    static channel_t *StatusIncAddr;
    static channel_t *ErrCountIncAddr;
    static channel_t *TimeoutCountIncAddr;
    static channel_t *ResetCountIncAddr;
    static channel_t *ClinOKaddr;

    if (firsttime) {
        if (SouthIAm) {
            StatusIncAddr = channels_find_by_name("status_inc2_s");
            ErrCountIncAddr = channels_find_by_name("err_count_inc2_s");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_inc2_s");
            ResetCountIncAddr = channels_find_by_name("reset_count_inc2_s");
            ClinOKaddr = channels_find_by_name("ok_elclin2");
        } else {
            StatusIncAddr = channels_find_by_name("status_inc1_n");
            ErrCountIncAddr = channels_find_by_name("err_count_inc1_n");
            TimeoutCountIncAddr = channels_find_by_name("timeout_count_inc1_n");
            ResetCountIncAddr = channels_find_by_name("reset_count_inc1_n");
            ClinOKaddr = channels_find_by_name("ok_elclin1");
        }
        firsttime = 0;
    }
    SET_UINT8(StatusIncAddr, inc_status.state);
    SET_UINT16(ErrCountIncAddr, inc_status.err_count);
    SET_UINT16(TimeoutCountIncAddr, inc_status.timeout_count);
    SET_UINT16(ResetCountIncAddr, inc_status.reset_count);
    if (inc_status.state == INC_READING) {
        inc_ok = 1;
    } else {
        inc_ok = 0;
    }
    SET_UINT8(ClinOKaddr, inc_ok);
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
        if (inc_verbose_level) {
            blast_info("Sending command: %s", inc_cmd_buf);
        }
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_warn("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
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
        if (inc_verbose_level) {
            blast_info("Sending command: %s", inc_cmd_buf);
        }
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_warn("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
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
        if (inc_verbose_level) {
            blast_info("Sending command: %s", inc_cmd_buf);
        }
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_warn("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Send message to INC_DEV_ADDR to begin auto reporting
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
        if (inc_verbose_level) {
            blast_info("Sending command: %s", inc_cmd_buf);
        }
        success = ph_stm_write(serial->stream, inc_cmd_buf, sizeof(inc_cmd_buf), &nwrote);
        ph_stm_flush(serial->stream);
    }
    if (!success || (nwrote != sizeof(inc_cmd_buf))) {
        blast_warn("Failed to write all requested bytes: %ld / %ld", nwrote, sizeof(inc_cmd_buf));
    }
    return success;
}


/**
 * @brief Extract a float from 3 bytes of inclinometer message data
 * @details Algorithm based on Jewell Instruments MEMS Communication Manual
 * Appendix B for DMH devices.
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

    // Check message length
    if (len_inc_buf != INC_DATA_RESP_BUF_LEN) {
        if (!have_warned) {
            blast_warn("Received %d bytes instead of %d.\n",
                (uint16_t)len_inc_buf, INC_DATA_RESP_BUF_LEN);
            have_warned = 1;
        }
        return;
    }

    // Reconstruct message in proper order
    uint8_t ph_idx = 0U;
    for (uint8_t i = 0U; i < INC_DATA_RESP_BUF_LEN; ++i) {
        ph_idx = inc_msg_idx_to_ph_idx(i, INC_DATA_RESP_BUF_LEN);
        protocol_ordered_buf[i] = inc_buf[ph_idx];
    }

    // Check checksum and pass along data
    uint8_t checksum_idx = inc_get_msg_checksum_idx(protocol_ordered_buf);
    msg_checksum = protocol_ordered_buf[checksum_idx];
    our_checksum = inc_calc_checksum(protocol_ordered_buf);

    if (our_checksum != msg_checksum) {
        blast_warn("Checksum mismatch! Calc'd 0x%X instead of 0x%X", our_checksum, msg_checksum);
    } else {
        x_deg = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_X);
        y_deg = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_Y);
        celsius = inc_get_msg_value(protocol_ordered_buf, INC_DATA_RESP_IDX_T);
        inc_set_framedata(x_deg, y_deg, celsius);
    }
}


/**
 * @brief Inclinometer callback function handling events from the serial device.
 * @param serial part of the callback
 * @param why unused
 * @param m_data unused
 */
static void inc_process_data(ph_serial_t *serial, ph_iomask_t why, void *m_data)
{
    ph_unused_parameter(m_data);

    // Used to search input buffer for incoming data
    const char INC_DATA_HEADER[INC_DATA_HEADER_LEN] = {
        INC_MEMS_ID,
        INC_DATA_RESP_MSG_LEN,
        INC_DEV_ADDR,
        inc_calc_ack_byte(INC_CMD_REPORT_MODE)};
    ph_buf_t *buf;

    // First check to see whether we have been asked to reset the inclinometer.
    if (CommandData.inc_reset) {
        inc_status.state = INC_RESET;
        CommandData.inc_reset = 0;
        return;
    }
    // If we woke up due to timeout, then the assumption is that we need to
    // re-initialize the inclinometer stream
    if ((why & PH_IOMASK_TIME)) {
        blast_info("We timed out, count = %d , status = %d\n",
                               inc_status.timeout_count, inc_status.state);
        inc_status.timeout_count++;
        inc_status.state = INC_ERROR;
        ph_stm_flush(serial->stream);
        if (inc_status.timeout_count > INC_TIMEOUT_THRESHOLD) {
            inc_status.state = INC_RESET;
        }
        return;
    }
    // If we woke up due to data in buffer, search for the data message header
    if (why & PH_IOMASK_READ) {
        if (inc_verbose_level) {
            blast_info("Reading inc data!");
        }
        inc_status.state = INC_READING;
        buf = ph_serial_read_record(serial, INC_DATA_HEADER, 4);
        if (!buf) {
            return;
        }
        inc_get_data((char*)ph_buf_mem(buf), ph_buf_len(buf));
        ph_buf_delref(buf);
        inc_status.error_warned = 0;
    }
    // If we woke up due to a read error, record it and optionally trigger a
    // reset
    if (why & PH_IOMASK_ERR) {
        blast_info("ERROR");
        if (inc_status.state != INC_RESET) {
            inc_status.state = INC_ERROR;
            inc_status.err_count++;
        }
        if (!(inc_status.error_warned)) {
            blast_err("Error reading from the inclinometer! %s", strerror(errno));
            inc_status.error_warned = 1;
        }
        if (inc_status.err_count > INC_ERR_THRESHOLD) {
            inc_status.state = INC_RESET;
            inc_status.err_count = 0;
            inc_status.error_warned = 0;
        }
    }
}


/**
 * @brief This initialization function can be called at any time to close,
 * re-open and re-initialize the inclinometer.
 */
void initialize_inclinometer(void)
{
    static int has_warned = 0;
    static int firsttime = 1;

    // Prepare to (re)init
    if (inc_comm) {
        ph_serial_free(inc_comm);
    }
    inc_set_framedata(0, 0, 0);
    inc_status.state = INC_INIT;
    inc_status.err_count = 0;
    inc_status.timeout_count = 0;

    // Try to open the serial port
    inc_comm = ph_serial_open(INCCOM, NULL, NULL);
    if (!inc_comm) {
        if (!has_warned) {
            blast_err("\n\n\n\n\nCould not open Inclinometer port %s\n\n\n\n\n", INCCOM);
        }
        has_warned = 1;
        return;
    } else {
        has_warned = 0;
    }

    // Try to set up device parameters
    ph_serial_setspeed(inc_comm, B115200);

    inc_comm->callback = inc_process_data;
    inc_comm->timeout_duration.tv_sec = 1;

    bool success = false;
    success = inc_cmd_set_all_baud(inc_comm);
    if (!success) {
        inc_status.err_count++;
    }
    success = inc_cmd_set_all_addr(inc_comm);
    if (!success) {
        inc_status.err_count++;
    }
    success = inc_cmd_continuous(inc_comm);
    if (!success) {
        inc_status.err_count++;
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
        if (inc_status.state == INC_RESET) {
            if (inc_status.reset_count >= INC_RESET_THRESHOLD) {
                if (!has_warned) {
                    blast_warn("Still not able to connect to the inc. reset_count = %d", inc_status.reset_count);
                }
                has_warned = 1;
                inc_status.reset_count = 0;
            }
            if (inc_verbose_level) {
                blast_info("Received a request to reset the inclinometer communications.");
            }
            reset_inc();
            inc_status.reset_count++;
            if (inc_verbose_level) {
                blast_info("Inclinometer reset complete. reset counter = %d", inc_status.reset_count);
            }
        }
        usleep(100000);
    }
    return NULL;
}
