/*
 * labjack.c:
 *
 * This software is copyright
 *  (C) 2014-2016 University of Pennsylvania
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
 * 59 Temple Place, Suite 330, Boston,          MA  02111-1307  USA
 *
 * History:
 * Created on: May 5, 2016 by ian
 */


#include <stdint.h>
#include <glib.h>
#include <modbus/modbus.h>
#include <errno.h>

#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"
#include "phenom/queue.h"

#include "command_struct.h"
#include "blast.h"
#include "mcp.h"
#include "mputs.h"
#include "tx.h"
#include "labjack_functions.h"
#include "labjack.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

#define NUM_LABJACK_AIN 14
extern labjack_state_t state[NUM_LABJACKS];
extern int16_t InCharge;
extern labjack_digital_in_t labjack_digital;

static const uint32_t min_backoff_sec = 5;
static const uint32_t max_backoff_sec = 30;


/**
 * @brief labjack command structure that includes the labjack to be commanded,
 * the address to send the command to, the value to be commanded, and the phenom
 * queue entry that will handle the command queueing for execution
 * 
 */
typedef struct labjack_command {
    PH_STAILQ_ENTRY(labjack_command) q;
    int labjack;
    int address;
    float command;
} labjack_command_t;


/**
 * @brief redefines a libphenom queue as a labjack command queue
 * 
 */
typedef PH_STAILQ_HEAD(labjack_command_q, labjack_command) labjack_commandq_t;

labjack_commandq_t s_labjack_command;


/**
 * @brief executes the next item in the FIFO labjack command queue when called
 * 
 */
static void labjack_execute_command_queue(void) {
    labjack_command_t *cmd, *tcmd;
    PH_STAILQ_FOREACH_SAFE(cmd, &s_labjack_command, q, tcmd) {
        if (InCharge && (state[cmd->labjack].initialized)) {
            heater_write(cmd->labjack, cmd->address, cmd->command);
        }
        PH_STAILQ_REMOVE_HEAD(&s_labjack_command, q);
        free(cmd);
    }
}


/**
 * @brief initializes the labjack digital i/o channel pointers
 * 
 */
void init_labjack_digital(void) {
    labjack_digital.status_charcoal_heater_Addr = channels_find_by_name("status_charcoal_heater");
    labjack_digital.status_250_LNA_Addr = channels_find_by_name("status_250_LNA");
    labjack_digital.status_1K_heater_Addr = channels_find_by_name("status_1K_heater");
    labjack_digital.status_charcoal_hs_Addr = channels_find_by_name("status_charcoal_hs");
    labjack_digital.status_500_LNA_Addr = channels_find_by_name("status_500_LNA");
    labjack_digital.status_350_LNA_Addr = channels_find_by_name("status_350_LNA");
    // blast_info("init channels for labjack digital");
}


/**
 * @brief function that queues the desired LabJack command
 * 
 * @param m_labjack which labjack to talk to
 * @param m_address which address to write to
 * @param m_command commanded value to send
 */
void labjack_queue_command(int m_labjack, int m_address, float m_command) {
    labjack_command_t *cmd;

    cmd = balloc(err, sizeof(labjack_command_t));
    cmd->labjack = m_labjack;
    cmd->address = m_address;
    cmd->command = m_command;
    PH_STAILQ_INSERT_TAIL(&s_labjack_command, cmd, q);
}


// This isn't working now.  TODO(laura): Fix read from internal FLASH memory.
// not really worth fixing, general diff is less than 1/10^6 based on python
/**
 * @brief gets the internal LabJack calibration if it differs from factory.
 * 
 * @param m_state labjack state structure
 * @param devCal labjack calibration structure
 * @return int -1 on failure, 0 on success
 */
int labjack_get_cal(labjack_state_t *m_state, labjack_device_cal_t *devCal)
{
	const unsigned int EFAdd_CalValues = 0x3C4000;
	const int FLASH_PTR_ADDRESS	= 61810;

	// 3 frames	of 13 values, one frame	of 2 values
	const int FLASH_READ_ADDRESS = 61812;
	const int FLASH_READ_NUM_REGS[4] = {26, 26, 26, 4};
    uint16_t err_data[2] = {0}; // Used to read labjack specific error codes.
	static uint16_t have_warned_write_mem = 0;
	static uint16_t have_warned_read_cal = 0;
    int ret = 0;

	float calValue = 0.0;
	int calIndex = 0;
	uint16_t data[26];

	int i = 0;
	int j = 0;

	for(i = 0; i < 4; i++) {
		// Set the pointer. This indicates which part of the memory we want to read
		blast_info("i = %i, Flash Memory Address = %i", i, EFAdd_CalValues + i * 13 * 4);
        labjack_set_short(EFAdd_CalValues + i * 13 * 4, data);
        if ((ret = modbus_write_registers(m_state->cmd_mb, FLASH_PTR_ADDRESS, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!have_warned_write_mem) {
                blast_err("Could not set memory to read cal info (index = %i): %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            have_warned_write_mem = 1;
            m_state->calibration_read = 0;
            return -1;
        }
// uint32ToBytes(EFAdd_CalValues + i * 13 * 4, data);
// if(writeMultipleRegistersTCP(sock, FLASH_PTR_ADDRESS, 2, data) < 0)
// return -1;

		// Read the calibration constants
        if ((ret = modbus_read_registers(m_state->cmd_mb, FLASH_READ_ADDRESS, FLASH_READ_NUM_REGS[i], data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!have_warned_read_cal) {
                blast_err("Could not read cal info (index = %i): %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            have_warned_read_cal = 1;
            m_state->calibration_read = 0;
            return -1;
        }
// if(readMultipleRegistersTCP(sock, FLASH_READ_ADDRESS, FLASH_READ_NUM_REGS[i], data) < 0)
//     return -1;

		for(j = 0; j < FLASH_READ_NUM_REGS[i]*2; j+=4) {
			calValue = labjack_get_float(&data[j]);
			((float *)devCal)[calIndex]	= calValue;
			blast_info("Dev Cal i=%i, j=%i, data[j] = %u, val=%f", i, j, data[j], calValue);
			calIndex++;
		}
	}
	blast_info("Successfully read labjack calibration info.");
    have_warned_read_cal = 0;
    have_warned_write_mem = 0;
    m_state->calibration_read = 1;
	return 0;
}


/**
 * @brief initializes the data stream from the specified labjack
 * 
 * @param m_state labjack state structure to initialize from
 */
static void init_labjack_stream_commands(labjack_state_t *m_state)
{
    int ret = 0;
    int m_state_number;
    uint16_t data[2] = {0}; // Used to write floats.
    uint16_t err_data[2] = {0}; // Used to read labjack specific error codes.
    labjack_data_t *state_data = (labjack_data_t*)m_state->conn_data;

    // Configure stream
    float scanRate = LJ_STREAM_RATE; // Scans per second. Samples per second = scanRate * numAddresses
    unsigned int numAddresses = state_data->num_channels;
    unsigned int samplesPerPacket = numAddresses*state_data->scans_per_packet;
    float settling = 10.0; // 10 microseconds
    unsigned int resolutionIndex = 0;
    unsigned int bufferSizeBytes = 0;
    unsigned int autoTarget = STREAM_TARGET_ETHERNET;
    unsigned int numScans = 0; // 0 = Run continuously.
    unsigned int scanListAddresses[MAX_NUM_ADDRESSES] = {0};
    uint16_t nChanList[1] = {0};
    float rangeList[MAX_NUM_ADDRESSES];

	  // blast_info("Attempting to set registers for labjack%02d streaming.", m_state->which);
    // Disable streaming (otherwise we can't set the other streaming registers.
    labjack_set_short(0, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_ENABLE_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not disable streaming (could be streaming is off already): %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
    }
    // Write to appropriate Modbus registers to setup and start labjack streaming.
    labjack_set_float(scanRate, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SCANRATE_HZ_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream scan rate at address: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(numAddresses, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_NUM_ADDRESSES_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream number of addresses: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(samplesPerPacket, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SAMPLES_PER_PACKET_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set samples per packet: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_float(settling, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SETTLING_US_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream settling time: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(resolutionIndex, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_RESOLUTION_INDEX_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream resolution: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(bufferSizeBytes, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_BUFFER_SIZE_BYTES_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream buffer size: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(autoTarget, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_AUTO_TARGET_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set stream auto target: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
    labjack_set_short(numScans, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_NUM_SCANS_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not set continuous scanning: %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }

	 // blast_info("Setting Modbus register addresses for labjack%02d streaming.", m_state->which);

    // Using a loop to add Modbus addresses for AIN0 - AIN(NUM_ADDRESSES-1) to the
    // stream scan and configure the analog input settings.
    for (int i = 0; i < numAddresses; i++) {
        scanListAddresses[i] = i*2; // AIN(i) (Modbus address i*2)
        nChanList[0] = 199; // Negative channel is 199 (single ended)
        // rangeList[i] = 10.0; // 0.0 = +/-10V, 10.0 = +/-10V, 1.0 = +/-1V, 0.1 = +/-0.1V, or 0.01 = +/-0.01V.
	    labjack_set_short(scanListAddresses[i], data);
        m_state_number = m_state->which;
        if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_SCANLIST_ADDRESS_ADDR + i*2, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            if (!m_state->have_warned_write_reg) {
                blast_err("Could not set scan address %d: %s. Data sent [0]=%d, [1]=%d",
                    scanListAddresses[i], modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            m_state->has_comm_stream_error = 1;
            m_state->have_warned_write_reg = 1;
            return;
        }
        // old BLAST settings for ROXs to set range from -1 to 1 instead of -10 to 10
/*         if (m_state_number == 1) {
            rangeList[0] = 0.0;
            rangeList[1] = 0.0;
            rangeList[2] = 1.0;
            rangeList[3] = 1.0;
            rangeList[4] = 1.0;
            rangeList[5] = 1.0;
            rangeList[6] = 1.0;
            rangeList[7] = 1.0;
            rangeList[8] = 1.0;
            rangeList[9] = 1.0;
            rangeList[10] = 1.0;
            rangeList[11] = 0.0;
            rangeList[12] = 0.0;
            rangeList[13] = 0.0;
            labjack_set_float(rangeList[i], data);
        }
        if (!(m_state_number == 1)) {
            rangeList[i] = 0.0;
            labjack_set_float(rangeList[i], data);
        } */
        // set the range from -10 to 10
        rangeList[i] = 0.0;
        labjack_set_float(rangeList[i], data);

        if ((ret = modbus_write_registers(m_state->cmd_mb, AIN0_RANGE_ADDR + i*2, 2, data)) < 0) {
            ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
            int max_tries = 10;
            usleep(100);
            for (int tries = 1; tries < max_tries; tries++) {
                if ((ret = modbus_write_registers(m_state->cmd_mb, AIN0_RANGE_ADDR + i*2, 2, data)) < 0) {
                    ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
                    usleep(100);
                } else {
                    break;
                }
            }
            if (!m_state->have_warned_write_reg) {
                blast_err("Could not set %d-th AIN range: %s. Data sent [0]=%d, [1]=%d",
                    i, modbus_strerror(errno), data[0], data[1]);
                if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
            }
            m_state->has_comm_stream_error = 1;
            m_state->have_warned_write_reg = 1;
            return;
        } else {
        }
    }
    if ((ret = modbus_write_registers(m_state->cmd_mb, 43902, 1, nChanList)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
            blast_err("Could not set AIN negative channel: %s. Data sent %d", modbus_strerror(errno), nChanList[0]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
	// blast_info("Attempting to enable streaming for labjack%02d.", m_state->which);

	// Last step: enable streaming
    labjack_set_short(1, data);
    if ((ret = modbus_write_registers(m_state->cmd_mb, STREAM_ENABLE_ADDR, 2, data)) < 0) {
        ret = modbus_read_registers(m_state->cmd_mb, LJ_MODBUS_ERROR_INFO_ADDR, 2, err_data);
        if (!m_state->have_warned_write_reg) {
           blast_err("Could not enable streaming (could be streaming is off already): %s. Data sent [0]=%d, [1]=%d",
                modbus_strerror(errno), data[0], data[1]);
            if (ret > 0) blast_err("Specific labjack error code is: %d)", err_data[0]);
        }
        m_state->has_comm_stream_error = 1;
        m_state->have_warned_write_reg = 1;
        return;
    }
	// blast_info("Stream configuration commanding completed.");
	m_state->has_comm_stream_error = 0;
	m_state->have_warned_write_reg = 0;
	m_state->comm_stream_state = 1;
}


/**
 * @brief Handle a connection callback from @connect_lj.  The connection may succeed or fail.
 * If it fails, we increase the backoff time and reschedule another attempt.
 *
 * @param m_sock Pointer to the new sock that is created on a successful connection
 * @param m_status Status of the connection
 * @param m_errcode If the status indicates an error, this value is the errno
 * @param m_addr Unused
 * @param m_elapsed Unused
 * @param m_data Pointer to our LabJack State variable
 */
static void connected(ph_sock_t *m_sock, int m_status, int m_errcode, const ph_sockaddr_t *m_addr,
                      struct timeval *m_elapsed, void *m_data)
{
    ph_unused_parameter(m_elapsed);
    ph_unused_parameter(m_addr);
    labjack_state_t *state = (labjack_state_t*) m_data;

    switch (m_status) {
        case PH_SOCK_CONNECT_GAI_ERR:
            if (!state->have_warned_connect) blast_err("resolve %s:%d failed %s",
                    state->address, state->port, gai_strerror(m_errcode));
            state->have_warned_connect = 1;

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;

        case PH_SOCK_CONNECT_ERRNO:
            if (!state->have_warned_connect) blast_err("connect %s:%d failed: `Error %d: %s`",
                    state->address, state->port, m_errcode, strerror(m_errcode));
            state->have_warned_connect = 1;

            if (state->backoff_sec < max_backoff_sec) state->backoff_sec += 5;
            ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);
            return;
    }

    blast_info("Connected to LabJack at %s", state->address);
    state->have_warned_connect = 0;

    /// If we had an old socket from an invalid connection, free the reference here
    if (state->sock) ph_sock_free(state->sock);

    state->sock = m_sock;
    state->connected = true;
    // TODO(IAN): replace this when we have a LabJack command structure again
    // CommandData.Relays.labjack[state->which] = 1;
    state->backoff_sec = min_backoff_sec;
    m_sock->callback = labjack_process_stream;
    m_sock->timeout_duration.tv_sec = 2;
    m_sock->timeout_duration.tv_usec = 0;
    m_sock->job.data = state;
    ph_sock_enable(state->sock, true);
}


/**
 * @brief Process an incoming LabJack packet.  If we have an error, we'll disable
 * the socket and schedule a reconnection attempt.  Otherwise, read and store the
 * camera data.
 *
 * @param m_sock Unused
 * @param m_why Flag indicating why the routine was called
 * @param m_data Pointer to our state data
 */
void labjack_process_stream(ph_sock_t *m_sock, ph_iomask_t m_why, void *m_data)
{
    ph_buf_t *buf = NULL, *prev_buf = NULL;
    labjack_state_t *state = (labjack_state_t*) m_data;
    labjack_data_pkt_t *data_pkt;
    labjack_data_t *state_data = (labjack_data_t*)state->conn_data;
    static uint32_t gainList[MAX_NUM_ADDRESSES];
    static uint32_t gainList2[MAX_NUM_ADDRESSES];
    static labjack_device_cal_t labjack_cal;
    size_t read_buf_size;
    int ret, i, state_number;
    state_number = state->which;

    if (!state->calibration_read) {
        // old BLAST ROX readout gain change
        /* // gain index 0 = +/-10V. Used for conversion to volts.
        if (state_number == 1) {
            gainList2[0] = 0;
            gainList2[1] = 0;
            gainList2[2] = 1;
            gainList2[3] = 1;
            gainList2[4] = 1;
            gainList2[5] = 1;
            gainList2[6] = 1;
            gainList2[7] = 1;
            gainList2[8] = 1;
            gainList2[9] = 1;
            gainList2[10] = 1;
            gainList2[11] = 0;
            gainList2[12] = 0;
            gainList2[13] = 0;
        } else {
            
        } */
        for (i = 0; i < state_data->num_channels; i++) {
            gainList[i] = 0;
        }
        // For now read nominal calibration data (rather than specific calibration data from the device.
        // TODO(laura) fix labjack_get_cal and use that instead
        labjack_get_nominal_cal(state, &labjack_cal);
        //        labjack_get_cal(state, &labjack_cal);
    }
    /**
     * If we have an error, or do not receive data from the LabJack in the expected
     * amount of time, we tear down the socket and schedule a reconnection attempt.
     */
    if ((m_why & (PH_IOMASK_ERR|PH_IOMASK_TIME) || (state->force_reconnect))) {
        state->connected = false;

        // effectively resetting all the stuff from connect_lj
        blast_err("Reconnected Cmd and Data for LabJack at %s", state->address);
        ph_sock_shutdown(m_sock, PH_SOCK_SHUT_RDWR);
        ph_sock_enable(m_sock, 0);
        // TODO(IAN): replace this state variable once we have a labjack structure again
        // CommandData.Relays.labjack[state->which] = 0;
        state->force_reconnect = false;
        ph_job_set_timer_in_ms(&state->connect_job, state->backoff_sec * 1000);

        // restart command thread
        state->shutdown = true;
        while (state->cmd_mb) usleep(10000);
        state->shutdown = false;
        initialize_labjack_commands(state->which);
        return;
    }
    read_buf_size = sizeof(labjack_data_header_t) + state_data->num_channels * state_data->scans_per_packet * 2;

    // read as many packets as necessary to clear the queue
    do {
        if (prev_buf) ph_buf_delref(prev_buf); // had an old packet, but also have a newer packet, so remove old one
        prev_buf = buf; // the new packet is the next old packet
        buf = ph_sock_read_bytes_exact(m_sock, read_buf_size);
    } while (buf);
    buf = prev_buf; // reference the newest non-null packet

    if (!buf) return; /// We do not have (and never had) enough data
    data_pkt = (labjack_data_pkt_t*)ph_buf_mem(buf);

    // Correct for the fact that Labjack readout is MSB first.
    ret = labjack_data_word_swap(data_pkt, read_buf_size);
    if (data_pkt->header.resp.trans_id != ++(state_data->trans_id)) {
        // blast_warn("Expected transaction ID %d but received %d from LabJack at %s",
        //            state_data->trans_id, data_pkt->header.resp.trans_id, state->address);
    }
    state_data->trans_id = data_pkt->header.resp.trans_id;

    if (data_pkt->header.resp.type != STREAM_TYPE) {
        blast_warn("Unknown packet type %d received from LabJack at %s", data_pkt->header.resp.type, state->address);
        ph_buf_delref(buf);
        return;
    }

    // TODO(laura): Finish adding error handling.
    switch (data_pkt->header.status) {
        case STREAM_STATUS_AUTO_RECOVER_ACTIVE:
        case STREAM_STATUS_AUTO_RECOVER_END:
        case STREAM_STATUS_AUTO_RECOVER_END_OVERFLOW:
        case STREAM_STATUS_BURST_COMPLETE:
        case STREAM_STATUS_SCAN_OVERLAP:
            break;
    }

    memcpy(state_data->data, data_pkt->data, state_data->num_channels * sizeof(uint16_t));

    // Convert digital data into voltages.
    if (state->calibration_read) {
        if ((state_number == 1)) {
            labjack_convert_stream_data(state, &labjack_cal, gainList2, state_data->num_channels);
        } else {
            labjack_convert_stream_data(state, &labjack_cal, gainList, state_data->num_channels);
        }
    }
    ph_buf_delref(buf);
}


/**
 * @brief Handles the connection job.  Formatted this way to allow us to schedule
 * a future timeout in the PH_JOB infrastructure
 *
 * @param m_job Unused
 * @param m_why Unused
 * @param m_data Pointer to the labjack State variable
 */
static void connect_lj(ph_job_t *m_job, ph_iomask_t m_why, void *m_data)
{
    ph_unused_parameter(m_job);
    ph_unused_parameter(m_why);
    labjack_state_t *state = (labjack_state_t*)m_data;

    state->initialized = true;

    if (!state->have_warned_connect) blast_info("Connecting to %s", state->address);
    state->have_warned_connect = 1;
    ph_sock_resolve_and_connect(state->address, state->port, 0,
        &state->timeout, PH_SOCK_CONNECT_RESOLVE_SYSTEM, connected, m_data);
}


/**
 * @brief checks to see if any of the non multiplexed flight labjacks
 * are initialized. These are 0, 1, 2, 3, 4, 10(miscroscroll)
 * 
 * @return int 1 if any labjack is initalized 0 otherwise
 */
static int initialized(void) {
    if (state[0].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    }
    if (state[1].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    }
    if (state[2].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    }
    if (state[3].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    }
    if (state[4].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    }
    if (state[10].initialized) {
        // blast_info("a labjack was seen");
        return 1;
    } else {
        // blast_info("no labjack was seen");
        return 0;
    }
}


/**
 * @brief chooses the first non-multiplexed labjack that is connected to execute the command queue for all of them.
 * 
 */
void labjack_choose_execute(void) {
    int init = initialized();
    static bool has_warned = false;
    // blast_info("set_q = %d, init = %d", CommandData.Labjack_Queue.set_q, init);
    if (CommandData.Labjack_Queue.set_q == 1 && init) {
        // blast_info("setting cmd queue executor");
        if (state[0].connected == 1) {
            CommandData.Labjack_Queue.set_q = 0;
            CommandData.Labjack_Queue.which_q[0] = 1;
            blast_info("Labjack 1 is executing the command queue");
        } else if (state[1].connected == 1) {
            CommandData.Labjack_Queue.set_q = 0;
            CommandData.Labjack_Queue.which_q[1] = 1;
            blast_info("Labjack 2 is executing the command queue");
        } else if (state[2].connected == 1) {
            CommandData.Labjack_Queue.set_q = 0;
            CommandData.Labjack_Queue.which_q[2] = 1;
            blast_info("Labjack 3 is executing the command queue");
        } else if (state[3].connected == 1) {
            CommandData.Labjack_Queue.set_q = 0;
            CommandData.Labjack_Queue.which_q[3] = 1;
            blast_info("Labjack 4 is executing the command queue");
        } else if (state[4].connected == 1) {
            CommandData.Labjack_Queue.set_q = 0;
            CommandData.Labjack_Queue.which_q[4] = 1;
            blast_info("Labjack 5 is executing the command queue");
        } else if (state[9].connected == 1) {
        CommandData.Labjack_Queue.set_q = 0;
        CommandData.Labjack_Queue.which_q[9] = 1;
        blast_info("Labjack 10 is executing the command queue");
        } else {
            if (!has_warned) blast_info("no queue selected, trying again every 1s");
            has_warned = true;
        }
    }
}


/**
 * @brief forcibly sets which labjack is executing the command queue
 * 
 * @param which which labjack you want to be the executor
 */
void set_execute(int which) {
    // resetting executor
    CommandData.Labjack_Queue.which_q[0] = 0;
    CommandData.Labjack_Queue.which_q[1] = 0;
    CommandData.Labjack_Queue.which_q[2] = 0;
    CommandData.Labjack_Queue.which_q[3] = 0;
    CommandData.Labjack_Queue.which_q[4] = 0;
    CommandData.Labjack_Queue.which_q[5] = 0;
    CommandData.Labjack_Queue.which_q[6] = 0;
    CommandData.Labjack_Queue.which_q[7] = 0;
    CommandData.Labjack_Queue.which_q[8] = 0;
    CommandData.Labjack_Queue.which_q[9] = 0;
    CommandData.Labjack_Queue.which_q[10] = 0;
    CommandData.Labjack_Queue.which_q[which] = 1;
}


/**
 * @brief forces a reconnect attempt on the specified labjack
 * 
 * @param which which labjack to reconnect
 */
void set_reconnect(int which) {
    if (!InCharge) return;

    // force a reconnect of the stream data
    state[which].initialized = false;
    state[which].have_warned_connect = 0;
    state[which].force_reconnect = true;

    blast_info("Forced reconnect on %d\n", which);
}


/**
 * @brief endlessly looping thread which talks to the labjack over modbus
 * 
 * @param m_lj labjack structure to initialize this for
 * @return void* typical thread value, all NULL
 */
void *labjack_cmd_thread(void *m_lj) {
    labjack_state_t *m_state = (labjack_state_t*)m_lj;
    // int labjack = m_state->which;
    char tname[10];
    snprintf(tname, sizeof(tname), "LJCMD%1d", m_state->which);
    ph_thread_set_name(tname);
    nameThread(tname);

    // blast_info("Starting Labjack%02d Commanding at IP %s", m_state->which, m_state->address);
    m_state->req_comm_stream_state = 1;
    m_state->comm_stream_state = 0;
    m_state->has_comm_stream_error = 0;

    {
        struct hostent *lj_ent = gethostbyname(m_state->address);
        uint32_t hostaddr;
        if (!lj_ent) {
            blast_err("Could not resolve %s!", m_state->address);
            return NULL;
        }
        hostaddr = *(uint32_t*)(lj_ent->h_addr_list[0]);

        snprintf(m_state->ip, sizeof(m_state->ip), "%d.%d.%d.%d",
                 (hostaddr & 0xff), ((hostaddr >> 8) & 0xff),
                 ((hostaddr >> 16) & 0xff), ((hostaddr >> 24) & 0xff));
        // blast_info("Labjack%02d address %s corresponds to IP %s", m_state->which, m_state->address, m_state->ip);
    }
    while (!m_state->shutdown) {
        usleep(10000);
        if (!m_state->cmd_mb) {
            m_state->cmd_mb = modbus_new_tcp(m_state->ip, 502);

            modbus_set_slave(m_state->cmd_mb, 1);
            modbus_set_response_timeout(m_state->cmd_mb, 1, 0);
            modbus_set_error_recovery(m_state->cmd_mb,
                                      MODBUS_ERROR_RECOVERY_LINK | MODBUS_ERROR_RECOVERY_PROTOCOL);

            if (modbus_connect(m_state->cmd_mb)) {
                if (!m_state->have_warned_connect) {
                    blast_err("Could not connect to ModBUS at %s: %s", m_state->address,
                            modbus_strerror(errno));
                }
                modbus_free(m_state->cmd_mb);
                m_state->cmd_mb = NULL;
                m_state->have_warned_connect = 1;
                continue;
            }
            m_state->have_warned_connect = 0;
        }

        /*  Start streaming */
        if (m_state->req_comm_stream_state && !m_state->comm_stream_state) {
            init_labjack_stream_commands(m_state);
        }

        if (CommandData.Labjack_Queue.which_q[m_state->which] == 1) {
            // if (CommandData.Labjack_Queue.lj_q_on == 0) {
            //     blast_info("queue set by LJ %d", m_state->which);
            // }
            labjack_execute_command_queue();
            CommandData.Labjack_Queue.lj_q_on = 1;
        }
        /*
          // Set DAC level
            modbus_set_float(m_state->DAC[0], &dac_buffer[0]);
            modbus_set_float(m_state->DAC[1], &dac_buffer[2]);
            if (modbus_write_registers(m_state->cmd_mb, 1000, 4, dac_buffer) < 0) {
                if (!m_state->have_warned_write_reg) {
                    blast_err("Could not write DAC Modbus registers: %s", modbus_strerror(errno));
                }
                m_state->have_warned_write_reg = 1;
                continue;
            } */
    }

		// reset the q
		int qstate = CommandData.Labjack_Queue.which_q[m_state->which];
		CommandData.Labjack_Queue.which_q[m_state->which] = 0;
		if (qstate) CommandData.Labjack_Queue.set_q = 1;
        if (qstate) CommandData.Labjack_Queue.lj_q_on = 0;

    // close the modbus
    m_state->comm_stream_state = 0;
    modbus_close(m_state->cmd_mb);
    modbus_free(m_state->cmd_mb);
    m_state->cmd_mb = NULL;

    return NULL;
}


/**
 * @brief Create labjack commanding thread.
 * Called by mcp during startup.
 * 
 * @param m_which which labjack to inititalize
 * @return ph_thread_t* spawns the command thread
 */
ph_thread_t* initialize_labjack_commands(int m_which)
{
    // blast_info("start_labjack_command: creating labjack %d ModBus thread", m_which);
    return ph_thread_spawn(labjack_cmd_thread, (void*) &state[m_which]);
}


/**
 * @brief initializes the command queue structure for use in commanding labjacks
 * 
 */
void initialize_labjack_queue(void) {
    PH_STAILQ_INIT(&s_labjack_command);
}

/**
 * @brief Initialize the labjack I/O routine.  The state variable tracks each
 * labjack connection and is passed to the connect job.
 * 
 * @param m_which which labjack to initialize
 * @param m_numchannels number of AIN channels to call for
 * @param m_scans_per_packet how many scans of the AIN per packet (1)
 */
void labjack_networking_init(int m_which, size_t m_numchannels, size_t m_scans_per_packet)
{
    // blast_dbg("Labjack Init for %d", m_which);


    state[m_which].connected = false;
    // TODO(IAN): replace this when we have labjack commanddata again
    // CommandData.Relays.labjack[m_which] = 0;
    state[m_which].have_warned_version = false;
    state[m_which].backoff_sec = min_backoff_sec;
    state[m_which].timeout.tv_sec = 5;
    state[m_which].timeout.tv_usec = 0;
    ph_job_init(&(state[m_which].connect_job));
    state[m_which].connect_job.callback = connect_lj;
    state[m_which].connect_job.data = &state[m_which];
    labjack_data_t *data_state = calloc(1, sizeof(labjack_data_t) +
        m_numchannels * m_scans_per_packet * sizeof(uint16_t));
    data_state->num_channels = m_numchannels;
    data_state->scans_per_packet = m_scans_per_packet;
    state[m_which].conn_data = data_state;
    ph_job_dispatch_now(&(state[m_which].connect_job));
}


/**
 * @brief deprecated manner of storing the labjack AIN data in fancy named channels
 * 
 */
void store_labjack_data(void)
{
    static channel_t *LabjackCryoAINAddr[NUM_LABJACKS][NUM_LABJACK_AIN];
	char channel_name[128] = {0};
	int i, j;
    static int firsttime = 1;

    if (firsttime) {
        firsttime = 0;
        for (i = 0; i < NUM_LABJACKS; i++) {
            for (j = 0; j < NUM_LABJACK_AIN; j++) {
                snprintf(channel_name, sizeof(channel_name), "ain%02d%s", j, state[i].channel_postfix);
                LabjackCryoAINAddr[i][j] = channels_find_by_name(channel_name);
            }
        }
    }

    for (i = 0; i < NUM_LABJACKS; i++) {
        for (j = 0; j < NUM_LABJACK_AIN; j++) {
            SET_SCALED_VALUE(LabjackCryoAINAddr[i][j], state[i].AIN[j]);
//            blast_info("ain%02d%s = %f", j, state[i].channel_postfix, state[i].AIN[j]);
        }
    }
}


/**
 * @brief wrapper function which just holds all the calls for initializing the non-multiplexed labjacks.
 * we currently have this set to 5 different ones.
 * 
 * @param set_1 1/0 do we start up labjack 1
 * @param set_2 1/0 do we start up labjack 2
 * @param set_3 1/0 do we start up labjack 3
 * @param set_4 1/0 do we start up labjack 4
 * @param set_5 1/0 do we start up labjack 5
 * @param q_set do we start the command queue up?
 */
void init_labjacks(int set_1, int set_2, int set_3, int set_4, int set_5, int q_set) {
    if (q_set == 1) {
       initialize_labjack_queue();
    }
    if (set_1 == 1) {
        labjack_networking_init(LABJACK_OF_POWER, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
        initialize_labjack_commands(LABJACK_OF_POWER);
    }
    if (set_2 == 1) {
        labjack_networking_init(LABJACK_IF_POWER, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
        initialize_labjack_commands(LABJACK_IF_POWER);
    }
    if (set_3 == 1) {
        labjack_networking_init(LABJACK_MOTOR_POWER, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
        initialize_labjack_commands(LABJACK_MOTOR_POWER);
    }
    if (set_4 == 1) {
        labjack_networking_init(LABJACK_OF_2, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
        initialize_labjack_commands(LABJACK_OF_2);
    }
    if (set_5 == 1) {
        labjack_networking_init(LABJACK_OF_3, LABJACK_CRYO_NCHAN, LABJACK_CRYO_SPP);
        initialize_labjack_commands(LABJACK_OF_3);
    }
}
