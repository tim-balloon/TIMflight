/* 
 * evtm.c: Ethernet Via TeleMetry (EVTM)
 * Used for sending UDP multicast packets via LOS and TDRSS links
 * 
 * This software  is copyright 
 *  (C) University of Pennsylvania, Philadelphia 2023
 *
 * This file is part of mcp, as used for the Terahertz Intensity Mapper (TIM).
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
 * Created on: Sep 19, 2018 by Shubh Agrawal
 */

#include <math.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>

// Javier's special libraries
#include <linklist.h>
#include <linklist_compress.h>

#include "mcp.h"
#include "FIFO.h"
#include "bitserver.h"
#include "evtm.h"
#include "blast.h"
#include "mputs.h"
#include "command_struct.h"

extern int16_t InCharge;

struct Fifo evtm_fifo_los = {0};
struct Fifo evtm_fifo_tdrss = {0};

/**
 * @brief Takes a pointer to a list of telemetries, and a type of evtm (LOS or TDRSS)
 * and compresses the data into a packet and sends it over UDP multicast.
 *
 * @param telemetries List of available telemetries.
 * @param evtm_type Type of evtm (LOS or TDRSS)
 */
void evtm_compress_and_send(struct evtmInfo *evtm_info) {
    void *telemetries = evtm_info->telemetries;
    enum evtmType evtm_type = evtm_info->evtm_type;

    int PORT;
    char *ADDR;
    int TELEMETRY_INDEX;
    struct Fifo *evtm_fifo;
    double BANDWIDTH;
    double ALLFRAME_FRACTION;

    if (evtm_type == EVTM_LOS) {
        // send to LOS multicast address
        PORT = EVTM_PORT_LOS;
        ADDR = EVTM_ADDR_LOS;
        TELEMETRY_INDEX = EVTM_LOS_TELEMETRY_INDEX;
        evtm_fifo = &evtm_fifo_los;
    } else if (evtm_type == EVTM_TDRSS) {
        // send to TDRSS multicast address
        PORT = EVTM_PORT_TDRSS;
        ADDR = EVTM_ADDR_TDRSS;
        TELEMETRY_INDEX = EVTM_TDRSS_TELEMETRY_INDEX;
        evtm_fifo = &evtm_fifo_tdrss;
    } else {
        blast_fatal("Invalid evtm type %d", evtm_type);
    }

    blast_info("Setting up EVTM %d, %s", evtm_type, ADDR);

    struct BITSender evtm_sender = {0};
    unsigned int fifosize = MAX(EVTM_MAX_SIZE, superframe->allframe_size);
    int rc = initBITSender(&evtm_sender, ADDR, PORT, FIFO_LEN, fifosize, EVTM_MAX_PACKET_SIZE);
    if (rc != 1) { // failing gracefully
        blast_fatal("initializing BITSender did not work for EVTM %s: check above error msg", ADDR);
    }

    linklist_t * ll = NULL, * ll_old = NULL, * ll_saved = NULL;
    linklist_t ** ll_array = telemetries;

    uint8_t * compbuffer = calloc(1, fifosize);
    unsigned int allframe_bytes = 0;
    double bandwidth = 0;
    uint32_t transmit_size = 0;

    char *thread_name;
    asprintf(&thread_name, "EVTM %d: %s", evtm_type, ADDR);
    nameThread(thread_name);

    while (1) {
        ll = ll_array[TELEMETRY_INDEX];
        if (ll != ll_old) {
            if (ll) {
                blast_info("EVTM %d, %s: serial is 0x%x", evtm_type, ADDR, *(uint32_t *) ll->serial);
                blast_info("EVTM %d, %s: linklist set to \"%s\"", evtm_type, ADDR, ll->name);
            } else {
                blast_info("EVTM %d, %s: linklist set to NULL", evtm_type, ADDR);
            }
        }
        ll_old = ll;

        // get the current bandwidth
        if (evtm_type == EVTM_LOS) {
            BANDWIDTH = CommandData.biphase_bw;
            ALLFRAME_FRACTION = CommandData.biphase_allframe_fraction;
        } else if (evtm_type == EVTM_TDRSS) {
            BANDWIDTH = CommandData.highrate_bw;
            ALLFRAME_FRACTION = CommandData.highrate_allframe_fraction;
        }
        if ((bandwidth != BANDWIDTH) || (ALLFRAME_FRACTION < 0.0001)) {
                allframe_bytes = 0;
        }
        bandwidth = BANDWIDTH;

        if (!fifoIsEmpty(evtm_fifo) && ll && InCharge) { // data ready to send
            if (!strcmp(ll->name, FILE_LINKLIST)) {
                if (ll->blocks[0].i >= ll->blocks[0].n) {
                    ll_array[TELEMETRY_INDEX] = ll_saved;
                    continue;
                }

                // use the full bandwidth
                transmit_size = bandwidth;

                // fill the downlink buffer as much as the downlink will allow
                unsigned int bytes_packed = 0;
                while ((bytes_packed + ll->blk_size) <= transmit_size) {
                    compress_linklist(compbuffer + bytes_packed, ll, getFifoRead(evtm_fifo));
                    bytes_packed += ll->blk_size;
                }
                decrementFifo(evtm_fifo);
            } else { // normal linklist
                ll_saved = ll;

                // send allframe if necessary
                if (allframe_bytes >= superframe->allframe_size) {
                    transmit_size = write_allframe(compbuffer, superframe, getFifoRead(evtm_fifo));
                    allframe_bytes = 0;
                } else {
                    transmit_size = MIN(ll->blk_size, bandwidth * (1.0 - ALLFRAME_FRACTION));
                    // compress the linklist
                    compress_linklist(compbuffer, ll, getFifoRead(evtm_fifo));

                    // bandwidth limit; frames are 1 Hz, so bandwidth == size
                    allframe_bytes += bandwidth * ALLFRAME_FRACTION;
                    decrementFifo(evtm_fifo);
                }
            }

            // no packetization if there is nothing to transmit
            if (!transmit_size) {
                continue;
            }
            // send the data via bitsender
            setBITSenderSerial(&evtm_sender, *(uint32_t *) ll->serial);
            setBITSenderFramenum(&evtm_sender, transmit_size);
            sendToBITSender(&evtm_sender, compbuffer, transmit_size, 0);
            memset(compbuffer, 0, EVTM_MAX_SIZE);
        } else {
            usleep(100000);
        }
    }
}
