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
 * @brief Setup pre-loop EVTM configuration.
 *
 * @param evtm_info Struct containing telemetries and evtm type.
 * @param evtm_setup Struct that will contain setup information for the evtm.
 * 
 * @return 0 if successful, -1 if not.
 */
int setup_EVTM_config(struct evtmInfo *evtm_info, struct evtmSetup *evtm_setup) {
    evtm_setup->telemetries = evtm_info->telemetries;
    evtm_setup->evtm_type = evtm_info->evtm_type;

    if (evtm_setup->evtm_type == EVTM_LOS) {
        // send to LOS multicast address
        evtm_setup->PORT = EVTM_PORT_LOS;
        evtm_setup->ADDR = EVTM_ADDR_LOS;
        evtm_setup->TELEMETRY_INDEX = EVTM_LOS_TELEMETRY_INDEX;
        evtm_setup->evtm_fifo = &evtm_fifo_los;
    } else if (evtm_setup->evtm_type == EVTM_TDRSS) {
        // send to TDRSS multicast address
        evtm_setup->PORT = EVTM_PORT_TDRSS;
        evtm_setup->ADDR = EVTM_ADDR_TDRSS;
        evtm_setup->TELEMETRY_INDEX = EVTM_TDRSS_TELEMETRY_INDEX;
        evtm_setup->evtm_fifo = &evtm_fifo_tdrss;
    } else {
        blast_fatal("Invalid evtm type %d", evtm_setup->evtm_type);
    }

    blast_info("Setting up EVTM %d, %s", evtm_setup->evtm_type, evtm_setup->ADDR);

    evtm_setup->evtm_sender = (struct BITSender) {0};
    evtm_setup->fifosize = MAX(EVTM_MAX_SIZE, superframe->allframe_size);
    int rc = initBITSender(&evtm_setup->evtm_sender, evtm_setup->ADDR, evtm_setup->PORT, \
                FIFO_LEN, evtm_setup->fifosize, EVTM_MAX_PACKET_SIZE);
    if (rc != 1) { // failing gracefully
        blast_fatal("initializing BITSender did not work for EVTM %s: check above error msg", evtm_setup->ADDR);
    }

    evtm_setup->ll = NULL;
    evtm_setup->ll_old = NULL;
    evtm_setup->ll_saved = NULL;
    evtm_setup->ll_array = evtm_setup->telemetries;

    evtm_setup->compbuffer = calloc(1, evtm_setup->fifosize);
    evtm_setup->allframe_bytes = 0;
    evtm_setup->bandwidth = 0;
    evtm_setup->transmit_size = 0;

    char *thread_name;
    asprintf(&thread_name, "EVTM %d: %s", evtm_setup->evtm_type, evtm_setup->ADDR);
    nameThread(thread_name);

    return 0;
}


// TODO(shubh): perhaps make this reliant on a CommandData value later on TBD
/**
 * @brief Checks if the evtm is being tested.
 * 
 * @return 1 if not testing, 0 if testing.
 */
int testing_evtm() {
    return 0; // default behavior is to not test and run the infinite loop
    // we make a mock function when testing around this function
}

/**
 * @brief Infinite loop for sending data via EVTM.
 *
 * @param es (short for evtmSetup) Struct containing setup information for the evtm.
 */
void *infinite_loop_EVTM(struct evtmSetup *es) {
    linklist_t * ll = es->ll;
    linklist_t * ll_old = es->ll_old;
    linklist_t * ll_saved = es->ll_saved;
    ll = es->ll_array[es->TELEMETRY_INDEX];
    if (ll != ll_old) {
        if (ll) {
            blast_info("EVTM %d, %s: serial is 0x%x", es->evtm_type, es->ADDR, \
                    *(uint32_t *) ll->serial);
            blast_info("EVTM %d, %s: linklist set to \"%s\"", es->evtm_type, \
                    es->ADDR, ll->name);
        } else {
            blast_info("EVTM %d, %s: linklist set to NULL", es->evtm_type, es->ADDR);
        }
    }
    ll_old = ll;

    // get the current bandwidth
    if (es->evtm_type == EVTM_LOS) {
        es->BANDWIDTH = CommandData.biphase_bw;
        es->ALLFRAME_FRACTION = CommandData.biphase_allframe_fraction;
    } else if (es->evtm_type == EVTM_TDRSS) {
        es->BANDWIDTH = CommandData.highrate_bw;
        es->ALLFRAME_FRACTION = CommandData.highrate_allframe_fraction;
    }
    if ((es->bandwidth != es->BANDWIDTH) || (es->ALLFRAME_FRACTION < 0.0001)) {
            es->allframe_bytes = 0;
    }
    es->bandwidth = es->BANDWIDTH;

    if (!fifoIsEmpty(es->evtm_fifo) && ll && InCharge) { // data ready to send
        if (!strcmp(ll->name, FILE_LINKLIST)) {
            if (ll->blocks[0].i >= ll->blocks[0].n) {
                es->ll_array[es->TELEMETRY_INDEX] = ll_saved;
                return;
            }

            // use the full bandwidth
            es->transmit_size = es->bandwidth;

            // fill the downlink buffer as much as the downlink will allow
            unsigned int bytes_packed = 0;
            while ((bytes_packed + ll->blk_size) <= es->transmit_size) {
                compress_linklist(es->compbuffer + bytes_packed, ll, getFifoRead(es->evtm_fifo));
                bytes_packed += ll->blk_size;
            }
            decrementFifo(es->evtm_fifo);
        } else { // normal linklist
            ll_saved = ll;

            // send allframe if necessary
            if (es->allframe_bytes >= superframe->allframe_size) {
                es->transmit_size = write_allframe(es->compbuffer, superframe, getFifoRead(es->evtm_fifo));
                es->allframe_bytes = 0;
            } else {
                es->transmit_size = MIN(ll->blk_size, es->bandwidth * (1.0 - es->ALLFRAME_FRACTION));
                // compress the linklist
                compress_linklist(es->compbuffer, ll, getFifoRead(es->evtm_fifo));

                // bandwidth limit; frames are 1 Hz, so bandwidth == size
                es->allframe_bytes += es->bandwidth * es->ALLFRAME_FRACTION;
                decrementFifo(es->evtm_fifo);
            }
        }

        // no packetization if there is nothing to transmit
        if (!es->transmit_size) {
            return;
        }
        // send the data via bitsender
        setBITSenderSerial(&es->evtm_sender, *(uint32_t *) ll->serial);
        setBITSenderFramenum(&es->evtm_sender, es->transmit_size);
        sendToBITSender(&es->evtm_sender, es->compbuffer, es->transmit_size, 0);
        memset(es->compbuffer, 0, EVTM_MAX_SIZE);
    } else {
        usleep(100000);
    }
}



/**
 * @brief Takes a pointer to a list of telemetries, and a type of evtm (LOS or TDRSS)
 * and compresses the data into a packet and sends it over UDP multicast.
 *
 * @param evtm_info Struct containing telemetries and evtm type.
 */
void evtm_compress_and_send(struct evtmInfo *evtm_info) {
    struct evtmSetup evtm_setup = {{0}};
    if (setup_EVTM_config(evtm_info, &evtm_setup) != 0) {
        blast_fatal("EVTM setup failed");
    }
    while (!testing_evtm()) {
        infinite_loop_EVTM(&evtm_setup);
    }
}
