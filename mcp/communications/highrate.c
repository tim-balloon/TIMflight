/* mcp: the BLAST flight control program
 *
 * This software is copyright (C) 2018 Penn University
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>

// Javier's special sauce libraries
#include <linklist.h>
#include <linklist_compress.h>

#include "blast.h"
#include "mcp.h"
#include "command_struct.h"
#include "FIFO.h"
#include "bitserver.h"
#include "mputs.h"
#include "comms_serial.h"
#include "highrate.h"

struct Fifo highrate_fifo = {0};
struct Fifo sbd_fifo = {0};
linklist_t ** ll_array = NULL;
uint8_t * highrate_read_buffer = NULL;

/**
 * @brief Fills the science burst data buffer "b" with most important 255 bytes of the SBD linklist
 * 
 * @param b data buffer pointer
 * @param len size of the data buffer
 */
void fillSBData(unsigned char *b, int len)
{
    static linklist_t * sbd_ll = NULL, * sbd_ll_old = NULL;
    static uint8_t * compressed_buffer = NULL;
    static int first_time = 1;

    if (first_time) {
        compressed_buffer = calloc(1, MAX(HIGHRATE_MAX_SIZE, superframe->allframe_size));
        first_time = 0;
    }
    memset(b, 0, len);

    if (!ll_array) {
        return;
    }

    sbd_ll = ll_array[SBD_TELEMETRY_INDEX];
    if (sbd_ll != sbd_ll_old) {
        if (sbd_ll) {
            blast_info("SBD linklist set to \"%s\"", sbd_ll->name);
        } else {
            blast_info("SBD linklist set to NULL");
        }
    }
    sbd_ll_old = sbd_ll;

    // compress the data
    if (!highrate_read_buffer) {
        return;
    }
    compress_linklist(compressed_buffer, sbd_ll, highrate_read_buffer);

    memcpy(b+0, sbd_ll->serial, sizeof(uint32_t));
    memcpy(b+sizeof(uint32_t), compressed_buffer, len-sizeof(uint32_t));
}


/**
 * @brief Takes the array of available linklists and checks the current highrate desired linklist to
 * packetize and send down over the CSBF highrate link.
 * 
 * @param arg array of the various available linklists
 */
void highrate_compress_and_send(void *arg)
{
    linklist_t * ll = NULL;
    linklist_t * ll_old = NULL;
    linklist_t * ll_saved = NULL;
    comms_serial_t * serial = comms_serial_new(NULL);

    ll_array = arg;

    unsigned int fifosize = MAX(HIGHRATE_MAX_SIZE, superframe->allframe_size);
    unsigned int csbf_packet_size = HIGHRATE_DATA_PACKET_SIZE+CSBF_HEADER_SIZE+HIGHRATE_CHECKSUM_SIZE;
    uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
    unsigned int buffer_size = ((fifosize-1)/datasize+1)*datasize;

    uint8_t * csbf_packet = calloc(1, csbf_packet_size);
    uint8_t * csbf_header = csbf_packet+0;
    uint8_t * header_buffer = csbf_header+CSBF_HEADER_SIZE;
    uint8_t * data_buffer = header_buffer+PACKET_HEADER_SIZE;
    uint8_t * csbf_checksum = header_buffer+HIGHRATE_DATA_PACKET_SIZE;

    uint8_t * compbuffer = calloc(1, buffer_size);
    unsigned int allframe_bytes = 0;
    double bandwidth = 0;
    uint32_t transmit_size = 0;
    int i;
    int get_serial_fd = 1;

    // packetization variables
    uint16_t i_pkt = 0;
    uint16_t n_pkt = 0;
    unsigned int backoff = 2;
    unsigned int max_backoff = 60;

    nameThread("Highrate");

    while (true) {
        while (get_serial_fd) {
            if ((comms_serial_connect(serial, HIGHRATE_PORT) == NETSOCK_OK) &&
                comms_serial_setspeed(serial, B115200)) {
                break;
            }
            sleep(backoff);
            blast_info("Could not connect to highrate port. Will try again in %d seconds...", backoff);
            if (backoff < max_backoff) {
                backoff *= 2;
            }
            if (backoff > max_backoff) {
                backoff = max_backoff;
            }
        }
        get_serial_fd = 0;

        // get the current pointer to the pilot linklist
        ll = ll_array[HIGHRATE_TELEMETRY_INDEX];
        if (ll != ll_old) {
            if (ll) {
                blast_info("Highrate linklist set to \"%s\"", ll->name);
            } else {
                blast_info("Highrate linklist set to NULL");
            }
        }
        ll_old = ll;

        // get the current bandwidth
        if ((bandwidth != CommandData.highrate_bw) || (CommandData.highrate_allframe_fraction < 0.001)) {
            allframe_bytes = 0;
        }
        bandwidth = CommandData.highrate_bw;

        if (!fifoIsEmpty(&highrate_fifo) && ll) { // data is ready to be sent
            highrate_read_buffer = getFifoRead(&highrate_fifo);

            if (!strcmp(ll->name, FILE_LINKLIST)) { // special file downlinking
                // done sending, so revert to other linklist
                if (ll->blocks[0].i >= ll->blocks[0].n) {
                    ll_array[HIGHRATE_TELEMETRY_INDEX] = ll_saved;
                    continue;
                }
                // use the full bandwidth
                transmit_size = bandwidth;

                // fill the downlink buffer as much as the downlink will allow
                unsigned int bytes_packed = 0;
                while ((bytes_packed+ll->blk_size) <= transmit_size) {
                    compress_linklist(compbuffer+bytes_packed, ll, highrate_read_buffer);
                    bytes_packed += ll->blk_size;
                }
                decrementFifo(&highrate_fifo);

            } else { // normal linklist
                ll_saved = ll;

                // send allframe if necessary
                if (allframe_bytes >= superframe->allframe_size) {
                    transmit_size = write_allframe(compbuffer, superframe, highrate_read_buffer);
                    allframe_bytes = 0;
                } else {
                    // bandwidth limit; frames are 1 Hz, so bandwidth == size
                    transmit_size = MIN(ll->blk_size, bandwidth*(1.0-CommandData.highrate_allframe_fraction));

                    // compress the linklist
                    compress_linklist(compbuffer, ll, highrate_read_buffer);

                    // bandwidth limit; frames are 1 Hz, so bandwidth == size
                    allframe_bytes += bandwidth*CommandData.highrate_allframe_fraction;
                    decrementFifo(&highrate_fifo);
                }
            }

                    // no packetization if there is nothing to transmit
            if (!transmit_size) {
                continue;
            }

            // set initialization for packetization
            uint8_t * chunk = NULL;
            uint32_t chunksize = datasize;
            i_pkt = 0;
            n_pkt = 1;

            // write the CSBF header
            csbf_header[0] = HIGHRATE_SYNC1;
            csbf_header[1] = (CommandData.highrate_through_tdrss) ? HIGHRATE_TDRSS_SYNC2 : HIGHRATE_IRIDIUM_SYNC2;
            csbf_header[2] = HIGHRATE_ORIGIN_COMM1;
            csbf_header[3] = 0x69; // !zero

            while ((i_pkt < n_pkt) && (chunk = packetizeBuffer(compbuffer, transmit_size,
                                            &chunksize, &i_pkt, &n_pkt))) {
                // set the size for the csbf header
                csbf_header[4] = ((chunksize+PACKET_HEADER_SIZE) >> 8) & 0xff; // msb of size
                csbf_header[5] = (chunksize+PACKET_HEADER_SIZE) & 0xff;  // lsb of size

                // have packet header serials match the linklist serials
                writeHeader(header_buffer, *(uint32_t *) ll->serial, transmit_size, i_pkt, n_pkt);

                // copy the data to the csbf packet
                memcpy(data_buffer, chunk, chunksize);

                // compute checksum
                csbf_checksum = data_buffer+chunksize;
                *csbf_checksum = 0;
                for (i = 2; i < CSBF_HEADER_SIZE; i++) *csbf_checksum += csbf_header[i];
                for (i = 0; i < PACKET_HEADER_SIZE; i++) *csbf_checksum += header_buffer[i];
                for (i = 0; i < chunksize; i++) *csbf_checksum += chunk[i];
                /*
                for (i = 0; i < csbf_packet_size; i++) {
                if (i % 32 == 0) printf("\n");
                printf("0x%.2x ", csbf_packet[i]);
                }
                printf("\n");

                blast_info("Transmit size %d, datasize %d, i %d, n %d, csbf_packet_size %d, checksum 0x%x", transmit_size, datasize, i_pkt, n_pkt, csbf_packet_size, *csbf_checksum);
                */

                // send the full packet
                unsigned int sendsize = chunksize+CSBF_HEADER_SIZE+PACKET_HEADER_SIZE+HIGHRATE_CHECKSUM_SIZE;
                int wrote = write(serial->sock->fd, csbf_packet, sendsize);
                if (wrote < 0) { // send csbf header
                    get_serial_fd = 1;
                    break;
                } else if (wrote != sendsize) {
                    blast_err("Could only send %d/%d bytes\n", wrote, sendsize);
                }

                memset(data_buffer, 0, HIGHRATE_DATA_PACKET_SIZE - PACKET_HEADER_SIZE + HIGHRATE_CHECKSUM_SIZE);

                i_pkt++;
                usleep(1000);
            }

            memset(compbuffer, 0, buffer_size);
        } else {
            usleep(100000);
        }
    }
}
