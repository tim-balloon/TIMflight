/* 
 * evtm.h: Ethernet Via TeleMetry (EVTM) header file 
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

#include "bitserver.h"
#include "FIFO.h"
#include "linklist.h"

#ifndef INCLUDE_EVTM_H
#define INCLUDE_EVTM_H

#define EVTM_PORT_LOS 20001 // port that mcp will spew LOS evtm UDP data to
#define EVTM_ADDR_LOS "239.255.0.1" // IP that evtm LOS data is sent to

#define EVTM_PORT_TDRSS 20002 // port that mcp will spew evtm TRDSS UDP data to
#define EVTM_ADDR_TDRSS "239.255.0.2" // IP that evtm TRDSS data is sent to
// ground station (and groundhog) should be set up to listen to these IPs and ports

#define EVTM_MAX_PACKET_SIZE 1460 // maximum size of a packet to be sent over EVTM
// MTU (1500) - IPv4 header (20) - UDP header (8) - 12 (some other header stuff?)
// tcpdump gives bad udp length errors
#define EVTM_MAX_SIZE (superframe->size*2) // maximum compressed frame size to be send over EVTM

#define FIFO_LEN 10 // length of FIFO buffer

#define EVTM_NOT_TESTING 1 // set to 0 if testing

extern struct Fifo evtm_fifo_los;
extern struct Fifo evtm_fifo_tdrss;

enum evtmType {
    EVTM_LOS = 0,
    EVTM_TDRSS = 1,
    EVTM_NUM_TYPES = 2,
};

struct evtmInfo {
    void *telemetries;
    enum evtmType evtm_type;
};

struct evtmSetup {
    void* telemetries;
    enum evtmType evtm_type;
    int PORT;
    char *ADDR;
    int TELEMETRY_INDEX;
    struct Fifo *evtm_fifo;
    double BANDWIDTH;
    double ALLFRAME_FRACTION;
    struct BITSender evtm_sender;
    unsigned int fifosize;
    linklist_t * ll; 
    linklist_t * ll_old;
    linklist_t * ll_saved;
    linklist_t ** ll_array;
    uint8_t * compbuffer;
    unsigned int allframe_bytes;
    double bandwidth;
    uint32_t transmit_size;
};

int testing_evtm();
int setup_EVTM_config(struct evtmInfo *evtm_info, struct evtmSetup *evtm_setup);
void *infinite_loop_EVTM(struct evtmSetup *es);
void evtm_compress_and_send(struct evtmInfo *evtm_info);

#endif /* INCLUDE_EVTM_H */