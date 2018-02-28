/* 
 * linklist.c: 
 *
 * This software is copyright 
 *  (C) 2015-2018 University of Toronto, Toronto, ON
 *
 * This file is part of the SuperBIT project, modified and adapted for BLAST-TNG.
 *
 * linklist is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * linklist is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Feb 15, 2018 by Javier Romualdez
 */
/**
 * Description:
 *
 * This file contains functions for compressing and sending pilot data using linklists. 
 *
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

#include "mcp.h"
#include "FIFO.h"
#include "bitserver.h"
#include "linklist.h"
#include "linklist_compress.h"
#include "pilot.h"
#include "blast.h"
#include "mputs.h"
#include "command_struct.h"

extern int16_t InCharge;

struct Fifo pilot_fifo = {0};

void pilot_compress_and_send(void *arg) {
  // initialize UDP connection using bitserver/BITSender
  struct BITSender pilotsender = {0};
  unsigned int fifosize = MAX(PILOT_MAX_SIZE, allframe_size);
  initBITSender(&pilotsender, PILOT_ADDR, PILOT_PORT, 10, fifosize, PILOT_MAX_PACKET_SIZE);
  linklist_t * ll = NULL, * ll_old = NULL;
  linklist_t ** ll_array = arg;

  uint8_t * compbuffer = calloc(1, fifosize);
  int allframe_count = 0;
  uint32_t bandwidth = 0, transmit_size = 0;

  nameThread("Pilot");

  while (1) {
    // get the current pointer to the pilot linklist
    ll = ll_array[PILOT_TELEMETRY_INDEX];
    if (ll != ll_old) {
        if (ll) blast_info("Pilot linklist set to \"%s\"", ll->name);
        else blast_info("Pilot linklist set to NULL");
    }
    ll_old = ll;

    // get the current bandwidth
    bandwidth = CommandData.pilot_bw;

    if (!fifoIsEmpty(&pilot_fifo) && ll && InCharge) { // data is ready to be sent

      // send allframe if necessary
      if (!allframe_count) {
      //  write_allframe(compbuffer, getFifoRead(&pilot_fifo));
      //  setBITSenderFramenum(&pilotsender, allframe_size);
      //  sendToBITSender(&pilotsender, compbuffer, allframe_size, 0);
      }

      // compress the linklist
      int retval = compress_linklist(compbuffer, ll, getFifoRead(&pilot_fifo));
      decrementFifo(&pilot_fifo);

      if (!retval) continue;

      // compute the transmit size based on bandwidth
      transmit_size = MIN(ll->blk_size, bandwidth); // frames are 1 Hz, so bandwidth == size

      // have packet header serials match the linklist serials
      setBITSenderSerial(&pilotsender, *(uint32_t *) ll->serial);

      // commendeer the framenum for total transmit size
      setBITSenderFramenum(&pilotsender, transmit_size);

      // send the data to the ground station via bitsender
      sendToBITSender(&pilotsender, compbuffer, transmit_size, 0);

      memset(compbuffer, 0, PILOT_MAX_SIZE);
      allframe_count = (allframe_count + 1) % PILOT_ALLFRAME_PERIOD;
    } else {
      usleep(100000); // zzz...
    }
  }
}