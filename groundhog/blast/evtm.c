/**
 * @file evtm.c
 * @author Shubh Agrawal (shubh@sas.upenn.edu)
 * @brief This file is part of GroundHog, created for the Terahertz Intensity Mapper (TIM) project.
 * @date 2023-09-26
 * 
 * Copyright (c) 2023 University of Pennsylvania
 * 
 */

#include <math.h>
#include <stdbool.h>
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
#include <time.h>
#include <sys/time.h>

#include "groundhog_funcs.h"
#include "groundhog.h"
#include "bitserver.h"

extern struct TlmReport evtm_los_report = {0};
extern struct TlmReport evtm_tdrss_report = {0};

/**
 * @brief setup configuration for receiving UDP packets via EVTM
 * 
 * @param arg: pointer to the UDPSetup struct
 * @param es: pointer to the EVTMRecvSetup struct
 */
void EVTM_setup_receiver(struct UDPSetup *udpsetup, struct EVTMRecvSetup *es) {
  if (udpsetup->downlink_index == LOS_EVTM) {
    es->report = &evtm_los_report;
  } else if (udpsetup->downlink_index == TDRSS_EVTM) {
    es->report = &evtm_tdrss_report;
  } else {
    groundhog_fatal("Invalid downlink index for EVTM receiver\n");
  }
  es->udpsetup = udpsetup;
  es->recvbuffer = NULL;
  es->serial = 0;
  es->prev_serial = 0;
  es->ll = NULL;
  es->blk_size = 0;
  es->recv_size = 0;
  es->transmit_size = 0;
  es->framenum = 0;
  es->af = 0;

  es->local_allframe = calloc(1, superframe->allframe_size);
  es->ll_rawfile = NULL;
  es->compbuffer = calloc(1, udpsetup->maxsize);
  es->bad_serial_count = 0;

  // initialize UDP connection via bitserver/BITRecver
  initBITRecver(&es->udprecver, udpsetup->addr, udpsetup->port, FIFO_LEN, udpsetup->maxsize, udpsetup->packetsize);
}

/**
 * @brief body of infinite loop to get the linklist serial for the data received
 * 
 * @param es: pointer to the EVTMRecvSetup struct
 */
int EVTM_receiver_get_linklist(struct EVTMRecvSetup *es) {
  es->recvbuffer = getBITRecverAddr(&es->udprecver, &es->recv_size);
  printf("here1\n");
  printf("es: %p\n", es);
  printf("es->recvbuffer: %p\n", es->recvbuffer);
  // printf("func return value: %p\n", getBITRecverAddr(&es->udprecver, &es->recv_size));
  printf("(uint32_t *) es->recvbuffer: %p\n", (uint32_t *) es->recvbuffer);
  printf("*(uint32_t *) es->recvbuffer: %d\n", * (uint32_t *) es->recvbuffer);
  printf("here2\n");
  es->serial = *(uint32_t *) es->recvbuffer;
  printf("here3\n");
  groundhog_info("[%s] Receiving serial packets (0x%x)\n", es->udpsetup->name, es->serial);
  if (!(es->ll = linklist_lookup_by_serial(es->serial))) {
    removeBITRecverAddr(&es->udprecver);
    if (verbose) groundhog_info("[%s] Receiving bad serial packets (0x%x)\n", es->udpsetup->name, es->serial);
    es->bad_serial_count++;
    return 1;
  } else {
    es->bad_serial_count = 0;
    return 0;
  }
}

/**
 * @brief body of infinite loop for receiving UDP packets via EVTM
 * 
 * @param es: pointer to the EVTMRecvSetup struct
 */
void EVTM_receiver_loop_body(struct EVTMRecvSetup *es) {
  setBITRecverSerial(&es->udprecver, es->serial);
  es->blk_size = recvFromBITRecver(&es->udprecver, es->compbuffer, es->udpsetup->maxsize, 0);
  if (es->blk_size < 0) {
      groundhog_info("Malformed packed received on %s\n", es->udpsetup->name);
      return;
  }

  // hijacking frame number for transmit size
  es->transmit_size = es->udprecver.frame_num;

  if (groundhog_check_for_fileblocks(es->ll, FILE_LINKLIST)) {
    // unpack and extract to disk
    es->framenum = groundhog_unpack_fileblocks(es->ll, es->transmit_size, es->compbuffer, NULL,
                                               NULL, NULL, NULL, GROUNDHOG_EXTRACT_TO_DISK);
  } else { // write linklist data to disk
    // set flags for data extraction
    unsigned int flags = 0;
    if (es->serial != es->prev_serial) flags |= GROUNDHOG_OPEN_NEW_RAWFILE;
    es->prev_serial = es->serial;

    // process the linklist and write the data to disk
    es->framenum = groundhog_process_and_write(es->ll, es->transmit_size, es->compbuffer, es->local_allframe,
                                               es->udpsetup->name, es->udpsetup->name, &es->ll_rawfile, flags);
  }

  // fill out the telemetry report
  es->report->ll = es->ll;
  es->report->framenum = abs(es->framenum);
  es->report->allframe = es->af;
  memset(es->compbuffer, 0, es->udpsetup->maxsize);
}

/**
 * @brief enables the infinite loop in the EVTM receiver function
 * 
 * TODO(shubh): make this function work with a CommandData entry
 */
int EVTM_Recv_enable_loop() {
  return 1;
}


/**
 * @brief main groundhog function for receiving UDP packets via EVTM
 * 
 * @param arg: pointer to the UDPSetup struct
 */
void EVTM_udp_receive(void *arg) {
  struct EVTMRecvSetup es;
  EVTM_setup_receiver((struct UDPSetup *) arg, &es);
  while (EVTM_Recv_enable_loop()) {
    while (EVTM_receiver_get_linklist(&es)) { } // set up the linklist serial
    EVTM_receiver_loop_body(&es);
  }
}
