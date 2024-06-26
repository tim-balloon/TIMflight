/**
 * @file groundhog.h
 * 
 * Copyright 20nn ********
 */ 

#ifndef INCLUDE_GROUNDHOG_H
#define INCLUDE_GROUNDHOG_H

#include "groundhog_funcs.h"

/* ------ BEGIN CUSTOM GROUNDHOG DEFINITIONS ------ */

// groundhog customization
#define GROUNDHOG_MAX_FRAMES_RESET 900
#define GSE_PACKET_HEADER_SIZE 2048
#define GSE_PACKET_HEADER_ORIGIN 0xe

#define DEFAULT_ARCHIVE_DIR "/data/groundhog/"

// BLAST general
#include "blast.h"
#include "blast_time.h"
#define ROACH_CHANNEL_REF_NAME "kidA_roachN"
#define ROACH_CHANNEL_REF_INDEX_NAME "kidA_roachN_index"

// BLAST telemetry
#include "channels_tng.h"
#include "derived.h"
#include "FIFO.h"
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a):(b))
#endif

// TODO(shubh): remove deprecated telemetry links after testing
enum DownLinkTypes {PILOT, BI0, HIGHRATE, LOS_EVTM, TDRSS_EVTM,
                      NUM_DOWNLINKS};

// BLAST pilot
#include "bitserver.h"
#include "pilot.h"
void udp_receive(void *arg);

// BLAST biphase
#include "bbc_pci.h"
#include "decom_pci.h"
#include "bi0.h"
void biphase_receive(void *arg);

// BLAST highrate
#define CSBFHeader_NAMESTR_SIZE 80
#define LINKNAME_SIZE 80
#include "highrate.h"
#include "comms_serial.h"
void highrate_receive(void *arg);

// TIM EVTM
struct EVTMRecvSetup {
    struct TlmReport *report;
    struct UDPSetup *udpsetup;
    struct BITRecver udprecver;
    uint8_t *recvbuffer;
    uint32_t serial;
    uint32_t prev_serial;
    linklist_t *ll;
    int32_t blk_size;
    uint32_t recv_size;
    uint32_t transmit_size;
    int64_t framenum;
    int af;
    uint8_t *local_allframe;
    linklist_rawfile_t *ll_rawfile;
    uint8_t *compbuffer;
    int bad_serial_count;
};

#define FIFO_LEN 10
#include "evtm.h"

void EVTM_setup_receiver(struct UDPSetup *udpsetup, struct EVTMRecvSetup *es);
int EVTM_receiver_get_linklist(struct EVTMRecvSetup *es);
void EVTM_receiver_loop_body(struct EVTMRecvSetup *es);
int EVTM_Recv_enable_loop();
void EVTM_udp_receive(void *arg);

#endif // INCLUDE_GROUNDHOG_H
