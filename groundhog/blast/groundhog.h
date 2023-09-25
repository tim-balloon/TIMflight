#include "groundhog_funcs.h"

/* ------ BEGIN CUSTOM GROUNDHOG DEFINITIONS ------ */

// groundhog customization
#define GROUNDHOG_MAX_FRAMES_RESET 900

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
enum DownLinkTypes {PILOT, BI0, HIGHRATE, TDRSS_EVTM, LOS_EVTM,
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
#include "highrate.h"
#include "comms_serial.h"
void highrate_receive(void *arg);

// TIM EVTM
struct EVTMRecvSetup {
    struct TlmReport *report;
    struct UDPSetup *udpsetup;
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
#include "evtm.h"
void EVTM_setup_receiver(void *arg, struct EVTMRecvSetup *es);
void EVTM_receiver_get_linklist(struct EVTMRecvSetup *es);
void EVTM_receiver_loop_body(struct EVTMRecvSetup *es);
void EVTM_udp_receive(void *arg);
