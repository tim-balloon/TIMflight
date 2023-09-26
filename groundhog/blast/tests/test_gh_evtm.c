/**
 * @file test_gh_evtm.c
 * @author Shubh Agrawal (shubh@sas.upenn.edu)
 * @brief This file is part of GroundHog, created for the Terahertz Intensity Mapper (TIM) project.
 * @date 2023-09-26
 * 
 * Copyright (c) 2023 University of Pennsylvania
 * 
 */

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include <float.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <netinet/in.h> // socket stuff
#include <sys/socket.h> // socket stuff

#include "linklist.h"
#include "linklist_connect.h"
#include "groundhog_funcs.h"
#include "FIFO.h"
#include "blast.h"
#include "derived.h"
#include "bitserver.h"
#include "channels_tng.h"
#include "evtm.h"
#include "groundhog.h"

struct UDPSetup evtm_los_setup = {0};
struct UDPSetup evtm_tdrss_setup = {0};

struct TlmReport evtm_los_report = {0};
struct TlmReport evtm_tdrss_report = {0};

// ----------------------------------------------------------------------
// Wrapper Functions
// ----------------------------------------------------------------------
// if compiler is invoked with --wrap=<func>, then the linker will resolve
// <func> to __wrap_<func> instead of <func>.
// this allows us to mock BITServer behavior.

// function_called allows us to check how many times that function was called
// unfortunately, it does not let us check if that count is 0 :(

void __wrap_initBITRecver(struct BITRecver *server, const char *recv_addr,
    unsigned int port, unsigned int fifo_length,  unsigned int fifo_maxsize, unsigned int packet_maxsize) {
        check_expected(recv_addr);
        check_expected(port);
        check_expected(fifo_length);
        check_expected(fifo_maxsize);
        check_expected(packet_maxsize);
        function_called();
    }

/**
 * @brief Helper function to get a EVTMRecvSetup struct needed in unit tests
 * 
 */
struct EVTMRecvSetup get_evtm_recv_setup_variables(DownLinkTypes evtm_type, \
            struct UDPSetup *udpsetup) {
    if (evtm_type != LOS_EVTM && evtm_type != TDRSS_EVTM) {
        groundhog_error("Invalid downlink index for EVTM receiver in test file\n");
    }
    struct EVTMRecvSetup evtm_recv_setup = {
        .report = (evtm_type == LOS_EVTM) ? &evtm_los_report : &evtm_tdrss_report,
        .udpsetup = udpsetup,
        .recvbuffer = NULL,
        .serial = 0,
        .prev_serial = 0,
        .ll = NULL,
        .blk_size = 0,
        .recv_size = 0,
        .transmit_size = 0,
        .framenum = 0,
        .af = 0,
        .local_allframe = NULL,
        .ll_rawfile = NULL,
        .compbuffer = NULL,
        .bad_serial_count = 0
    };
    return evtm_recv_setup;
}

/**
 * @brief setup configuration for testing EVTM groundhog code
 * 
 * @return int 
 */
static int GH_EVTM_setup_unit_tests(void **state) {
    channels_initialize(channel_list);
    linklist_t ** ll_list = calloc(MAX_NUM_LINKLIST_FILES, sizeof(linklist_t *));
    load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, ll_list, LL_INCLUDE_ALLFRAME);
    evtm_los_setup = {"EVTM_LOS", EVTM_ADDR_LOS, EVTM_PORT_LOS, EVTM_MAX_SIZE, \
                        EVTM_MAX_PACKET_SIZE, LOS_EVTM};
    evtm_tdrss_setup = {"EVTM_TDRSS", EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, EVTM_MAX_SIZE, \
                        EVTM_MAX_PACKET_SIZE, TDRSS_EVTM};
    return 0;
}

void test_EVTM_setup_receiver(void **state) {
    assert_true(1);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_EVTM_setup_receiver, GH_EVTM_setup_unit_tests, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
