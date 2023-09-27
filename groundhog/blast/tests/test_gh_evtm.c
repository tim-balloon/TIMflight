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
#include "groundhog.h"
#include "groundhog_funcs.c"
#include "FIFO.h"
#include "blast.h"
#include "derived.h"
#include "bitserver.h"
#include "channels_tng.h"
#include "evtm.c"

// ----------------------------------------------------------------------
// Wrapper Functions
// ----------------------------------------------------------------------
// if compiler is invoked with --wrap=<func>, then the linker will resolve
// <func> to __wrap_<func> instead of <func>.
// this allows us to mock BITServer behavior.

// function_called allows us to check how many times that function was called
// unfortunately, it does not let us check if that count is 0 :(

void __wrap_groundhog_write_calspecs(char *fname) {
    channels_write_calspecs(fname, derived_list);
}

void __wrap_initBITRecver(struct BITRecver *server, const char *recv_addr,
    unsigned int port, unsigned int fifo_length,  unsigned int fifo_maxsize, unsigned int packet_maxsize) {
    check_expected(recv_addr);
    check_expected(port);
    check_expected(fifo_length);
    check_expected(fifo_maxsize);
    check_expected(packet_maxsize);
    function_called();
}

uint8_t *__wrap_getBITRecverAddr(struct BITRecver *server, unsigned int *size) {
    check_expected(size);
    function_called();
    return mock_ptr_type(uint8_t);
}

void __wrap_removeBITRecverAddr(struct BITRecver *server) {
    function_called();
}

void __wrap_setBITRecverSerial(struct BITRecver *server, uint32_t serial) {
    check_expected(serial);
    function_called();
}

int __wrap_recvFromBITRecver(struct BITRecver *server, uint8_t *buffer, unsigned int *size, uint8_t flags) {
    check_expected(buffer);
    check_expected(size);
    check_expected(flags);
    function_called();
    return mock_type(int);
}

/**
 * @brief Helper function to get a EVTMRecvSetup struct needed in unit tests
 * 
 */
struct EVTMRecvSetup get_evtm_recv_struct(int evtm_type, struct UDPSetup *udpsetup) {
    if (evtm_type != LOS_EVTM && evtm_type != TDRSS_EVTM) {
        groundhog_fatal("Invalid downlink index for EVTM receiver in test file\n");
    }
    struct EVTMRecvSetup evtm_recv_setup = {
        .report = (evtm_type == LOS_EVTM) ? &evtm_los_report : &evtm_tdrss_report,
        .udpsetup = udpsetup,
        .udprecver = {0},
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
 * @return status code
 */
static int GH_EVTM_setup_unit_tests(void **state) {
    channels_initialize(channel_list);

    struct UDPSetup evtm_los_setup = {"EVTM_LOS", EVTM_ADDR_LOS, EVTM_PORT_LOS, \
                                    EVTM_MAX_SIZE, EVTM_MAX_PACKET_SIZE, LOS_EVTM};
    struct UDPSetup evtm_tdrss_setup = {"EVTM_TDRSS", EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, \
                                    EVTM_MAX_SIZE, EVTM_MAX_PACKET_SIZE, TDRSS_EVTM};

    *state = calloc(1, sizeof(struct UDPSetup));
    *(state+1) = calloc(1, sizeof(struct UDPSetup));
    memcpy(*state, &evtm_los_setup, sizeof(struct UDPSetup));
    memcpy(*(state+1), &evtm_tdrss_setup, sizeof(struct UDPSetup));
    return 0;
}

// ----------------------------------------------------------------------
// Testing Functions
// ----------------------------------------------------------------------

/**
 * @brief helper test that the EVTM_setup_receiver function sets up the EVTMRecvSetup struct correctly
 *          given the structs corresponding to either EVTM downlink type
 */
void test_GH_EVTM_setup_receiver_one_evtm_type(void **state, struct TlmReport *report, int evtm_type) {
    struct UDPSetup *udpsetup = (struct UDPSetup *) ((evtm_type == LOS_EVTM) ? state[0] : state[1]);
    expect_string(__wrap_initBITRecver, recv_addr, udpsetup->addr);
    expect_value(__wrap_initBITRecver, port, udpsetup->port);
    expect_value(__wrap_initBITRecver, fifo_length, FIFO_LEN);
    expect_value(__wrap_initBITRecver, fifo_maxsize, udpsetup->maxsize);
    expect_value(__wrap_initBITRecver, packet_maxsize, udpsetup->packetsize);
    expect_function_calls(__wrap_initBITRecver, 1);

    printf("udpsetup: %p\n", udpsetup);
    printf("udpsetup->downlink_index: %d\n", udpsetup->downlink_index);

    struct EVTMRecvSetup es;
    EVTM_setup_receiver(udpsetup, &es);
    assert_ptr_equal(es.report, report);
    assert_ptr_equal(es.udpsetup, udpsetup);
    assert_null(es.recvbuffer);
    assert_int_equal(es.serial, 0);
    assert_int_equal(es.prev_serial, 0);
    assert_null(es.ll);
    assert_int_equal(es.blk_size, 0);
    assert_int_equal(es.recv_size, 0);
    assert_int_equal(es.transmit_size, 0);
    assert_int_equal(es.framenum, 0);
    assert_int_equal(es.af, 0);
    assert_non_null(es.local_allframe);
    assert_null(es.ll_rawfile);
    assert_non_null(es.compbuffer);
    assert_int_equal(es.bad_serial_count, 0);
}

/**
 * @brief Test that the EVTM_setup_receiver function sets up the EVTMRecvSetup struct correctly
 */
void test_GH_EVTM_setup_receiver(void **state) {
    test_GH_EVTM_setup_receiver_one_evtm_type(state, &evtm_los_report, LOS_EVTM);
    test_GH_EVTM_setup_receiver_one_evtm_type(state, &evtm_tdrss_report, TDRSS_EVTM);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_GH_EVTM_setup_receiver, GH_EVTM_setup_unit_tests, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
