/**
 * @file test_gh_evtm.c
 * @author Shubh Agrawal (shubh@sas.upenn.edu)
 * @brief This file is part of GroundHog, created for the Terahertz Intensity Mapper (TIM) project.
 * @date 2023-09-26
 * 
 * Copyright (c) 2023 University of Pennsylvania
 * 
 */

#define SERIAL_TEST_VAL 3273884366
#define INCORRECT_SERIAL 1511
#define LOOP_TEST_VAL 121
#define LOOP_TEST_VAL_FILELIST 1511

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
#include "groundhog_funcs.h"
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
    return mock_type(uint8_t *);
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


int __wrap_groundhog_unpack_fileblocks(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags) {
    check_expected(ll);
    check_expected(transmit_size);
    check_expected(compbuffer);
    check_expected(local_allframe);
    check_expected(filename_str);
    check_expected(disp_str);
    check_expected(ll_rawfile);
    check_expected(flags);
    function_called();
    return mock_type(int);
}


int64_t __wrap_groundhog_process_and_write(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags) {
    check_expected(ll);
    check_expected(transmit_size);
    check_expected(compbuffer);
    check_expected(local_allframe);
    check_expected(filename_str);
    check_expected(disp_str);
    check_expected(ll_rawfile);
    check_expected(flags);
    function_called();
    return mock_type(int64_t);
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
static int GH_EVTM_start_tests(void **state) {
    channels_initialize(channel_list);
    linklist_t ** ll_list = calloc(MAX_NUM_LINKLIST_FILES, sizeof(linklist_t *));
    load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, ll_list, LL_INCLUDE_ALLFRAME);
    linklist_generate_lookup(ll_list);

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


/**
 * @brief teardown configuration for testing EVTM groundhog code
 * 
 * @return status code
 */
static int GH_EVTM_teardown_tests(void **state) {
    free(*state);
    free(*(state+1));
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
 * @brief Test that the EVTM_setup_receiver function sets up the EVTMRecvSetup struct correctly given the structs
 *        corresponding to the Line of Sight (LOS) EVTM downlink type
 */
void test_GH_EVTM_setup_receiver_LOS(void **state) {
    test_GH_EVTM_setup_receiver_one_evtm_type(state, &evtm_los_report, LOS_EVTM);
}


/**
 * @brief Test that the EVTM_setup_receiver function sets up the EVTMRecvSetup struct correctly given the structs
 *        corresponding to the Tracking and Data Relay Satellite System (TDRSS) EVTM downlink type
 */
void test_GH_EVTM_setup_receiver_TDRSS(void **state) {
    test_GH_EVTM_setup_receiver_one_evtm_type(state, &evtm_tdrss_report, TDRSS_EVTM);
}


// -----------------------------------------------------------------------------


/**
 * @brief helper test that the EVTM_receiver_get_linklist function gets the correct linklist
 */
void test_GH_EVTM_receiver_get_linklist(void **state) {
    struct UDPSetup *udpsetup = (struct UDPSetup *) state[0];
    struct EVTMRecvSetup es = get_evtm_recv_struct(LOS_EVTM, udpsetup);
    uint32_t serial = SERIAL_TEST_VAL;
    linklist_t *ll = linklist_lookup_by_serial(serial);


    expect_value(__wrap_getBITRecverAddr, size, &es.recv_size);
    expect_function_calls(__wrap_getBITRecverAddr, 1);
    will_return(__wrap_getBITRecverAddr, &serial);
    // __wrap_removeBITRecverAddr should not be called

    assert_int_equal(EVTM_receiver_get_linklist(&es), 0);
    assert_ptr_equal(es.ll, ll);
    assert_int_equal(es.bad_serial_count, 0);
    assert_int_equal(es.serial, serial);
}


/**
 * @brief Test that the EVTM_receiver_get_linklist func fails when given an incorrect serial
 */
void test_GH_EVTM_receiver_get_linklist_fails(void **state) {
    struct UDPSetup *udpsetup = (struct UDPSetup *) state[0];
    struct EVTMRecvSetup es = get_evtm_recv_struct(LOS_EVTM, udpsetup);
    uint32_t serial = INCORRECT_SERIAL;
    linklist_t *ll = linklist_lookup_by_serial(serial);

    expect_value(__wrap_getBITRecverAddr, size, &es.recv_size);
    expect_function_calls(__wrap_getBITRecverAddr, 1);
    will_return(__wrap_getBITRecverAddr, &serial);
    expect_function_calls(__wrap_removeBITRecverAddr, 1);

    assert_int_equal(EVTM_receiver_get_linklist(&es), 1);
    assert_null(es.ll);
    assert_int_equal(es.bad_serial_count, 1);
    assert_int_equal(es.serial, serial);
}


/**
 * @brief test EVTM_receiver_loop_body when the linklist is not filelist, 
 *        for a given EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_one_evtm_type(void **state, int evtm_type) {
    struct UDPSetup *udpsetup = (struct UDPSetup *) state[evtm_type - LOS_EVTM];
    struct EVTMRecvSetup es = get_evtm_recv_struct(evtm_type, udpsetup);
    es.ll = linklist_lookup_by_serial(SERIAL_TEST_VAL);
    es.compbuffer = calloc(1, udpsetup->maxsize);

    expect_value(__wrap_setBITRecverSerial, serial, 0);
    expect_function_calls(__wrap_setBITRecverSerial, 1);
    expect_value(__wrap_recvFromBITRecver, buffer, es.compbuffer);
    expect_value(__wrap_recvFromBITRecver, size, EVTM_MAX_SIZE);
    expect_value(__wrap_recvFromBITRecver, flags, 0);
    expect_function_calls(__wrap_recvFromBITRecver, 1);
    will_return(__wrap_recvFromBITRecver, LOOP_TEST_VAL);

    expect_value(__wrap_groundhog_process_and_write, ll, es.ll);
    expect_value(__wrap_groundhog_process_and_write, transmit_size, 0);
    expect_value(__wrap_groundhog_process_and_write, compbuffer, es.compbuffer);
    expect_value(__wrap_groundhog_process_and_write, local_allframe, 0);
    expect_string(__wrap_groundhog_process_and_write, filename_str, udpsetup->name);
    expect_string(__wrap_groundhog_process_and_write, disp_str, udpsetup->name);
    expect_value(__wrap_groundhog_process_and_write, ll_rawfile, &es.ll_rawfile);
    expect_value(__wrap_groundhog_process_and_write, flags, 0);
    expect_function_calls(__wrap_groundhog_process_and_write, 1);
    will_return(__wrap_groundhog_process_and_write, LOOP_TEST_VAL);

    EVTM_receiver_loop_body(&es);
    assert_int_equal(es.serial, 0);
    assert_int_equal(es.prev_serial, 0);
    assert_int_equal(es.blk_size, LOOP_TEST_VAL);
    assert_int_equal(es.framenum, LOOP_TEST_VAL);
    assert_ptr_equal(es.report->ll, es.ll);
    assert_int_equal(es.report->allframe, 0);
    assert_int_equal(es.bad_serial_count, 0);
    free(es.compbuffer);
}


/**
 * @brief test EVTM_receiver_loop_body when the linklist is not filelist, 
 *        for the Line of Sight (LOS) EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_LOS(void **state) {
    test_GH_EVTM_receiver_loop_body_one_evtm_type(state, LOS_EVTM);
}


/**
 * @brief test EVTM_receiver_loop_body when the linklist is not filelist, 
 *        for the Tracking and Data Relay Satellite System (TDRSS) EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_TDRSS(void **state) {
    test_GH_EVTM_receiver_loop_body_one_evtm_type(state, TDRSS_EVTM);
}


// -----------------------------------------------------------------------------


/**
 * @brief test EVTM_receiver_loop_body when the linklist is filelist
 *        for a given EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_filelist_one_evtm_type(void **state, int evtm_type) {
    struct UDPSetup *udpsetup = (struct UDPSetup *) state[evtm_type - LOS_EVTM];
    struct EVTMRecvSetup es = get_evtm_recv_struct(evtm_type, udpsetup);
    es.ll = linklist_lookup_by_serial(SERIAL_TEST_VAL);
    es.compbuffer = calloc(1, udpsetup->maxsize);
    // force the control to go to the remaining part of code
    memcpy(es.ll->name, FILE_LINKLIST, strlen(FILE_LINKLIST) + 1);

    expect_value(__wrap_setBITRecverSerial, serial, 0);
    expect_function_calls(__wrap_setBITRecverSerial, 1);
    expect_value(__wrap_recvFromBITRecver, buffer, es.compbuffer);
    expect_value(__wrap_recvFromBITRecver, size, EVTM_MAX_SIZE);
    expect_value(__wrap_recvFromBITRecver, flags, 0);
    expect_function_calls(__wrap_recvFromBITRecver, 1);
    will_return(__wrap_recvFromBITRecver, LOOP_TEST_VAL_FILELIST);

    expect_value(__wrap_groundhog_unpack_fileblocks, ll, es.ll);
    expect_value(__wrap_groundhog_unpack_fileblocks, transmit_size, 0);
    expect_value(__wrap_groundhog_unpack_fileblocks, compbuffer, es.compbuffer);
    expect_value(__wrap_groundhog_unpack_fileblocks, local_allframe, 0);
    expect_value(__wrap_groundhog_unpack_fileblocks, filename_str, NULL);
    expect_value(__wrap_groundhog_unpack_fileblocks, disp_str, NULL);
    expect_value(__wrap_groundhog_unpack_fileblocks, ll_rawfile, NULL);
    expect_value(__wrap_groundhog_unpack_fileblocks, flags, GROUNDHOG_EXTRACT_TO_DISK);
    will_return(__wrap_groundhog_unpack_fileblocks, LOOP_TEST_VAL_FILELIST);

    expect_function_calls(__wrap_groundhog_unpack_fileblocks, 1);

    EVTM_receiver_loop_body(&es);

    assert_int_equal(es.serial, 0);
    assert_int_equal(es.prev_serial, 0);
    assert_int_equal(es.blk_size, LOOP_TEST_VAL_FILELIST);
    assert_int_equal(es.framenum, LOOP_TEST_VAL_FILELIST);
    assert_ptr_equal(es.report->ll, es.ll);
    assert_int_equal(es.report->allframe, 0);
    assert_int_equal(es.bad_serial_count, 0);
    free(es.compbuffer);
}


/**
 * @brief test EVTM_receiver_loop_body when the linklist is filelist
 *        for the Line of Sight (LOS) EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_filelist_LOS(void **state) {
    test_GH_EVTM_receiver_loop_body_filelist_one_evtm_type(state, LOS_EVTM);
}


/**
 * @brief test EVTM_receiver_loop_body when the linklist is filelist
 *        for the Tracking and Data Relay Satellite System (TDRSS) EVTM downlink type
 */
void test_GH_EVTM_receiver_loop_body_filelist_TDRSS(void **state) {
    test_GH_EVTM_receiver_loop_body_filelist_one_evtm_type(state, TDRSS_EVTM);
}


// -----------------------------------------------------------------------------


/**
 * @brief test EVTM_receiver_loop_body fails gracefully
 */
void test_GH_EVTM_receiver_loop_body_fails(void **state) {
    for (int evtm_type = LOS_EVTM; evtm_type <= TDRSS_EVTM; evtm_type++) {
        struct UDPSetup *udpsetup = (struct UDPSetup *) state[evtm_type - LOS_EVTM];
        struct EVTMRecvSetup es = get_evtm_recv_struct(evtm_type, udpsetup);
        expect_value(__wrap_setBITRecverSerial, serial, 0);
        expect_function_calls(__wrap_setBITRecverSerial, 1);
        expect_value(__wrap_recvFromBITRecver, buffer, NULL);
        expect_value(__wrap_recvFromBITRecver, size, EVTM_MAX_SIZE);
        expect_value(__wrap_recvFromBITRecver, flags, 0);
        expect_function_calls(__wrap_recvFromBITRecver, 1);
        will_return(__wrap_recvFromBITRecver, -1);

        EVTM_receiver_loop_body(&es);
        assert_int_equal(es.serial, 0);
        assert_int_equal(es.prev_serial, 0);
        assert_int_equal(es.blk_size, -1);
        assert_int_equal(es.framenum, 0);
        assert_null(es.ll);
    }
}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_GH_EVTM_setup_receiver_LOS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_setup_receiver_TDRSS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_get_linklist, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_get_linklist_fails, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_loop_body_LOS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_loop_body_TDRSS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_loop_body_filelist_LOS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_loop_body_filelist_TDRSS, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
        cmocka_unit_test_setup_teardown(test_GH_EVTM_receiver_loop_body_fails, \
                                            GH_EVTM_start_tests, GH_EVTM_teardown_tests),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
