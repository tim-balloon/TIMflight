/**
 * @file test_evtm.c
 *
 * @date September 21, 2023
 * @author Shubh Agrawal
 *
 * @brief This file is part of MCP, created for the Terahertz Intensity Mapper (TIM) project.
 *
 * This software is copyright (C) 2023 University of Pennsylvania.
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include <float.h>
#include <stdlib.h>
#include <mcp.h>

#include "evtm.c"
#include "mcp_mock_decl.c"
#include "blast.h"
#include "channels_tng.h"
#include "FIFO.h"

// if compiler is invoked with --wrap=<func>, then the linker will resolve
// <func> to __wrap_<func> instead of <func>.
// this allows us to mock BITServer behavior.

// function_called allows us to check how many times that function was called
// unfortunately, it does not let us check if that count is 0 :(

void __wrap_bprintf(buos_t l, const char *fmt, ...) {
    char message[BUOS_MAX];
    va_list argptr;

    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);

    bputs(info, message);

    if (l == fatal) {
        check_expected(fmt);
        function_called();
    }
}

int __wrap_initBITSender(struct BITSender *server, const char *send_addr,
  unsigned int port, unsigned int fifo_length, unsigned int fifo_maxsize, unsigned int packet_maxsize) {
    assert_non_null(server);
    assert_int_equal(fifo_length, FIFO_LEN);
    assert_int_equal(fifo_maxsize, MAX(EVTM_MAX_SIZE, superframe->allframe_size*2));
    assert_int_equal(packet_maxsize, EVTM_MAX_PACKET_SIZE);

    check_expected(send_addr);
    check_expected(port);
    function_called();
    return mock_type(int);
}

void __wrap_setBITSenderSerial(struct BITSender *sender, uint32_t serial) {
    function_called();
    assert_non_null(sender);
    check_expected(serial);
}

int __wrap_setBITSenderFramenum(struct BITSender *sender, uint32_t framenum) {
    function_called();
    assert_non_null(sender);
    check_expected(framenum);
    return mock_type(int);
}

int __wrap_sendToBITSender(struct BITSender *sender, uint8_t *data, unsigned int size, uint8_t priority) {
    function_called();
    assert_non_null(sender);
    check_expected(data);
    check_expected(size);
    check_expected(priority);
    return mock_type(int);
}

int __wrap_testing_evtm() {
    return 1; // we are testing EVTM, this will result in the infinite loop not running
}

/**
 * @brief gives a evtmSetup struct with the given parameters, similar to how the evtm.c function would
 */
struct evtmSetup get_evtm_setup_variables(int evtm_type, char *addr, int port, \
                                int telemetry_index, struct Fifo *evtm_fifo) {
    struct evtmSetup evtm_setup = {
        .telemetries = telemetries_linklist,
        .evtm_type = evtm_type,
        .PORT = port,
        .ADDR = addr,
        .TELEMETRY_INDEX = telemetry_index,
        .evtm_fifo = evtm_fifo,
        .BANDWIDTH = 0,
        .ALLFRAME_FRACTION = 0,
        .evtm_sender = {0},
        .fifosize = MAX(EVTM_MAX_SIZE, superframe->allframe_size*2),
        .ll = NULL,
        .ll_old = NULL,
        .ll_saved = NULL,
        .ll_array = telemetries_linklist,
        .compbuffer = calloc(1, MAX(EVTM_MAX_SIZE, superframe->allframe_size*2)),
        .allframe_bytes = 0,
        .bandwidth = 0,
        .transmit_size = 0
    };
    return evtm_setup;
}

/**
 * @brief Setup function to initialize the telemetries linklist, like in mcp.c
 */
static int setup_EVTM(void **state) {
    // need to initialize the command data to set the correct bandwidth and allframe fraction
    InitCommandData();

    // initialize the telemetries linklist
    // linklist_t are initialized in parse_linklist_format_opt
    channels_initialize(channel_list);
    load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, linklist_array, 0);

    // TODO(shubh): currently all linklists are set to the pilot/all-tm linklist for testing purposes.
    // THIS NEEDS TO BE CHANGED: ALL_TELEMETRY_NAME -> CommandData.XXXX_linklist_name
    telemetries_linklist[EVTM_LOS_TELEMETRY_INDEX] =
        linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);
    telemetries_linklist[EVTM_TDRSS_TELEMETRY_INDEX] =
        linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

    // initialize the evtm fifos
    allocFifo(&evtm_fifo_los, 3, superframe->size);
    allocFifo(&evtm_fifo_tdrss, 3, superframe->size);

    return 0;
}


/**
 * @brief test that setup_EVTM_config works for given EVTM telemetry type
 */
void test_setup_EVTM_config(void **state, int evtm_type, char *addr, int port, \
                                int telemetry_index, struct Fifo *evtm_fifo) {
    struct evtmSetup evtm_setup = {{0}};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = evtm_type};

    expect_string(__wrap_initBITSender, send_addr, addr);
    expect_value(__wrap_initBITSender, port, port);
    will_return(__wrap_initBITSender, 1);
    expect_function_calls(__wrap_initBITSender, 1);

    assert_int_equal(setup_EVTM_config(&evtm_info, &evtm_setup), 0);

    assert_non_null(&evtm_setup);
    assert_int_equal(evtm_setup.evtm_type, evtm_type);
    assert_int_equal(evtm_setup.PORT, port);
    assert_ptr_equal(evtm_setup.telemetries, telemetries_linklist);
    assert_string_equal(evtm_setup.ADDR, addr);
    assert_int_equal(evtm_setup.TELEMETRY_INDEX, telemetry_index);
    assert_int_equal(evtm_setup.fifosize, MAX(EVTM_MAX_SIZE, superframe->allframe_size*2));
    assert_ptr_equal(evtm_setup.evtm_fifo, evtm_fifo);
    assert_int_equal(evtm_setup.bandwidth, 0);
    assert_int_equal(evtm_setup.transmit_size, 0);
    assert_null(evtm_setup.ll);
    assert_null(evtm_setup.ll_old);
    assert_null(evtm_setup.ll_saved);
    assert_ptr_equal(evtm_setup.ll_array, telemetries_linklist);
    assert_non_null(evtm_setup.compbuffer);
    assert_int_equal(evtm_setup.allframe_bytes, 0);
}

/**
 * @brief test that setup_EVTM_config works for Line of Sight (LOS) telemetry
 */
void test_setup_EVTM_config_LOS(void **state) {
    test_setup_EVTM_config(state, EVTM_LOS, EVTM_ADDR_LOS, EVTM_PORT_LOS, EVTM_LOS_TELEMETRY_INDEX, \
                                &evtm_fifo_los);
}

/**
 * @brief test that setup_EVTM_config works for Tracking and Data Relay Satellite System (TDRSS) telemetry
 */
void test_setup_EVTM_config_TDRSS(void **state) {
    test_setup_EVTM_config(state, EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, EVTM_TDRSS_TELEMETRY_INDEX,
                                &evtm_fifo_tdrss);
}

/**
 * @brief test that setup_EVTM_config fails gracefully for invalid telemetry type
 */
void test_setup_EVTM_config_fails(void **state) {
    struct evtmSetup evtm_setup = {{0}};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = 1511};
    expect_value(__wrap_bprintf, fmt, "%s:%d (%s):Invalid evtm type %d");
    expect_function_calls(__wrap_bprintf, 1);
    assert_int_equal(setup_EVTM_config(&evtm_info, &evtm_setup), -1);
}

/**
 * @brief test that setup_EVTM_config fails gracefully for invalid BITSender initialization
 */
void test_setup_EVTM_config_fails_initBITSender(void **state) {
    expect_string(__wrap_initBITSender, send_addr, EVTM_ADDR_LOS);
    expect_value(__wrap_initBITSender, port, EVTM_PORT_LOS);
    will_return(__wrap_initBITSender, -1);
    expect_function_calls(__wrap_initBITSender, 1);
    expect_function_calls(__wrap_bprintf, 1);

    struct evtmSetup evtm_setup = {0};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = EVTM_LOS};
    expect_value(__wrap_bprintf, fmt, \
        "%s:%d (%s):initializing BITSender did not work for EVTM %s: check above error msg");
    assert_int_equal(setup_EVTM_config(&evtm_info, &evtm_setup), -1);
}

/**
 * @brief test that infinite_loop_EVTM works for given EVTM telemetry type
 */
void test_infinite_loop_EVTM(void **state, struct evtmSetup evtm_setup, int evtm_type, char *expected_telemetry_name, \
                                uint32_t expected_transmit_size, unsigned int expected_allframe_bytes) {
    if (expected_transmit_size > 0) {
        // none of the BITSender functions should be called if transmit size is non-positive
        expect_function_calls(__wrap_setBITSenderSerial, 1);
        expect_function_calls(__wrap_setBITSenderFramenum, 1);
        expect_function_calls(__wrap_sendToBITSender, 1);
        // assert that the loop is run exactly once

        expect_value(__wrap_setBITSenderSerial, serial, \
                        *(uint32_t *) linklist_find_by_name(expected_telemetry_name, linklist_array)->serial);
        expect_value(__wrap_setBITSenderFramenum, framenum, expected_transmit_size);
        expect_value(__wrap_sendToBITSender, data, evtm_setup.compbuffer);
        expect_value(__wrap_sendToBITSender, size, expected_transmit_size);
        expect_value(__wrap_sendToBITSender, priority, 0);
        will_return(__wrap_sendToBITSender, 0);
        will_return(__wrap_setBITSenderFramenum, 0);
    }

    // write one element to the fifo
    uint8_t * master_superframe_buffer = calloc(1, superframe->size);
    memcpy(getFifoWrite(evtm_setup.evtm_fifo), master_superframe_buffer, superframe->size);
    incrementFifo(evtm_setup.evtm_fifo);

    infinite_loop_EVTM(&evtm_setup);

    assert_int_equal(evtm_setup.allframe_bytes, expected_allframe_bytes);
}

/**
 * @brief test that infinite_loop_EVTM works for either telemetry at nominal transmit size
 */
void test_infinite_loop_EVTM_nominal(void **state) {
    linklist_t * ll = linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

    double bandwidth = CommandData.biphase_bw;
    double ALLFRAME_FRACTION = CommandData.biphase_allframe_fraction;
    uint32_t expected_transmit_size = MIN(ll->blk_size, bandwidth * (1.0 - ALLFRAME_FRACTION));
    unsigned int expected_allframe_bytes = bandwidth * ALLFRAME_FRACTION;
    struct evtmSetup evtm_setup_LOS = get_evtm_setup_variables(EVTM_LOS, EVTM_ADDR_LOS, EVTM_PORT_LOS, \
                                EVTM_LOS_TELEMETRY_INDEX, &evtm_fifo_los);
    test_infinite_loop_EVTM(state, evtm_setup_LOS, EVTM_LOS, ALL_TELEMETRY_NAME, \
                                expected_transmit_size, expected_allframe_bytes);

    bandwidth = CommandData.highrate_bw;
    ALLFRAME_FRACTION = CommandData.highrate_allframe_fraction;
    expected_transmit_size = MIN(ll->blk_size, bandwidth * (1.0 - ALLFRAME_FRACTION));
    expected_allframe_bytes = bandwidth * ALLFRAME_FRACTION;
    struct evtmSetup evtm_setup_TDRSS = get_evtm_setup_variables(EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, \
                                EVTM_TDRSS_TELEMETRY_INDEX, &evtm_fifo_tdrss);
    test_infinite_loop_EVTM(state, evtm_setup_TDRSS, EVTM_TDRSS, ALL_TELEMETRY_NAME, \
                                expected_transmit_size, expected_allframe_bytes);
}

/**
 * @brief test that infinite_loop_EVTM works for either telemetry at allframe transmit size
 */
void test_infinite_loop_EVTM_allframe(void **state) {
    linklist_t * ll = linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

    double bandwidth = CommandData.biphase_bw;
    double ALLFRAME_FRACTION = CommandData.biphase_allframe_fraction;
    uint32_t expected_transmit_size = MIN(ll->blk_size, bandwidth * (1.0 - ALLFRAME_FRACTION));
    unsigned int expected_allframe_bytes = bandwidth * ALLFRAME_FRACTION;
    struct evtmSetup evtm_setup_LOS = get_evtm_setup_variables(EVTM_LOS, EVTM_ADDR_LOS, EVTM_PORT_LOS, \
                                EVTM_LOS_TELEMETRY_INDEX, &evtm_fifo_los);
    evtm_setup_LOS.allframe_bytes = superframe->allframe_size; // to trigger the allframe send
    test_infinite_loop_EVTM(state, evtm_setup_LOS, EVTM_LOS, ALL_TELEMETRY_NAME, \
                                expected_transmit_size, expected_allframe_bytes);

    bandwidth = CommandData.highrate_bw;
    ALLFRAME_FRACTION = CommandData.highrate_allframe_fraction;
    expected_transmit_size = MIN(ll->blk_size, bandwidth * (1.0 - ALLFRAME_FRACTION));
    expected_allframe_bytes = bandwidth * ALLFRAME_FRACTION;
    struct evtmSetup evtm_setup_TDRSS = get_evtm_setup_variables(EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, \
                                EVTM_TDRSS_TELEMETRY_INDEX, &evtm_fifo_tdrss);
    evtm_setup_TDRSS.allframe_bytes = superframe->allframe_size; // to trigger the allframe send
    test_infinite_loop_EVTM(state, evtm_setup_TDRSS, EVTM_TDRSS, ALL_TELEMETRY_NAME, \
                                expected_transmit_size, expected_allframe_bytes);
}

/**
 * @brief test that none of the BITSender functions are called when transmit size is set to 0.
 */
void test_infinite_loop_EVTM_transmit_size_zero(void **state) {
    // note that we will get an error if any of the BITSender functions are called
    // due to the lack of expect_value() calls

    // force the transmit size to be zero by setting bandwidth to 0
    double bandwidth = CommandData.biphase_bw;
    CommandData.biphase_bw = 0;
    struct evtmSetup evtm_setup_LOS = get_evtm_setup_variables(EVTM_LOS, EVTM_ADDR_LOS, EVTM_PORT_LOS, \
                                EVTM_LOS_TELEMETRY_INDEX, &evtm_fifo_los);
    test_infinite_loop_EVTM(state, evtm_setup_LOS, EVTM_LOS, ALL_TELEMETRY_NAME, \
                                0, 0); // allframe bytes should not be incremented
    CommandData.biphase_bw = bandwidth; // restore

    bandwidth = CommandData.highrate_bw;
    CommandData.highrate_bw = 0;
    struct evtmSetup evtm_setup_TDRSS = get_evtm_setup_variables(EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, \
                                EVTM_TDRSS_TELEMETRY_INDEX, &evtm_fifo_tdrss);
    test_infinite_loop_EVTM(state, evtm_setup_TDRSS, EVTM_TDRSS, ALL_TELEMETRY_NAME, \
                                0, 0); // allframe bytes should not be incremented
    CommandData.highrate_bw = bandwidth; // restore
}


/**
 * @brief test that the infinite loop works when the linklist is the FILE_LINKLIST
 */
void test_infinite_loop_EVTM_filelinklist(void **state) {
    linklist_t * ll = linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array);

    memset(ll->name, 0, LINKLIST_SHORT_FILENAME_SIZE);
    strncpy(&ll->name, &FILE_LINKLIST, strlen(FILE_LINKLIST));

    // initialize the blocks array
    ll->blocks = (struct block_container *) calloc(1, sizeof(struct block_container));
    ll->blocks[0] = (struct block_container) {0};
    ll->blocks[0].n = 1;
    ll->blocks[0].i = 0;
    ll->num_blocks = 1;

    double bandwidth = CommandData.biphase_bw;
    struct evtmSetup evtm_setup_LOS = get_evtm_setup_variables(EVTM_LOS, EVTM_ADDR_LOS, EVTM_PORT_LOS, \
                                EVTM_LOS_TELEMETRY_INDEX, &evtm_fifo_los);
    test_infinite_loop_EVTM(state, evtm_setup_LOS, EVTM_LOS, FILE_LINKLIST, bandwidth, 0);

    bandwidth = CommandData.highrate_bw;
    struct evtmSetup evtm_setup_TDRSS = get_evtm_setup_variables(EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, \
                                EVTM_TDRSS_TELEMETRY_INDEX, &evtm_fifo_tdrss);
    test_infinite_loop_EVTM(state, evtm_setup_TDRSS, EVTM_TDRSS, FILE_LINKLIST, bandwidth, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_LOS, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_TDRSS, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_fails, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_fails_initBITSender, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_infinite_loop_EVTM_nominal, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_infinite_loop_EVTM_allframe, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_infinite_loop_EVTM_transmit_size_zero, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_infinite_loop_EVTM_filelinklist, setup_EVTM, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
