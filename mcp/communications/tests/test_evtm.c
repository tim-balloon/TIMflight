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

// if compiler is invoked with --wrap=<func>, then the linker will resolve
// <func> to __wrap_<func> instead of <func>.
// this allows us to mock BITServer behavior.

void __wrap_bprintf(buos_t l, const char *fmt, ...) {
    char message[BUOS_MAX];
    va_list argptr;

    va_start(argptr, fmt);
    vsnprintf(message, BUOS_MAX, fmt, argptr);
    va_end(argptr);

    bputs(info, message);

    if (l == fatal) {
        check_expected(fmt);
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

    return mock_type(int);
}

void __wrap_setBITSenderSerial(struct BITSender *sender, uint32_t serial) {
    check_expected(sender);
    check_expected(serial);
}

int __wrap_setBITSenderFramenum(struct BITSender *sender, uint32_t framenum) {
    check_expected(sender);
    check_expected(framenum);
    return mock_type(int);
}

int __wrap_sendToBITSender(struct BITSender *sender, uint8_t *data, unsigned int size, uint8_t priority) {
    check_expected(sender);
    check_expected(data);
    check_expected(size);
    check_expected(priority);
    return mock_type(int);
}

int __wrap_testing_evtm() {
    return 1; // we are testing EVTM, this will result in the infinite loop not running
}

/**
 * @brief Setup function to initialize the telemetries linklist, like in mcp.c
 */
static int setup_EVTM(void **state) {
    // initialize the telemetries linklist
    // typically linklist_t are initialized in parse_linklist_format_opt

    channels_initialize(channel_list);
    load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, linklist_array, 0);

    // TODO(shubh): currently all linklists are set to the pilot linklist for testing purposes.
    // THIS NEEDS TO BE CHANGED: CommandData.pilot_linklist_name -> CommandData.XXXX_linklist_name
    telemetries_linklist[EVTM_LOS_TELEMETRY_INDEX] =
        linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    telemetries_linklist[EVTM_TDRSS_TELEMETRY_INDEX] =
        linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);

    return 0;
}


/**
 * @brief test that setup_EVTM_config works for given EVTM telemetry type
 */
void test_setup_EVTM_config(void **state, int evtm_type, char *addr, int port, \
                                int telemetry_index, struct Fifo *evtm_fifo, char *name) {
    struct evtmSetup evtm_setup = {{0}};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = evtm_type};

    expect_string(__wrap_initBITSender, send_addr, addr);
    expect_value(__wrap_initBITSender, port, port);
    will_return(__wrap_initBITSender, 1);

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
                                &evtm_fifo_los, "LOS");
}

/**
 * @brief test that setup_EVTM_config works for Tracking and Data Relay Satellite System (TDRSS) telemetry
 */
void test_setup_EVTM_config_TDRSS(void **state) {
    test_setup_EVTM_config(state, EVTM_TDRSS, EVTM_ADDR_TDRSS, EVTM_PORT_TDRSS, EVTM_TDRSS_TELEMETRY_INDEX, \
                                &evtm_fifo_tdrss, "TDRSS");
}

/**
 * @brief test that setup_EVTM_config fails gracefully for invalid telemetry type
 */
void test_setup_EVTM_config_fails(void **state) {
    struct evtmSetup evtm_setup = {{0}};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = 1511};
    expect_value(__wrap_bprintf, fmt, "%s:%d (%s):Invalid evtm type %d");
    assert_int_equal(setup_EVTM_config(&evtm_info, &evtm_setup), -1);
}

/**
 * @brief test that setup_EVTM_config fails gracefully for invalid BITSender initialization
 */
void test_setup_EVTM_config_fails_initBITSender(void **state) {
    expect_string(__wrap_initBITSender, send_addr, EVTM_ADDR_LOS);
    expect_value(__wrap_initBITSender, port, EVTM_PORT_LOS);
    will_return(__wrap_initBITSender, -1);

    struct evtmSetup evtm_setup = {0};
    struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = EVTM_LOS};
    expect_value(__wrap_bprintf, fmt, \
        "%s:%d (%s):initializing BITSender did not work for EVTM %s: check above error msg");
    assert_int_equal(setup_EVTM_config(&evtm_info, &evtm_setup), -1);
}

/**
 * @brief test that infinite_loop_EVTM works for given EVTM telemetry type
 */



int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_LOS, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_TDRSS, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_fails, setup_EVTM, NULL),
        cmocka_unit_test_setup_teardown(test_setup_EVTM_config_fails_initBITSender, setup_EVTM, NULL),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
