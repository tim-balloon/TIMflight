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

// if compiler is invoked with --wrap=<func>, then the linker will resolve
// <func> to __wrap_<func> instead of <func>.
// this allows us to mock BITServer behavior.

int __wrap_initBITSender(struct BITServer *server, const char *addr, unsigned int port, \
    unsigned int max_packet_size, unsigned int max_size, unsigned int max_queue_size) {
    check_expected(server);
    check_expected(addr);
    check_expected(port);
    check_expected(max_packet_size);
    check_expected(max_size);
    check_expected(max_queue_size);
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
    return 1 // we are testing EVTM, this will result in the infinite loop not running
}

/**
 * @brief Setup function to initialize the telemetries linklist, like in mcp.c
 */
static int setup_EVTM(void **state) {
    // initialize the telemetries linklist
    // load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, linklist_array, 0);
    // generate_housekeeping_linklist(linklist_find_by_name(ALL_TELEMETRY_NAME, linklist_array), ALL_TELEMETRY_NAME);
    // linklist_generate_lookup(linklist_array);

    // // TODO(shubh): currently all linklists are set to the pilot linklist for
    // // testing purposes.
    // // THIS NEEDS TO BE CHANGED: CommandData.pilot_linklist_name -> CommandData.XXXX_linklist_name

    // // load the latest linklist into telemetry
    // telemetries_linklist[PILOT_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    // telemetries_linklist[BI0_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    // telemetries_linklist[HIGHRATE_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    // telemetries_linklist[SBD_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    // telemetries_linklist[EVTM_LOS_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);
    // telemetries_linklist[EVTM_TDRSS_TELEMETRY_INDEX] =
    //     linklist_find_by_name(CommandData.pilot_linklist_name, linklist_array);

    return 0;
}


/**
 * @brief test that BITSender gets the correct inputs for Line of Sight EVTM
 */
void test_evtm_los(void **state) {
    // struct evtmInfo evtm_info = {.telemetries = telemetries_linklist, .evtm_type = EVTM_LOS};
    // expect_value(__wrap_initBITServer, server, NULL);
    // expect_string(__wrap_initBITServer, addr, EVTM_ADDR_LOS);
    // expect_value(__wrap_initBITServer, port, EVTM_PORT_LOS);
    // expect_value(__wrap_initBITServer, max_packet_size, EVTM_MAX_PACKET_SIZE);
    // expect_value(__wrap_initBITServer, max_size, MAX(EVTM_MAX_SIZE, superframe->allframe_size*2));
    // expect_value(__wrap_initBITServer, max_queue_size, FIFO_LEN);
    // will_return(__wrap_initBITServer, 1);

    // expect_value(__wrap_setBITSenderSerial, sender, NULL);
    // expect_value(__wrap_setBITSenderSerial, serial, \
    //     *(uint32_t *) telemetries_linklist[EVTM_LOS_TELEMETRY_INDEX]->serial);

    // expect_value(__wrap_setBITSenderFramenum, sender, NULL);
    // expect_value(__wrap_setBITSenderFramenum, framenum, 0);
}


int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_evtm_los, setup_EVTM),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
