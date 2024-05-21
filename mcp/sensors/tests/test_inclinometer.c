/**
 * @file test_inclinometer.c
 *
 * @date May 20, 2024
 * @author evanmayer
 *
 * @brief This file is part of MCP, created for the TIMballoon project
 *
 * This software is copyright (C) 2024 University of Arizona
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

#include "inclinometer.c"
#include "mcp_mock_decl.c"


#define NUM_TEST_BUFS 12U
#define NUM_PAYLOAD_VALS 3U

char test_inc_bufs[NUM_TEST_BUFS][EXPECTED_MSG_LEN] =
{
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x07, 0x18, 0x6E},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x70, 0x03, 0x06, 0x98, 0xEC},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x70, 0x03, 0x07, 0x21, 0x76},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x06, 0x90, 0xE5},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x06, 0x70, 0xC5},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x07, 0x21, 0x77},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x07, 0x26, 0x7C},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x06, 0x92, 0xE7},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x07, 0x07, 0x5D},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x24, 0x10, 0x00, 0x71, 0x03, 0x06, 0x94, 0xE9},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x07, 0x16, 0x10, 0x00, 0x67, 0x03, 0x12, 0x41, 0x8B},
    {0x68, 0x0D, 0x00, 0x84, 0x10, 0x06, 0x98, 0x10, 0x00, 0x56, 0x03, 0x13, 0x47, 0x02}
};

float test_inc_floats[NUM_TEST_BUFS][NUM_PAYLOAD_VALS] =
{
    {-0.624, -0.071, 30.718},
    {-0.624, -0.070, 30.698},
    {-0.624, -0.070, 30.721},
    {-0.624, -0.071, 30.690},
    {-0.624, -0.071, 30.670},
    {-0.624, -0.071, 30.721},
    {-0.624, -0.071, 30.726},
    {-0.624, -0.071, 30.692},
    {-0.624, -0.071, 30.707},
    {-0.624, -0.071, 30.694},
    {-0.716, -0.067, 31.241},
    {-0.698, -0.056, 31.347}
};

// ============================================================================
// Setup/teardown functions (test fixtures)
// ============================================================================

// ============================================================================
// Test functions
// ============================================================================
/**
 * @brief Test extracting message length from message
 */
void test_inc_get_msg_len(void **state)
{
    uint8_t msg_len = inc_get_msg_len(test_inc_bufs[0]);
    assert_int_equal(0x0D, msg_len);
}

/**
 * @brief Test finding checksum index from message length
 */
void test_inc_get_msg_checksum_idx(void **state)
{
    uint8_t msg_checksum_idx = inc_get_msg_checksum_idx(test_inc_bufs[0]);
    assert_int_equal(13U, msg_checksum_idx);
}


/**
 * @brief Test checksum calculation
 */
void test_inc_calc_checksum(void **state)
{
    uint8_t checksum_idx = 0U;
    uint8_t msg_checksum = 0U;
    for (uint8_t i = 0; i < NUM_TEST_BUFS; ++i) {
        checksum_idx = inc_get_msg_checksum_idx(test_inc_bufs[i]);
        msg_checksum = inc_calc_checksum(test_inc_bufs[i]);
        assert_int_equal((uint8_t)test_inc_bufs[i][checksum_idx], msg_checksum);
    }
}


/**
 * @brief Test message payload calculation
 */
void test_inc_get_msg_value(void **state)
{
    float msg_x_deg = 0.0;
    float msg_y_deg = 0.0;
    float msg_celsius = 0.0;
    for (uint8_t i = 0; i < NUM_TEST_BUFS; ++i) {
        msg_x_deg = inc_get_msg_value(test_inc_bufs[i], MSG_X_IDX);
        msg_y_deg = inc_get_msg_value(test_inc_bufs[i], MSG_Y_IDX);
        msg_celsius = inc_get_msg_value(test_inc_bufs[i], MSG_T_IDX);
        assert_float_equal(test_inc_floats[i][0], msg_x_deg, DBL_EPSILON);
        assert_float_equal(test_inc_floats[i][1], msg_y_deg, DBL_EPSILON);
        assert_float_equal(test_inc_floats[i][2], msg_celsius, DBL_EPSILON);
    }
}


int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_inc_get_msg_len),
        cmocka_unit_test(test_inc_get_msg_checksum_idx),
        cmocka_unit_test(test_inc_calc_checksum),
        cmocka_unit_test(test_inc_get_msg_value),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
