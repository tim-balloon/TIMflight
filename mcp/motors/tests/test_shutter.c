/**
 * @file test_shutter.c
 *
 * @date Jan 30, 2023
 * @author evanmayer
 *
 * @brief This file is part of MCP, created for the TIMballoon project
 *
 * This software is copyright (C) 2023 University of Arizona
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

#include "actuators.c"

#include "mcp_mock_decl.c"


// ============================================================================
// Mock objects
// ============================================================================
// Mock objects named __wrap_<funcName> are bound to the symbol <funcName> when
// the linker is invoked with --wrap=<funcName>, replacing it in any
// compilation units this file is linked to. The real func is available at
// __real_<funcName>.

int __wrap_EZBus_SetIHold(struct ezbus* bus, char who, int current);
int __wrap_EZBus_SetIHold(struct ezbus* bus, char who, int current)
{
    check_expected(who);
    check_expected_ptr(current);

    return mock_type(int);
}

// ============================================================================
// Setup/teardown functions (test fixtures)
// ============================================================================


// ============================================================================
// Test functions
// ============================================================================
/**
 * @brief Test successful turn-off of shutter hold current
 */
static void test_TurnOffShutterSuccess(void **state)
{
    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK); // retval

    CommandData.actbus.shutter_hold_i = 1;
    TurnOffShutter();
    assert_int_equal(CommandData.actbus.shutter_hold_i, 0);
}

/**
 * @brief Test failure to turn off shutter hold current
 */
static void test_TurnOffShutterFail(void **state)
{
    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_SERR_BUSY); // retval

    CommandData.actbus.shutter_hold_i = 1;
    TurnOffShutter();
    assert_int_equal(CommandData.actbus.shutter_hold_i, 1);
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_TurnOffShutterSuccess),
        cmocka_unit_test(test_TurnOffShutterFail),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
