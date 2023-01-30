/**
 * @file test_balance.c
 *
 * @date Jan 12, 2023
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

#include "balance.c"
#include "actuators.c"

#include "mcp_mock_decl.c"


// ============================================================================
// Mock objects
// ============================================================================
// Mock objects named __wrap_<funcName> are bound to the symbol <funcName> when
// the linker is invoked with --wrap=<funcName>, replacing it in any
// compilation units this file is linked to. The real func is available at
// __real_<funcName>.

int __wrap_EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val);
int __wrap_EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val)
{
    check_expected(who);
    check_expected_ptr(what);
    *val = mock_type(int);
    return mock_type(int);
}

// ============================================================================
// Setup/teardown functions (text fixtures)
// ============================================================================
/**
 * @brief Set up the structs for balance system tests
 */
static int SetupEzBus(void **state)
{
    *state = calloc(1, sizeof(struct ezbus));
    struct ezbus bus;

    // Spoof actuator bus: open a pseudoterminal and give it to the bus init,
    // where the attributes (baud, etc.) will be set.
    int fd = getpt();
    char *ttyName;
    ttyName = ptsname(fd);
    if (-1 == unlockpt(fd)) {
        fail_msg("Failed to open pseudoterminal in SetupEzBus(): %s", strerror(errno));
    }

    int ret = EZBus_Init(&bus, ttyName, "UnitTestBus", EZ_CHAT_ACT);
    assert_int_equal(ret, EZ_ERR_OK);
    memcpy(*state, &bus, sizeof(struct ezbus));
    return 0;
}

/**
 * @brief Tear down the structs for balance system tests
 */
static int TearDownEzBus(void **state)
{
    // Note, we rely on the kernel to close pseudoterm fd on exit.
    free(*state);
    return 0;
}


// ============================================================================
// Test functions
// ============================================================================
/**
 * @brief Test balance system logic: filtering
 */
void test_ControlBalanceFilter(void **state)
{
    // Set up vars in balance.c scope to spoof
    motor_index = 1;

    // Test elevation current averaging
    ElevMotorData[0].current = 0.0;
    ControlBalance();
    assert_float_equal(balance_state.i_el_avg, 0.0, DBL_EPSILON);
    ElevMotorData[0].current = 150.0;
    ControlBalance();
    assert_float_equal(balance_state.i_el_avg, 1.0, DBL_EPSILON);
    ElevMotorData[0].current = 150.0;
    ControlBalance();
    assert_float_equal(balance_state.i_el_avg, 1.993333333, DBL_EPSILON);
}

/**
 * @brief Test balance system logic: balance system commanded off
 */
void test_ControlBalanceCommandOff(void **state)
{
    // Set up for an allowed move, but prevent with balance mode
    CommandData.balance.mode = bal_rest;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;
    balance_state.dir = negative;
    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1); // no move
}

/**
 * @brief Test balance system logic: slewing
 */
void test_ControlBalanceSlewing(void **state)
{
    // Set up for an allowed move, but prevent with pointing slew
    CommandData.balance.mode = bal_manual;
    CommandData.pointing_mode.nw = 1;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;
    balance_state.dir = negative;
    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1); // no move
}

/**
 * @brief Test balance system logic: scanning el
 */
void test_ControlBalanceElScan(void **state)
{
    // Set up for an allowed move, but prevent with pointing mode type
    CommandData.balance.mode = bal_manual;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_EL_SCAN;
    balance_state.dir = negative;
    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1); // no move
}

/**
 * @brief Test balance system logic: manual no move
 */
void test_ControlBalanceManualNoMove(void **state)
{
    CommandData.balance.mode = bal_manual;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;
    CommandData.balance.bal_move_type = 1; // no move
    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1); // no move
    assert_int_equal(balance_state.dir, CommandData.balance.bal_move_type);
}

/**
 * @brief Test balance system logic: manual move
 */
void test_ControlBalanceManualMove(void **state)
{
    CommandData.balance.mode = bal_manual;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;
    CommandData.balance.bal_move_type = 2;
    ControlBalance();
    assert_int_equal(balance_state.do_move, 1);
    assert_int_equal(balance_state.dir, 2);
    assert_int_equal(balance_state.dir, CommandData.balance.bal_move_type);
}

/**
 * @brief Test balance system logic: current is inside deadband and > 0
 */
void test_ControlBalanceAutoInDeadbandPos(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    // Ensure current is inside deadband
    motor_index = 1;
    ElevMotorData[0].current = 0.5;
    balance_state.i_el_avg = 0.5;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;
    balance_state.moving = 1;

    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1);
}

/**
 * @brief Test balance system logic: current is inside deadband and < 0
 */
void test_ControlBalanceAutoInDeadbandNeg(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    // Ensure current is inside deadband
    motor_index = 1;
    ElevMotorData[0].current = -0.5;
    balance_state.i_el_avg = -0.5;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;
    balance_state.moving = 1;

    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, 1);
}

/**
 * @brief Test balance system logic: current is approaching deadband, > 0
 */
void test_ControlBalanceAutoBalancingPos(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    motor_index = 1;
    ElevMotorData[0].current = 1.25;
    balance_state.i_el_avg = 1.25;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;
    balance_state.moving = 1;

    // A correction moves toward negative dir
    ControlBalance();
    assert_int_equal(balance_state.do_move, 1);
    assert_int_equal(balance_state.dir, 0);
}

/**
 * @brief Test balance system logic: current is approaching deadband, < 0
 */
void test_ControlBalanceAutoBalancingNeg(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    motor_index = 1;
    ElevMotorData[0].current = -1.25;
    balance_state.i_el_avg = -1.25;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;
    balance_state.moving = 1;

    // A correction moves toward positive dir
    ControlBalance();
    assert_int_equal(balance_state.do_move, 1);
    assert_int_equal(balance_state.dir, 2);
}

/**
 * @brief Test balance system logic: current is outside deadband, > 0
 */
void test_ControlBalanceAutoUnbalancedPos(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    motor_index = 1;
    ElevMotorData[0].current = 2.0;
    balance_state.i_el_avg = 2.0;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;

    // A correction moves toward negative dir
    ControlBalance();
    assert_int_equal(balance_state.do_move, 1);
    assert_int_equal(balance_state.dir, 0);
}

/**
 * @brief Test balance system logic: current is outside deadband, < 0
 */
void test_ControlBalanceAutoUnbalancedNeg(void **state)
{
    CommandData.balance.mode = bal_auto;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    motor_index = 1;
    ElevMotorData[0].current = -2.0;
    balance_state.i_el_avg = -2.0;
    CommandData.balance.i_el_on_bal = 1.5;
    CommandData.balance.i_el_off_bal = 1.0;

    // A correction moves toward positive dir
    ControlBalance();
    assert_int_equal(balance_state.do_move, 1);
    assert_int_equal(balance_state.dir, 2);
}

/**
 * @brief Test balance system logic: unrecognized mode, do nothing
 */
void test_ControlBalanceFallthrough(void **state)
{
    CommandData.balance.mode = 3;
    CommandData.pointing_mode.nw = 0;
    CommandData.pointing_mode.mode = P_AZEL_GOTO;

    ControlBalance();
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, no_move);
}

/**
 * @brief Test balance algorithm EZStepper commanding: first time execution
 */
void test_DoBalanceFirstTime(void **state)
{
    // Get fixture: ezbus
    struct ezbus bus = *(struct ezbus *)*state;
    // WriteBalance_5Hz expects this to be done first
    channels_initialize(channel_list);

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 15); // first EZStepper ReadInt is limit switches, make it 15 (1111)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    // firsttime resets values of balance_state
    DoBalance(&bus);
    assert_int_equal(balance_state.moving, 0);
}

/**
 * @brief Test balance algorithm EZStepper commanding: begin a positive move
 */
void test_DoBalanceBeginMovePos(void **state)
{
    struct ezbus bus = *(struct ezbus *)*state;
    // WriteBalance_5Hz expects this to be done first
    channels_initialize(channel_list);

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 15); // first EZStepper ReadInt is limit switches, make it 15 (1111)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    balance_state.do_move = 1;
    balance_state.moving = 0;
    balance_state.dir = positive;

    DoBalance(&bus);
    assert_int_equal(balance_state.moving, 1);
}

/**
 * @brief Test balance algorithm EZStepper commanding: begin a negative move
 */
void test_DoBalanceBeginMoveNeg(void **state)
{
    struct ezbus bus = *(struct ezbus *)*state;
    // WriteBalance_5Hz expects this to be done first
    channels_initialize(channel_list);

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 15); // first EZStepper ReadInt is limit switches, make it 15 (1111)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    balance_state.do_move = 1;
    balance_state.moving = 0;
    balance_state.dir = negative;

    DoBalance(&bus);
    assert_int_equal(balance_state.moving, 1);
}

/**
 * @brief Test balance algorithm EZStepper commanding: halt a move
 */
void test_DoBalanceHaltMove(void **state)
{
    struct ezbus bus = *(struct ezbus *)*state;
    // WriteBalance_5Hz expects this to be done first
    channels_initialize(channel_list);

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 15); // first EZStepper ReadInt is limit switches, make it 15 (1111)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    balance_state.do_move = 0;
    balance_state.moving = 1;
    balance_state.dir = negative;

    DoBalance(&bus);
    assert_int_equal(balance_state.moving, 0);
}

/**
 * @brief Test balance algorithm EZStepper commanding: limit switches
 */
void test_DoBalanceCheckLimits(void **state)
{
    struct ezbus bus = *(struct ezbus *)*state;
    // WriteBalance_5Hz expects this to be done first
    channels_initialize(channel_list);

    // Negative limit test

    // Set the initial state of the struct used by DoBalance()
    balance_state.do_move = 1;
    balance_state.moving = 1;
    balance_state.dir = negative;
    CommandData.balance.mode = bal_auto;

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 7); // second EZStepper ReadInt is limit switch, make it 7 (0111)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    DoBalance(&bus);
    assert_int_equal(balance_state.pos, 0);
    assert_int_equal(balance_state.lims, 7);
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, no_move);
    assert_int_equal(balance_state.moving, 0);
    assert_int_equal(CommandData.balance.mode, bal_rest);

    // Positive limit test

    balance_state.do_move = 1;
    balance_state.moving = 1;
    balance_state.dir = positive;
    CommandData.balance.mode = bal_auto;

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 0); // first EZStepper ReadInt is pos, make it 0
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_ReadInt, who, GetActAddr(BALANCENUM));
    expect_string(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, 11); // second EZStepper ReadInt is limit switch, make it 11 (1011)
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK); // retval

    DoBalance(&bus);
    assert_int_equal(balance_state.pos, 0);
    assert_int_equal(balance_state.lims, 11);
    assert_int_equal(balance_state.do_move, 0);
    assert_int_equal(balance_state.dir, no_move);
    assert_int_equal(balance_state.moving, 0);
    assert_int_equal(CommandData.balance.mode, bal_rest);
}


int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_ControlBalanceFilter),
        cmocka_unit_test(test_ControlBalanceCommandOff),
        cmocka_unit_test(test_ControlBalanceSlewing),
        cmocka_unit_test(test_ControlBalanceElScan),
        cmocka_unit_test(test_ControlBalanceManualNoMove),
        cmocka_unit_test(test_ControlBalanceManualMove),
        cmocka_unit_test(test_ControlBalanceAutoInDeadbandPos),
        cmocka_unit_test(test_ControlBalanceAutoInDeadbandNeg),
        cmocka_unit_test(test_ControlBalanceAutoBalancingPos),
        cmocka_unit_test(test_ControlBalanceAutoBalancingNeg),
        cmocka_unit_test(test_ControlBalanceAutoUnbalancedPos),
        cmocka_unit_test(test_ControlBalanceAutoUnbalancedNeg),
        cmocka_unit_test(test_ControlBalanceFallthrough),
        // cmocka_unit_test(test_WriteBalance_5Hz), // not essential, TM logging
        // !!! order matters here, due to static firsttime in DoBalance() !!!
        // !!! test_DoBalanceFirstTime must run first !!!
        cmocka_unit_test_setup_teardown(test_DoBalanceFirstTime, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_DoBalanceBeginMovePos, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_DoBalanceBeginMoveNeg, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_DoBalanceHaltMove, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_DoBalanceCheckLimits, SetupEzBus, TearDownEzBus),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
