/**
 * @file test_pointing.c
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

#include "balance.c"

#include "mcp_mock_decl.c"


// ============================================================================
// Setup/teardown functions (text fixtures)
// ============================================================================
/**
 * @brief Set up the structs for El Solution tests
 */
// static int SetupElSolution(void **state)
// {
//     *state = calloc(1, sizeof(struct ElSolutionStruct));
//     const struct ElSolutionStruct ElSol = {
//         .angle = 45.0,
//         .variance = 0.5,
//         .samp_weight = 1.0,
//         .sys_var = 0.5,
//         .trim = 0.0,
//         .last_input = 45.0,
//         .gy_int = 0.0,
//         .offset_gy = 1.0,
//         .FC = 1.0,
//         .n_solutions = 0,
//         .since_last = 0,
//         .fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct)),
//         .new_offset_ifel_gy = 0.0,
//         .int_ifel = 0.0,
//         .prev_sol_el = 45.0
//     };
//     init_fir(ElSol.fs, FIR_LENGTH, 0, 0);
//     memcpy(*state, &ElSol, sizeof(struct ElSolutionStruct));
//     return 0;
// }

/**
 * @brief Tear down the structs for El Solution tests
 */
// static int TearDownElSolution(void **state)
// {
//     free(*state);
//     return 0;
// }


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
 * @brief Test balance system logic: current is outside deadband and positive
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
 * @brief Test balance system logic: current is outside deadband and negative
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
        // cmocka_unit_test(test_WriteBalance_5Hz), // not essential, TM logging
        // cmocka_unit_test(test_DoBalance), // hardware interface: EZBus
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
