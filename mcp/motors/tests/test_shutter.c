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
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_SetIMove(struct ezbus* bus, char who, int current);
int __wrap_EZBus_SetIMove(struct ezbus* bus, char who, int current)
{
    check_expected(who);
    check_expected_ptr(current);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_SetVel(struct ezbus* bus, char who, int vel);
int __wrap_EZBus_SetVel(struct ezbus* bus, char who, int vel)
{
    check_expected(who);
    check_expected_ptr(vel);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_SetAccel(struct ezbus* bus, char who, int acc);
int __wrap_EZBus_SetAccel(struct ezbus* bus, char who, int acc)
{
    check_expected(who);
    check_expected_ptr(acc);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_MoveComm(struct ezbus* bus, char who, const char* what);
int __wrap_EZBus_MoveComm(struct ezbus* bus, char who, const char* what)
{
    check_expected(who);
    check_expected_ptr(what);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_Take(struct ezbus* bus, char who);
int __wrap_EZBus_Take(struct ezbus* bus, char who)
{
    check_expected(who);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_Stop(struct ezbus* bus, char who);
int __wrap_EZBus_Stop(struct ezbus* bus, char who)
{
    check_expected(who);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_Release(struct ezbus* bus, char who);
int __wrap_EZBus_Release(struct ezbus* bus, char who)
{
    check_expected(who);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_Comm(struct ezbus* bus, char who, const char* what);
int __wrap_EZBus_Comm(struct ezbus* bus, char who, const char* what)
{
    check_expected(who);
    check_expected_ptr(what);
    function_called();
    return mock_type(int);
}

int __wrap_EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val);
int __wrap_EZBus_ReadInt(struct ezbus* bus, char who, const char* what, int* val)
{
    check_expected(who);
    check_expected_ptr(what);
    *val = mock_type(int);
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
static void test_TurnOffShutter_Success(void **state)
{
    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    CommandData.actbus.shutter_hold_i = 1;
    TurnOffShutter();
    assert_int_equal(CommandData.actbus.shutter_hold_i, 0);
}

/**
 * @brief Test failure to turn off shutter hold current
 */
static void test_TurnOffShutter_Fail(void **state)
{
    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_SERR_BUSY);
    expect_function_call(__wrap_EZBus_SetIHold);

    CommandData.actbus.shutter_hold_i = 1;
    TurnOffShutter();
    assert_int_equal(CommandData.actbus.shutter_hold_i, 1);
}

/**
 * @brief Test shutter stepper motor init
 */
static void test_InitializeShutter(void **state)
{
    CommandData.actbus.shutter_move_i = 42;
    CommandData.actbus.shutter_hold_i = 43;
    CommandData.actbus.shutter_vel = 44;
    CommandData.actbus.shutter_acc = 45;

    expect_value(__wrap_EZBus_SetIMove, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIMove, current, 42);
    will_return(__wrap_EZBus_SetIMove, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIMove);

    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 43);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    expect_value(__wrap_EZBus_SetVel, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetVel, vel, 44);
    will_return(__wrap_EZBus_SetVel, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetVel);

    expect_value(__wrap_EZBus_SetAccel, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetAccel, acc, 45);
    will_return(__wrap_EZBus_SetAccel, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetAccel);

    InitializeShutter();
}

/**
 * @brief Test shutter "reset": set hold current to 0 and re-close
 */
static void test_ResetShutter_Success(void **state)
{
    CommandData.actbus.shutter_move_i = 42;
    CommandData.actbus.shutter_hold_i = 43;
    CommandData.actbus.shutter_vel = 44;
    CommandData.actbus.shutter_acc = 45;

    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, CommandData.actbus.shutter_hold_i);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);

    ResetShutter();
}

/**
 * @brief Test shutter "reset": set hold current to 0 and re-close,
 * but move fails.
 */
static void test_ResetShutter_Fail(void **state)
{
    CommandData.actbus.shutter_move_i = 42;
    CommandData.actbus.shutter_hold_i = 43;
    CommandData.actbus.shutter_vel = 44;
    CommandData.actbus.shutter_acc = 45;

    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, CommandData.actbus.shutter_hold_i);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_BAD_WHO); // some random bad retval
    expect_function_call(__wrap_EZBus_MoveComm);

    ResetShutter();
}

/**
 * @brief Test shutter close: cancel overrides all logic.
 */
static void test_KeepClosedShutter_Cancel(void **state)
{
    int cancel = 1;
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    shutter_data.move_commanded = 42;

    KeepClosedShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 42);
}

/**
 * @brief Test shutter close: endless negative move
 */
static void test_KeepClosedShutter_NotClosedSuccess(void **state)
{
    int cancel = 0;
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    shutter_data.move_commanded = 0;

    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);

    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);

    KeepClosedShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 1);
}

/**
 * @brief Test shutter close: endless negative move, but move command fails
 */
static void test_KeepClosedShutter_NotClosedFail(void **state)
{
    int cancel = 0;
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    shutter_data.move_commanded = 0;

    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_MoveComm);

    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);

    KeepClosedShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 1);
}

/**
 * @brief Test shutter close: already closed
 */
static void test_KeepClosedShutter_Closed(void **state)
{
    int cancel = 0;
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    shutter_data.move_commanded = 1;

    KeepClosedShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 0);
}

/**
 * @brief Test shutter open: cancel overrides all logic.
 */
static void test_KeepOpenShutter_Cancel(void **state)
{
    int cancel = 1;
    shutter_data.lims = !SHUTTER_OPEN_BIT;
    shutter_data.move_commanded = 42;

    KeepOpenShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 42);
}

/**
 * @brief Test shutter open: endless positive move
 */
static void test_KeepOpenShutter_NotOpenSuccess(void **state)
{
    int cancel = 0;
    shutter_data.lims = !SHUTTER_OPEN_BIT;
    shutter_data.move_commanded = 0;

    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "P0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);

    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);

    KeepOpenShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 1);
}

/**
 * @brief Test shutter open: endless positive move, but move command fails
 */
static void test_KeepOpenShutter_NotOpenFail(void **state)
{
    int cancel = 0;
    shutter_data.lims = !SHUTTER_OPEN_BIT;
    shutter_data.move_commanded = 0;

    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "P0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_MoveComm);

    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);

    KeepOpenShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 1);
}

/**
 * @brief Test shutter open: already open
 */
static void test_KeepOpenShutter_Open(void **state)
{
    int cancel = 0;
    shutter_data.lims = SHUTTER_OPEN_BIT;
    CommandData.actbus.shutter_step = 42;

    KeepOpenShutter(&cancel);

    assert_int_equal(shutter_data.move_commanded, 0);
}

/**
 * @brief Test open+close shutter: shutter not closed yet
 */
static void test_OpenCloseShutter_OpenSuccess(void **state)
{
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    CommandData.actbus.shutter_step = 42;

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_Comm, who, id[SHUTTERNUM]);
    expect_string(__wrap_EZBus_Comm, what, "h0z5000h50V10000P424D42R");
    will_return(__wrap_EZBus_Comm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Comm);

    OpenCloseShutter();
}

/**
 * @brief Test open+close shutter: shutter not closed yet, but move cmd fails
 */
static void test_OpenCloseShutter_OpenFail(void **state)
{
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    CommandData.actbus.shutter_step = 42;

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    expect_value(__wrap_EZBus_Comm, who, id[SHUTTERNUM]);
    expect_string(__wrap_EZBus_Comm, what, "h0z5000h50V10000P424D42R");
    will_return(__wrap_EZBus_Comm, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_Comm);

    OpenCloseShutter();
}

/**
 * @brief Test open+close shutter: shutter already closed
 */
static void test_OpenCloseShutter_Closed(void **state)
{
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    CommandData.actbus.shutter_step = 42;

    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);

    OpenCloseShutter();
}

/**
 * @brief Test close shutter: not yet closed
 */
static void test_CloseShutter_OpenSuccess(void **state)
{
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    shutter_data.state = SHUTTER_UNK;

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);

    CloseShutter();

    assert_int_equal(shutter_data.state, SHUTTER_UNK);
}

/**
 * @brief Test close shutter: not yet closed, but fails to move
 */
static void test_CloseShutter_OpenFail(void **state)
{
    shutter_data.lims = !SHUTTER_CLOSED_BIT;
    shutter_data.state = SHUTTER_UNK;

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_MoveComm);

    CloseShutter();

    assert_int_equal(shutter_data.state, SHUTTER_UNK);
}

/**
 * @brief Test close shutter: already closed
 */
static void test_CloseShutter_Closed(void **state)
{
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    shutter_data.state = SHUTTER_UNK;

    CloseShutter();

    assert_int_equal(shutter_data.state, SHUTTER_CLOSED);
}

/**
 * @brief Test open shutter: not yet open
 */
static void test_OpenShutter_ClosedSuccess(void **state)
{
    shutter_data.lims = !SHUTTER_OPEN_BIT;
    shutter_data.state = SHUTTER_UNK;

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "P0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);

    OpenShutter();

    assert_int_equal(shutter_data.state, SHUTTER_UNK);
}

/**
 * @brief Test open shutter: not yet open, but fails to move
 */
static void test_OpenShutter_ClosedFail(void **state)
{
    shutter_data.lims = !SHUTTER_OPEN_BIT;
    shutter_data.state = SHUTTER_UNK;

    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "P0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_MoveComm);

    OpenShutter();

    assert_int_equal(shutter_data.state, SHUTTER_UNK);
}

/**
 * @brief Test close shutter: already open
 */
static void test_OpenShutter_Open(void **state)
{
    shutter_data.lims = SHUTTER_OPEN_BIT;
    shutter_data.state = SHUTTER_UNK;

    OpenShutter();

    assert_int_equal(shutter_data.state, -42);
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_TurnOffShutter_Success),
        cmocka_unit_test(test_TurnOffShutter_Fail),
        cmocka_unit_test(test_InitializeShutter),
        cmocka_unit_test(test_ResetShutter_Success),
        cmocka_unit_test(test_ResetShutter_Fail),
        cmocka_unit_test(test_KeepClosedShutter_Cancel),
        cmocka_unit_test(test_KeepClosedShutter_NotClosedSuccess),
        cmocka_unit_test(test_KeepClosedShutter_NotClosedFail),
        cmocka_unit_test(test_KeepClosedShutter_Closed),
        cmocka_unit_test(test_KeepOpenShutter_Cancel),
        cmocka_unit_test(test_KeepOpenShutter_NotOpenSuccess),
        cmocka_unit_test(test_KeepOpenShutter_NotOpenFail),
        cmocka_unit_test(test_KeepOpenShutter_Open),
        cmocka_unit_test(test_OpenCloseShutter_OpenSuccess),
        cmocka_unit_test(test_OpenCloseShutter_OpenFail),
        cmocka_unit_test(test_OpenCloseShutter_Closed),
        cmocka_unit_test(test_CloseShutter_OpenSuccess),
        cmocka_unit_test(test_CloseShutter_OpenFail),
        cmocka_unit_test(test_CloseShutter_Closed),
        // TODO(evanmayer): CloseSlowShutter
        cmocka_unit_test(test_OpenShutter_ClosedSuccess),
        cmocka_unit_test(test_OpenShutter_ClosedFail),
        cmocka_unit_test(test_OpenShutter_Open),
        // TODO(evanmayer): GetShutterData
        // TODO(evanmayer): DoShutter
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
