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
    function_called();
    return mock_type(int);
}


int __wrap_EZBus_IsBusy(struct ezbus* bus, char who);
int __wrap_EZBus_IsBusy(struct ezbus* bus, char who)
{
    check_expected(who);
    function_called();
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

    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
}


/**
 * @brief Test get shutter data: query all digital inputs and commanded motor
 * position.
 */
static void test_GetShutterDataSuccess(void **state)
{
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    shutter_data.pos = -1;

    expect_value(__wrap_EZBus_ReadInt, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, SHUTTER_OPEN_BIT);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    expect_value(__wrap_EZBus_ReadInt, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 42);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    assert_int_equal(GetShutterData(&bus, id[SHUTTERNUM], &shutter_data.lims, &shutter_data.pos), 0);
    assert_int_equal(shutter_data.lims, SHUTTER_OPEN_BIT);
    assert_int_equal(shutter_data.pos, 42);
}


/**
 * @brief Test get shutter data: query all digital inputs and commanded motor
 * position. Failure paths.
 */
static void test_GetShutterDataFail(void **state)
{
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    shutter_data.pos = -1;

    expect_value(__wrap_EZBus_ReadInt, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, SHUTTER_OPEN_BIT);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_ReadInt);

    expect_value(__wrap_EZBus_ReadInt, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 42);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_ReadInt);

    assert_int_equal(GetShutterData(&bus, id[SHUTTERNUM], &shutter_data.lims, &shutter_data.pos), -1);
    assert_int_equal(shutter_data.lims, SHUTTER_OPEN_BIT);
    assert_int_equal(shutter_data.pos, 42);
}


/**
 * @brief Test GetShutterAction: goal is open
 */
static void test_GetShutterAction_Open(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_OPEN;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_OPEN);
    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_NOP);
}


/**
 * @brief Test GetShutterAction: goal is closed
 */
static void test_GetShutterAction_Closed(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_CLOSED;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_CLOSE);
    assert_int_equal(shutter_data.state, SHUTTER_UNK);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_CLOSED);
}


/**
 * @brief Test GetShutterAction: goal is closed, via slow method
 */
static void test_GetShutterAction_ClosedSlow(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_CLOSED_SLOW;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_CLOSE_SLOW);
    assert_int_equal(shutter_data.state, SHUTTER_UNK);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_CLOSED_SLOW);
}


/**
 * @brief Test GetShutterAction: goal is open then closed
 */
static void test_GetShutterAction_OpenClose(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_CLOSED2;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_OPEN_CLOSE);
    assert_int_equal(shutter_data.state, SHUTTER_UNK);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_NOP);
}


/**
 * @brief Test GetShutterAction: goal is shutter initialized
 */
static void test_GetShutterAction_Init(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_INIT;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_INIT);
    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_NOP);
}


/**
 * @brief Test GetShutterAction: goal is shutter reset
 */
static void test_GetShutterAction_Reset(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_RESET;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_RESET);
    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_NOP);
}


/**
 * @brief Test GetShutterAction: goal is shutter off
 */
static void test_GetShutterAction_Off(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_OFF;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_OFF);
    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_NOP);
}


/**
 * @brief Test GetShutterAction: goal is keeping shutter closed
 */
static void test_GetShutterAction_KeepClosed(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_KEEPCLOSED;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_KEEPCLOSED);
    assert_int_equal(shutter_data.state, SHUTTER_CLOSED);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_KEEPCLOSED);
}


/**
 * @brief Test GetShutterAction: goal is keeping shutter open
 */
static void test_GetShutterAction_KeepOpen(void **state)
{
    shutter_data.state = SHUTTER_UNK;
    CommandData.actbus.shutter_goal = SHUTTER_KEEPOPEN;
    uint32_t action = GetShutterAction(&shutter_data.state, &CommandData.actbus.shutter_goal);

    assert_int_equal(action, SHUTTER_DO_KEEPOPEN);
    assert_int_equal(shutter_data.state, SHUTTER_OPEN);
    assert_int_equal(CommandData.actbus.shutter_goal, SHUTTER_KEEPOPEN);
}

/**
 * @brief Test DoShutterAction. Note that the full code path of each action
 * done is tested separately - we just mock the subfunction's calls here to
 * to stepper API to make the test work. Maybe they could be mocked out,
 * because this test current repeats a lot of stuff from those subfunctions.
 */
static void test_DoShutterAction(void **state)
{
    int move_commanded = 1;
    // SHUTTER_DO_OFF
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);
    expect_value(__wrap_EZBus_SetIHold, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_SetIHold, current, 0);
    will_return(__wrap_EZBus_SetIHold, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_SetIHold);
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_OFF, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_CLOSE
    expect_value(__wrap_EZBus_IsBusy, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_IsBusy, 0);
    expect_function_call(__wrap_EZBus_IsBusy);
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_MoveComm, who, id[SHUTTERNUM]);
    expect_value(__wrap_EZBus_MoveComm, what, "D0");
    will_return(__wrap_EZBus_MoveComm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_MoveComm);
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_CLOSE, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_CLOSE_SLOW
    // may need to be removed when this (deprecated) method of
    // shutter closing is removed
    expect_value(__wrap_EZBus_IsBusy, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_IsBusy, 0);
    expect_function_call(__wrap_EZBus_IsBusy);
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    int tmp = shutter_data.lims;
    shutter_data.lims = SHUTTER_CLOSED_BIT;
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_CLOSE_SLOW, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;
    shutter_data.lims = tmp;

    // SHUTTER_DO_OPEN_CLOSE
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);
    expect_value(__wrap_EZBus_Comm, who, id[SHUTTERNUM]);
    expect_string(__wrap_EZBus_Comm, what, "h0z5000h50V10000P424D42R");
    will_return(__wrap_EZBus_Comm, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Comm);
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_OPEN_CLOSE, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_OPEN
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);
    // MoveComm call depends on shutter_data, dont expect it
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_OPEN, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_INIT
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);
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
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_INIT, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_RESET
    expect_value(__wrap_EZBus_Take, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Take, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Take);
    expect_value(__wrap_EZBus_Stop, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Stop, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Stop);
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
    expect_value(__wrap_EZBus_Release, who, id[SHUTTERNUM]);
    will_return(__wrap_EZBus_Release, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_Release);
    DoShutterAction(SHUTTER_DO_RESET, &move_commanded);
    assert_int_equal(move_commanded, 0);
    move_commanded = 1;

    // SHUTTER_DO_KEEPCLOSED
    shutter_data.move_commanded = 1;
    DoShutterAction(SHUTTER_DO_KEEPCLOSED, &move_commanded);
    assert_int_equal(shutter_data.move_commanded, 1);

    // SHUTTER_DO_KEEPOPEN
    shutter_data.move_commanded = 1;
    DoShutterAction(SHUTTER_DO_KEEPOPEN, &move_commanded);
    assert_int_equal(shutter_data.move_commanded, 0);

    DoShutterAction(SHUTTER_DO_NOP, &move_commanded);
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
        // TODO(evanmayer): CloseSlowShutter is deprecated, not tested for now
        cmocka_unit_test(test_OpenShutter_ClosedSuccess),
        cmocka_unit_test(test_OpenShutter_ClosedFail),
        cmocka_unit_test(test_OpenShutter_Open),
        cmocka_unit_test(test_GetShutterDataSuccess),
        cmocka_unit_test(test_GetShutterDataFail),
        cmocka_unit_test(test_GetShutterAction_Open),
        cmocka_unit_test(test_GetShutterAction_Closed),
        cmocka_unit_test(test_GetShutterAction_ClosedSlow),
        cmocka_unit_test(test_GetShutterAction_OpenClose),
        cmocka_unit_test(test_GetShutterAction_Init),
        cmocka_unit_test(test_GetShutterAction_Reset),
        cmocka_unit_test(test_GetShutterAction_Off),
        cmocka_unit_test(test_GetShutterAction_KeepClosed),
        cmocka_unit_test(test_GetShutterAction_KeepOpen),
        cmocka_unit_test(test_DoShutterAction)
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
