/**
 * @file test_el_lock.c
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

#include "actuators.c"

#include "mcp_mock_decl.c"


// ============================================================================
// Mock objects
// ============================================================================
// Mock objects named __wrap_<funcName> are bound to the symbol <funcName> when
// the linker is invoked with --wrap=<funcName>, replacing it in any
// compilation units this file is linked to. The real func is available at
// __real_<funcName>.

int __wrap_EZBus_IsTaken(struct ezbus* bus, char who);
int __wrap_EZBus_IsTaken(struct ezbus* bus, char who)
{
    check_expected(who);
    return mock_type(int);
}

int __wrap_EZBus_Comm(struct ezbus* bus, char who, const char* what);
int __wrap_EZBus_Comm(struct ezbus* bus, char who, const char* what)
{
    check_expected(who);
    check_expected_ptr(what);

    lock_data.adc[0] = mock_type(int16_t);
    lock_data.adc[1] = mock_type(int16_t);
    lock_data.adc[2] = mock_type(int16_t);
    lock_data.adc[3] = mock_type(int16_t);

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

int __wrap_EZBus_Stop(struct ezbus* bus, char who, const char* what, int* val);
int __wrap_EZBus_Stop(struct ezbus* bus, char who, const char* what, int* val)
{
    return EZ_ERR_OK;
}

int __wrap_EZBus_RelMove(struct ezbus* bus, char who, int delta);
int __wrap_EZBus_RelMove(struct ezbus* bus, char who, int delta)
{
    return EZ_ERR_OK;
}

int __wrap_EZBus_MoveVel(struct ezbus* bus, char who, int vel);
int __wrap_EZBus_MoveVel(struct ezbus* bus, char who, int vel)
{
    return EZ_ERR_OK;
}


int __wrap_EZBus_SetIHold(struct ezbus* bus, char who, int current);
int __wrap_EZBus_SetIHold(struct ezbus* bus, char who, int current)
{
    return EZ_ERR_OK;
}


int __wrap_EZBus_SetIMove(struct ezbus* bus, char who, int current);
int __wrap_EZBus_SetIMove(struct ezbus* bus, char who, int current)
{
    return EZ_ERR_OK;
}


int __wrap_EZBus_SetVel(struct ezbus* bus, char who, int vel);
int __wrap_EZBus_SetVel(struct ezbus* bus, char who, int vel)
{
    return EZ_ERR_OK;
}


int __wrap_EZBus_SetAccel(struct ezbus* bus, char who, int acc);
int __wrap_EZBus_SetAccel(struct ezbus* bus, char who, int acc)
{
    return EZ_ERR_OK;
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
    return EZ_ERR_OK;
}


int __wrap_EZBus_Release(struct ezbus* bus, char who);
int __wrap_EZBus_Release(struct ezbus* bus, char who)
{
    return EZ_ERR_OK;
}


int __wrap_EZBus_IsBusy(struct ezbus* bus, char who);
int __wrap_EZBus_IsBusy(struct ezbus* bus, char who)
{
    return EZ_ERR_OK;
}

// ============================================================================
// Setup/teardown functions (test fixtures)
// ============================================================================
/**
 * @brief Set up the structs for el lock system tests
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
 * @brief Tear down the structs for el lock system tests
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
/*
 * @brief Test querying elevation lock limit switches
 */
/**
 * @brief Test get lock data: query all digital inputs and commanded motor
 * position.
 */
static void test_GetLockDataSuccess(void **state)
{
    lock_data.lims = LOCK_CLOSED_BIT;
    lock_data.pos = -1;

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, LOCK_OPEN_BIT);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 42);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    assert_int_equal(GetLockData(&bus, id[LOCKNUM], &lock_data.lims, &lock_data.pos), 0);
    assert_int_equal(lock_data.lims, LOCK_OPEN_BIT);
    assert_int_equal(lock_data.pos, 42);
}

/**
 * @brief Test get lock data: query all digital inputs and commanded motor
 * position. Failure paths.
 */
static void test_GetLockDataFail(void **state)
{
    lock_data.lims = LOCK_CLOSED_BIT;
    lock_data.pos = -1;

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, LOCK_OPEN_BIT);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_ReadInt);

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 42);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_BAD_WHO);
    expect_function_call(__wrap_EZBus_ReadInt);

    assert_int_equal(GetLockData(&bus, id[LOCKNUM], &lock_data.lims, &lock_data.pos), -1);
    assert_int_equal(lock_data.lims, LOCK_OPEN_BIT);
    assert_int_equal(lock_data.pos, 42);
}

void test_SetLockStateNotInCharge(void **state)
{
    channels_initialize(channel_list);

    // Ensure proper lock_data.state is set based on data from in charge comp
    // lock closed - lock pin fully extended
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("lims_lock"), 0b1101);
    SetLockState(1); // counter = 1
    assert_int_equal(lock_data.state, 770);
    assert_int_equal(lock_data.lims, 0b1101);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("lims_lock"), 0b1110);
    SetLockState(1); // counter = 2
    assert_int_equal(lock_data.state, 769);
    assert_int_equal(lock_data.lims, 0b1110);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("lims_lock"), 0b1111);
    SetLockState(1); // counter = 3
    assert_int_equal(lock_data.state, 768);
    assert_int_equal(lock_data.lims, 0b1111);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("lims_lock"), 0b1111);
    ACSData.enc_motor_elev = 100.0;
    SetLockState(1); // counter = 4
    assert_int_equal(lock_data.state, 256);
    assert_int_equal(lock_data.lims, 0b1111);
    ACSData.enc_motor_elev = 0.0;
}

void test_SetLockStateInCharge(void **state)
{
    channels_initialize(channel_list);

    // lock closed - lock pin fully extended
    lock_data.state = LS_DRIVE_UNK;
    lock_data.lims = 13;
    SetLockState(0); // counter = 1
    assert_int_equal(lock_data.state, 770);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    lock_data.lims = 14;
    SetLockState(0); // counter = 2
    assert_int_equal(lock_data.state, 769);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    lock_data.lims = 15;
    SetLockState(0); // counter = 3
    assert_int_equal(lock_data.state, 768);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    lock_data.lims = 15;
    ACSData.enc_motor_elev = 100.0;
    SetLockState(0); // counter = 4
    assert_int_equal(lock_data.state, 256);
    ACSData.enc_motor_elev = 0.0;
}

void test_GetLockAction(void **state)
{
    int action = LA_EXIT;

    lock_timeout = -1; // no timeout

    // Goal: lock open, drive off
    uint32_t lock_goal = LS_OPEN | LS_DRIVE_OFF;

    // State: lock open, drive off
    uint32_t lock_state = LS_OPEN | LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: lock open, drive not off
    lock_state = LS_OPEN;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: lock not open, retracting
    lock_state = LS_DRIVE_RET;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: lock not open, drive off
    lock_state = LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_RETRACT);
    // State: lock not open, drive not off
    lock_state = 0x0;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: lock closed, drive off
    lock_goal = LS_CLOSED | LS_DRIVE_OFF;

    // State: lock not open, drive off
    lock_state = 0x6;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: lock not open, drive not off
    lock_state = LS_CLOSED;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but LS_DRIVE_STP??
    lock_state = 577;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: el axis is lockable, but extending, so drive not off
    lock_state = LS_EL_OK | LS_DRIVE_EXT;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: el axis is lockable, but we have commanded stop
    // (nothing seems to command this)
    lock_state = LS_DRIVE_STP;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: lock ok to close, but not closed, and drive stop is commanded
    lock_state = (LS_EL_OK | !LS_CLOSED | LS_DRIVE_STP);
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but lock is not closed and drive is off
    lock_state = LS_EL_OK | LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXTEND);
    // fall-through case
    lock_state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but lock is not closed and drive is not off
    lock_state = (LS_OPEN | !LS_DRIVE_OFF);
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el is not lockable but we are ignoring this
    // We just tested the other logic under this branch,
    // so only test one for this other way of entering it.
    lock_state = (!LS_EL_OK) | LS_DRIVE_OFF;
    lock_goal = LS_CLOSED | LS_DRIVE_OFF | LS_IGNORE_EL;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXTEND);
    // State: el is not in range and we are not ignoring it
    lock_state = (!LS_EL_OK) | LS_DRIVE_OFF;
    lock_goal = LS_CLOSED | LS_DRIVE_OFF | !LS_IGNORE_EL;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    lock_state = (!LS_EL_OK) | !LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: drive off
    lock_goal = LS_DRIVE_OFF;

    // State: drive already off
    lock_state = LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: drive not off yet
    lock_state = !LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: unrecognized lock goal: assume drive off
    lock_goal = !(LS_OPEN | LS_CLOSED | LS_DRIVE_OFF);
    lock_state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(lock_goal, LS_DRIVE_OFF);

    // Timeout case: should always stop
    lock_timeout = 0;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
}

// void test_DoLockFixWeirdStates(void **state)
// {
//     // later
// }

void test_DoLockAction(void **state)
{
    // Stop motor
    uint32_t lock_state = LS_OPEN;
    int lock_timeout = -1;
    DoLockAction(LA_STOP, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_OFF);

    // Extend lock pin
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockAction(LA_EXTEND, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, DRIVE_TIMEOUT);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_EXT);

    // Retract lock pin
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockAction(LA_RETRACT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, DRIVE_TIMEOUT);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_RET);

    // Wait
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockAction(LA_WAIT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, LS_OPEN);

    // Exit
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockAction(LA_EXIT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, LS_OPEN);
}

void test_DoLock(void **state)
{
    // NOTE(evanmayer): a lot of this function is hard to test. It's supposed
    // to be an infinite loop, and the exit condition is supposed to be driven
    // by an update in another function. We'll have to take < 100% coverage
    // for this one until a refactor can be done.

    // Test timeout breakout

    // Mocking for GetLockData

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?4");
    will_return(__wrap_EZBus_ReadInt, LOCK_OPEN_BIT);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    expect_value(__wrap_EZBus_ReadInt, who, id[LOCKNUM]);
    expect_value(__wrap_EZBus_ReadInt, what, "?0");
    will_return(__wrap_EZBus_ReadInt, 42);
    will_return(__wrap_EZBus_ReadInt, EZ_ERR_OK);
    expect_function_call(__wrap_EZBus_ReadInt);

    // Mocking for SetLockState
    channels_initialize(channel_list);

    // Make action be exit
    CommandData.actbus.lock_goal = !(LS_OPEN | LS_CLOSED | LS_DRIVE_OFF);
    lock_data.state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    lock_timeout = -1;
    DoLock();
    assert_int_equal(lock_timeout, -1);
}

// ============================================================================
// (Old) Test functions
// ============================================================================
/**
 * @brief Test querying elevation lock ADCs
 */
void test_GetLockADCsReturnOld(void **state)
{
    lock_data.adc[0] = 42;
    lock_data.adc[1] = 43;
    lock_data.adc[2] = 44;
    lock_data.adc[3] = 45;

    expect_value(__wrap_EZBus_IsTaken, who, '5');
    will_return(__wrap_EZBus_IsTaken, 1); // not ok

    GetLockADCsOld(); // counter = 1
    // Ensure lock ADC data is unchanged
    assert_int_equal(lock_data.adc[0], 42);
    assert_int_equal(lock_data.adc[1], 43);
    assert_int_equal(lock_data.adc[2], 44);
    assert_int_equal(lock_data.adc[3], 45);
}

void test_GetLockADCsOld(void **state)
{
    expect_value(__wrap_EZBus_IsTaken, who, '5');
    will_return(__wrap_EZBus_IsTaken, EZ_ERR_OK); // retval

    expect_value(__wrap_EZBus_Comm, who, '5');
    expect_string(__wrap_EZBus_Comm, what, "?aa");
    // Pack data into bus buffer
    will_return(__wrap_EZBus_Comm, 1);
    will_return(__wrap_EZBus_Comm, 2);
    will_return(__wrap_EZBus_Comm, 3);
    will_return(__wrap_EZBus_Comm, 4);
    will_return(__wrap_EZBus_Comm, EZ_ERR_OK); // retval

    GetLockADCsOld(); // counter = 2
    // Ensure lock data was retrieved from buffer
    assert_int_equal(lock_data.adc[0], 1);
    assert_int_equal(lock_data.adc[1], 2);
    assert_int_equal(lock_data.adc[2], 3);
    assert_int_equal(lock_data.adc[3], 4);
}

void test_SetLockStateNotInChargeOld(void **state)
{
    channels_initialize(channel_list);

    // lock closed - lock pin fully extended
    // Ensure proper lock_data.state is set based on data from in charge comp
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 0);
    SetLockStateOld(1); // counter = 1
    assert_int_equal(lock_data.state, 770);
    assert_int_equal(lock_data.adc[1], 0);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 16000);
    SetLockStateOld(1); // counter = 2
    assert_int_equal(lock_data.state, 769);
    assert_int_equal(lock_data.adc[1], 16000);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 3499);
    SetLockStateOld(1); // counter = 3
    assert_int_equal(lock_data.state, 768);
    assert_int_equal(lock_data.adc[1], 3499);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 3499);
    ACSData.enc_motor_elev = 100.0;
    SetLockStateOld(1); // counter = 4
    assert_int_equal(lock_data.state, 256);
    assert_int_equal(lock_data.adc[1], 3499);
    ACSData.enc_motor_elev = 0.0;
}

void test_SetLockStateInChargeOld(void **state)
{
    channels_initialize(channel_list);

    // lock closed - lock pin fully extended
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 0;
    SetLockStateOld(0); // counter = 1
    assert_int_equal(lock_data.state, 770);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 16000;
    SetLockStateOld(0); // counter = 2
    assert_int_equal(lock_data.state, 769);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 3499;
    SetLockStateOld(0); // counter = 3
    assert_int_equal(lock_data.state, 768);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 3499;
    ACSData.enc_motor_elev = 100.0;
    SetLockStateOld(0); // counter = 4
    assert_int_equal(lock_data.state, 256);
    ACSData.enc_motor_elev = 0.0;
}

void test_GetLockActionOld(void **state)
{
    int action = LA_EXIT;

    lock_timeout = -1; // no timeout

    // Goal: lock open, drive off
    uint32_t lock_goal = LS_OPEN | LS_DRIVE_OFF;

    // State: lock open, drive off
    uint32_t lock_state = LS_OPEN | LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: lock open, drive not off
    lock_state = LS_OPEN;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: lock not open, retracting
    lock_state = LS_DRIVE_RET;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: lock not open, drive off
    lock_state = LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_RETRACT);
    // State: lock not open, drive not off
    lock_state = 0x0;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: lock closed, drive off
    lock_goal = LS_CLOSED | LS_DRIVE_OFF;

    // State: lock not open, drive off
    lock_state = 0x6;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: lock not open, drive not off
    lock_state = LS_CLOSED;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but LS_DRIVE_STP??
    lock_state = 577;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: el axis is lockable, but extending, so drive not off
    lock_state = LS_EL_OK | LS_DRIVE_EXT;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    // State: el axis is lockable, but we have commanded stop
    // (nothing seems to command this)
    lock_state = LS_DRIVE_STP;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: lock ok to close, but not closed, and drive stop is commanded
    lock_state = (LS_EL_OK | !LS_CLOSED | LS_DRIVE_STP);
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but lock is not closed and drive is off
    lock_state = LS_EL_OK | LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXTEND);
    // fall-through case
    lock_state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el axis is lockable, but lock is not closed and drive is not off
    lock_state = (LS_OPEN | !LS_DRIVE_OFF);
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
    // State: el is not lockable but we are ignoring this
    // We just tested the other logic under this branch,
    // so only test one for this other way of entering it.
    lock_state = (!LS_EL_OK) | LS_DRIVE_OFF;
    lock_goal = LS_CLOSED | LS_DRIVE_OFF | LS_IGNORE_EL;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXTEND);
    // State: el is not in range and we are not ignoring it
    lock_state = (!LS_EL_OK) | LS_DRIVE_OFF;
    lock_goal = LS_CLOSED | LS_DRIVE_OFF | !LS_IGNORE_EL;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_WAIT);
    lock_state = (!LS_EL_OK) | !LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: drive off
    lock_goal = LS_DRIVE_OFF;

    // State: drive already off
    lock_state = LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: drive not off yet
    lock_state = !LS_DRIVE_OFF;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Goal: unrecognized lock goal: assume drive off
    lock_goal = !(LS_OPEN | LS_CLOSED | LS_DRIVE_OFF);
    lock_state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(lock_goal, LS_DRIVE_OFF);

    // Timeout case: should always stop
    lock_timeout = 0;
    action = GetLockActionOld(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
}

// void test_DoLockFixWeirdStates(void **state)
// {
//     // later
// }

void test_DoLockActionOld(void **state)
{
    // Stop motor
    uint32_t lock_state = LS_OPEN;
    int lock_timeout = -1;
    DoLockActionOld(LA_STOP, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_OFF);

    // Extend lock pin
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockActionOld(LA_EXTEND, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, DRIVE_TIMEOUT);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_EXT);

    // Retract lock pin
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockActionOld(LA_RETRACT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, DRIVE_TIMEOUT);
    assert_int_equal(lock_state, (LS_OPEN & ~LS_DRIVE_MASK) | LS_DRIVE_RET);

    // Wait
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockActionOld(LA_WAIT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, LS_OPEN);

    // Exit
    lock_state = LS_OPEN;
    lock_timeout = -1;
    DoLockActionOld(LA_EXIT, &lock_timeout, &lock_state);
    assert_int_equal(lock_timeout, -1);
    assert_int_equal(lock_state, LS_OPEN);
}

void test_DoLockOld(void **state)
{
    // NOTE(evanmayer): a lot of this function is hard to test. It's supposed
    // to be an infinite loop, and the exit condition is supposed to be driven
    // by an update in another function. We'll have to take < 100% coverage
    // for this one until a refactor can be done.

    // Test timeout breakout

    // Mocking for GetLockADCs
    expect_value(__wrap_EZBus_IsTaken, who, '5');
    will_return(__wrap_EZBus_IsTaken, EZ_ERR_OK); // retval
    expect_value(__wrap_EZBus_Comm, who, '5');
    expect_string(__wrap_EZBus_Comm, what, "?aa");
    will_return(__wrap_EZBus_Comm, 1);
    will_return(__wrap_EZBus_Comm, 2);
    will_return(__wrap_EZBus_Comm, 3);
    will_return(__wrap_EZBus_Comm, 4);
    will_return(__wrap_EZBus_Comm, EZ_ERR_OK); // retval

    // Mocking for SetLockState
    channels_initialize(channel_list);

    // Make action be exit
    CommandData.actbus.lock_goal = !(LS_OPEN | LS_CLOSED | LS_DRIVE_OFF);
    lock_data.state = (LS_EL_OK | !LS_CLOSED | !LS_DRIVE_STP);
    lock_timeout = -1;
    DoLockOld();
    assert_int_equal(lock_timeout, -1);
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_GetLockDataSuccess, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_GetLockDataFail, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateNotInCharge, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateInCharge, SetupEzBus, TearDownEzBus),
        cmocka_unit_test(test_GetLockAction),
        // cmocka_unit_test(test_DoLockFixWeirdStates), // TODO(evanmayer) separate logic piece, less critical
        cmocka_unit_test(test_DoLockAction),
        cmocka_unit_test(test_DoLock),

        cmocka_unit_test_setup_teardown(test_GetLockADCsReturnOld, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_GetLockADCsOld, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateNotInChargeOld, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateInChargeOld, SetupEzBus, TearDownEzBus),
        cmocka_unit_test(test_GetLockActionOld),
        cmocka_unit_test(test_DoLockActionOld),
        cmocka_unit_test(test_DoLockOld),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
