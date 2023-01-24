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

    return mock_type(int);
}

// ============================================================================
// Setup/teardown functions (text fixtures)
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

/**
 * @brief Load mocked functions relevant to DoLock with expected parameters
 * and mock effects.
 */
void ExpectReturnDoLock(void)
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
}


// ============================================================================
// Test functions
// ============================================================================
/**
 * @brief Test querying elevation lock ADCs
 */
void test_GetLockADCsReturn(void **state)
{
    lock_data.adc[0] = 42;
    lock_data.adc[1] = 43;
    lock_data.adc[2] = 44;
    lock_data.adc[3] = 45;

    expect_value(__wrap_EZBus_IsTaken, who, '5');
    will_return(__wrap_EZBus_IsTaken, 1); // not ok

    GetLockADCs(); // counter = 1
    // Ensure lock ADC data is unchanged
    assert_int_equal(lock_data.adc[0], 42);
    assert_int_equal(lock_data.adc[1], 43);
    assert_int_equal(lock_data.adc[2], 44);
    assert_int_equal(lock_data.adc[3], 45);
}

void test_GetLockADCs(void **state)
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

    GetLockADCs(); // counter = 2
    // Ensure lock data was retrieved from buffer
    assert_int_equal(lock_data.adc[0], 1);
    assert_int_equal(lock_data.adc[1], 2);
    assert_int_equal(lock_data.adc[2], 3);
    assert_int_equal(lock_data.adc[3], 4);
}

void test_SetLockStateNotInCharge(void **state)
{
    channels_initialize(channel_list);

    // lock closed - lock pin fully extended
    // Ensure proper lock_data.state is set based on data from in charge comp
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 0);
    SetLockState(1); // counter = 1
    assert_int_equal(lock_data.state, 770);
    assert_int_equal(lock_data.adc[1], 0);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 16000);
    SetLockState(1); // counter = 2
    assert_int_equal(lock_data.state, 769);
    assert_int_equal(lock_data.adc[1], 16000);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 3499);
    SetLockState(1); // counter = 3
    assert_int_equal(lock_data.state, 768);
    assert_int_equal(lock_data.adc[1], 3499);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    SET_UINT16(channels_find_by_name("state_lock"), LS_DRIVE_UNK);
    SET_UINT16(channels_find_by_name("pot_lock"), 3499);
    ACSData.enc_motor_elev = 100.0;
    SetLockState(1); // counter = 4
    assert_int_equal(lock_data.state, 256);
    assert_int_equal(lock_data.adc[1], 3499);
    ACSData.enc_motor_elev = 0.0;
}

void test_SetLockStateInCharge(void **state)
{
    channels_initialize(channel_list);

    // lock closed - lock pin fully extended
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 0;
    SetLockState(0); // counter = 1
    assert_int_equal(lock_data.state, 770);

    // lock open - lock pin retracted
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 16000;
    SetLockState(0); // counter = 2
    assert_int_equal(lock_data.state, 769);

    // lock is near either limit?
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 3499;
    SetLockState(0); // counter = 3
    assert_int_equal(lock_data.state, 768);

    // elevation axis is outside of safe range for inserting pin
    lock_data.state = LS_DRIVE_UNK;
    lock_data.adc[1] = 3499;
    ACSData.enc_motor_elev = 100.0;
    SetLockState(0); // counter = 4
    assert_int_equal(lock_data.state, 256);
    ACSData.enc_motor_elev = 0.0;
}

void test_DoLockFixWeirdStates(void **state)
{
    // later
}

void test_GetLockAction(void **state)
{
    int action = LA_EXIT;

    lock_timeout = -1; // no timeout

    // Goal: lock open, drive off
    int lock_goal = 0x5; // LS_OPEN | LS_DRIVE_OFF

    // State: lock open, drive off
    uint32_t lock_state = 0x5;
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
    lock_goal = 0x6; // LS_CLOSED | LS_DRIVE_OFF

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
    // State: el axis is lockable, but lock is not closed and drive is off
    lock_state = LS_EL_OK | LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXTEND);
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
    lock_goal = 0x4;

    // State: drive already off
    lock_state = LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_EXIT);
    // State: drive not off yet
    lock_state = !LS_DRIVE_OFF;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);

    // Timeout case: should always stop
    lock_timeout = 0;
    action = GetLockAction(lock_state, lock_timeout, &lock_goal);
    assert_int_equal(action, LA_STOP);
}

// void test_DoLockSelectAction_LockOpenDriveOff(void **state)
// {
    // ExpectReturnDoLock();

    // lock_timeout = -1;

    // lock_data.state = LS_DRIVE_UNK;
    // CommandData.actbus.lock_goal = 0x5; // LS_OPEN | LS_DRIVE_OFF

    // assert_int_equal(DoLock(), LA_EXIT);
// }

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup_teardown(test_GetLockADCsReturn, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_GetLockADCs, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateNotInCharge, SetupEzBus, TearDownEzBus),
        cmocka_unit_test_setup_teardown(test_SetLockStateInCharge, SetupEzBus, TearDownEzBus),
        // cmocka_unit_test(test_DoLockFixWeirdStates), // TODO(evanmayer) separate logic piece, less critical
        cmocka_unit_test(test_GetLockAction),
        // cmocka_unit_test(test_DoLockSelectAction_LockOpenDriveOff),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
