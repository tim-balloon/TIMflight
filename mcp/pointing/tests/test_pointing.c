/**
 * @file test_pointing.c
 *
 * @date Jan 10, 2023
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

#include "pointing.c"

#include "mcp_mock_decl.c"


// ============================================================================
// Setup/teardown functions (text fixtures)
// ============================================================================
/**
 * @brief Set up the structs for El Solution tests
 */
static int SetupElSolution(void **state)
{
    *state = calloc(1, sizeof(struct ElSolutionStruct));
    const struct ElSolutionStruct ElSol = {
        .angle = 45.0,
        .variance = 0.5,
        .samp_weight = 1.0,
        .sys_var = 0.5,
        .trim = 0.0,
        .last_input = 45.0,
        .gy_int = 0.0,
        .offset_gy = 1.0,
        .FC = 1.0,
        .n_solutions = 0,
        .since_last = 0,
        .fs = (struct FirStruct *) balloc(fatal, sizeof(struct FirStruct)),
        .new_offset_ifel_gy = 0.0,
        .int_ifel = 0.0,
        .prev_sol_el = 45.0
    };
    init_fir(ElSol.fs, FIR_LENGTH, 0, 0);
    memcpy(*state, &ElSol, sizeof(struct ElSolutionStruct));
    return 0;
}

/**
 * @brief Tear down the structs for El Solution tests
 */
static int TearDownElSolution(void **state)
{
    free(*state);
    return 0;
}

/**
 * @brief Set up the structs for Az Solution tests
 */
static int SetupAzSolution(void **state)
{
    *state = calloc(1, sizeof(struct AzSolutionStruct));

    const struct AzSolutionStruct AzSol = {
        .angle = 45.0, // solution's current angle
        .variance = 0.5, // solution's current sample variance
        .samp_weight = 1.0, // sample weight per sample
        .sys_var = 0.5, // sytematic variance - can't do better than this
        .trim = 0.0, // externally set trim to solution
        .last_input = 45.0, // last good data point
        .ifroll_gy_int = 1.0, // integral of the gyro since the last solution
        .ifyaw_gy_int = 1.0, // integral of the gyro since the last solution
        .offset_ifroll_gy = 1.0, // offset associated with solution
        .offset_ifyaw_gy = 1.0,
        .FC = 1.0, // filter constant
        .n_solutions = 0, // number of angle inputs
        .since_last = 0,
        .fs2 = NULL,
        .fs3 = NULL,
        .new_offset_ifyaw_gy = 0.0,
        .new_offset_ifroll_gy = 0.0,
        .d_az = 0.0,
        .int_ifroll = 0.0,
        .int_ifyaw = 0.0,
        .prev_sol_az = 0.0,
    };
    memcpy(*state, &AzSol, sizeof(struct AzSolutionStruct));
    return 0;
}

/**
 * @brief Tear down the structs for Az Solution tests
 */
static int TearDownAzSolution(void **state)
{
    free(*state);
    return 0;
}

// ============================================================================
// Test functions
// ============================================================================
/**
 * @brief "impossible" case: 100% sure of new data
 */
void test_AddElSolution_noVar(void **state)
{
    // Get fixture: new measurement template
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;
    // Fake result struct
    struct ElAttStruct ElAtt;
    ElAtt.el = 0.0;
    ElAtt.offset_gy = 2.0;
    ElAtt.weight = 1.0;

    ElSol.variance = 0.0;
    ElSol.sys_var = 0.0;
    AddElSolution(&ElAtt, &ElSol, 0); // don't add offset
    assert_float_equal(ElAtt.el, 45.0, DBL_EPSILON); // el should be 100% new meas
    assert_float_equal(ElAtt.offset_gy, 2.0, DBL_EPSILON);
    assert_float_equal(ElAtt.weight, 1 + 1e30, DBL_EPSILON);
}

/**
 * @brief Incorporate gyro offset
 */
void test_AddElSolution_gyroOffset(void **state)
{
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;
    struct ElAttStruct ElAtt;
    ElAtt.el = 45.0;
    ElAtt.offset_gy = 2.0;
    ElAtt.weight = 1.0;

    AddElSolution(&ElAtt, &ElSol, 1);
    assert_float_equal(ElAtt.el, 45.0, DBL_EPSILON);
    assert_float_equal(ElAtt.offset_gy, 1.5, DBL_EPSILON);
    assert_float_equal(ElAtt.weight, 2.0, DBL_EPSILON);
}

/**
 * @brief Basic functionality: resultant el should be weighted by weights
 */
void test_AddElSolution_basic(void **state)
{
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;
    struct ElAttStruct ElAtt;
    ElAtt.el = 0.0;
    ElAtt.offset_gy = 2.0;
    ElAtt.weight = 1.0;

    AddElSolution(&ElAtt, &ElSol, 1);
    assert_float_equal(ElAtt.el, 45.0 / 2.0, DBL_EPSILON);
    assert_float_equal(ElAtt.offset_gy, 1.5, DBL_EPSILON);
    assert_float_equal(ElAtt.weight, 2.0, DBL_EPSILON);
}

/**
 * @brief "impossible" case: 100% sure of new data
 */
void test_AddAzSolution_noVar(void **state)
{
    // Get fixture: new measurement template
    struct AzSolutionStruct AzSol = *(struct AzSolutionStruct *)*state;
    // Fake result struct
    struct AzAttStruct AzAtt;
    AzAtt.az = 0.0;
    AzAtt.offset_ifroll_gy = 2.0;
    AzAtt.offset_ifyaw_gy = 2.0;
    AzAtt.weight = 1.0;

    AzSol.variance = 0.0;
    AzSol.sys_var = 0.0;
    AddAzSolution(&AzAtt, &AzSol, 0); // don't add offset
    assert_float_equal(AzAtt.az, 45.0, DBL_EPSILON); // az should be 100% new meas
    // offsets should remain the same
    assert_float_equal(AzAtt.offset_ifroll_gy, 2.0, DBL_EPSILON);
    assert_float_equal(AzAtt.offset_ifyaw_gy, 2.0, DBL_EPSILON);
    // weights should add
    assert_float_equal(AzAtt.weight, 1 + 1e30, DBL_EPSILON);
}

/**
 * @brief Incorporate gyro offset
 */
void test_AddAzSolution_gyroOffset(void **state)
{
    struct AzSolutionStruct AzSol = *(struct AzSolutionStruct *)*state;
    struct AzAttStruct AzAtt;
    AzAtt.az = 45.0;
    AzAtt.offset_ifroll_gy = 2.0;
    AzAtt.offset_ifyaw_gy = 2.0;
    AzAtt.weight = 1.0;

    // add offset
    AddAzSolution(&AzAtt, &AzSol, 1); // incorporate gyro offset
    assert_float_equal(AzAtt.az, 45.0, DBL_EPSILON);
    assert_float_equal(AzAtt.offset_ifroll_gy, 1.5, DBL_EPSILON);
    assert_float_equal(AzAtt.offset_ifyaw_gy, 1.5, DBL_EPSILON);
    assert_float_equal(AzAtt.weight, 2.0, DBL_EPSILON);
}

/**
 * @brief Basic functionality: resultant az should be weighted by weights
 */
void test_AddAzSolution_basic(void **state)
{
    struct AzSolutionStruct AzSol = *(struct AzSolutionStruct *)*state;
    struct AzAttStruct AzAtt;
    AzAtt.az = 0.0;
    AzAtt.offset_ifroll_gy = 2.0;
    AzAtt.offset_ifyaw_gy = 2.0;
    AzAtt.weight = 1.0;

    AddAzSolution(&AzAtt, &AzSol, 1);
    assert_float_equal(AzAtt.az, 45.0 / 2.0, DBL_EPSILON);
    assert_float_equal(AzAtt.offset_ifroll_gy, 1.5, DBL_EPSILON);
    assert_float_equal(AzAtt.offset_ifyaw_gy, 1.5, DBL_EPSILON);
    assert_float_equal(AzAtt.weight, 2.0, DBL_EPSILON);
}

/**
 * @brief Basic functionality: use gyro data to evolve an El solution
 */
void test_EvolveElSolution_basic(void **state)
{
    // Get fixture: new measurement template
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;

    // new data to use when evolving the solution struct
    double gyro = 1.0; // angular rate, deg/sec
    double gy_off = 1.0; // angular rate bias, deg/sec
    double new_angle = 50.0; // absolute reference angle
    int new_reading = 1; // evolve solution or fall through

    CommandData.pointing_mode.nw = 0;
    ElSol.n_solutions = 11;

    EvolveElSolution(&ElSol, gyro, gy_off, new_angle, new_reading);
    assert_float_equal(ElSol.angle, 46.680000, DBL_EPSILON);
    assert_float_equal(ElSol.variance, 0.33333364444429925, DBL_EPSILON);
    assert_float_equal(ElSol.samp_weight, 1.0, DBL_EPSILON);
    assert_float_equal(ElSol.sys_var, 0.5, DBL_EPSILON);
    assert_float_equal(ElSol.trim, 0.0, DBL_EPSILON);
    assert_float_equal(ElSol.last_input, 50.0, DBL_EPSILON);
    assert_float_equal(ElSol.gy_int, 0.0, DBL_EPSILON);
    assert_float_equal(ElSol.offset_gy, 0.0, DBL_EPSILON);
    assert_float_equal(ElSol.FC, 1.0, DBL_EPSILON);
    assert_int_equal(ElSol.since_last, 1);
    assert_float_equal(ElSol.new_offset_ifel_gy, 0.0, DBL_EPSILON); // 5 deg step is bunk
    assert_float_equal(ElSol.int_ifel, 1e-2, DBL_EPSILON);
    assert_float_equal(ElSol.prev_sol_el, 45.0, DBL_EPSILON);
    assert_int_equal(ElSol.n_solutions, 12);
}

/**
 * @brief Slew veto case
 */
void test_EvolveElSolution_slewVeto(void **state)
{
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;

    double gyro = 1.0; // angular rate, deg/sec
    double gy_off = 1.0; // angular rate bias, deg/sec
    double new_angle = 50.0; // absolute reference angle
    int new_reading = 1; // evolve solution or fall through

    CommandData.pointing_mode.nw = 1; // slew veto, apparently
    ElSol.n_solutions = 11;
    ElSol.new_offset_ifel_gy = 1.0;
    ElSol.offset_gy = 1.0;

    EvolveElSolution(&ElSol, gyro, gy_off, new_angle, new_reading);
    assert_float_equal(ElSol.angle, 46.680000, DBL_EPSILON);
    assert_float_equal(ElSol.variance, 0.33333364444429925, DBL_EPSILON);
    assert_float_equal(ElSol.samp_weight, 1.0, DBL_EPSILON);
    assert_float_equal(ElSol.sys_var, 0.5, DBL_EPSILON);
    assert_float_equal(ElSol.trim, 0.0, DBL_EPSILON);
    assert_float_equal(ElSol.last_input, 50.0, DBL_EPSILON);
    assert_float_equal(ElSol.gy_int, 0.0, DBL_EPSILON);
    assert_float_equal(ElSol.offset_gy, 1.0, DBL_EPSILON);
    assert_float_equal(ElSol.FC, 1.0, DBL_EPSILON);
    assert_int_equal(ElSol.since_last, 1);
    assert_float_equal(ElSol.new_offset_ifel_gy, 1.0, DBL_EPSILON); // 5 deg step is bunk
    assert_float_equal(ElSol.int_ifel, 1e-2, DBL_EPSILON);
    assert_float_equal(ElSol.prev_sol_el, 45.0, DBL_EPSILON);
    assert_int_equal(ElSol.n_solutions, 11);
}

/**
 * @brief Test exponential moving avg filter
 * @details Artificially construct a situation where the filtered value is 1.
 */
void test_exponential_moving_average(void **state)
{
    // (consider this "inverting" the filter)
    double running_avg = -6.108494293;
    double newval = 100.0;
    double halflife = 10.0;
    double ret = exponential_moving_average(running_avg, newval, halflife);

    assert_float_equal(ret, 1.0, DBL_EPSILON);
}

/**
 * @brief Test a few cases of setting El/Az via Ra/Dec inputs
 */
void test_SetRaDec(void **state)
{
    // "Simple" case: pointing straight up on the equator at hour angle = lst
    point_index = 1;
    PointingData[0].lst = 0.0;
    PointingData[0].lat = 0.0;
    NewAzEl.fresh = -1;

    SetRaDec(0.0, 0.0);

    assert_float_equal(NewAzEl.az, 0.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 90.0, FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);

    // Test dec: same thing, but point along axis of rotation
    PointingData[0].lst = 0.0;
    PointingData[0].lat = 0.0;
    NewAzEl.fresh = -1;

    SetRaDec(0.0, 90.0);

    assert_float_equal(NewAzEl.az, 0.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0.0, FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);

    // Test ra: same thing, but point along direction of rotation
    PointingData[0].lst = 0.0;
    PointingData[0].lat = 0.0;
    NewAzEl.fresh = -1;

    SetRaDec(-90.0, 0.0);

    assert_float_equal(NewAzEl.az, 90.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0.0, FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);
}

/**
 * @brief Test externally setting the GPS lat/lon
 */
void test_set_position(void **state)
{
    SIPData.GPSpos.lat = 0;
    SIPData.GPSpos.lon = 0;

    set_position(40.0, 45.0);

    assert_float_equal(SIPData.GPSpos.lat, 40.0, FLT_EPSILON);
    assert_float_equal(SIPData.GPSpos.lon, 45.0, FLT_EPSILON);
}

/**
 * @brief Test setting trim values to those from star camera data
 */
void test_SetTrimToSC(void **state)
{
    point_index = 1;
    // source, deg
    PointingData[0].xsc_az[0] = 45.0;
    PointingData[0].xsc_el[0] = 45.0;
    // dest struct
    NewAzEl.az = 0.0;
    NewAzEl.el = 0.0;
    NewAzEl.rate = 0.0;
    NewAzEl.fresh = 0;

    SetTrimToSC(0);

    assert_float_equal(NewAzEl.az, 45.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 45.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.rate, 360.0, FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);
}

/**
 * @brief Test setting one star camera's trim values off of another's
 */
void test_trim_xsc(void **state)
{
    point_index = 1;
    PointingData[0].el = 45.0;
    // dest, deg
    PointingData[0].xsc_az[1] = 45.0;
    PointingData[0].xsc_el[1] = 45.0;
    // source, deg
    PointingData[0].xsc_az[0] = 0.0;
    PointingData[0].xsc_el[0] = 0.0;
    // results struct
    CommandData.XSC[1].el_trim = from_degrees(45.0);
    CommandData.XSC[1].cross_el_trim = from_degrees(45. * cos(from_degrees(PointingData[0].el)));
    // trim 1 to 0
    trim_xsc(0);

    assert_float_equal(CommandData.XSC[1].el_trim, 0.0, FLT_EPSILON);
    assert_float_equal(CommandData.XSC[1].cross_el_trim, 0.0, FLT_EPSILON);
}

/**
 * @brief Test setting Az/El trim values manually
 */
void test_AzElTrim(void **state)
{
    AzElTrim(0.0, 0.0);

    assert_float_equal(NewAzEl.az, 0.0, FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0.0, FLT_EPSILON);
}

/**
 * @brief Test clearing the Az/El trim values
 */
void test_ClearTrim(void **state)
{
    ClearTrim();
    assert_int_equal(NewAzEl.fresh, -1);
}


int main(void)
{
    const struct CMUnitTest tests[] = {
        // cmocka_unit_test(test_MagConvert), // TODO(evanmayer): hard to write test for manually - data lookup
        // cmocka_unit_test(test_PSSConvert), // TODO(evanmayer): big function, may need to be broken up
        // cmocka_unit_test(test_record_gyro_history), // TODO(evanmayer): not much going on, low priority
        // cmocka_unit_test(test_XSCHasNewSolution), // TODO(evanmayer): save until after new star cameras integrated
        // cmocka_unit_test(test_EvolveXSCSolution), // TODO(evanmayer): big, complicated, high priority
        cmocka_unit_test_setup_teardown(test_AddElSolution_noVar, SetupElSolution, TearDownElSolution),
        cmocka_unit_test_setup_teardown(test_AddElSolution_gyroOffset, SetupElSolution, TearDownElSolution),
        cmocka_unit_test_setup_teardown(test_AddElSolution_basic, SetupElSolution, TearDownElSolution),
        cmocka_unit_test_setup_teardown(test_AddAzSolution_noVar, SetupAzSolution, TearDownAzSolution),
        cmocka_unit_test_setup_teardown(test_AddAzSolution_gyroOffset, SetupAzSolution, TearDownAzSolution),
        cmocka_unit_test_setup_teardown(test_AddAzSolution_basic, SetupAzSolution, TearDownAzSolution),
        cmocka_unit_test_setup_teardown(test_EvolveElSolution_basic, SetupElSolution, TearDownElSolution),
        cmocka_unit_test_setup_teardown(test_EvolveElSolution_slewVeto, SetupElSolution, TearDownElSolution),
        // cmocka_unit_test(test_EvolveAzSolution), // TODO(evanmayer)
        // cmocka_unit_test(test_xsc_calculate_full_pointing_estimated_location), // just unit conversions and data
        // shuffling
        // cmocka_unit_test(test_AutoTrimToSC), // TODO(evanmayer): save until after new star cameras integrated
        cmocka_unit_test(test_exponential_moving_average),
        // cmocka_unit_test(test_ReadICCPointing), // TODO(evanmayer): not much going on, low priority
        // cmocka_unit_test(test_Pointing), // TODO(evanmayer): Too big for UT, needs to be broken up
        cmocka_unit_test(test_SetRaDec),
        cmocka_unit_test(test_set_position),
        cmocka_unit_test(test_SetTrimToSC),
        cmocka_unit_test(test_trim_xsc),
        cmocka_unit_test(test_AzElTrim),
        cmocka_unit_test(test_ClearTrim),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
