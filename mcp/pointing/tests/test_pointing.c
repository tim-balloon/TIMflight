#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include <float.h>

#include "pointing.c"

#include "mcp_mock_decl.c"


/**
 * @brief Set up the structs for El Solution tests
 */
static int SetupElSolution(void **state)
{
    static struct ElSolutionStruct ElSol = {
        .angle = 45.,
        .variance = .5,
        .samp_weight = 1.,
        .sys_var = .5,
        .trim = 0.,
        .last_input = 45.,
        .gy_int = 0.,
        .offset_gy = 1.,
        .FC = 1.,
        .n_solutions = 0,
        .since_last = 0,
        .fs = NULL,
        .new_offset_ifel_gy = 0.,
        .int_ifel = 0.,
        .prev_sol_el = 45.
    };
    *state = &ElSol;
    return 0;
}

/**
 * @brief Tear down the structs for El Solution tests
 */
static int TearDownElSolution(void **state)
{
    return 0;
}

void test_AddElSolution(void **state)
{
    // Get fixture
    struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;

    // Fake result struct
    struct ElAttStruct ElAtt = {
        0.0, // el
        2.0, // offset_gy
        1.0 // weight
    };

    // "impossible" case: 100% sure of new data
    ElSol.variance = 0.;
    ElSol.sys_var = 0.;
    AddElSolution(&ElAtt, &ElSol, 0); // don't add offset
    assert_float_equal(ElAtt.el, 45., DBL_EPSILON);
    assert_float_equal(ElAtt.offset_gy, 2., DBL_EPSILON);
    assert_float_equal(ElAtt.weight, 1 + 1e30, DBL_EPSILON);

    // add offset
    SetupElSolution(state);
    AddElSolution(&ElAtt, &ElSol, 1); // incorporate gyro offset
    assert_float_equal(ElAtt.el, 45., DBL_EPSILON);
    assert_float_equal(ElAtt.offset_gy, 1.5, DBL_EPSILON);
    assert_float_equal(ElAtt.weight, 1 + 1e30, DBL_EPSILON);
}

void test_AddAzSolution(void **state)
{
    
}

void test_exponential_moving_average(void **state)
{
    // artificially construct a situation where the filtered value is 1.
    double running_avg = -6.108494293;
    double newval = 100.;
    double halflife = 10.;
    double ret = exponential_moving_average(running_avg, newval, halflife);

    assert_float_equal(ret, 1., DBL_EPSILON);
}

void test_SetRaDec(void **state)
{
    // "Simple" case: pointing straight up on the equator at hour angle = lst
    point_index = 1;
    PointingData[0].lst = 0.;
    PointingData[0].lat = 0.;
    NewAzEl.fresh = -1;

    SetRaDec(0., 0.);

    assert_float_equal(NewAzEl.az, 0., FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 90., FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);

    // Test dec: same thing, but point along axis of rotation
    PointingData[0].lst = 0.;
    PointingData[0].lat = 0.;
    NewAzEl.fresh = -1;

    SetRaDec(0., 90.);

    assert_float_equal(NewAzEl.az, 0., FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0., FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);

    // Test ra: same thing, but point along direction of rotation
    PointingData[0].lst = 0.;
    PointingData[0].lat = 0.;
    NewAzEl.fresh = -1;

    SetRaDec(-90., 0.);

    assert_float_equal(NewAzEl.az, 90., FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0., FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);
}

void test_set_position(void **state)
{
    SIPData.GPSpos.lat = 0;
    SIPData.GPSpos.lon = 0;

    set_position(40., 45.);

    assert_float_equal(SIPData.GPSpos.lat, 40., FLT_EPSILON);
    assert_float_equal(SIPData.GPSpos.lon, 45., FLT_EPSILON);
}

void test_SetTrimToSC(void **state)
{
    point_index = 1;
    // source, deg
    PointingData[0].xsc_az[0] = 45.;
    PointingData[0].xsc_el[0] = 45.;
    // dest struct
    NewAzEl.az = 0.;
    NewAzEl.el = 0.;
    NewAzEl.rate = 0.;
    NewAzEl.fresh = 0;

    SetTrimToSC(0);

    assert_float_equal(NewAzEl.az, 45., FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 45., FLT_EPSILON);
    assert_float_equal(NewAzEl.rate, 360., FLT_EPSILON);
    assert_int_equal(NewAzEl.fresh, 1);
}

void test_trim_xsc(void **state)
{
    point_index = 1;
    PointingData[0].el = 45.;
    // dest, deg
    PointingData[0].xsc_az[1] = 45.;
    PointingData[0].xsc_el[1] = 45.;
    // source, deg
    PointingData[0].xsc_az[0] = 0.;
    PointingData[0].xsc_el[0] = 0.;
    // results struct
    CommandData.XSC[1].el_trim = from_degrees(45.);
    CommandData.XSC[1].cross_el_trim = from_degrees(45. * cos(from_degrees(PointingData[0].el)));
    // trim 1 to 0
    trim_xsc(0);

    assert_float_equal(CommandData.XSC[1].el_trim, 0., FLT_EPSILON);
    assert_float_equal(CommandData.XSC[1].cross_el_trim, 0., FLT_EPSILON);
}

void test_AzElTrim(void **state)
{
    AzElTrim(0., 0.);

    assert_float_equal(NewAzEl.az, 0., FLT_EPSILON);
    assert_float_equal(NewAzEl.el, 0., FLT_EPSILON);
}

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
        // TODO(evanmayer): lots of good logic to test, but will change after new star camera integrated
        // cmocka_unit_test(test_XSCHasNewSolution),
        // cmocka_unit_test(test_EvolveXSCSolution), // TODO(evanmayer): big, complicated
        // cmocka_unit_test(test_EvolveElSolution), // TODO(evanmayer)
        cmocka_unit_test_setup_teardown(test_AddElSolution, SetupElSolution, TearDownElSolution),
        cmocka_unit_test(test_AddAzSolution), // TODO(evanmayer)
        // cmocka_unit_test(test_EvolveAzSolution), // TODO(evanmayer)
        // TODO(evanmayer): just unit conversions and data shuffling
        // cmocka_unit_test(test_xsc_calculate_full_pointing_estimated_location),
        // TODO(evanmayer): lots of good logic to test, but will change after new star camera integrated
        // cmocka_unit_test(test_AutoTrimToSC), // TODO(evanmayer)
        cmocka_unit_test(test_exponential_moving_average),
        // cmocka_unit_test(test_ReadICCPointing), // TODO(evanmayer)
        // cmocka_unit_test(test_Pointing), // TODO(evanmayer): Too big for UT, needs to be broken up
        cmocka_unit_test(test_SetRaDec),
        cmocka_unit_test(test_SetTrimToSC),
        cmocka_unit_test(test_trim_xsc),
        cmocka_unit_test(test_AzElTrim),
        cmocka_unit_test(test_ClearTrim),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
