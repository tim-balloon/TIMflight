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
 * @brief "impossible" case: 100% sure of new data
 */
// void test_AddElSolution_noVar(void **state)
// {
//         // Get fixture: new measurement template
//     struct ElSolutionStruct ElSol = *(struct ElSolutionStruct *)*state;
//     // Fake result struct
//     struct ElAttStruct ElAtt;
//     ElAtt.el = 0.0;
//     ElAtt.offset_gy = 2.0;
//     ElAtt.weight = 1.0;

//     ElSol.variance = 0.0;
//     ElSol.sys_var = 0.0;
//     AddElSolution(&ElAtt, &ElSol, 0); // don't add offset
//     assert_float_equal(ElAtt.el, 45.0, DBL_EPSILON); // el should be 100% new meas
//     assert_float_equal(ElAtt.offset_gy, 2.0, DBL_EPSILON);
//     assert_float_equal(ElAtt.weight, 1 + 1e30, DBL_EPSILON);
// }


/**
 * @brief Test clearing the Az/El trim values
 */
// void test_ClearTrim(void **state)
// {
//     ClearTrim();
//     assert_int_equal(NewAzEl.fresh, -1);
// }


int main(void)
{
    const struct CMUnitTest tests[] = {
        // cmocka_unit_test_setup_teardown(test_AddElSolution_noVar, SetupElSolution, TearDownElSolution),
        // cmocka_unit_test(test_ClearTrim),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
