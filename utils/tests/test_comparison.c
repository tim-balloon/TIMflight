#include <math.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include "../include/comparison.h"


void test_relative_equality(void **state)
{
    double x = 1.0;
    double y = 0.999;
    double eps = 0.01;
    assert_int_equal(1, is_relatively_equal(x, y, eps));
}

void test_relative_inequality(void **state)
{
    double x = 1.0;
    double y = 0.9;
    double eps = 0.01;
    assert_int_equal(0, is_relatively_equal(x, y, eps));
}

void test_relative_inequality_a_inf(void **state)
{
    double x = INFINITY;
    double y = 1.0;
    double eps = 0.01;
    assert_int_equal(0, is_relatively_equal(x, y, eps));
}

void test_relative_inequality_b_inf(void **state)
{
    double x = 1.0;
    double y = INFINITY;
    double eps = 0.01;
    assert_int_equal(0, is_relatively_equal(x, y, eps));
}

void test_relative_inequality_a_nan(void **state)
{
    double x = NAN;
    double y = 1.0;
    double eps = 0.01;
    assert_int_equal(0, is_relatively_equal(x, y, eps));
}

void test_relative_inequality_b_nan(void **state)
{
    double x = 1.0;
    double y = NAN;
    double eps = 1.0e-8;
    assert_int_equal(0, is_relatively_equal(x, y, eps));
}

void test_almost_equality(void **state)
{
    int maxulps = 3;
    double x = 0.0;
    double y = x;
    for (int i = 0; i < maxulps - 1; i++)
        y = nextafter(y, 1.0);
    assert_int_equal(1, is_almost_equal(x, y, maxulps));
}

void test_almost_inequality(void **state)
{
    int maxulps = 3;
    double x = 0.0;
    double y = x;
    for (int i = 0; i < maxulps + 1; i++)
        y = nextafter(y, 1.0);
    assert_int_equal(0, is_almost_equal(x, y, maxulps));
}

void test_almost_inequality_a_inf(void **state)
{
    int maxulps = 3;
    double x = INFINITY;
    double y = 1.0;
    assert_int_equal(0, is_almost_equal(x, y, maxulps));
}

void test_almost_inequality_b_inf(void **state)
{
    int maxulps = 3;
    double x = 1.0;
    double y = INFINITY;
    assert_int_equal(0, is_almost_equal(x, y, maxulps));
}

void test_almost_inequality_a_nan(void **state)
{
    int maxulps = 3;
    double x = NAN;
    double y = 1.0;
    assert_int_equal(0, is_almost_equal(x, y, maxulps));
}

void test_almost_inequality_b_nan(void **state)
{
    int maxulps = 3;
    double x = 1.0;
    double y = NAN;
    assert_int_equal(0, is_almost_equal(x, y, maxulps));
}

int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_relative_equality),
        cmocka_unit_test(test_relative_inequality),
        cmocka_unit_test(test_relative_inequality_a_inf),
        cmocka_unit_test(test_relative_inequality_b_inf),
        cmocka_unit_test(test_relative_inequality_a_nan),
        cmocka_unit_test(test_relative_inequality_b_nan),
        cmocka_unit_test(test_almost_equality),
        cmocka_unit_test(test_almost_inequality),
        cmocka_unit_test(test_almost_inequality_a_inf),
        cmocka_unit_test(test_almost_inequality_b_inf),
        cmocka_unit_test(test_almost_inequality_a_nan),
        cmocka_unit_test(test_almost_inequality_b_nan),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
