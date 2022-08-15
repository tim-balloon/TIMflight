#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <cmocka.h>
#include "pointing.c"


void test_AzElTrim(void **state)
{
    AzElTrim(0., 0.);

    assert_float_equal(NewAzEl.az, 0., 1e-9);
    assert_float_equal(NewAzEl.el, 0., 1e-9);
}


int main(void)
{
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_AzElTrim),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
