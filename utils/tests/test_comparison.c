#include <math.h>

#include <unity.h>

#include "comparison.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_relative_equality(void) {
    double x = 1.0;
    double y = cos(0.0);
    double eps = 1.0e-8;
    TEST_ASSERT_EQUAL_INT(1, test_relative_equality(x, y, eps))
}

void test_relative_inequality(void) {
    double x = 1.0;
    double y = cos(0.0);
    double eps = 1.0e-15;
    TEST_ASSERT_EQUAL_INT(0, test_relative_equality(x, y, eps))
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_relative_equality);
    RUN_TEST(test_relative_inequality);
    return UNITY_END();
}


