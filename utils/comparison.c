/**
 * @file utils.c
 * @brief Definitions of general utility functions.
 *
 * This software is copyright (C) 2023 University of Arizona
 */
#include <math.h>
#include <stdlib.h>
#include "include/comparison.h"


int is_relatively_equal(double a, double b, double maxrelerr)
{
    if (a == b)
        return 1;
    if (isnan(a) || isnan(b))
        return 0;
    if (isinf(a) || isinf(b))
        return 0;
    double relerr = fabs(b) > fabs(a) ? fabs((a - b) / b) : fabs((a - b) / a);
    if (relerr <= maxrelerr)
        return 1;
    return 0;
}

int is_almost_equal(double a, double b, const int maxulps)
{
    if (a == b)
        return 1;
    if (isnan(a) || isnan(b))
        return 0;
    if (isinf(a) || isinf(b))
        return 0;
    int idiff = abs(*((int *) &a) - *((int *) &b));
    if (idiff <= maxulps)
        return 1;
    return 0;
}
