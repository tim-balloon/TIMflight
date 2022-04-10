/**
 * @file utils.c
 * @brief Definitions of general utility functions.
 */
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "include/comparison.h"


int relatively_equal(double a, double b, double maxrelerr)
{
    if (a == b)
        return 1;
    double relerr = fabs(b) > fabs(a) ? fabs((a - b) / b) : fabs((a - b) / a);
    if (relerr <= maxrelerr)
        return 1;
    return 0;
}

int almost_equal(double a, double b, int maxulps)
{
    if (a == b)
        return 1;
    int idiff = abs(*((int *) &a) - *((int *) &b))
    if (idiff <= maxulps)
        return 1;
    return 0;
}