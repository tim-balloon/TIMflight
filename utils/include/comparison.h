/**
 * @file utils.h
 * @brief General utility functions.
 */
#ifndef TIMFLIGHT_UTILS_COMPARISON_H
#define TIMFLIGHT_UTILS_COMPARISON_H

/**
 * @fn int relatively_equal(double a, double b, double maxreldiff)
 * @brief Safely compares floating-point numbers.
 *
 * Function compares two floating-point numbers by calculating their relative
 * difference. If the relative difference is less than the given threshold,
 * the numbers are considered to be the same.
 *
 * @param x the first number to compare
 * @param y the second number to compare
 * @param reldiff maximal relative difference between two (almost) equal numbers
 * @return a positive integer if numbers are equal, 0 otherwise
 *
 * @note More information regarding comparing floating-point numbers
 * accurately can be found at the following websites:
 *   - <a href="https://floating-point-gui.de/"></a>
 *   - <a href=https://bitbashing.io/comparing-floats.html"></a>
 */
int relatively_equal(double a, double b, double maxreldiff);

/**
 * @fn int almost_equal(double a, double b, int maxulps)
 * @brief Safely compares floating-point numbers.
 *
 * Function compares two floating-point numbers by measuring their mutual
 * distance in the units of the least precision (ULP). Numbers whose mutual
 * distance is less than the provided threshold are considered to be the same.
 *
 * The threshold should be a small, positive integer.
 *
 * @param a the first number to compare
 * @param b the second number to compare
 * @param maxulps maximal distance between two (almost) equal numbers
 * @return a postive integer if numbers are equal, 0 otherwise
 *
 * @note More information regarding comparing floating-point numbers
 * accurately can be found at the following websites:
 *   - <a href=https://bitbashing.io/comparing-floats.html">this</a>
 *   - <a href="https://floating-point-gui.de/"></a>
 */
int almost_equal(double a, double b, int maxulps);

#endif /* TIMFLIGHT_UTILS_COMPARISON_H */
