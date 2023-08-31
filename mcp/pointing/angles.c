/**
 * @file angles.c
 *
 * @date Aug 5, 2015
 * @author seth
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
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

#include "angles.h"
#include <conversions.h>
#include <math.h>
#include <float.h>
#include <time.h>


/**
 * @brief Calculates the angular distance between two points on the celestial sphere
 * 
 * @param ra0 RA of the first point
 * @param dec0 Declination of the first point
 * @param ra1 RA of the second point
 * @param dec1 Declination of the second point
 * @return double angular distance between the two points
 */
double angular_distance(double ra0, double dec0, double ra1, double dec1)
{
    return acos(sin(dec0)*sin(dec1) + cos(dec0)*cos(dec1)*cos(ra0-ra1));
}


/**
 * @brief Approximates the azimuth from the cross elevation and elevation of the telescope.
 * Apparently, xel = az*cos(el)
 * 
 * @param cross_el Cross elevation angle of the telescope
 * @param el Elevation angle of the telescope
 * @return double approximate azimuth
 */
double approximate_az_from_cross_el(double cross_el, double el)
{
    if (el >= from_degrees(0.0) && el < from_degrees(85.0)) {
        return cross_el / cos(el);
    }
    return cross_el;
}


/**
 * @brief Wraps an angle to between 0 and the maximum value. 
 * Basically a version of like angle modulo.
 * 
 * @param angle Angle to wrap
 * @param max Maximum value of the wrap
 * @return double wrapped angle
 */
double wrap_to(double angle, double max)
{
    angle = remainder(angle, max);
    if (angle < 0) {
        angle += max;
    }
    return angle;
}


/**
 * @brief discretizing angles to integer values, this wraps
 * the angle to the range of [0, max]
 * 
 * @param angle integer rounded/reported angle
 * @param max Maximum allowed angle
 * @return int wrapped angle
 */
int wrap_to_ints(int angle, int max)
{
    angle = remainder(angle, max);
    if (angle < 0) {
        angle += max;
    }
    return angle;
}


/**
 * @brief Compares the number stored in value to the minimum
 * and maximum values. If it is under the minimum, the value is 
 * set to minimum and True is returned. If it is above the maximum
 * the value is set to maximum and True is returned. The returned
 * value is set to False if the passed in value is within the limits.
 * 
 * @param value pointer to an integer value
 * @param min minimum value to check against
 * @param max maximum value to check against
 * @return true if either condition is met
 * @return false if no condition is met
 */
bool limit_value_to_ints(int* value, int min, int max)
{
    if (*value < min) {
        *value = min;
        return true;
    }
    if (*value > max) {
        *value = max;
        return true;
    }
    return false;
}


/**
 * @brief Compares the number stored in value to the minimum
 * and maximum values. If it is under the minimum, the value is 
 * set to minimum and True is returned. If it is above the maximum
 * the value is set to maximum and True is returned. The returned
 * value is set to False if the passed in value is within the limits.
 * 
 * @param value pointer to a double to compare
 * @param min minimum allowed value to be compared against
 * @param max maximum allowed value to be compared against
 * @return true if the double stored in value is out of range
 * @return false otherwise
 */
bool limit_value_to(double* value, double min, double max)
{
    if (*value < min) {
        *value = min;
        return true;
    }
    if (*value > max) {
        *value = max;
        return true;
    }
    return false;
}


/**
 * @brief "unwinds" an angle to be +/- 180 degrees of the reference
 * 
 * @param reference zero point of the unwinding
 * @param angle angle to convert to +/- 180 of reference
 * @return double unwound angle
 */
double unwind_around(double reference, double angle)
{
    // Adjust angle to be +/- 180 of the reference.
    return (reference + remainder(angle - reference, 360.0));
}


/**
 * @brief Takes an arbitrary angle and modifies it to 
 * be between 0 and 360 degrees. Does not care about
 * winding number or directionality.
 * 
 * @param m_angle angle to convert to 0-360
 * @return double limited to 0 to 360.0 degrees
 */
double normalize_angle_360(double m_angle)
{
    while (m_angle < 0.0) {
        m_angle += 360.0;
    }
    while (m_angle >= 360.0) {
        m_angle -= 360.0;
    }
    return m_angle;
}


/**
 * @brief Takes an arbitrary angle and converts it to
 * be in the range of -180 to 180 degrees. This removes 
 * winding number while retaining left or right rotations.
 * 
 * @param m_angle angle to convert to +/- 180
 * @return double new angle.
 */
double normalize_angle_180(double m_angle)
{
    while (m_angle < -180.0) {
        m_angle += 360.0;
    }
    while (m_angle >= 180.0) {
        m_angle -= 360.0;
    }
    return m_angle;
}


/**
 * @brief Converts equatorial coordinates (RA, DEC) to horizontal coordinates (Az, El).
 * 
 * @param ra_hours Input right ascension
 * @param dec_deg Input declination
 * @param lst_s Input local sidereal time
 * @param lat_deg Input latitude
 * @param az_deg Output pointer to store azimuth
 * @param el_deg Output pointer to store elevation
 */
void equatorial_to_horizontal(double ra_hours, double dec_deg, time_t lst_s, double lat_deg,
                              double* az_deg, double* el_deg)
{
    double ha = from_seconds(lst_s) - from_hours(ra_hours);

    double sh, ch, sd, cd, sp, cp, x, y, z, r, a;

    sincos(ha, &sh, &ch);
    sincos(from_degrees(dec_deg), &sd, &cd);
    sincos(from_degrees(lat_deg), &sp, &cp);

    /* Az,El as x,y,z */
    x = -ch * cd * sp + sd * cp;
    y = -sh * cd;
    z = ch * cd * cp + sd * sp;

    /* To spherical */
    r = sqrt(x * x + y * y);
    a = (fabs(r) < DBL_MIN) ? 0.0 : atan2(y, x);

    *az_deg = to_degrees((a < 0.0) ? a + (2.0 * M_PI) : a);
    *el_deg = to_degrees(atan2(z, r));
}


/**
 * @brief Converts horizontal coordinates (Az, El) to equatorial coordinates (RA, DEC).
 * 
 * @param az_deg Input azimuth angle
 * @param el_deg Input elevation angle
 * @param lst_s Input local sidereal time
 * @param lat_deg Input latitude
 * @param ra_hours Output pointer to store the Right Ascension
 * @param dec_deg Output pointer to store the Declination
 */
void horizontal_to_equatorial(double az_deg, double el_deg, time_t lst_s, double lat_deg,
                              double* ra_hours, double* dec_deg)
{
    double ha = 0.0;
    double dec = 0.0;

    double sa, ca, se, ce, sp, cp, x, y, z, r;

    sincos(from_degrees(az_deg), &sa, &ca);
    sincos(from_degrees(el_deg), &se, &ce);
    sincos(from_degrees(lat_deg), &sp, &cp);

    /* HA,Dec as x,y,z */
    x = -ca * ce * sp + se * cp;
    y = -sa * ce;
    z = ca * ce * cp + se * sp;

    /* To spherical */
    r = sqrt(x * x + y * y);
    ha = (r == 0.0) ? 0.0 : atan2(y, x);
    dec = atan2(z, r);

    *ra_hours = to_hours(from_seconds(lst_s) - ha);
    *ra_hours = wrap_to(*ra_hours, 24.0);
    *dec_deg = to_degrees(dec);
}

