/**
 * @file inclinometer.h
 *
 * @date Nov 23, 2015

 * @author James & Juzz

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

#ifndef INCLUDE_INCLINOMETER_H_
#define INCLUDE_INCLINOMETER_H_

// #define DEBUG_INCLINOMETER

void initialize_inclinometer(void);
void *monitor_inclinometer(void *m_arg);

// TODO(anyone): Characterize Inclinometer
#define INCX_B 0.0
#define INCX_M 1.0
#define INCY_B 0.0
#define INCY_M 1.0

/* Inclinometer Az Calibration */
#define INC_ALIGNMENT     0.0

// convert inc readings to sine and cosine
// calibrated in Palestine, July 11, 2010
// Best fit to inc_x and inc_y
// y = -3000*sin(x-19)+33050 : inc_x
// y = 3000*cos(x-19)+33310 : inc_y
// x is dgps theta in degrees.
// The defines for x and y are no longer used.
// #define INCX_M (1.0)
// #define INCX_B (1.0)
// #define INCY_M (1.0)
// #define INCY_B (1.0)
#define INCZ_M (1.0)
#define INCZ_B (1.0)

#define FAST_INC

void store_1hz_inc(void);

#endif /* INCLUDE_INCLINOMETER_H_ */
