/* 
 * pointing.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 23, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_POINTING_H_
#define INCLUDE_POINTING_H_

#include "channels_tng.h"

// size of the gyro ring buffer, age is in centiseconds
#define GY_HISTORY_AGE_CS 4400

// nominal "at float" altitude = 100 kft
#define FLOAT_ALT 30480
// number of frames to wait once we pass that float altitude before saying OK to go
#define FRAMES_TO_OK_ATFLOAT 100

#define OFFSET_GY_IFEL   (0)
#define OFFSET_GY_IFROLL (0)
#define OFFSET_GY_IFYAW  (0)

// length of the FIR pointing filter (30 minutes)
#define FIR_LENGTH (60*30 * SR)
// length of the GPS FIR filter
#define GPS_FIR_LENGTH (60*30 * 1)

/* Calibrations of the az of each sensor  */
#define PSS_ALIGNMENT     0.0
// PSS azimuth in deg
#define PSS_AZ_SEPARATION 21.30
#define PSS_AZ_OFFSET 15.466
#define PSS0_BETA (180 + PSS_AZ_OFFSET)
#define PSS1_BETA (PSS0_BETA + 2 * PSS_AZ_SEPARATION)
#define PSS2_BETA (PSS1_BETA + 2 * PSS_AZ_SEPARATION)
#define PSS3_BETA (PSS2_BETA + 2 * PSS_AZ_SEPARATION)
// TODO(evanmayer): science flight shall have 8
// #define PSS4_BETA
// #define PSS5_BETA
// #define PSS6_BETA
// #define PSS7_BETA
// #define PSS8_BETA

// PSS elevation in deg
#define PSS0_ALPHA 25.0
#define PSS1_ALPHA 25.0
#define PSS2_ALPHA 25.0
#define PSS3_ALPHA 25.0
// #define PSS4_ALPHA 25.0
// #define PSS5_ALPHA 25.0
// #define PSS6_ALPHA 25.0
// #define PSS7_ALPHA 25.0

// PSS roll in deg
#define PSS0_PSI 180.0
#define PSS1_PSI 180.0
#define PSS2_PSI 180.0
#define PSS3_PSI 180.0
// #define PSS4_PSI 180.0
// #define PSS5_PSI 180.0
// #define PSS6_PSI 180.0
// #define PSS7_PSI 180.0

// distance from pinhole to sensor in mm
// before dance floor tests changed this from 10.12 mm to 8.10 mm
// because previous outside tests gave 8 deg for a 10 deg az change
#define PSS0_D 8.10
#define PSS1_D 8.10
#define PSS2_D 8.10
#define PSS3_D 8.10
// #define PSS4_D 8.10
// #define PSS5_D 8.10
// #define PSS6_D 8.10
// #define PSS7_D 8.10

#define PSS_L  10.0     // 10 mm = effective length of active area
#define PSS_IMAX  8192.  // Maximum current (place holder for now)
// TODO(Paul): PSS_NOISE based on outside tests with Giles at LDB,
// should change before flight based on desk/dance floor tests
// #define PSS_NOISE     0.2
#define PSS_XSTRETCH  1.  // 0.995
#define PSS_YSTRETCH  1.  // 1.008


#define NUM_CHARS_CHAN_P_ICC   128

/**
 * @brief read from the in charge computer channels
 * 
 */
typedef struct {
    void *pval;
    char ch_name[NUM_CHARS_CHAN_P_ICC];
    E_TYPE var_type;
    channel_t *ch;
} read_icc_t;

void set_position(double m_lat, double m_lon);
void SetRaDec(double ra, double dec);
void SetSafeDAz(double ref, double *A);
void UnwindDiff(double ref, double *A);
void trim_xsc(int);

#endif /* INCLUDE_POINTING_H_ */
