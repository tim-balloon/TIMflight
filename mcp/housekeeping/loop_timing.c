/* 
 * loop_timing.c:
 *
 * This software is copyright (C) 2024 Evan Mayer
 *
 * This file is part of mcp, created for the TIM Balloon Project.
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
 * Created on: Aug 28, 2024 by Evan Mayer
 */

#include "loop_timing.h"

extern int16_t SouthIAm;

/**
 * @brief Initializes variables and structs for tracking loop timing.
 * @return int 0 if successful, return code of `clock_gettime` if that call is
 * not.
 */
int init_loop_timing(void)
{
    int error_code = 0;

    lastNsec_1hz = 0;
    lastSec_1hz = 0;
    deltaTimeSec_1hz = 0.0;

    lastNsec_2hz = 0;
    lastSec_2hz = 0;
    deltaTimeSec_2hz = 0.0;

    lastNsec_5hz = 0;
    lastSec_5hz = 0;
    deltaTimeSec_5hz = 0.0;

    lastNsec_20hz = 0;
    lastSec_20hz = 0;
    deltaTimeSec_20hz = 0.0;

    lastNsec_80hz = 0;
    lastSec_80hz = 0;
    deltaTimeSec_80hz = 0.0;

    lastNsec_100hz = 0;
    lastSec_100hz = 0;
    deltaTimeSec_100hz = 0.0;

    lastNsec_122hz = 0;
    lastSec_122hz = 0;
    deltaTimeSec_122hz = 0.0;

    lastNsec_200hz = 0;
    lastSec_200hz = 0;
    deltaTimeSec_200hz = 0.0;

    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_1hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_2hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_5hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_20hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_80hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_100hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_122hz)) < 0) {
        return error_code;
    }
    if ((error_code = clock_gettime(CLOCK_REALTIME, &end_200hz)) < 0) {
        return error_code;
    }
    return error_code;
}

/**
 * @brief Query the system clock, calculate time deltas, and send to telemetry.
 * Shall be called at the end of each loop, before `share_data` and
 * `add_frame_to_superframe`.
 * @details Contains a system call to clock_getttime for the realtime clock,
 * but otherwise should be lightweight
 * @param caller 
 */
void record_loop_timing(E_RATE caller)
{
    static bool first_time = true;
    static channel_t* delta_1_hz_addr;
    static channel_t* delta_2_hz_addr;
    static channel_t* delta_5_hz_addr;
    static channel_t* delta_20_hz_addr;
    static channel_t* delta_80_hz_addr;
    static channel_t* delta_100_hz_addr;
    static channel_t* delta_122_hz_addr;
    static channel_t* delta_200_hz_addr;

    if (first_time) {
        if (SouthIAm) {
            delta_1_hz_addr = channels_find_by_name("delta_t_1hz_s");
            delta_2_hz_addr = channels_find_by_name("delta_t_2hz_s");
            delta_5_hz_addr = channels_find_by_name("delta_t_5hz_s");
            delta_20_hz_addr = channels_find_by_name("delta_t_20hz_s");
            delta_80_hz_addr = channels_find_by_name("delta_t_80hz_s");
            delta_100_hz_addr = channels_find_by_name("delta_t_100hz_s");
            delta_122_hz_addr = channels_find_by_name("delta_t_122hz_s");
            delta_200_hz_addr = channels_find_by_name("delta_t_200hz_s");
        } else {
            delta_1_hz_addr = channels_find_by_name("delta_t_1hz_n");
            delta_2_hz_addr = channels_find_by_name("delta_t_2hz_n");
            delta_5_hz_addr = channels_find_by_name("delta_t_5hz_n");
            delta_20_hz_addr = channels_find_by_name("delta_t_20hz_n");
            delta_80_hz_addr = channels_find_by_name("delta_t_80hz_n");
            delta_100_hz_addr = channels_find_by_name("delta_t_100hz_n");
            delta_122_hz_addr = channels_find_by_name("delta_t_122hz_n");
            delta_200_hz_addr = channels_find_by_name("delta_t_200hz_n");
        }
        first_time = false;
    }

    switch (caller) {
        case RATE_200HZ:
            clock_gettime(CLOCK_REALTIME, &end_200hz);
            deltaTimeSec_200hz = (end_200hz.tv_sec - lastSec_200hz) + (end_200hz.tv_nsec - lastNsec_200hz) / 1e9;
            lastSec_200hz = end_200hz.tv_sec;
            lastNsec_200hz = end_200hz.tv_nsec;
            SET_SCALED_VALUE(delta_200_hz_addr, deltaTimeSec_200hz);
            break;
        case RATE_122HZ:
            clock_gettime(CLOCK_REALTIME, &end_122hz);
            deltaTimeSec_122hz = (end_122hz.tv_sec - lastSec_122hz) + (end_122hz.tv_nsec - lastNsec_122hz) / 1e9;
            lastSec_122hz = end_122hz.tv_sec;
            lastNsec_122hz = end_122hz.tv_nsec;
            SET_SCALED_VALUE(delta_122_hz_addr, deltaTimeSec_122hz);
            break;
        case RATE_100HZ:
            clock_gettime(CLOCK_REALTIME, &end_100hz);
            deltaTimeSec_100hz = (end_100hz.tv_sec - lastSec_100hz) + (end_100hz.tv_nsec - lastNsec_100hz) / 1e9;
            lastSec_100hz = end_100hz.tv_sec;
            lastNsec_100hz = end_100hz.tv_nsec;
            SET_SCALED_VALUE(delta_100_hz_addr, deltaTimeSec_100hz);
            break;
        case RATE_80HZ:
            clock_gettime(CLOCK_REALTIME, &end_80hz);
            deltaTimeSec_80hz = (end_80hz.tv_sec - lastSec_80hz) + (end_80hz.tv_nsec - lastNsec_80hz) / 1e9;
            lastSec_80hz = end_80hz.tv_sec;
            lastNsec_80hz = end_80hz.tv_nsec;
            SET_SCALED_VALUE(delta_80_hz_addr, deltaTimeSec_80hz);
            break;
        case RATE_20HZ:
            clock_gettime(CLOCK_REALTIME, &end_20hz);
            deltaTimeSec_20hz = (end_20hz.tv_sec - lastSec_20hz) + (end_20hz.tv_nsec - lastNsec_20hz) / 1e9;
            lastSec_20hz = end_20hz.tv_sec;
            lastNsec_20hz = end_20hz.tv_nsec;
            SET_SCALED_VALUE(delta_20_hz_addr, deltaTimeSec_20hz);
            break;
        case RATE_5HZ:
            clock_gettime(CLOCK_REALTIME, &end_5hz);
            deltaTimeSec_5hz = (end_5hz.tv_sec - lastSec_5hz) + (end_5hz.tv_nsec - lastNsec_5hz) / 1e9;
            lastSec_5hz = end_5hz.tv_sec;
            lastNsec_5hz = end_5hz.tv_nsec;
            SET_SCALED_VALUE(delta_5_hz_addr, deltaTimeSec_5hz);
            break;
        case RATE_2HZ:
            clock_gettime(CLOCK_REALTIME, &end_2hz);
            deltaTimeSec_2hz = (end_2hz.tv_sec - lastSec_2hz) + (end_2hz.tv_nsec - lastNsec_2hz) / 1e9;
            lastSec_2hz = end_2hz.tv_sec;
            lastNsec_2hz = end_2hz.tv_nsec;
            SET_SCALED_VALUE(delta_2_hz_addr, deltaTimeSec_2hz);
            break;
        case RATE_1HZ:
            clock_gettime(CLOCK_REALTIME, &end_1hz);
            deltaTimeSec_1hz = (end_1hz.tv_sec - lastSec_1hz) + (end_1hz.tv_nsec - lastNsec_1hz) / 1e9;
            lastSec_1hz = end_1hz.tv_sec;
            lastNsec_1hz = end_1hz.tv_nsec;
            SET_SCALED_VALUE(delta_1_hz_addr, deltaTimeSec_1hz);
            break;
        default:
            break;
    }
}
