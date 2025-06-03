/* 
 * cryo_tauhk.c: interface for cryo housekkeping via TauHK
 * 
 * This software  is copyright 
 *  (C) University of Pennsylvania, Philadelphia 2025
 *
 * This file is part of mcp, as used for the Terahertz Intensity Mapper (TIM).
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
 * Created on: June 3, 2025 by Shubh Agrawal
 */

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "cryo_tauhk.h"

extern int16_t InCharge;

/**
 * @brief Function that gets called in the mcp main thread, likely 1 Hz loop. This function
 * will first set up the pointers to the channels where the thermistor data needs to be unpacked to
 * during the first call and then grab the data from the labjack data structures. Subsequent calls skip
 * that overhead and solely grab the data and place it in the telemetry stream.
 * 
 */
void read_thermistors(void) {
    static int first_time = 1;
    static channel_t* therm_1_Addr, *therm_2_Addr, *therm_3_Addr, *therm_4_Addr;
    if (1) {
        if (first_time) {
            therm_1_Addr = channels_find_by_name("heater_ic_hsw_dac");
            therm_2_Addr = channels_find_by_name("heater_ic_hsw_pwm");
            therm_3_Addr = channels_find_by_name("heater_lw_fpa_dac");
            therm_4_Addr = channels_find_by_name("heater_lw_fpa_pwm");
        }
        SET_SCALED_VALUE(therm_1_Addr, 1.0);
        SET_SCALED_VALUE(therm_2_Addr, 2.0);
        SET_SCALED_VALUE(therm_3_Addr, 3.0);
        SET_SCALED_VALUE(therm_4_Addr, 4.0);
    }
}
