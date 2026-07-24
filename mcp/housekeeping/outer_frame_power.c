/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2002-2006 University of Toronto
 
 This file is part of mcp.
 
 mcp is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 at your option) any later version.
 
 mcp is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with mcp; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
 created by Ian Lowe 2-16-23
 **************************************************************************/


/*************************************************************************
 
 outer_frame_power.c -- mcp code to control power relays
 
 *************************************************************************/
#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "blast.h"
#include "multiplexed_labjack.h"
#include "outer_frame_power.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

/**
 * @brief holds the local state information for the outer frame power commands
 * 
 */
struct outer_frame_power {
    int relay_1_on, relay_1_off, relay_2_on, relay_2_off;
    int relay_3_on, relay_3_off, relay_4_on, relay_4_off;
    int relay_5_on, relay_5_off, relay_6_on, relay_6_off;
    int relay_7_on, relay_7_off, relay_8_on, relay_8_off;
    int relay_9_on, relay_9_off, relay_10_on, relay_10_off;
};

struct outer_frame_power of_pbob;


/**
 * @brief initializes the data channel pointers and stores the voltage monitoring data.
 * 
 */
static void read_of_vm(void) {
    static int first_time = 1;
    // declare pointers to the data channels
    static channel_t * of_vm1_Addr;
    static channel_t * of_vm2_Addr;
    static channel_t * of_vm3_Addr;
    if (first_time) {
        first_time = 0;
        // now link the pointers to their approrpriate channels
        of_vm1_Addr = channels_find_by_name("outer_frame_vm_1");
        of_vm2_Addr = channels_find_by_name("outer_frame_vm_2");
        of_vm3_Addr = channels_find_by_name("outer_frame_vm_3");
    }
    if (InCharge && state[LABJACK_OF_POWER].connected) {
        // grab the value out of the labjack
        SET_SCALED_VALUE(of_vm1_Addr, labjack_get_value(LABJACK_OF_POWER, VM1));
        SET_SCALED_VALUE(of_vm2_Addr, labjack_get_value(LABJACK_OF_POWER, VM2));
        SET_SCALED_VALUE(of_vm3_Addr, labjack_get_value(LABJACK_OF_POWER, VM3));
    }
}


/**
 * @brief initializes the channel pointers and stores the current monitoring data
 * 
 */
static void read_of_im(void) {
    static int first_time = 1;
    // declare pointers to the data channels
    static channel_t * of_relay_1_Addr;
    static channel_t * of_relay_2_Addr;
    static channel_t * of_relay_3_Addr;
    static channel_t * of_relay_4_Addr;
    static channel_t * of_relay_5_Addr;
    static channel_t * of_relay_6_Addr;
    static channel_t * of_relay_7_Addr;
    static channel_t * of_relay_8_Addr;
    static channel_t * of_relay_9_Addr;
    static channel_t * of_relay_10_Addr;
    if (first_time) {
        first_time = 0;
        of_relay_1_Addr = channels_find_by_name("current_fc1");
        of_relay_2_Addr = channels_find_by_name("current_fc2");
        of_relay_3_Addr = channels_find_by_name("current_of_eth");
        of_relay_4_Addr = channels_find_by_name("current_watchdog");
        of_relay_5_Addr = channels_find_by_name("current_gondola_thermometry");
        of_relay_6_Addr = channels_find_by_name("current_acomp_box");
        of_relay_7_Addr = channels_find_by_name("current_motor_eth");
        of_relay_8_Addr = channels_find_by_name("current_magnetometer");
        of_relay_9_Addr = channels_find_by_name("current_of_relay_9");
        of_relay_10_Addr = channels_find_by_name("current_of_relay_10");
    }
    if (InCharge && state[LABJACK_OF_POWER].connected) {
        SET_SCALED_VALUE(of_relay_1_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_1));
        SET_SCALED_VALUE(of_relay_2_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_2));
        SET_SCALED_VALUE(of_relay_3_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_3));
        SET_SCALED_VALUE(of_relay_4_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_4));
        SET_SCALED_VALUE(of_relay_5_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_5));
        SET_SCALED_VALUE(of_relay_6_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_6));
        SET_SCALED_VALUE(of_relay_7_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_7));
        SET_SCALED_VALUE(of_relay_8_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_8));
        SET_SCALED_VALUE(of_relay_9_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_9));
        SET_SCALED_VALUE(of_relay_10_Addr, labjack_get_value(LABJACK_OF_POWER, IM_OF_RELAY_10));
    }
}


/**
 * @brief wrapper function to store the voltage and current monitoring data
 * 
 */
void log_of_pbob_analog(void) {
    // Internal functions are InCharge and connected protected already
    read_of_vm();
    read_of_im();
}


/**
 * @brief clears the command data structure values after we read them in to the local struct
 * 
 */
static void clear_of_pbob_cmd_data(void) {
    // just write all zeros to the command data
    CommandData.of_power.relay_1_off = 0;
    CommandData.of_power.relay_1_on = 0;
    CommandData.of_power.relay_2_off = 0;
    CommandData.of_power.relay_2_on = 0;
    CommandData.of_power.relay_3_off = 0;
    CommandData.of_power.relay_3_on = 0;
    CommandData.of_power.relay_4_off = 0;
    CommandData.of_power.relay_4_on = 0;
    CommandData.of_power.relay_5_off = 0;
    CommandData.of_power.relay_5_on = 0;
    CommandData.of_power.relay_6_off = 0;
    CommandData.of_power.relay_6_on = 0;
    CommandData.of_power.relay_7_off = 0;
    CommandData.of_power.relay_7_on = 0;
    CommandData.of_power.relay_8_off = 0;
    CommandData.of_power.relay_8_on = 0;
    CommandData.of_power.relay_9_off = 0;
    CommandData.of_power.relay_9_on = 0;
    CommandData.of_power.relay_10_off = 0;
    CommandData.of_power.relay_10_on = 0;
}


/**
 * @brief copies the command data values for relay states to the local structure
 * 
 */
static void update_from_cmd_data(void) {
    of_pbob.relay_1_off = CommandData.of_power.relay_1_off;
    of_pbob.relay_1_on = CommandData.of_power.relay_1_on;
    of_pbob.relay_2_off = CommandData.of_power.relay_2_off;
    of_pbob.relay_2_on = CommandData.of_power.relay_2_on;
    of_pbob.relay_3_off = CommandData.of_power.relay_3_off;
    of_pbob.relay_3_on = CommandData.of_power.relay_3_on;
    of_pbob.relay_4_off = CommandData.of_power.relay_4_off;
    of_pbob.relay_4_on = CommandData.of_power.relay_4_on;
    of_pbob.relay_5_off = CommandData.of_power.relay_5_off;
    of_pbob.relay_5_on = CommandData.of_power.relay_5_on;
    of_pbob.relay_6_off = CommandData.of_power.relay_6_off;
    of_pbob.relay_6_on = CommandData.of_power.relay_6_on;
    of_pbob.relay_7_off = CommandData.of_power.relay_7_off;
    of_pbob.relay_7_on = CommandData.of_power.relay_7_on;
    of_pbob.relay_8_off = CommandData.of_power.relay_8_off;
    of_pbob.relay_8_on = CommandData.of_power.relay_8_on;
    of_pbob.relay_9_off = CommandData.of_power.relay_9_off;
    of_pbob.relay_9_on = CommandData.of_power.relay_9_on;
    of_pbob.relay_10_off = CommandData.of_power.relay_10_off;
    of_pbob.relay_10_on = CommandData.of_power.relay_10_on;
}


/**
 * @brief generates the falling edge of any commanded relay pulses
 * 
 */
static void end_all_pulses(void) {
    // here we check to see if any DIO lines are on and turn them off if they are
    // we also clear the "memory" of the local struct. This could be done with memset
    // but I prefer it to be done so obviously for documentation.
    if (of_pbob.relay_1_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_1_ON, 0);
        of_pbob.relay_1_on = 0;
    }
    if (of_pbob.relay_1_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_1_OFF, 0);
        of_pbob.relay_1_off = 0;
    }
    if (of_pbob.relay_2_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_2_ON, 0);
        of_pbob.relay_2_on = 0;
    }
    if (of_pbob.relay_2_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_2_OFF, 0);
        of_pbob.relay_2_off = 0;
    }
    // if (of_pbob.relay_3_on) {
    //     labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_3_ON, 0);
    //     of_pbob.relay_3_on = 0;
    // }
    // if (of_pbob.relay_3_off) {
    //     labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_3_OFF, 0);
    //     of_pbob.relay_3_off = 0;
    // }
    if (of_pbob.relay_4_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_4_ON, 0);
        of_pbob.relay_4_on = 0;
    }
    if (of_pbob.relay_4_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_4_OFF, 0);
        of_pbob.relay_4_off = 0;
    }
    if (of_pbob.relay_5_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_5_ON, 0);
        of_pbob.relay_5_on = 0;
    }
    if (of_pbob.relay_5_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_5_OFF, 0);
        of_pbob.relay_5_off = 0;
    }
    if (of_pbob.relay_6_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_6_ON, 0);
        of_pbob.relay_6_on = 0;
    }
    if (of_pbob.relay_6_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_6_OFF, 0);
        of_pbob.relay_6_off = 0;
    }
    if (of_pbob.relay_7_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_7_ON, 0);
        of_pbob.relay_7_on = 0;
    }
    if (of_pbob.relay_7_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_7_OFF, 0);
        of_pbob.relay_7_off = 0;
    }
    if (of_pbob.relay_8_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_8_ON, 0);
        of_pbob.relay_8_on = 0;
    }
    if (of_pbob.relay_8_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_8_OFF, 0);
        of_pbob.relay_8_off = 0;
    }
    if (of_pbob.relay_9_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_9_ON, 0);
        of_pbob.relay_9_on = 0;
    }
    if (of_pbob.relay_9_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_9_OFF, 0);
        of_pbob.relay_9_off = 0;
    }
    if (of_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_10_ON, 0);
        of_pbob.relay_10_on = 0;
    }
    if (of_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_10_OFF, 0);
        of_pbob.relay_10_off = 0;
    }
}


/**
 * @brief generates the rising edge of any commanded relay pulses
 * 
 */
static void start_pulse(void) {
    // check to see which pulse we are supposed to start and do it
    // we only do one at a time so we better return if we find one.
    if (of_pbob.relay_1_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_1_ON, 1);
        return;
    }
    if (of_pbob.relay_1_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_1_OFF, 1);
        return;
    }
    if (of_pbob.relay_2_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_2_ON, 1);
        return;
    }
    if (of_pbob.relay_2_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_2_OFF, 1);
        return;
    }
    // if (of_pbob.relay_3_on) {
    //     labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_3_ON, 1);
    //     return;
    // }
    // if (of_pbob.relay_3_off) {
    //     labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_3_OFF, 1);
    //     return;
    // }
    if (of_pbob.relay_4_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_4_ON, 1);
        return;
    }
    if (of_pbob.relay_4_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_4_OFF, 1);
        return;
    }
    if (of_pbob.relay_5_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_5_ON, 1);
        return;
    }
    if (of_pbob.relay_5_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_5_OFF, 1);
        return;
    }
    if (of_pbob.relay_6_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_6_ON, 1);
        return;
    }
    if (of_pbob.relay_6_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_6_OFF, 1);
        return;
    }
    if (of_pbob.relay_7_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_7_ON, 1);
        return;
    }
    if (of_pbob.relay_7_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_7_OFF, 1);
        return;
    }
    if (of_pbob.relay_8_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_8_ON, 1);
        return;
    }
    if (of_pbob.relay_8_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_8_OFF, 1);
        return;
    }
    if (of_pbob.relay_9_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_9_ON, 1);
        return;
    }
    if (of_pbob.relay_9_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_9_OFF, 1);
        return;
    }
    if (of_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_10_ON, 1);
        return;
    }
    if (of_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_RELAY_10_OFF, 1);
        return;
    }
}


/**
 * @brief wrapper function that handles reading and clearing command data as well as pulse generation
 * 
 */
void of_pbob_commanding(void) {
    if (InCharge && state[LABJACK_OF_POWER].connected) {
        end_all_pulses();
        if (CommandData.of_power.update_pbob == 1) {
            update_from_cmd_data();
            clear_of_pbob_cmd_data();
            CommandData.of_power.update_pbob = 0;
        }
        start_pulse();
    }
}
