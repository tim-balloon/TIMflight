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
 
 created by Ian Lowe 7-11-24
 **************************************************************************/


/*************************************************************************
 
 motor_box_power.c -- mcp code to control power relays
 
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
#include "motor_box_power.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

/**
 * @brief holds the local state information for the outer frame power commands
 * 
 */
struct motor_box_power {
    int relay_1_on, relay_1_off, relay_2_on, relay_2_off;
    int relay_3_on, relay_3_off, relay_4_on, relay_4_off;
    int relay_5_on, relay_5_off, relay_6_on, relay_6_off;
    int relay_7_on, relay_7_off, relay_8_on, relay_8_off;
    int relay_9_on, relay_9_off, relay_10_on, relay_10_off;
};

struct motor_box_power motor_pbob;

/**
 * @brief clears the command data structure values after we read them in to the local struct
 * 
 */
static void clear_motor_pbob_cmd_data(void) {
    // just write all zeros to the command data
    CommandData.motor_power.relay_1_off = 0;
    CommandData.motor_power.relay_1_on = 0;
    CommandData.motor_power.relay_2_off = 0;
    CommandData.motor_power.relay_2_on = 0;
    CommandData.motor_power.relay_3_off = 0;
    CommandData.motor_power.relay_3_on = 0;
    CommandData.motor_power.relay_4_off = 0;
    CommandData.motor_power.relay_4_on = 0;
    CommandData.motor_power.relay_5_off = 0;
    CommandData.motor_power.relay_5_on = 0;
    CommandData.motor_power.relay_6_off = 0;
    CommandData.motor_power.relay_6_on = 0;
    CommandData.motor_power.relay_7_off = 0;
    CommandData.motor_power.relay_7_on = 0;
    CommandData.motor_power.relay_8_off = 0;
    CommandData.motor_power.relay_8_on = 0;
    CommandData.motor_power.relay_9_off = 0;
    CommandData.motor_power.relay_9_on = 0;
    CommandData.motor_power.relay_10_off = 0;
    CommandData.motor_power.relay_10_on = 0;
}

/**
 * @brief copies the command data values for relay states to the local structure
 * 
 */
static void update_from_cmd_data(void) {
    motor_pbob.relay_1_off = CommandData.motor_power.relay_1_off;
    motor_pbob.relay_1_on = CommandData.motor_power.relay_1_on;
    motor_pbob.relay_2_off = CommandData.motor_power.relay_2_off;
    motor_pbob.relay_2_on = CommandData.motor_power.relay_2_on;
    motor_pbob.relay_3_off = CommandData.motor_power.relay_3_off;
    motor_pbob.relay_3_on = CommandData.motor_power.relay_3_on;
    motor_pbob.relay_4_off = CommandData.motor_power.relay_4_off;
    motor_pbob.relay_4_on = CommandData.motor_power.relay_4_on;
    motor_pbob.relay_5_off = CommandData.motor_power.relay_5_off;
    motor_pbob.relay_5_on = CommandData.motor_power.relay_5_on;
    motor_pbob.relay_6_off = CommandData.motor_power.relay_6_off;
    motor_pbob.relay_6_on = CommandData.motor_power.relay_6_on;
    motor_pbob.relay_7_off = CommandData.motor_power.relay_7_off;
    motor_pbob.relay_7_on = CommandData.motor_power.relay_7_on;
    motor_pbob.relay_8_off = CommandData.motor_power.relay_8_off;
    motor_pbob.relay_8_on = CommandData.motor_power.relay_8_on;
    motor_pbob.relay_9_off = CommandData.motor_power.relay_9_off;
    motor_pbob.relay_9_on = CommandData.motor_power.relay_9_on;
    motor_pbob.relay_10_off = CommandData.motor_power.relay_10_off;
    motor_pbob.relay_10_on = CommandData.motor_power.relay_10_on;
}

/**
 * @brief generates the falling edge of any commanded relay pulses
 * 
 */
static void end_all_pulses(void) {
    // here we check to see if any DIO lines are on and turn them off if they are
    // we also clear the "memory" of the local struct. This could be done with memset
    // but I prefer it to be done so obviously for documentation.
    if (motor_pbob.relay_1_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_1_ON, 0);
        motor_pbob.relay_1_on = 0;
    }
    if (motor_pbob.relay_1_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_1_OFF, 0);
        motor_pbob.relay_1_off = 0;
    }
    if (motor_pbob.relay_2_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_2_ON, 0);
        motor_pbob.relay_2_on = 0;
    }
    if (motor_pbob.relay_2_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_2_OFF, 0);
        motor_pbob.relay_2_off = 0;
    }
    if (motor_pbob.relay_3_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_3_ON, 0);
        motor_pbob.relay_3_on = 0;
    }
    if (motor_pbob.relay_3_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_3_OFF, 0);
        motor_pbob.relay_3_off = 0;
    }
    if (motor_pbob.relay_4_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_4_ON, 0);
        motor_pbob.relay_4_on = 0;
    }
    if (motor_pbob.relay_4_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_4_OFF, 0);
        motor_pbob.relay_4_off = 0;
    }
    if (motor_pbob.relay_5_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_5_ON, 0);
        motor_pbob.relay_5_on = 0;
    }
    if (motor_pbob.relay_5_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_5_OFF, 0);
        motor_pbob.relay_5_off = 0;
    }
    if (motor_pbob.relay_6_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_6_ON, 0);
        motor_pbob.relay_6_on = 0;
    }
    if (motor_pbob.relay_6_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_6_OFF, 0);
        motor_pbob.relay_6_off = 0;
    }
    if (motor_pbob.relay_7_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_7_ON, 0);
        motor_pbob.relay_7_on = 0;
    }
    if (motor_pbob.relay_7_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_7_OFF, 0);
        motor_pbob.relay_7_off = 0;
    }
    if (motor_pbob.relay_8_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_8_ON, 0);
        motor_pbob.relay_8_on = 0;
    }
    if (motor_pbob.relay_8_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_8_OFF, 0);
        motor_pbob.relay_8_off = 0;
    }
    if (motor_pbob.relay_9_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_9_ON, 0);
        motor_pbob.relay_9_on = 0;
    }
    if (motor_pbob.relay_9_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_9_OFF, 0);
        motor_pbob.relay_9_off = 0;
    }
    if (motor_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_10_ON, 0);
        motor_pbob.relay_10_on = 0;
    }
    if (motor_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_10_OFF, 0);
        motor_pbob.relay_10_off = 0;
    }
}

/**
 * @brief generates the rising edge of any commanded relay pulses
 * 
 */
static void start_pulse(void) {
    // check to see which pulse we are supposed to start and do it
    // we only do one at a time so we better return if we find one.
    if (motor_pbob.relay_1_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_1_ON, 1);
        return;
    }
    if (motor_pbob.relay_1_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_1_OFF, 1);
        return;
    }
    if (motor_pbob.relay_2_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_2_ON, 1);
        return;
    }
    if (motor_pbob.relay_2_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_2_OFF, 1);
        return;
    }
    if (motor_pbob.relay_3_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_3_ON, 1);
        return;
    }
    if (motor_pbob.relay_3_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_3_OFF, 1);
        return;
    }
    if (motor_pbob.relay_4_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_4_ON, 1);
        return;
    }
    if (motor_pbob.relay_4_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_4_OFF, 1);
        return;
    }
    if (motor_pbob.relay_5_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_5_ON, 1);
        return;
    }
    if (motor_pbob.relay_5_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_5_OFF, 1);
        return;
    }
    if (motor_pbob.relay_6_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_6_ON, 1);
        return;
    }
    if (motor_pbob.relay_6_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_6_OFF, 1);
        return;
    }
    if (motor_pbob.relay_7_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_7_ON, 1);
        return;
    }
    if (motor_pbob.relay_7_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_7_OFF, 1);
        return;
    }
    if (motor_pbob.relay_8_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_8_ON, 1);
        return;
    }
    if (motor_pbob.relay_8_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_8_OFF, 1);
        return;
    }
    if (motor_pbob.relay_9_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_9_ON, 1);
        return;
    }
    if (motor_pbob.relay_9_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_9_OFF, 1);
        return;
    }
    if (motor_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_10_ON, 1);
        return;
    }
    if (motor_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_MOTOR_POWER, RELAY_10_OFF, 1);
        return;
    }
}

// we call this in MCP
/**
 * @brief wrapper function that handles reading and clearing command data as well as pulse generation
 * 
 */
void motor_pbob_commanding(void) {
    if (InCharge && state[LABJACK_MOTOR_POWER].connected) {
        end_all_pulses();
        if (CommandData.motor_power.update_pbob == 1) {
            update_from_cmd_data();
            clear_of_pbob_cmd_data();
            CommandData.motor_power.update_pbob = 0;
        }
        start_pulse();
    }
}
