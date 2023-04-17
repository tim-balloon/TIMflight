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
 
 inner_frame_power.c -- mcp code to control power relays
 
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
#include "inner_frame_power.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

struct inner_frame_power {
    int sc1_on, sc1_off, cryo_hk_on, cryo_hk_off;
    int gyros_on, gyros_off, rfsoc_on, rfsoc_off;
    int steppers_on, steppers_off;
    int if_inc_on, if_inc_off, sc2_on, sc2_off;
    int cryo_power_on, cryo_power_off, relay_10_on, relay_10_off;
};

struct inner_frame_power if_pbob;

// function to process in the inner frame voltage monitor data
static void read_if_vm(void) {
    static int first_time = 1;
    // declare pointers to the data channels
    static channel_t * if_vm1_Addr;
    static channel_t * if_vm2_Addr;
    static channel_t * if_vm3_Addr;
    if (first_time) {
        first_time = 0;
        // now link the pointers to their approrpriate channels
        if_vm1_Addr = channels_find_by_name("inner_frame_vm_1");
        if_vm2_Addr = channels_find_by_name("inner_frame_vm_2");
        if_vm3_Addr = channels_find_by_name("inner_frame_vm_3");
    }
    if (InCharge && state[LABJACK_IF_POWER].connected) {
        // grab the value out of the labjack
        SET_SCALED_VALUE(if_vm1_Addr, labjack_get_value(LABJACK_IF_POWER, VM1));
        SET_SCALED_VALUE(if_vm2_Addr, labjack_get_value(LABJACK_IF_POWER, VM2));
        SET_SCALED_VALUE(if_vm3_Addr, labjack_get_value(LABJACK_IF_POWER, VM3));
    }
}

// function to process the inner frame current monitoring data
static void read_if_im(void) {
    static int first_time = 1;
    static channel_t * sc1_im_Addr;
    static channel_t * cryo_digital_im_Addr;
    static channel_t * gyros_im_Addr;
    static channel_t * rfsoc_im_Addr;
    static channel_t * if_eth_im_Addr;
    static channel_t * stepper_im_Addr;
    static channel_t * if_inclinometer_Addr;
    static channel_t * sc2_im_Addr;
    static channel_t * cryo_analog_im_Addr;
    static channel_t * last_resort_im_Addr;
    if (first_time) {
        first_time = 0;
        sc1_im_Addr = channels_find_by_name("current_sc1");
        cryo_digital_im_Addr = channels_find_by_name("current_cryo_digital");
        gyros_im_Addr = channels_find_by_name("current_gyros");
        rfsoc_im_Addr = channels_find_by_name("current_rfsoc");
        if_eth_im_Addr = channels_find_by_name("current_if_eth");
        stepper_im_Addr = channels_find_by_name("current_steppers");
        if_inclinometer_Addr = channels_find_by_name("current_if_inclinometer");
        sc2_im_Addr = channels_find_by_name("current_sc2");
        cryo_analog_im_Addr = channels_find_by_name("current_cryo_analog");
        last_resort_im_Addr = channels_find_by_name("current_last_resort");
    }
    if (InCharge && state[LABJACK_IF_POWER].connected) {
        SET_SCALED_VALUE(sc1_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_SC1));
        SET_SCALED_VALUE(cryo_digital_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_CRYO_DIG));
        SET_SCALED_VALUE(gyros_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_GYROS));
        SET_SCALED_VALUE(rfsoc_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_RFSOC));
        SET_SCALED_VALUE(if_eth_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_IF_ETH));
        SET_SCALED_VALUE(stepper_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_STEPPERS));
        SET_SCALED_VALUE(if_inclinometer_Addr, labjack_get_value(LABJACK_IF_POWER, IM_IF_INC));
        SET_SCALED_VALUE(sc2_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_SC2));
        SET_SCALED_VALUE(cryo_analog_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_CRYO_AN));
        SET_SCALED_VALUE(last_resort_im_Addr, labjack_get_value(LABJACK_IF_POWER, IM_LAST_RESORT));
    }
}

// Function for other users to employ for reading the voltages and currents
void log_if_pbob_analog(void) {
    // Internal functions are InCharge and connected protected already
    read_if_vm();
    read_if_im();
}

static void end_all_pulses(void) {
    // here we check to see if any DIO lines are on and turn them off if they are
    // we also clear the "memory" of the local struct. This could be done with memset
    // but I prefer it to be done so obviously for documentation.
    if (if_pbob.sc1_on) {
        labjack_queue_command(LABJACK_IF_POWER, SC1_ON, 0);
        if_pbob.sc1_on = 0;
    }
    if (if_pbob.sc1_off) {
        labjack_queue_command(LABJACK_IF_POWER, SC1_OFF, 0);
        if_pbob.sc1_off = 0;
    }
    if (if_pbob.cryo_hk_on) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_READOUT_ON, 0);
        if_pbob.cryo_hk_on = 0;
    }
    if (if_pbob.cryo_hk_off) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_READOUT_OFF, 0);
        if_pbob.cryo_hk_off = 0;
    }
    if (if_pbob.gyros_on) {
        labjack_queue_command(LABJACK_IF_POWER, GYROS_ON, 0);
        if_pbob.gyros_on = 0;
    }
    if (if_pbob.gyros_off) {
        labjack_queue_command(LABJACK_IF_POWER, GYROS_OFF, 0);
        if_pbob.gyros_off = 0;
    }
    if (if_pbob.rfsoc_on) {
        labjack_queue_command(LABJACK_IF_POWER, RFSOC_ON, 0);
        if_pbob.rfsoc_on = 0;
    }
    if (if_pbob.rfsoc_off) {
        labjack_queue_command(LABJACK_IF_POWER, RFSOC_OFF, 0);
        if_pbob.rfsoc_off = 0;
    }
    if (if_pbob.steppers_on) {
        labjack_queue_command(LABJACK_IF_POWER, IF_STEPPERS_ON, 0);
        if_pbob.steppers_on = 0;
    }
    if (if_pbob.steppers_off) {
        labjack_queue_command(LABJACK_IF_POWER, IF_STEPPERS_OFF, 0);
        if_pbob.steppers_off = 0;
    }
    if (if_pbob.if_inc_on) {
        labjack_queue_command(LABJACK_IF_POWER, IF_INC_ON, 0);
        if_pbob.if_inc_on = 0;
    }
    if (if_pbob.if_inc_off) {
        labjack_queue_command(LABJACK_IF_POWER, IF_INC_OFF, 0);
        if_pbob.if_inc_off = 0;
    }
    if (if_pbob.sc2_on) {
        labjack_queue_command(LABJACK_IF_POWER, SC2_ON, 0);
        if_pbob.sc2_on = 0;
    }
    if (if_pbob.sc2_off) {
        labjack_queue_command(LABJACK_IF_POWER, SC2_OFF, 0);
        if_pbob.sc2_off = 0;
    }
    if (if_pbob.cryo_power_on) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_SUPPLY_ON, 0);
        if_pbob.cryo_power_on = 0;
    }
    if (if_pbob.cryo_power_off) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_SUPPLY_OFF, 0);
        if_pbob.cryo_power_off = 0;
    }
    if (if_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_IF_POWER, RELAY_10_ON, 0);
        if_pbob.relay_10_on = 0;
    }
    if (if_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_IF_POWER, RELAY_10_OFF, 0);
        if_pbob.relay_10_off = 0;
    }
}

static void start_pulse(void) {
    // check to see which pulse we are supposed to start and do it
    // we only do one at a time so we better return if we find one.
    if (if_pbob.sc1_on) {
        labjack_queue_command(LABJACK_IF_POWER, SC1_ON, 1);
        return;
    }
    if (if_pbob.sc1_off) {
        labjack_queue_command(LABJACK_IF_POWER, SC1_OFF, 1);
        return;
    }
    if (if_pbob.cryo_hk_on) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_READOUT_ON, 1);
        return;
    }
    if (if_pbob.cryo_hk_off) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_READOUT_OFF, 1);
        return;
    }
    if (if_pbob.gyros_on) {
        labjack_queue_command(LABJACK_IF_POWER, GYROS_ON, 1);
        return;
    }
    if (if_pbob.gyros_off) {
        labjack_queue_command(LABJACK_IF_POWER, GYROS_OFF, 1);
        return;
    }
    if (if_pbob.rfsoc_on) {
        labjack_queue_command(LABJACK_IF_POWER, RFSOC_ON, 1);
        return;
    }
    if (if_pbob.rfsoc_off) {
        labjack_queue_command(LABJACK_IF_POWER, RFSOC_OFF, 1);
        return;
    }
    if (if_pbob.steppers_on) {
        labjack_queue_command(LABJACK_IF_POWER, IF_STEPPERS_ON, 1);
        return;
    }
    if (if_pbob.steppers_off) {
        labjack_queue_command(LABJACK_IF_POWER, IF_STEPPERS_OFF, 1);
        return;
    }
    if (if_pbob.if_inc_on) {
        labjack_queue_command(LABJACK_IF_POWER, IF_INC_ON, 1);
        return;
    }
    if (if_pbob.if_inc_off) {
        labjack_queue_command(LABJACK_IF_POWER, IF_INC_OFF, 1);
        return;
    }
    if (if_pbob.sc2_on) {
        labjack_queue_command(LABJACK_IF_POWER, SC2_ON, 1);
        return;
    }
    if (if_pbob.sc2_off) {
        labjack_queue_command(LABJACK_IF_POWER, SC2_OFF, 1);
        return;
    }
    if (if_pbob.cryo_power_on) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_SUPPLY_ON, 1);
        return;
    }
    if (if_pbob.cryo_power_off) {
        labjack_queue_command(LABJACK_IF_POWER, CRYO_HK_SUPPLY_OFF, 1);
        return;
    }
    if (if_pbob.relay_10_on) {
        labjack_queue_command(LABJACK_IF_POWER, RELAY_10_ON, 1);
        return;
    }
    if (if_pbob.relay_10_off) {
        labjack_queue_command(LABJACK_IF_POWER, RELAY_10_OFF, 1);
        return;
    }
}


static void clear_if_pbob_cmd_data(void) {
    // just write all zeros to the command data
    CommandData.if_power.relay_1_off = 0;
    CommandData.if_power.relay_1_on = 0;
    CommandData.if_power.relay_2_off = 0;
    CommandData.if_power.relay_2_on = 0;
    CommandData.if_power.relay_3_off = 0;
    CommandData.if_power.relay_3_on = 0;
    CommandData.if_power.relay_4_off = 0;
    CommandData.if_power.relay_4_on = 0;
    // IF does not let LJ access relay 5
    CommandData.if_power.relay_6_off = 0;
    CommandData.if_power.relay_6_on = 0;
    CommandData.if_power.relay_7_off = 0;
    CommandData.if_power.relay_7_on = 0;
    CommandData.if_power.relay_8_off = 0;
    CommandData.if_power.relay_8_on = 0;
    CommandData.if_power.relay_9_off = 0;
    CommandData.if_power.relay_9_on = 0;
    CommandData.if_power.relay_10_off = 0;
    CommandData.if_power.relay_10_on = 0;
}

static void update_from_cmd_data(void) {
    if_pbob.sc1_off = CommandData.if_power.relay_1_off;
    if_pbob.sc1_on = CommandData.if_power.relay_1_on;
    if_pbob.cryo_hk_off = CommandData.if_power.relay_2_off;
    if_pbob.cryo_hk_on = CommandData.if_power.relay_2_on;
    if_pbob.gyros_off = CommandData.if_power.relay_3_off;
    if_pbob.gyros_on = CommandData.if_power.relay_3_on;
    if_pbob.rfsoc_off = CommandData.if_power.relay_4_off;
    if_pbob.rfsoc_on = CommandData.if_power.relay_4_on;
    if_pbob.steppers_off = CommandData.if_power.relay_6_off;
    if_pbob.steppers_on = CommandData.if_power.relay_6_on;
    if_pbob.if_inc_off = CommandData.if_power.relay_7_off;
    if_pbob.if_inc_on = CommandData.if_power.relay_7_on;
    if_pbob.sc2_off = CommandData.if_power.relay_8_off;
    if_pbob.sc2_on = CommandData.if_power.relay_8_on;
    if_pbob.cryo_power_off = CommandData.if_power.relay_9_off;
    if_pbob.cryo_power_on = CommandData.if_power.relay_9_on;
    if_pbob.relay_10_off = CommandData.if_power.relay_10_off;
    if_pbob.relay_10_on = CommandData.if_power.relay_10_on;
}


// Logic here is to queue the end of any pulses to make sure things are off
// before checking for updates and moving that information to the local
// data structure if there are. Once this is complete we start the requested pulse
// and continue on our way. This works for 1 relay at a time but the temporal spacing
// is quite short, only .2 seconds between function calls.
void if_pbob_commanding(void) {
    if (InCharge && state[LABJACK_IF_POWER].connected) {
        end_all_pulses();
        if (CommandData.if_power.update_pbob == 1) {
            update_from_cmd_data();
            clear_if_pbob_cmd_data();
            CommandData.if_power.update_pbob = 0;
        }
        start_pulse();
    }
}
