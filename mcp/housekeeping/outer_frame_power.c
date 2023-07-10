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

struct outer_frame_power {
    int fc1_on, fc1_off, fc2_on, fc2_off;
    int motor_lj_on, motor_lj_off, unassigned_on, unassigned_off;
    int of_inc_on, of_inc_off;
    int mag_on, mag_off, therm_on, therm_off;
    int gps_on, gps_off, pss_on, pss_off;
};

struct outer_frame_power of_pbob;

// function to process in the outer frame voltage monitor data
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


// function to process in the outer frame current monitoring data
static void read_of_im(void) {
    static int first_time = 1;
    // declare pointers to the data channels
    static channel_t * fc1_im_Addr;
    static channel_t * fc2_im_Addr;
    static channel_t * of_eth_im_Addr;
    static channel_t * motor_lj_im_Addr;
    static channel_t * unassigned_im_Addr;
    static channel_t * of_inclinometer_im_Addr;
    static channel_t * magnetomer_im_Addr;
    static channel_t * gondola_therm_im_Addr;
    static channel_t * gps_ntp_im_Addr;
    static channel_t * pss_im_Addr;
    if (first_time) {
        first_time = 0;
        fc1_im_Addr = channels_find_by_name("current_fc1");
        fc2_im_Addr = channels_find_by_name("current_fc2");
        of_eth_im_Addr = channels_find_by_name("current_of_eth");
        motor_lj_im_Addr = channels_find_by_name("current_motor_box_lj");
        unassigned_im_Addr = channels_find_by_name("current_unassigned");
        of_inclinometer_im_Addr = channels_find_by_name("current_of_inclinometer");
        magnetomer_im_Addr = channels_find_by_name("current_magnetometers");
        gondola_therm_im_Addr = channels_find_by_name("current_gondola_thermometry");
        gps_ntp_im_Addr = channels_find_by_name("current_gps_ntp");
        pss_im_Addr = channels_find_by_name("current_pss_box");
    }
    if (InCharge && state[LABJACK_OF_POWER].connected) {
        SET_SCALED_VALUE(fc1_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_FC1));
        SET_SCALED_VALUE(fc2_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_FC2));
        SET_SCALED_VALUE(of_eth_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_OF_ETH));
        SET_SCALED_VALUE(motor_lj_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_MOT_LJ));
        SET_SCALED_VALUE(unassigned_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_UNASSIGNED));
        SET_SCALED_VALUE(of_inclinometer_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_OF_INC));
        SET_SCALED_VALUE(magnetomer_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_MAG));
        SET_SCALED_VALUE(gondola_therm_im_Addr, labjack_get_value(LABJACK_OF_POWER, IM_THERM));
        SET_SCALED_VALUE(gps_ntp_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_GPS));
        SET_SCALED_VALUE(pss_im_Addr , labjack_get_value(LABJACK_OF_POWER, IM_PSS));
    }
}


// Function for other users to employ for reading the voltages and currents
void log_of_pbob_analog(void) {
    // Internal functions are InCharge and connected protected already
    read_of_vm();
    read_of_im();
}


static void clear_of_pbob_cmd_data(void) {
    // just write all zeros to the command data
    CommandData.of_power.relay_1_off = 0;
    CommandData.of_power.relay_1_on = 0;
    CommandData.of_power.relay_2_off = 0;
    CommandData.of_power.relay_2_on = 0;
    // OF doesn't let LJ access relay 3
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

static void update_from_cmd_data(void) {
    of_pbob.fc1_off = CommandData.of_power.relay_1_off;
    of_pbob.fc1_on = CommandData.of_power.relay_1_on;
    of_pbob.fc2_off = CommandData.of_power.relay_2_off;
    of_pbob.fc2_on = CommandData.of_power.relay_2_on;
    of_pbob.motor_lj_off = CommandData.of_power.relay_4_off;
    of_pbob.motor_lj_on = CommandData.of_power.relay_4_on;
    of_pbob.unassigned_off = CommandData.of_power.relay_5_off;
    of_pbob.unassigned_on = CommandData.of_power.relay_5_on;
    of_pbob.of_inc_off = CommandData.of_power.relay_6_off;
    of_pbob.of_inc_on = CommandData.of_power.relay_6_on;
    of_pbob.mag_off = CommandData.of_power.relay_7_off;
    of_pbob.mag_on = CommandData.of_power.relay_7_on;
    of_pbob.therm_off = CommandData.of_power.relay_8_off;
    of_pbob.therm_on = CommandData.of_power.relay_8_on;
    of_pbob.gps_off = CommandData.of_power.relay_9_off;
    of_pbob.gps_on = CommandData.of_power.relay_9_on;
    of_pbob.pss_off = CommandData.of_power.relay_10_off;
    of_pbob.pss_on = CommandData.of_power.relay_10_on;
}

static void end_all_pulses(void) {
    // here we check to see if any DIO lines are on and turn them off if they are
    // we also clear the "memory" of the local struct. This could be done with memset
    // but I prefer it to be done so obviously for documentation.
    if (of_pbob.fc1_on) {
        labjack_queue_command(LABJACK_OF_POWER, FC1_ON, 0);
        of_pbob.fc1_on = 0;
    }
    if (of_pbob.fc1_off) {
        labjack_queue_command(LABJACK_OF_POWER, FC1_OFF, 0);
        of_pbob.fc1_off = 0;
    }
    if (of_pbob.fc2_on) {
        labjack_queue_command(LABJACK_OF_POWER, FC2_ON, 0);
        of_pbob.fc2_on = 0;
    }
    if (of_pbob.fc2_off) {
        labjack_queue_command(LABJACK_OF_POWER, FC2_OFF, 0);
        of_pbob.fc2_off = 0;
    }
    if (of_pbob.motor_lj_on) {
        labjack_queue_command(LABJACK_OF_POWER, MOTOR_LJ_ON, 0);
        of_pbob.motor_lj_on = 0;
    }
    if (of_pbob.motor_lj_off) {
        labjack_queue_command(LABJACK_OF_POWER, MOTOR_LJ_OFF, 0);
        of_pbob.motor_lj_off = 0;
    }
    if (of_pbob.unassigned_on) {
        labjack_queue_command(LABJACK_OF_POWER, RELAY_5_ON, 0);
        of_pbob.unassigned_on = 0;
    }
    if (of_pbob.unassigned_off) {
        labjack_queue_command(LABJACK_OF_POWER, RELAY_5_OFF, 0);
        of_pbob.unassigned_off = 0;
    }
    if (of_pbob.of_inc_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_INC_ON, 0);
        of_pbob.of_inc_on = 0;
    }
    if (of_pbob.of_inc_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_INC_OFF, 0);
        of_pbob.of_inc_off = 0;
    }
    if (of_pbob.mag_on) {
        labjack_queue_command(LABJACK_OF_POWER, MAG_ON, 0);
        of_pbob.mag_on = 0;
    }
    if (of_pbob.mag_off) {
        labjack_queue_command(LABJACK_OF_POWER, MAG_OFF, 0);
        of_pbob.mag_off = 0;
    }
    if (of_pbob.therm_on) {
        labjack_queue_command(LABJACK_OF_POWER, THERMISTORS_ON, 0);
        of_pbob.therm_on = 0;
    }
    if (of_pbob.therm_off) {
        labjack_queue_command(LABJACK_OF_POWER, THERM_READOUT_OFF, 0);
        of_pbob.therm_off = 0;
    }
    if (of_pbob.gps_on) {
        labjack_queue_command(LABJACK_OF_POWER, GPS_NTP_ON, 0);
        of_pbob.gps_on = 0;
    }
    if (of_pbob.gps_off) {
        labjack_queue_command(LABJACK_OF_POWER, GPS_NTP_OFF, 0);
        of_pbob.gps_off = 0;
    }
    if (of_pbob.pss_on) {
        labjack_queue_command(LABJACK_OF_POWER, PSS_ON, 0);
        of_pbob.pss_on = 0;
    }
    if (of_pbob.pss_off) {
        labjack_queue_command(LABJACK_OF_POWER, PSS_OFF, 0);
        of_pbob.pss_off = 0;
    }
}

static void start_pulse(void) {
    // check to see which pulse we are supposed to start and do it
    // we only do one at a time so we better return if we find one.
    if (of_pbob.fc1_on) {
        blast_info("turning on FC1");
        labjack_queue_command(LABJACK_OF_POWER, FC1_ON, 1);
        return;
    }
    if (of_pbob.fc1_off) {
        labjack_queue_command(LABJACK_OF_POWER, FC1_OFF, 1);
        return;
    }
    if (of_pbob.fc2_on) {
        labjack_queue_command(LABJACK_OF_POWER, FC2_ON, 1);
        return;
    }
    if (of_pbob.fc2_off) {
        labjack_queue_command(LABJACK_OF_POWER, FC2_OFF, 1);
        return;
    }
    if (of_pbob.motor_lj_on) {
        labjack_queue_command(LABJACK_OF_POWER, MOTOR_LJ_ON, 1);
        return;
    }
    if (of_pbob.motor_lj_off) {
        labjack_queue_command(LABJACK_OF_POWER, MOTOR_LJ_OFF, 1);
        return;
    }
    if (of_pbob.unassigned_on) {
        labjack_queue_command(LABJACK_OF_POWER, RELAY_5_ON, 1);
        return;
    }
    if (of_pbob.unassigned_off) {
        labjack_queue_command(LABJACK_OF_POWER, RELAY_5_OFF, 1);
        return;
    }
    if (of_pbob.of_inc_on) {
        labjack_queue_command(LABJACK_OF_POWER, OF_INC_ON, 1);
        return;
    }
    if (of_pbob.of_inc_off) {
        labjack_queue_command(LABJACK_OF_POWER, OF_INC_OFF, 1);
        return;
    }
    if (of_pbob.mag_on) {
        labjack_queue_command(LABJACK_OF_POWER, MAG_ON, 1);
        return;
    }
    if (of_pbob.mag_off) {
        labjack_queue_command(LABJACK_OF_POWER, MAG_OFF, 1);
        return;
    }
    if (of_pbob.therm_on) {
        labjack_queue_command(LABJACK_OF_POWER, THERMISTORS_ON, 1);
        return;
    }
    if (of_pbob.therm_off) {
        labjack_queue_command(LABJACK_OF_POWER, THERMISTORS_OFF, 1);
        return;
    }
    if (of_pbob.gps_on) {
        labjack_queue_command(LABJACK_OF_POWER, GPS_NTP_ON, 1);
        return;
    }
    if (of_pbob.gps_off) {
        labjack_queue_command(LABJACK_OF_POWER, GPS_NTP_OFF, 1);
        return;
    }
    if (of_pbob.pss_on) {
        labjack_queue_command(LABJACK_OF_POWER, PSS_ON, 1);
        return;
    }
    if (of_pbob.pss_off) {
        labjack_queue_command(LABJACK_OF_POWER, PSS_OFF, 1);
        return;
    }
}

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
