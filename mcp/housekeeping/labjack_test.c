/***************************************************************************
 mcp: the TIM master control program
 
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
 
 created by Ian Lowe 1-24-23
 **************************************************************************/


/*************************************************************************
 
 labjack_test.c -- mcp code to provide a test setup for a labjack on ubuntu 20.04
 
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
#include "labjack_test.h"

extern labjack_state_t state[NUM_LABJACKS];


/**
 * @brief read the value on analog input i to labjack cryo 1 (192.168.1.110) and
 * print it to the blast log. Simple test function
 * 
 * @param i which AIN channel to read
 */
void read_from_lj(int i) {
    float read_val = 0;
    read_val = labjack_get_value(LABJACK_CRYO_1, i);
    blast_info("We see %f volts on analog input 0", read_val);
}
