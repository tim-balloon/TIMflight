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
 
 created by Ian Lowe 4-25-23
 **************************************************************************/


/*************************************************************************
 
 star_camera_transmit.c -- mcp code to create data streams to SC1 and SC2
 
 *************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>

#include "star_camera_structs.h"
#include "star_camera_solutions.h"


// The two below functions serve the same purpose as above
// but, instead of camera parameters, they store image solution data
static void assign_solution_data_to_channel_sc1(struct mcp_astrometry scm) {
    static int first_time = 1;
    static channel_t * sc1_rawtime_Addr, * sc1_ra_Addr, * sc1_dec_Addr;
    static channel_t * sc1_image_rms_Addr, * sc1_fr_Addr, * sc1_ps_Addr;
    static channel_t * sc1_ir_Addr, * sc1_alt_Addr, * sc1_az_Addr;
    if (first_time == 1) {
        first_time = 0;
        sc1_rawtime_Addr = channels_find_by_name("sc1_rawtime");
        sc1_ra_Addr = channels_find_by_name("sc1_ra");
        sc1_dec_Addr = channels_find_by_name("sc1_dec");
        sc1_image_rms_Addr = channels_find_by_name("sc1_image_rms");
        sc1_fr_Addr = channels_find_by_name("sc1_fr");
        sc1_ps_Addr = channels_find_by_name("sc1_ps");
        sc1_ir_Addr = channels_find_by_name("sc1_ir");
        sc1_alt_Addr = channels_find_by_name("sc1_alt");
        sc1_az_Addr = channels_find_by_name("sc1_az");
    }
    SET_SCALED_VALUE(sc1_rawtime_Addr, scm.rawtime);
    SET_SCALED_VALUE(sc1_ra_Addr, scm.ra_observed);
    SET_SCALED_VALUE(sc1_dec_Addr, scm.dec_observed);
    SET_SCALED_VALUE(sc1_image_rms_Addr, scm.image_rms);
    SET_SCALED_VALUE(sc1_fr_Addr, scm.fr);
    SET_SCALED_VALUE(sc1_ps_Addr, scm.ps);
    SET_SCALED_VALUE(sc1_ir_Addr, scm.ir);
    SET_SCALED_VALUE(sc1_alt_Addr, scm.alt);
    SET_SCALED_VALUE(sc1_az_Addr, scm.az);
}

static void assign_solution_data_to_channel_sc2(struct mcp_astrometry scm) {
    static int first_time = 1;
    static channel_t * sc2_rawtime_Addr, * sc2_ra_Addr, * sc2_dec_Addr;
    static channel_t * sc2_image_rms_Addr, * sc2_fr_Addr, * sc2_ps_Addr;
    static channel_t * sc2_ir_Addr, * sc2_alt_Addr, * sc2_az_Addr;
    if (first_time == 1) {
        first_time = 0;
        sc2_rawtime_Addr = channels_find_by_name("sc2_rawtime");
        sc2_ra_Addr = channels_find_by_name("sc2_ra");
        sc2_dec_Addr = channels_find_by_name("sc2_dec");
        sc2_image_rms_Addr = channels_find_by_name("sc2_image_rms");
        sc2_fr_Addr = channels_find_by_name("sc2_fr");
        sc2_ps_Addr = channels_find_by_name("sc2_ps");
        sc2_ir_Addr = channels_find_by_name("sc2_ir");
        sc2_alt_Addr = channels_find_by_name("sc2_alt");
        sc2_az_Addr = channels_find_by_name("sc2_az");
    }
    SET_SCALED_VALUE(sc2_rawtime_Addr, scm.rawtime);
    SET_SCALED_VALUE(sc2_ra_Addr, scm.ra_observed);
    SET_SCALED_VALUE(sc2_dec_Addr, scm.dec_observed);
    SET_SCALED_VALUE(sc2_image_rms_Addr, scm.image_rms);
    SET_SCALED_VALUE(sc2_fr_Addr, scm.fr);
    SET_SCALED_VALUE(sc2_ps_Addr, scm.ps);
    SET_SCALED_VALUE(sc2_ir_Addr, scm.ir);
    SET_SCALED_VALUE(sc2_alt_Addr, scm.alt);
    SET_SCALED_VALUE(sc2_az_Addr, scm.az);
}