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
 
 star_camera_receive.c -- mcp code to connect to data streams from SC1 and SC2
 
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
#include "star_camera_receive.h"

// a lot of boilerplate stuff to set up the UDP sockets in a minimal way


// rather than the fancy obfuscated XSC0/1 or ISC/OSC stuff previously used
// I will just create separate functions for SC1 and SC2 so this can be easily
// removed or edited by future users

// function that runs in the receive thread to put data into the proper channels
static void assign_param_data_to_channel_sc1(struct star_cam_return scr) {
    static int first_time = 1;
    // first thing we need to do is create all the channel pointers
    static channel_t * sc1_spike_limit_Addr, * sc1_dyanmic_hp_Addr, * sc1_r_smooth_Addr;
    static channel_t * sc1_high_pass_filter_Addr, * sc1_r_high_pass_filter_Addr;
    static channel_t * sc1_search_border_Addr, * sc1_filter_return_Addr, * sc1_n_sigma_Addr;
    static channel_t * sc1_unique_star_spacing_Addr, * sc1_make_static_hp_mask_Addr;
    static channel_t * sc1_use_static_hp_mask_Addr, * sc1_time_limit_Addr, * sc1_logodds_Addr;
    static channel_t * sc1_latitude_Addr, * sc1_longitude_Addr, * sc1_hm_Addr;
    static channel_t * sc1_focus_position_Addr, * sc1_focus_inf_Addr, * sc1_aperture_steps_Addr;
    static channel_t * sc1_max_aperture_Addr, * sc1_min_focus_pos_Addr, * sc1_max_focus_pos_Addr;
    static channel_t * sc1_current_aperture_Addr, * sc1_exposure_time_Addr, * sc1_focus_mode_Addr;
    static channel_t * sc1_start_focus_pos_Addr, * sc1_end_focus_pos_Addr, * sc1_focus_step_Addr;
    static channel_t * sc1_photos_per_focus_Addr;
    if (first_time == 1) {
        first_time = 0;
        sc1_spike_limit_Addr = channels_find_by_name("sc1_spike_limit");
        sc1_dyanmic_hp_Addr = channels_find_by_name("sc1_dynamic_hp");
        sc1_r_smooth_Addr = channels_find_by_name("sc1_r_smooth");
        sc1_high_pass_filter_Addr = channels_find_by_name("sc1_high_pass_filter");
        sc1_r_high_pass_filter_Addr = channels_find_by_name("sc1_r_high_pass_filter");
        sc1_search_border_Addr = channels_find_by_name("sc1_search_border");
        sc1_filter_return_Addr = channels_find_by_name("sc1_filter_return");
        sc1_n_sigma_Addr = channels_find_by_name("sc1_n_sigma");
        sc1_unique_star_spacing_Addr = channels_find_by_name("sc1_unique_star_spacing");
        sc1_make_static_hp_mask_Addr = channels_find_by_name("sc1_make_static_hp_mask");
        sc1_use_static_hp_mask_Addr = channels_find_by_name("sc1_use_static_hp_mask");
        sc1_time_limit_Addr = channels_find_by_name("sc1_time_limit");
        sc1_logodds_Addr = channels_find_by_name("sc1_logodds");
        sc1_latitude_Addr = channels_find_by_name("sc1_latitude");
        sc1_longitude_Addr = channels_find_by_name("sc1_longitude");
        sc1_hm_Addr = channels_find_by_name("sc1_hm");
        sc1_focus_position_Addr = channels_find_by_name("sc1_focus_position");
        sc1_focus_inf_Addr = channels_find_by_name("sc1_focus_inf");
        sc1_aperture_steps_Addr = channels_find_by_name("sc1_aperture_steps");
        sc1_max_aperture_Addr = channels_find_by_name("sc1_max_aperture");
        sc1_min_focus_pos_Addr = channels_find_by_name("sc1_min_focus_pos");
        sc1_max_focus_pos_Addr = channels_find_by_name("sc1_max_focus_pos");
        sc1_current_aperture_Addr = channels_find_by_name("");
        sc1_exposure_time_Addr = channels_find_by_name("");
        sc1_focus_mode_Addr = channels_find_by_name("");
        sc1_start_focus_pos_Addr = channels_find_by_name("");
        sc1_end_focus_pos_Addr = channels_find_by_name("");
        sc1_focus_step_Addr = channels_find_by_name("");
        sc1_photos_per_focus_Addr = channels_find_by_name("");
    }
    SET_SCALED_VALUE(sc1_spike_limit_Addr, scr.blobParams[0]);
    SET_SCALED_VALUE(sc1_dyanmic_hp_Addr, scr.blobParams[1]);
    SET_SCALED_VALUE(sc1_r_smooth_Addr, scr.blobParams[2]);
    SET_SCALED_VALUE(sc1_high_pass_filter_Addr, scr.blobParams[3]);
    SET_SCALED_VALUE(sc1_r_high_pass_filter_Addr, scr.blobParams[4]);
    SET_SCALED_VALUE(sc1_search_border_Addr, scr.blobParams[5]);
    SET_SCALED_VALUE(sc1_filter_return_Addr, scr.blobParams[6]);
    SET_SCALED_VALUE(sc1_n_sigma_Addr, scr.blobParams[7]);
    SET_SCALED_VALUE(sc1_unique_star_spacing_Addr, scr.blobParams[8]);
    SET_SCALED_VALUE(sc1_make_static_hp_mask_Addr, scr.makeHP);
    SET_SCALED_VALUE(sc1_use_static_hp_mask_Addr, scr.useHP);
    SET_SCALED_VALUE(sc1_time_limit_Addr, scr.solveTimeLimit);
    SET_SCALED_VALUE(sc1_logodds_Addr, scr.logOdds);
    SET_SCALED_VALUE(sc1_latitude_Addr, scr.latitude);
    SET_SCALED_VALUE(sc1_longitude_Addr, scr.longitude);
    SET_SCALED_VALUE(sc1_hm_Addr, scr.heightWGS84);
    SET_SCALED_VALUE(sc1_focus_position_Addr, scr.focusPos);
    SET_SCALED_VALUE(sc1_focus_inf_Addr, scr.setFocusInf);
    SET_SCALED_VALUE(sc1_aperture_steps_Addr, scr.apertureSteps);
    SET_SCALED_VALUE(sc1_max_aperture_Addr, scr.maxAperture);
    SET_SCALED_VALUE(sc1_min_focus_pos_Addr, scr.minFocusPos);
    SET_SCALED_VALUE(sc1_max_focus_pos_Addr, scr.maxFocusPos);
    SET_SCALED_VALUE(sc1_current_aperture_Addr, scr.aperture);
    SET_SCALED_VALUE(sc1_exposure_time_Addr, scr.exposureTime);
    SET_SCALED_VALUE(sc1_focus_mode_Addr, scr.focusMode);
    SET_SCALED_VALUE(sc1_start_focus_pos_Addr, scr.startPos);
    SET_SCALED_VALUE(sc1_end_focus_pos_Addr, scr.endPos);
    SET_SCALED_VALUE(sc1_focus_step_Addr, scr.focusStep);
    SET_SCALED_VALUE(sc1_photos_per_focus_Addr, scr.photosPerStep);
}

// function that runs in the receive thread to put data into the proper channels
static void assign_param_data_to_channel_sc2(struct star_cam_return scr) {
    static int first_time = 1;
    // first thing we need to do is create all the channel pointers
    static channel_t * sc2_spike_limit_Addr, * sc2_dyanmic_hp_Addr, * sc2_r_smooth_Addr;
    static channel_t * sc2_high_pass_filter_Addr, * sc2_r_high_pass_filter_Addr;
    static channel_t * sc2_search_border_Addr, * sc2_filter_return_Addr, * sc2_n_sigma_Addr;
    static channel_t * sc2_unique_star_spacing_Addr, * sc2_make_static_hp_mask_Addr;
    static channel_t * sc2_use_static_hp_mask_Addr, * sc2_time_limit_Addr, * sc2_logodds_Addr;
    static channel_t * sc2_latitude_Addr, * sc2_longitude_Addr, * sc2_hm_Addr;
    static channel_t * sc2_focus_position_Addr, * sc2_focus_inf_Addr, * sc2_aperture_steps_Addr;
    static channel_t * sc2_max_aperture_Addr, * sc2_min_focus_pos_Addr, * sc2_max_focus_pos_Addr;
    static channel_t * sc2_current_aperture_Addr, * sc2_exposure_time_Addr, * sc2_focus_mode_Addr;
    static channel_t * sc2_start_focus_pos_Addr, * sc2_end_focus_pos_Addr, * sc2_focus_step_Addr;
    static channel_t * sc2_photos_per_focus_Addr;
    if (first_time == 1) {
        first_time = 0;
        sc2_spike_limit_Addr = channels_find_by_name("sc2_spike_limit");
        sc2_dyanmic_hp_Addr = channels_find_by_name("sc2_dynamic_hp");
        sc2_r_smooth_Addr = channels_find_by_name("sc2_r_smooth");
        sc2_high_pass_filter_Addr = channels_find_by_name("sc2_high_pass_filter");
        sc2_r_high_pass_filter_Addr = channels_find_by_name("sc2_r_high_pass_filter");
        sc2_search_border_Addr = channels_find_by_name("sc2_search_border");
        sc2_filter_return_Addr = channels_find_by_name("sc2_filter_return");
        sc2_n_sigma_Addr = channels_find_by_name("sc2_n_sigma");
        sc2_unique_star_spacing_Addr = channels_find_by_name("sc2_unique_star_spacing");
        sc2_make_static_hp_mask_Addr = channels_find_by_name("sc2_make_static_hp_mask");
        sc2_use_static_hp_mask_Addr = channels_find_by_name("sc2_use_static_hp_mask");
        sc2_time_limit_Addr = channels_find_by_name("sc2_time_limit");
        sc2_logodds_Addr = channels_find_by_name("sc2_logodds");
        sc2_latitude_Addr = channels_find_by_name("sc2_latitude");
        sc2_longitude_Addr = channels_find_by_name("sc2_longitude");
        sc2_hm_Addr = channels_find_by_name("sc2_hm");
        sc2_focus_position_Addr = channels_find_by_name("sc2_focus_position");
        sc2_focus_inf_Addr = channels_find_by_name("sc2_focus_inf");
        sc2_aperture_steps_Addr = channels_find_by_name("sc2_aperture_steps");
        sc2_max_aperture_Addr = channels_find_by_name("sc2_max_aperture");
        sc2_min_focus_pos_Addr = channels_find_by_name("sc2_min_focus_pos");
        sc2_max_focus_pos_Addr = channels_find_by_name("sc2_max_focus_pos");
        sc2_current_aperture_Addr = channels_find_by_name("");
        sc2_exposure_time_Addr = channels_find_by_name("");
        sc2_focus_mode_Addr = channels_find_by_name("");
        sc2_start_focus_pos_Addr = channels_find_by_name("");
        sc2_end_focus_pos_Addr = channels_find_by_name("");
        sc2_focus_step_Addr = channels_find_by_name("");
        sc2_photos_per_focus_Addr = channels_find_by_name("");
    }
    SET_SCALED_VALUE(sc2_spike_limit_Addr, scr.blobParams[0]);
    SET_SCALED_VALUE(sc2_dyanmic_hp_Addr, scr.blobParams[1]);
    SET_SCALED_VALUE(sc2_r_smooth_Addr, scr.blobParams[2]);
    SET_SCALED_VALUE(sc2_high_pass_filter_Addr, scr.blobParams[3]);
    SET_SCALED_VALUE(sc2_r_high_pass_filter_Addr, scr.blobParams[4]);
    SET_SCALED_VALUE(sc2_search_border_Addr, scr.blobParams[5]);
    SET_SCALED_VALUE(sc2_filter_return_Addr, scr.blobParams[6]);
    SET_SCALED_VALUE(sc2_n_sigma_Addr, scr.blobParams[7]);
    SET_SCALED_VALUE(sc2_unique_star_spacing_Addr, scr.blobParams[8]);
    SET_SCALED_VALUE(sc2_make_static_hp_mask_Addr, scr.makeHP);
    SET_SCALED_VALUE(sc2_use_static_hp_mask_Addr, scr.useHP);
    SET_SCALED_VALUE(sc2_time_limit_Addr, scr.solveTimeLimit);
    SET_SCALED_VALUE(sc2_logodds_Addr, scr.logOdds);
    SET_SCALED_VALUE(sc2_latitude_Addr, scr.latitude);
    SET_SCALED_VALUE(sc2_longitude_Addr, scr.longitude);
    SET_SCALED_VALUE(sc2_hm_Addr, scr.heightWGS84);
    SET_SCALED_VALUE(sc2_focus_position_Addr, scr.focusPos);
    SET_SCALED_VALUE(sc2_focus_inf_Addr, scr.setFocusInf);
    SET_SCALED_VALUE(sc2_aperture_steps_Addr, scr.apertureSteps);
    SET_SCALED_VALUE(sc2_max_aperture_Addr, scr.maxAperture);
    SET_SCALED_VALUE(sc2_min_focus_pos_Addr, scr.minFocusPos);
    SET_SCALED_VALUE(sc2_max_focus_pos_Addr, scr.maxFocusPos);
    SET_SCALED_VALUE(sc2_current_aperture_Addr, scr.aperture);
    SET_SCALED_VALUE(sc2_exposure_time_Addr, scr.exposureTime);
    SET_SCALED_VALUE(sc2_focus_mode_Addr, scr.focusMode);
    SET_SCALED_VALUE(sc2_start_focus_pos_Addr, scr.startPos);
    SET_SCALED_VALUE(sc2_end_focus_pos_Addr, scr.endPos);
    SET_SCALED_VALUE(sc2_focus_step_Addr, scr.focusStep);
    SET_SCALED_VALUE(sc2_photos_per_focus_Addr, scr.photosPerStep);
}

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

// Now that we have the general boilerplate and useful functions out of the way
// I will add the thread function that
