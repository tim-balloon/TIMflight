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
 
 created by Ian Lowe 5-23-23
 **************************************************************************/


/*************************************************************************
 
 star_camera_solutions.c -- mcp code to receive image solutions from SC1 and SC2
 
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
#include "command_struct.h"

// The two below functions serve the same purpose as above
// but, instead of camera parameters, they store image solution data
static void assign_solution_data_to_channel_sc1(struct mcp_astrometry scm) {
    static int first_time = 1;
    static channel_t * sc1_rawtime_Addr, * sc1_ra_Addr, * sc1_dec_Addr;
    static channel_t * sc1_image_rms_Addr, * sc1_fr_Addr, * sc1_ps_Addr;
    static channel_t * sc1_ir_Addr, * sc1_alt_Addr, * sc1_az_Addr;
    static channel_t * sc1_photo_time_Addr, * sc1_ra_j2000_Addr, * sc1_dec_j2000_Addr;
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
        sc1_photo_time_Addr = channels_find_by_name("sc1_photo_time");
        sc1_ra_j2000_Addr = channels_find_by_name("sc1_ra_j2000");
        sc1_dec_j2000_Addr = channels_find_by_name("sc1_dec_j2000");
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
    SET_SCALED_VALUE(sc1_photo_time_Addr, scm.photo_time);
    SET_SCALED_VALUE(sc1_ra_j2000_Addr, scm.ra_j2000);
    SET_SCALED_VALUE(sc1_dec_j2000_Addr, scm.dec_j2000);
}

static void assign_solution_data_to_channel_sc2(struct mcp_astrometry scm) {
    static int first_time = 1;
    static channel_t * sc2_rawtime_Addr, * sc2_ra_Addr, * sc2_dec_Addr;
    static channel_t * sc2_image_rms_Addr, * sc2_fr_Addr, * sc2_ps_Addr;
    static channel_t * sc2_ir_Addr, * sc2_alt_Addr, * sc2_az_Addr;
    static channel_t * sc2_photo_time_Addr, * sc2_ra_j2000_Addr, * sc2_dec_j2000_Addr;
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
        sc2_photo_time_Addr = channels_find_by_name("sc2_photo_time");
        sc2_ra_j2000_Addr = channels_find_by_name("sc2_ra_j2000");
        sc2_dec_j2000_Addr = channels_find_by_name("sc2_dec_j2000");
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
    SET_SCALED_VALUE(sc2_photo_time_Addr, scm.photo_time);
    SET_SCALED_VALUE(sc2_ra_j2000_Addr, scm.ra_j2000);
    SET_SCALED_VALUE(sc2_dec_j2000_Addr, scm.dec_j2000);
}

static int which_sc(struct socket_data socket_setup) {
    if (strcmp(socket_setup.ipAddr, SC1_IP_ADDR) == 0) {
        return 1;
    } else if (strcmp(socket_setup.ipAddr, SC2_IP_ADDR) == 0) {
        return 2;
    } else {
        return -1;
    }
}

static void where_to_unpack(struct mcp_astrometry data, int which_sc) {
    if (which_sc == 1) {
        assign_solution_data_to_channel_sc1(data);
        return;
    } else if (which_sc == 2) {
        assign_solution_data_to_channel_sc2(data);
        return;
    } else {
        blast_err("Invalid star camera address provided to decider");
    }
}

// points the listening thread at the right status boolean to watch
static int check_sc_thread_bool(int which) {
    if (which == 1) {
        return CommandData.sc_bools.sc1_image_bool;
    } else if (which == 2) {
        return CommandData.sc_bools.sc2_image_bool;
    } else {
        blast_err("Invalid star camera to check thread bool %d", which);
        return -1;
    }
}

static int check_for_reset(int which) {
    if (which == 1 && CommandData.sc_resets.reset_sc1_image) {
        // reset the flag
        CommandData.sc_resets.reset_sc1_image = 0;
        return 1;
    } else if (which == 2 && CommandData.sc_resets.reset_sc2_image) {
        CommandData.sc_resets.reset_sc2_image = 0;
        return 1;
    } else {
        return 0;
    }
}

void *image_receive_thread(void *args) {
    struct socket_data * socket_target = args;
    int first_time = 1;
    int sockfd;
    struct addrinfo hints, *servinfo, *servinfoCheck;
    int returnval;
    int numbytes;
    int listening = 1;
    int which_sc;
    struct sockaddr_storage their_addr;
    struct mcp_astrometry received_solutions;
    socklen_t addr_len;
    char ipAddr[INET_ADDRSTRLEN];
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to use IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP
    int err;
    if (!strcmp(socket_target->ipAddr, SC1_IP_ADDR)) {
        which_sc = 1;
        blast_info("Image socket pointed at SC1\n");
    } else if (!strcmp(socket_target->ipAddr, SC2_IP_ADDR)) {
        which_sc = 2;
        blast_info("Image socket pointed at SC2\n");
    } else {
        blast_info("Invalid IP target for data source\n");
        return NULL;
    }
    while (listening) {
        listening = check_sc_thread_bool(which_sc);
        if (first_time) {
            first_time = 0;
            if ((returnval = getaddrinfo(NULL, socket_target->port, &hints, &servinfo)) != 0) {
                blast_err("getaddrinfo: %s\n", gai_strerror(returnval));
                return NULL;
            }
            // loop through all the results and bind to the first we can
            for (servinfoCheck = servinfo; servinfoCheck != NULL; servinfoCheck = servinfoCheck->ai_next) {
                // since a success needs both of these, but it is inefficient to do fails twice,
                // continue skips the second if on a fail
                if ((sockfd = socket(servinfoCheck->ai_family, servinfoCheck->ai_socktype,
                        servinfoCheck->ai_protocol)) == -1) {
                    perror("listener: socket");
                    continue;
                }
                if (bind(sockfd, servinfoCheck->ai_addr, servinfoCheck->ai_addrlen) == -1) {
                    close(sockfd);
                    perror("listener: bind");
                    continue;
                }
                break;
            }
            if (servinfoCheck == NULL) {
                blast_err("listener: failed to bind socket\n");
                return NULL;
            }
            // set the read timeout (if there isnt a message) to 100ms
            struct timeval read_timeout;
            read_timeout.tv_sec = 0;
            read_timeout.tv_usec = 100000;
            setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout);
            // now we set up the print statement vars
            // need to cast the socket address to an INET still address
            struct sockaddr_in *ipv = (struct sockaddr_in *)servinfo->ai_addr;
            // then pass the pointer to translation and put it in a string
            inet_ntop(AF_INET, &(ipv->sin_addr), ipAddr, INET_ADDRSTRLEN);
            blast_info("Image receiving target is: %s\n", socket_target->ipAddr);
        }
        numbytes = recvfrom(sockfd, &received_solutions, sizeof(struct mcp_astrometry)+1 ,
         0, (struct sockaddr *)&their_addr, &addr_len);
        // we get an error everytime it times out, but EAGAIN is ok, other ones are bad.
        if (numbytes == -1) {
            err = errno;
            if (err != EAGAIN) {
                blast_err("Errno is %d\n", err);
                blast_err("Error is %s\n", strerror(err));
                perror("Recvfrom");
            }
        } else {
            where_to_unpack(received_solutions, which_sc);
        }
        if (check_for_reset(which_sc)) {
            first_time = 1;
            freeaddrinfo(servinfo);
            close(sockfd);
        } else {
            usleep(200000);
        }
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
}

