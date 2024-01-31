/***************************************************************************
 mcp: the TIM master control program
 
 This software is copyright (C) 2023 University of Arizona
 
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

#include "star_camera_trigger.h"
#include "star_camera_transmit.h"
#include "command_struct.h"

#define SC_STANDARD_TIMEOUT_SEC 2
#define SC_GYRO_CHECK_INTERVAL_USEC 10000

struct star_cam_trigger sc1_trigger;
struct star_cam_trigger sc2_trigger;

struct timeval trig_timer;

/**
 * @brief checks to see if the azimuth velocity given by
 * the gyros is within the limiting band to take a SC image. We
 * also check for a "forced trigger" event which can be commanded from
 * the ground for the purpose of testing SC or in flight to take many.
 * If we are within the limits we set the trigger to 1, otherwise
 * we set it to zero.
 * 
 * @return int 1 if any pointing data allows for trigger or it is forced, 0 otherwise
 */
static int check_az_vel_data(void) {
    static int first_time = 1;
    if (first_time) {
        first_time = 0;
        sc1_trigger.fc = which_fc_am_i();
        sc1_trigger.incharge = InCharge;
        snprintf(sc1_trigger.target, sizeof(sc1_trigger.target), "%s", SC1_IP_ADDR);
        sc2_trigger.fc = which_fc_am_i();
        sc2_trigger.incharge = InCharge;
        snprintf(sc2_trigger.target, sizeof(sc2_trigger.target), "%s", SC2_IP_ADDR);
    }
    if (CommandData.sc_trigger.force_trigger_starcam == 1) {
        CommandData.sc_trigger.force_trigger_starcam = 0;
        sc1_trigger.trigger = 1;
        sc2_trigger.trigger = 1;
        return 1;
    } else if (AZ_VEL_LIMIT > PointingData[0].gy_az && PointingData[0].gy_az > -AZ_VEL_LIMIT
     && CommandData.sc_trigger.enable_sc_gyro_trigger == 1) {
        sc1_trigger.trigger = 1;
        sc2_trigger.trigger = 1;
        return 1;
    } else if (AZ_VEL_LIMIT > PointingData[1].gy_az && PointingData[1].gy_az > -AZ_VEL_LIMIT
     && CommandData.sc_trigger.enable_sc_gyro_trigger == 1) {
        sc1_trigger.trigger = 1;
        sc2_trigger.trigger = 1;
        return 1;
    } else if (AZ_VEL_LIMIT > PointingData[2].gy_az && PointingData[2].gy_az > -AZ_VEL_LIMIT
     && CommandData.sc_trigger.enable_sc_gyro_trigger == 1) {
        sc1_trigger.trigger = 1;
        sc2_trigger.trigger = 1;
        return 1;
    } else {
        sc1_trigger.trigger = 0;
        sc2_trigger.trigger = 0;
        return 0;
    }
}


/**
 * @brief checks the command data structure for an updated timeout value.
 * 
 * @return int number of seconds to set the timeout to, 2 if
 * we do not update, whatever we tell this if we do update.
 */
static int sc_timeout_update(void) {
    if (CommandData.sc_trigger.starcam_image_timeout_update == 1) {
        CommandData.sc_trigger.starcam_image_timeout_update = 0;
        return CommandData.sc_trigger.starcam_image_timeout;
    } else {
        return SC_STANDARD_TIMEOUT_SEC;
    }
}


/**
 * @brief Sets up the thread which monitors the velocity to send star camera triggers
 * to the star camera computers.
 * 
 * @param args void* cast socket data that contains IP addr and port number to connect to
 */
void *star_camera_trigger_thread(void *args) {
    struct socket_data * socket_target = args;
    static int first_time = 1;
    int sleep_interval_usec = SC_GYRO_CHECK_INTERVAL_USEC; // 10ms intervals to check the gyro data
    int sleep_photo_interval_sec = SC_STANDARD_TIMEOUT_SEC;
    // after the turnaround trigger we wont try to trigger for 2 seconds
    int sockfd;
    struct addrinfo hints;
    struct addrinfo *servinfo;
    struct addrinfo *servinfoCheck;
    int returnval;
    int bytes_sent;
    int length;
    int *retval;
    char message_str[40];
    char ipAddr[INET_ADDRSTRLEN];
    int sending = 1;
    int which_sc;
    int packet_status = 0;
    int skip_sleep = 0; // for skipping sleep if we error out
    struct star_cam_trigger *scTrigger;
    // set the status of this socket to 1 (started)
    if (!strcmp(socket_target->ipAddr, SC1_IP_ADDR)) {
        scTrigger = &sc1_trigger;
        which_sc = 1;
        snprintf(message_str, sizeof(message_str), "%s", "SC1 trigger thread");
        blast_info("Trigger socket pointed at SC1\n");
    } else if (!strcmp(socket_target->ipAddr, SC2_IP_ADDR)) {
        scTrigger = &sc2_trigger;
        which_sc = 2;
        snprintf(message_str, sizeof(message_str), "%s", "SC2 trigger thread");
        blast_info("Trigger socket pointed at SC2\n");
    } else {
        blast_info("IP provided is %s, expected is %s or %s\n", socket_target->ipAddr, SC1_IP_ADDR, SC2_IP_ADDR);
        blast_err("Invalid ip target for star camera\n");
        return NULL;
    }
    while (sending) {
        sending = check_sc_thread_bool(which_sc);
        sleep_photo_interval_sec = sc_timeout_update();
        if (first_time == 1) {
            first_time = 0;
            memset(&hints, 0, sizeof(hints));
            hints.ai_family = AF_INET; // set to AF_INET to use IPv4
            hints.ai_socktype = SOCK_DGRAM;
            // fill out address info and return if it fails.
            if ((returnval = getaddrinfo(socket_target->ipAddr, socket_target->port, &hints, &servinfo)) != 0) {
                blast_err("getaddrinfo: %s\n", gai_strerror(returnval));
                return NULL;
        }
        // now we make a socket with this info
        for (servinfoCheck = servinfo; servinfoCheck != NULL; servinfoCheck = servinfoCheck->ai_next) {
            if ((sockfd = socket(servinfoCheck->ai_family,
             servinfoCheck->ai_socktype, servinfoCheck->ai_protocol)) == -1) {
                perror("talker: socket");
                continue;
            }
            break;
        }
        // check to see if we made a socket
        if (servinfoCheck == NULL) {
            // set status to 0 (dead) if this fails
            blast_err("talker: failed to create socket\n");
            return NULL;
        }
        // if we pass all of these checks then
        // we set up the print statement vars
        // need to cast the socket address to an INET still address
        struct sockaddr_in *ipv = (struct sockaddr_in *)servinfo->ai_addr;
        // then pass the pointer to translation and put it in a string
        inet_ntop(AF_INET, &(ipv->sin_addr), ipAddr, INET_ADDRSTRLEN);
        blast_info("IP target is: %s\n", ipAddr);
        // now the "str" is packed with the IP address string
        // first time setup of the socket is done
        }
        packet_status = check_az_vel_data();
        if (packet_status) {
            if (!strcmp(scTrigger->target, ipAddr)) {
                packet_status = 0;
                length = sizeof(*scTrigger);
                if ((bytes_sent = sendto(sockfd, scTrigger, length, 0,
                    servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                    perror("talker: sendto");
                    skip_sleep = 1;
                }
                if (skip_sleep == 1) {
                    skip_sleep = 0;
                } else {
                    blast_info("%s sent trigger packet to %s:%s\n", message_str, ipAddr, socket_target->port);
                    // testing latency
                    gettimeofday(&trig_timer, NULL);
                    sleep(sleep_photo_interval_sec);
                }
            } else {
                blast_err("Target destination differs from thread target.\n");
            }
        }
        // Here we check to see if we're supposed to reset the socket
        // If we want to reset we just free everything and tell it to do first time again
        // otherwise we just sleep for 0.2 seconds and repeat
        if (check_for_reset(which_sc)) {
            blast_info("received reset command...");
            first_time = 1;
            freeaddrinfo(servinfo);
            close(sockfd);
        } else {
            usleep(sleep_interval_usec);
        }
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
}
