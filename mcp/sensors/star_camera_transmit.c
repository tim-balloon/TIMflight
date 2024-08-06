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

#include "socket_utils.h"
#include "star_camera_transmit.h"
#include "command_struct.h"

struct star_cam_capture sc1_command_packet;
struct star_cam_capture sc2_command_packet;


int which_fc_am_i(void) {
    // check to see if FC2
    if (SouthIAm) {
        return 2;
    } else {
        return 1;
    }
}


/**
 * @brief This function trawls the SC1 command data variables
 * and places the correct values in the data packet
 * to send to the star camera.
 * return value indicates that a packet is ready to send
 * and will modify a thread variable to let it know to send
 * 
 * @return int = 1 to change a status variable once the function finishes
 */
static int prepare_command_packet_sc1(void) {
    // just start populating fields
    sc1_command_packet.fc = which_fc_am_i();
    sc1_command_packet.inCharge = InCharge;
    snprintf(sc1_command_packet.target, sizeof(sc1_command_packet.target), "%s", SC1_IP_ADDR);
    sc1_command_packet.logOdds = CommandData.sc1_commands.logOdds;
    sc1_command_packet.update_logOdds = CommandData.sc1_commands.update_logOdds;
    sc1_command_packet.latitude = CommandData.sc1_commands.latitude;
    sc1_command_packet.update_lat = CommandData.sc1_commands.update_lat;
    sc1_command_packet.longitude = CommandData.sc1_commands.longitude;
    sc1_command_packet.update_lon = CommandData.sc1_commands.update_lon;
    sc1_command_packet.heightWGS84 = CommandData.sc1_commands.heightWGS84;
    sc1_command_packet.update_height = CommandData.sc1_commands.update_height;
    sc1_command_packet.exposureTime = CommandData.sc1_commands.exposureTime;
    sc1_command_packet.update_exposureTime = CommandData.sc1_commands.update_exposureTime;
    sc1_command_packet.solveTimeLimit = CommandData.sc1_commands.solveTimeLimit;
    sc1_command_packet.update_solveTimeLimit = CommandData.sc1_commands.update_solveTimeLimit;
    sc1_command_packet.focusPos = CommandData.sc1_commands.focusPos;
    sc1_command_packet.update_focusPos = CommandData.sc1_commands.update_focusPos;
    sc1_command_packet.focusMode = CommandData.sc1_commands.focusMode;
    sc1_command_packet.update_focusMode = CommandData.sc1_commands.update_focusMode;
    sc1_command_packet.startPos = CommandData.sc1_commands.startPos;
    sc1_command_packet.update_startPos = CommandData.sc1_commands.update_startPos;
    sc1_command_packet.endPos = CommandData.sc1_commands.endPos;
    sc1_command_packet.update_endPos = CommandData.sc1_commands.update_endPos;
    sc1_command_packet.focusStep = CommandData.sc1_commands.focusStep;
    sc1_command_packet.update_focusStep = CommandData.sc1_commands.update_focusStep;
    sc1_command_packet.photosPerStep = CommandData.sc1_commands.photosPerStep;
    sc1_command_packet.update_photosPerStep = CommandData.sc1_commands.update_photosPerStep;
    sc1_command_packet.setFocusInf = CommandData.sc1_commands.setFocusInf;
    sc1_command_packet.update_setFocusInf = CommandData.sc1_commands.update_setFocusInf;
    sc1_command_packet.apertureSteps = CommandData.sc1_commands.apertureSteps;
    sc1_command_packet.update_apertureSteps = CommandData.sc1_commands.update_apertureSteps;
    sc1_command_packet.maxAperture = CommandData.sc1_commands.maxAperture;
    sc1_command_packet.update_maxAperture = CommandData.sc1_commands.update_maxAperture;
    sc1_command_packet.makeHP = CommandData.sc1_commands.makeHP;
    sc1_command_packet.update_makeHP = CommandData.sc1_commands.update_makeHP;
    sc1_command_packet.useHP = CommandData.sc1_commands.useHP;
    sc1_command_packet.update_useHP = CommandData.sc1_commands.update_useHP;
    sc1_command_packet.blobParams[0] = CommandData.sc1_commands.blobParams[0];
    sc1_command_packet.update_blobParams[0] = CommandData.sc1_commands.update_blobParams[0];
    sc1_command_packet.blobParams[1] = CommandData.sc1_commands.blobParams[1];
    sc1_command_packet.update_blobParams[1] = CommandData.sc1_commands.update_blobParams[1];
    sc1_command_packet.blobParams[2] = CommandData.sc1_commands.blobParams[2];
    sc1_command_packet.update_blobParams[2] = CommandData.sc1_commands.update_blobParams[2];
    sc1_command_packet.blobParams[3] = CommandData.sc1_commands.blobParams[3];
    sc1_command_packet.update_blobParams[3] = CommandData.sc1_commands.update_blobParams[3];
    sc1_command_packet.blobParams[4] = CommandData.sc1_commands.blobParams[4];
    sc1_command_packet.update_blobParams[4] = CommandData.sc1_commands.update_blobParams[4];
    sc1_command_packet.blobParams[5] = CommandData.sc1_commands.blobParams[5];
    sc1_command_packet.update_blobParams[5] = CommandData.sc1_commands.update_blobParams[5];
    sc1_command_packet.blobParams[6] = CommandData.sc1_commands.blobParams[6];
    sc1_command_packet.update_blobParams[6] = CommandData.sc1_commands.update_blobParams[6];
    sc1_command_packet.blobParams[7] = CommandData.sc1_commands.blobParams[7];
    sc1_command_packet.update_blobParams[7] = CommandData.sc1_commands.update_blobParams[7];
    sc1_command_packet.blobParams[8] = CommandData.sc1_commands.blobParams[8];
    sc1_command_packet.update_blobParams[8] = CommandData.sc1_commands.update_blobParams[8];
    sc1_command_packet.trigger_mode = CommandData.sc1_commands.trigger_mode;
    sc1_command_packet.update_trigger_mode = CommandData.sc1_commands.update_trigger_mode;
    sc1_command_packet.trigger_timeout_us = CommandData.sc1_commands.trigger_timeout_us;
    sc1_command_packet.update_trigger_timeout_us = CommandData.sc1_commands.update_trigger_timeout_us;
    return 1;
}


/**
 * @brief This function trawls the SC1 command data variables
 * and places the correct values in the data packet
 * to send to the star camera.
 * return value indicates that a packet is ready to send
 * and will modify a thread variable to let it know to send
 * 
 * @return int = 1 to let the thread function know a packet is ready
 */
static int prepare_command_packet_sc2(void) {
    // just start populating fields
    sc2_command_packet.fc = which_fc_am_i();
    sc2_command_packet.inCharge = InCharge;
    snprintf(sc2_command_packet.target, sizeof(sc2_command_packet.target), "%s", SC2_IP_ADDR);
    sc2_command_packet.logOdds = CommandData.sc2_commands.logOdds;
    sc2_command_packet.update_logOdds = CommandData.sc2_commands.update_logOdds;
    sc2_command_packet.latitude = CommandData.sc2_commands.latitude;
    sc2_command_packet.update_lat = CommandData.sc2_commands.update_lat;
    sc2_command_packet.longitude = CommandData.sc2_commands.longitude;
    sc2_command_packet.update_lon = CommandData.sc2_commands.update_lon;
    sc2_command_packet.heightWGS84 = CommandData.sc2_commands.heightWGS84;
    sc2_command_packet.update_height = CommandData.sc2_commands.update_height;
    sc2_command_packet.exposureTime = CommandData.sc2_commands.exposureTime;
    sc2_command_packet.update_exposureTime = CommandData.sc2_commands.update_exposureTime;
    sc2_command_packet.solveTimeLimit = CommandData.sc2_commands.solveTimeLimit;
    sc2_command_packet.update_solveTimeLimit = CommandData.sc2_commands.update_solveTimeLimit;
    sc2_command_packet.focusPos = CommandData.sc2_commands.focusPos;
    sc2_command_packet.update_focusPos = CommandData.sc2_commands.update_focusPos;
    sc2_command_packet.focusMode = CommandData.sc2_commands.focusMode;
    sc2_command_packet.update_focusMode = CommandData.sc2_commands.update_focusMode;
    sc2_command_packet.startPos = CommandData.sc2_commands.startPos;
    sc2_command_packet.update_startPos = CommandData.sc2_commands.update_startPos;
    sc2_command_packet.endPos = CommandData.sc2_commands.endPos;
    sc2_command_packet.update_endPos = CommandData.sc2_commands.update_endPos;
    sc2_command_packet.focusStep = CommandData.sc2_commands.focusStep;
    sc2_command_packet.update_focusStep = CommandData.sc2_commands.update_focusStep;
    sc2_command_packet.photosPerStep = CommandData.sc2_commands.photosPerStep;
    sc2_command_packet.update_photosPerStep = CommandData.sc2_commands.update_photosPerStep;
    sc2_command_packet.setFocusInf = CommandData.sc2_commands.setFocusInf;
    sc2_command_packet.update_setFocusInf = CommandData.sc2_commands.update_setFocusInf;
    sc2_command_packet.apertureSteps = CommandData.sc2_commands.apertureSteps;
    sc2_command_packet.update_apertureSteps = CommandData.sc2_commands.update_apertureSteps;
    sc2_command_packet.maxAperture = CommandData.sc2_commands.maxAperture;
    sc2_command_packet.update_maxAperture = CommandData.sc2_commands.update_maxAperture;
    sc2_command_packet.makeHP = CommandData.sc2_commands.makeHP;
    sc2_command_packet.update_makeHP = CommandData.sc2_commands.update_makeHP;
    sc2_command_packet.useHP = CommandData.sc2_commands.useHP;
    sc2_command_packet.update_useHP = CommandData.sc2_commands.update_useHP;
    sc2_command_packet.blobParams[0] = CommandData.sc2_commands.blobParams[0];
    sc2_command_packet.update_blobParams[0] = CommandData.sc2_commands.update_blobParams[0];
    sc2_command_packet.blobParams[1] = CommandData.sc2_commands.blobParams[1];
    sc2_command_packet.update_blobParams[1] = CommandData.sc2_commands.update_blobParams[1];
    sc2_command_packet.blobParams[2] = CommandData.sc2_commands.blobParams[2];
    sc2_command_packet.update_blobParams[2] = CommandData.sc2_commands.update_blobParams[2];
    sc2_command_packet.blobParams[3] = CommandData.sc2_commands.blobParams[3];
    sc2_command_packet.update_blobParams[3] = CommandData.sc2_commands.update_blobParams[3];
    sc2_command_packet.blobParams[4] = CommandData.sc2_commands.blobParams[4];
    sc2_command_packet.update_blobParams[4] = CommandData.sc2_commands.update_blobParams[4];
    sc2_command_packet.blobParams[5] = CommandData.sc2_commands.blobParams[5];
    sc2_command_packet.update_blobParams[5] = CommandData.sc2_commands.update_blobParams[5];
    sc2_command_packet.blobParams[6] = CommandData.sc2_commands.blobParams[6];
    sc2_command_packet.update_blobParams[6] = CommandData.sc2_commands.update_blobParams[6];
    sc2_command_packet.blobParams[7] = CommandData.sc2_commands.blobParams[7];
    sc2_command_packet.update_blobParams[7] = CommandData.sc2_commands.update_blobParams[7];
    sc2_command_packet.blobParams[8] = CommandData.sc2_commands.blobParams[8];
    sc2_command_packet.update_blobParams[8] = CommandData.sc2_commands.update_blobParams[8];
    sc2_command_packet.trigger_mode = CommandData.sc2_commands.trigger_mode;
    sc2_command_packet.update_trigger_mode = CommandData.sc2_commands.update_trigger_mode;
    sc2_command_packet.trigger_timeout_us = CommandData.sc2_commands.trigger_timeout_us;
    sc2_command_packet.update_trigger_timeout_us = CommandData.sc2_commands.update_trigger_timeout_us;
    // now that everything is yoinked from the command data we just return 1 to let it know we can send
    return 1;
}


/**
 * @brief To be called after we pack the packet so we make sure that
 * no rogue parameters are hanging around to muck it up
 */
static void clear_command_data_sc1(void) {
    memset(&CommandData.sc1_commands, 0, sizeof(CommandData.sc1_commands));
}


/**
 * @brief To be called after we pack the packet so we make sure that
 * no rogue parameters are hanging around to muck it up
 */
static void clear_command_data_sc2(void) {
    memset(&CommandData.sc2_commands, 0, sizeof(CommandData.sc2_commands));
}


/**
 * @brief To be called after transmitting the packet to make sure that
 * we don't have old packet data sticking around (in charge??)
 */
static void clear_packet_data_sc1(void) {
    memset(&sc1_command_packet, 0, sizeof(sc1_command_packet));
}


/**
 * @brief To be called after transmitting the packet to make sure that
 * we don't have old packet data sticking around (in charge??)
 */
static void clear_packet_data_sc2(void) {
    memset(&sc2_command_packet, 0, sizeof(sc2_command_packet));
}


/**
 * @brief points the talker thread at the right commands to watch
 * 
 * @param which which star camera we are talking to
 * @return int lets us know if we should be sending a command packet or not
 */
static int check_sc_send_commands(int which) {
    if (which == 1) {
        return CommandData.sc1_commands.send_commands;
    } else if (which == 2) {
        return CommandData.sc2_commands.send_commands;
    } else {
        blast_err("Invalid star camera to check commands %d", which);
        return -1;
    }
}


/**
 * @brief points the talker thread at the right status boolean to watch
 * 
 * @param which which star camera are we talking to, 1 or 2
 * @return int check to see if we want to gracefully kill the thread
 */
int check_sc_thread_bool(int which) {
    if (which == 1) {
        return CommandData.sc_bools.sc1_command_bool;
    } else if (which == 2) {
        return CommandData.sc_bools.sc2_command_bool;
    } else {
        blast_err("Invalid star camera to check thread bool %d", which);
        return -1;
    }
}


/**
 * @brief allows the thread to be mostly star camera # agnostic and use
 * the # information to prepare the correct packet
 * 
 * @param which which star camera we are talking to
 * @return int return the value letting the thread know the packet is ready
 */
static int prepare_packet(int which) {
    if (which == 1) {
        return prepare_command_packet_sc1();
    } else if (which == 2) {
        return prepare_command_packet_sc2();
    } else {
        blast_err("Invalid star camera to pack packets %d", which);
        return -1;
    }
}


/**
 * @brief star camera # agnostic function to clear old data from packets
 * 
 * @param which star camera number
 */
static void clear_structs(int which) {
    if (which == 1) {
        clear_command_data_sc1();
        clear_packet_data_sc1();
        return;
    } else if (which == 2) {
        clear_command_data_sc2();
        clear_packet_data_sc2();
        return;
    } else {
        blast_err("Invalid star camera to clear structs %d", which);
        return;
    }
}


/**
 * @brief function to check for command flags to reset the star camera
 * UDP socket if we want to do it nicely without swapping computers
 * 
 * @param which star camera number
 * @return int flag to let us know if we need to reset the socket
 */
int check_for_reset(int which) {
    if (which == 1 && CommandData.sc_resets.reset_sc1_comm) {
        // reset the flag
        CommandData.sc_resets.reset_sc1_comm = 0;
        return 1;
    } else if (which == 2 && CommandData.sc_resets.reset_sc2_comm) {
        CommandData.sc_resets.reset_sc2_comm = 0;
        return 1;
    } else {
        return 0;
    }
}


/**
 * @brief function that forces the command boolean that controls the 
 * sending socket to be 1 so that we continue sending
 * 
 */
void reset_command_bools(void) {
    CommandData.sc_bools.sc1_command_bool = 1;
    CommandData.sc_bools.sc2_command_bool = 1;
}

/**
 * @brief helper function that is called in the sending loop to 
 * append the current gondola position to the command packet and 
 * continuously update the SC position information.
 * 
*/
static void update_gondola_position_sc1(void) {
    static int first_time = 1;
    static channel_t *lat_Addr, *lon_Addr, *alt_Addr;
    float lat, lon, alt;
    if (first_time) {
        first_time = 0;
        lat_Addr = channels_find_by_name("lat");
        lon_Addr = channels_find_by_name("lon");
        alt_Addr = channels_find_by_name("alt");
    }
    GET_SCALED_VALUE(lat_Addr, lat);
    GET_SCALED_VALUE(lon_Addr, lon);
    GET_SCALED_VALUE(alt_Addr, alt);
    sc1_command_packet.fc = which_fc_am_i();
    sc1_command_packet.inCharge = InCharge;
    snprintf(sc1_command_packet.target, sizeof(sc1_command_packet.target), "%s", SC1_IP_ADDR);
    sc1_command_packet.latitude = lat;
    sc1_command_packet.update_lat = 1;
    sc1_command_packet.longitude = lon;
    sc1_command_packet.update_lon = 1;
    sc1_command_packet.heightWGS84 = alt;
    sc1_command_packet.update_height = 1;
}

/**
 * @brief helper function that is called in the sending loop to 
 * append the current gondola position to the command packet and 
 * continuously update the SC position information.
 * 
*/
static void update_gondola_position_sc2(void) {
    static int first_time = 1;
    static channel_t *lat_Addr, *lon_Addr, *alt_Addr;
    float lat, lon, alt;
    if (first_time) {
        first_time = 0;
        lat_Addr = channels_find_by_name("lat");
        lon_Addr = channels_find_by_name("lon");
        alt_Addr = channels_find_by_name("alt");
    }
    GET_SCALED_VALUE(lat_Addr, lat);
    GET_SCALED_VALUE(lon_Addr, lon);
    GET_SCALED_VALUE(alt_Addr, alt);
    sc2_command_packet.fc = which_fc_am_i();
    sc2_command_packet.inCharge = InCharge;
    snprintf(sc2_command_packet.target, sizeof(sc2_command_packet.target), "%s", SC2_IP_ADDR);
    sc2_command_packet.latitude = lat;
    sc2_command_packet.update_lat = 1;
    sc2_command_packet.longitude = lon;
    sc2_command_packet.update_lon = 1;
    sc2_command_packet.heightWGS84 = alt;
    sc2_command_packet.update_height = 1;
}

/**
 * @brief allows the thread to be mostly star camera # agnostic and use
 * the # information to append to the correct packet
 * 
 * @param which which star camera we are talking to
 */
static void append_position(int which) {
    if (which == 1) {
        return update_gondola_position_sc1();
    } else if (which == 2) {
        return update_gondola_position_sc2();
    } else {
        blast_err("Invalid star camera to pack packets %d", which);
    }
}


/**
 * @brief our omnibus function for vomiting the packets to the star camera
 * 
 * @param args p_threads take a void pointer that must be typecast to the
 * appropriate argument after resolving the threaded function call. In this case it takes type
 * struct socketData * which must be populated with the IP address and port of the star camera
 * to talk to
 * @return void* We just return null at the end, another p_thread requirement that this is a pointer
 */
void *star_camera_command_thread(void *args) {
    struct socketData * socket_target = args;
    static int first_time = 1;
    int sleep_interval_usec = 200000;
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
    struct star_cam_capture *scCommands;
    int sending = 1;
    int which_sc;
    int packet_status = 0;
    // set the status of this socket to 1 (started)
    if (!strcmp(socket_target->ipAddr, SC1_IP_ADDR)) {
        scCommands = &sc1_command_packet;
        which_sc = 1;
        snprintf(message_str, sizeof(message_str), "%s", "SC1 command thread");
        blast_info("Command socket pointed at SC1\n");
    } else if (!strcmp(socket_target->ipAddr, SC2_IP_ADDR)) {
        scCommands = &sc2_command_packet;
        which_sc = 2;
        snprintf(message_str, sizeof(message_str), "%s", "SC2 command thread");
        blast_info("Command socket pointed at SC2\n");
    } else {
        blast_info("IP provided is %s, expected is %s or %s\n", socket_target->ipAddr, SC1_IP_ADDR, SC2_IP_ADDR);
        blast_err("Invalid ip target for star camera\n");
        return NULL;
    }
    while (sending) {
        sending = check_sc_thread_bool(which_sc);
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
        // Here we move on the main portion of the data comms
        // check to see if a command packet has been packed
        if (check_sc_send_commands(which_sc)) {
            packet_status = prepare_packet(which_sc);
        }
        if (CommandData.update_position_sc) {
            append_position(which_sc);
            packet_status = 1;
            // blast_info("current positions from channels are:\nlat = %f\nlon = %f\nalt = %f",
            // sc1_command_packet.latitude, sc1_command_packet.longitude, sc1_command_packet.heightWGS84);
        }
        if (packet_status) {
            if (!strcmp(scCommands->target, ipAddr)) {
                packet_status = 0;
                length = sizeof(*scCommands);
                if ((bytes_sent = sendto(sockfd, scCommands, length, 0,
                    servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                    perror("talker: sendto");
                }
                // blast_info("%s sent packet to %s:%s\n", message_str, ipAddr, socket_target->port);
                // clear the commands packet
                // blast_info("packet incharge is %d, real variable is %d\n", scCommands->inCharge, InCharge);
                clear_structs(which_sc);
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
    reset_command_bools();
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
}
