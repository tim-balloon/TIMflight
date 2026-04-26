/* 
 * cryo_tauhk_cmd.c: interface for cryo commanding via TauHK
 * 
 * This software  is copyright 
 *  (C) University of Pennsylvania, Philadelphia 2026
 *
 * This file is part of mcp, as used for the Terahertz Intensity Mapper (TIM).
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: April 24, 2026 by Shubh Agrawal
 */

#include <math.h>
#include <stdio.h>
#include <netdb.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <sys/socket.h> // socket stuff
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>

#include "socket_utils.h"
#include "command_struct.h"
#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "cryo_tauhk_cmd.h"
#include "blast.h"

extern int16_t InCharge;

/**
 * @brief copy over command into a packet to send to cryo commander
 * copied from rfsoc commanding code
 * @param packet packet that I want to fill and 
 * @param commands pointer to the  commands substructure in commanddata (set by thread)
 */
void generate_command_packet(struct cryo_command_data* packet, cryo_command_t* commands) {
    packet->incharge = InCharge;
    packet->command_num = commands->command;
    packet->param1 = commands->param1;
    packet->param2 = commands->param2;
    packet->param3 = commands->param3;
    packet->param4 = commands->param4;
    packet->param5 = commands->param5;
    reset_command_packet(commands);
};

/**
 * @brief to be called after sending the packet to ensure we don't send multiple copies
 * copied from rfsoc commanding code
 * @param packet packet to reset to 0
 */
void reset_command_packet(struct cryo_command_data* packet) {
    memset(packet, 0, sizeof(*packet));
}

/**
 * @brief 
 * 
 * @param commands pointer to the associated commanddata substruct that contains
 * the cryo commanding information, copied from rfsoc commanding code
 * @return int 
 */
int check_command_ready(cryo_command_t* commands) {
    if (commands->command_ready == 1) {
        commands->command_ready = 0;
        return 1;
    }
    return 0;
};

// here goes the actual thread that we call
void * send_cryo_commands(void* args) {
    struct cryo_command_data cryo_packet;
    cryo_command_t* command_pointer;
    struct socketData * socket_target = args;
    int sleep_interval_usec = 500000; // sleep for 0.5 seconds between attempts to send packets
    int first_time = 1;
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
    int which_sc;
    int packet_status = 0;
    // check which fc I am and set my pointer properly
    command_pointer = &CommandData.cryo_command;
    while (1) {
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
        packet_status = check_command_ready(command_pointer);
        if (packet_status == 1) {
            blast_info("Got a cryo command packet,  going into send!\n");
        }
        if (packet_status) {
            generate_command_packet(&cryo_packet, command_pointer);
            if (!strcmp(socket_target->ipAddr, ipAddr)) {
                packet_status = 0;
                length = sizeof(cryo_packet);
                if ((bytes_sent = sendto(sockfd, &cryo_packet, length, 0,
                    servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                    perror("talker: sendto");
                }
                blast_info("sent packet to %s port %s\n", socket_target->ipAddr, socket_target->port);
                blast_info("data was %i %i %f %f %f %f %f", cryo_packet.incharge,
                    cryo_packet.command_num, cryo_packet.param1, cryo_packet.param2, cryo_packet.param3,
                    cryo_packet.param4, cryo_packet.param5);
            } else {
                blast_err("Target destination %s differs from thread target %s.\n", socket_target->ipAddr, ipAddr);
            }
        }
        usleep(sleep_interval_usec);
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
};
