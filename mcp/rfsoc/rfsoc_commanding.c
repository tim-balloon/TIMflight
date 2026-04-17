/* rfsoc_commanding.c: TIM threads and functionality for executing
 * the mcp to rfsoc commanding
 *
 * This software is copyright (C) 2026 University of Arizona
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

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
#include "rfsoc_commanding.h"
#include "command_struct.h"

/**
 * @brief returns which FC this is 
 * (should probably be put in a higher up .c /.h because I stole this from myself)
 * 
 * @return int which FC
 */
int which_fc_am_i(void) {
    // check to see if FC2
    if (SouthIAm) {
        return 2;
    } else {
        return 1;
    }
}

/**
 * @brief 
 * 
 * @param packet packet that I want to fill and 
 * @param commands pointer to the correct rfsoc commands substructure in commanddata (set by thread)
 */
void generate_command_packet(struct rfsoc_data* packet, rfsoc_commands_t* commands){
    packet->incharge = InCharge;
    packet->drone_num = commands->drone;
    packet->command_num = commands->command;
    packet->param1 = commands->param1;
    packet->param2 = commands->param2;
    packet->param3 = commands->param3;
    packet->param4 = commands->param4;
    packet->param5 = commands->param5;
};

/**
 * @brief to be called after sending the packet to ensure we don't send multiple copies
 * 
 * @param packet packet to reset to 0
 */
void reset_command_packet(struct rfsoc_data* packet) {
    memset(packet, 0, sizeof(*packet));
}

/**
 * @brief 
 * 
 * @param commands pointer to the associated commanddata substruct that contains
 * the rfsoc commanding information
 * @return int 
 */
int check_command_ready(rfsoc_commands_t* commands) {
    if (commands->command_ready == 1)
    {
        commands->command_ready = 0;
        return 1;
    }
    return 0;
};

// here goes the actual thread that we call
void * send_commands(void* args) {
    struct rfsoc_data rfsoc_packet;
    rfsoc_commands_t* command_pointer;
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
    if (which_fc_am_i() == 1)
    {
        command_pointer = &CommandData.rfsoc_commands1;
        blast_info("I am FC1 and setting up RFSOC commands as such\n");
    }
    else if (which_fc_am_i == 2)
    {
        command_pointer = &CommandData.rfsoc_commands2;
        blast_info("I am FC2 and setting up RFSOC commands as such\n");
    }
    else
    {
        blast_err("Somehow told RFSOC I am neither FC1 nor FC2 ... exiting thread\n");
        return;
    }
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
        if (packet_status) {
            generate_command_packet(&rfsoc_packet, command_pointer);
            if (!strcmp(socket_target->ipAddr, ipAddr)) {
                packet_status = 0;
                length = sizeof(rfsoc_packet);
                if ((bytes_sent = sendto(sockfd, &rfsoc_packet, length, 0,
                    servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                    perror("talker: sendto");
                }
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