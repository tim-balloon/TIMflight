/* analysis_comp_sharing.c: TIM threads and functions for sharing pointing with acomp1
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
#include "analysis_comp_sharing.h"
#include "command_struct.h"
#include "tx.h"
#include "channels_tng.h"

int proceed_with_loop_trigger = 0;

/**
 * @brief takes a pointer to a pointing data packet, sets up channel pointers,
 * and fills the packet with the most recent data from each channel
 * 
 * @param packet the pointer to the packet that we will fill from channels
 */
void fill_packet_from_channels(pointing_data* packet) {
    static channel_t *az_Addr, *el_Addr, *lat_Addr, *lon_Addr;
    static channel_t *height_Addr, *time_Addr, *time_usec_Addr;
    static int first_time = 1;
    if (first_time) {
        first_time = 0;
        az_Addr = channels_find_by_name("az");
        el_Addr = channels_find_by_name("el");
        lat_Addr = channels_find_by_name("lat");
        lon_Addr = channels_find_by_name("lon");
        height_Addr = channels_find_by_name("alt");
        time_Addr = channels_find_by_name("time");
        time_usec_Addr = channels_find_by_name("time_usec");
    }
    GET_SCALED_VALUE(az_Addr, packet->az);
    GET_SCALED_VALUE(el_Addr, packet->el);
    GET_SCALED_VALUE(lat_Addr, packet->latitude);
    GET_SCALED_VALUE(lon_Addr, packet->longitude);
    GET_VALUE(height_Addr, packet->height);
    GET_VALUE(time_Addr, packet->time);
    GET_VALUE(time_usec_Addr, packet->time_usec);
}

/**
 * @brief clears the data stored in a pointing packet to ensure old data never leaks in
 * 
 * @param packet the pointer to the packet that we will clear
 */
void clear_packet_data(pointing_data* packet) {
    memset(packet, 0, sizeof(pointing_data));
}

/**
 * @brief triggers the acomp send loop to prepare and 
 * send a packet of pointing data to the acomp
 * has in charge protection to ensure that only the in charge
 * computer is sending the data to the acomp
 * 
 */
void get_acomp_shared_data_1hz(void) {
    if (InCharge) {
        proceed_with_loop_trigger = 1;
    }
}

// here goes the actual thread that we call
void * send_pointing_data_acomp(void* args) {
    pointing_data acomp_packet;
    struct socketData * socket_target = args;
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
        while (!proceed_with_loop_trigger) {
            usleep(500);
        }
        fill_packet_from_channels(&acomp_packet);
        if (!strcmp(socket_target->ipAddr, ipAddr)) {
            length = sizeof(acomp_packet);
            printf("length of packet is %d!!!!!!!\n", length);
            if ((bytes_sent = sendto(sockfd, &acomp_packet, length, 0,
                servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                perror("talker: sendto");
            }
        } else {
            blast_err("Target destination %s differs from thread target %s.\n", socket_target->ipAddr, ipAddr);
        }
        proceed_with_loop_trigger = 0;
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
};

