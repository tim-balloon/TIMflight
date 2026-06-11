/* analysis_comp_trigger.c: TIM threads and functions for triggering data save on acomp1
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
#include "analysis_comp_trigger.h"
#include "command_struct.h"
#include "tx.h"
#include "channels_tng.h"

/**
 * @brief function that goes in the 1hz loop to perform a delayed stop taking data countdown.
 * 
 */
void acomp_countdown_1hz(void) {
    if (CommandData.acomp_commands.counting == 1) {
        if (CommandData.acomp_commands.countdown == 0) {
            CommandData.acomp_commands.counting = 0;
            CommandData.acomp_commands.send_trigger = 1;
            CommandData.acomp_commands.trigger_value = 0;
            return;
        }
        CommandData.acomp_commands.countdown--;
    }
}



// here goes the actual thread that we call
void * send_data_trigger_acomp(void* args) {
    struct trigger acomp_trigger;
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
        if ((CommandData.acomp_commands.send_trigger == 1) && InCharge) {
            acomp_trigger.take_data = CommandData.acomp_commands.trigger_value; // pull the value into the packet
            CommandData.acomp_commands.send_trigger = 0; // don't try to send again
            if (!strcmp(socket_target->ipAddr, ipAddr)) {
                length = sizeof(acomp_trigger);
                if ((bytes_sent = sendto(sockfd, &acomp_trigger, length, 0,
                    servinfo->ai_addr, servinfo->ai_addrlen)) == -1) {
                    perror("talker: sendto");
                }
            } else {
                blast_err("Target destination %s differs from thread target %s.\n", socket_target->ipAddr, ipAddr);
            }
        } else {
            usleep(100000);
        }
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
};
