/* 
 * socket_utils.h: 
 *
 * This software is copyright (C) 2024 University of Arizona
 *
 * This file is part of mcp, created for the TIM Project.
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
 * Created on: Apr 4, 2024 by Evan Mayer
 */

#ifndef SOCKET_UTILS_H
#define SOCKET_UTILS_H

#include <stdio.h>
#include <blast.h>

#define SOCKET_IP_LEN 16
#define SOCKET_PORT_LEN 6

// helper for initializing socket connections
struct socketData {
    char ipAddr[SOCKET_IP_LEN];
    char port[SOCKET_PORT_LEN];
};

/**
 * @brief Helper to start threads that need IP/port info passed as arguments
 * 
 * @param ipaddr 
 * @param port 
 * @param data struct into which the IP and port are stored
 * @returns Result of last snprintf call
 */
int populateSocketData(char * ipaddr, char * port, struct socketData *data);

#endif
