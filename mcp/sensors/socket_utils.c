/* 
 * socket_utils.c: 
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

#include "socket_utils.h"


int populateSocketData(char * ipaddr, char * port, struct socketData *data) {
    int ret = 0;
    ret = snprintf(data->ipAddr, sizeof(data->ipAddr), "%s", ipaddr);
    if (ret < 0) {
        blast_err("Could not populate socket data struct address");
        return ret;
    }
    ret = snprintf(data->port, sizeof(data->port), "%s", port);
    if (ret < 0) {
        blast_err("Could not populate socket data struct port");
        return ret;
    }
    return ret;
}
