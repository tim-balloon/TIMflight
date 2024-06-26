/* 
 * data_sharing_server.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
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
 * Created on: Mar 13, 2018 by Javier Romualdez
 */

#ifndef INCLUDE_DATA_SHARING_SERVER_H_
#define INCLUDE_DATA_SHARING_SERVER_H_

// TODO(javier): get hostnames instead of IPs

// FC1 IP address (also known as "north")
#define NORTH_IP "192.168.1.3"
// FC2 IP address (also known as "south")
#define SOUTH_IP "192.168.1.4"
// shared data sharing port
#define DATA_SHARING_PORT 42224
// fast shared data sharing port
#define FAST_DATA_SHARING_PORT 42524

void data_sharing_init(linklist_t **);
void share_data(E_RATE);
void share_superframe(uint8_t *);
void send_fast_data();
bool recv_fast_data();

#endif /* INCLUDE_DATA_SHARING_SERVER_H_ */
