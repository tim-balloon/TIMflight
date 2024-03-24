/**
 * @file csbf_dgps.h
 *
 * @date Mar 21, 2024
 * @author evanmayer
 *
 * @brief This file is part of MCP, created for the TIM project
 *
 * This software is copyright (C) 2024 University of Arizona
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef CSBF_DGPS_H_
#define CSBF_DGPS_H_

#include "gps.h"


// converts minutes to degrees
#define GPS_MINS_TO_DEG (1.0/60.0)

// maps DGPS states to integers
typedef enum {
    DGPS_WAIT_FOR_START = 0,
    DGPS_READING_PKT,
} e_dgps_read_status;

// Packages a NMEA sentence handler with it's identifying string
typedef struct {
    void (*proc)(const char*);
    char str[16];
} nmea_handler_t;

extern struct DGPSAttStruct CSBFGPSAz;
extern struct GPSInfoStruct CSBFGPSData;

void * DGPSmonitorSerial(void *);
void * DGPSmonitorUDP(void *);

void StartDGPSmonitors(void);

#endif /* CSBF_DGPS_H_ */
