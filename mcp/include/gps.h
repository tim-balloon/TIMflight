/**
 * @file gps.h
 *
 * @date Mar 28, 2018
 * @author javier
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2011-2015 University of Pennsylvania
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_GPS_H_
#define INCLUDE_GPS_H_


/**
 * @brief GPS data structure containing the positioning as well as QC data
 * 
 */
struct GPSInfoStruct {
  double latitude; // [deg] +ve north
  double longitude; // [deg] +ve east
  double altitude; // [m] +ve up
  int num_sat; // [] number of satellites
  int quality; // [] GPS quality

  int isnew; // [] whether or not a GPS solution is new
  int reading;
};

// in gps.c
extern struct GPSInfoStruct GPSData;
void * GPSMonitor(void *);

#endif
