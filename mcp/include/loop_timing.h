/* 
 * loop_timing.h:
 *
 * This software is copyright (C) 2024 Evan Mayer
 *
 * This file is part of mcp, created for the TIM Balloon Project.
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
 * Created on: Aug 28, 2024 by Evan Mayer
 */

#include <stdint.h>
#include <time.h>

#include "mcp.h"
#include "channel_macros.h"
#include "channels_tng.h"

#ifndef LOOP_TIMING_H
#define LOOP_TIMING_H

struct timespec end_1hz;
int lastNsec_1hz;
int lastSec_1hz;
double deltaTimeSec_1hz;

struct timespec end_2hz;
int lastNsec_2hz;
int lastSec_2hz;
double deltaTimeSec_2hz;

struct timespec end_5hz;
int lastNsec_5hz;
int lastSec_5hz;
double deltaTimeSec_5hz;

struct timespec end_20hz;
int lastNsec_20hz;
int lastSec_20hz;
double deltaTimeSec_20hz;

struct timespec end_80hz;
int lastNsec_80hz;
int lastSec_80hz;
double deltaTimeSec_80hz;

struct timespec end_100hz;
int lastNsec_100hz;
int lastSec_100hz;
double deltaTimeSec_100hz;

struct timespec end_122hz;
int lastNsec_122hz;
int lastSec_122hz;
double deltaTimeSec_122hz;

struct timespec end_200hz;
int lastNsec_200hz;
int lastSec_200hz;
double deltaTimeSec_200hz;

int init_loop_timing(void);
void record_loop_timing(E_RATE caller);

#endif // LOOP_TIMING_H