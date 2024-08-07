/* mcp: the master control program
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of mcp.
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
 */

#ifndef INCLUDE_MCP_H
#define INCLUDE_MCP_H

#include <pthread.h>
#include <stdbool.h>
#include "channels_tng.h"
#include "calibrate.h"
#include "blast.h"

extern time_t mcp_systime(time_t *t);
extern struct frameBuffer hiGain_buffer;
extern bool shutdown_mcp;

// TODO(seth): Move to buffer header file
#define GETREADINDEX(i) ((i+2) % 3)  /* i - 1 modulo 3 */
#define INC_INDEX(i) ((i + 1) %3)    /* i + 1 modulo 3 */

struct chat_buf {
  char msg[4][2 * 20]; /* 4 buffers of FAST_PER_SLOW BLASTbus words of characters */
  int reading; /* the buffer we're currently reading from */
  int writing; /* the buffer we're currently writing to */
};

extern struct tm start_time;
void force_incharge(void);

#define DEFAULT_INCHARGE !SouthIAm

// telemetry defines

// how many options we have
#define NUM_TELEMETRIES 6
// iridium pilot index
#define PILOT_TELEMETRY_INDEX 0
// biphase index
#define BI0_TELEMETRY_INDEX 1
// tdrss highrate index
#define HIGHRATE_TELEMETRY_INDEX 2
// science burst data index
#define SBD_TELEMETRY_INDEX 3
#define EVTM_LOS_TELEMETRY_INDEX 4
#define EVTM_TDRSS_TELEMETRY_INDEX 5

// Max Slew Veto (how long you can timeout slewing)
#define VETO_MAX 60000

#define TEMPORAL_OFFSET 0

#define MAX_LINE_LENGTH 1024

extern int16_t SouthIAm;
extern int16_t InCharge;
extern int16_t InChargeSet;

extern int ResetLog;

// #define USE_XY_THREAD /* TODO(lmf): Comment out (or remove) for flight */
#endif
