/* 
 * actuators.h: 
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
 * Created on: Apr 16, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_ACTUATORS_H_
#define INCLUDE_ACTUATORS_H_
// set the actbus default chatter level
#define ACTBUS_CHATTER	EZ_CHAT_ACT    // EZ_CHAT_ACT (normal) | EZ_CHAT_BUS (debugging)
// Set for the lock pin:
// - n2: limit switch mode
// - an16384: listen to Switch1 and Switch2 for limits
// - f1: limit switch polarity normally open
// - j256: microstep resolution
#define LOCK_PREAMBLE "n2an16384f1j256"
// El axis must be within this distance to engage lock pin
#define LOCK_THRESHOLD_DEG 0.5
#define LOCK_DEFAULT_VEL 110000
// Set for shutter:
// - j256: microstep resolution
// - n2: limit switch mode
#define SHUTTER_PREAMBLE "j256n2"
// Set for secondary actuators:
// - aE25600: encoder/microstep ratio
// - aC50: coarse correction band
// - ac%d: fine correction tolerance
// - au5: stall retries
// - n8: enable encoder feedback mode
// NB: this is a printf template now, requires a move tolerance (ac) to be set, default from BLAST-Pol is 2
// but ac5 has been tested by Peter and Paul
#define ACT_PREAMBLE  "aE25600aC50ac%dau5n8"

void StoreActBus(void);
void *ActuatorBus(void *param);
int GetActAddr(int ind);
uint32_t GetShutterAction(uint32_t* shutter_state, uint32_t* shutter_goal);

#endif /* INCLUDE_ACTUATORS_H_ */
