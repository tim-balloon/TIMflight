/* 
 * cryo_tauhk_cmd.h: interface for cryo commanding via TauHK
 * 
 * This software  is copyright 
 *  (C) University of Pennsylvania, Philadelphia 2026
 *
 * This file is part of mcp, as used for the Terahertz Intensity Mapper (TIM).
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
 * Created on: April 24, 2026 by Shubh Agrawal
 */


#ifndef INCLUDE_CRYO_HK_CMD_H
#define INCLUDE_CRYO_HK_CMD_H

#include "mcp.h"
#define CRYO_HK_CMD_IP "192.168.0.69"
#define CRYO_HK_CMD_PORT_FC1 "1567"
#define CRYO_HK_CMD_PORT_FC2 "1568" // two ports to make Ian happy

extern int16_t InCharge;

typedef struct cryo_command_data {
    int incharge;
    int command_num;
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
} cryo_command_data;

void * send_cryo_commands(void* args);

#endif
