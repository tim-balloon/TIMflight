/**
 * @file mcp_mock_decl.c
 *
 * @date Jan 12, 2023
 * @author evanmayer
 *
 * @brief This file is part of MCP, created for the TIMballoon project
 *
 * This software is copyright (C) 2023 University of Arizona
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

// This file exists because the mcp target defines some stuff as extern, for
// many other translation units.
// Because mcp.c has main() we can't compile it AND a unit test executable.
// Therefore, mock out the definitions in a new translation unit, which unit
// tests may compile mcp libraries against.

// Stuff mcp.c defines as extern
int16_t SouthIAm = 0;
int16_t InCharge = 1;
int16_t InChargeSet = 0;
linklist_t * linklist_array[MAX_NUM_LINKLIST_FILES] = {NULL};
linklist_t * telemetries_linklist[NUM_TELEMETRIES] = {NULL, NULL, NULL, NULL};
int ResetLog = 0;
time_t mcp_systime(time_t *t)
{
    time_t the_time = time(NULL);
    return the_time;
}
bool shutdown_mcp = false;
// Stuff in scheduler_tng.c defines as extern
unsigned int sched_lst = 0;
int doing_schedule = 0;
int LoadUplinkFile(int slot) {
    return 1;
}
