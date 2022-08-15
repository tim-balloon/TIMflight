/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
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


#include "channels_tng.h"
#include "channel_macros.h"
#include "tx.h"
#include "command_struct.h"


extern int16_t InCharge; /* tx.c */


void VideoTx(void)
{
    static channel_t* bitsVtxAddr;
    static int firsttime = 1;
    int vtx_bits = 0;

    if (firsttime) {
        firsttime = 0;
        bitsVtxAddr = channels_find_by_name("bits_vtx");
    }

    if (CommandData.vtx_sel[0] == vtx_xsc1) vtx_bits |= 0x1;
    if (CommandData.vtx_sel[1] == vtx_xsc0) vtx_bits |= 0x4;

    SET_VALUE(bitsVtxAddr, vtx_bits);
}

