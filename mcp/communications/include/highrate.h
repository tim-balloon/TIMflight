/* mcp: the BLAST flight control program
 *
 * This software is copyright (C) 2018 Penn University
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef INCLUDE_HIGHRATE_H
#define INCLUDE_HIGHRATE_H

#define HIGHRATE_MAX_SIZE 1150000
#define HIGHRATE_PORT "/dev/ttyHighRate"
#define DIRECT_PORT "/dev/ttyDirect"

#define CSBF_HEADER_SIZE 6
#define HIGHRATE_DATA_PACKET_SIZE 2034 // 11500
#define HIGHRATE_CHECKSUM_SIZE 1
#define HIGHRATE_SYNC1 0xfa
#define LOWRATE_COMM1_SYNC2 0xfa
#define LOWRATE_COMM2_SYNC2 0xfb
#define HIGHRATE_TDRSS_SYNC2 0xff
#define HIGHRATE_IRIDIUM_SYNC2 0xfd
#define HIGHRATE_ORIGIN_COMM1 0x02
#define HIGHRATE_ORIGIN_COMM2 0x0a

extern struct Fifo highrate_fifo;
extern struct Fifo sbd_fifo;

void highrate_compress_and_send(void *);


#endif /* INCLUDE_HIGHRATE_H */

