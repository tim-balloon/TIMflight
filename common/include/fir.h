/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2005 University of Toronto
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


#ifndef INCLUDE_FIR_H
#define INCLUDE_FIR_H

#define NSTAGE 8
typedef enum {
    wind_oldblast = 0,
    wind_blackman,
    wind_gauss,
    wind_lanczos,
    wind_hamming,
    wind_hann,
    wind_rect
} e_windows;

typedef struct FirStruct {
  int i_w[NSTAGE];
  int ns;
  double *w;
  double sum[NSTAGE];
  double out;
} fir_t;

void init_fir(fir_t *fs, int N, int window, double alpha);
void deinit_fir(fir_t *fs);
double fir_filter(double x, fir_t *fs);

#endif
