/* analysis_comp_sharing.h: TIM data structures and function primals mcp to acomp 1 data link
 *
 * This software is copyright (C) 2026 University of Arizona
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_ACOMP_SHARING_H
#define INCLUDE_ACOMP_SHARING_H

#include "mcp.h"
#define ACOMP1_IP "192.168.1.6"
#define ACOMP_POINTING_PORT "1213"

extern int16_t InCharge;

typedef struct pointing_data { // need to finish designing this
    float az;
    float el;
    float latitude;
    float longitude;
    uint16_t height;
    uint32_t time;
    uint32_t time_usec;
} pointing_data;

void fill_packet_from_channels(pointing_data* packet);
void clear_packet_data(pointing_data* packet);
void get_acomp_shared_data_1hz(void);
void * send_pointing_data_acomp(void* args);

#endif