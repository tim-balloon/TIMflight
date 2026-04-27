/* rfsoc_commanding.h: TIM data structures and function primals for
 * the mcp to rfsoc commanding
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

#ifndef INCLUDE_RFSOC_COMMANDING_H
#define INCLUDE_RFSOC_COMMANDING_H

#include "mcp.h"
#define RFSOC_IP_1 "192.168.1.98" // SUPER DUPER PLACEHOLDER
#define RFSOC_IP_2 "192.168.1.99" // SUPER DUPER PLACEHOLDER
#define RFSOC_PORT_FC1 "5346"
#define RFSOC_PORT_FC2 "5347"

extern int16_t InCharge;

typedef struct rfsoc_data {
    int incharge;
    int drone_num;
    int command_num;
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
} rfsoc_data;

void * send_rfsoc_commands(void* args);

#endif