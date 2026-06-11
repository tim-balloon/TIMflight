/* analysis_comp_trigger.h: TIM data structures and function primals mcp to acomp 1 trigger data link
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

#ifndef INCLUDE_ACOMP_TRIGGER_H
#define INCLUDE_ACOMP_TRIGGER_H

#include "mcp.h"
#define ACOMP_TRIGGER_PORT "1212"

extern int16_t InCharge;

struct trigger {
    int take_data;
};

void acomp_countdown_1hz(void);
void * send_data_trigger_acomp(void* args);


#endif