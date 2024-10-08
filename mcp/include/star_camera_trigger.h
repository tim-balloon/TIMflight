/* star_camera_trigger.h
 *
 * This software is copyright (C) 2023 University of Arizona
 *
 * This file is part of the TIM flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "mcp.h"
#include "pointing_struct.h"
#include "pointing.h"


extern struct PointingDataStruct PointingData[3];
extern int32_t sc_trigger_framenum[2];
extern time_t sc_trigger_lst[2];
extern double sc_trigger_lat[2];

void *star_camera_trigger_thread(void *args);
void update_az_vel_limit_tlm(void);