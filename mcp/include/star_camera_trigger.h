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

// ~1 pixel streaking limit velocity for 6.2" pixels
// with a 100ms exposure time. units = deg/s
#define AZ_VEL_LIMIT 0.0167

extern struct PointingDataStruct PointingData[3];

void *star_camera_trigger_thread(void *args);
