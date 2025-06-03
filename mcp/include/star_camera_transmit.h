/* star_camera_transmit.h
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
#include "star_camera_structs.h"

extern int16_t InCharge;
int which_fc_am_i(void);
int check_sc_thread_bool(int which);
int check_for_reset(int which);
void reset_command_bools(void);
void *star_camera_command_thread(void *args);
