/* command_list.h: BLAST command specification file definitions
 *
 * This software is copyright (C) 2002-2012 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
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
void *star_camera_command_thread(void *args);
void populate_socket_data(char * ipaddr, char * port, struct socket_data *data);
