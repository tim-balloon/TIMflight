/* slowdl_struct.c: contains the slow channel lists
 *
 * This software is copyright (C) 2013 University of Toronto
 *
 * This file is part of mcp/pcm licensed under the GNU General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "slowdl.h"

/**
 * @brief struct for the slow downlink, initialized only with time and plover. This is the 
 * Science Burst Data last resort I believe (255 byte packet) which is packed in another 
 * telemetry file (slowdl.c in common/).
 * 
 */
struct SlowDlStruct slowDLList[] = {
  {"plover", 'u', SDL_RAW},
  {"time", 'U', SDL_RAW},
  {""}
};

