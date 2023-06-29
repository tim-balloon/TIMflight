/* star_camera_structs.h: TIM data structures for the star cameras
 *
 * This software is copyright (C) 2021-2023 University of Arizona
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// Defines for the star camera communications
// on FC
#define SC1_RECEIVE_PARAM_PORT "4950"
#define SC2_RECEIVE_PARAM_PORT "4951"
#define SC1_RECEIVE_SOLVE_PORT "4960"
#define SC2_RECEIVE_SOLVE_PORT "4961"
// on SCC
#define SC1_COMMAND_PORT_FC1 "4950"
#define SC2_COMMAND_PORT_FC1 "4950"
#define SC1_COMMAND_PORT_FC2 "4951"
#define SC2_COMMAND_PORT_FC2 "4951"

// IP addresses for the SCs
#define SC1_IP_ADDR "192.168.1.137"
#define SC2_IP_ADDR "192.168.1.138"

// Structure of the expected data packet to come down with every solution
struct mcp_astrometry {
    double ra_j2000;
    double dec_j2000;
    double ra_observed;
    double dec_observed;
    double rawtime;
    double image_rms;
    double fr;
    double ps;
    double ir;
    double alt;
    double az;
    double photo_time;
};

// Command packet structure for the star cameras
struct star_cam_capture {
    int fc;
    char target[16];
    int inCharge; // make sure an incharge FC sent this just in case
    double logOdds; // significance of point sources
    int update_logOdds; // is this a new commanded value?
    double latitude; // payload lat
    int update_lat; // is this a new commanded value?
    double longitude; // payload long
    int update_lon; // is this a new commanded value?
    double heightWGS84; // payload alt above reference surface
    int update_height; // is this a new commanded value?
    double exposureTime; // milliseconds
    int update_exposureTime; // is this a new commanded value?
    double solveTimeLimit; // time allowed to solve an image
    int update_solveTimeLimit; // is this a new commanded value?
    float focusPos; // desired focus position, encoder units
    int update_focusPos; // is this a new commanded value?
    int focusMode; // autofocus or manual?
    int update_focusMode; // is this a new commanded value?
    int startPos; // start of autofocus range
    int update_startPos; // is this a new commanded value?
    int endPos; // end of autofocus range
    int update_endPos; // is this a new commanded value?
    int focusStep; // step size in encoder units
    int update_focusStep; // is this a new commanded value?
    int photosPerStep; // ...
    int update_photosPerStep; // is this a new commanded value?
    int setFocusInf; // move focus position to infinity
    int update_setFocusInf; // is this a new commanded value?
    int apertureSteps; // number of positions +/- to move the aperture
    int update_apertureSteps; // is this a new commanded value?
    int maxAperture; // open aperture fully
    int update_maxAperture; // is this a new commanded value?
    int makeHP; // set to 20 to do it, makes a new static hot pixel map
    int update_makeHP; // is this a new commanded value?
    int useHP; // use the hot pixel map to mask bad pixels
    int update_useHP; // is this a new commanded value?
    float blobParams[9]; // blobfinding parameters...
    int update_blobParams[9]; // is this a new commanded value?
    // 0 = spike limit
    // where dynamic hot pixel will designate as hp
    // 1 = dynamic hot pixels
    // (bool) search for dynamic hot pixels
    // 2 = smoothing radius
    // image smooth filter radius [px]
    // 3 = high pass filter
    // 0 == off, 1 == on
    // 4 = high pass filter radius
    // image high pass filter radius [px]
    // 5 = centroid search border
    // px dist from image edge to start star search
    // 6 = filter return image
    // 1 == true; 0 = false
    // 7 = n sigma
    // pixels > this*noise + mean = blobs
    // 8 = unique star spacing
    // min. pixel spacing between stars [px]
    // float deltaFocus; // how far to move the focus from current pos (maybe)
};

// Star camera returned data for parameters
struct star_cam_return {
    double logOdds; // significance of point sources
    double latitude; // payload lat
    double longitude; // payload long
    double heightWGS84; // payload alt above reference surface
    double exposureTime; // milliseconds
    double solveTimeLimit; // time allowed to solve an image
    float focusPos; // desired focus position, encoder units
    int minFocusPos; // encoder minimum allowed focus
    int maxFocusPos; // encoder maximum allowed focus
    int focusMode; // autofocus or manual?
    int startPos; // start of autofocus range
    int endPos; // end of autofocus range
    int focusStep; // step size in encoder units
    int photosPerStep; // photos per AF step
    int setFocusInf; // move focus position to infinity
    int apertureSteps; // number of positions +/- to move the aperture
    int maxAperture; // open aperture fully
    float aperture; // current aperture setting
    int makeHP; // set to 20 to do it, makes a new static hot pixel map
    int useHP; // use the hot pixel map to mask bad pixels
    float blobParams[9]; // blobfinding parameters...
    // 0 = spike limit
    // where dynamic hot pixel will designate as hp
    // 1 = dynamic hot pixels
    // (bool) search for dynamic hot pixels
    // 2 = smoothing radius
    // image smooth filter radius [px]
    // 3 = high pass filter
    // 0 == off, 1 == on
    // 4 = high pass filter radius
    // image high pass filter radius [px]
    // 5 = centroid search border
    // px dist from image edge to start star search
    // 6 = filter return image
    // 1 == true; 0 = false
    // 7 = n sigma
    // pixels > this*noise + mean = blobs
    // 8 = unique star spacing
    // min. pixel spacing between stars [px]
    // float deltaFocus; // how far to move the focus from current pos (maybe)
};

// packaged thread-type agnostic structure for passing to pthreads
struct socket_data {
    char ipAddr[16];
    char port[5];
    /* struct star_cam_capture * camera_commands;
    struct star_cam_return * camera_params;
    struct mcp_astrometry * image_solutions; */
};
