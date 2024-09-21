/**
 * @file gps.c
 *
 * @date Sept 19, 2024
 * @author evanmayer
 *
 * @brief This file is part of MCP, created for the BLASTPol project
 *
 * This software is copyright (C) 2024 University of Arizona
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This module uses example code provided by the gpsd project:
 * https://gpsd.gitlab.io/gpsd/gpsd-client-example-code.html
 * That software is used and provided here with the license: 2-clause BSD
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <endian.h>
#include <errno.h>
#include <math.h>
#include <gps.h>

#include "mcp.h"
#include "mputs.h"
#include "tim_gps.h"

#undef TIM_GPS_CHATTER

struct GPSInfoStruct TIMGPSData = {0};

/**
 * @brief Connect to gpsd
 * @return -1 on failure to open port to gpsd, 0 on success
 */
int16_t tim_gps_init(struct gps_data_t *pgps_data) {
    static bool have_warned_port = false;

    if (0 != gps_open(TIM_GPSD_HOSTNAME, TIM_GPSD_PORT, pgps_data)) {
        if (!have_warned_port) {
            have_warned_port = true;
            blast_err("gpsd client: port open error.\n");
        }
        return -1;
    } else {
        have_warned_port = false;
    }

    (void)gps_stream(pgps_data, WATCH_ENABLE | WATCH_JSON, NULL);
    return 0;
}


/**
 * @brief thread function for monitoring the GPS and logging
 * 
 * @param arg GPSinfo struct but all threads cast args as void *
 * @return void* none
 */
void * GPSMonitor(void * arg) {
    // Thread-local pointer to struct passed in by caller
    struct GPSInfoStruct * gps_info = (struct GPSInfoStruct *) arg;
    // For receipt of gpsd data
    struct gps_data_t gps_data = {0};

    static bool have_warned_read = false;
    uint16_t error_count = 0;

    nameThread("TIMGPS");
    blast_info("Started TIM GPS thread");

    while (!shutdown_mcp) {
        // If first time, start connection.
        // If we got here due to a read error, restart connection.
        tim_gps_init(&gps_data);

        while (gps_waiting(&gps_data, TIM_GPSD_WAITING_TIME_USEC)) {
            if (-1 == gps_read(&gps_data, NULL, 0)) {
                error_count++;
                if (!have_warned_read) {
                    have_warned_read = true;
                    blast_err("gpsd client: read error. Count: %d\n", error_count);
                }
                if (TIM_GPSD_MAX_ERROR_COUNT < error_count) {
                    error_count = 0U;
                    break;
                }
            } else {
                have_warned_read = false;
            }

            // Mode not updated yet, no fix, 2d fix, or 3d fix
            if (MODE_SET != (MODE_SET & gps_data.set)) {
                // did not even get mode, nothing to see here
                #ifdef TIM_GPS_CHATTER
                    blast_info("gpsd client: no-fix mode: %lu\n", gps_data.set);
                #endif
                continue;
            }

            // Always valid: fix type/"quality"
            gps_info->quality = gps_data.status;
            #ifdef TIM_GPS_CHATTER
                blast_info("gpsd client: fix status: %d\n", gps_data.status);
            #endif

            // Number of satellites used in solution
            gps_info->num_sat = gps_data.satellites_used;
            #ifdef TIM_GPS_CHATTER
                blast_info("gpsd client: num_satellites: %d\n", gps_data.satellites_used);
            #endif

            if (0 > gps_data.fix.mode ||
                MODE_STR_NUM <= gps_data.fix.mode) {
                gps_data.fix.mode = 0;
            }
            #ifdef TIM_GPS_CHATTER
                blast_info("gpsd client: fix mode: %s (%d) Time: ",
                    mode_str[gps_data.fix.mode],
                    gps_data.fix.mode);
            #endif

            // Time - not used because chrony is in charge of system time
            if (TIME_SET == (TIME_SET & gps_data.set)) {
                // not 32 bit safe
                #ifdef TIM_GPS_CHATTER
                    blast_info("%ld.%09ld ", gps_data.fix.time.tv_sec,
                        gps_data.fix.time.tv_nsec);
                #endif
            } else {
                #ifdef TIM_GPS_CHATTER
                    blast_info("n/a ");
                #endif
            }

            // LLA
            if (isfinite(gps_data.fix.latitude) &&
                isfinite(gps_data.fix.longitude) &&
                isfinite(gps_data.fix.altitude)) {
                // Display data from the GPS receiver if valid.
                gps_info->latitude = gps_data.fix.latitude;
                gps_info->longitude = gps_data.fix.longitude;
                gps_info->altitude = gps_data.fix.altMSL;
                gps_info->isnew = 1;
                #ifdef TIM_GPS_CHATTER
                    blast_info("Lat %.6f Lon %.6f Alt %.6f\n",
                        gps_data.fix.latitude,
                        gps_data.fix.longitude,
                        gps_data.fix.altMSL);
                #endif
            } else {
                #ifdef TIM_GPS_CHATTER
                    blast_info("Lat n/a Lon n/a Alt n/a\n");
                #endif
            }
        }
    }
    (void)gps_stream(&gps_data, WATCH_DISABLE, NULL);
    (void)gps_close(&gps_data);
    return NULL;
}
