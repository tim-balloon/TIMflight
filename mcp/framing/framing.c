/* 
 * framing.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Aug 12, 2014 by seth
 */

#include <stdio.h>
#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

#include <mosquitto.h>

#include <blast.h>
#include <blast_time.h>
#include <channels_tng.h>
#include <crc.h>
#include <derived.h>
#include <mputs.h>

#include <command_struct.h>
#include "mcp.h"

static int frame_stop;
static struct mosquitto *mosq = NULL;
static struct mosquitto *mosq_other = NULL;
extern int16_t SouthIAm;
extern int16_t InCharge;

static int32_t mcp_488hz_framenum = -1;
static int32_t mcp_244hz_framenum = -1;
static int32_t mcp_200hz_framenum = -1;
static int32_t mcp_100hz_framenum = -1;
static int32_t mcp_5hz_framenum = -1;
static int32_t mcp_1hz_framenum = -1;

/**
 * @brief Returns the current MCP framenumber of the 200Hz Frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_200hz_framenum(void)
{
    return mcp_200hz_framenum;
}


/**
 * @brief Returns the current MCP framenumber of the 100Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_100hz_framenum(void)
{
    return mcp_100hz_framenum;
}


/**
 * @brief Returns the current MCP framenumber of the 5Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_5hz_framenum(void)
{
    return mcp_5hz_framenum;
}


/**
 * @brief Returns the current MCP framenumber of the 1Hz frames
 * @return -1 before initialization, framenumber after
 */
int32_t get_1hz_framenum(void)
{
    return mcp_1hz_framenum;
}


/**
 * @brief Function that checks the "level" input vs mosquitto error and warning defines
 * then prints the message to the log if one is found
 * 
 * @param mosq unused
 * @param userdata unused
 * @param level parameter to check vs error or warning from mosquitto
 * @param str Error or warning message to display
 */
static void frame_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
    if (level & (MOSQ_LOG_ERR | MOSQ_LOG_WARNING)) {
        blast_info("%s\n", str);
    }
}


/**
 * @brief function that publishes the 1Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_1hz(void)
{
    static channel_t *mcp_1hz_framenum_addr = NULL;
    static channel_t *mcp_1hz_framenum_dl_addr = NULL;
    static char frame_name[32];
    if (mcp_1hz_framenum_addr == NULL) {
        mcp_1hz_framenum_addr = channels_find_by_name("mcp_1hz_framecount");
        mcp_1hz_framenum_dl_addr = channels_find_by_name("mcp_1hz_framecount_dl");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/1Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_1hz_framenum++;
    SET_INT32(mcp_1hz_framenum_addr, mcp_1hz_framenum);
    SET_INT32(mcp_1hz_framenum_dl_addr, mcp_1hz_framenum);
    if (frame_size[RATE_1HZ]) {
        if (1) {
            // blast_warn("the size of 1hz data is %zu", sizeof(channel_data[RATE_1HZ]));
        }
        // mosquitto_publish(mosq, NULL, frame_name,
        //                   frame_size[RATE_1HZ], channel_data[RATE_1HZ], 0, false);
    }
}


/**
 * @brief function that publishes the 5Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_5hz(void)
{
    static channel_t *mcp_5hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_5hz_framenum_addr == NULL) {
        mcp_5hz_framenum_addr = channels_find_by_name("mcp_5hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/5Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_5hz_framenum++;
    SET_INT32(mcp_5hz_framenum_addr, mcp_5hz_framenum);
    if (frame_size[RATE_5HZ]) {
        if (mcp_5hz_framenum % 5 == 1) {
            // blast_warn("the size of the 5hz frame is %zu", frame_size[RATE_5HZ]);
        }
        // mosquitto_publish(mosq, NULL, frame_name,
        //         frame_size[RATE_5HZ], channel_data[RATE_5HZ], 0, false);
    }
}


/**
 * @brief function that publishes the 100Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_100hz(void)
{
    static channel_t *mcp_100hz_framenum_addr = NULL;
    static char frame_name[32];
    if (mcp_100hz_framenum_addr == NULL) {
        mcp_100hz_framenum_addr = channels_find_by_name("mcp_100hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/100Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_100hz_framenum++;
    SET_INT32(mcp_100hz_framenum_addr, mcp_100hz_framenum);
    if (frame_size[RATE_100HZ]) {
        if (mcp_100hz_framenum % 100 == 1) {
            // blast_warn("the size of the 100hz data is %zu", frame_size[RATE_100HZ]);
        }
        // mosquitto_publish(mosq, NULL, frame_name,
        //        frame_size[RATE_100HZ], channel_data[RATE_100HZ], 0, false);
    }
}


/**
 * @brief function that publishes the 200Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_200hz(void)
{
    static channel_t *mcp_200hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_200hz_framenum_addr == NULL) {
        mcp_200hz_framenum_addr = channels_find_by_name("mcp_200hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/200Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_200hz_framenum++;
    SET_INT32(mcp_200hz_framenum_addr, mcp_200hz_framenum);
    if (frame_size[RATE_200HZ]) {
        if (mcp_200hz_framenum % 200 == 1) {
            // blast_warn("the size of the 200hz frame is %zu", frame_size[RATE_200HZ]);
        }
        // mosquitto_publish(mosq, NULL, frame_name,
        //         frame_size[RATE_200HZ], channel_data[RATE_200HZ], 0, false);
    }
}


/**
 * @brief function that publishes the 244Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_244hz(void)
{
    static channel_t *mcp_244hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_244hz_framenum_addr == NULL) {
        mcp_244hz_framenum_addr = channels_find_by_name("mcp_244hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/244Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_244hz_framenum++;
    SET_INT32(mcp_244hz_framenum_addr, mcp_244hz_framenum);
    if (frame_size[RATE_244HZ]) {
        if ((mcp_244hz_framenum % 244) == 1) {
           // blast_warn("size of 244hz is %zu", frame_size[RATE_244HZ]);
        }
        // mosquitto_publish(mosq, NULL, frame_name,
        //         frame_size[RATE_244HZ], channel_data[RATE_244HZ], 0, false);
    }
}


/**
 * @brief function that publishes the 488Hz frames to the mosquitto server - deprecated
 * 
 */
void framing_publish_488hz(void)
{
    static channel_t *mcp_488hz_framenum_addr = NULL;
    static char frame_name[32];

    if (mcp_488hz_framenum_addr == NULL) {
        mcp_488hz_framenum_addr = channels_find_by_name("mcp_488hz_framecount");
        snprintf(frame_name, sizeof(frame_name), "frames/fc/%d/488Hz", SouthIAm + 1);
    }

    if (frame_stop) return;

    mcp_488hz_framenum++;
    SET_INT32(mcp_488hz_framenum_addr, mcp_488hz_framenum);
    if (frame_size[RATE_488HZ]) {
        // mosquitto_publish(mosq, NULL, frame_name,
        //         frame_size[RATE_488HZ], channel_data[RATE_488HZ], 0, false);
    }
}


/**
 * @brief function that publishes command data to the mosquitto server - deprecated
 * 
 * @param m_commanddata the command data structure from MCP
 */
void framing_publish_command_data(struct CommandDataStruct *m_commanddata)
{
    // mosquitto_publish(mosq, NULL, "commanddata", sizeof(struct CommandDataStruct), m_commanddata, 1, 1);
}


/**
 * @brief function that checks the data source vs rate as well as the rate itself vs allowed parameters
 * to give us an error or warning that we are doing invalid things (unrecognized rates, rates not matching sources etc)
 * 
 * @param m_src data source
 * @param m_rate data expected rate
 * @param m_data nominally the data, unused
 * @param m_len size of the frame
 */
static void framing_handle_data(const char *m_src, const char *m_rate, const void *m_data, const int m_len)
{
    RATE_LOOKUP_T *rate;

    if (!m_src || !m_rate) {
        blast_err("Err in pointers");
        return;
    }
    if (!m_len) {
        blast_warn("Zero-length string for frame");
        return;
    }

    for (rate = RATE_LOOKUP_TABLE; rate->position < RATE_END; rate++) {
        if (strncasecmp(rate->text, m_rate, BLAST_LOOKUP_TABLE_TEXT_SIZE) == 0) break;
    }
    if (rate->position == RATE_END) {
        blast_err("Did not recognize rate %s!", m_rate);
        return;
    }
}


/**
 * @brief callback function to handle the data in a mosquitto message
 * 
 * @param mosq unused
 * @param userdata unused
 * @param message message to send over the mosquitto publishing server
 */
static void framing_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    char **topics;
    int count;

    if (message->payloadlen) {
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {
            if (count == 4 && topics[0] && strcmp(topics[0], "frames") == 0) {
                framing_handle_data(topics[1], topics[3], message->payload, message->payloadlen);
            }
            mosquitto_sub_topic_tokens_free(&topics, count);
        }
    }
}



/**
 * @brief Received data from "other" flight computer including CommandData struct,
 * EtherCAT state, Flight computer temps, etc.
 * @param mosq Pointer to mosq connection to other flight computer
 * @param userdata
 * @param message
 */
static void framing_shared_data_callback(struct mosquitto *mosq, void *userdata,
                                         const struct mosquitto_message *message)
{
    char **topics;
    int count;
    static int has_warned = 0;

    if (message->payloadlen > 0) {
        if (mosquitto_sub_topic_tokenise(message->topic, &topics, &count) == MOSQ_ERR_SUCCESS) {
            if (topics[0] && strcmp(topics[0], "commanddata") == 0) {
                if (!InCharge && (sizeof(CommandData) == message->payloadlen)) {
                    struct CommandDataStruct temp_command_data = {0};
                    uint32_t prev_crc;

                    memcpy(&temp_command_data, message->payload, message->payloadlen);
                    prev_crc = temp_command_data.checksum;
                    temp_command_data.checksum = 0;
                    if (prev_crc == crc32_le(0, (uint8_t*)&temp_command_data, sizeof(temp_command_data))) {
                        has_warned = 0;
                        memcpy(&CommandData, &temp_command_data, sizeof(CommandData));
                    } else if (!has_warned) {
                        has_warned = 1;
                        blast_err("Received invalid CommandData from in charge flight computer!"
                                " Please check that both FCs are running the same mcp");
                    }
                }
            }
            mosquitto_sub_topic_tokens_free(&topics, count);
        }
    }
}


/**
 * @brief Checks if a mosquitto connection was succesful and then sucbscribes the mosquitto
 * object passed in to the "commanddata" information channel
 * 
 * @param m_mosq Mosquitto data server structure to receive or share data
 * @param obj unused
 * @param rc Value to check if the mosquitto connection was successful
 */
static void framing_shared_connect_callback(struct mosquitto *m_mosq, void *obj, int rc)
{
    if (rc == MOSQ_ERR_SUCCESS) {
        mosquitto_subscribe(m_mosq, NULL, "commanddata", 1);
    }
}


/**
 * @brief Connects to a shared data mosquitto server
 * 
 * @return int 0 on succesful connection, -1 on failure
 */
int framing_shared_data_init(void)
{
    char id[4] = "fcX";
    char host[4] = "fcX";

    int ret = 0;
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    snprintf(id, sizeof(id), "fc%d", SouthIAm + 1);
    snprintf(host, sizeof(host), "fc%d", (SouthIAm + 2) % 2 + 1);

    mosq_other = mosquitto_new(id, clean_session, NULL);
    if (!mosq_other) {
        blast_err("mosquitto_new() failed creating other");
        return -1;
    }
    mosquitto_log_callback_set(mosq_other, frame_log_callback);
    mosquitto_message_callback_set(mosq_other, framing_shared_data_callback);
    mosquitto_connect_callback_set(mosq_other, framing_shared_connect_callback);

    if ((ret = mosquitto_connect_async(mosq_other, host, port, keepalive)) != MOSQ_ERR_SUCCESS) {
        if (ret == MOSQ_ERR_INVAL) {
        	blast_err("Unable to connect to mosquitto server: Invalid Parameters!");
        } else {
        	if (errno == EINPROGRESS) {
        		/* Do nothing, connection in progress */
        	} else {
        		blast_strerror("Unable to connect to mosquitto server!");
        		return -1;
        	}
        }
    }

    mosquitto_reconnect_delay_set(mosq_other, 1, 10, 1);
    mosquitto_loop_start(mosq_other);
    return 0;
}


/**
 * @brief Initializes the mosquitto library and associated framing routines.
 * 
 * @param channel_list list of channels in tx_struct_tng.c
 * @param m_derived list of derived channels from the above (derived.c)
 * @return int 0 on success, -1 on failure
 */
int framing_init(channel_t *channel_list, derived_tng_t *m_derived)
{
    channel_header_t *channels_pkg = NULL;
    derived_header_t *derived_pkg = NULL;

    char id[4] = "fcX";
    char host[4] = "fcX";
    char topic[64];

    int ret = 0;
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;

    snprintf(id, sizeof(id), "fc%d", SouthIAm + 1);
    snprintf(host, sizeof(host), "fc%d", SouthIAm + 1);
    mosquitto_lib_init();
    mosq = mosquitto_new(id, clean_session, NULL);
    if (!mosq) {
        perror("mosquitto_new() failed");
        return -1;
    }
    mosquitto_log_callback_set(mosq, frame_log_callback);
    mosquitto_message_callback_set(mosq, framing_message_callback);

    if (mosquitto_connect_async(mosq, host, port, keepalive)) {
        if (ret == MOSQ_ERR_INVAL) {
        	blast_err("Unable to connect to mosquitto server: Invalid Parameters!");
        } else {
        	if (errno == EINPROGRESS) {
        		/* Do nothing, connection in progress */
        	} else {
        		blast_strerror("Unable to connect to mosquitto server!");
        		return -1;
        	}
        }
    }

    /**
     * Set up the channels and derived packages for subscribers
     */
    if (!(channels_pkg = channels_create_map(channel_list))) {
        blast_err("Exiting framing routine because we cannot get the channel list");
        return -1;
    }

    mosquitto_reconnect_delay_set(mosq, 1, 10, 1);
    mosquitto_loop_start(mosq);

    snprintf(topic, sizeof(topic), "channels/fc/%d", SouthIAm + 1);
    mosquitto_publish(mosq, NULL, topic,
            sizeof(channel_header_t) + channels_pkg->length * sizeof(struct channel_packed), channels_pkg, 1, true);
    bfree(err, channels_pkg);

    if (!(derived_pkg = channels_create_derived_map(m_derived))) {
        blast_warn("Failed sending derived packages");
    } else {
        snprintf(topic, sizeof(topic), "derived/fc/%d", SouthIAm + 1);
        mosquitto_publish(mosq, NULL, topic,
                sizeof(derived_header_t) + derived_pkg->length * sizeof(derived_tng_t), derived_pkg, 1, true);
        bfree(err, derived_pkg);
    }

    return 0;
}


/**
 * @brief cleanly shuts down the mosquitto server when MCP is shutting down
 * 
 */
void framing_shutdown(void)
{
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, true);
    frame_stop = 1;
}

