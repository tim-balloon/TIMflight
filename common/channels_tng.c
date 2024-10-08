/* 
 * channels_v2.c: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
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
 * Created on: Aug 5, 2014 by seth
 */

/**
 * Code Overview:
 *
 * This file provides the interface to the Channels structure for BLAST-TNG.
 *
 * Channels lives in tx_struct_tng.c and list all known channels for the experiment.
 * On startup, the channels structure is read and separated into a number of distinct
 * frames based on rate.  Thus there is a 1Hz frame, a 5Hz frame, a 100Hz frame,
 * etc.  The frames are packed structures, ordered first by their source.
 *
 * Each frame also has a small header that records the timestamp and computer id.
 *
 * On MCP, there are frame structures for each computer (UEI1, UEI2, FC1, FC2, SC1, SC2).
 * The data are populated by listening to the MQTT server.
 *
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <glib.h>
#include <math.h>
#include <ctype.h>

#include "blast.h"
#include "PMurHash.h"
#include "channels_tng.h"
#include "derived.h"
#include "linklist.h"

char * ROACH_TYPES[NUM_RTYPES] = {"i", "q", "df"};
static GHashTable *frame_table = NULL;
static int channel_count[RATE_END][TYPE_END] = {{0}};
int channels_count = 0;

void *channel_data[RATE_END] = {0};
size_t frame_size[RATE_END] = {0};
static void *channel_ptr[RATE_END] = {0};

// conversion from channel type to superframe type
uint8_t superframe_type_array[TYPE_END+1] = {
  SF_UINT8, SF_UINT16, SF_UINT32, SF_UINT64,
  SF_INT8, SF_INT16, SF_INT32, SF_INT64,
  SF_FLOAT32, SF_FLOAT64, SF_NUM
};
unsigned int superframe_offset[RATE_END] = {0};
superframe_t * superframe = NULL;

size_t channel_size(channel_t *m_channel)
{
    size_t retsize = 0;
    switch (m_channel->type) {
        case TYPE_INT8:
        case TYPE_UINT8:
            retsize = 1;
            break;
        case TYPE_INT16:
        case TYPE_UINT16:
            retsize = 2;
            break;
        case TYPE_INT32:
        case TYPE_UINT32:
        case TYPE_FLOAT:
            retsize = 4;
            break;
        case TYPE_INT64:
        case TYPE_UINT64:
        case TYPE_DOUBLE:
            retsize = 8;
            break;
        default:
            blast_fatal("Invalid Channel size!");
    }
    return retsize;
}
unsigned int get_spf(unsigned int rate)
{
    switch (rate) {
        case RATE_1HZ:
            return 1;
        case RATE_2HZ:
            return 2;
        case RATE_5HZ:
            return 5;
        case RATE_100HZ:
            return 100;
        case RATE_122HZ:
            return 122;
        case RATE_200HZ:
            return 200;
        case RATE_244HZ:
            return 244;
        case RATE_488HZ:
            return 488;
        default:
            blast_err("Invalid rate %d", rate);
            return 0;
    }
}

static guint channel_hash(gconstpointer m_data)
{
    const char *field_name = (const char*)m_data;

    return PMurHash32(BLAST_MAGIC32, field_name, strnlen(field_name, FIELD_LEN));
}

static void channel_map_fields(gpointer m_key, gpointer m_channel, gpointer m_userdata)
{
    channel_t *channel = (channel_t*)m_channel;

    /// If channel is invalid, do not process
    if (!channel->field[0]) return;

    /**
     * channel_ptr maintains the current location in the array of the next free element.
     * We assign this location to the channel pointer, allowing us to reference the
     * individual location in the future based on a lookup in the hash table.
     */
    if (channel->rate < RATE_END) {
        if (!channel_ptr[channel->rate]) {
            blast_fatal("Invalid Channel setup");
        }
        channel->var = channel_ptr[channel->rate];
        channel_ptr[channel->rate] += channel_size(channel);
    } else {
        blast_fatal("Could not map %d to rate!", channel->rate);
    }
}

/**
 * Takes an aligned channel list and re-packs it into a packed structure for sharing over
 * MQTT.
 * @param m_channel_list Pointer to the aligned channel list
 * @return Newly allocated channel_header_t structure or NULL on failure
 */
channel_header_t *channels_create_map(channel_t *m_channel_list)
{
    channel_header_t *new_pkt = NULL;
    size_t channel_count;

    for (channel_count = 0; m_channel_list[channel_count].field[0]; channel_count++)
        continue;
    channel_count++; // Add one extra channel to allow for the NULL terminating field

    new_pkt = balloc(err, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);
    if (!new_pkt) return NULL;
    memset(new_pkt, 0, sizeof(channel_header_t) + sizeof(struct channel_packed) * channel_count);

    new_pkt->magic = BLAST_MAGIC32;
    new_pkt->version = BLAST_TNG_CH_VERSION;
    new_pkt->length = channel_count;
    new_pkt->crc = 0;

    /**
     * Copy over the data values one at a time from the aligned to the packed structure
     */
    for (size_t i = 0; i < channel_count; i++) {
        memcpy(new_pkt->data[i].field, m_channel_list[i].field, FIELD_LEN);
        new_pkt->data[i].m_c2e = m_channel_list[i].m_c2e;
        new_pkt->data[i].b_e2e = m_channel_list[i].b_e2e;
        new_pkt->data[i].type = m_channel_list[i].type;
        new_pkt->data[i].rate = m_channel_list[i].rate;
        memcpy(new_pkt->data[i].quantity, m_channel_list[i].quantity, UNITS_LEN);
        memcpy(new_pkt->data[i].units, m_channel_list[i].units, UNITS_LEN);
    }

    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(channel_header_t) +
                              sizeof(struct channel_packed) * channel_count);

    return new_pkt;
}

/**
 * Takes an aligned channel list for a single down sample rate and re-packs it into a
 * packed structure for sharing over MQTT or writing as a head in the data files stored
 * on the external HDs.
 * @param m_channel_list Pointer to the aligned channel list
 * @param m_rate Pointer to the aligned channel list
 * @return Newly allocated channel_header_t structure or NULL on failure
 */
channel_header_t *channels_create_rate_map(channel_t *m_channel_list, E_RATE m_rate)
{
    channel_header_t *new_pkt = NULL;
    size_t channels_count, total_channels_count;
    uint32_t i_channels, i_rate = 0;

	for (total_channels_count = 0; m_channel_list[total_channels_count].field[0]; total_channels_count++)
        continue;
    total_channels_count++; // Add one extra channel to allow for the NULL terminating field

	channels_count = (channel_count[m_rate][TYPE_INT8]+channel_count[m_rate][TYPE_UINT8]) +
                       2 * (channel_count[m_rate][TYPE_INT16]+channel_count[m_rate][TYPE_UINT16]) +
                       4 * (channel_count[m_rate][TYPE_INT32]+channel_count[m_rate][TYPE_UINT32] +
                            channel_count[m_rate][TYPE_FLOAT]) +
                       8 * (channel_count[m_rate][TYPE_INT64]+channel_count[m_rate][TYPE_UINT64] +
                            channel_count[m_rate][TYPE_DOUBLE]);
    new_pkt = balloc(err, sizeof(channel_header_t) + sizeof(struct channel_packed) * channels_count);
    if (!new_pkt) return NULL;
    memset(new_pkt, 0, sizeof(channel_header_t) + sizeof(struct channel_packed) * channels_count);

    new_pkt->magic = BLAST_MAGIC32;
    new_pkt->version = BLAST_TNG_CH_VERSION;
    new_pkt->length = channels_count;
    new_pkt->crc = 0;

    /**
     * Copy over the data values one at a time from the aligned to the packed structure
     */
    for (i_channels = 0; i_channels < total_channels_count; i_channels++) {
    	if (m_channel_list[i_channels].rate == m_rate) {
	    	if (i_rate >= channels_count) {
    			blast_err("More channels found with rate index %i (>=%i) than allocated (%i)",
    					   (uint16_t) m_rate, (uint32_t) i_rate, (uint32_t) total_channels_count);
    			return(new_pkt);
    		}
    	    memcpy(new_pkt->data[i_rate].field, m_channel_list[i_channels].field, FIELD_LEN);
            new_pkt->data[i_rate].m_c2e = m_channel_list[i_channels].m_c2e;
            new_pkt->data[i_rate].b_e2e = m_channel_list[i_channels].b_e2e;
            new_pkt->data[i_rate].type = m_channel_list[i_channels].type;
            new_pkt->data[i_rate].rate = m_channel_list[i_channels].rate;
            memcpy(new_pkt->data[i_rate].quantity, m_channel_list[i_channels].quantity, UNITS_LEN);
            memcpy(new_pkt->data[i_rate].units, m_channel_list[i_channels].units, UNITS_LEN);
            i_rate++;
    	}
	}
    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(channel_header_t) +
                              sizeof(struct channel_packed) * channels_count);

    return new_pkt;
}

/**
 * Takes an aligned channel list and re-packs it into a packed structure for sharing over
 * MQTT.
 * @param m_channel_list Pointer to the aligned channel list
 * @return Newly allocated channel_header_t structure or NULL on failure
 */
derived_header_t *channels_create_derived_map(derived_tng_t *m_derived)
{
    derived_header_t *new_pkt = NULL;
    size_t channel_count;

    for (channel_count = 0; m_derived[channel_count].type != DERIVED_EOC_MARKER; channel_count++)
        continue;
    channel_count++; // Add one extra channel to allow for the NULL terminating field

    new_pkt = balloc(err, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);
    if (!new_pkt) return NULL;
    memset(new_pkt, 0, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);

    new_pkt->magic = BLAST_MAGIC32;
    new_pkt->version = BLAST_TNG_CH_VERSION | 0x20; // 0x20 marks the packet as a derived packet
    new_pkt->length = channel_count;
    new_pkt->crc = 0;

    /**
     * Copy over the data values.  Union structure is already packed.
     */
    memcpy(new_pkt->data, m_derived, channel_count * sizeof(derived_tng_t));


    new_pkt->crc = PMurHash32(BLAST_MAGIC32, new_pkt, sizeof(derived_header_t) + sizeof(derived_tng_t) * channel_count);

    return new_pkt;
}

/**
 * Translates a stored channel map to the channel_list structure
 * @param m_map Pointer to the #channel_header_t structure storing our packet
 * @param m_len Length in bytes of the packet passed via m_map
 * @param m_channel_list Double pointer to where we will store the channel_list
 * @return -1 on failure, positive number of channels read otherwise
 */
int channels_read_map(channel_header_t *m_map, size_t m_len, channel_t **m_channel_list)
{
    uint32_t crcval = m_map->crc;

    if (m_map->version != BLAST_TNG_CH_VERSION) {
        blast_err("Unknown derived channels version %d", m_map->version);
        return -1;
    }

    if (m_len < sizeof(channel_header_t)) {
        blast_err("Invalid size %zu for channel packet", m_len);
        return -1;
    }

    if (m_len != sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed)) {
        blast_err(" m-Map has length %u", m_map->length);
        blast_err("struct channelpacked has size %zu", sizeof(struct channel_packed));
        blast_err("channel header has size %zu", sizeof(channel_header_t));
        blast_err("Length of data packet %zu does not match header data %zu",
                  m_len, sizeof(channel_header_t) + m_map->length * sizeof(struct channel_packed));
        return -1;
    }

    m_map->crc = 0;
    if (crcval != PMurHash32(BLAST_MAGIC32, m_map, m_len)) {
        blast_err("CRC match failed!");
        return -1;
    }
    m_map->crc = crcval;

    *m_channel_list = calloc(m_map->length, sizeof(channel_t));
    if (!(*m_channel_list)) return -1;

    /**
     * Copy over the data values one at a time from the packed to the aligned structure
     */
    for (size_t channel_count = 0; channel_count < m_map->length; channel_count++) {
        memcpy((*m_channel_list)[channel_count].field, m_map->data[channel_count].field, FIELD_LEN);
        (*m_channel_list)[channel_count].m_c2e = m_map->data[channel_count].m_c2e;
        (*m_channel_list)[channel_count].b_e2e = m_map->data[channel_count].b_e2e;
        (*m_channel_list)[channel_count].type = m_map->data[channel_count].type;
        (*m_channel_list)[channel_count].rate = m_map->data[channel_count].rate;
        memcpy((*m_channel_list)[channel_count].quantity, m_map->data[channel_count].quantity, UNITS_LEN);
        memcpy((*m_channel_list)[channel_count].units, m_map->data[channel_count].units, UNITS_LEN);
        (*m_channel_list)[channel_count].var = NULL;
    }

    return m_map->length;
}

/**
 * Translates a stored derived packet to the derived structure
 * @param m_map Pointer to the #derived_header_t structure storing our packet
 * @param m_len Length in bytes of the packet passed via m_map
 * @param m_channel_list Double pointer to where we will store the channel_list
 * @return -1 on failure, positive number of channels read otherwise
 */
int channels_read_derived_map(derived_header_t *m_map, size_t m_len, derived_tng_t **m_channel_list)
{
    uint32_t crcval = m_map->crc;

    if (m_map->version != (BLAST_TNG_CH_VERSION | 0x20)) {  // 0x20 marks the packet as a derived packet
        blast_err("Unknown channels version %d", m_map->version);
        return -1;
    }

    if (m_len < sizeof(derived_header_t)) {
        blast_err("Invalid size %zu for derived packet", m_len);
        return -1;
    }

    if (m_len != sizeof(derived_header_t) + m_map->length * sizeof(derived_tng_t)) {
        blast_err("Length of data packet %zu does not match header data %zu",
                  m_len, sizeof(derived_header_t) + m_map->length * sizeof(derived_tng_t));
        return -1;
    }

    m_map->crc = 0;
    if (crcval != PMurHash32(BLAST_MAGIC32, m_map, m_len)) {
        blast_err("CRC match failed!");
        return -1;
    }
    m_map->crc = crcval;

    *m_channel_list = balloc(err, sizeof(derived_tng_t) * m_map->length);
    if (!(*m_channel_list)) return -1;


    memcpy(*m_channel_list, m_map->data, sizeof(derived_tng_t) * m_map->length);

    return m_map->length;
}

channel_t *channels_find_by_name(const char *m_name)
{
    channel_t *retval = (channel_t*)g_hash_table_lookup(frame_table, m_name);

    if (!retval) blast_err("Could not find %s!\n", m_name);
    return retval;
}

int channels_check_size_of_frame(E_RATE m_rate, size_t m_len)
{
	if (m_len != frame_size[m_rate]) {
		blast_err("Size mismatch storing data for %s! Got %zu bytes, expected %zu",
		        RATE_LOOKUP_TABLE[m_rate].text, m_len, frame_size[m_rate]);
		return -1;
	}
    return 0;
}

int channels_store_data(E_RATE m_rate, const void *m_data, size_t m_len)
{
	if (m_len != frame_size[m_rate]) {
		blast_err("Size mismatch storing data for %s! Got %zu bytes, expected %zu",
		        RATE_LOOKUP_TABLE[m_rate].text, m_len, frame_size[m_rate]);
		return -1;
	}

	memcpy(channel_data[m_rate], m_data, m_len);
	return 0;
}

/**
 * Initialize the channels structure and associated hash tables.
 * @return 0 on success.  -1 otherwise
 */
int channels_initialize(const channel_t * const m_channel_list)
{
    const channel_t *channel;

    if (frame_table) g_hash_table_destroy(frame_table);
    frame_table = g_hash_table_new(channel_hash, g_str_equal);

    if (frame_table == NULL) return -1;

    for (int j = 0; j < RATE_END; j++) {
        for (int k = 0; k < TYPE_END; k++) {
          channel_count[j][k] = 0;
        }
        if (channel_data[j]) {
            free(channel_data[j]);
            channel_data[j] = NULL;
        }
    }

    /**
     * First Pass:  Add each entry in the channels array to a hash table for later lookup.
     * Then count each type of channel, separating by source, variable type and rate
     */
    channels_count = 0;
    for (channel = m_channel_list; channel->field[0]; channel++) {
        g_hash_table_insert(frame_table, (gpointer)channel->field, (gpointer)channel);
        if (channel->rate < RATE_END && channel->type < TYPE_END) {
            channel_count[channel->rate][channel->type]++;
        } else {
            blast_fatal("Could not map %d and %d to rate and type!, %s", channel->rate, channel->type, channel->field);
            return 1;
        }
        channels_count++;
    }

    /**
     * Second Pass: Allocate a set of packed arrays representing the data frames for each source/rate.
     * We also set channel_ptr, our placeholder for the next free element in the array, to the first entry in each frame.
     */

    for (int rate = 0; rate < RATE_END; rate++) {
        frame_size[rate] = (channel_count[rate][TYPE_INT8]+channel_count[rate][TYPE_UINT8]) +
                       2 * (channel_count[rate][TYPE_INT16]+channel_count[rate][TYPE_UINT16]) +
                       4 * (channel_count[rate][TYPE_INT32]+channel_count[rate][TYPE_UINT32] +
                            channel_count[rate][TYPE_FLOAT]) +
                       8 * (channel_count[rate][TYPE_INT64]+channel_count[rate][TYPE_UINT64] +
                            channel_count[rate][TYPE_DOUBLE]);

        if (frame_size[rate]) {
            /**
             * Ensure that we can dereference the data without knowing its type by
             * taking at least 8 bytes
             */
            size_t allocated_size = MAX(frame_size[rate], sizeof(uint64_t));
            channel_data[rate] = calloc(1, allocated_size);
            blast_info("Allocating %zu bytes for %u channels at %s", frame_size[rate],
                    (channel_count[rate][TYPE_INT8]+channel_count[rate][TYPE_UINT8]) +
                    (channel_count[rate][TYPE_INT16]+channel_count[rate][TYPE_UINT16]) +
                    (channel_count[rate][TYPE_INT32]+channel_count[rate][TYPE_UINT32] +
                            channel_count[rate][TYPE_FLOAT]) +
                    (channel_count[rate][TYPE_INT64]+channel_count[rate][TYPE_UINT64] +
                            channel_count[rate][TYPE_DOUBLE]),
                    RATE_LOOKUP_TABLE[rate].text);
        } else {
            channel_data[rate] = NULL;
        }
        channel_ptr[rate] = channel_data[rate];
    }

    /**
     * Third Pass: Iterate over the hash table and assign the lookup pointers to their place in the frame.
     */
    // deprecated in ubuntu 20.04 and onward, replacing with loop
    // g_hash_table_foreach(frame_table, channel_map_fields, NULL);
    for (channel_t * channel = m_channel_list; channel->field[0]; channel++) {
        if (channel->rate < RATE_END) {
            if (!channel_ptr[channel->rate]) {
                blast_fatal("Invalid Channel setup");
            }
            channel->var = channel_ptr[channel->rate];
            channel_ptr[channel->rate] += channel_size(channel);
        } else {
            blast_fatal("Could not map %d to rate!", channel->rate);
        }
    }

    // generate superframe
    superframe = channels_generate_superframe(m_channel_list);

    blast_startup("Successfully initialized Channels data structures");
    return 0;
}

int get_roach_index(unsigned int roach, unsigned int kid, unsigned int rtype) {
  if (roach > NUM_ROACHES) {
    blast_err("Invalid roach %d", roach);
    return -1;
  }
  if (kid >= NUM_KIDS) {
    blast_err("Invalid kid %d", kid);
    return -1;
  }
  if (rtype >= NUM_RTYPES) {
    blast_err("Invalid rtype %d", rtype);
    return -1;
  }

  return kid+(roach-1)*NUM_KIDS+rtype*NUM_ROACHES*NUM_KIDS;
}

void read_roach_index(unsigned int *roach, unsigned int *kid, unsigned int *rtype, unsigned int roach_index) {
  if (rtype) *rtype = roach_index/(NUM_ROACHES*NUM_KIDS);
  roach_index %= NUM_ROACHES*NUM_KIDS;

  if (roach) *roach = roach_index/(NUM_KIDS)+1;
  roach_index %= NUM_KIDS;

  if (kid) *kid = roach_index;

  if (*roach > NUM_ROACHES) {
    blast_err("Invalid roach %d", *roach);
    *roach = 1;
  }
  if (*kid >= NUM_KIDS) {
    blast_err("Invalid kid %d", *kid);
    *kid = 0;
  }
  if (*rtype >= NUM_RTYPES) {
    blast_err("Invalid rtype %d", *rtype);
    *rtype = 0;
  }
}

#define MAX_ROACH_NAME 64

void make_name_from_roach_index(unsigned int roach_index, char * name) {
  unsigned int roach = 0, kid = 0, rtype = 0;
  read_roach_index(&roach, &kid, &rtype, roach_index);

  if (roach > NUM_ROACHES) {
    blast_err("Invalid roach %d", roach);
    return;
  }
  if (kid >= NUM_KIDS) {
    blast_err("Invalid kid %d", kid);
    return;
  }
  if (rtype >= NUM_RTYPES) {
    blast_err("Invalid rtype %d", rtype);
    return;
  }

  snprintf(name, MAX_ROACH_NAME, "%s_kid%.04d_roach%.01d", ROACH_TYPES[rtype], kid, roach);
}

linklist_t * generate_housekeeping_linklist(linklist_t * ll_hk, char * name) {
    int i;
    int count = 0;
    unsigned int blk_size = 0;

    for (i = 0; i < ll_hk->n_entries; i++) {
        if (ll_hk->items[i].tlm) {
            if ((strncmp(ll_hk->items[i].tlm->field+2, "kid", 3) == 0) &&
                 (strncmp(ll_hk->items[i].tlm->field+10, "roach", 5) == 0)) {
                // linklist_info("Starting to ignore fields after %s\n", ll_hk->items[i].tlm->field);
                break;
            }
        }
        blk_size += ll_hk->items[i].blk_size;
        count++;
    }

    // modify the linklist name
    strncpy(ll_hk->name, name, 63);

    // modify the linklist size
    ll_hk->n_entries = count;

    // modify the linklist bulk size
    ll_hk->blk_size = blk_size;

    // modify the serial
    /*
    for (i = 0; i < MD5_DIGEST_LENGTH; i++) {
      if (!name[i]) break;
      ll_hk->serial[i] ^= name[i];
    }
    */

    return ll_hk;
}

#define EXTRA_SF_ENTRIES 2

superframe_t * channels_generate_superframe(const channel_t * const m_channel_list) {
    superframe_entry_t * sf = calloc(channels_count+EXTRA_SF_ENTRIES+1, sizeof(superframe_entry_t));

    unsigned int sf_size = 0;

    int rate = 0;
    for (rate = 0; rate < RATE_END; rate++) {
      superframe_offset[rate] = sf_size;
      sf_size += frame_size[rate]*get_spf(rate);
    }

    int i = 0;
    const channel_t *channel;
    for (channel = m_channel_list; channel->field[0]; channel++) {
       strncpy(sf[i].field, channel->field, FIELD_LEN-1);
       sf[i].type = superframe_type_array[channel->type];
       sf[i].spf = get_spf(channel->rate);
       sf[i].start = (int64_t) (channel->var-channel_data[channel->rate])+superframe_offset[channel->rate];
       sf[i].skip = frame_size[channel->rate];
       if (strlen(channel->quantity)) strncpy(sf[i].quantity, channel->quantity, UNITS_LEN-1);
       if (strlen(channel->units)) strncpy(sf[i].units, channel->units, UNITS_LEN-1);
       sf[i].var = channel->var;

       i++;
    }

    // null terminate
    sf[i].field[0] = '\0';

    return linklist_build_superframe(sf, &channel_data_to_double, &channel_double_to_data, SF_USE_BIG_ENDIAN);
}

/**
 * add_frame_to_superframe
 * 
 * Takes a BLAST frame at a given rate and copies it to the superframe.
 * -> frame: BLAST frame to be copied to the superframe
 * -> rate: the rate type for the BLAST frame
 */
unsigned int add_frame_to_superframe(void * frame, E_RATE rate, void * superframe, unsigned int * frame_location)
{
  if (!superframe) {
    blast_err("Superframe is not allocated. Fix!");
    return 0;
  }
  if (!frame) {
    blast_err("Frame pointer is NULL. Fix!");
    return 0;
  }

  // clear the frame if wrapping has occurred (ensures no split data)
  if (*frame_location == 0) {
    memset(superframe+superframe_offset[rate], 0, frame_size[rate]*get_spf(rate));
  }

  // copy the frame to the superframe
  memcpy(superframe+superframe_offset[rate]+frame_size[rate]*(*frame_location), frame, frame_size[rate]);

  // update the frame location
  *frame_location = ((*frame_location)+1)%get_spf(rate);

  // return the next frame location in the superframe
  return *frame_location;
}

/**
 * extract_frame_from_superframe
 * 
 * Extracts a BLAST frame at a given rate from superframe and copies it to a given buffer.
 * -> frame: BLAST frame to be copied from the superframe
 * -> rate: the rate type for the BLAST frame
 */
unsigned int extract_frame_from_superframe(void * frame, E_RATE rate, void * superframe, unsigned int * frame_location)
{
  if (!superframe) {
    blast_err("Superframe is not allocated. Fix!");
    return 0;
  }
  if (!frame) {
    blast_err("Frame pointer is NULL. Fix!");
    return 0;
  }

  // copy the frame from the superframe
  memcpy(frame, superframe+superframe_offset[rate]+frame_size[rate]*(*frame_location), frame_size[rate]);

  // update the frame location
  *frame_location = ((*frame_location)+1)%get_spf(rate);

  // return the next frame location in the superframe
  return *frame_location;
}

double channel_data_to_double(uint8_t * data, uint8_t type)
{
  switch (type) {
    case SF_FLOAT64 : return bedtoh(*((double *) data));
    case SF_FLOAT32 : return beftoh(*((float *) data));
    case SF_INT16 : return (int16_t) be16toh(*((int16_t *) data));
    case SF_UINT16 : return be16toh(*((uint16_t *) data));
    case SF_INT32 : return (int32_t) be32toh(*((int32_t *) data));
    case SF_UINT32 : return be32toh(*((uint32_t *) data));
    case SF_INT8 : return *((int8_t *) data);
    case SF_UINT8 : return *((uint8_t *) data);
    default : return 0;
  }
  return 0;
}
int channel_double_to_data(uint8_t * data, double dub, uint8_t type)
{
  if (type == SF_FLOAT64) {
    htobed(dub, *(uint64_t*) data);
    return 8;
  } else if (type == SF_FLOAT32) {
    htobef(dub, *(uint32_t*) data);
    return 4;
  } else if (type == SF_INT16) {
    int16_t s = dub;
    *(int16_t*) data = htobe16(s);
    return 2;
  } else if (type == SF_UINT16) {
    uint16_t u = dub;
    *(uint16_t*) data = htobe16(u);
    return 2;
  } else if (type == SF_INT32) {
    int32_t i = dub;
    *(int32_t*) data = htobe32(i);
    return 4;
  } else if (type == SF_UINT32) {
    uint32_t i = dub;
    *(uint32_t*) data = htobe32(i);
    return 4;
  } else if (type == SF_INT8) {
    *(int8_t*) data = dub;
    return 1;
  } else if (type == SF_UINT8) {
    *(uint8_t*) data = dub;
    return 1;
  }
  return 0;
}

void channels_write_calspecs_item(FILE *calspecsfile, derived_tng_t *derived) {
    int j;

    switch (derived->type) {
      case 'w':
      case 'b':
        fprintf(calspecsfile, "%s BIT %s %u %u\n", derived->bitword.field, derived->bitword.source,
                           derived->bitword.offset, derived->bitword.length);
        break;
      case 't':
        fprintf(calspecsfile, "%s LINTERP %s %s\n", derived->linterp.field, derived->linterp.source,
                          derived->linterp.lut);
        break;
      case 'c':
        fprintf(calspecsfile, "%s LINCOM 1 %s %.16f %.16f\n", derived->lincom.field,
                          derived->lincom.source, derived->lincom.m_c2e, derived->lincom.b_e2e);
        break;
      case '2':
        fprintf(calspecsfile, "%s LINCOM 2 %s %.16f %.16f %s %.16f %.16f\n", derived->lincom2.field,
          derived->lincom2.source, derived->lincom2.m_c2e, derived->lincom2.b_e2e,
          derived->lincom2.source2, derived->lincom2.m2_c2e, derived->lincom2.b2_e2e);
        break;
      case '#':
        break;
      case 'u':
        if (derived->units.quantity[0]) {
          fprintf(calspecsfile, "%s/quantity STRING \"", derived->units.source);
          for (j = 0; j < strlen(derived->units.quantity); j++) {
            if (derived->units.quantity[j] == 92) fprintf(calspecsfile, "\\"); // fix getdata escape
            fprintf(calspecsfile, "%c", derived->units.quantity[j]);
          }
          fprintf(calspecsfile, "\"\n");
        }
        if (derived->units.units[0]) {
          fprintf(calspecsfile, "%s/units STRING \"", derived->units.source);
          for (j = 0; j < strlen(derived->units.units); j++) {
            if (derived->units.units[j] == 92) fprintf(calspecsfile, "\\"); // fix getdata escape
            fprintf(calspecsfile, "%c", derived->units.units[j]);
          }
          fprintf(calspecsfile, "\"\n");
        }
        break;
      case 'p':
        fprintf(calspecsfile, "%s PHASE %s %d\n", derived->phase.field, derived->phase.source, derived->phase.shift);
        break;
      case 'r':
        fprintf(calspecsfile, "%s RECIP %s %.16f\n", derived->recip.field, derived->recip.source,
                           derived->recip.dividend);
        break;
      case '*':
        fprintf(calspecsfile, "%s MULTIPLY %s %s\n", derived->math.field, derived->math.source, derived->math.source2);
        break;
      case '/':
        fprintf(calspecsfile, "%s DIVIDE %s %s\n", derived->math.field, derived->math.source, derived->math.source2);
        break;
      case 'x':
        fprintf(calspecsfile, "%s MPLEX %s %s %d %d\n", derived->mplex.field, derived->mplex.source,
                                derived->mplex.index, derived->mplex.value, derived->mplex.max);
        break;
      default:
        blast_warn("Unknown type %c", derived->type);
        break;
    }
}

void channels_write_calspecs(char * fname, derived_tng_t *m_derived)
{
  FILE * calspecsfile = fopen(fname, "w");
  if (!calspecsfile) {
    blast_err("Could not open \"%s\" as calspecs file\n", fname);
    return;
  }

  for (derived_tng_t *derived = m_derived; derived && derived->type != DERIVED_EOC_MARKER; derived++) {
    channels_write_calspecs_item(calspecsfile, derived);
  }

  derived_tng_t derived = {0};
  char tmp_str[128] = {0};
  for (channel_t *channel = channel_list; channel->field[0]; channel++) {
    snprintf(tmp_str, sizeof(tmp_str), "%s", channel->field);
    double m = channel->m_c2e;
    double b = channel->b_e2e;

    // don't do roach channels; we have something special for that (see below)
    if ((strstr(tmp_str, "roach") != NULL) && (strstr(tmp_str, "kid") != NULL)) {
      continue;
    }

    /// By default we set the converted field to upper case
    for (int i = 0; tmp_str[i]; i++) tmp_str[i] = toupper(tmp_str[i]);
    /// If our scale/offset are unity/zero respectively, tell defile to use the easier zero-phase
    if (fabs(m - 1.0) <= DBL_EPSILON && fabs(b - 0.0) <= DBL_EPSILON) {
      derived.type = 'p';
      snprintf(derived.phase.field, sizeof(derived.phase.field), "%s", tmp_str);
      snprintf(derived.phase.source, sizeof(derived.phase.source), "%s", channel->field);
      derived.phase.shift = 0;
    } else {
      derived.type = 'c';
      snprintf(derived.lincom.field, sizeof(derived.lincom.field), "%s", tmp_str);
      snprintf(derived.lincom.source, sizeof(derived.lincom.source), "%s", channel->field);
      derived.lincom.m_c2e = m;
      derived.lincom.b_e2e = b;
    }
    channels_write_calspecs_item(calspecsfile, &derived);
  }

  // something special: generate an array of derived fields for each roach channel
  // multiplex roach index fields will point to the corresponding name for display in kst
  fprintf(calspecsfile, ROACH_NAME_SARRAY " SARRAY");

  int kid = 0, roach = 1, rtype = 0;
  for (rtype = 0; rtype < NUM_RTYPES; rtype++) {
    for (roach = 1; roach <= NUM_ROACHES; roach++) {
      for (kid = 0; kid < NUM_KIDS; kid++) {
        char tlm_name[64] = {0};
        unsigned int index = get_roach_index(roach, kid, rtype);
        make_name_from_roach_index(index, tlm_name);
        for (int i = 0; tlm_name[i]; i++) tlm_name[i] = toupper(tlm_name[i]);
        fprintf(calspecsfile, " '%s'", tlm_name);
      }
    }
  }
  fprintf(calspecsfile, "\n");
  for (int i = 0; i < NUM_ROACH_TLM; i++) {
    char c = 65+i;
    fprintf(calspecsfile, "KID%c_ROACHN_NAME SINDIR kid%c_roachN_index " ROACH_NAME_SARRAY "\n", c, c);
  }
  // link multiplexed fields to derived fields
  for (rtype = 0; rtype < NUM_RTYPES; rtype++) {
    for (roach = 1; roach <= NUM_ROACHES; roach++) {
      for (kid = 0; kid < NUM_KIDS; kid++) {
        int index = get_roach_index(roach, kid, rtype);
        derived.type = 'x';
        make_name_from_roach_index(index, derived.mplex.field);
        for (int i = 0; derived.mplex.field[i]; i++) {
          derived.mplex.field[i] = toupper(derived.mplex.field[i]);
        }
        char c = 65+((roach-1)*3)+rtype;
        snprintf(derived.mplex.source, sizeof(derived.mplex.source), "kid%c_roachN", c);
        snprintf(derived.mplex.index, sizeof(derived.mplex.index), "kid%c_roachN_index", c);
        derived.mplex.value = index;
        derived.mplex.max = 0;

        channels_write_calspecs_item(calspecsfile, &derived);
      }
    }
  }




  fflush(calspecsfile);
  fclose(calspecsfile);
}
