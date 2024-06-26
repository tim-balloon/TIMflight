/**************************************************************************
 *   Copyright (C) 2012 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <libusb-1.0/libusb.h>
#include <sys/time.h>

#include "binarybuffer.h"
#include "mpsse.h"
#include "blast.h"

static uint8_t bits;

/* Compatibility define for older libusb-1.0 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#ifdef _DEBUG_JTAG_IO_
#define DEBUG_IO(expr...) blast_dbg(expr)
#define DEBUG_PRINT_BUF(buf, len) \
    do { \
        char buf_string[32 * 3 + 1]; \
        int buf_string_pos = 0; \
        for (int i = 0; i < len; i++) { \
            /* NOLINT */ buf_string_pos += sprintf(buf_string + buf_string_pos, " %02x", buf[i]); \
            if (i % 32 == 32 - 1) { \
                blast_dbg("%s", buf_string); \
                buf_string_pos = 0; \
            } \
        } \
        if (buf_string_pos > 0) \
            blast_info("%s", buf_string);\
    } while (0)
#else
#define DEBUG_IO(expr...) do {} while (0)
#define DEBUG_PRINT_BUF(buf, len) do {} while (0)
#endif

#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define FTDI_DEVICE_IN_REQTYPE (0x80 | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)

#define BITMODE_MPSSE 0x02

#define SIO_RESET_REQUEST             0x00
#define SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define SIO_GET_LATENCY_TIMER_REQUEST 0x0A
#define SIO_SET_BITMODE_REQUEST       0x0B

#define SIO_RESET_SIO 0
#define SIO_RESET_PURGE_RX 1
#define SIO_RESET_PURGE_TX 2

/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool string_descriptor_equal(libusb_device_handle *device, uint8_t str_index,
    const char *string)
{
    int retval;
    char desc_string[256]; /* Max size of string descriptor */
    retval = libusb_get_string_descriptor_ascii(device, str_index, (unsigned char *)desc_string,
            sizeof(desc_string));
    if (retval < 0) {
        blast_err("libusb_get_string_descriptor_ascii() failed with %s", libusb_error_name(retval));
        return false;
    }
    return strncmp(string, desc_string, sizeof(desc_string)) == 0;
}

/* Helper to open a libusb device that matches vid, pid, product string and/or serial string.
 * Set any field to 0 as a wildcard. If the device is found true is returned, with ctx containing
 * the already opened handle. ctx->interface must be set to the desired interface (channel) number
 * prior to calling this function. */
static bool open_matching_device(struct mpsse_ctx *ctx, const uint16_t *vid, const uint16_t *pid,
    const char *product, const char *serial)
{
    libusb_device **list;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor *config0;
    int reterr;
    bool found = false;
    ssize_t cnt = libusb_get_device_list(ctx->usb_ctx, &list);

  static unsigned int has_warned;

    if (cnt < 0)
        if (!has_warned) blast_err("libusb_get_device_list() failed with %s", libusb_error_name(cnt));

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *device = list[i];

        reterr = libusb_get_device_descriptor(device, &desc);
        if (reterr != LIBUSB_SUCCESS) {
            if (!has_warned) blast_err("libusb_get_device_descriptor() failed with %s", libusb_error_name(reterr));
            continue;
        }

        if (vid && *vid != desc.idVendor)
            continue;
        if (pid && *pid != desc.idProduct)
            continue;

        reterr = libusb_open(device, &ctx->usb_dev);
        if (reterr != LIBUSB_SUCCESS) {
            if (!has_warned) blast_err("libusb_open() failed with %s",
                  libusb_error_name(reterr));
            continue;
        }

        if (product && !string_descriptor_equal(ctx->usb_dev, desc.iProduct, product)) {
            if (!has_warned) libusb_close(ctx->usb_dev);
            continue;
        }

        if (serial && !string_descriptor_equal(ctx->usb_dev, desc.iSerialNumber, serial)) {
            if (!has_warned) libusb_close(ctx->usb_dev);
            continue;
        }

        found = true;
        blast_info("Success opening MPSSE device with VID=%04x, PID=%04x and serial=%s", *vid, *pid, serial);

        break;
    }

    libusb_free_device_list(list, 1);

    if (!found) {
        if (!has_warned) blast_err("no device found");
    has_warned = 1;
        return false;
    }

    reterr = libusb_get_config_descriptor(libusb_get_device(ctx->usb_dev), 0, &config0);
    if (reterr != LIBUSB_SUCCESS) {
        if (!has_warned) blast_err("libusb_get_config_descriptor() failed with %s", libusb_error_name(reterr));
        libusb_close(ctx->usb_dev);
    has_warned = 1;
        return false;
    }

    /* Make sure the first configuration is selected */
    int cfg;
    reterr = libusb_get_configuration(ctx->usb_dev, &cfg);
    if (reterr != LIBUSB_SUCCESS) {
        if (!has_warned) blast_err("libusb_get_configuration() failed with %s", libusb_error_name(reterr));
        goto error;
    }

    if (desc.bNumConfigurations > 0 && cfg != config0->bConfigurationValue) {
        reterr = libusb_set_configuration(ctx->usb_dev, config0->bConfigurationValue);
        if (reterr != LIBUSB_SUCCESS) {
            if (!has_warned) blast_err("libusb_set_configuration() failed with %s", libusb_error_name(reterr));
            goto error;
        }
    }

    /* Try to detach ftdi_sio kernel module */
    reterr = libusb_detach_kernel_driver(ctx->usb_dev, ctx->interface);
    if (reterr != LIBUSB_SUCCESS && reterr != LIBUSB_ERROR_NOT_FOUND
            && reterr != LIBUSB_ERROR_NOT_SUPPORTED) {
        if (!has_warned) blast_err("libusb_detach_kernel_driver() failed with %s", libusb_error_name(reterr));
        goto error;
    }

    reterr = libusb_claim_interface(ctx->usb_dev, ctx->interface);
    if (reterr != LIBUSB_SUCCESS) {
        if (!has_warned) blast_err("libusb_claim_interface() failed with %s", libusb_error_name(reterr));
        goto error;
    }

    /* Reset FTDI device */
    reterr = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            SIO_RESET_REQUEST, SIO_RESET_SIO,
            ctx->index, NULL, 0, ctx->usb_write_timeout);
    if (reterr < 0) {
        if (!has_warned) blast_err("failed to reset FTDI device: %s", libusb_error_name(reterr));
        goto error;
    }

    switch (desc.bcdDevice) {
    case 0x500:
        ctx->type = TYPE_FT2232C;
        break;
    case 0x700:
        ctx->type = TYPE_FT2232H;
        break;
    case 0x800:
        ctx->type = TYPE_FT4232H;
        break;
    case 0x900:
        ctx->type = TYPE_FT232H;
        break;
    default:
        if (!has_warned) blast_err("unsupported FTDI chip type: 0x%04x", desc.bcdDevice);
        goto error;
    }

    /* Determine maximum packet size and endpoint addresses */
    if (!(desc.bNumConfigurations > 0 && ctx->interface < config0->bNumInterfaces
            && config0->interface[ctx->interface].num_altsetting > 0))
        goto desc_error;

    const struct libusb_interface_descriptor *descriptor;
    descriptor = &config0->interface[ctx->interface].altsetting[0];
    if (descriptor->bNumEndpoints != 2)
        goto desc_error;

    ctx->in_ep = 0;
    ctx->out_ep = 0;
    for (int i = 0; i < descriptor->bNumEndpoints; i++) {
        if (descriptor->endpoint[i].bEndpointAddress & 0x80) {
            ctx->in_ep = descriptor->endpoint[i].bEndpointAddress;
            ctx->max_packet_size =
                    descriptor->endpoint[i].wMaxPacketSize;
        } else {
            ctx->out_ep = descriptor->endpoint[i].bEndpointAddress;
        }
    }

    if (ctx->in_ep == 0 || ctx->out_ep == 0)
        goto desc_error;

    libusb_free_config_descriptor(config0);
  has_warned = 0;
    return true;

desc_error:
    if (!has_warned) blast_err("unrecognized USB device descriptor");
error:
  has_warned = 1;
    libusb_free_config_descriptor(config0);
    libusb_close(ctx->usb_dev);
    return false;
}

struct mpsse_ctx *mpsse_open(const uint16_t *vid, const uint16_t *pid, const char *description,
    const char *serial, int channel)
{
    struct mpsse_ctx *ctx = calloc(1, sizeof(*ctx));
    int reterr;

  static unsigned int has_warned;

    if (!ctx)
        return 0;

    bit_copy_queue_init(&ctx->read_queue);
    ctx->read_chunk_size = 16384; // 2**14, in bytes
    ctx->read_size = 16384;
    ctx->write_size = 16384;
    ctx->read_chunk = malloc(ctx->read_chunk_size);
    ctx->read_buffer = malloc(ctx->read_size);
    ctx->write_buffer = malloc(ctx->write_size);
    if (!ctx->read_chunk || !ctx->read_buffer || !ctx->write_buffer)
        goto error;

    ctx->interface = channel;
    ctx->index = channel + 1;
    ctx->usb_read_timeout = 5000;
    ctx->usb_write_timeout = 5000;

    reterr = libusb_init(&ctx->usb_ctx);
    if (reterr != LIBUSB_SUCCESS) {
        if (!has_warned) blast_err("libusb_init() failed with %s", libusb_error_name(reterr));
        goto error;
    }

    if (!open_matching_device(ctx, vid, pid, description, serial)) {
        /* Four hex digits plus terminating zero each */
        char vidstr[5];
        char pidstr[5];
        if (!has_warned) blast_err("unable to open ftdi device with vid %s, pid %s, description '%s' and "
                "serial '%s'",
                vid ? sprintf(vidstr, "%04x", *vid), vidstr : "*", // NOLINT
                pid ? sprintf(pidstr, "%04x", *pid), pidstr : "*", // NOLINT
                description ? description : "*",
                serial ? serial : "*");
        ctx->usb_dev = 0;
        goto error;
    }

    // Previous value was 255 rather than 2. This sets read timeout
    reterr = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE,
            SIO_SET_LATENCY_TIMER_REQUEST, 2, ctx->index, NULL, 0,
            ctx->usb_write_timeout);
    if (reterr < 0) {
        if (!has_warned) blast_err("unable to set latency timer: %s", libusb_error_name(reterr));
        goto error;
    }

    reterr = libusb_control_transfer(ctx->usb_dev,
            FTDI_DEVICE_OUT_REQTYPE,
            SIO_SET_BITMODE_REQUEST,
            0x0b | (BITMODE_MPSSE << 8),
            ctx->index,
            NULL,
            0,
            ctx->usb_write_timeout);
    if (reterr < 0) {
        if (!has_warned) blast_err("unable to set MPSSE bitmode: %s", libusb_error_name(reterr));
        goto error;
    }
  has_warned = 0;

    mpsse_purge(ctx);

    return ctx;
error:
  has_warned = 1;
    mpsse_close(ctx);
    return 0;
}

void mpsse_reset_purge_close(struct mpsse_ctx *ctx)
{
    int reterr;
    reterr = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_SET_BITMODE_REQUEST,
             0x0000, ctx->index, NULL, 0, ctx->usb_write_timeout);
    if (reterr < 0) {
        blast_info("error trying to set bitmode to RESET");
    }
    mpsse_purge(ctx);
    mpsse_close(ctx);
}


void mpsse_close(struct mpsse_ctx *ctx)
{
    if (ctx->usb_dev)
        libusb_close(ctx->usb_dev);
    if (ctx->usb_ctx)
        libusb_exit(ctx->usb_ctx);
    bit_copy_discard(&ctx->read_queue);
    if (ctx->write_buffer)
        free(ctx->write_buffer);
    if (ctx->read_buffer)
        free(ctx->read_buffer);
    if (ctx->read_chunk)
        free(ctx->read_chunk);

    free(ctx);
}

bool mpsse_is_high_speed(struct mpsse_ctx *ctx)
{
    return ctx->type != TYPE_FT2232C;
}

void mpsse_purge(struct mpsse_ctx *ctx)
{
    int reterr;
    blast_dbg("-");
    ctx->write_count = 0;
    ctx->read_count = 0;
    ctx->retval = ERROR_OK;
    bit_copy_discard(&ctx->read_queue);
    reterr = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST,
            SIO_RESET_PURGE_RX, ctx->index, NULL, 0, ctx->usb_write_timeout);
    if (reterr < 0) {
        blast_err("unable to purge ftdi rx buffers: %s", libusb_error_name(reterr));
        return;
    }

    reterr = libusb_control_transfer(ctx->usb_dev, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST,
            SIO_RESET_PURGE_TX, ctx->index, NULL, 0, ctx->usb_write_timeout);
    if (reterr < 0) {
        blast_err("unable to purge ftdi tx buffers: %s", libusb_error_name(reterr));
        return;
    }
}

static unsigned buffer_write_space(struct mpsse_ctx *ctx)
{
    /* Reserve one byte for SEND_IMMEDIATE */
    return ctx->write_size - ctx->write_count - 1;
}

static unsigned buffer_read_space(struct mpsse_ctx *ctx)
{
    return ctx->read_size - ctx->read_count;
}

static void buffer_write_byte(struct mpsse_ctx *ctx, uint8_t data)
{
    // DEBUG_IO("About to write: %02x", data);
    assert(ctx->write_count < ctx->write_size);
    ctx->write_buffer[ctx->write_count++] = data;
}

static unsigned buffer_write(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
    unsigned bit_count)
{
    // DEBUG_IO("About to write %d bits starting at %d", bit_count, out_offset);
    // DEBUG_PRINT_BUF(out, bit_count/8);
    assert(ctx->write_count + DIV_ROUND_UP(bit_count, 8) <= ctx->write_size);
    bit_copy(ctx->write_buffer + ctx->write_count, 0, out, out_offset, bit_count);
    ctx->write_count += DIV_ROUND_UP(bit_count, 8);
    return bit_count;
}

static unsigned buffer_add_read(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset,
    unsigned bit_count, unsigned offset)
{
    // DEBUG_IO("%d bits, offset %d", bit_count, offset);
    assert(ctx->read_count + DIV_ROUND_UP(bit_count, 8) <= ctx->read_size);
    bit_copy_queued(&ctx->read_queue, in, in_offset, ctx->read_buffer + ctx->read_count, offset,
        bit_count);
    ctx->read_count += DIV_ROUND_UP(bit_count, 8);
    return bit_count;
}

void mpsse_clock_data_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
    unsigned length, uint8_t mode)
{
    mpsse_clock_data(ctx, out, out_offset, 0, 0, length, mode);
}

void mpsse_clock_data_in(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
    uint8_t mode)
{
    mpsse_clock_data(ctx, 0, 0, in, in_offset, length, mode);
}

void mpsse_clock_data(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
    unsigned in_offset, unsigned length, uint8_t mode)
{
    /* TODO: Fix MSB first modes */
    DEBUG_IO("Data %s%s, need to write %d bits", in ? "in" : "", out ? "out" : "", length);

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    /* TODO: On H chips, use command 0x8E/0x8F if in and out are both 0 */
    if (out || (!out && !in))
        mode |= 0x10;
    if (in)
        mode |= 0x20;


    while (length > 0) {
        /* Guarantee buffer space enough for a minimum size transfer */
        if (buffer_write_space(ctx) + (length < 8) < (out || (!out && !in) ? 4 : 3) \
            || (in && buffer_read_space(ctx) < 1)) {
            DEBUG_IO("About to flush in mpsse_clock_data, buffer has only %d bits lefts but want to write %d bits", \
                buffer_write_space(ctx), length);
            ctx->retval = mpsse_flush(ctx);
        }

        if (length < 8) {
            /* Transfer remaining bits in bit mode */
            buffer_write_byte(ctx, 0x02 | mode);
            buffer_write_byte(ctx, length - 1);
            if (out) {
                out_offset += buffer_write(ctx, out, out_offset, length);
            }
            if (in) {
                in_offset += buffer_add_read(ctx, in, in_offset, length, 8 - length);
            }
            if (!out && !in) {
                buffer_write_byte(ctx, 0x00);
            }
            length = 0;
        } else {
            /* Byte transfer */
            unsigned this_bytes = length / 8;
            /* MPSSE command limit */
            if (this_bytes > 65536) {
                this_bytes = 65536;
            }
            /* Buffer space limit. We already made sure there's space for the minimum
             * transfer. */
            if ((out || (!out && !in)) && this_bytes + 3 > buffer_write_space(ctx)) {
                this_bytes = buffer_write_space(ctx) - 3;
            }
            if (in && this_bytes > buffer_read_space(ctx)) {
                this_bytes = buffer_read_space(ctx);
            }

            if (this_bytes > 0) {
                buffer_write_byte(ctx, mode);
                buffer_write_byte(ctx, (this_bytes - 1) & 0xff);
                buffer_write_byte(ctx, (this_bytes - 1) >> 8);
                if (out) {
                    DEBUG_IO("This is the main writing loop, about to write %d bytes =  %d bits", \
                        this_bytes, this_bytes*8);
                    // DEBUG_PRINT_BUF(out, this_bytes);
                    out_offset += buffer_write(ctx,
                            out,
                            out_offset,
                            this_bytes * 8);
                }
                if (in) {
                    in_offset += buffer_add_read(ctx,
                            in,
                            in_offset,
                            this_bytes * 8,
                            0);
                }
                if (!out && !in) {
                    for (unsigned n = 0; n < this_bytes; n++)
                        buffer_write_byte(ctx, 0x00);
                }
                length -= this_bytes * 8;
                DEBUG_IO("After writing %d bits, length in bits still to write is %d", this_bytes*8, length);
            }
        }
    }
}


void mpsse_set_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir)
{
    DEBUG_IO("-");

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 3) {
        DEBUG_IO("About to flush in mpsse_set_data_bits_low_byte");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, 0x80);
    buffer_write_byte(ctx, data);
    buffer_write_byte(ctx, dir);
}

void mpsse_set_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir)
{
    DEBUG_IO("-");

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 3) {
        DEBUG_IO("About to flush in mpsse_set_data_bits_high_byte");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, 0x82);
    buffer_write_byte(ctx, data);
    buffer_write_byte(ctx, dir);
}

void mpsse_read_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t *data)
{
    DEBUG_IO("-");

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 1 || buffer_read_space(ctx) < 1) {
        DEBUG_IO("About to flush in mpsse_read_data_bits_low");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, 0x81);
    buffer_add_read(ctx, data, 0, 8, 0);
}

void mpsse_read_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t *data)
{
    DEBUG_IO("-");

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 1 || buffer_read_space(ctx) < 1) {
        DEBUG_IO("About to flush in mpsse_read_data_bits_high");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, 0x83);
    buffer_add_read(ctx, data, 0, 8, 0);
}

static void single_byte_boolean_helper(struct mpsse_ctx *ctx, bool var, uint8_t val_if_true,
    uint8_t val_if_false)
{
    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 1) {
        DEBUG_IO("About to flush in single_byte_boolean_helper");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, var ? val_if_true : val_if_false);
}

void mpsse_loopback_config(struct mpsse_ctx *ctx, bool enable)
{
    blast_dbg("%s", enable ? "on" : "off");
    single_byte_boolean_helper(ctx, enable, 0x84, 0x85);
}

void mpsse_set_divisor(struct mpsse_ctx *ctx, uint16_t divisor)
{
    blast_dbg("%d", divisor);

    if (ctx->retval != ERROR_OK) {
        DEBUG_IO("Ignoring command due to previous error");
        return;
    }

    if (buffer_write_space(ctx) < 3) {
        DEBUG_IO("About to flush in mpsse_set_divisor");
        ctx->retval = mpsse_flush(ctx);
    }

    buffer_write_byte(ctx, 0x86);
    buffer_write_byte(ctx, divisor & 0xff);
    buffer_write_byte(ctx, divisor >> 8);
}

int mpsse_divide_by_5_config(struct mpsse_ctx *ctx, bool enable)
{
    if (!mpsse_is_high_speed(ctx))
        return ERROR_FAIL;

    blast_dbg("%s", enable ? "on" : "off");
    single_byte_boolean_helper(ctx, enable, 0x8b, 0x8a);

    return ERROR_OK;
}

int mpsse_rtck_config(struct mpsse_ctx *ctx, bool enable)
{
    if (!mpsse_is_high_speed(ctx))
        return ERROR_FAIL;

    blast_dbg("%s", enable ? "on" : "off");
    single_byte_boolean_helper(ctx, enable, 0x96, 0x97);

    return ERROR_OK;
}

int mpsse_set_frequency(struct mpsse_ctx *ctx, int frequency)
{
    blast_dbg("target %d Hz", frequency);
    assert(frequency >= 0);
    int base_clock;

    if (frequency == 0)
        return mpsse_rtck_config(ctx, true);

    mpsse_rtck_config(ctx, false); /* just try */

    if (frequency > 60000000 / 2 / 65536 && mpsse_divide_by_5_config(ctx, false) == ERROR_OK) {
        base_clock = 60000000;
    } else {
        mpsse_divide_by_5_config(ctx, true); /* just try */
        base_clock = 12000000;
    }

    int divisor = (base_clock / 2 + frequency - 1) / frequency - 1;
    if (divisor > 65535)
        divisor = 65535;
    assert(divisor >= 0);

    mpsse_set_divisor(ctx, divisor);

    frequency = base_clock / 2 / (1 + divisor);
    blast_dbg("actually %d Hz", frequency);

    return frequency;
}

/* Context needed by the callbacks */
struct transfer_result {
    struct mpsse_ctx *ctx;
    bool done;
    unsigned transferred;
};

static LIBUSB_CALL void read_cb(struct libusb_transfer *transfer)
{
    struct transfer_result *res = transfer->user_data;
    struct mpsse_ctx *ctx = res->ctx;

    unsigned packet_size = ctx->max_packet_size;

    // DEBUG_PRINT_BUF(transfer->buffer, transfer->actual_length);

    /* Strip the two status bytes sent at the beginning of each USB packet
     * while copying the chunk buffer to the read buffer */
    unsigned num_packets = DIV_ROUND_UP(transfer->actual_length, packet_size);
    unsigned chunk_remains = transfer->actual_length;
    for (unsigned i = 0; i < num_packets && chunk_remains > 2; i++) {
        unsigned this_size = packet_size - 2;
        if (this_size > chunk_remains - 2)
            this_size = chunk_remains - 2;
        if (this_size > ctx->read_count - res->transferred)
            this_size = ctx->read_count - res->transferred;
        memcpy(ctx->read_buffer + res->transferred,
            ctx->read_chunk + packet_size * i + 2,
            this_size);
        res->transferred += this_size;
        chunk_remains -= this_size + 2;
        if (res->transferred == ctx->read_count) {
            res->done = true;
            break;
        }
    }

    DEBUG_IO("raw chunk %d, transferred %d of %d", transfer->actual_length, res->transferred,
        ctx->read_count);

    if (!res->done)
        if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
            res->done = true;
}

static LIBUSB_CALL void write_cb(struct libusb_transfer *transfer)
{
    struct transfer_result *res = transfer->user_data;
    struct mpsse_ctx *ctx = res->ctx;

    res->transferred += transfer->actual_length;

    // DEBUG_IO("transferred %d of %d", res->transferred, ctx->write_count);
    // DEBUG_PRINT_BUF(transfer->buffer, transfer->actual_length);
  blast_info("transferred %d of %d", res->transferred, ctx->write_count);

    if (res->transferred == ctx->write_count) {
        res->done = true;
    } else {
        transfer->length = ctx->write_count - res->transferred;
        transfer->buffer = ctx->write_buffer + res->transferred;
        if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
            res->done = true;
    }
}

int mpsse_flush(struct mpsse_ctx *ctx)
{
    int retval = ctx->retval;

    if (retval != ERROR_OK) {
        DEBUG_IO("Ignoring flush due to previous error");
        assert(ctx->write_count == 0 && ctx->read_count == 0);
        ctx->retval = ERROR_OK;
        return retval;
    }

    DEBUG_IO("write %d%s, read %d", ctx->write_count, ctx->read_count ? "+1" : "", ctx->read_count);
    assert(ctx->write_count > 0 || ctx->read_count == 0); /* No read data without write data */


    if (ctx->write_count == 0)
        return retval;


    struct libusb_transfer *read_transfer = 0;
    struct transfer_result read_result = { .ctx = ctx, .done = true };
    if (ctx->read_count) {
        buffer_write_byte(ctx, 0x87); /* SEND_IMMEDIATE */
        read_result.done = false;
        /* delay read transaction to ensure the FTDI chip can support us with data
           immediately after processing the MPSSE commands in the write transaction */
    }


    struct transfer_result write_result = { .ctx = ctx, .done = false };
    struct libusb_transfer *write_transfer = libusb_alloc_transfer(0);
    DEBUG_PRINT_BUF(ctx->write_buffer, ctx->write_count);
    libusb_fill_bulk_transfer(write_transfer, ctx->usb_dev, ctx->out_ep, ctx->write_buffer,
        ctx->write_count, write_cb, &write_result, ctx->usb_write_timeout);
    retval = libusb_submit_transfer(write_transfer);

    if (ctx->read_count) {
        read_transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(read_transfer, ctx->usb_dev, ctx->in_ep, ctx->read_chunk,
            ctx->read_chunk_size, read_cb, &read_result,
            ctx->usb_read_timeout);
        retval = libusb_submit_transfer(read_transfer);
    }

    /* Polling loop, more or less taken from libftdi */
    while (!write_result.done || !read_result.done) {
        retval = libusb_handle_events(ctx->usb_ctx);
    // blast_info("is write_result.done after handl_events? %d", (int) write_result.done);
        // TODO(void): Evaluate GDB Keepalive function
        // keep_alive();
        if (retval != LIBUSB_SUCCESS && retval != LIBUSB_ERROR_INTERRUPTED) {
            libusb_cancel_transfer(write_transfer);
    //        blast_dbg("Cancelling transfer because retval = %d", retval);
            if (read_transfer)
                libusb_cancel_transfer(read_transfer);
            while (!write_result.done || !read_result.done)
                if (libusb_handle_events(ctx->usb_ctx) != LIBUSB_SUCCESS)
                    break;
        }
    }
    if (retval != LIBUSB_SUCCESS) {
        blast_err("libusb_handle_events() failed with %s", libusb_error_name(retval));
        retval = ERROR_FAIL;
    } else if (write_result.transferred < ctx->write_count) {
        blast_err("ftdi device did not accept all data: %d, tried %d",
            write_result.transferred,
            ctx->write_count);
        retval = ERROR_FAIL;
    } else if (read_result.transferred < ctx->read_count) {
        blast_err("ftdi device did not return all data: %d, expected %d",
            read_result.transferred,
            ctx->read_count);
        retval = ERROR_FAIL;
    } else if (ctx->read_count) {
        ctx->write_count = 0;
        ctx->read_count = 0;
        bit_copy_execute(&ctx->read_queue);
        retval = ERROR_OK;
    } else {
        ctx->write_count = 0;
        bit_copy_discard(&ctx->read_queue);
        retval = ERROR_OK;
    }
    libusb_free_transfer(write_transfer);
    if (read_transfer)
        libusb_free_transfer(read_transfer);

    if (retval != ERROR_OK)
        mpsse_purge(ctx);

    return retval;
}
