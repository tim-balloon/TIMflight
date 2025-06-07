/* 
 * cryo_tauhk.c: interface for cryo housekkeping via TauHK
 * 
 * This software  is copyright 
 *  (C) University of Pennsylvania, Philadelphia 2025
 *
 * This file is part of mcp, as used for the Terahertz Intensity Mapper (TIM).
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
 * Created on: June 3, 2025 by Shubh Agrawal
 */

#include <math.h>
#include <stdio.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <sys/socket.h> // socket stuff
#include <unistd.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "cryo_tauhk.h"
#include "blast.h"

extern int16_t InCharge;

HKDataOne hk_data_one = {0};
HKDataTwenty hk_data_twenty = {0};
HKDataEighty hk_data_eighty = {0};

/**
 * @brief 
 * 
 */
int _sockfd_create(struct sockaddr_in *server_addr, int port) {
    int sockfd;
    while (1) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            blast_err("socket creation failed in cryo for port %d, retval %d, %s",
                      port, sockfd, strerror(errno));
            sleep(1); // wait before retrying
            continue;
        }
        blast_info("UDP socket created successfully for port %d", port);
        break;
    }

    memset(server_addr, 0, sizeof(struct sockaddr_in));
    server_addr->sin_family = AF_INET;
    server_addr->sin_addr.s_addr = INADDR_ANY;
    server_addr->sin_port = htons(port);

    while (1) {
        int binderr = bind(sockfd, (struct sockaddr *) server_addr, sizeof(*server_addr));
        if (binderr < 0) {
            blast_err("bind failed, retval %d, %s", binderr, strerror(errno));
            sleep(1); // wait before retrying
            continue;
        }
        break; // exit loop if bind is successful
    }
    blast_info("UDP socket created and bound to port %d", port);
    return sockfd;
}


/**
 * @brief
 */
void udp_receive_cryo_hk_1Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(&server_addr, CRYO_HK_1HZ_PORT);
    char buffer[UDP_MAX_SIZE];

    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            blast_err("recvfrom failed in cryo udp_receive_cryo_hk_1Hz");
            continue; // retry on error
        }
        if (n < sizeof(HKDataOne)) {
            blast_err("Received packet too small: %zd bytes", n);
            continue; // retry on error
        }
        // Process the received data
        blast_info("Received cryo HK 1Hz data of size %zd bytes", n);
        memcpy(&hk_data_one, buffer, sizeof(HKDataOne));
    }
}

/**
 * @brief
 */
void udp_receive_cryo_hk_20Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(&server_addr, CRYO_HK_20HZ_PORT);
    char buffer[UDP_MAX_SIZE];

    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            blast_err("recvfrom failed in cryo udp_receive_cryo_hk_20Hz");
            continue; // retry on error
        }
        if (n < sizeof(HKDataTwenty)) {
            blast_err("Received packet too small: %zd bytes", n);
            continue; // retry on error
        }
        // Process the received data
        blast_info("Received cryo HK 20Hz data of size %zd bytes", n);
        memcpy(&hk_data_twenty, buffer, sizeof(HKDataTwenty));
    }
}

/**
 * @brief
 */
void udp_receive_cryo_hk_80Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(&server_addr, CRYO_HK_80HZ_PORT);
    char buffer[UDP_MAX_SIZE];

    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            blast_err("recvfrom failed in cryo udp_receive_cryo_hk_80Hz");
            continue; // retry on error
        }
        if (n < sizeof(HKDataEighty)) {
            blast_err("Received packet too small: %zd bytes", n);
            continue; // retry on error
        }
        // Process the received data
        blast_info("Received cryo HK 80Hz data of size %zd bytes", n);
        memcpy(&hk_data_eighty, buffer, sizeof(HKDataEighty));
    }
}


/**
 * @brief
 */
void set_channels_cryo_hk_1Hz(void) {
    static int first_time = 1;
    static channel_t *diode_4he_film_enabled_chop_Addr;
    static channel_t *diode_4he_film_enabled_dc_Addr;
    static channel_t *diode_4he_hsw_enabled_chop_Addr;
    static channel_t *diode_4he_hsw_enabled_dc_Addr;
    static channel_t *diode_4he_pump_enabled_chop_Addr;
    static channel_t *diode_4he_pump_enabled_dc_Addr;
    static channel_t *diode_4k_filt_enabled_chop_Addr;
    static channel_t *diode_4k_filt_enabled_dc_Addr;
    static channel_t *diode_4k_plate_enabled_chop_Addr;
    static channel_t *diode_4k_plate_enabled_dc_Addr;
    static channel_t *diode_4k_shield_enabled_chop_Addr;
    static channel_t *diode_4k_shield_enabled_dc_Addr;
    static channel_t *diode_4k_wall_top_enabled_chop_Addr;
    static channel_t *diode_4k_wall_top_enabled_dc_Addr;
    static channel_t *diode_ic_hsw_enabled_chop_Addr;
    static channel_t *diode_ic_hsw_enabled_dc_Addr;
    static channel_t *diode_ic_pump_enabled_chop_Addr;
    static channel_t *diode_ic_pump_enabled_dc_Addr;
    static channel_t *diode_lw_div_enabled_chop_Addr;
    static channel_t *diode_lw_div_enabled_dc_Addr;
    static channel_t *diode_lw_grat_enabled_chop_Addr;
    static channel_t *diode_lw_grat_enabled_dc_Addr;
    static channel_t *diode_lw_mag_enabled_chop_Addr;
    static channel_t *diode_lw_mag_enabled_dc_Addr;
    static channel_t *diode_spare_enabled_chop_Addr;
    static channel_t *diode_spare_enabled_dc_Addr;
    static channel_t *diode_sw_div_enabled_chop_Addr;
    static channel_t *diode_sw_div_enabled_dc_Addr;
    static channel_t *diode_sw_grat_enabled_chop_Addr;
    static channel_t *diode_sw_grat_enabled_dc_Addr;
    static channel_t *diode_sw_mag_enabled_chop_Addr;
    static channel_t *diode_sw_mag_enabled_dc_Addr;
    static channel_t *diode_uc_hsw_enabled_chop_Addr;
    static channel_t *diode_uc_hsw_enabled_dc_Addr;
    static channel_t *diode_uc_pump_enabled_chop_Addr;
    static channel_t *diode_uc_pump_enabled_dc_Addr;
    static channel_t *diode_vcs1_filt_enabled_chop_Addr;
    static channel_t *diode_vcs1_filt_enabled_dc_Addr;
    static channel_t *diode_vcs1_hex_enabled_chop_Addr;
    static channel_t *diode_vcs1_hex_enabled_dc_Addr;
    static channel_t *diode_vcs1_top_enabled_chop_Addr;
    static channel_t *diode_vcs1_top_enabled_dc_Addr;
    static channel_t *diode_vcs2_filt_enabled_chop_Addr;
    static channel_t *diode_vcs2_filt_enabled_dc_Addr;
    static channel_t *diode_vcs2_hex_enabled_chop_Addr;
    static channel_t *diode_vcs2_hex_enabled_dc_Addr;
    static channel_t *diode_vcs2_top_enabled_chop_Addr;
    static channel_t *diode_vcs2_top_enabled_dc_Addr;
    static channel_t *global_cryoheater_enable_Addr;
    static channel_t *htr_4he_hsw_dac_Addr;
    static channel_t *htr_4he_hsw_overcurrent_monitor_Addr;
    static channel_t *htr_4he_hsw_pwm_Addr;
    static channel_t *htr_4he_hsw_volts_Addr;
    static channel_t *htr_4he_pmp_dac_Addr;
    static channel_t *htr_4he_pmp_overcurrent_monitor_Addr;
    static channel_t *htr_4he_pmp_pwm_Addr;
    static channel_t *htr_4he_pmp_volts_Addr;
    static channel_t *htr_ic_hsw_dac_Addr;
    static channel_t *htr_ic_hsw_overcurrent_monitor_Addr;
    static channel_t *htr_ic_hsw_pwm_Addr;
    static channel_t *htr_ic_hsw_volts_Addr;
    static channel_t *htr_ic_pump_dac_Addr;
    static channel_t *htr_ic_pump_overcurrent_monitor_Addr;
    static channel_t *htr_ic_pump_pwm_Addr;
    static channel_t *htr_ic_pump_volts_Addr;
    static channel_t *htr_lw_fpa_dac_Addr;
    static channel_t *htr_lw_fpa_overcurrent_monitor_Addr;
    static channel_t *htr_lw_fpa_pwm_Addr;
    static channel_t *htr_lw_fpa_volts_Addr;
    static channel_t *htr_sw_fpa_dac_Addr;
    static channel_t *htr_sw_fpa_overcurrent_monitor_Addr;
    static channel_t *htr_sw_fpa_pwm_Addr;
    static channel_t *htr_sw_fpa_volts_Addr;
    static channel_t *htr_uc_hsw_dac_Addr;
    static channel_t *htr_uc_hsw_overcurrent_monitor_Addr;
    static channel_t *htr_uc_hsw_pwm_Addr;
    static channel_t *htr_uc_hsw_volts_Addr;
    static channel_t *htr_uc_pump_dac_Addr;
    static channel_t *htr_uc_pump_overcurrent_monitor_Addr;
    static channel_t *htr_uc_pump_pwm_Addr;
    static channel_t *htr_uc_pump_volts_Addr;
    static channel_t *htr_vcs1_dac_Addr;
    static channel_t *htr_vcs1_overcurrent_monitor_Addr;
    static channel_t *htr_vcs1_pwm_Addr;
    static channel_t *htr_vcs1_volts_Addr;
    static channel_t *htr_vcs2_dac_Addr;
    static channel_t *htr_vcs2_overcurrent_monitor_Addr;
    static channel_t *htr_vcs2_pwm_Addr;
    static channel_t *htr_vcs2_volts_Addr;
    static channel_t *rtd_1_2_logdac_Addr;
    static channel_t *rtd_1_3_logdac_Addr;
    static channel_t *rtd_1_4_logdac_Addr;
    static channel_t *rtd_1_5_logdac_Addr;
    static channel_t *rtd_1_6_logdac_Addr;
    static channel_t *rtd_1_7_logdac_Addr;
    static channel_t *rtd_4he_head_logdac_Addr;
    static channel_t *rtd_ic_head_logdac_Addr;
    static channel_t *rtd_lw_fpu_1k_logdac_Addr;
    static channel_t *rtd_lw_fpu_250_logdac_Addr;
    static channel_t *rtd_lw_fpu_350_logdac_Addr;
    static channel_t *rtd_strap_inter_logdac_Addr;
    static channel_t *rtd_sw_fpu_1k_logdac_Addr;
    static channel_t *rtd_sw_fpu_250_logdac_Addr;
    static channel_t *rtd_sw_fpu_350_logdac_Addr;
    static channel_t *rtd_uc_head_logdac_Addr;
    if (InCharge) {
        if (first_time) {
            diode_4he_film_enabled_chop_Addr = channels_find_by_name("diode_4he_film_enabled_chop");
            diode_4he_film_enabled_dc_Addr = channels_find_by_name("diode_4he_film_enabled_dc");
            diode_4he_hsw_enabled_chop_Addr = channels_find_by_name("diode_4he_hsw_enabled_chop");
            diode_4he_hsw_enabled_dc_Addr = channels_find_by_name("diode_4he_hsw_enabled_dc");
            diode_4he_pump_enabled_chop_Addr = channels_find_by_name("diode_4he_pump_enabled_chop");
            diode_4he_pump_enabled_dc_Addr = channels_find_by_name("diode_4he_pump_enabled_dc");
            diode_4k_filt_enabled_chop_Addr = channels_find_by_name("diode_4k_filt_enabled_chop");
            diode_4k_filt_enabled_dc_Addr = channels_find_by_name("diode_4k_filt_enabled_dc");
            diode_4k_plate_enabled_chop_Addr = channels_find_by_name("diode_4k_plate_enabled_chop");
            diode_4k_plate_enabled_dc_Addr = channels_find_by_name("diode_4k_plate_enabled_dc");
            diode_4k_shield_enabled_chop_Addr = channels_find_by_name("diode_4k_shield_enabled_chop");
            diode_4k_shield_enabled_dc_Addr = channels_find_by_name("diode_4k_shield_enabled_dc");
            diode_4k_wall_top_enabled_chop_Addr = channels_find_by_name("diode_4k_wall_top_enabled_chop");
            diode_4k_wall_top_enabled_dc_Addr = channels_find_by_name("diode_4k_wall_top_enabled_dc");
            diode_ic_hsw_enabled_chop_Addr = channels_find_by_name("diode_ic_hsw_enabled_chop");
            diode_ic_hsw_enabled_dc_Addr = channels_find_by_name("diode_ic_hsw_enabled_dc");
            diode_ic_pump_enabled_chop_Addr = channels_find_by_name("diode_ic_pump_enabled_chop");
            diode_ic_pump_enabled_dc_Addr = channels_find_by_name("diode_ic_pump_enabled_dc");
            diode_lw_div_enabled_chop_Addr = channels_find_by_name("diode_lw_div_enabled_chop");
            diode_lw_div_enabled_dc_Addr = channels_find_by_name("diode_lw_div_enabled_dc");
            diode_lw_grat_enabled_chop_Addr = channels_find_by_name("diode_lw_grat_enabled_chop");
            diode_lw_grat_enabled_dc_Addr = channels_find_by_name("diode_lw_grat_enabled_dc");
            diode_lw_mag_enabled_chop_Addr = channels_find_by_name("diode_lw_mag_enabled_chop");
            diode_lw_mag_enabled_dc_Addr = channels_find_by_name("diode_lw_mag_enabled_dc");
            diode_spare_enabled_chop_Addr = channels_find_by_name("diode_spare_enabled_chop");
            diode_spare_enabled_dc_Addr = channels_find_by_name("diode_spare_enabled_dc");
            diode_sw_div_enabled_chop_Addr = channels_find_by_name("diode_sw_div_enabled_chop");
            diode_sw_div_enabled_dc_Addr = channels_find_by_name("diode_sw_div_enabled_dc");
            diode_sw_grat_enabled_chop_Addr = channels_find_by_name("diode_sw_grat_enabled_chop");
            diode_sw_grat_enabled_dc_Addr = channels_find_by_name("diode_sw_grat_enabled_dc");
            diode_sw_mag_enabled_chop_Addr = channels_find_by_name("diode_sw_mag_enabled_chop");
            diode_sw_mag_enabled_dc_Addr = channels_find_by_name("diode_sw_mag_enabled_dc");
            diode_uc_hsw_enabled_chop_Addr = channels_find_by_name("diode_uc_hsw_enabled_chop");
            diode_uc_hsw_enabled_dc_Addr = channels_find_by_name("diode_uc_hsw_enabled_dc");
            diode_uc_pump_enabled_chop_Addr = channels_find_by_name("diode_uc_pump_enabled_chop");
            diode_uc_pump_enabled_dc_Addr = channels_find_by_name("diode_uc_pump_enabled_dc");
            diode_vcs1_filt_enabled_chop_Addr = channels_find_by_name("diode_vcs1_filt_enabled_chop");
            diode_vcs1_filt_enabled_dc_Addr = channels_find_by_name("diode_vcs1_filt_enabled_dc");
            diode_vcs1_hex_enabled_chop_Addr = channels_find_by_name("diode_vcs1_hex_enabled_chop");
            diode_vcs1_hex_enabled_dc_Addr = channels_find_by_name("diode_vcs1_hex_enabled_dc");
            diode_vcs1_top_enabled_chop_Addr = channels_find_by_name("diode_vcs1_top_enabled_chop");
            diode_vcs1_top_enabled_dc_Addr = channels_find_by_name("diode_vcs1_top_enabled_dc");
            diode_vcs2_filt_enabled_chop_Addr = channels_find_by_name("diode_vcs2_filt_enabled_chop");
            diode_vcs2_filt_enabled_dc_Addr = channels_find_by_name("diode_vcs2_filt_enabled_dc");
            diode_vcs2_hex_enabled_chop_Addr = channels_find_by_name("diode_vcs2_hex_enabled_chop");
            diode_vcs2_hex_enabled_dc_Addr = channels_find_by_name("diode_vcs2_hex_enabled_dc");
            diode_vcs2_top_enabled_chop_Addr = channels_find_by_name("diode_vcs2_top_enabled_chop");
            diode_vcs2_top_enabled_dc_Addr = channels_find_by_name("diode_vcs2_top_enabled_dc");
            global_cryoheater_enable_Addr = channels_find_by_name("global_cryoheater_enable");
            htr_4he_hsw_dac_Addr = channels_find_by_name("htr_4he_hsw_dac");
            htr_4he_hsw_overcurrent_monitor_Addr = channels_find_by_name("htr_4he_hsw_overcurrent_monitor");
            htr_4he_hsw_pwm_Addr = channels_find_by_name("htr_4he_hsw_pwm");
            htr_4he_hsw_volts_Addr = channels_find_by_name("htr_4he_hsw_volts");
            htr_4he_pmp_dac_Addr = channels_find_by_name("htr_4he_pmp_dac");
            htr_4he_pmp_overcurrent_monitor_Addr = channels_find_by_name("htr_4he_pmp_overcurrent_monitor");
            htr_4he_pmp_pwm_Addr = channels_find_by_name("htr_4he_pmp_pwm");
            htr_4he_pmp_volts_Addr = channels_find_by_name("htr_4he_pmp_volts");
            htr_ic_hsw_dac_Addr = channels_find_by_name("htr_ic_hsw_dac");
            htr_ic_hsw_overcurrent_monitor_Addr = channels_find_by_name("htr_ic_hsw_overcurrent_monitor");
            htr_ic_hsw_pwm_Addr = channels_find_by_name("htr_ic_hsw_pwm");
            htr_ic_hsw_volts_Addr = channels_find_by_name("htr_ic_hsw_volts");
            htr_ic_pump_dac_Addr = channels_find_by_name("htr_ic_pump_dac");
            htr_ic_pump_overcurrent_monitor_Addr = channels_find_by_name("htr_ic_pump_overcurrent_monitor");
            htr_ic_pump_pwm_Addr = channels_find_by_name("htr_ic_pump_pwm");
            htr_ic_pump_volts_Addr = channels_find_by_name("htr_ic_pump_volts");
            htr_lw_fpa_dac_Addr = channels_find_by_name("htr_lw_fpa_dac");
            htr_lw_fpa_overcurrent_monitor_Addr = channels_find_by_name("htr_lw_fpa_overcurrent_monitor");
            htr_lw_fpa_pwm_Addr = channels_find_by_name("htr_lw_fpa_pwm");
            htr_lw_fpa_volts_Addr = channels_find_by_name("htr_lw_fpa_volts");
            htr_sw_fpa_dac_Addr = channels_find_by_name("htr_sw_fpa_dac");
            htr_sw_fpa_overcurrent_monitor_Addr = channels_find_by_name("htr_sw_fpa_overcurrent_monitor");
            htr_sw_fpa_pwm_Addr = channels_find_by_name("htr_sw_fpa_pwm");
            htr_sw_fpa_volts_Addr = channels_find_by_name("htr_sw_fpa_volts");
            htr_uc_hsw_dac_Addr = channels_find_by_name("htr_uc_hsw_dac");
            htr_uc_hsw_overcurrent_monitor_Addr = channels_find_by_name("htr_uc_hsw_overcurrent_monitor");
            htr_uc_hsw_pwm_Addr = channels_find_by_name("htr_uc_hsw_pwm");
            htr_uc_hsw_volts_Addr = channels_find_by_name("htr_uc_hsw_volts");
            htr_uc_pump_dac_Addr = channels_find_by_name("htr_uc_pump_dac");
            htr_uc_pump_overcurrent_monitor_Addr = channels_find_by_name("htr_uc_pump_overcurrent_monitor");
            htr_uc_pump_pwm_Addr = channels_find_by_name("htr_uc_pump_pwm");
            htr_uc_pump_volts_Addr = channels_find_by_name("htr_uc_pump_volts");
            htr_vcs1_dac_Addr = channels_find_by_name("htr_vcs1_dac");
            htr_vcs1_overcurrent_monitor_Addr = channels_find_by_name("htr_vcs1_overcurrent_monitor");
            htr_vcs1_pwm_Addr = channels_find_by_name("htr_vcs1_pwm");
            htr_vcs1_volts_Addr = channels_find_by_name("htr_vcs1_volts");
            htr_vcs2_dac_Addr = channels_find_by_name("htr_vcs2_dac");
            htr_vcs2_overcurrent_monitor_Addr = channels_find_by_name("htr_vcs2_overcurrent_monitor");
            htr_vcs2_pwm_Addr = channels_find_by_name("htr_vcs2_pwm");
            htr_vcs2_volts_Addr = channels_find_by_name("htr_vcs2_volts");
            rtd_1_2_logdac_Addr = channels_find_by_name("rtd_1_2_logdac");
            rtd_1_3_logdac_Addr = channels_find_by_name("rtd_1_3_logdac");
            rtd_1_4_logdac_Addr = channels_find_by_name("rtd_1_4_logdac");
            rtd_1_5_logdac_Addr = channels_find_by_name("rtd_1_5_logdac");
            rtd_1_6_logdac_Addr = channels_find_by_name("rtd_1_6_logdac");
            rtd_1_7_logdac_Addr = channels_find_by_name("rtd_1_7_logdac");
            rtd_4he_head_logdac_Addr = channels_find_by_name("rtd_4he_head_logdac");
            rtd_ic_head_logdac_Addr = channels_find_by_name("rtd_ic_head_logdac");
            rtd_lw_fpu_1k_logdac_Addr = channels_find_by_name("rtd_lw_fpu_1k_logdac");
            rtd_lw_fpu_250_logdac_Addr = channels_find_by_name("rtd_lw_fpu_250_logdac");
            rtd_lw_fpu_350_logdac_Addr = channels_find_by_name("rtd_lw_fpu_350_logdac");
            rtd_strap_inter_logdac_Addr = channels_find_by_name("rtd_strap_inter_logdac");
            rtd_sw_fpu_1k_logdac_Addr = channels_find_by_name("rtd_sw_fpu_1k_logdac");
            rtd_sw_fpu_250_logdac_Addr = channels_find_by_name("rtd_sw_fpu_250_logdac");
            rtd_sw_fpu_350_logdac_Addr = channels_find_by_name("rtd_sw_fpu_350_logdac");
            rtd_uc_head_logdac_Addr = channels_find_by_name("rtd_uc_head_logdac");
            first_time = 0;
        }
        SET_SCALED_VALUE(diode_4he_film_enabled_chop_Addr, hk_data_one.diode_4he_film_enabled_chop);
        SET_SCALED_VALUE(diode_4he_film_enabled_dc_Addr, hk_data_one.diode_4he_film_enabled_dc);
        SET_SCALED_VALUE(diode_4he_hsw_enabled_chop_Addr, hk_data_one.diode_4he_hsw_enabled_chop);
        SET_SCALED_VALUE(diode_4he_hsw_enabled_dc_Addr, hk_data_one.diode_4he_hsw_enabled_dc);
        SET_SCALED_VALUE(diode_4he_pump_enabled_chop_Addr, hk_data_one.diode_4he_pump_enabled_chop);
        SET_SCALED_VALUE(diode_4he_pump_enabled_dc_Addr, hk_data_one.diode_4he_pump_enabled_dc);
        SET_SCALED_VALUE(diode_4k_filt_enabled_chop_Addr, hk_data_one.diode_4k_filt_enabled_chop);
        SET_SCALED_VALUE(diode_4k_filt_enabled_dc_Addr, hk_data_one.diode_4k_filt_enabled_dc);
        SET_SCALED_VALUE(diode_4k_plate_enabled_chop_Addr, hk_data_one.diode_4k_plate_enabled_chop);
        SET_SCALED_VALUE(diode_4k_plate_enabled_dc_Addr, hk_data_one.diode_4k_plate_enabled_dc);
        SET_SCALED_VALUE(diode_4k_shield_enabled_chop_Addr, hk_data_one.diode_4k_shield_enabled_chop);
        SET_SCALED_VALUE(diode_4k_shield_enabled_dc_Addr, hk_data_one.diode_4k_shield_enabled_dc);
        SET_SCALED_VALUE(diode_4k_wall_top_enabled_chop_Addr, hk_data_one.diode_4k_wall_top_enabled_chop);
        SET_SCALED_VALUE(diode_4k_wall_top_enabled_dc_Addr, hk_data_one.diode_4k_wall_top_enabled_dc);
        SET_SCALED_VALUE(diode_ic_hsw_enabled_chop_Addr, hk_data_one.diode_ic_hsw_enabled_chop);
        SET_SCALED_VALUE(diode_ic_hsw_enabled_dc_Addr, hk_data_one.diode_ic_hsw_enabled_dc);
        SET_SCALED_VALUE(diode_ic_pump_enabled_chop_Addr, hk_data_one.diode_ic_pump_enabled_chop);
        SET_SCALED_VALUE(diode_ic_pump_enabled_dc_Addr, hk_data_one.diode_ic_pump_enabled_dc);
        SET_SCALED_VALUE(diode_lw_div_enabled_chop_Addr, hk_data_one.diode_lw_div_enabled_chop);
        SET_SCALED_VALUE(diode_lw_div_enabled_dc_Addr, hk_data_one.diode_lw_div_enabled_dc);
        SET_SCALED_VALUE(diode_lw_grat_enabled_chop_Addr, hk_data_one.diode_lw_grat_enabled_chop);
        SET_SCALED_VALUE(diode_lw_grat_enabled_dc_Addr, hk_data_one.diode_lw_grat_enabled_dc);
        SET_SCALED_VALUE(diode_lw_mag_enabled_chop_Addr, hk_data_one.diode_lw_mag_enabled_chop);
        SET_SCALED_VALUE(diode_lw_mag_enabled_dc_Addr, hk_data_one.diode_lw_mag_enabled_dc);
        SET_SCALED_VALUE(diode_spare_enabled_chop_Addr, hk_data_one.diode_spare_enabled_chop);
        SET_SCALED_VALUE(diode_spare_enabled_dc_Addr, hk_data_one.diode_spare_enabled_dc);
        SET_SCALED_VALUE(diode_sw_div_enabled_chop_Addr, hk_data_one.diode_sw_div_enabled_chop);
        SET_SCALED_VALUE(diode_sw_div_enabled_dc_Addr, hk_data_one.diode_sw_div_enabled_dc);
        SET_SCALED_VALUE(diode_sw_grat_enabled_chop_Addr, hk_data_one.diode_sw_grat_enabled_chop);
        SET_SCALED_VALUE(diode_sw_grat_enabled_dc_Addr, hk_data_one.diode_sw_grat_enabled_dc);
        SET_SCALED_VALUE(diode_sw_mag_enabled_chop_Addr, hk_data_one.diode_sw_mag_enabled_chop);
        SET_SCALED_VALUE(diode_sw_mag_enabled_dc_Addr, hk_data_one.diode_sw_mag_enabled_dc);
        SET_SCALED_VALUE(diode_uc_hsw_enabled_chop_Addr, hk_data_one.diode_uc_hsw_enabled_chop);
        SET_SCALED_VALUE(diode_uc_hsw_enabled_dc_Addr, hk_data_one.diode_uc_hsw_enabled_dc);
        SET_SCALED_VALUE(diode_uc_pump_enabled_chop_Addr, hk_data_one.diode_uc_pump_enabled_chop);
        SET_SCALED_VALUE(diode_uc_pump_enabled_dc_Addr, hk_data_one.diode_uc_pump_enabled_dc);
        SET_SCALED_VALUE(diode_vcs1_filt_enabled_chop_Addr, hk_data_one.diode_vcs1_filt_enabled_chop);
        SET_SCALED_VALUE(diode_vcs1_filt_enabled_dc_Addr, hk_data_one.diode_vcs1_filt_enabled_dc);
        SET_SCALED_VALUE(diode_vcs1_hex_enabled_chop_Addr, hk_data_one.diode_vcs1_hex_enabled_chop);
        SET_SCALED_VALUE(diode_vcs1_hex_enabled_dc_Addr, hk_data_one.diode_vcs1_hex_enabled_dc);
        SET_SCALED_VALUE(diode_vcs1_top_enabled_chop_Addr, hk_data_one.diode_vcs1_top_enabled_chop);
        SET_SCALED_VALUE(diode_vcs1_top_enabled_dc_Addr, hk_data_one.diode_vcs1_top_enabled_dc);
        SET_SCALED_VALUE(diode_vcs2_filt_enabled_chop_Addr, hk_data_one.diode_vcs2_filt_enabled_chop);
        SET_SCALED_VALUE(diode_vcs2_filt_enabled_dc_Addr, hk_data_one.diode_vcs2_filt_enabled_dc);
        SET_SCALED_VALUE(diode_vcs2_hex_enabled_chop_Addr, hk_data_one.diode_vcs2_hex_enabled_chop);
        SET_SCALED_VALUE(diode_vcs2_hex_enabled_dc_Addr, hk_data_one.diode_vcs2_hex_enabled_dc);
        SET_SCALED_VALUE(diode_vcs2_top_enabled_chop_Addr, hk_data_one.diode_vcs2_top_enabled_chop);
        SET_SCALED_VALUE(diode_vcs2_top_enabled_dc_Addr, hk_data_one.diode_vcs2_top_enabled_dc);
        SET_SCALED_VALUE(global_cryoheater_enable_Addr, hk_data_one.global_cryoheater_enable);
        SET_SCALED_VALUE(htr_4he_hsw_dac_Addr, hk_data_one.htr_4he_hsw_dac);
        SET_SCALED_VALUE(htr_4he_hsw_overcurrent_monitor_Addr, hk_data_one.htr_4he_hsw_overcurrent_monitor);
        SET_SCALED_VALUE(htr_4he_hsw_pwm_Addr, hk_data_one.htr_4he_hsw_pwm);
        SET_SCALED_VALUE(htr_4he_hsw_volts_Addr, hk_data_one.htr_4he_hsw_volts);
        SET_SCALED_VALUE(htr_4he_pmp_dac_Addr, hk_data_one.htr_4he_pmp_dac);
        SET_SCALED_VALUE(htr_4he_pmp_overcurrent_monitor_Addr, hk_data_one.htr_4he_pmp_overcurrent_monitor);
        SET_SCALED_VALUE(htr_4he_pmp_pwm_Addr, hk_data_one.htr_4he_pmp_pwm);
        SET_SCALED_VALUE(htr_4he_pmp_volts_Addr, hk_data_one.htr_4he_pmp_volts);
        SET_SCALED_VALUE(htr_ic_hsw_dac_Addr, hk_data_one.htr_ic_hsw_dac);
        SET_SCALED_VALUE(htr_ic_hsw_overcurrent_monitor_Addr, hk_data_one.htr_ic_hsw_overcurrent_monitor);
        SET_SCALED_VALUE(htr_ic_hsw_pwm_Addr, hk_data_one.htr_ic_hsw_pwm);
        SET_SCALED_VALUE(htr_ic_hsw_volts_Addr, hk_data_one.htr_ic_hsw_volts);
        SET_SCALED_VALUE(htr_ic_pump_dac_Addr, hk_data_one.htr_ic_pump_dac);
        SET_SCALED_VALUE(htr_ic_pump_overcurrent_monitor_Addr, hk_data_one.htr_ic_pump_overcurrent_monitor);
        SET_SCALED_VALUE(htr_ic_pump_pwm_Addr, hk_data_one.htr_ic_pump_pwm);
        SET_SCALED_VALUE(htr_ic_pump_volts_Addr, hk_data_one.htr_ic_pump_volts);
        SET_SCALED_VALUE(htr_lw_fpa_dac_Addr, hk_data_one.htr_lw_fpa_dac);
        SET_SCALED_VALUE(htr_lw_fpa_overcurrent_monitor_Addr, hk_data_one.htr_lw_fpa_overcurrent_monitor);
        SET_SCALED_VALUE(htr_lw_fpa_pwm_Addr, hk_data_one.htr_lw_fpa_pwm);
        SET_SCALED_VALUE(htr_lw_fpa_volts_Addr, hk_data_one.htr_lw_fpa_volts);
        SET_SCALED_VALUE(htr_sw_fpa_dac_Addr, hk_data_one.htr_sw_fpa_dac);
        SET_SCALED_VALUE(htr_sw_fpa_overcurrent_monitor_Addr, hk_data_one.htr_sw_fpa_overcurrent_monitor);
        SET_SCALED_VALUE(htr_sw_fpa_pwm_Addr, hk_data_one.htr_sw_fpa_pwm);
        SET_SCALED_VALUE(htr_sw_fpa_volts_Addr, hk_data_one.htr_sw_fpa_volts);
        SET_SCALED_VALUE(htr_uc_hsw_dac_Addr, hk_data_one.htr_uc_hsw_dac);
        SET_SCALED_VALUE(htr_uc_hsw_overcurrent_monitor_Addr, hk_data_one.htr_uc_hsw_overcurrent_monitor);
        SET_SCALED_VALUE(htr_uc_hsw_pwm_Addr, hk_data_one.htr_uc_hsw_pwm);
        SET_SCALED_VALUE(htr_uc_hsw_volts_Addr, hk_data_one.htr_uc_hsw_volts);
        SET_SCALED_VALUE(htr_uc_pump_dac_Addr, hk_data_one.htr_uc_pump_dac);
        SET_SCALED_VALUE(htr_uc_pump_overcurrent_monitor_Addr, hk_data_one.htr_uc_pump_overcurrent_monitor);
        SET_SCALED_VALUE(htr_uc_pump_pwm_Addr, hk_data_one.htr_uc_pump_pwm);
        SET_SCALED_VALUE(htr_uc_pump_volts_Addr, hk_data_one.htr_uc_pump_volts);
        SET_SCALED_VALUE(htr_vcs1_dac_Addr, hk_data_one.htr_vcs1_dac);
        SET_SCALED_VALUE(htr_vcs1_overcurrent_monitor_Addr, hk_data_one.htr_vcs1_overcurrent_monitor);
        SET_SCALED_VALUE(htr_vcs1_pwm_Addr, hk_data_one.htr_vcs1_pwm);
        SET_SCALED_VALUE(htr_vcs1_volts_Addr, hk_data_one.htr_vcs1_volts);
        SET_SCALED_VALUE(htr_vcs2_dac_Addr, hk_data_one.htr_vcs2_dac);
        SET_SCALED_VALUE(htr_vcs2_overcurrent_monitor_Addr, hk_data_one.htr_vcs2_overcurrent_monitor);
        SET_SCALED_VALUE(htr_vcs2_pwm_Addr, hk_data_one.htr_vcs2_pwm);
        SET_SCALED_VALUE(htr_vcs2_volts_Addr, hk_data_one.htr_vcs2_volts);
        SET_SCALED_VALUE(rtd_1_2_logdac_Addr, hk_data_one.rtd_1_2_logdac);
        SET_SCALED_VALUE(rtd_1_3_logdac_Addr, hk_data_one.rtd_1_3_logdac);
        SET_SCALED_VALUE(rtd_1_4_logdac_Addr, hk_data_one.rtd_1_4_logdac);
        SET_SCALED_VALUE(rtd_1_5_logdac_Addr, hk_data_one.rtd_1_5_logdac);
        SET_SCALED_VALUE(rtd_1_6_logdac_Addr, hk_data_one.rtd_1_6_logdac);
        SET_SCALED_VALUE(rtd_1_7_logdac_Addr, hk_data_one.rtd_1_7_logdac);
        SET_SCALED_VALUE(rtd_4he_head_logdac_Addr, hk_data_one.rtd_4he_head_logdac);
        SET_SCALED_VALUE(rtd_ic_head_logdac_Addr, hk_data_one.rtd_ic_head_logdac);
        SET_SCALED_VALUE(rtd_lw_fpu_1k_logdac_Addr, hk_data_one.rtd_lw_fpu_1k_logdac);
        SET_SCALED_VALUE(rtd_lw_fpu_250_logdac_Addr, hk_data_one.rtd_lw_fpu_250_logdac);
        SET_SCALED_VALUE(rtd_lw_fpu_350_logdac_Addr, hk_data_one.rtd_lw_fpu_350_logdac);
        SET_SCALED_VALUE(rtd_strap_inter_logdac_Addr, hk_data_one.rtd_strap_inter_logdac);
        SET_SCALED_VALUE(rtd_sw_fpu_1k_logdac_Addr, hk_data_one.rtd_sw_fpu_1k_logdac);
        SET_SCALED_VALUE(rtd_sw_fpu_250_logdac_Addr, hk_data_one.rtd_sw_fpu_250_logdac);
        SET_SCALED_VALUE(rtd_sw_fpu_350_logdac_Addr, hk_data_one.rtd_sw_fpu_350_logdac);
        SET_SCALED_VALUE(rtd_uc_head_logdac_Addr, hk_data_one.rtd_uc_head_logdac);
    }
}


/**
 * @brief
 */
void set_channels_cryo_hk_20Hz(void) {
    static int first_time = 1;
    static channel_t *diode_4he_film_code_Addr;
    static channel_t *diode_4he_film_voltage_Addr;
    static channel_t *diode_4he_hsw_code_Addr;
    static channel_t *diode_4he_hsw_voltage_Addr;
    static channel_t *diode_4he_pump_code_Addr;
    static channel_t *diode_4he_pump_voltage_Addr;
    static channel_t *diode_4k_filt_code_Addr;
    static channel_t *diode_4k_filt_voltage_Addr;
    static channel_t *diode_4k_plate_code_Addr;
    static channel_t *diode_4k_plate_voltage_Addr;
    static channel_t *diode_4k_shield_code_Addr;
    static channel_t *diode_4k_shield_voltage_Addr;
    static channel_t *diode_4k_wall_top_code_Addr;
    static channel_t *diode_4k_wall_top_voltage_Addr;
    static channel_t *diode_ic_hsw_code_Addr;
    static channel_t *diode_ic_hsw_voltage_Addr;
    static channel_t *diode_ic_pump_code_Addr;
    static channel_t *diode_ic_pump_voltage_Addr;
    static channel_t *diode_lw_div_code_Addr;
    static channel_t *diode_lw_div_voltage_Addr;
    static channel_t *diode_lw_grat_code_Addr;
    static channel_t *diode_lw_grat_voltage_Addr;
    static channel_t *diode_lw_mag_code_Addr;
    static channel_t *diode_lw_mag_voltage_Addr;
    static channel_t *diode_spare_code_Addr;
    static channel_t *diode_spare_voltage_Addr;
    static channel_t *diode_sw_div_code_Addr;
    static channel_t *diode_sw_div_voltage_Addr;
    static channel_t *diode_sw_grat_code_Addr;
    static channel_t *diode_sw_grat_voltage_Addr;
    static channel_t *diode_sw_mag_code_Addr;
    static channel_t *diode_sw_mag_voltage_Addr;
    static channel_t *diode_uc_hsw_code_Addr;
    static channel_t *diode_uc_hsw_voltage_Addr;
    static channel_t *diode_uc_pump_code_Addr;
    static channel_t *diode_uc_pump_voltage_Addr;
    static channel_t *diode_vcs1_filt_code_Addr;
    static channel_t *diode_vcs1_filt_voltage_Addr;
    static channel_t *diode_vcs1_hex_code_Addr;
    static channel_t *diode_vcs1_hex_voltage_Addr;
    static channel_t *diode_vcs1_top_code_Addr;
    static channel_t *diode_vcs1_top_voltage_Addr;
    static channel_t *diode_vcs2_filt_code_Addr;
    static channel_t *diode_vcs2_filt_voltage_Addr;
    static channel_t *diode_vcs2_hex_code_Addr;
    static channel_t *diode_vcs2_hex_voltage_Addr;
    static channel_t *diode_vcs2_top_code_Addr;
    static channel_t *diode_vcs2_top_voltage_Addr;
    static channel_t *htr_4he_hsw_code_Addr;
    static channel_t *htr_4he_hsw_current_Addr;
    static channel_t *htr_4he_hsw_overcurrent_Addr;
    static channel_t *htr_4he_hsw_resistance_Addr;
    static channel_t *htr_4he_hsw_voltage_Addr;
    static channel_t *htr_4he_pmp_code_Addr;
    static channel_t *htr_4he_pmp_current_Addr;
    static channel_t *htr_4he_pmp_overcurrent_Addr;
    static channel_t *htr_4he_pmp_resistance_Addr;
    static channel_t *htr_4he_pmp_voltage_Addr;
    static channel_t *htr_ic_hsw_code_Addr;
    static channel_t *htr_ic_hsw_current_Addr;
    static channel_t *htr_ic_hsw_overcurrent_Addr;
    static channel_t *htr_ic_hsw_resistance_Addr;
    static channel_t *htr_ic_hsw_voltage_Addr;
    static channel_t *htr_ic_pump_code_Addr;
    static channel_t *htr_ic_pump_current_Addr;
    static channel_t *htr_ic_pump_overcurrent_Addr;
    static channel_t *htr_ic_pump_resistance_Addr;
    static channel_t *htr_ic_pump_voltage_Addr;
    static channel_t *htr_lw_fpa_code_Addr;
    static channel_t *htr_lw_fpa_current_Addr;
    static channel_t *htr_lw_fpa_overcurrent_Addr;
    static channel_t *htr_lw_fpa_resistance_Addr;
    static channel_t *htr_lw_fpa_voltage_Addr;
    static channel_t *htr_sw_fpa_code_Addr;
    static channel_t *htr_sw_fpa_current_Addr;
    static channel_t *htr_sw_fpa_overcurrent_Addr;
    static channel_t *htr_sw_fpa_resistance_Addr;
    static channel_t *htr_sw_fpa_voltage_Addr;
    static channel_t *htr_uc_hsw_code_Addr;
    static channel_t *htr_uc_hsw_current_Addr;
    static channel_t *htr_uc_hsw_overcurrent_Addr;
    static channel_t *htr_uc_hsw_resistance_Addr;
    static channel_t *htr_uc_hsw_voltage_Addr;
    static channel_t *htr_uc_pump_code_Addr;
    static channel_t *htr_uc_pump_current_Addr;
    static channel_t *htr_uc_pump_overcurrent_Addr;
    static channel_t *htr_uc_pump_resistance_Addr;
    static channel_t *htr_uc_pump_voltage_Addr;
    static channel_t *htr_vcs1_code_Addr;
    static channel_t *htr_vcs1_current_Addr;
    static channel_t *htr_vcs1_overcurrent_Addr;
    static channel_t *htr_vcs1_resistance_Addr;
    static channel_t *htr_vcs1_voltage_Addr;
    static channel_t *htr_vcs2_code_Addr;
    static channel_t *htr_vcs2_current_Addr;
    static channel_t *htr_vcs2_overcurrent_Addr;
    static channel_t *htr_vcs2_resistance_Addr;
    static channel_t *htr_vcs2_voltage_Addr;
    if (InCharge) {
        if (first_time) {
            diode_4he_film_code_Addr = channels_find_by_name("diode_4he_film_code");
            diode_4he_film_voltage_Addr = channels_find_by_name("diode_4he_film_voltage");
            diode_4he_hsw_code_Addr = channels_find_by_name("diode_4he_hsw_code");
            diode_4he_hsw_voltage_Addr = channels_find_by_name("diode_4he_hsw_voltage");
            diode_4he_pump_code_Addr = channels_find_by_name("diode_4he_pump_code");
            diode_4he_pump_voltage_Addr = channels_find_by_name("diode_4he_pump_voltage");
            diode_4k_filt_code_Addr = channels_find_by_name("diode_4k_filt_code");
            diode_4k_filt_voltage_Addr = channels_find_by_name("diode_4k_filt_voltage");
            diode_4k_plate_code_Addr = channels_find_by_name("diode_4k_plate_code");
            diode_4k_plate_voltage_Addr = channels_find_by_name("diode_4k_plate_voltage");
            diode_4k_shield_code_Addr = channels_find_by_name("diode_4k_shield_code");
            diode_4k_shield_voltage_Addr = channels_find_by_name("diode_4k_shield_voltage");
            diode_4k_wall_top_code_Addr = channels_find_by_name("diode_4k_wall_top_code");
            diode_4k_wall_top_voltage_Addr = channels_find_by_name("diode_4k_wall_top_voltage");
            diode_ic_hsw_code_Addr = channels_find_by_name("diode_ic_hsw_code");
            diode_ic_hsw_voltage_Addr = channels_find_by_name("diode_ic_hsw_voltage");
            diode_ic_pump_code_Addr = channels_find_by_name("diode_ic_pump_code");
            diode_ic_pump_voltage_Addr = channels_find_by_name("diode_ic_pump_voltage");
            diode_lw_div_code_Addr = channels_find_by_name("diode_lw_div_code");
            diode_lw_div_voltage_Addr = channels_find_by_name("diode_lw_div_voltage");
            diode_lw_grat_code_Addr = channels_find_by_name("diode_lw_grat_code");
            diode_lw_grat_voltage_Addr = channels_find_by_name("diode_lw_grat_voltage");
            diode_lw_mag_code_Addr = channels_find_by_name("diode_lw_mag_code");
            diode_lw_mag_voltage_Addr = channels_find_by_name("diode_lw_mag_voltage");
            diode_spare_code_Addr = channels_find_by_name("diode_spare_code");
            diode_spare_voltage_Addr = channels_find_by_name("diode_spare_voltage");
            diode_sw_div_code_Addr = channels_find_by_name("diode_sw_div_code");
            diode_sw_div_voltage_Addr = channels_find_by_name("diode_sw_div_voltage");
            diode_sw_grat_code_Addr = channels_find_by_name("diode_sw_grat_code");
            diode_sw_grat_voltage_Addr = channels_find_by_name("diode_sw_grat_voltage");
            diode_sw_mag_code_Addr = channels_find_by_name("diode_sw_mag_code");
            diode_sw_mag_voltage_Addr = channels_find_by_name("diode_sw_mag_voltage");
            diode_uc_hsw_code_Addr = channels_find_by_name("diode_uc_hsw_code");
            diode_uc_hsw_voltage_Addr = channels_find_by_name("diode_uc_hsw_voltage");
            diode_uc_pump_code_Addr = channels_find_by_name("diode_uc_pump_code");
            diode_uc_pump_voltage_Addr = channels_find_by_name("diode_uc_pump_voltage");
            diode_vcs1_filt_code_Addr = channels_find_by_name("diode_vcs1_filt_code");
            diode_vcs1_filt_voltage_Addr = channels_find_by_name("diode_vcs1_filt_voltage");
            diode_vcs1_hex_code_Addr = channels_find_by_name("diode_vcs1_hex_code");
            diode_vcs1_hex_voltage_Addr = channels_find_by_name("diode_vcs1_hex_voltage");
            diode_vcs1_top_code_Addr = channels_find_by_name("diode_vcs1_top_code");
            diode_vcs1_top_voltage_Addr = channels_find_by_name("diode_vcs1_top_voltage");
            diode_vcs2_filt_code_Addr = channels_find_by_name("diode_vcs2_filt_code");
            diode_vcs2_filt_voltage_Addr = channels_find_by_name("diode_vcs2_filt_voltage");
            diode_vcs2_hex_code_Addr = channels_find_by_name("diode_vcs2_hex_code");
            diode_vcs2_hex_voltage_Addr = channels_find_by_name("diode_vcs2_hex_voltage");
            diode_vcs2_top_code_Addr = channels_find_by_name("diode_vcs2_top_code");
            diode_vcs2_top_voltage_Addr = channels_find_by_name("diode_vcs2_top_voltage");
            htr_4he_hsw_code_Addr = channels_find_by_name("htr_4he_hsw_code");
            htr_4he_hsw_current_Addr = channels_find_by_name("htr_4he_hsw_current");
            htr_4he_hsw_overcurrent_Addr = channels_find_by_name("htr_4he_hsw_overcurrent");
            htr_4he_hsw_resistance_Addr = channels_find_by_name("htr_4he_hsw_resistance");
            htr_4he_hsw_voltage_Addr = channels_find_by_name("htr_4he_hsw_voltage");
            htr_4he_pmp_code_Addr = channels_find_by_name("htr_4he_pmp_code");
            htr_4he_pmp_current_Addr = channels_find_by_name("htr_4he_pmp_current");
            htr_4he_pmp_overcurrent_Addr = channels_find_by_name("htr_4he_pmp_overcurrent");
            htr_4he_pmp_resistance_Addr = channels_find_by_name("htr_4he_pmp_resistance");
            htr_4he_pmp_voltage_Addr = channels_find_by_name("htr_4he_pmp_voltage");
            htr_ic_hsw_code_Addr = channels_find_by_name("htr_ic_hsw_code");
            htr_ic_hsw_current_Addr = channels_find_by_name("htr_ic_hsw_current");
            htr_ic_hsw_overcurrent_Addr = channels_find_by_name("htr_ic_hsw_overcurrent");
            htr_ic_hsw_resistance_Addr = channels_find_by_name("htr_ic_hsw_resistance");
            htr_ic_hsw_voltage_Addr = channels_find_by_name("htr_ic_hsw_voltage");
            htr_ic_pump_code_Addr = channels_find_by_name("htr_ic_pump_code");
            htr_ic_pump_current_Addr = channels_find_by_name("htr_ic_pump_current");
            htr_ic_pump_overcurrent_Addr = channels_find_by_name("htr_ic_pump_overcurrent");
            htr_ic_pump_resistance_Addr = channels_find_by_name("htr_ic_pump_resistance");
            htr_ic_pump_voltage_Addr = channels_find_by_name("htr_ic_pump_voltage");
            htr_lw_fpa_code_Addr = channels_find_by_name("htr_lw_fpa_code");
            htr_lw_fpa_current_Addr = channels_find_by_name("htr_lw_fpa_current");
            htr_lw_fpa_overcurrent_Addr = channels_find_by_name("htr_lw_fpa_overcurrent");
            htr_lw_fpa_resistance_Addr = channels_find_by_name("htr_lw_fpa_resistance");
            htr_lw_fpa_voltage_Addr = channels_find_by_name("htr_lw_fpa_voltage");
            htr_sw_fpa_code_Addr = channels_find_by_name("htr_sw_fpa_code");
            htr_sw_fpa_current_Addr = channels_find_by_name("htr_sw_fpa_current");
            htr_sw_fpa_overcurrent_Addr = channels_find_by_name("htr_sw_fpa_overcurrent");
            htr_sw_fpa_resistance_Addr = channels_find_by_name("htr_sw_fpa_resistance");
            htr_sw_fpa_voltage_Addr = channels_find_by_name("htr_sw_fpa_voltage");
            htr_uc_hsw_code_Addr = channels_find_by_name("htr_uc_hsw_code");
            htr_uc_hsw_current_Addr = channels_find_by_name("htr_uc_hsw_current");
            htr_uc_hsw_overcurrent_Addr = channels_find_by_name("htr_uc_hsw_overcurrent");
            htr_uc_hsw_resistance_Addr = channels_find_by_name("htr_uc_hsw_resistance");
            htr_uc_hsw_voltage_Addr = channels_find_by_name("htr_uc_hsw_voltage");
            htr_uc_pump_code_Addr = channels_find_by_name("htr_uc_pump_code");
            htr_uc_pump_current_Addr = channels_find_by_name("htr_uc_pump_current");
            htr_uc_pump_overcurrent_Addr = channels_find_by_name("htr_uc_pump_overcurrent");
            htr_uc_pump_resistance_Addr = channels_find_by_name("htr_uc_pump_resistance");
            htr_uc_pump_voltage_Addr = channels_find_by_name("htr_uc_pump_voltage");
            htr_vcs1_code_Addr = channels_find_by_name("htr_vcs1_code");
            htr_vcs1_current_Addr = channels_find_by_name("htr_vcs1_current");
            htr_vcs1_overcurrent_Addr = channels_find_by_name("htr_vcs1_overcurrent");
            htr_vcs1_resistance_Addr = channels_find_by_name("htr_vcs1_resistance");
            htr_vcs1_voltage_Addr = channels_find_by_name("htr_vcs1_voltage");
            htr_vcs2_code_Addr = channels_find_by_name("htr_vcs2_code");
            htr_vcs2_current_Addr = channels_find_by_name("htr_vcs2_current");
            htr_vcs2_overcurrent_Addr = channels_find_by_name("htr_vcs2_overcurrent");
            htr_vcs2_resistance_Addr = channels_find_by_name("htr_vcs2_resistance");
            htr_vcs2_voltage_Addr = channels_find_by_name("htr_vcs2_voltage");
            first_time = 0;
        }
        SET_SCALED_VALUE(diode_4he_film_code_Addr, hk_data_twenty.diode_4he_film_code);
        SET_SCALED_VALUE(diode_4he_film_voltage_Addr, hk_data_twenty.diode_4he_film_voltage);
        SET_SCALED_VALUE(diode_4he_hsw_code_Addr, hk_data_twenty.diode_4he_hsw_code);
        SET_SCALED_VALUE(diode_4he_hsw_voltage_Addr, hk_data_twenty.diode_4he_hsw_voltage);
        SET_SCALED_VALUE(diode_4he_pump_code_Addr, hk_data_twenty.diode_4he_pump_code);
        SET_SCALED_VALUE(diode_4he_pump_voltage_Addr, hk_data_twenty.diode_4he_pump_voltage);
        SET_SCALED_VALUE(diode_4k_filt_code_Addr, hk_data_twenty.diode_4k_filt_code);
        SET_SCALED_VALUE(diode_4k_filt_voltage_Addr, hk_data_twenty.diode_4k_filt_voltage);
        SET_SCALED_VALUE(diode_4k_plate_code_Addr, hk_data_twenty.diode_4k_plate_code);
        SET_SCALED_VALUE(diode_4k_plate_voltage_Addr, hk_data_twenty.diode_4k_plate_voltage);
        SET_SCALED_VALUE(diode_4k_shield_code_Addr, hk_data_twenty.diode_4k_shield_code);
        SET_SCALED_VALUE(diode_4k_shield_voltage_Addr, hk_data_twenty.diode_4k_shield_voltage);
        SET_SCALED_VALUE(diode_4k_wall_top_code_Addr, hk_data_twenty.diode_4k_wall_top_code);
        SET_SCALED_VALUE(diode_4k_wall_top_voltage_Addr, hk_data_twenty.diode_4k_wall_top_voltage);
        SET_SCALED_VALUE(diode_ic_hsw_code_Addr, hk_data_twenty.diode_ic_hsw_code);
        SET_SCALED_VALUE(diode_ic_hsw_voltage_Addr, hk_data_twenty.diode_ic_hsw_voltage);
        SET_SCALED_VALUE(diode_ic_pump_code_Addr, hk_data_twenty.diode_ic_pump_code);
        SET_SCALED_VALUE(diode_ic_pump_voltage_Addr, hk_data_twenty.diode_ic_pump_voltage);
        SET_SCALED_VALUE(diode_lw_div_code_Addr, hk_data_twenty.diode_lw_div_code);
        SET_SCALED_VALUE(diode_lw_div_voltage_Addr, hk_data_twenty.diode_lw_div_voltage);
        SET_SCALED_VALUE(diode_lw_grat_code_Addr, hk_data_twenty.diode_lw_grat_code);
        SET_SCALED_VALUE(diode_lw_grat_voltage_Addr, hk_data_twenty.diode_lw_grat_voltage);
        SET_SCALED_VALUE(diode_lw_mag_code_Addr, hk_data_twenty.diode_lw_mag_code);
        SET_SCALED_VALUE(diode_lw_mag_voltage_Addr, hk_data_twenty.diode_lw_mag_voltage);
        SET_SCALED_VALUE(diode_spare_code_Addr, hk_data_twenty.diode_spare_code);
        SET_SCALED_VALUE(diode_spare_voltage_Addr, hk_data_twenty.diode_spare_voltage);
        SET_SCALED_VALUE(diode_sw_div_code_Addr, hk_data_twenty.diode_sw_div_code);
        SET_SCALED_VALUE(diode_sw_div_voltage_Addr, hk_data_twenty.diode_sw_div_voltage);
        SET_SCALED_VALUE(diode_sw_grat_code_Addr, hk_data_twenty.diode_sw_grat_code);
        SET_SCALED_VALUE(diode_sw_grat_voltage_Addr, hk_data_twenty.diode_sw_grat_voltage);
        SET_SCALED_VALUE(diode_sw_mag_code_Addr, hk_data_twenty.diode_sw_mag_code);
        SET_SCALED_VALUE(diode_sw_mag_voltage_Addr, hk_data_twenty.diode_sw_mag_voltage);
        SET_SCALED_VALUE(diode_uc_hsw_code_Addr, hk_data_twenty.diode_uc_hsw_code);
        SET_SCALED_VALUE(diode_uc_hsw_voltage_Addr, hk_data_twenty.diode_uc_hsw_voltage);
        SET_SCALED_VALUE(diode_uc_pump_code_Addr, hk_data_twenty.diode_uc_pump_code);
        SET_SCALED_VALUE(diode_uc_pump_voltage_Addr, hk_data_twenty.diode_uc_pump_voltage);
        SET_SCALED_VALUE(diode_vcs1_filt_code_Addr, hk_data_twenty.diode_vcs1_filt_code);
        SET_SCALED_VALUE(diode_vcs1_filt_voltage_Addr, hk_data_twenty.diode_vcs1_filt_voltage);
        SET_SCALED_VALUE(diode_vcs1_hex_code_Addr, hk_data_twenty.diode_vcs1_hex_code);
        SET_SCALED_VALUE(diode_vcs1_hex_voltage_Addr, hk_data_twenty.diode_vcs1_hex_voltage);
        SET_SCALED_VALUE(diode_vcs1_top_code_Addr, hk_data_twenty.diode_vcs1_top_code);
        SET_SCALED_VALUE(diode_vcs1_top_voltage_Addr, hk_data_twenty.diode_vcs1_top_voltage);
        SET_SCALED_VALUE(diode_vcs2_filt_code_Addr, hk_data_twenty.diode_vcs2_filt_code);
        SET_SCALED_VALUE(diode_vcs2_filt_voltage_Addr, hk_data_twenty.diode_vcs2_filt_voltage);
        SET_SCALED_VALUE(diode_vcs2_hex_code_Addr, hk_data_twenty.diode_vcs2_hex_code);
        SET_SCALED_VALUE(diode_vcs2_hex_voltage_Addr, hk_data_twenty.diode_vcs2_hex_voltage);
        SET_SCALED_VALUE(diode_vcs2_top_code_Addr, hk_data_twenty.diode_vcs2_top_code);
        SET_SCALED_VALUE(diode_vcs2_top_voltage_Addr, hk_data_twenty.diode_vcs2_top_voltage);
        SET_SCALED_VALUE(htr_4he_hsw_code_Addr, hk_data_twenty.htr_4he_hsw_code);
        SET_SCALED_VALUE(htr_4he_hsw_current_Addr, hk_data_twenty.htr_4he_hsw_current);
        SET_SCALED_VALUE(htr_4he_hsw_overcurrent_Addr, hk_data_twenty.htr_4he_hsw_overcurrent);
        SET_SCALED_VALUE(htr_4he_hsw_resistance_Addr, hk_data_twenty.htr_4he_hsw_resistance);
        SET_SCALED_VALUE(htr_4he_hsw_voltage_Addr, hk_data_twenty.htr_4he_hsw_voltage);
        SET_SCALED_VALUE(htr_4he_pmp_code_Addr, hk_data_twenty.htr_4he_pmp_code);
        SET_SCALED_VALUE(htr_4he_pmp_current_Addr, hk_data_twenty.htr_4he_pmp_current);
        SET_SCALED_VALUE(htr_4he_pmp_overcurrent_Addr, hk_data_twenty.htr_4he_pmp_overcurrent);
        SET_SCALED_VALUE(htr_4he_pmp_resistance_Addr, hk_data_twenty.htr_4he_pmp_resistance);
        SET_SCALED_VALUE(htr_4he_pmp_voltage_Addr, hk_data_twenty.htr_4he_pmp_voltage);
        SET_SCALED_VALUE(htr_ic_hsw_code_Addr, hk_data_twenty.htr_ic_hsw_code);
        SET_SCALED_VALUE(htr_ic_hsw_current_Addr, hk_data_twenty.htr_ic_hsw_current);
        SET_SCALED_VALUE(htr_ic_hsw_overcurrent_Addr, hk_data_twenty.htr_ic_hsw_overcurrent);
        SET_SCALED_VALUE(htr_ic_hsw_resistance_Addr, hk_data_twenty.htr_ic_hsw_resistance);
        SET_SCALED_VALUE(htr_ic_hsw_voltage_Addr, hk_data_twenty.htr_ic_hsw_voltage);
        SET_SCALED_VALUE(htr_ic_pump_code_Addr, hk_data_twenty.htr_ic_pump_code);
        SET_SCALED_VALUE(htr_ic_pump_current_Addr, hk_data_twenty.htr_ic_pump_current);
        SET_SCALED_VALUE(htr_ic_pump_overcurrent_Addr, hk_data_twenty.htr_ic_pump_overcurrent);
        SET_SCALED_VALUE(htr_ic_pump_resistance_Addr, hk_data_twenty.htr_ic_pump_resistance);
        SET_SCALED_VALUE(htr_ic_pump_voltage_Addr, hk_data_twenty.htr_ic_pump_voltage);
        SET_SCALED_VALUE(htr_lw_fpa_code_Addr, hk_data_twenty.htr_lw_fpa_code);
        SET_SCALED_VALUE(htr_lw_fpa_current_Addr, hk_data_twenty.htr_lw_fpa_current);
        SET_SCALED_VALUE(htr_lw_fpa_overcurrent_Addr, hk_data_twenty.htr_lw_fpa_overcurrent);
        SET_SCALED_VALUE(htr_lw_fpa_resistance_Addr, hk_data_twenty.htr_lw_fpa_resistance);
        SET_SCALED_VALUE(htr_lw_fpa_voltage_Addr, hk_data_twenty.htr_lw_fpa_voltage);
        SET_SCALED_VALUE(htr_sw_fpa_code_Addr, hk_data_twenty.htr_sw_fpa_code);
        SET_SCALED_VALUE(htr_sw_fpa_current_Addr, hk_data_twenty.htr_sw_fpa_current);
        SET_SCALED_VALUE(htr_sw_fpa_overcurrent_Addr, hk_data_twenty.htr_sw_fpa_overcurrent);
        SET_SCALED_VALUE(htr_sw_fpa_resistance_Addr, hk_data_twenty.htr_sw_fpa_resistance);
        SET_SCALED_VALUE(htr_sw_fpa_voltage_Addr, hk_data_twenty.htr_sw_fpa_voltage);
        SET_SCALED_VALUE(htr_uc_hsw_code_Addr, hk_data_twenty.htr_uc_hsw_code);
        SET_SCALED_VALUE(htr_uc_hsw_current_Addr, hk_data_twenty.htr_uc_hsw_current);
        SET_SCALED_VALUE(htr_uc_hsw_overcurrent_Addr, hk_data_twenty.htr_uc_hsw_overcurrent);
        SET_SCALED_VALUE(htr_uc_hsw_resistance_Addr, hk_data_twenty.htr_uc_hsw_resistance);
        SET_SCALED_VALUE(htr_uc_hsw_voltage_Addr, hk_data_twenty.htr_uc_hsw_voltage);
        SET_SCALED_VALUE(htr_uc_pump_code_Addr, hk_data_twenty.htr_uc_pump_code);
        SET_SCALED_VALUE(htr_uc_pump_current_Addr, hk_data_twenty.htr_uc_pump_current);
        SET_SCALED_VALUE(htr_uc_pump_overcurrent_Addr, hk_data_twenty.htr_uc_pump_overcurrent);
        SET_SCALED_VALUE(htr_uc_pump_resistance_Addr, hk_data_twenty.htr_uc_pump_resistance);
        SET_SCALED_VALUE(htr_uc_pump_voltage_Addr, hk_data_twenty.htr_uc_pump_voltage);
        SET_SCALED_VALUE(htr_vcs1_code_Addr, hk_data_twenty.htr_vcs1_code);
        SET_SCALED_VALUE(htr_vcs1_current_Addr, hk_data_twenty.htr_vcs1_current);
        SET_SCALED_VALUE(htr_vcs1_overcurrent_Addr, hk_data_twenty.htr_vcs1_overcurrent);
        SET_SCALED_VALUE(htr_vcs1_resistance_Addr, hk_data_twenty.htr_vcs1_resistance);
        SET_SCALED_VALUE(htr_vcs1_voltage_Addr, hk_data_twenty.htr_vcs1_voltage);
        SET_SCALED_VALUE(htr_vcs2_code_Addr, hk_data_twenty.htr_vcs2_code);
        SET_SCALED_VALUE(htr_vcs2_current_Addr, hk_data_twenty.htr_vcs2_current);
        SET_SCALED_VALUE(htr_vcs2_overcurrent_Addr, hk_data_twenty.htr_vcs2_overcurrent);
        SET_SCALED_VALUE(htr_vcs2_resistance_Addr, hk_data_twenty.htr_vcs2_resistance);
        SET_SCALED_VALUE(htr_vcs2_voltage_Addr, hk_data_twenty.htr_vcs2_voltage);
    }
}


/**
 * @brief
 */
void set_channels_cryo_hk_80Hz(void) {
    static int first_time = 1;
    static channel_t *rtd_1_2_code_Addr;
    static channel_t *rtd_1_2_resistance_Addr;
    static channel_t *rtd_1_2_voltage_Addr;
    static channel_t *rtd_1_3_code_Addr;
    static channel_t *rtd_1_3_resistance_Addr;
    static channel_t *rtd_1_3_voltage_Addr;
    static channel_t *rtd_1_4_code_Addr;
    static channel_t *rtd_1_4_resistance_Addr;
    static channel_t *rtd_1_4_voltage_Addr;
    static channel_t *rtd_1_5_code_Addr;
    static channel_t *rtd_1_5_resistance_Addr;
    static channel_t *rtd_1_5_voltage_Addr;
    static channel_t *rtd_1_6_code_Addr;
    static channel_t *rtd_1_6_resistance_Addr;
    static channel_t *rtd_1_6_voltage_Addr;
    static channel_t *rtd_1_7_code_Addr;
    static channel_t *rtd_1_7_resistance_Addr;
    static channel_t *rtd_1_7_voltage_Addr;
    static channel_t *rtd_4he_head_code_Addr;
    static channel_t *rtd_4he_head_resistance_Addr;
    static channel_t *rtd_4he_head_voltage_Addr;
    static channel_t *rtd_ic_head_code_Addr;
    static channel_t *rtd_ic_head_resistance_Addr;
    static channel_t *rtd_ic_head_voltage_Addr;
    static channel_t *rtd_lw_fpu_1k_code_Addr;
    static channel_t *rtd_lw_fpu_1k_resistance_Addr;
    static channel_t *rtd_lw_fpu_1k_voltage_Addr;
    static channel_t *rtd_lw_fpu_250_code_Addr;
    static channel_t *rtd_lw_fpu_250_resistance_Addr;
    static channel_t *rtd_lw_fpu_250_voltage_Addr;
    static channel_t *rtd_lw_fpu_350_code_Addr;
    static channel_t *rtd_lw_fpu_350_resistance_Addr;
    static channel_t *rtd_lw_fpu_350_voltage_Addr;
    static channel_t *rtd_strap_inter_code_Addr;
    static channel_t *rtd_strap_inter_resistance_Addr;
    static channel_t *rtd_strap_inter_voltage_Addr;
    static channel_t *rtd_sw_fpu_1k_code_Addr;
    static channel_t *rtd_sw_fpu_1k_resistance_Addr;
    static channel_t *rtd_sw_fpu_1k_voltage_Addr;
    static channel_t *rtd_sw_fpu_250_code_Addr;
    static channel_t *rtd_sw_fpu_250_resistance_Addr;
    static channel_t *rtd_sw_fpu_250_voltage_Addr;
    static channel_t *rtd_sw_fpu_350_code_Addr;
    static channel_t *rtd_sw_fpu_350_resistance_Addr;
    static channel_t *rtd_sw_fpu_350_voltage_Addr;
    static channel_t *rtd_uc_head_code_Addr;
    static channel_t *rtd_uc_head_resistance_Addr;
    static channel_t *rtd_uc_head_voltage_Addr;
    static channel_t *time_mcu_Addr;
    if (InCharge) {
        if (first_time) {
            rtd_1_2_code_Addr = channels_find_by_name("rtd_1_2_code");
            rtd_1_2_resistance_Addr = channels_find_by_name("rtd_1_2_resistance");
            rtd_1_2_voltage_Addr = channels_find_by_name("rtd_1_2_voltage");
            rtd_1_3_code_Addr = channels_find_by_name("rtd_1_3_code");
            rtd_1_3_resistance_Addr = channels_find_by_name("rtd_1_3_resistance");
            rtd_1_3_voltage_Addr = channels_find_by_name("rtd_1_3_voltage");
            rtd_1_4_code_Addr = channels_find_by_name("rtd_1_4_code");
            rtd_1_4_resistance_Addr = channels_find_by_name("rtd_1_4_resistance");
            rtd_1_4_voltage_Addr = channels_find_by_name("rtd_1_4_voltage");
            rtd_1_5_code_Addr = channels_find_by_name("rtd_1_5_code");
            rtd_1_5_resistance_Addr = channels_find_by_name("rtd_1_5_resistance");
            rtd_1_5_voltage_Addr = channels_find_by_name("rtd_1_5_voltage");
            rtd_1_6_code_Addr = channels_find_by_name("rtd_1_6_code");
            rtd_1_6_resistance_Addr = channels_find_by_name("rtd_1_6_resistance");
            rtd_1_6_voltage_Addr = channels_find_by_name("rtd_1_6_voltage");
            rtd_1_7_code_Addr = channels_find_by_name("rtd_1_7_code");
            rtd_1_7_resistance_Addr = channels_find_by_name("rtd_1_7_resistance");
            rtd_1_7_voltage_Addr = channels_find_by_name("rtd_1_7_voltage");
            rtd_4he_head_code_Addr = channels_find_by_name("rtd_4he_head_code");
            rtd_4he_head_resistance_Addr = channels_find_by_name("rtd_4he_head_resistance");
            rtd_4he_head_voltage_Addr = channels_find_by_name("rtd_4he_head_voltage");
            rtd_ic_head_code_Addr = channels_find_by_name("rtd_ic_head_code");
            rtd_ic_head_resistance_Addr = channels_find_by_name("rtd_ic_head_resistance");
            rtd_ic_head_voltage_Addr = channels_find_by_name("rtd_ic_head_voltage");
            rtd_lw_fpu_1k_code_Addr = channels_find_by_name("rtd_lw_fpu_1k_code");
            rtd_lw_fpu_1k_resistance_Addr = channels_find_by_name("rtd_lw_fpu_1k_resistance");
            rtd_lw_fpu_1k_voltage_Addr = channels_find_by_name("rtd_lw_fpu_1k_voltage");
            rtd_lw_fpu_250_code_Addr = channels_find_by_name("rtd_lw_fpu_250_code");
            rtd_lw_fpu_250_resistance_Addr = channels_find_by_name("rtd_lw_fpu_250_resistance");
            rtd_lw_fpu_250_voltage_Addr = channels_find_by_name("rtd_lw_fpu_250_voltage");
            rtd_lw_fpu_350_code_Addr = channels_find_by_name("rtd_lw_fpu_350_code");
            rtd_lw_fpu_350_resistance_Addr = channels_find_by_name("rtd_lw_fpu_350_resistance");
            rtd_lw_fpu_350_voltage_Addr = channels_find_by_name("rtd_lw_fpu_350_voltage");
            rtd_strap_inter_code_Addr = channels_find_by_name("rtd_strap_inter_code");
            rtd_strap_inter_resistance_Addr = channels_find_by_name("rtd_strap_inter_resistance");
            rtd_strap_inter_voltage_Addr = channels_find_by_name("rtd_strap_inter_voltage");
            rtd_sw_fpu_1k_code_Addr = channels_find_by_name("rtd_sw_fpu_1k_code");
            rtd_sw_fpu_1k_resistance_Addr = channels_find_by_name("rtd_sw_fpu_1k_resistance");
            rtd_sw_fpu_1k_voltage_Addr = channels_find_by_name("rtd_sw_fpu_1k_voltage");
            rtd_sw_fpu_250_code_Addr = channels_find_by_name("rtd_sw_fpu_250_code");
            rtd_sw_fpu_250_resistance_Addr = channels_find_by_name("rtd_sw_fpu_250_resistance");
            rtd_sw_fpu_250_voltage_Addr = channels_find_by_name("rtd_sw_fpu_250_voltage");
            rtd_sw_fpu_350_code_Addr = channels_find_by_name("rtd_sw_fpu_350_code");
            rtd_sw_fpu_350_resistance_Addr = channels_find_by_name("rtd_sw_fpu_350_resistance");
            rtd_sw_fpu_350_voltage_Addr = channels_find_by_name("rtd_sw_fpu_350_voltage");
            rtd_uc_head_code_Addr = channels_find_by_name("rtd_uc_head_code");
            rtd_uc_head_resistance_Addr = channels_find_by_name("rtd_uc_head_resistance");
            rtd_uc_head_voltage_Addr = channels_find_by_name("rtd_uc_head_voltage");
            time_mcu_Addr = channels_find_by_name("time_mcu");
            first_time = 0;
        }
        SET_SCALED_VALUE(rtd_1_2_code_Addr, hk_data_eighty.rtd_1_2_code);
        SET_SCALED_VALUE(rtd_1_2_resistance_Addr, hk_data_eighty.rtd_1_2_resistance);
        SET_SCALED_VALUE(rtd_1_2_voltage_Addr, hk_data_eighty.rtd_1_2_voltage);
        SET_SCALED_VALUE(rtd_1_3_code_Addr, hk_data_eighty.rtd_1_3_code);
        SET_SCALED_VALUE(rtd_1_3_resistance_Addr, hk_data_eighty.rtd_1_3_resistance);
        SET_SCALED_VALUE(rtd_1_3_voltage_Addr, hk_data_eighty.rtd_1_3_voltage);
        SET_SCALED_VALUE(rtd_1_4_code_Addr, hk_data_eighty.rtd_1_4_code);
        SET_SCALED_VALUE(rtd_1_4_resistance_Addr, hk_data_eighty.rtd_1_4_resistance);
        SET_SCALED_VALUE(rtd_1_4_voltage_Addr, hk_data_eighty.rtd_1_4_voltage);
        SET_SCALED_VALUE(rtd_1_5_code_Addr, hk_data_eighty.rtd_1_5_code);
        SET_SCALED_VALUE(rtd_1_5_resistance_Addr, hk_data_eighty.rtd_1_5_resistance);
        SET_SCALED_VALUE(rtd_1_5_voltage_Addr, hk_data_eighty.rtd_1_5_voltage);
        SET_SCALED_VALUE(rtd_1_6_code_Addr, hk_data_eighty.rtd_1_6_code);
        SET_SCALED_VALUE(rtd_1_6_resistance_Addr, hk_data_eighty.rtd_1_6_resistance);
        SET_SCALED_VALUE(rtd_1_6_voltage_Addr, hk_data_eighty.rtd_1_6_voltage);
        SET_SCALED_VALUE(rtd_1_7_code_Addr, hk_data_eighty.rtd_1_7_code);
        SET_SCALED_VALUE(rtd_1_7_resistance_Addr, hk_data_eighty.rtd_1_7_resistance);
        SET_SCALED_VALUE(rtd_1_7_voltage_Addr, hk_data_eighty.rtd_1_7_voltage);
        SET_SCALED_VALUE(rtd_4he_head_code_Addr, hk_data_eighty.rtd_4he_head_code);
        SET_SCALED_VALUE(rtd_4he_head_resistance_Addr, hk_data_eighty.rtd_4he_head_resistance);
        SET_SCALED_VALUE(rtd_4he_head_voltage_Addr, hk_data_eighty.rtd_4he_head_voltage);
        SET_SCALED_VALUE(rtd_ic_head_code_Addr, hk_data_eighty.rtd_ic_head_code);
        SET_SCALED_VALUE(rtd_ic_head_resistance_Addr, hk_data_eighty.rtd_ic_head_resistance);
        SET_SCALED_VALUE(rtd_ic_head_voltage_Addr, hk_data_eighty.rtd_ic_head_voltage);
        SET_SCALED_VALUE(rtd_lw_fpu_1k_code_Addr, hk_data_eighty.rtd_lw_fpu_1k_code);
        SET_SCALED_VALUE(rtd_lw_fpu_1k_resistance_Addr, hk_data_eighty.rtd_lw_fpu_1k_resistance);
        SET_SCALED_VALUE(rtd_lw_fpu_1k_voltage_Addr, hk_data_eighty.rtd_lw_fpu_1k_voltage);
        SET_SCALED_VALUE(rtd_lw_fpu_250_code_Addr, hk_data_eighty.rtd_lw_fpu_250_code);
        SET_SCALED_VALUE(rtd_lw_fpu_250_resistance_Addr, hk_data_eighty.rtd_lw_fpu_250_resistance);
        SET_SCALED_VALUE(rtd_lw_fpu_250_voltage_Addr, hk_data_eighty.rtd_lw_fpu_250_voltage);
        SET_SCALED_VALUE(rtd_lw_fpu_350_code_Addr, hk_data_eighty.rtd_lw_fpu_350_code);
        SET_SCALED_VALUE(rtd_lw_fpu_350_resistance_Addr, hk_data_eighty.rtd_lw_fpu_350_resistance);
        SET_SCALED_VALUE(rtd_lw_fpu_350_voltage_Addr, hk_data_eighty.rtd_lw_fpu_350_voltage);
        SET_SCALED_VALUE(rtd_strap_inter_code_Addr, hk_data_eighty.rtd_strap_inter_code);
        SET_SCALED_VALUE(rtd_strap_inter_resistance_Addr, hk_data_eighty.rtd_strap_inter_resistance);
        SET_SCALED_VALUE(rtd_strap_inter_voltage_Addr, hk_data_eighty.rtd_strap_inter_voltage);
        SET_SCALED_VALUE(rtd_sw_fpu_1k_code_Addr, hk_data_eighty.rtd_sw_fpu_1k_code);
        SET_SCALED_VALUE(rtd_sw_fpu_1k_resistance_Addr, hk_data_eighty.rtd_sw_fpu_1k_resistance);
        SET_SCALED_VALUE(rtd_sw_fpu_1k_voltage_Addr, hk_data_eighty.rtd_sw_fpu_1k_voltage);
        SET_SCALED_VALUE(rtd_sw_fpu_250_code_Addr, hk_data_eighty.rtd_sw_fpu_250_code);
        SET_SCALED_VALUE(rtd_sw_fpu_250_resistance_Addr, hk_data_eighty.rtd_sw_fpu_250_resistance);
        SET_SCALED_VALUE(rtd_sw_fpu_250_voltage_Addr, hk_data_eighty.rtd_sw_fpu_250_voltage);
        SET_SCALED_VALUE(rtd_sw_fpu_350_code_Addr, hk_data_eighty.rtd_sw_fpu_350_code);
        SET_SCALED_VALUE(rtd_sw_fpu_350_resistance_Addr, hk_data_eighty.rtd_sw_fpu_350_resistance);
        SET_SCALED_VALUE(rtd_sw_fpu_350_voltage_Addr, hk_data_eighty.rtd_sw_fpu_350_voltage);
        SET_SCALED_VALUE(rtd_uc_head_code_Addr, hk_data_eighty.rtd_uc_head_code);
        SET_SCALED_VALUE(rtd_uc_head_resistance_Addr, hk_data_eighty.rtd_uc_head_resistance);
        SET_SCALED_VALUE(rtd_uc_head_voltage_Addr, hk_data_eighty.rtd_uc_head_voltage);
        SET_SCALED_VALUE(time_mcu_Addr, hk_data_eighty.time_mcu);
    }
}
