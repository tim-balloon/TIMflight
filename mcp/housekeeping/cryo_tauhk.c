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
int _sockfd_create(struct sockaddr_in server_addr, struct sockaddr_in client_addr) {
    char buffer[UDP_MAX_SIZE];
    int sockfd;
    while (1) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            blast_err("socket creation failed in cryo udp_receive_cryo_hk_1Hz");
            sleep(1); // wait before retrying
            continue;
        }
        break;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(CRYO_HK_1HZ_PORT);

    while (1) {
        if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            blast_err("bind failed");
            sleep(1); // wait before retrying
            continue;
        }
        break; // exit loop if bind is successful
    }
    blast_info("UDP socket created and bound to port %d", CRYO_HK_1HZ_PORT);
    return sockfd;
}


/**
 * @brief
 */
void udp_receive_cryo_hk_1Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(server_addr, client_addr);
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
        memcpy(&hk_data_one, buffer, sizeof(HKDataOne));
    }
}

/**
 * @brief
 */
void udp_receive_cryo_hk_20Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(server_addr, client_addr);
    char buffer[UDP_MAX_SIZE];

    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            blast_err("recvfrom failed in cryo udp_receive_cryo_hk_1Hz");
            continue; // retry on error
        }
        if (n < sizeof(HKDataTwenty)) {
            blast_err("Received packet too small: %zd bytes", n);
            continue; // retry on error
        }
        // Process the received data
        memcpy(&hk_data_twenty, buffer, sizeof(HKDataOne));
    }
}

/**
 * @brief
 */
void udp_receive_cryo_hk_80Hz(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int sockfd = _sockfd_create(server_addr, client_addr);
    char buffer[UDP_MAX_SIZE];

    while (1) {
        ssize_t n = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                (struct sockaddr *)&client_addr, &addr_len);
        if (n < 0) {
            blast_err("recvfrom failed in cryo udp_receive_cryo_hk_1Hz");
            continue; // retry on error
        }
        if (n < sizeof(HKDataEighty)) {
            blast_err("Received packet too small: %zd bytes", n);
            continue; // retry on error
        }
        // Process the received data
        memcpy(&hk_data_eighty, buffer, sizeof(HKDataEighty));
    }
}


/**
 * @brief
 */
void set_channels_cryo_hk_1Hz(void) {
    static int first_time = 1;
    static channel_t* therm_1_Addr;
    if (InCharge) {
        if (first_time) {
            therm_1_Addr = channels_find_by_name("htr_4he_pmp_dac");
            first_time = 0;
        }
        SET_SCALED_VALUE(therm_1_Addr, hk_data_one.htr_4he_pmp_dac);
    }
}

/**
 * @brief
 */
void set_channels_cryo_hk_20Hz(void) {
    static int first_time = 1;
    static channel_t* therm_1_Addr;
    if (InCharge) {
        if (first_time) {
            therm_1_Addr = channels_find_by_name("diode_4he_film_voltage");
            first_time = 0;
        }
        SET_SCALED_VALUE(therm_1_Addr, hk_data_twenty.diode_4he_film_voltage);
    }
}

/**
 * @brief
 */
void set_channels_cryo_hk_80Hz(void) {
    static int first_time = 1;
    static channel_t* therm_1_Addr;
    if (InCharge) {
        if (first_time) {
            therm_1_Addr = channels_find_by_name("rtd_lw_fpu_250_resistance");
            first_time = 0;
        }
        SET_SCALED_VALUE(therm_1_Addr, hk_data_eighty.rtd_lw_fpu_250_resistance);
    }
}
