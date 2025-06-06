/* 
 * cryo_tauhk.h: interface for cryo housekkeping via TauHK
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

#define CRYO_HK_1HZ_PORT 1212
#define CRYO_HK_20HZ_PORT 1213
#define CRYO_HK_80HZ_PORT 1214

#define UDP_MAX_SIZE 1024

typedef struct {
    int32_t diode_4he_film_enabled_chop;
    int32_t diode_4he_film_enabled_dc;
    int32_t diode_4he_hsw_enabled_chop;
    int32_t diode_4he_hsw_enabled_dc;
    int32_t diode_4he_pump_enabled_chop;
    int32_t diode_4he_pump_enabled_dc;
    int32_t diode_4k_filt_enabled_chop;
    int32_t diode_4k_filt_enabled_dc;
    int32_t diode_4k_plate_enabled_chop;
    int32_t diode_4k_plate_enabled_dc;
    int32_t diode_4k_shield_enabled_chop;
    int32_t diode_4k_shield_enabled_dc;
    int32_t diode_4k_wall_top_enabled_chop;
    int32_t diode_4k_wall_top_enabled_dc;
    int32_t diode_ic_hsw_enabled_chop;
    int32_t diode_ic_hsw_enabled_dc;
    int32_t diode_ic_pump_enabled_chop;
    int32_t diode_ic_pump_enabled_dc;
    int32_t diode_lw_div_enabled_chop;
    int32_t diode_lw_div_enabled_dc;
    int32_t diode_lw_grat_enabled_chop;
    int32_t diode_lw_grat_enabled_dc;
    int32_t diode_lw_mag_enabled_chop;
    int32_t diode_lw_mag_enabled_dc;
    int32_t diode_spare_enabled_chop;
    int32_t diode_spare_enabled_dc;
    int32_t diode_sw_div_enabled_chop;
    int32_t diode_sw_div_enabled_dc;
    int32_t diode_sw_grat_enabled_chop;
    int32_t diode_sw_grat_enabled_dc;
    int32_t diode_sw_mag_enabled_chop;
    int32_t diode_sw_mag_enabled_dc;
    int32_t diode_uc_hsw_enabled_chop;
    int32_t diode_uc_hsw_enabled_dc;
    int32_t diode_uc_pump_enabled_chop;
    int32_t diode_uc_pump_enabled_dc;
    int32_t diode_vcs1_filt_enabled_chop;
    int32_t diode_vcs1_filt_enabled_dc;
    int32_t diode_vcs1_hex_enabled_chop;
    int32_t diode_vcs1_hex_enabled_dc;
    int32_t diode_vcs1_top_enabled_chop;
    int32_t diode_vcs1_top_enabled_dc;
    int32_t diode_vcs2_filt_enabled_chop;
    int32_t diode_vcs2_filt_enabled_dc;
    int32_t diode_vcs2_hex_enabled_chop;
    int32_t diode_vcs2_hex_enabled_dc;
    int32_t diode_vcs2_top_enabled_chop;
    int32_t diode_vcs2_top_enabled_dc;
    int32_t global_cryoheater_enable;
    int32_t htr_4he_hsw_dac;
    int32_t htr_4he_hsw_overcurrent_monitor;
    int32_t htr_4he_hsw_pwm;
    float htr_4he_hsw_volts;
    int32_t htr_4he_pmp_dac;
    int32_t htr_4he_pmp_overcurrent_monitor;
    int32_t htr_4he_pmp_pwm;
    float htr_4he_pmp_volts;
    int32_t htr_ic_hsw_dac;
    int32_t htr_ic_hsw_overcurrent_monitor;
    int32_t htr_ic_hsw_pwm;
    float htr_ic_hsw_volts;
    int32_t htr_ic_pump_dac;
    int32_t htr_ic_pump_overcurrent_monitor;
    int32_t htr_ic_pump_pwm;
    float htr_ic_pump_volts;
    int32_t htr_lw_fpa_dac;
    int32_t htr_lw_fpa_overcurrent_monitor;
    int32_t htr_lw_fpa_pwm;
    float htr_lw_fpa_volts;
    int32_t htr_sw_fpa_dac;
    int32_t htr_sw_fpa_overcurrent_monitor;
    int32_t htr_sw_fpa_pwm;
    float htr_sw_fpa_volts;
    int32_t htr_uc_hsw_dac;
    int32_t htr_uc_hsw_overcurrent_monitor;
    int32_t htr_uc_hsw_pwm;
    float htr_uc_hsw_volts;
    int32_t htr_uc_pump_dac;
    int32_t htr_uc_pump_overcurrent_monitor;
    int32_t htr_uc_pump_pwm;
    float htr_uc_pump_volts;
    int32_t htr_vcs1_dac;
    int32_t htr_vcs1_overcurrent_monitor;
    int32_t htr_vcs1_pwm;
    float htr_vcs1_volts;
    int32_t htr_vcs2_dac;
    int32_t htr_vcs2_overcurrent_monitor;
    int32_t htr_vcs2_pwm;
    float htr_vcs2_volts;
    int32_t rtd_1_2_logdac;
    int32_t rtd_1_3_logdac;
    int32_t rtd_1_4_logdac;
    int32_t rtd_1_5_logdac;
    int32_t rtd_1_6_logdac;
    int32_t rtd_1_7_logdac;
    int32_t rtd_4he_head_logdac;
    int32_t rtd_ic_head_logdac;
    int32_t rtd_lw_fpu_1k_logdac;
    int32_t rtd_lw_fpu_250_logdac;
    int32_t rtd_lw_fpu_350_logdac;
    int32_t rtd_strap_inter_logdac;
    int32_t rtd_sw_fpu_1k_logdac;
    int32_t rtd_sw_fpu_250_logdac;
    int32_t rtd_sw_fpu_350_logdac;
    int32_t rtd_uc_head_logdac;
} HKDataOne;

typedef struct {
    int32_t diode_4he_film_code;
    float diode_4he_film_voltage;
    int32_t diode_4he_hsw_code;
    float diode_4he_hsw_voltage;
    int32_t diode_4he_pump_code;
    float diode_4he_pump_voltage;
    int32_t diode_4k_filt_code;
    float diode_4k_filt_voltage;
    int32_t diode_4k_plate_code;
    float diode_4k_plate_voltage;
    int32_t diode_4k_shield_code;
    float diode_4k_shield_voltage;
    int32_t diode_4k_wall_top_code;
    float diode_4k_wall_top_voltage;
    int32_t diode_ic_hsw_code;
    float diode_ic_hsw_voltage;
    int32_t diode_ic_pump_code;
    float diode_ic_pump_voltage;
    int32_t diode_lw_div_code;
    float diode_lw_div_voltage;
    int32_t diode_lw_grat_code;
    float diode_lw_grat_voltage;
    int32_t diode_lw_mag_code;
    float diode_lw_mag_voltage;
    int32_t diode_spare_code;
    float diode_spare_voltage;
    int32_t diode_sw_div_code;
    float diode_sw_div_voltage;
    int32_t diode_sw_grat_code;
    float diode_sw_grat_voltage;
    int32_t diode_sw_mag_code;
    float diode_sw_mag_voltage;
    int32_t diode_uc_hsw_code;
    float diode_uc_hsw_voltage;
    int32_t diode_uc_pump_code;
    float diode_uc_pump_voltage;
    int32_t diode_vcs1_filt_code;
    float diode_vcs1_filt_voltage;
    int32_t diode_vcs1_hex_code;
    float diode_vcs1_hex_voltage;
    int32_t diode_vcs1_top_code;
    float diode_vcs1_top_voltage;
    int32_t diode_vcs2_filt_code;
    float diode_vcs2_filt_voltage;
    int32_t diode_vcs2_hex_code;
    float diode_vcs2_hex_voltage;
    int32_t diode_vcs2_top_code;
    float diode_vcs2_top_voltage;
    int32_t htr_4he_hsw_code;
    float htr_4he_hsw_current;
    int32_t htr_4he_hsw_overcurrent;
    float htr_4he_hsw_resistance;
    float htr_4he_hsw_voltage;
    int32_t htr_4he_pmp_code;
    float htr_4he_pmp_current;
    int32_t htr_4he_pmp_overcurrent;
    float htr_4he_pmp_resistance;
    float htr_4he_pmp_voltage;
    int32_t htr_ic_hsw_code;
    float htr_ic_hsw_current;
    int32_t htr_ic_hsw_overcurrent;
    float htr_ic_hsw_resistance;
    float htr_ic_hsw_voltage;
    int32_t htr_ic_pump_code;
    float htr_ic_pump_current;
    int32_t htr_ic_pump_overcurrent;
    float htr_ic_pump_resistance;
    float htr_ic_pump_voltage;
    int32_t htr_lw_fpa_code;
    float htr_lw_fpa_current;
    int32_t htr_lw_fpa_overcurrent;
    float htr_lw_fpa_resistance;
    float htr_lw_fpa_voltage;
    int32_t htr_sw_fpa_code;
    float htr_sw_fpa_current;
    int32_t htr_sw_fpa_overcurrent;
    float htr_sw_fpa_resistance;
    float htr_sw_fpa_voltage;
    int32_t htr_uc_hsw_code;
    float htr_uc_hsw_current;
    int32_t htr_uc_hsw_overcurrent;
    float htr_uc_hsw_resistance;
    float htr_uc_hsw_voltage;
    int32_t htr_uc_pump_code;
    float htr_uc_pump_current;
    int32_t htr_uc_pump_overcurrent;
    float htr_uc_pump_resistance;
    float htr_uc_pump_voltage;
    int32_t htr_vcs1_code;
    float htr_vcs1_current;
    int32_t htr_vcs1_overcurrent;
    float htr_vcs1_resistance;
    float htr_vcs1_voltage;
    int32_t htr_vcs2_code;
    float htr_vcs2_current;
    int32_t htr_vcs2_overcurrent;
    float htr_vcs2_resistance;
    float htr_vcs2_voltage;
} HKDataTwenty;

typedef struct {
    int32_t rtd_1_2_code;
    float rtd_1_2_resistance;
    float rtd_1_2_voltage;
    int32_t rtd_1_3_code;
    float rtd_1_3_resistance;
    float rtd_1_3_voltage;
    int32_t rtd_1_4_code;
    float rtd_1_4_resistance;
    float rtd_1_4_voltage;
    int32_t rtd_1_5_code;
    float rtd_1_5_resistance;
    float rtd_1_5_voltage;
    int32_t rtd_1_6_code;
    float rtd_1_6_resistance;
    float rtd_1_6_voltage;
    int32_t rtd_1_7_code;
    float rtd_1_7_resistance;
    float rtd_1_7_voltage;
    int32_t rtd_4he_head_code;
    float rtd_4he_head_resistance;
    float rtd_4he_head_voltage;
    int32_t rtd_ic_head_code;
    float rtd_ic_head_resistance;
    float rtd_ic_head_voltage;
    int32_t rtd_lw_fpu_1k_code;
    float rtd_lw_fpu_1k_resistance;
    float rtd_lw_fpu_1k_voltage;
    int32_t rtd_lw_fpu_250_code;
    float rtd_lw_fpu_250_resistance;
    float rtd_lw_fpu_250_voltage;
    int32_t rtd_lw_fpu_350_code;
    float rtd_lw_fpu_350_resistance;
    float rtd_lw_fpu_350_voltage;
    int32_t rtd_strap_inter_code;
    float rtd_strap_inter_resistance;
    float rtd_strap_inter_voltage;
    int32_t rtd_sw_fpu_1k_code;
    float rtd_sw_fpu_1k_resistance;
    float rtd_sw_fpu_1k_voltage;
    int32_t rtd_sw_fpu_250_code;
    float rtd_sw_fpu_250_resistance;
    float rtd_sw_fpu_250_voltage;
    int32_t rtd_sw_fpu_350_code;
    float rtd_sw_fpu_350_resistance;
    float rtd_sw_fpu_350_voltage;
    int32_t rtd_uc_head_code;
    float rtd_uc_head_resistance;
    float rtd_uc_head_voltage;
    int32_t time_mcu;
} HKDataEighty;

void udp_receive_cryo_hk_1Hz(void *arg);
void udp_receive_cryo_hk_20Hz(void *arg);
void udp_receive_cryo_hk_80Hz(void *arg);

void set_channels_cryo_hk_1Hz(void);
void set_channels_cryo_hk_20Hz(void);
void set_channels_cryo_hk_80Hz(void);
