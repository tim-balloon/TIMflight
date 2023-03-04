/* derived.c: a list of derived channels
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 *
 * IF YOU ADD, MODIFY, OR DELETE *ANY* CHANNELS IN THIS FILE YOU *MUST*
 * RECOMPILE AND RESTART THE DECOM DAEMON (DECOMD) ON ARWEN!
 *
 * !XXX!!XXX!!XXX!!XXX!!XXX!! BIG ALL CAPS WARNING !!XXX!!XXX!!XXX!!XXX!!XXX!!
 */

#include "derived.h"

/* Don's Handy Guide to adding derived channels:
 *
 * There are seven types of derived channels which can be added to the format
 * file.  Channel names are, of course, case sensitive.  On start-up, both
 * mcp and decomd run sanity checks on the channel lists, including the derived
 * channel list.  The checks will fail for the derived channel list if either
 * a source channel doesn't exist or a derived channel has a name already in
 * use.  Listed below are the derived channel types and their arguments.  If
 * more derived channel types are desired, please ask me to add them.  --dvw
 *
 * o LINCOM: Single field calibration.  Arguments:
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Calibration Slope (double)
 *   4.  Calibration Intercept (double)
 * o LINCOM2: Two field linear combination
 *   1.  Derived Channel Name (string)
 *   2.  Source 1 Channel Name (string)
 *   3.  Source 1 Calibration Slope (double)
 *   4.  Source 1 Calibration Intercept (double)
 *   5.  Source 2 Channel Name (string)
 *   6.  Source 2 Calibration Slope (double)
 *   7.  Source 2 Calibration Intercept (double)
 * o LINTERP: Linearly interpolated look up table
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  Full Path to Look Up Table File (string)
 * o BITS: A bit channel extracted from a larger source channel
 *   1.  Derived Channel Name (string)
 *   2.  Source Channel Name (string)
 *   3.  First bit of the bitword, numbered from zero (integer)
 *   4.  Length of the bitword (integer)
 * o COMMENT: A literal comment to be inserted into the format file
 *   1.  Comment Text (string) -- there is no need to include the comment
 *       delimiter (#).
 * o UNITS: meta data defining units and quantity for existing fields
 *   1.  Source Channel Name (string)
 *   2.  String for the quantity (eg "Temperature")
 *   3.  String for the Units (eg, "^oC")
 *
 *
 * In addition to the derived channels derived below, defile will add the
 * "Nice CPU Values" (to wit: CPU_SEC, CPU_HOUR, etc.), properly offset to the
 * start of the file, to the end of the format file.
 */

#define LUT_DIR "/data/etc/blast/"
// offset looks like 500mV which gets multiplied up by 5 in mx+b
#define CURR_LOOP_OFFSET -2.5000

derived_tng_t derived_list[] = {
  /* Pointing */
  COMMENT("Microsecond Resolution Time"),
  LINCOM2("Time", "time_usec", 1.0E-6, 0, "time",  1, 0),
  UNITS("Time", "Time", "s"),

  COMMENT("General BLAST Status"),
  BITWORD("SOUTH_I_AM", "status_mcc", 0, 1),
  BITWORD("AT_FLOAT", "status_mcc", 1, 1),
  BITWORD("UPLINK_SCHED", "status_mcc", 2, 1),
  BITWORD("BLAST_SUCKS", "status_mcc", 3, 1),
  BITWORD("SCHEDULE", "status_mcc", 4, 3),
  BITWORD("INCHARGE", "status_mcc", 7, 1),
  BITWORD("SLOT_SCHED", "status_mcc", 8, 8),

#ifndef BOLOTEST
    COMMENT("Pointing Stuff"),
    LINCOM("X_H_P", "x_p", 0.0003662109375, 0),

    BITWORD("VETO_EL_MOTOR_ENC", "veto_sensor", 0, 1),
    BITWORD("VETO_XSC0", "veto_sensor", 1, 1),
//    BITWORD("VETO_EL_ENC", "veto_sensor", 2, 1),
//  (LMF 2018-12-20 not needed.  Can be used for another sensor.)
    BITWORD("VETO_MAG1", "veto_sensor", 3, 1),
    BITWORD("VETO_MAG2", "veto_sensor", 4, 1),
    BITWORD("VETO_EL_CLIN", "veto_sensor", 5, 1),
    BITWORD("VETO_XSC1", "veto_sensor", 6, 1),
    BITWORD("IS_SCHED", "veto_sensor", 7, 1),
    BITWORD("AZ_AUTO_GYRO", "veto_sensor", 8, 1),
    BITWORD("EL_AUTO_GYRO", "veto_sensor", 9, 1),
    BITWORD("DISABLE_EL", "veto_sensor", 10, 1),
    BITWORD("DISABLE_AZ", "veto_sensor", 11, 1),
    BITWORD("FORCE_EL", "veto_sensor", 12, 1),
    BITWORD("VETO_PSS", "veto_sensor", 13, 1),
    BITWORD("VETO_DGPS", "veto_sensor", 14, 1),
    COMMENT("ACS Digital Signals"),

    /* charge controller (CC) faults and alarms */
    COMMENT("Charge Controller Bitfields"),

    BITWORD("F_OVERCURRENT_CC1", "fault_cc1", 0, 1),
    BITWORD("F_FET_SHORT_CC1", "fault_cc1", 1, 1),
    BITWORD("F_SOFTWARE_BUG_CC1", "fault_cc1", 2, 1),
    BITWORD("F_BATT_HVD_CC1", "fault_cc1", 3, 1),
    BITWORD("F_ARR_HVD_CC1", "fault_cc1", 4, 1),
    BITWORD("F_DIP_CHANGED_CC1", "fault_cc1", 5, 1),
    BITWORD("F_SETTING_CHANGE_CC1", "fault_cc1", 6, 1),
    BITWORD("F_RTS_SHORT_CC1", "fault_cc1", 7, 1),
    BITWORD("F_RTS_DISCONN_CC1", "fault_cc1", 8, 1),
    BITWORD("F_EEPROM_LIM_CC1", "fault_cc1", 9, 1),
    BITWORD("F_SLAVE_TO_CC1", "fault_cc1", 10, 1),

    BITWORD("F_OVERCURRENT_CC2", "fault_cc2", 0, 1),
    BITWORD("F_FET_SHORT_CC2", "fault_cc2", 1, 1),
    BITWORD("F_SOFTWARE_BUG_CC2", "fault_cc2", 2, 1),
    BITWORD("F_BATT_HVD_CC2", "fault_cc2", 3, 1),
    BITWORD("F_ARR_HVD_CC2", "fault_cc2", 4, 1),
    BITWORD("F_DIP_CHANGED_CC2", "fault_cc2", 5, 1),
    BITWORD("F_SETTING_CHANGE_CC2", "fault_cc2", 6, 1),
    BITWORD("F_RTS_SHORT_CC2", "fault_cc2", 7, 1),
    BITWORD("F_RTS_DISCONN_CC2", "fault_cc2", 8, 1),
    BITWORD("F_EEPROM_LIM_CC2", "fault_cc2", 9, 1),
    BITWORD("F_SLAVE_TO_CC2", "fault_cc2", 10, 1),

    BITWORD("A_RTS_OPEN_CC1", "alarm_cc1", 0, 1),
    BITWORD("A_RTS_SHORT_CC1", "alarm_cc1", 1, 1),
    BITWORD("A_RTS_DISCONN_CC1", "alarm_cc1", 2, 1),
    BITWORD("A_TSENSE_OPEN_CC1", "alarm_cc1", 3, 1),
    BITWORD("A_TSENSE_SHORT_CC1", "alarm_cc1", 4, 1),
    BITWORD("A_HITEMP_LIM_CC1", "alarm_cc1", 5, 1),
    BITWORD("A_CURRENT_LIM_CC1", "alarm_cc1", 6, 1),
    BITWORD("A_CURRENT_OFFSET_CC1", "alarm_cc1", 7, 1),
    BITWORD("A_BATT_RANGE_CC1", "alarm_cc1", 8, 1),
    BITWORD("A_BATTSENSE_DISC_CC1", "alarm_cc1", 9, 1),
    BITWORD("A_UNCALIB_CC1", "alarm_cc1", 10, 1),
    BITWORD("A_RTS_MISWIRE_CC1", "alarm_cc1", 11, 1),
    BITWORD("A_HVD_CC1", "alarm_cc1", 12, 1),
    BITWORD("A_SYS_MISWIRE_CC1", "alarm_cc1", 14, 1),
    BITWORD("A_FET_OPEN_CC1", "alarm_cc1", 15, 1),

    BITWORD("A_RTS_OPEN_CC2", "alarm_cc2", 0, 1),
    BITWORD("A_RTS_SHORT_CC2", "alarm_cc2", 1, 1),
    BITWORD("A_RTS_DISCONN_CC2", "alarm_cc2", 2, 1),
    BITWORD("A_TSENSE_OPEN_CC2", "alarm_cc2", 3, 1),
    BITWORD("A_TSENSE_SHORT_CC2", "alarm_cc2", 4, 1),
    BITWORD("A_HITEMP_LIM_CC2", "alarm_cc2", 5, 1),
    BITWORD("A_CURRENT_LIM_CC2", "alarm_cc2", 6, 1),
    BITWORD("A_CURRENT_OFFSET_CC2", "alarm_cc2", 7, 1),
    BITWORD("A_BATT_RANGE_CC2", "alarm_cc2", 8, 1),
    BITWORD("A_BATTSENSE_DISC_CC2", "alarm_cc2", 9, 1),
    BITWORD("A_UNCALIB_CC2", "alarm_cc2", 10, 1),
    BITWORD("A_RTS_MISWIRE_CC2", "alarm_cc2", 11, 1),
    BITWORD("A_HVD_CC2", "alarm_cc2", 12, 1),
    BITWORD("A_SYS_MISWIRE_CC2", "alarm_cc2", 14, 1),
    BITWORD("A_FET_OPEN_CC2", "alarm_cc2", 15, 1),

    BITWORD("A_VP12_OFF_CC1", "alarm_cc1", 16, 1),
    BITWORD("C_A_HI_INPUT_LIM_CC1", "alarm_cc1", 17, 1),
    BITWORD("C_A_ADC_MAX_IN_CC1", "alarm_cc1", 18, 1),
    BITWORD("C_A_RESET_CC1", "alarm_cc1", 19, 1),

    BITWORD("A_VP12_OFF_CC2", "alarm_cc2", 16, 1),
    BITWORD("C_A_HI_INPUT_LIM_CC2", "alarm_cc2", 17, 1),
    BITWORD("C_A_ADC_MAX_IN_CC2", "alarm_cc2", 18, 1),
    BITWORD("C_A_RESET_CC2", "alarm_cc2", 19, 1),

    BITWORD("ACT0_INIT_ACTBUS", "status_actbus", 0, 1),
    BITWORD("ACT1_INIT_ACTBUS", "status_actbus", 1, 1),
    BITWORD("ACT2_INIT_ACTBUS", "status_actbus", 2, 1),
    BITWORD("BAL_INIT_ACTBUS", "status_actbus", 3, 1),
    BITWORD("LOCK_INIT_ACTBUS", "status_actbus", 4, 1),
    BITWORD("SHUTTER_INIT_ACTBUS", "status_actbus", 6, 1),
    BITWORD("PUMPED_POT_INIT_ACTBUS", "status_actbus", 7, 1),
    BITWORD("PUMP_VALVE_INIT_ACTBUS", "status_actbus", 8, 1),
    BITWORD("FILL_VALVE_INIT_ACTBUS", "status_actbus", 9, 1),

    BITWORD("DIR_BAL", "status_bal", 0, 2),
    BITWORD("INIT_BAL", "status_bal", 2, 1),
    BITWORD("DO_MOVE_BAL", "status_bal", 3, 1),
    BITWORD("MOVING_BAL", "status_bal", 4, 1),
    BITWORD("MODE_BAL", "status_bal", 5, 2),

      #endif

    COMMENT("Lock Motor/Actuators"),
    BITWORD("LS_OPEN_LOCK", "state_lock", 0, 1),
    BITWORD("LS_CLOSED_LOCK", "state_lock", 1, 1),
    BITWORD("LS_DRIVE_OFF_LOCK", "state_lock", 2, 1),
    BITWORD("LS_POT_RAIL_LOCK", "state_lock", 3, 1),
    BITWORD("LS_DRIVE_EXT_LOCK", "state_lock", 4, 1),
    BITWORD("LS_DRIVE_RET_LOCK", "state_lock", 5, 1),
    BITWORD("LS_DRIVE_STP_LOCK", "state_lock", 6, 1),
    BITWORD("LS_DRIVE_JIG_LOCK", "state_lock", 7, 1),
    BITWORD("LS_DRIVE_UNK_LOCK", "state_lock", 8, 1),
    BITWORD("LS_EL_OK_LOCK", "state_lock", 9, 1),
    BITWORD("LS_IGNORE_EL_LOCK", "state_lock", 10, 1),
    BITWORD("LS_DRIVE_FORCE_LOCK", "state_lock", 11, 1),

//      ),

  /* Secondary Focus */
    COMMENT("Secondary Focus"),
    LINCOM2("REL_FOCUS_SF", "CORRECTION_SF", 1, 0, "OFFSET_SF", 1, 0),
    LINCOM2("VETO_SF", "WAIT_SF", 1, 0, "AGE_SF", -1, 0),

    LINCOM2("Pos_0_act", "POS_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
    LINCOM2("Pos_1_act", "POS_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
    LINCOM2("Pos_2_act", "POS_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
    LINCOM2("Enc_0_act", "ENC_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
    LINCOM2("Enc_1_act", "ENC_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
    LINCOM2("Enc_2_act", "ENC_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
    LINCOM2("Goal_0_act", "GOAL_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
    LINCOM2("Goal_1_act", "GOAL_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
    LINCOM2("Goal_2_act", "GOAL_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),
    LINCOM2("Dr_0_act", "DR_0_ACT", 1, 0, "OFFSET_0_ACT", 1, 0),
    LINCOM2("Dr_1_act", "DR_1_ACT", 1, 0, "OFFSET_1_ACT", 1, 0),
    LINCOM2("Dr_2_act", "DR_2_ACT", 1, 0, "OFFSET_2_ACT", 1, 0),

    BITWORD("TRIM_WAIT_ACT", "flags_act", 0, 1),
    BITWORD("TRIMMED_ACT", "flags_act", 1, 1),
    BITWORD("BUSY_0_ACT", "flags_act", 2, 1),
    BITWORD("BUSY_1_ACT", "flags_act", 3, 1),
    BITWORD("BUSY_2_ACT", "flags_act", 4, 1),
    BITWORD("BAD_MOVE_ACT", "flags_act", 5, 1),

    BITWORD("LATCHED_DATA_CRC_EL", "latched_fault_el", 0, 1),
    BITWORD("LATCHED_AMP_INT_EL", "latched_fault_el", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_EL", "latched_fault_el", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_EL", "latched_fault_el", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_EL", "latched_fault_el", 4, 1),
    BITWORD("LATCHED_OVERVOLT_EL", "latched_fault_el", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_EL", "latched_fault_el", 6, 1),
    BITWORD("LATCHED_FEEDBACK_EL", "latched_fault_el", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_EL", "latched_fault_el", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_EL", "latched_fault_el", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_EL", "latched_fault_el", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_EL", "latched_fault_el", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_EL", "latched_fault_el", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_EL", "latched_fault_el", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_EL", "latched_fault_el", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_EL", "latched_fault_el", 15, 1),

    BITWORD("LATCHED_DATA_CRC_PIV", "latched_fault_piv", 0, 1),
    BITWORD("LATCHED_AMP_INT_PIV", "latched_fault_piv", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_PIV", "latched_fault_piv", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_PIV", "latched_fault_piv", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_PIV", "latched_fault_piv", 4, 1),
    BITWORD("LATCHED_OVERVOLT_PIV", "latched_fault_piv", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_PIV", "latched_fault_piv", 6, 1),
    BITWORD("LATCHED_FEEDBACK_PIV", "latched_fault_piv", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_PIV", "latched_fault_piv", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_PIV", "latched_fault_piv", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_PIV", "latched_fault_piv", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_PIV", "latched_fault_piv", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_PIV", "latched_fault_piv", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_PIV", "latched_fault_piv", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_PIV", "latched_fault_piv", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_PIV", "latched_fault_piv", 15, 1),

    BITWORD("LATCHED_DATA_CRC_RW", "latched_fault_rw", 0, 1),
    BITWORD("LATCHED_AMP_INT_RW", "latched_fault_rw", 1, 1),
    BITWORD("LATCHED_SHORT_CIRC_RW", "latched_fault_rw", 2, 1),
    BITWORD("LATCHED_AMP_OVER_T_RW", "latched_fault_rw", 3, 1),
    BITWORD("LATCHED_MOTOR_OVER_T_RW", "latched_fault_rw", 4, 1),
    BITWORD("LATCHED_OVERVOLT_RW", "latched_fault_rw", 5, 1),
    BITWORD("LATCHED_UNDERVOLT_RW", "latched_fault_rw", 6, 1),
    BITWORD("LATCHED_FEEDBACK_RW", "latched_fault_rw", 7, 1),
    BITWORD("LATCHED_PHASING_ERR_RW", "latched_fault_rw", 8, 1),
    BITWORD("LATCHED_TRACKING_ERR_RW", "latched_fault_rw", 9, 1),
    BITWORD("LATCHED_OVERCURRENT_RW", "latched_fault_rw", 10, 1),
    BITWORD("LATCHED_FPGA_FAIL_RW", "latched_fault_rw", 11, 1),
    BITWORD("LATCHED_CMD_INPUT_LOST_RW", "latched_fault_rw", 12, 1),
    BITWORD("LATCHED_FPGA_FAIL2_RW", "latched_fault_rw", 13, 1),
    BITWORD("LATCHED_SAFETY_CIRC_RW", "latched_fault_rw", 14, 1),
    BITWORD("LATCHED_CONT_CURRENT_RW", "latched_fault_rw", 15, 1),

    BITWORD("ST_SHORT_CIRC_EL", "status_el", 0, 1),
    BITWORD("ST_AMP_OVER_T_EL", "status_el", 1, 1),
    BITWORD("ST_OVER_V_EL", "status_el", 2, 1),
    BITWORD("ST_UNDER_V_EL", "status_el", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_EL", "status_el", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_EL", "status_el", 6, 1),
    BITWORD("ST_I_LIMITED_EL", "status_el", 7, 1),
    BITWORD("ST_VOLT_LIM_EL", "status_el", 8, 1),
    BITWORD("ST_LIM_SW_POS_EL", "status_el", 9, 1),
    BITWORD("ST_LIM_SW_NEG_EL", "status_el", 10, 1),

    BITWORD("ST_DISAB_HWARE_EL", "status_el", 11, 1),
    BITWORD("ST_DISAB_SWARE_EL", "status_el", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_EL", "status_el", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_EL", "status_el", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_EL", "status_el", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_EL", "status_el", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_EL", "status_el", 17, 1),
    BITWORD("ST_FOLLOW_ERR_EL", "status_el", 18, 1),
    BITWORD("ST_FOLLOW_WARN_EL", "status_el", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_EL", "status_el", 20, 1),
    BITWORD("ST_ENCODER_WRAP_EL", "status_el", 21, 1),
    BITWORD("ST_AMP_FAULT_EL", "status_el", 22, 1),
    BITWORD("ST_VEL_LIMITED_EL", "status_el", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_EL", "status_el", 24, 1),

    BITWORD("ST_IN_MOTION_EL", "status_el", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_EL", "status_el", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_EL", "status_el", 29, 1),

    BITWORD("ST_SHORT_CIRC_PIV", "status_piv", 0, 1),
    BITWORD("ST_AMP_OVER_T_PIV", "status_piv", 1, 1),
    BITWORD("ST_OVER_V_PIV", "status_piv", 2, 1),
    BITWORD("ST_UNDER_V_PIV", "status_piv", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_PIV", "status_piv", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_PIV", "status_piv", 6, 1),
    BITWORD("ST_I_LIMITED_PIV", "status_piv", 7, 1),
    BITWORD("ST_VOLT_LIM_PIV", "status_piv", 8, 1),
    BITWORD("ST_LIM_SW_POS_PIV", "status_piv", 9, 1),
    BITWORD("ST_LIM_SW_NEG_PIV", "status_piv", 10, 1),

    BITWORD("ST_DISAB_HWARE_PIV", "status_piv", 11, 1),
    BITWORD("ST_DISAB_SWARE_PIV", "status_piv", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_PIV", "status_piv", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_PIV", "status_piv", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_PIV", "status_piv", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_PIV", "status_piv", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_PIV", "status_piv", 17, 1),
    BITWORD("ST_FOLLOW_ERR_PIV", "status_piv", 18, 1),
    BITWORD("ST_FOLLOW_WARN_PIV", "status_piv", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_PIV", "status_piv", 20, 1),
    BITWORD("ST_ENCODER_WRAP_PIV", "status_piv", 21, 1),
    BITWORD("ST_AMP_FAULT_PIV", "status_piv", 22, 1),
    BITWORD("ST_VEL_LIMITED_PIV", "status_piv", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_PIV", "status_piv", 24, 1),

    BITWORD("ST_IN_MOTION_PIV", "status_piv", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_PIV", "status_piv", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_PIV", "status_piv", 29, 1),

    BITWORD("ST_SHORT_CIRC_RW", "status_rw", 0, 1),
    BITWORD("ST_AMP_OVER_T_RW", "status_rw", 1, 1),
    BITWORD("ST_OVER_V_RW", "status_rw", 2, 1),
    BITWORD("ST_UNDER_V_RW", "status_rw", 3, 1),

    BITWORD("ST_FEEDBACK_ERR_RW", "status_rw", 5, 1),
    BITWORD("ST_MOT_PHAS_ERR_RW", "status_rw", 6, 1),
    BITWORD("ST_I_LIMITED_RW", "status_rw", 7, 1),
    BITWORD("ST_VOLT_LIM_RW", "status_rw", 8, 1),
    BITWORD("ST_LIM_SW_POS_RW", "status_rw", 9, 1),
    BITWORD("ST_LIM_SW_NEG_RW", "status_rw", 10, 1),

    BITWORD("ST_DISAB_HWARE_RW", "status_rw", 11, 1),
    BITWORD("ST_DISAB_SWARE_RW", "status_rw", 12, 1),
    BITWORD("ST_ATTEMPT_STOP_RW", "status_rw", 13, 1),
    BITWORD("ST_MOT_BREAK_ACT_RW", "status_rw", 14, 1),
    BITWORD("ST_PWM_OUT_DIS_RW", "status_rw", 15, 1),
    BITWORD("ST_POS_SOFT_LIM_RW", "status_rw", 16, 1),
    BITWORD("ST_NEG_SOFT_LIM_RW", "status_rw", 17, 1),
    BITWORD("ST_FOLLOW_ERR_RW", "status_rw", 18, 1),
    BITWORD("ST_FOLLOW_WARN_RW", "status_rw", 19, 1),
    BITWORD("ST_AMP_HAS_RESET_RW", "status_rw", 20, 1),
    BITWORD("ST_ENCODER_WRAP_RW", "status_rw", 21, 1),
    BITWORD("ST_AMP_FAULT_RW", "status_rw", 22, 1),
    BITWORD("ST_VEL_LIMITED_RW", "status_rw", 23, 1),
    BITWORD("ST_ACCEL_LIMITED_RW", "status_rw", 24, 1),

    BITWORD("ST_IN_MOTION_RW", "status_rw", 27, 1),
    BITWORD("ST_V_OUT_TRACK_W_RW", "status_rw", 28, 1),
    BITWORD("ST_PHASE_NOT_INIT_RW", "status_rw", 29, 1),
    BITWORD("NET_ST_NODE_STATUS_RW", "network_status_rw", 0, 2),
    BITWORD("NET_ST_SYNC_MISSING_RW", "network_status_rw", 4, 1),
    BITWORD("NET_ST_GUARD_ERR_RW", "network_status_rw", 5, 1),
    BITWORD("NET_ST_BUS_OFF_RW", "network_status_rw", 8, 1),
    BITWORD("NET_ST_SYNC_TRANS_ERROR_RW", "network_status_rw", 9, 1),
    BITWORD("NET_ST_SYNC_REC_ERROR_RW", "network_status_rw", 10, 1),
    BITWORD("NET_ST_SYNC_TRANS_WARNING_RW", "network_status_rw", 11, 1),
    BITWORD("NET_ST_SYNC_REC_WARNING_RW", "network_status_rw", 12, 1),
    BITWORD("STATE_READY_EL", "state_el", 0, 1),
    BITWORD("STATE_SWITCHED_ON_EL", "state_el", 1, 1),
    BITWORD("STATE_OP_ENABLED_EL", "state_el", 2, 1),
    BITWORD("STATE_FAULT_EL", "state_el", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_EL", "state_el", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_EL", "state_el", 5, 1),
    BITWORD("STATE_ON_DISABLE_EL", "state_el", 6, 1),
    BITWORD("STATE_WARNING_EL", "state_el", 7, 1),
    BITWORD("STATE_ABORTED_EL", "state_el", 8, 1),
    BITWORD("STATE_REMOTE_EL", "state_el", 9, 1),
    BITWORD("STATE_ON_TARGET_EL", "state_el", 10, 1),
    BITWORD("STATE_AMP_LIMIT_EL", "state_el", 11, 1),
    BITWORD("STATE_MOVING_EL", "state_el", 14, 1),
    BITWORD("NET_ST_NODE_STATUS_EL", "network_status_el", 0, 2),
    BITWORD("NET_ST_SYNC_MISSING_EL", "network_status_el", 4, 1),
    BITWORD("NET_ST_GUARD_ERR_EL", "network_status_el", 5, 1),
    BITWORD("NET_ST_BUS_OFF_EL", "network_status_el", 8, 1),
    BITWORD("NET_ST_SYNC_TRANS_ERROR_EL", "network_status_el", 9, 1),
    BITWORD("NET_ST_SYNC_REC_ERROR_EL", "network_status_el", 10, 1),
    BITWORD("NET_ST_SYNC_TRANS_WARNING_EL", "network_status_el", 11, 1),
    BITWORD("NET_ST_SYNC_REC_WARNING_EL", "network_status_el", 12, 1),
    BITWORD("STATE_READY_RW", "state_rw", 0, 1),
    BITWORD("STATE_SWITCHED_ON_RW", "state_rw", 1, 1),
    BITWORD("STATE_OP_ENABLED_RW", "state_rw", 2, 1),
    BITWORD("STATE_FAULT_RW", "state_rw", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_RW", "state_rw", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_RW", "state_rw", 5, 1),
    BITWORD("STATE_ON_DISABLE_RW", "state_rw", 6, 1),
    BITWORD("STATE_WARNING_RW", "state_rw", 7, 1),
    BITWORD("STATE_ABORTED_RW", "state_rw", 8, 1),
    BITWORD("STATE_REMOTE_RW", "state_rw", 9, 1),
    BITWORD("STATE_ON_TARGET_RW", "state_rw", 10, 1),
    BITWORD("STATE_AMP_LIMIT_RW", "state_rw", 11, 1),
    BITWORD("STATE_MOVING_RW", "state_rw", 14, 1),

    BITWORD("STATE_READY_PIV", "state_piv", 0, 1),
    BITWORD("STATE_SWITCHED_ON_PIV", "state_piv", 1, 1),
    BITWORD("STATE_OP_ENABLED_PIV", "state_piv", 2, 1),
    BITWORD("STATE_FAULT_PIV", "state_piv", 3, 1),
    BITWORD("STATE_VOLTAGE_ENABLED_PIV", "state_piv", 4, 1),
    BITWORD("STATE_NOT_QUICK_STOP_PIV", "state_piv", 5, 1),
    BITWORD("STATE_ON_DISABLE_PIV", "state_piv", 6, 1),
    BITWORD("STATE_WARNING_PIV", "state_piv", 7, 1),
    BITWORD("STATE_ABORTED_PIV", "state_piv", 8, 1),
    BITWORD("STATE_REMOTE_PIV", "state_piv", 9, 1),
    BITWORD("STATE_ON_TARGET_PIV", "state_piv", 10, 1),
    BITWORD("STATE_AMP_LIMIT_PIV", "state_piv", 11, 1),
    BITWORD("STATE_MOVING_PIV", "state_piv", 14, 1),
    BITWORD("NET_ST_NODE_STATUS_PIV", "network_status_piv", 0, 2),
    BITWORD("NET_ST_SYNC_MISSING_PIV", "network_status_piv", 4, 1),
    BITWORD("NET_ST_GUARD_ERR_PIV", "network_status_piv", 5, 1),
    BITWORD("NET_ST_BUS_OFF_PIV", "network_status_piv", 8, 1),
    BITWORD("NET_ST_SYNC_TRANS_ERROR_PIV", "network_status_piv", 9, 1),
    BITWORD("NET_ST_SYNC_REC_ERROR_PIV", "network_status_piv", 10, 1),
    BITWORD("NET_ST_SYNC_TRANS_WARNING_PIVOT", "network_status_piv", 11, 1),
    BITWORD("NET_ST_SYNC_REC_WARNING_PIVOT", "network_status_piv", 12, 1),
    BITWORD("MC_EC_CMD_RESET", "mc_cmd_status", 0, 1),
    BITWORD("MC_EC_CMD_FIX_RW", "mc_cmd_status", 1, 1),
    BITWORD("MC_EC_CMD_FIX_EL", "mc_cmd_status", 2, 1),
    BITWORD("MC_EC_CMD_FIX_PIV", "mc_cmd_status", 3, 1),
    BITWORD("SLAVE_INDEX_EC_RW", "status_ec_rw", 0, 3),
    BITWORD("COMMS_OK_EC_RW", "status_ec_rw", 3, 1),
    BITWORD("SLAVE_ERROR_EC_RW", "status_ec_rw", 4, 1),
    BITWORD("HAS_DC_EC_RW", "status_ec_rw", 5, 1),
    BITWORD("IS_MC_EC_RW", "status_ec_rw", 6, 1),
    BITWORD("SLAVE_INDEX_EC_EL", "status_ec_el", 0, 3),
    BITWORD("COMMS_OK_EC_EL", "status_ec_el", 3, 1),
    BITWORD("SLAVE_ERROR_EC_EL", "status_ec_el", 4, 1),
    BITWORD("HAS_DC_EC_EL", "status_ec_el", 5, 1),
    BITWORD("IS_MC_EC_EL", "status_ec_el", 6, 1),
    BITWORD("SLAVE_INDEX_EC_PIV", "status_ec_piv", 0, 3),
    BITWORD("COMMS_OK_EC_PIV", "status_ec_piv", 3, 1),
    BITWORD("SLAVE_ERROR_EC_PIV", "status_ec_piv", 4, 1),
    BITWORD("HAS_DC_EC_PIV", "status_ec_piv", 5, 1),
    BITWORD("IS_MC_EC_PIV", "status_ec_piv", 6, 1),
    BITWORD("CTL_ON_READ_PIV", "control_word_read_piv", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_READ_PIV", "control_word_read_piv", 1, 1),
    BITWORD("CTL_QUICK_STOP_READ_PIV", "control_word_read_piv", 2, 1),
    BITWORD("CTL_ENABLE_READ_PIV", "control_word_read_piv", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_READ_PIV", "control_word_read_piv", 4, 3),
    BITWORD("CTL_RESET_FAULT_READ_PIV", "control_word_read_piv", 7, 1),
    BITWORD("CTL_HALT_READ_PIV", "control_word_read_piv", 8, 1),
    BITWORD("CTL_ON_WRITE_PIV", "control_word_write_piv", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_WRITE_PIV", "control_word_write_piv", 1, 1),
    BITWORD("CTL_QUICK_STOP_WRITE_PIV", "control_word_write_piv", 2, 1),
    BITWORD("CTL_ENABLE_WRITE_PIV", "control_word_write_piv", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_WRITE_PIV", "control_word_write_piv", 4, 3),
    BITWORD("CTL_RESET_FAULT_WRITE_PIV", "control_word_write_piv", 7, 1),
    BITWORD("CTL_HALT_WRITE_PIV", "control_word_write_piv", 8, 1),
    BITWORD("CTL_ON_READ_EL", "control_word_read_el", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_READ_EL", "control_word_read_el", 1, 1),
    BITWORD("CTL_QUICK_STOP_READ_EL", "control_word_read_el", 2, 1),
    BITWORD("CTL_ENABLE_READ_EL", "control_word_read_el", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_READ_EL", "control_word_read_el", 4, 3),
    BITWORD("CTL_RESET_FAULT_READ_EL", "control_word_read_el", 7, 1),
    BITWORD("CTL_HALT_READ_EL", "control_word_read_el", 8, 1),
    BITWORD("CTL_ON_WRITE_EL", "control_word_write_el", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_WRITE_EL", "control_word_write_el", 1, 1),
    BITWORD("CTL_QUICK_STOP_WRITE_EL", "control_word_write_el", 2, 1),
    BITWORD("CTL_ENABLE_WRITE_EL", "control_word_write_el", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_WRITE_EL", "control_word_write_el", 4, 3),
    BITWORD("CTL_RESET_FAULT_WRITE_EL", "control_word_write_el", 7, 1),
    BITWORD("CTL_HALT_WRITE_EL", "control_word_write_rw", 8, 1),
    BITWORD("CTL_ON_READ_RW", "control_word_read_rw", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_READ_RW", "control_word_read_rw", 1, 1),
    BITWORD("CTL_QUICK_STOP_READ_RW", "control_word_read_rw", 2, 1),
    BITWORD("CTL_ENABLE_READ_RW", "control_word_read_rw", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_READ_RW", "control_word_read_rw", 4, 3),
    BITWORD("CTL_RESET_FAULT_READ_RW", "control_word_read_rw", 7, 1),
    BITWORD("CTL_HALT_READ_RW", "control_word_read_rw", 8, 1),
    BITWORD("CTL_ON_WRITE_RW", "control_word_write_rw", 0, 1),
    BITWORD("CTL_ENABLE_VOLTAGE_WRITE_RW", "control_word_write_rw", 1, 1),
    BITWORD("CTL_QUICK_STOP_WRITE_RW", "control_word_write_rw", 2, 1),
    BITWORD("CTL_ENABLE_WRITE_RW", "control_word_write_rw", 3, 1),
    BITWORD("CTL_MODE_OF_OPERATION_WRITE_RW", "control_word_write_rw", 4, 3),
    BITWORD("CTL_RESET_FAULT_WRITE_RW", "control_word_write_rw", 7, 1),
    BITWORD("CTL_HALT_WRITE_RW", "control_word_write_rw", 8, 1),

    // Various LabJack channels
    // voltage monitoring
    LINCOM("Outer_frame_voltage_1", "outer_frame_vm_1", 10, 0),
    LINCOM("Outer_frame_voltage_2", "outer_frame_vm_2", 10, 0),
    LINCOM("Outer_frame_voltage_3", "outer_frame_vm_3", 10, 0),
    LINCOM("Inner_frame_voltage_1", "inner_frame_vm_1", 10, 0),
    LINCOM("Inner_frame_voltage_2", "inner_frame_vm_2", 10, 0),
    LINCOM("Inner_frame_voltage_3", "inner_frame_vm_3", 10, 0),

    // current monitoring
    LINCOM("Current_fc1", "current_fc1", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_fc2", "current_fc2", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_of_eth", "current_of_eth", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_motor_lj", "current_motor_box_lj", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_of_unassigned_1", "current_unassigned", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_of_inclinometer", "current_of_inclinometer", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_magnetometer", "current_magnetometers", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_gondola_thermometry", "current_gondola_thermometry", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_gps_ntp", "current_gps_ntp", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_pss_box", "current_pss_box", 5, CURR_LOOP_OFFSET),

    LINCOM("Current_sc1", "current_sc1", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_cryo_digital", "current_cryo_digital", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_gyros", "current_gyros", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_rfsoc", "current_rfsoc", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_if_eth", "current_if_eth", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_steppers", "current_steppers", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_if_inclinometer", "current_if_inclinometer", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_sc2", "current_sc2", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_cryo_analog", "current_cryo_analog", 5, CURR_LOOP_OFFSET),
    LINCOM("Current_last_resort_valve", "current_unassigned", 5, CURR_LOOP_OFFSET),


    // Roach Channels
    // may be a reference for status channels for TIM
    // BITWORD("SCAN_RETUNE_TOP_TRIGGER_ROACH", "scan_retune_trigger_roach", 0, 1),
    // BITWORD("SCAN_RETUNE_BOTTOM_TRIGGER_ROACH", "scan_retune_trigger_roach", 1, 1),


    // we may want these as a reference for RFSoC status channels
    /*
    BITWORD("HAS_QDR_CAL_ROACH5", "status_roach5", 0, 1),
    BITWORD("FULL_LOOP_FAIL_ROACH5", "status_roach5", 1, 1),
    BITWORD("HAS_TARG_TONES_ROACH5", "status_roach5", 2, 1),
    BITWORD("IS_STREAMING_ROACH5", "status_roach5", 3, 1),
    BITWORD("IS_SWEEPING_ROACH5", "status_roach5", 4, 2),
    BITWORD("HAS_VNA_SWEEP_ROACH5", "status_roach5", 6, 1),
    BITWORD("HAS_TARG_SWEEP_ROACH5", "status_roach5", 7, 1),
    BITWORD("WRITE_FLAG_ROACH5", "status_roach5", 8, 1),
    BITWORD("HAS_REF_PARAMS_ROACH5", "status_roach5", 9, 1),
    BITWORD("EXT_REF_ROACH5", "status_roach5", 10, 1),
    BITWORD("HAS_VNA_TONES_ROACH5", "status_roach5", 11, 1),
    BITWORD("SWEEP_FAIL_ROACH5", "status_roach5", 12, 1),
    BITWORD("QDR_CAL_FAIL_ROACH5", "status_roach5", 13, 1),
    BITWORD("FIRMWARE_UPLOAD_FAIL_ROACH5", "status_roach5", 14, 1),
    BITWORD("HAS_FIRMWARE_ROACH5", "status_roach5", 15, 1),
    BITWORD("TONE_FINDING_ERROR_ROACH5", "status_roach5", 16, 2),
    BITWORD("KATCP_CONNECT_ERROR_ROACH5", "status_roach5", 18, 1),
    BITWORD("IS_COMPRESSING_ROACH5", "status_roach5", 19, 1),
    BITWORD("DOING_FULL_LOOP_ROACH5", "status_roach5", 20, 1),
    BITWORD("DOING_FIND_KIDS_LOOP_ROACH5", "status_roach5", 21, 1),
    BITWORD("IS_FINDING_KIDS_ROACH5", "status_roach5", 22, 1),
    BITWORD("TRNAROUND_LOOP_FAIL_ROACH5", "status_roach5", 23, 1),
    BITWORD("DOING_TURNAROUND_LOOP_ROACH5", "status_roach5", 24, 1),
    BITWORD("AUTO_ALLOW_TRNAROUND_LOOP_ROACH5", "status_roach5", 25, 1),
    BITWORD("ALLOW_CHOP_LO_ROACH5", "status_roach5", 26, 1),
    // BITWORD("IS_CHOPPING_LO_ROACH5", "status_roach5", 27, 1),
    BITWORD("PI_REBOOT_WARNING_ROACH5", "status_roach5", 28, 1),
    BITWORD("DATA_STREAM_ERROR_ROACH5", "status_roach5", 29, 1),
    BITWORD("WAITING_FOR_LAMP_ROACH5", "status_roach5", 30, 1),
    BITWORD("HAS_LAMP_CONTROL_ROACH5", "status_roach5", 31, 1),
     */

    // status channels
    // thermistors below!
    // Kept as a reference for future channels
    // LINTERP("Rt_therm1", "THERMISTOR_1", LUT_DIR "Thermistor_VtoR.lut"),
    // LINTERP("Tt_if_front", "Rt_therm1", LUT_DIR "Thermistor_RtoT.lut"),

    // Kept as a reference for future channels
    // BITWORD("Labjack0_conn_status", "labjack_conn_status", 0, 1),

    BITWORD("TRIGGER_XSC0", "trigger_xsc", 0, 1),
    BITWORD("TRIGGER_XSC1", "trigger_xsc", 1, 1),
    BITWORD("TRIGGER_STATE_XSC", "trigger_xsc", 2, 6),

  END_OF_DERIVED_CHANNELS
};
