
/* calibrate.h: field calibrations for the BLASTBus (formerly in channels.h)
 *
 * This software is copyright (C) 2011 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef CALIBRATE_H
#define CALIBRATE_H

/* Fix things up so we can include this in C++ applications */
#ifdef __cplusplus
extern "C" {
#endif

/* 100Hz frame sample rate people are incredibly lazy
 and this got used everywhere despite being blastbus
*/
#define SR (100)

// TODO(evanmayer): we need to figure out if the new motors obey this convention
/**
 * Scaling factors for each motor.  These are hard-wired based on the encoder/resolver
 */
#define RW_ENCODER_COUNTS (1 << 13)
// Scaling factors for each motor.  These are hard-wired based on the encoder/resolver
#define RW_COUNTS_PER_REV (1 << 13)
// Scaling factors for each motor.  These are hard-wired based on the encoder/resolver
#define PIV_RESOLVER_COUNTS (1 << 13)

#define EL_LOAD_ENCODER_COUNTS (1 << 13) /* This is the External, absolute encoder mounted on the inner frame */
#define EL_LOAD_COUNTS_PER_REV (1 << 13)
#define EL_MOTOR_ENCODER_COUNTS (1 << 13) /* No gearbox */
#define EL_MOTOR_COUNTS_PER_REV (1 << 13)

#define RW_ENCODER_SCALING (360.0 / RW_ENCODER_COUNTS)
#define EL_MOTOR_ENCODER_SCALING ((-1.0)*360.0 / EL_MOTOR_ENCODER_COUNTS)
#define EL_LOAD_ENCODER_SCALING (-360.0 / EL_LOAD_ENCODER_COUNTS)
#define PIV_RESOLVER_SCALING (360.0 / PIV_RESOLVER_COUNTS)
#define EL_MOTOR_CURRENT_SCALING (-1.0) /* So that current > 0 -> El increase */

/* Gains and offsets to normalize to -1 to 1: cal = (counts + B)*M */
#define M_32UNI (1.0/2147483648.0)
#define B_32UNI (-2147483648.0)
#define M_16UNI (1.0/32768.0)
#define B_16UNI (-32768.0)

/* Gains and offsets for the labjack AIN channels: cal = (counts + B)*M */
#define M_16LJAIN (10.8/32768.0)
#define B_16LJAIN (-10.8)

/* Gains and offsets for pointing sensors: */
#define M_16MAG (1.0/15000.0)

/* bare thermometer conversion to Volts. No negative values allowed */
#define M_16T (4.096/32768.0/2.0)
#define B_16T (0.0)
/* AD590 calibrations. To Kelvin */
#define M_16_AD590	(M_16T/2.2E-3)
#define B_16_AD590	(B_16T)

/* offset of encoder.  Reset if encoder has been unmounted. */
/* This is the elevation at which the encoder wraps around */
#define ENC_RAW_EL_OFFSET (321.74) //LMF 17-Dec-2018
                                   /* Note this is referenced relative to lock pin hole 25*/
// #define ENC_RAW_EL_OFFSET (291.84) //PCA 11-May-2017
//                                    /* Note this is referenced relative to lock pin hole 0*/

/* to get proper wrapping in KST, the encoder elevation type should be
 * 'u' for 135 <= ENC_EL_RAW_OFFSET < 315 and 's' otherwise */
#define ENC_ELEV_TYPE 'u'

#define STAGE_X_THROW 78500
#define STAGE_Y_THROW 78250

#define ACTENC_TO_UM 1.05833333333 /* mm/enc.counts = 24000 counts/inch */
//#define ACTENC_OFFSET 1000000 /* this number should be arbitrarily larger than
                                 //the maximum throw */


/* Thermal model numbers, from EP */
// TODO(evanmayer): set position focus in encoder units for TIM if we use this
#define T_PRIMARY_FOCUS   296.15 /* = 23C */
#define T_SECONDARY_FOCUS 296.15 /* = 23C */
#define POSITION_FOCUS     33333 /* absolute counts */

#ifndef CAM_WIDTH
#define CAM_WIDTH 1530.0  //should always be the larger dimension
#endif

// hours to long int cap and inverse
#define H2LI (4294967296.0/24.0)
#define LI2H (1.0/H2LI)
// degrees to long int cap and inverse
#define DEG2LI (4294967296.0/360.0)
#define LI2DEG (1.0/DEG2LI)
// radians to long int cap and inverse
#define RAD2LI (4294967296.0/2/M_PI)
#define LI2RAD (1.0/RAD2LI)
// degrees to int and inverse
#define DEG2I (65536.0/360.0)
#define I2DEG (1.0/DEG2I)
// radians to int and inverse
#define RAD2I (65536.0/2/M_PI)
#define I2RAD (1.0/RAD2I)
// hours to int and inverse
#define H2I (65536.0/24.0)
#define I2H (1.0/H2I)
// ???
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

/* Labjack Voltage Calibration */
/* Modified by Ian Summer 2023 */
#define LABJACK_M ( 10.34/32768)
#define LABJACK_B (-10.5869)
/* modified Ian and Mark Palestine */
/* This is with gain set to 10 or range -1 to 1 Volts */
#define CRYO_R_M ( 1.034/32768 )
#define CRYO_R_B (-1.05869 )
/* Current ranges +/-15 Amps*/
#define CUR15_M (15/32768.0)
#define CUR15_B (-15.0)

#ifdef __cplusplus
}
#endif

#endif
