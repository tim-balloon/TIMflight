/***************************************************************************
 mcp: the BLAST master control program
 
 This software is copyright (C) 2023 University of Arizona
 
 This file is part of mcp.
 
 mcp is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 at your option) any later version.
 
 mcp is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with mcp; if not, write to the Free Software Foundation, Inc.,
 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 
 created by Ian Lowe 3-14-23
 **************************************************************************/


/*************************************************************************
 
 gondola_thermometry.c -- mcp code to readout the gondola thermistors
 
 *************************************************************************/

#include <math.h>
#include <stdio.h>

#include "mcp.h"
#include "channels_tng.h"
#include "lut.h"
#include "tx.h"
#include "command_struct.h"
#include "labjack.h"
#include "labjack_functions.h"
#include "blast.h"
#include "multiplexed_labjack.h"
#include "gondola_thermometry.h"

extern int16_t InCharge;
extern labjack_state_t state[NUM_LABJACKS];

void read_thermistors(void) {
    static int first_time = 1;
    static channel_t* therm_1_Addr, *therm_2_Addr, *therm_3_Addr, *therm_4_Addr;
    static channel_t* therm_5_Addr, *therm_6_Addr, *therm_7_Addr, *therm_8_Addr;
    static channel_t* therm_9_Addr, *therm_10_Addr, *therm_11_Addr, *therm_12_Addr;
    static channel_t* therm_13_Addr, *therm_14_Addr, *therm_15_Addr, *therm_16_Addr;
    static channel_t* therm_17_Addr, *therm_18_Addr, *therm_19_Addr, *therm_20_Addr;
    static channel_t* therm_21_Addr, *therm_22_Addr, *therm_23_Addr, *therm_24_Addr;
    static channel_t* therm_25_Addr, *therm_26_Addr, *therm_27_Addr, *therm_28_Addr;
    static channel_t* therm_29_Addr, *therm_30_Addr, *therm_31_Addr, *therm_32_Addr;
    static channel_t* therm_33_Addr, *therm_34_Addr, *therm_35_Addr, *therm_36_Addr;
    static channel_t* therm_37_Addr, *therm_38_Addr, *therm_39_Addr, *therm_40_Addr;
    static channel_t* therm_41_Addr, *therm_42_Addr, *therm_43_Addr, *therm_44_Addr;
    static channel_t* therm_45_Addr, *therm_46_Addr, *therm_47_Addr, *therm_48_Addr;
    static channel_t* therm_49_Addr, *therm_50_Addr, *therm_51_Addr, *therm_52_Addr;
    static channel_t* therm_53_Addr, *therm_54_Addr, *therm_55_Addr, *therm_56_Addr;
    static channel_t* therm_57_Addr, *therm_58_Addr, *therm_59_Addr, *therm_60_Addr;
    static channel_t* therm_61_Addr, *therm_62_Addr, *therm_63_Addr, *therm_64_Addr;
    static channel_t* therm_65_Addr, *therm_66_Addr, *therm_67_Addr, *therm_68_Addr;
    static channel_t* therm_69_Addr, *therm_70_Addr, *therm_71_Addr, *therm_72_Addr;
    static channel_t* therm_73_Addr, *therm_74_Addr, *therm_75_Addr, *therm_76_Addr;
    static channel_t* therm_77_Addr, *therm_78_Addr, *therm_79_Addr, *therm_80_Addr;
    static channel_t* therm_81_Addr, *therm_82_Addr, *therm_83_Addr, *therm_84_Addr;
    if (state[LABJACK_MULT_OF].connected && InCharge) {
        if (first_time) {
            therm_1_Addr = channels_find_by_name("thermistor_1");
            therm_2_Addr = channels_find_by_name("thermistor_2");
            therm_3_Addr = channels_find_by_name("thermistor_3");
            therm_4_Addr = channels_find_by_name("thermistor_4");
            therm_5_Addr = channels_find_by_name("thermistor_5");
            therm_6_Addr = channels_find_by_name("thermistor_6");
            therm_7_Addr = channels_find_by_name("thermistor_7");
            therm_8_Addr = channels_find_by_name("thermistor_8");
            therm_9_Addr = channels_find_by_name("thermistor_9");
            therm_10_Addr = channels_find_by_name("thermistor_10");
            therm_11_Addr = channels_find_by_name("thermistor_11");
            therm_12_Addr = channels_find_by_name("thermistor_12");
            therm_13_Addr = channels_find_by_name("thermistor_13");
            therm_14_Addr = channels_find_by_name("thermistor_14");
            therm_15_Addr = channels_find_by_name("thermistor_15");
            therm_16_Addr = channels_find_by_name("thermistor_16");
            therm_17_Addr = channels_find_by_name("thermistor_17");
            therm_18_Addr = channels_find_by_name("thermistor_18");
            therm_19_Addr = channels_find_by_name("thermistor_19");
            therm_20_Addr = channels_find_by_name("thermistor_20");
            therm_21_Addr = channels_find_by_name("thermistor_21");
            therm_22_Addr = channels_find_by_name("thermistor_22");
            therm_23_Addr = channels_find_by_name("thermistor_23");
            therm_24_Addr = channels_find_by_name("thermistor_24");
            therm_25_Addr = channels_find_by_name("thermistor_25");
            therm_26_Addr = channels_find_by_name("thermistor_26");
            therm_27_Addr = channels_find_by_name("thermistor_27");
            therm_28_Addr = channels_find_by_name("thermistor_28");
            therm_29_Addr = channels_find_by_name("thermistor_29");
            therm_30_Addr = channels_find_by_name("thermistor_30");
            therm_31_Addr = channels_find_by_name("thermistor_31");
            therm_32_Addr = channels_find_by_name("thermistor_32");
            therm_33_Addr = channels_find_by_name("thermistor_33");
            therm_34_Addr = channels_find_by_name("thermistor_34");
            therm_35_Addr = channels_find_by_name("thermistor_35");
            therm_36_Addr = channels_find_by_name("thermistor_36");
            therm_37_Addr = channels_find_by_name("thermistor_37");
            therm_38_Addr = channels_find_by_name("thermistor_38");
            therm_39_Addr = channels_find_by_name("thermistor_39");
            therm_40_Addr = channels_find_by_name("thermistor_40");
            therm_41_Addr = channels_find_by_name("thermistor_41");
            therm_42_Addr = channels_find_by_name("thermistor_42");
            therm_43_Addr = channels_find_by_name("thermistor_43");
            therm_44_Addr = channels_find_by_name("thermistor_44");
            therm_45_Addr = channels_find_by_name("thermistor_45");
            therm_46_Addr = channels_find_by_name("thermistor_46");
            therm_47_Addr = channels_find_by_name("thermistor_47");
            therm_48_Addr = channels_find_by_name("thermistor_48");
            therm_49_Addr = channels_find_by_name("thermistor_49");
            therm_50_Addr = channels_find_by_name("thermistor_50");
            therm_51_Addr = channels_find_by_name("thermistor_51");
            therm_52_Addr = channels_find_by_name("thermistor_52");
            therm_53_Addr = channels_find_by_name("thermistor_53");
            therm_54_Addr = channels_find_by_name("thermistor_54");
            therm_55_Addr = channels_find_by_name("thermistor_55");
            therm_56_Addr = channels_find_by_name("thermistor_56");
            therm_57_Addr = channels_find_by_name("thermistor_57");
            therm_58_Addr = channels_find_by_name("thermistor_58");
            therm_59_Addr = channels_find_by_name("thermistor_59");
            therm_60_Addr = channels_find_by_name("thermistor_60");
            therm_61_Addr = channels_find_by_name("thermistor_61");
            therm_62_Addr = channels_find_by_name("thermistor_62");
            therm_63_Addr = channels_find_by_name("thermistor_63");
            therm_64_Addr = channels_find_by_name("thermistor_64");
            therm_65_Addr = channels_find_by_name("thermistor_65");
            therm_66_Addr = channels_find_by_name("thermistor_66");
            therm_67_Addr = channels_find_by_name("thermistor_67");
            therm_68_Addr = channels_find_by_name("thermistor_68");
            therm_69_Addr = channels_find_by_name("thermistor_69");
            therm_70_Addr = channels_find_by_name("thermistor_70");
            therm_71_Addr = channels_find_by_name("thermistor_71");
            therm_72_Addr = channels_find_by_name("thermistor_72");
            therm_73_Addr = channels_find_by_name("thermistor_73");
            therm_74_Addr = channels_find_by_name("thermistor_74");
            therm_75_Addr = channels_find_by_name("thermistor_75");
            therm_76_Addr = channels_find_by_name("thermistor_76");
            therm_77_Addr = channels_find_by_name("thermistor_77");
            therm_78_Addr = channels_find_by_name("thermistor_78");
            therm_79_Addr = channels_find_by_name("thermistor_79");
            therm_80_Addr = channels_find_by_name("thermistor_80");
            therm_81_Addr = channels_find_by_name("thermistor_81");
            therm_82_Addr = channels_find_by_name("thermistor_82");
            therm_83_Addr = channels_find_by_name("thermistor_83");
            therm_84_Addr = channels_find_by_name("thermistor_84");
        }
        SET_SCALED_VALUE(therm_1_Addr, labjack_get_value(LABJACK_MULT_OF, 0));
        SET_SCALED_VALUE(therm_2_Addr, labjack_get_value(LABJACK_MULT_OF, 1));
        SET_SCALED_VALUE(therm_3_Addr, labjack_get_value(LABJACK_MULT_OF, 2));
        SET_SCALED_VALUE(therm_4_Addr, labjack_get_value(LABJACK_MULT_OF, 3));
        SET_SCALED_VALUE(therm_5_Addr, labjack_get_value(LABJACK_MULT_OF, 4));
        SET_SCALED_VALUE(therm_6_Addr, labjack_get_value(LABJACK_MULT_OF, 5));
        SET_SCALED_VALUE(therm_7_Addr, labjack_get_value(LABJACK_MULT_OF, 6));
        SET_SCALED_VALUE(therm_8_Addr, labjack_get_value(LABJACK_MULT_OF, 7));
        SET_SCALED_VALUE(therm_9_Addr, labjack_get_value(LABJACK_MULT_OF, 8));
        SET_SCALED_VALUE(therm_10_Addr, labjack_get_value(LABJACK_MULT_OF, 9));
        SET_SCALED_VALUE(therm_11_Addr, labjack_get_value(LABJACK_MULT_OF, 10));
        SET_SCALED_VALUE(therm_12_Addr, labjack_get_value(LABJACK_MULT_OF, 11));
        SET_SCALED_VALUE(therm_13_Addr, labjack_get_value(LABJACK_MULT_OF, 12));
        SET_SCALED_VALUE(therm_14_Addr, labjack_get_value(LABJACK_MULT_OF, 13));
        SET_SCALED_VALUE(therm_15_Addr, labjack_get_value(LABJACK_MULT_OF, 14));
        SET_SCALED_VALUE(therm_16_Addr, labjack_get_value(LABJACK_MULT_OF, 15));
        SET_SCALED_VALUE(therm_17_Addr, labjack_get_value(LABJACK_MULT_OF, 16));
        SET_SCALED_VALUE(therm_18_Addr, labjack_get_value(LABJACK_MULT_OF, 17));
        SET_SCALED_VALUE(therm_19_Addr, labjack_get_value(LABJACK_MULT_OF, 18));
        SET_SCALED_VALUE(therm_20_Addr, labjack_get_value(LABJACK_MULT_OF, 19));
        SET_SCALED_VALUE(therm_21_Addr, labjack_get_value(LABJACK_MULT_OF, 20));
        SET_SCALED_VALUE(therm_22_Addr, labjack_get_value(LABJACK_MULT_OF, 21));
        SET_SCALED_VALUE(therm_23_Addr, labjack_get_value(LABJACK_MULT_OF, 22));
        SET_SCALED_VALUE(therm_24_Addr, labjack_get_value(LABJACK_MULT_OF, 23));
        SET_SCALED_VALUE(therm_25_Addr, labjack_get_value(LABJACK_MULT_OF, 24));
        SET_SCALED_VALUE(therm_26_Addr, labjack_get_value(LABJACK_MULT_OF, 25));
        SET_SCALED_VALUE(therm_27_Addr, labjack_get_value(LABJACK_MULT_OF, 26));
        SET_SCALED_VALUE(therm_28_Addr, labjack_get_value(LABJACK_MULT_OF, 27));
        SET_SCALED_VALUE(therm_29_Addr, labjack_get_value(LABJACK_MULT_OF, 28));
        SET_SCALED_VALUE(therm_30_Addr, labjack_get_value(LABJACK_MULT_OF, 29));
        SET_SCALED_VALUE(therm_31_Addr, labjack_get_value(LABJACK_MULT_OF, 30));
        SET_SCALED_VALUE(therm_32_Addr, labjack_get_value(LABJACK_MULT_OF, 31));
        SET_SCALED_VALUE(therm_33_Addr, labjack_get_value(LABJACK_MULT_OF, 32));
        SET_SCALED_VALUE(therm_34_Addr, labjack_get_value(LABJACK_MULT_OF, 33));
        SET_SCALED_VALUE(therm_35_Addr, labjack_get_value(LABJACK_MULT_OF, 34));
        SET_SCALED_VALUE(therm_36_Addr, labjack_get_value(LABJACK_MULT_OF, 35));
        SET_SCALED_VALUE(therm_37_Addr, labjack_get_value(LABJACK_MULT_OF, 36));
        SET_SCALED_VALUE(therm_38_Addr, labjack_get_value(LABJACK_MULT_OF, 37));
        SET_SCALED_VALUE(therm_39_Addr, labjack_get_value(LABJACK_MULT_OF, 38));
        SET_SCALED_VALUE(therm_40_Addr, labjack_get_value(LABJACK_MULT_OF, 39));
        SET_SCALED_VALUE(therm_41_Addr, labjack_get_value(LABJACK_MULT_OF, 40));
        SET_SCALED_VALUE(therm_42_Addr, labjack_get_value(LABJACK_MULT_OF, 41));
        SET_SCALED_VALUE(therm_43_Addr, labjack_get_value(LABJACK_MULT_OF, 42));
        SET_SCALED_VALUE(therm_44_Addr, labjack_get_value(LABJACK_MULT_OF, 43));
        SET_SCALED_VALUE(therm_45_Addr, labjack_get_value(LABJACK_MULT_OF, 44));
        SET_SCALED_VALUE(therm_46_Addr, labjack_get_value(LABJACK_MULT_OF, 45));
        SET_SCALED_VALUE(therm_47_Addr, labjack_get_value(LABJACK_MULT_OF, 46));
        SET_SCALED_VALUE(therm_48_Addr, labjack_get_value(LABJACK_MULT_OF, 47));
        SET_SCALED_VALUE(therm_49_Addr, labjack_get_value(LABJACK_MULT_OF, 48));
        SET_SCALED_VALUE(therm_50_Addr, labjack_get_value(LABJACK_MULT_OF, 49));
        SET_SCALED_VALUE(therm_51_Addr, labjack_get_value(LABJACK_MULT_OF, 50));
        SET_SCALED_VALUE(therm_52_Addr, labjack_get_value(LABJACK_MULT_OF, 51));
        SET_SCALED_VALUE(therm_53_Addr, labjack_get_value(LABJACK_MULT_OF, 52));
        SET_SCALED_VALUE(therm_54_Addr, labjack_get_value(LABJACK_MULT_OF, 53));
        SET_SCALED_VALUE(therm_55_Addr, labjack_get_value(LABJACK_MULT_OF, 54));
        SET_SCALED_VALUE(therm_56_Addr, labjack_get_value(LABJACK_MULT_OF, 55));
        SET_SCALED_VALUE(therm_57_Addr, labjack_get_value(LABJACK_MULT_OF, 56));
        SET_SCALED_VALUE(therm_58_Addr, labjack_get_value(LABJACK_MULT_OF, 57));
        SET_SCALED_VALUE(therm_59_Addr, labjack_get_value(LABJACK_MULT_OF, 58));
        SET_SCALED_VALUE(therm_60_Addr, labjack_get_value(LABJACK_MULT_OF, 59));
        SET_SCALED_VALUE(therm_61_Addr, labjack_get_value(LABJACK_MULT_OF, 60));
        SET_SCALED_VALUE(therm_62_Addr, labjack_get_value(LABJACK_MULT_OF, 61));
        SET_SCALED_VALUE(therm_63_Addr, labjack_get_value(LABJACK_MULT_OF, 62));
        SET_SCALED_VALUE(therm_64_Addr, labjack_get_value(LABJACK_MULT_OF, 63));
        SET_SCALED_VALUE(therm_65_Addr, labjack_get_value(LABJACK_MULT_OF, 64));
        SET_SCALED_VALUE(therm_66_Addr, labjack_get_value(LABJACK_MULT_OF, 65));
        SET_SCALED_VALUE(therm_67_Addr, labjack_get_value(LABJACK_MULT_OF, 66));
        SET_SCALED_VALUE(therm_68_Addr, labjack_get_value(LABJACK_MULT_OF, 67));
        SET_SCALED_VALUE(therm_69_Addr, labjack_get_value(LABJACK_MULT_OF, 68));
        SET_SCALED_VALUE(therm_70_Addr, labjack_get_value(LABJACK_MULT_OF, 69));
        SET_SCALED_VALUE(therm_71_Addr, labjack_get_value(LABJACK_MULT_OF, 70));
        SET_SCALED_VALUE(therm_72_Addr, labjack_get_value(LABJACK_MULT_OF, 71));
        SET_SCALED_VALUE(therm_73_Addr, labjack_get_value(LABJACK_MULT_OF, 72));
        SET_SCALED_VALUE(therm_74_Addr, labjack_get_value(LABJACK_MULT_OF, 73));
        SET_SCALED_VALUE(therm_75_Addr, labjack_get_value(LABJACK_MULT_OF, 74));
        SET_SCALED_VALUE(therm_76_Addr, labjack_get_value(LABJACK_MULT_OF, 75));
        SET_SCALED_VALUE(therm_77_Addr, labjack_get_value(LABJACK_MULT_OF, 76));
        SET_SCALED_VALUE(therm_78_Addr, labjack_get_value(LABJACK_MULT_OF, 77));
        SET_SCALED_VALUE(therm_79_Addr, labjack_get_value(LABJACK_MULT_OF, 78));
        SET_SCALED_VALUE(therm_80_Addr, labjack_get_value(LABJACK_MULT_OF, 79));
        SET_SCALED_VALUE(therm_81_Addr, labjack_get_value(LABJACK_MULT_OF, 80));
        SET_SCALED_VALUE(therm_82_Addr, labjack_get_value(LABJACK_MULT_OF, 81));
        SET_SCALED_VALUE(therm_83_Addr, labjack_get_value(LABJACK_MULT_OF, 82));
        SET_SCALED_VALUE(therm_84_Addr, labjack_get_value(LABJACK_MULT_OF, 83));
    }
}
