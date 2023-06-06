// Copyright 2023 Ian Lowe
//  inner_frame_power.h
//
//
//  Created by Ian Lowe on 2/16/23.
//
//

#define SC1_OFF 2001
#define SC1_ON 2002
#define CRYO_HK_READOUT_OFF 2003
#define CRYO_HK_READOUT_ON 2004
#define GYROS_OFF 2005
#define GYROS_ON 2006
#define RFSOC_OFF 2007
#define RFSOC_ON 2008
// Don't use these FFS
#define IF_ETH_ON 2009
#define IF_ETH_OFF 2010
// See above
#define IF_STEPPERS_OFF 2011
#define IF_STEPPERS_ON 2012
#define IF_INC_OFF 2013
#define IF_INC_ON 2014
#define SC2_OFF 2015
#define SC2_ON 2016
#define CRYO_HK_SUPPLY_OFF 2017
#define CRYO_HK_SUPPLY_ON 2018
// free assign
#define RELAY_10_OFF 2019
#define RELAY_10_ON 2020

// mapping of the analog inputs to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_SC1 13
#define IM_CRYO_DIG 11
#define IM_GYROS 9
#define IM_RFSOC 7
#define IM_IF_ETH 5
#define IM_STEPPERS 3
#define IM_IF_INC 1
#define IM_SC2 10
#define IM_CRYO_AN 8
#define IM_LAST_RESORT 6

void log_if_pbob_analog(void);
void if_pbob_commanding(void);