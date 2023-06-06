// Copyright 2023 Ian Lowe
//  outer_frame_power.h
//
//
//  Created by Ian Lowe on 2/16/23.
//
//

#define FC1_OFF 2001
#define FC1_ON 2002
#define FC2_OFF 2003
#define FC2_ON 2004
// Don't LJ command these 2
#define OF_ETH_ON 2005
#define OF_ETH_OFF 2006
// See above
#define MOTOR_LJ_OFF 2007
#define MOTOR_LJ_ON 2008
#define RELAY_5_OFF 2009
#define RELAY_5_ON 2010
#define OF_INC_OFF 2011
#define OF_INC_ON 2012
#define MAG_OFF 2013
#define MAG_ON 2014
#define THERMISTORS_OFF 2015
#define THERMISTORS_ON 2016
#define GPS_NTP_OFF 2017
#define GPS_NTP_ON 2018
#define PSS_OFF 2019
#define PSS_ON 2020

// mapping of the analog inputs to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_FC1 13
#define IM_FC2 11
#define IM_OF_ETH 9
#define IM_MOT_LJ 7
#define IM_UNASSIGNED 5
#define IM_OF_INC 3
#define IM_MAG 1
#define IM_THERM 10
#define IM_GPS 8
#define IM_PSS 6


void log_of_pbob_analog(void);
void of_pbob_commanding(void);