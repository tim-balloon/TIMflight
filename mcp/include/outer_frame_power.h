// Copyright 2023 Ian Lowe
//  outer_frame_power.h
//
//
//  Created by Ian Lowe on 2/16/23.
//
//

// map of register number to functionality for the OF power labjack
// updated Ian 5/11/2026
#define OF_RELAY_1_OFF 2001 // FC1
#define OF_RELAY_1_ON 2002
#define OF_RELAY_2_OFF 2003 // FC2
#define OF_RELAY_2_ON 2004
// #define OF_RELAY_3_OFF 2005 // OF ETH NEVER USE
// #define OF_RELAY_3_ON 2006
#define OF_RELAY_4_OFF 2007 // watchdog
#define OF_RELAY_4_ON 2008
#define OF_RELAY_5_OFF 2009 // thermistors
#define OF_RELAY_5_ON 2010
#define OF_RELAY_6_OFF 2011 // analysis comp
#define OF_RELAY_6_ON 2012
#define OF_RELAY_7_OFF 2013 // motor eth
#define OF_RELAY_7_ON 2014
#define OF_RELAY_8_OFF 2015 // mags
#define OF_RELAY_8_ON 2016
#define OF_RELAY_9_OFF 2017 // NC
#define OF_RELAY_9_ON 2018
#define OF_RELAY_10_OFF 2019 // NC
#define OF_RELAY_10_ON 2020

// mapping of the analog input numbers to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_OF_RELAY_1 13
#define IM_OF_RELAY_2 11
#define IM_OF_RELAY_3 9
#define IM_OF_RELAY_4 7
#define IM_OF_RELAY_5 5
#define IM_OF_RELAY_6 3
#define IM_OF_RELAY_7 1
#define IM_OF_RELAY_8 10
#define IM_OF_RELAY_9 8
#define IM_OF_RELAY_10 6


void log_of_pbob_analog(void);
void of_pbob_commanding(void);