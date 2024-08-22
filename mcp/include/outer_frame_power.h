// Copyright 2023 Ian Lowe
//  outer_frame_power.h
//
//
//  Created by Ian Lowe on 2/16/23.
//
//

// map of register number to functionality for the OF power labjack
#define FC1_OFF 2001
#define FC1_ON 2002
#define FC2_OFF 2003
#define FC2_ON 2004
#define GYROS_OFF 2005
#define GYROS_ON 2006
#define SC1_OFF 2007
#define SC1_ON 2008
#define SC2_OFF 2009
#define SC2_ON 2010
#define GPS_OFF 2011
#define GPS_ON 2012
#define THERMISTORS_OFF 2013
#define THERMISTORS_ON 2014
#define OF_RELAY_8_OFF 2015
#define OF_RELAY_8_ON 2016
#define OF_RELAY_9_OFF 2017
#define OF_RELAY_9_ON 2018
#define OF_RELAY_10_OFF 2019
#define OF_RELAY_10_ON 2020

// mapping of the analog input numbers to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_FC1 13
#define IM_FC2 11
#define IM_GYROS 9
#define IM_SC1 7
#define IM_SC2 5
#define IM_GPS 3
#define IM_THERM 1
#define IM_RELAY_8 10
#define IM_RELAY_9 8
#define IM_RELAY_10 6


void log_of_pbob_analog(void);
void of_pbob_commanding(void);