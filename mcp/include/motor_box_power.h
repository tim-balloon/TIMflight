// Copyright 2024 Ian Lowe
//  outer_frame_power.h
//
//
//  Created by Ian Lowe on 7/11/24.
//
//

// map of register number to functionality for the OF power labjack
// ECM: relay 4 controls OF Ethernet. Do not allow power cycling.
#define MOTOR_RELAY_1_OFF 2001
#define MOTOR_RELAY_1_ON 2002
#define MOTOR_RELAY_2_OFF 2003
#define MOTOR_RELAY_2_ON 2004
#define MOTOR_RELAY_3_OFF 2005
#define MOTOR_RELAY_3_ON 2006
// #define MOTOR_RELAY_4_OFF 2007
// #define MOTOR_RELAY_4_ON 2008
#define MOTOR_RELAY_5_OFF 2009
#define MOTOR_RELAY_5_ON 2010
#define MOTOR_RELAY_6_OFF 2011
#define MOTOR_RELAY_6_ON 2012
#define MOTOR_RELAY_7_OFF 2013
#define MOTOR_RELAY_7_ON 2014
#define MOTOR_RELAY_8_OFF 2015
#define MOTOR_RELAY_8_ON 2016
#define MOTOR_RELAY_9_OFF 2017
#define MOTOR_RELAY_9_ON 2018
#define MOTOR_RELAY_10_OFF 2019
#define MOTOR_RELAY_10_ON 2020

// mapping of the analog inputs to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_MOTOR_RELAY_1 13
#define IM_MOTOR_RELAY_2 11
#define IM_MOTOR_RELAY_3 9
#define IM_MOTOR_RELAY_4 7
#define IM_MOTOR_RELAY_5 5
#define IM_MOTOR_RELAY_6 3
#define IM_MOTOR_RELAY_7 1
#define IM_MOTOR_RELAY_8 10
#define IM_MOTOR_RELAY_9 8
#define IM_MOTOR_RELAY_10 6

void motor_pbob_commanding(void);