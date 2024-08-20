// Copyright 2023 Ian Lowe
//  inner_frame_power.h
//
//
//  Created by Ian Lowe on 2/16/23.
//
//

// Addresses for the DIO pins we write to in the modbus register map for the LABJACKs
// free assign
#define IF_RELAY_1_OFF 2001
#define IF_RELAY_1_ON 2002
#define IF_RELAY_2_OFF 2003
#define IF_RELAY_2_ON 2004
#define IF_RELAY_3_OFF 2005
#define IF_RELAY_3_ON 2006
#define IF_RELAY_4_OFF 2007
#define IF_RELAY_4_ON 2008
// TODO(evanmayer): the ON/OFF were swapped (ON, then OFF) for the below two,
// relative to the others in this list. Why? I have swapped them back to agree.
#define IF_RELAY_5_OFF 2009
#define IF_RELAY_5_ON 2010
#define IF_RELAY_6_OFF 2011
#define IF_RELAY_6_ON 2012
#define IF_RELAY_7_OFF 2013
#define IF_RELAY_7_ON 2014
#define IF_RELAY_8_OFF 2015
#define IF_RELAY_8_ON 2016
#define IF_RELAY_9_OFF 2017
#define IF_RELAY_9_ON 2018
#define IF_RELAY_10_OFF 2019
#define IF_RELAY_10_ON 2020

// mapping of the analog inputs to device on relay board
#define VM1 4
#define VM2 2
#define VM3 0
#define IM_IF_RELAY_1 13
#define IM_IF_RELAY_2 11
#define IM_IF_RELAY_3 9
#define IM_IF_RELAY_4 7
#define IM_IF_RELAY_5 5
#define IM_IF_RELAY_6 3
#define IM_IF_RELAY_7 1
#define IM_IF_RELAY_8 10
#define IM_IF_RELAY_9 8
#define IM_IF_RELAY_10 6

void log_if_pbob_analog(void);
void if_pbob_commanding(void);