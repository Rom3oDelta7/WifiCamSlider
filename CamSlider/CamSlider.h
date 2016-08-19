/*
   TABS=3

	WiFi Camera Slider Controller common definitions

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#ifndef CAMSLIDER_H
#define CAMSLIDER_H

// OSM 17HS24-0644S stepper motor parameters needed for motion calculations
#define STEPS_PER_REV		200
#define PULLEY_TEETH			20
#define BELT_PITCH			2						// 2 MM
#define INCHES_PER_MM		25.4
#define HS24_MAX_SPEED		592.0					// in steps/sec set using calculator at http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml

#define STEPS_PER_MM			(STEPS_PER_REV / (BELT_PITCH * PULLEY_TEETH))
#define INCHES_TO_STEPS(I)	(STEPS_PER_MM * (I) * INCHES_PER_MM)
#define STEPS_TO_INCHES(S)	((S)/(STEPS_PER_MM * INCHES_PER_MM))


typedef enum { STOP_HERE, REVERSE, ONE_CYCLE } EndstopMode;
typedef enum { CARRIAGE_STOP, CARRIAGE_TRAVEL, CARRIAGE_TRAVEL_REVERSE, PARKED } CarriageMode;
typedef enum { MOVE_DISABLED, MOVE_VIDEO, MOVE_TIMELAPSE } MoveMode;

// timelapse move inputs & parameters
typedef struct {
	int	moveDistance;						// distance to move in each interval
	int	moveInterval;						// delay in sec between moves
	int	moveTarget;							// total number of moves to take
	int	moveCount;							// realtime move count
	int	timerID;								// ID of the current timer
} T_TL_Data;

#endif