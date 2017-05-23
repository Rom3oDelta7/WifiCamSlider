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
#define STEPS_PER_REV		   200
#define PULLEY_TEETH			   20
#define BELT_PITCH			   2						// 2 MM
#define INCHES_PER_MM		   25.4
#define HS24_MAX_SPEED		   592.0					// in steps/sec set using calculator at http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml

#define MAX_TRAVEL_DISTANCE	120				   // maximum possible travel distance (inches)
#define MAX_TRAVEL_TIME			10800				   // maximum possible travel duration (sec)
#define MAX_IMAGES				2000				   // maximum number of images (timelapse mode)
#define CARR_SETTLE_MSEC		2250 				   // delay to stabilize carriage before triggering shutter
#define CARR_SETTLE_SEC			3					   // above delay in seconds

#define STEPS_PER_MM			   (STEPS_PER_REV / (BELT_PITCH * PULLEY_TEETH))
#define INCHES_TO_STEPS(I)	   (STEPS_PER_MM * (I) * INCHES_PER_MM)
#define STEPS_TO_INCHES(S)	   ((S)/(STEPS_PER_MM * INCHES_PER_MM))

#define EEPROMSIZE            128
#define CRED_ADDR             0


typedef enum:uint8_t { STOP_HERE, REVERSE, ONE_CYCLE } EndstopMode;
typedef enum:uint8_t { CARRIAGE_STOP, CARRIAGE_TRAVEL, CARRIAGE_TRAVEL_REVERSE, CARRIAGE_PARKED } CarriageMode;
typedef enum:uint8_t { MOVE_NOT_SET, MOVE_DISABLED, MOVE_VIDEO, MOVE_TIMELAPSE } MoveMode;
typedef enum:uint8_t { S_SHUTTER, S_MOVE, S_DELAY } TL_State;

// state data for carriage homing move
typedef struct {
   bool				homing;					         // indicates a home move action
   EndstopMode		lastEndstopState;		         // saved endstop state
   long				lastTargetPosition;	         // saved stepper position target
   float				lastTargetSpeed;		         // saved target speed
} Home_State;

// timelapse move inputs, parameters, state
typedef struct {
   bool		enabled;								      // enable timelapse movement
   int		totalDistance;						      // total distance traveled for the timelapse sequence (user input)
   int		totalDuration;						      // total timelapse duration (user input)
   int		totalImages;						      // total number of images to take (user input)
   int		moveDistance;						      // distance to move in each interval
   int		moveInterval;						      // delay in sec between moves
   int		imageCount;							      // realtime shutter activation count
   uint32_t	moveStartTime;						      // when move was initiated
   TL_State	state;								      // state for FSM
} TL_Data;

#endif