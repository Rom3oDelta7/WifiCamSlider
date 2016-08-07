/*
   TABS=3

	WiFi Camera Slider Controller common definitions

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#ifndef CAMSLIDER_H
#define CAMSLIDER_H

// OSM 17HS24-0644S stepper motor parameters needed for motoin calculations
#define STEPS_PER_REV		200
#define PULLEY_TEETH			20
#define BELT_PITCH			2						// 2 MM
#define INCHES_PER_MM		25.4
#define HS24_MAX_SPEED		592.0					// set using calculator at http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml

#define STEPS_PER_MM			(STEPS_PER_REV / (BELT_PITCH * PULLEY_TEETH))
#define INCHES_TO_STEPS(I)	(STEPS_PER_MM * (I) * INCHES_PER_MM)
#define STEPS_TO_INCHES(S)	((S)/(STEPS_PER_MM * INCHES_PER_MM))


typedef enum { STOP_HERE, REVERSE, ONE_CYCLE } EndstopMode;
typedef enum { CARRIAGE_STOP, CARRIAGE_TRAVEL, CARRIAGE_TRAVEL_REVERSE, PARKED } CarriageMode;

#endif