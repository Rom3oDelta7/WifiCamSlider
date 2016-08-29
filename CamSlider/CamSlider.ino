/*
   TABS=3


	WiFi Camera Slider Controller
	Controls the stepper motor on the slider as well as providing an HTTP server and AP to enable WiFi control from any browser without a WiFI network host.
	This version supports either a TB6612FNG (H Bridge) or an A4988 (chopper) controller using a WeMos D1 Mini ESP8266 (ESP-12F) devboard.
	
	
	Prior to running this sketch, the HTML file must be loaded into the ESP8266 SPIFFS file system.
	Currently, the instructions for installing and running the uplaod tool can be found here: http://esp8266.github.io/Arduino/versions/2.3.0/doc/filesystem.html

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#define DEBUG				0
//#define TB6612FNG							// TB6612FNG H-bridge controller module
#define A4988									// Stepstick A4988 controller module

#include <AccelStepper.h>					// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include "LED3.h"								// https://github.com/Rom3oDelta7/LED3
#include "SimpleTimer.h"					// http://playground.arduino.cc/Code/SimpleTimer
#include "CamSlider.h"

/*================================= stepper motor interface ==============================

This code was written for an OSM 17HS24-0644S stepper motor
 References:
	http://www.omc-stepperonline.com/nema-17-60ncm85ozin-064a-bipolar-stepper-motor-17hs240644s-p-19.html (product page)
	http://www.omc-stepperonline.com/download/pdf/17HS24-0644S.pdf (datasheet [partially in Chinese])
	http://www.osmtec.com/stepper_motors.htm (OSM stepper motor catalog)
	
TB6612FNG H Bridge controller:
	https://www.sparkfun.com/products/9457
	
Allegro A4988 motor controller
	http://www.electrodragon.com/w/Stepstick_Stepper_Driver_Board_A4988_V2
	
	The AccelStepper library is used for both controllers.
	
	
*/

// ================================= stepper ==============================================

#if defined(TB6612FNG)
#define MOTORA1				14						// WeMos D5
#define MOTORA2				12						// WeMos D6
#define MOTORB1				4						// WeMos D2
#define MOTORB2				5						// WeMos D1
#define STANDBY				16						// WeMos D0
#elif defined(A4988)
#define STEP					16						// WeMos D5
#define DIR						14						// WeMos D0; HIGH == FWD
#define ENABLE					12						// WeMos D6
#endif

#if defined(TB6612FNG)
AccelStepper stepper(AccelStepper::FULL4WIRE, MOTORA1, MOTORA2, MOTORB1, MOTORB2);
#elif defined(A4988)
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);
#endif

// ================================ slider controls =======================================
#if defined(TB6612FNG)
#define LIMIT_MOTOR		0				// motor-side endstop switch pin - WeMos pullup resistor on this pin (D3)
#define LIMIT_END			13				// other endstop switch pin (D7)
#elif defined(A4988)
#define LIMIT_MOTOR		0				// motor-side endstop switch pin - WeMos pullup (D3)
#define LIMIT_END			2				// other endstop switch pin - WeMos pullup (D4)
#endif

#define BOUNCE_DELAY		300			// delay window in msec to ignore pin value fluctuation

volatile	bool				newMove = false;							// true when we need to initiate a new move
volatile bool				clockwise = true;							// true: away, false: towards motor
volatile EndstopMode		endstopAction = STOP_HERE;				// action to take when an endstop is hit
volatile CarriageMode	carriageState = CARRIAGE_PARKED;		// current state of the carriage (for the motion state machine)
volatile bool				debounce = true;							// ISR debounce control flag (init true to avoid triggering on noise at startup)
volatile unsigned long 	currentTime = 0;							// set in loop() so we don't have to call it in the ISR
volatile unsigned long	debounceStart = 0;						// start of debounce window
volatile bool				plannedMoveEnd = false;					// true if calling endOfTravel for a planned move termination

long							targetPosition = 0;						// inches to travel
float							targetSpeed = 0.0;						// speed in steps/second
unsigned long				travelStart = 0;							// start of curent carriage movement
unsigned long				lastRunDuration = 0;						// duration of last movement
int							stepsTaken = 0;							// counts steps actually executed
bool							running = false;							// true only while the carriage is in motion



/* ===================================== LED =================================================

Status colors (solid):
----------------------
White		system initializing
Blue		camera triggering
Purple	Timelapse mode: motors stopped
Cyan		Video mode: motors stopped
Red		Disabled mode
Green		Carriage in motion

Error state colors (flashing):
------------------------------
Red		error opening SPIFFS file system
Yellow	error opening Video BODY HTML file
Purple	error opening Timelapse BODY file
Cyan		error opening Disabled BODY file
Orange	error opening CSS HTML file

*/

// pins
#if defined(TB6612FNG)
#define LED_RED			1												// WeMos TX - 1 & 3 MUST BE DISCONNECTED to use the Serial output (TX & RX)
#define LED_GREEN			3												// WeMos Rx
#define LED_BLUE			2												// WeMos D4
#elif defined(A4988)
#define LED_RED			13												// WeMos D7
#define LED_GREEN			5												// WeMos D1
#define LED_BLUE			4												// WeMos D2
#define CAM_TRIGGER		15												// WeMos D8 (pulldown)
#endif

#define CAM_TRIGGER_DURATION	100									// how long to hold the shutter button down in msec										

LED3			led(LED_RED, LED_GREEN, LED_BLUE, LED3_CATHODE);
SimpleTimer timer;														// for timelapse mode

extern 	MoveMode	sliderMode;											// input enable flag
extern 	TL_Data 	timelapse;											// data for timelapse moves
extern	Home_State homeState;										// saved state for homing moves

extern void setupWiFi(void);
extern void WiFiService(void);


/*
 light the LED to indicate status - THIS FUNCTION DOES NOT RETURN IF FATAL IS TRUE
*/
void statusLED ( const uint32_t color, const bool fatal = false ) {
	if ( fatal ) {
		while ( true ) {
			led.setLED3Color(color);
			delay(250);
			led.setLED3Color(LED3_OFF);
			delay(250);
		}
	} else {
		led.setLED3Color(color);
	}
}

/*
 endstop ISR (used for both endstops and end of planned moves (e.g. shorter distances that do not hit the limit switch))
 set flags & state to be used in loop()
 we clear the debounce flag after the debounce interval expires in loop() if this is called by an interrupt
*/
void endOfTravel ( void ) {
	if ( plannedMoveEnd || !debounce ) {
#if DEBUG > 0
		Serial.println("**** ENDSTOP HIT ****");
#endif
		switch ( endstopAction ) {
		case STOP_HERE:
			carriageState = CARRIAGE_STOP;
			if ( !plannedMoveEnd ) {
				// reverse direction IFF we hit the endstop limit switch
				clockwise = !clockwise;
			}
			break;
			
		case REVERSE:
			// reverse without stopping
			carriageState = CARRIAGE_TRAVEL_REVERSE;
			clockwise = !clockwise;
			newMove = true;										// execute the same move parameters in the opporsite direction
			break;
			
		case ONE_CYCLE:
			// return once, no stopping
			carriageState = CARRIAGE_TRAVEL_REVERSE;
			endstopAction = STOP_HERE;							// stop next time
			clockwise = !clockwise;
			newMove = true;
			break;
		
		default:
			break;
		}
		if ( !plannedMoveEnd ) {
			// set up debounce window (see loop())
			debounce = true;
			debounceStart = currentTime;			
		} else {
			plannedMoveEnd = false;
		}
	}
}

/*
 SETUP
*/
void setup ( void ) {
#if DEBUG > 0
	Serial.begin(19200);
	Serial.println("Initializing ...");
#endif
	
	// housekeeping
	statusLED(LED3_WHITE);
#if defined(TB6612FNG)
#if DEBUG > 0
	Serial.println(">> TB6612FNG controller configured <<");
#endif
	pinMode(MOTORA1, OUTPUT);
	pinMode(MOTORA2, OUTPUT);
	pinMode(MOTORB1, OUTPUT);
	pinMode(MOTORB2, OUTPUT);
	pinMode(STANDBY, OUTPUT);
	pinMode(LIMIT_MOTOR, INPUT);								// WeMos pullup resistor
	pinMode(LIMIT_END, INPUT_PULLUP);
	
	stepper.setEnablePin(STANDBY);							// set LOW to standby - internal pulldown in TB6612FNG

#elif defined(A4988)
#if DEBUG > 0
	Serial.println(">> A4988 controller configured <<");
#endif
	pinMode(STEP, OUTPUT);
	pinMode(DIR, OUTPUT);
	pinMode(ENABLE, OUTPUT);
	pinMode(LIMIT_MOTOR, INPUT);								// WeMos module pullup resistor on both pins
	pinMode(LIMIT_END, INPUT);
	pinMode(CAM_TRIGGER, OUTPUT);								// WeMos pulldown on this pin
	digitalWrite(CAM_TRIGGER, LOW);
	
	stepper.setEnablePin(ENABLE);								// set LOW to standby - internal pulldown in TB6612FNG
	stepper.setPinsInverted(false, false, true);			// inverted ENABLE pin on Allegro A4988
#endif
	
	stepper.setMaxSpeed(HS24_MAX_SPEED);					// max steps/sec 
	stepper.disableOutputs();									// don't energize the motors or enable controller until user initiates movement
	
	attachInterrupt(digitalPinToInterrupt(LIMIT_MOTOR), endOfTravel, FALLING);
	attachInterrupt(digitalPinToInterrupt(LIMIT_END), endOfTravel, FALLING);
	
	setupWiFi();
	// close the debounce window to stabilize initialization
	debounce = true;
	debounceStart = millis();
}


#if defined(A4988)
/*
 fire the camera shutter
*/
void triggerShutter ( void ) {
	led.setLED3Color(LED3_BLUE);								// color will be reset when move starts
	digitalWrite(CAM_TRIGGER, HIGH);
	delay(CAM_TRIGGER_DURATION);						
	digitalWrite(CAM_TRIGGER, LOW);
}
#elif defined(TB6612FNG)
void triggerShutter(void){}
#endif

/*
 small FSM for implementing a set of moves for timelapse photography
 sequence is:
    trigger the shutter
	 wait for selected delay time, moving at the end of the window
	 wait for the move to end
	 loop
	 
	 First 2 states (initial, end of wait) are handled here. The last is handled in loop/CARRIAGE_STOP
 
 not a real interrupt, so no need for volatile variables
 move parameters have already been verified
 
*/
void timelapseMove ( void ) {
	
	if ( timelapse.enabled && ((timelapse.totalImages - timelapse.imageCount) > 0) ) {
		if ( !timelapse.waitToMove ) {
			// trigger & delay setup
			delay(CARR_SETTLE_TIME * 1000);									// settling time after carriage move
			triggerShutter();
			if ( ++timelapse.imageCount < timelapse.totalImages ) {
				 /*
				  sequence is : travel move -> stabilization delay -> fire trigger
				  so the next call to timelapseMove is the total interval - stabilization delay - shutter delay
				*/
				timelapse.waitToMove = true;
				int moveTime = (int)((INCHES_TO_STEPS(timelapse.moveDistance) / HS24_MAX_SPEED) * 1000.0);				// inches per step / steps per second = seconds
				int timerDelay = ((timelapse.moveInterval - CARR_SETTLE_TIME) * 1000) - CAM_TRIGGER_DURATION;
				if ( timerDelay < moveTime ) {
					/*
					 min delay is the time it takes to move the carriage
					 input validation should prevent this, but just in case ...
					*/
					timerDelay = moveTime;
				}
				timer.setTimeout(timerDelay, timelapseMove);
#if DEBUG >= 2
				Serial.println(String("Move time is: ") + String(moveTime));
				Serial.println(String("NET move time: ") + String((timelapse.moveInterval - (int)ceil(moveTime)) * 1000));
#endif
			} else {
				// end of sequence
				timelapse.enabled = false;
			}
		} else {
			/*
			 inter-frame delay has expired, so set up the move
			 when the carriage stops, then complete the move sequence (loop():CARRIAGE_STOP)
			*/
#if DEBUG > 2
			Serial.println("Wait complete. Initiating T/L move");
#endif
			timelapse.waitToMove = false;
			timelapse.waitForStop = true;
			targetPosition = (long)INCHES_TO_STEPS(timelapse.moveDistance);
			targetSpeed = HS24_MAX_SPEED;
			newMove = true;
		}
	} 
}

/*
 LOOP
*/
void loop ( void ) {
	
	yield();
	WiFiService();

#if DEBUG >= 3
	// manual inputs for debugging - note that motor speed will be significantly slower if debug statements are being output
	static bool askForInput = true;
	
	if ( askForInput ) {
		Serial.print("*** INPUT DISTANCE IN INCHES and ELAPSED TIME: ");
		askForInput = false;
	}
	if ( Serial.available() ) {
		int inches, elapsed;
		
		inches = Serial.parseInt();
		elapsed = Serial.parseInt();

		if ( (inches > 0) && (elapsed > 0) ) {
			targetPosition = (long)INCHES_TO_STEPS(inches);
			targetSpeed = (float)(targetPosition / elapsed);							// steps per second
			newMove = true;
			askForInput = true;
			Serial.println(String("Target Position: ") + String(targetPosition) + String(" steps at ") + String(targetSpeed) + String(" steps/sec"));
		}
		Serial.flush();
	}
#endif

#if DEBUG >= 3
	static int counter = 0;
	if ( counter++ > 500 ) {
		Serial.println(String("Remaining steps: ") + String(stepper.distanceToGo()) + String(" carriage state: ") + String(carriageState));
		Serial.println(String("\tPOSITIONS: current: ") + String(stepper.currentPosition()) + String(" target: ") + String(stepper.targetPosition()));
		counter = 0;
	}
#endif
	/*
	  motion state machine - move flag always set OUTSIDE of this FSM
	  NOTE: becuase of the WiFi interface, it is essential to use the non-blocking stepper library calls for movement
	*/
	switch ( carriageState ) {
	case CARRIAGE_TRAVEL:
		/*
		 when moving CCW, the position will increment negatively from 0 (CW = positive)
		 however, distanceToGo in CCW rotation will start at target and INCREASE (in the negative direction)
		 ==> we need to check the absolute value of position rather than relying on distanceToGo() which
		 always increases positively regardless of direction (since it is subtracting a negative number in CCW rotation)
		*/
		if ( abs(stepper.currentPosition()) < targetPosition ) {
			// constant speed - no acceleration
			if ( stepper.runSpeed() ) {
				++stepsTaken;
			}								
		} else if ( stepsTaken == targetPosition ) {
			// target reached without hitting the endstop, so simulate it to initiate next step (if any)
			plannedMoveEnd = true;					// set when we did NOT hit the limit switch to get here
			endOfTravel();
		}
		break;
		
	case CARRIAGE_STOP:
#if DEBUG >= 1
			Serial.println(String("*** Traveled ") + String(targetPosition) + String(" steps in ") + String((float)((millis() - travelStart)/1000.0)) + String(" sec"));
#endif
		stepper.stop();
		stepper.runSpeedToPosition();				// a necessary part of the stop mechanism
		carriageState = CARRIAGE_PARKED;			// only place this is set other than initial condition
		stepper.disableOutputs();
		running = false;
		if ( travelStart ) {
			/*
			 capture elapsed time
			 almost never zero, but could be  if limit switch is pressed while not moving, for example
			*/
			lastRunDuration = millis() - travelStart;
			travelStart = 0;
		}
		if ( timelapse.enabled && timelapse.waitForStop ) {
			// end of a move within a timelapse sequence
			timelapse.waitForStop = false;
			timelapseMove();
		}
		if ( homeState.homing ) {
			// restore previous state at end of a homing move, otherwise UI and stepper params could conflict
			homeState.homing = false;
			targetPosition = homeState.lastTargetPosition;
			targetSpeed = homeState.lastTargetSpeed;
			endstopAction = homeState.lastEndstopState;
		}
		break;
		
	case CARRIAGE_TRAVEL_REVERSE:
		// momentarily stop the motion in the current direction - move flag will be set in the ISR so new (opposite) movement will be initiaited below
		stepper.stop();
		stepper.runSpeedToPosition();
		break;
		
	case CARRIAGE_PARKED:
		// set LED color to match mode button on user interface
		switch ( sliderMode ) {
		case MOVE_VIDEO:
			statusLED(LED3_CYAN);
			break;
			
		case MOVE_TIMELAPSE:
			statusLED(LED3_PURPLE);
			break;
			
		case MOVE_DISABLED:
		default:
			statusLED(LED3_RED);
			break;
		}
		// FALLTHRU
	default:
			break;
	}
	
	// check if it is time to close the debounce window
	currentTime = millis();
	if ( debounce && ((currentTime - debounceStart) > BOUNCE_DELAY) ) {
		debounce = false;
	}
	
	
	if ( newMove ) {
		// initiate a new move using current settings
#if DEBUG >= 1
		Serial.println(String(">>> Move to ") + String(targetPosition) + String(" at speed ") + String(targetSpeed) + String( " direction ") + String(clockwise));
#endif
		stepper.setCurrentPosition(0);
		stepper.moveTo(targetPosition);
		stepper.setSpeed(clockwise ? targetSpeed : -targetSpeed);
		if ( carriageState == CARRIAGE_PARKED ) {
			// enable the motor & controller only if it had been turned off
			stepper.enableOutputs();
		}
		carriageState = CARRIAGE_TRAVEL;
		travelStart = millis();
		running = true;
		stepsTaken = 0;
		statusLED(LED3_GREEN);
		newMove = false;
		if ( (digitalRead(LIMIT_MOTOR) == LOW) || (digitalRead(LIMIT_END) == LOW) ) {
			//ignore spurrious limit switch triggers when moving off the switch by closing the debounce window at start of the move 
			debounce = true;
			debounceStart = millis();
		}
	}
	timer.run();
 }
