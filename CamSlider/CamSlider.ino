/*
   TABS=3

	WiFi Camera Slider Controller
	Controls the stepper motor on the slider as well as providing an HTTP server and AP to enable WiFi control from any browser without a WiFI network host.
	This version uses the TB6612FNG controller and a WeMos D1 Mini ESP8266 (ESP-12F) devboard.
	
	References:
		Stepper library: http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
	
	Prior to running this sketch, the HTML file must be loaded into the ESP8266 SPIFFS file system.
	Currently, the instructions for installing and running the uplaod tool can be found here: http://esp8266.github.io/Arduino/versions/2.3.0/doc/filesystem.html

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#define DEBUG				2

#include <AccelStepper.h>

/*================================= stepper motor interface ==============================

This code was written for a 17HS24-0644S stepper motor
<fill in details here>
*/

#define MOTORA1			14
#define MOTORA2			12
#define MOTORB1			4
#define MOTORB2			5	
#define STANDBY			16

#define STEPS_PER_REV	200
#define PULLEY_TEETH		20
#define BELT_PITCH		2
#define STEPS_PER_MM		(STEPS_PER_REV / (BELT_PITCH * PULLEY_TEETH))
#define INCH_TO_MM		25.4

AccelStepper stepper(AccelStepper::FULL4WIRE, MOTORA1, MOTORA2, MOTORB1, MOTORB2);

// ================================ slider controls =======================================
#define LIMIT_MOTOR		0				// motor-side endstop switch pin - pullup resistor in this pin
#define LIMIT_END			13				// other endstop switch pin - use internal pullup

#define BOUNCE_DELAY		300			// delay window in msec to ignore pin value fluctuation

typedef enum { STOP_HERE, REVERSE, ONE_CYCLE } EndstopMode;
typedef enum { CARRIAGE_STOP, CARRIAGE_TRAVEL, CARRIAGE_TRAVEL_REVERSE, PARKED } CarriageMode;

volatile	bool				newMove = false;							// true when we need to initiate a new move
volatile bool				clockwise = true;
volatile EndstopMode		endstopAction = STOP_HERE;
volatile CarriageMode	carriageState = PARKED;
volatile bool				debounce = false;
volatile unsigned long 	currentTime = 0;							// set in loop() so we don't have to call it in the ISR
volatile unsigned long	debounceStart = 0;						// start of debounce window

long							targetPosition = 0;						// inches to travel
float							targetSpeed = 0.0;						// speed in steps/second


/*
 endstop ISR (used for both endstops)
 set flags & state to be used in loop()
 we clear the debounce flag after the debounce interval in loop()
*/
void endstopHit ( void ) {
	if ( !debounce ) {
#if DEBUG > 0
		Serial.println("**** ENDSTOP HIT ****");
#endif
		switch ( endstopAction ) {
		case STOP_HERE:
			// stop here
			carriageState = CARRIAGE_STOP;
			clockwise = !clockwise;							// next move user initiaites is opposite direction
			break;
			
		case REVERSE:
			// reverse direction
			carriageState = CARRIAGE_TRAVEL_REVERSE;
			clockwise = !clockwise;
			newMove = true;
			break;
			
		case ONE_CYCLE:
			// return once
			carriageState = CARRIAGE_TRAVEL_REVERSE;
			endstopAction = STOP_HERE;							// stop next time
			clockwise = !clockwise;
			newMove = true;
			break;
		
		default:
			break;
		}
		debounce = true;
		debounceStart = currentTime;
	}

}

void setup ( void ) {
#if DEBUG > 0
	Serial.begin(19200);
	Serial.println("Initializing ...");
#endif
	// housekeeping
	pinMode(MOTORA1, OUTPUT);
	pinMode(MOTORA2, OUTPUT);
	pinMode(MOTORB1, OUTPUT);
	pinMode(MOTORB2, OUTPUT);
	pinMode(STANDBY, OUTPUT);
	pinMode(LIMIT_MOTOR, INPUT);								// WeMos internal pullup resistor
	pinMode(LIMIT_END, INPUT_PULLUP);
	
	stepper.setMaxSpeed(592.0);								// max steps/sec - see http://www.daycounter.com/Calculators/Stepper-Motor-Calculator.phtml
	//stepper.setPinsInverted (false, false, true);		// inverted SYBY pin on Alegro A498
	//stepper.setEnablePin(STANDBY);							// for the Allegro A498 driver only
	stepper.disableOutputs();									// don't energize the motors until user initiates movement
	digitalWrite(STANDBY, LOW);								// set LOW to standby - internal pulldown in TB6612FNG
	
	
	attachInterrupt(digitalPinToInterrupt(LIMIT_MOTOR), endstopHit, FALLING);
	attachInterrupt(digitalPinToInterrupt(LIMIT_END), endstopHit, FALLING);
}


/*
  simple pin debounce function
*/
bool readDebounce ( const uint8_t pin, const uint8_t state = LOW ) {
	static 	bool 				windowClosed = false;
	static 	unsigned long 	debounceTime = 0;
	bool							returnValue = false;
	
	if ( windowClosed ) {
		// see if it is time to stop ignoring fluctuations
		if ( (millis() - debounceTime) > BOUNCE_DELAY ) {
			windowClosed = false;
		}
	}
	if ( !windowClosed ) {
		if ( digitalRead(pin) == state ) {
			returnValue = true;
			windowClosed = true;
		}
	}
	return returnValue;
}
 
void loop ( void ) {
	static long	travelStart = 0;
	
	yield();

#if DEBUG >= 2
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
			targetPosition = (long)(STEPS_PER_MM * inches * INCH_TO_MM);			// # of steps to move
			targetSpeed = (float)(targetPosition / elapsed);							// steps per second
			newMove = true;
			askForInput = true;
			Serial.println(String("Target Position: ") + String(targetPosition) + String(" steps at ") + String(targetSpeed) + String(" steps/sec"));
		}
		Serial.flush();
	}
#endif

	// check if the ISR debounce window is now closed
	currentTime = millis();
	if ( debounce && ((currentTime - debounceStart) > BOUNCE_DELAY) ) {
		debounce = false;
	}
	
	// movement state machine
#if DEBUG >= 3
	static int counter = 0;
	if ( counter++ > 500 ) {
		Serial.println(String("Reamaining steps: ") + String(stepper.distanceToGo()) + String(" carriage state: ") + String(carriageState));
		Serial.println(String("\tPOSITIONS: current: ") + String(stepper.currentPosition()) + String(" target: ") + String(stepper.targetPosition()));
		counter = 0;
	}
#endif
	switch ( carriageState ) {
	case CARRIAGE_TRAVEL:
		/*
		 when moving CCW, the position will increment negatively from 0 (CW = positive)
		 however, distanceToGo in CCW rotation will start at target and INCREASE ==> we need to check the 
		 absolute value of position rather than relying on distanceToGo() which always increases positively regardless of direction
		 (since it is subtracting a negative number in CCW rotation)
		*/
		if ( abs(stepper.currentPosition()) < targetPosition ) {
			stepper.runSpeed();								// constant speed - no acceleration
		} else {
			carriageState = CARRIAGE_STOP;
		}
		break;
		
	case CARRIAGE_STOP:
#if DEBUG >= 1
			// display this in phone interface   ZZZ
			Serial.println(String("*** Traveled ") + String(targetPosition) + String(" steps in ") + String((float)(millis() - travelStart)/1000.0) + String(" sec"));
#endif
		stepper.stop();
		stepper.runSpeedToPosition();
		carriageState = PARKED;
		stepper.disableOutputs();
		digitalWrite(STANDBY, LOW);
		break;
		
	case CARRIAGE_TRAVEL_REVERSE:
		// stop before we initiate the opposite move (flag will be true below)
		stepper.stop();
		stepper.runSpeedToPosition();
		carriageState = CARRIAGE_TRAVEL;
		break;
		
	case PARKED:
	default:
			break;
	}
	
	if ( newMove ) {
		// initiate a new move using current settings
#if DEBUG >= 1
		Serial.println(String(">>> Move to ") + String(targetPosition) + String(" at speed ") + String(targetSpeed) + String( " direction ") + String(clockwise));
#endif
		stepper.setCurrentPosition(0);
		stepper.moveTo(targetPosition);
		stepper.setSpeed(clockwise ? targetSpeed : (targetSpeed * -1.0));
		stepper.enableOutputs();
		digitalWrite(STANDBY, HIGH);
		carriageState = CARRIAGE_TRAVEL;
		travelStart = millis();
		newMove = false;
	}
 }
