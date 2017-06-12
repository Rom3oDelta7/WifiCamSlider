/*
   TABS=3
   WiFi Camera Slider Controller
   https://github.com/Rom3oDelta7/WifiCamSlider

   Controls the stepper motor on the slider utilizing an HTTP server (AP or STA mode) to enable WiFi control from any browser with or without without a WiFI network host.
   This version supports an A4988 (chopper) controller using a WeMos D1 Mini ESP8266 (ESP-12F) devboard.
   
   
   Prior to running this sketch, the HTML file must be loaded into the ESP8266 SPIFFS file system.
   Currently, the instructions for installing and running the uplaod tool can be found here: http://esp8266.github.io/Arduino/versions/2.3.0/doc/filesystem.html

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <LEDManager.h>
#include <DebugLib.h>
#include <ESP8266WebServer.h>
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <ESP8266WiFiType.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFi.h>

#define DEBUG    1                     // selective debug setting
#define DEBUG_INFO
#define DEBUG_LOG

#include <AccelStepper.h>					// http://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
#include <LEDManager.h>					   // https://github.com/Rom3oDelta7/LEDManager
#include <SimpleTimer.h>					// http://playground.arduino.cc/Code/SimpleTimer
#include "CamSlider.h"
#include "DebugLib.h"

/*================================= stepper motor interface ==============================

This code was written for an OSM 17HS24-0644S stepper motor
 References:
   http://www.omc-stepperonline.com/nema-17-60ncm85ozin-064a-bipolar-stepper-motor-17hs240644s-p-19.html (product page)
   http://www.omc-stepperonline.com/download/pdf/17HS24-0644S.pdf (datasheet [partially in Chinese])
   http://www.osmtec.com/stepper_motors.htm (OSM stepper motor catalog)
   
Allegro A4988 motor controller
   http://www.electrodragon.com/w/Stepstick_Stepper_Driver_Board_A4988_V2
*/

// ================================= stepper ==============================================

#define STEP				D0						                  // ESP 16
#define DIR					D5						                  // ESP 14; HIGH == FWD
#define ENABLE				D6						                  // ESP 12

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);


// ================================ slider controls =======================================

#define LIMIT_MOTOR		D3				                        // motor-side endstop switch pin - WeMos pullup (ESP 0)
#define LIMIT_END			D4				                        // other endstop switch pin - WeMos pullup (ESP 2)


#define BOUNCE_DELAY		300			                        // delay window in msec to ignore pin value fluctuation

volatile	bool				newMove = false;							// true when we need to initiate a new move
volatile bool				clockwise = true;							// true: away, false: towards motor
volatile EndstopMode		endstopAction = STOP_HERE;				// action to take when an endstop is hit
volatile CarriageMode	carriageState = CARRIAGE_PARKED;		// current state of the carriage (for the motion state machine)
volatile bool				debounce = true;							// ISR debounce control flag (init true to avoid triggering on noise at startup)
volatile unsigned long 	currentTime = 0;							// set in loop() so we don't have to call it in the ISR
volatile unsigned long	debounceStart = 0;						// start of debounce window
volatile bool				plannedMoveEnd = false;					// true if calling endOfTravel for a planned move termination

uint32_t                maxDistance = MAX_TRAVEL_DISTANCE;  // maximum slider length
long							targetPosition = 0;						// inches to travel
float							targetSpeed = 0.0;						// speed in steps/second
unsigned long				travelStart = 0;							// start of curent carriage movement
unsigned long				lastRunDuration = 0;						// duration of last movement
int							stepsTaken = 0;							// counts steps actually executed
bool							running = false;							// true only while the carriage is in motion



/* ===================================== LED =================================================

Status colors:
----------------------
White		              system initializing
Blue		              camera triggering
Purple (flashing)	     Timelapse mode: motors stopped
Cyan  (flashing)	     Video mode: motors stopped
Red  (flashing)		  Disabled mode
Green		              Carriage in motion

Error state colors (rapid flashing):
------------------------------
Red		              error opening SPIFFS file system
Yellow	              error opening Video BODY HTML file
Purple	              error opening Timelapse BODY file
Cyan		              error opening Disabled BODY file
Orange	              error opening CSS HTML file

Setup Colors
------------
Red/Green alternating  WiFi configuration required

*/

// pins
#define LED_RED			D7												// ESP 13
#define LED_GREEN			D1												// ESP 5
#define LED_BLUE			D2												// ESP 4

#define CAM_TRIGGER		D8												// ESP 15 (pulldown)

#define CAM_TRIGGER_DURATION	100									// how long to hold the shutter button down in msec										

RGBLED	   led(LED_RED, LED_GREEN, LED_BLUE);
SimpleTimer timer;														// for timelapse mode

extern 	MoveMode	sliderMode;											// input enable flag
extern 	TL_Data 	timelapse;											// data for timelapse moves
extern	Home_State homeState;										// saved state for homing moves
extern   bool calibrating;                                  // calibration state flag
extern   bool userConnected;                                // true when user has connected via a device

extern void setupWiFi(void);
extern void WiFiService(void);


/*
 indicate error status and LED and loop forever ...
*/
void fatalError ( const LEDColor color ) {
   led.setColor(color);
   led.setState(LEDState::BLINK_ON, 125);
   while ( true ) {
      // avoid WDTs
      delay(500);
      yield();
   }
}

/*
 endstop ISR (used for both endstops and end of planned moves (e.g. shorter distances that do not hit the limit switch))
 for planned moves, this fcn is called directly.
 set flags & state to be used in loop()
 we clear the debounce flag after the debounce interval expires in loop() if this is called by an interrupt
*/
void endOfTravel ( void ) {
   led.setState(LEDState::OFF);                       // (briefly) turn off move indicator
   if ( plannedMoveEnd ) {
      // all planned moves stop the carriage
      INFO(F("**** MOVE END ****"), "");
      carriageState = CARRIAGE_STOP;
      plannedMoveEnd = false;
   } else if ( !debounce ) {
      INFO(F("**** ENDSTOP HIT ****"), "");

      // limit switch triggered
      switch ( endstopAction ) {
      case STOP_HERE:
         carriageState = CARRIAGE_STOP;
         clockwise = !clockwise;
         break;
         
      case REVERSE:
         // reverse without stopping
         // ZZZ - better to continue with the current move rather than initiatiating a new one ??? ZZZ
         carriageState = CARRIAGE_TRAVEL_REVERSE;
         clockwise = !clockwise;
         newMove = true;										// execute the current move parameters in the opposite direction
         break;
         
      case ONE_CYCLE:
         // return once, no stopping
         carriageState = CARRIAGE_TRAVEL_REVERSE;
         endstopAction = STOP_HERE;							// stop next time
         clockwise = !clockwise;
         newMove = true;                              // execute the current move parameters in the opposite direction                        
         if ( calibrating ) {
            /*
             we just ran out to the end of the slider after a homing move
             capture the actual distance moved
             note that homing flag is still set, so the move initiated above will complete the homing move
            */
            maxDistance = STEPS_TO_INCHES(stepsTaken);
            calibrating = false;
         }
         break;
      
      default:
         break;
      }
      
      // set up debounce window (see loop())
      debounce = true;
      debounceStart = currentTime;			
   }
}

/*
 SETUP
*/
void setup ( void ) {
#if DEBUG > 0
   Serial.begin(115200);
   INFO(F("Initializing ..."), ".");
#endif
   
   // housekeeping
   led.setColor(LEDColor::WHITE);                    // white => initialization
   led.setState(LEDState::ON);
   EEPROM.begin(EEPROMSIZE);                         // for shadow copy of WiFi credentials

   pinMode(STEP, OUTPUT);
   pinMode(DIR, OUTPUT);
   pinMode(ENABLE, OUTPUT);
   pinMode(LIMIT_MOTOR, INPUT);								// WeMos module pullup resistor on both pins
   pinMode(LIMIT_END, INPUT);
   pinMode(CAM_TRIGGER, OUTPUT);								// WeMos pulldown on this pin
   digitalWrite(CAM_TRIGGER, LOW);
   
   stepper.setEnablePin(ENABLE);								// set LOW to standby - internal pulldown in TB6612FNG
   stepper.setPinsInverted(false, false, true);			// inverted ENABLE pin on Allegro A4988
   
   stepper.setMaxSpeed(HS24_MAX_SPEED);					// max steps/sec 
   stepper.disableOutputs();									// don't energize the motors or enable controller until user initiates movement
   
   attachInterrupt(digitalPinToInterrupt(LIMIT_MOTOR), endOfTravel, FALLING);
   attachInterrupt(digitalPinToInterrupt(LIMIT_END), endOfTravel, FALLING);
   
   setupWiFi();

   // close the debounce window to stabilize initialization
   debounce = true;
   debounceStart = millis();
}

/*
 fire the camera shutter
*/
void triggerShutter ( void ) {
   led.setColor(LEDColor::BLUE);								// color will be reset when move starts
   led.setState(LEDState::ON);
   digitalWrite(CAM_TRIGGER, HIGH);
   delay(CAM_TRIGGER_DURATION);						
   digitalWrite(CAM_TRIGGER, LOW);
}

/*
 small FSM for implementing a set of moves for timelapse photography
 sequence is:
    trigger the shutter
    pre-move delay
    initiate move
      CARIAGE_STOP state in loop calls this fcn again at end of move seq
    set timeout for remaining (post-move) delay (must be > minimum for carriage settling time)
    loop
 
 not a real interrupt, so no need for volatile variables
 move parameters have already been verified
 
*/
void timelapseMove ( void ) {
   
   if ( timelapse.enabled && (timelapse.imageCount < timelapse.totalImages) ) {
      switch ( timelapse.state ) {
      case S_SHUTTER:
         // fire the shutter then initiate first delay
         triggerShutter();
         
         if ( ++timelapse.imageCount < timelapse.totalImages ) {
            int moveTime = (int)floor((INCHES_TO_STEPS(timelapse.moveDistance) / HS24_MAX_SPEED) * 1000.0);
            int timerDelay = ((timelapse.moveInterval * 1000) - moveTime) / 2;
            timelapse.state = S_MOVE;
            timelapse.moveStartTime = millis();
            timer.setTimeout(timerDelay < 0 ? 0 : timerDelay, timelapseMove);
         } else {
            // final shutter trigger is the end of timelapse sequence
            timelapse.enabled = false;
         }
         break;
         
      case S_MOVE:
         // move the carriage - motion FSM will return to this fcn
         targetPosition = (long)INCHES_TO_STEPS(timelapse.moveDistance);
         targetSpeed = HS24_MAX_SPEED;
         timelapse.state = S_DELAY;
         newMove = true;
         break;
         
      case S_DELAY:
         // move complete; calculate & schedule the delay timeout for remainaing time
         int timerDelay = (timelapse.moveInterval * 1000) - (millis() - timelapse.moveStartTime);
         if (timerDelay < CARR_SETTLE_MSEC ) {
            timerDelay = CARR_SETTLE_MSEC;
         }
#if DEBUG >= 2
         Serial.println(String("Next timer call: ") + String(timerDelay));
#endif
         timer.setTimeout(timerDelay, timelapseMove);
         timelapse.state = S_SHUTTER;
         break;
         
      }
   }
}

/* 
 handle homing and calibration states (called at end of a homing movement)
 */
void handleHoming (void) {
   // if calibration routine is active, then we just homed the carriage, so the next step is to run out to the end of the slider
   if ( calibrating ) {
      // we clear the calibration flag when we hit the endstop
      targetPosition = (long)INCHES_TO_STEPS(MAX_TRAVEL_DISTANCE);
      targetSpeed = HS24_MAX_SPEED;
      endstopAction = ONE_CYCLE;
      clockwise = true;							// away from the motor
      newMove = true;
   } else {
      // restore previous state at end of a homing move, otherwise UI and stepper params could conflict
      homeState.homing = false;
      targetPosition = homeState.lastTargetPosition;
      targetSpeed = homeState.lastTargetSpeed;
      endstopAction = homeState.lastEndstopState;
      newMove = false;                     // handles the case when homing is requested but already on the motor endstop
   }
}

/*
 LOOP
*/
void loop ( void ) {
   static LEDColor lastColor = LEDColor::NONE;

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
       always increases regardless of direction (since it is subtracting a negative number in CCW rotation)
      */
      led.setState(LEDState::ON);            // workaround for LED timing issue where LED may remain off when stae changed from blinking to OFF
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
      if ( timelapse.enabled ) {
         // end of a move within a timelapse sequence - do the next sequence
         timelapseMove();
      }
      // homing & calibration
      if ( homeState.homing ) {
         handleHoming();
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
      case MOVE_NOT_SET:
            break;

      case MOVE_VIDEO:
         // only change color on a state change. Otherwise we are constantly resetting the state & preventing blinking
         if ( (lastColor != LEDColor::CYAN) || (led.getState() == LEDState::OFF) ) {
            led.setColor(LEDColor::CYAN);
            led.setState(LEDState::BLINK_ON);
            lastColor = LEDColor::CYAN;
         }
         break;
         
      case MOVE_TIMELAPSE:
         if ( (lastColor != LEDColor::MAGENTA) || (led.getState() == LEDState::OFF) ) {
            led.setColor(LEDColor::MAGENTA);
            led.setState(LEDState::BLINK_ON);
            lastColor = LEDColor::MAGENTA;
         }
         break;
         
      case MOVE_DISABLED:
      default:
         if ( (lastColor != LEDColor::RED) || (led.getState() == LEDState::OFF) ) {
            led.setColor(LEDColor::RED);
            led.setState(LEDState::BLINK_ON);
            lastColor = LEDColor::RED;
         }
         break;
      }
      // FALLTHRU
   default:
         break;
   }
   //INFO(F("LED state"), led.getState());
   
   // check if it is time to close the debounce window
   currentTime = millis();
   if ( debounce && ((currentTime - debounceStart) > BOUNCE_DELAY) ) {
      debounce = false;
   }
   
   // *********************** MOVE ENGINE **********************************************
   if ( newMove ) {
      // first ensure we are not already on an endstop
      if ( !(((digitalRead(LIMIT_MOTOR) == LOW) && !clockwise) || ((digitalRead(LIMIT_END) == LOW) && clockwise)) ) {
         // initiate a new move using current settings
#if DEBUG >= 1
         Serial.println(String(">>> Move to ") + String(targetPosition) + String(" at speed ") + String(targetSpeed) + String(" direction ") + String(clockwise));
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
         led.setColor(LEDColor::GREEN);
         led.setState(LEDState::ON);
         lastColor = LEDColor::GREEN;                      // must set to force LED change in CARRIAGE_PARKED state at end of move
         newMove = false;
         if ( (digitalRead(LIMIT_MOTOR) == LOW) || (digitalRead(LIMIT_END) == LOW) ) {
            //ignore spurrious limit switch triggers when moving off the switch by closing the debounce window at start of the move 
            debounce = true;
            debounceStart = millis();
         }
      } else if ( homeState.homing ) {
         // if we're homing or calibrating while on the endstop, we still need to handle the homing state changes and/or the calibration runout
         handleHoming();
      } else {
         // only possible movement is the opposite direction, so change it for the user
         clockwise = !clockwise;
      }
   }
   timer.run();
   if ( userConnected ) {
      ArduinoOTA.handle();
   }
 }
