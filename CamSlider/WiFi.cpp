/*
   TABS=3

   camera slider WiFi control interface
   
   Prior to running this sketch, the HTML file must be loaded into the ESP8266 SPIFFS file system.
   Currently, the instructions for installing and running the uplaod tool can be found here: http://esp8266.github.io/Arduino/versions/2.3.0/doc/filesystem.html

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#define DEBUG			1
#define DEBUG_ERROR
#define DEBUG_INFO
#define DEBUG_LOG

extern "C" {
   /*
     uses the ESP8266 SDK API, which is in C, so we need this extern to prevent gcc from mapping the function names to C++ namespace and thus
     preventing the link editor from finding these API functions
   */
   #include <user_interface.h>
}


#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <FS.h>
#include <LEDManager.h>                           //  download from https://github.com/Rom3oDelta7/LEDManager
#include <WiFiManager.h>                          // [Local Modifications] tapzu Wifi Manager RD7 fork: https://github.com/Rom3oDelta7/WiFiManager
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include "CamSlider.h"
#include "DebugLib.h"

// main sketch externs
extern volatile bool				newMove;					// true when we need to initiate a new move
extern volatile bool				clockwise;
extern volatile EndstopMode	endstopAction;			// action to take when an endstop is hit
extern volatile CarriageMode	carriageState;			// current state of the carriage (for the motion state machine)
extern long							targetPosition;		// inches to travel
extern float						targetSpeed;			// speed in steps/second
extern unsigned long				travelStart;			// start of curent carriage movement
extern unsigned long				lastRunDuration;		// duration of last movement
extern int							stepsTaken;				// counts steps actually executed
extern bool							running;					// true only while the carriage is in motion
extern RGBLED                 led;                 // status status LED 


// inline variable substitution
#define STATE_VAR					"%MODE%"					// mode (video, timelapse, disabled)
#define ENDSTOP_VAR				"%ENDSTOP%"				// endstop state (toggle)
#define DISTANCE_VAR				"%DISTANCE%"			// req distance to travel
#define DURATION_VAR				"%DURATION%"			// req travel time
#define TL_DISTANCE_VAR			"%TL_DISTANCE%"		// timelapse move distance (per move)
#define TL_DURATION_VAR			"%TL_DURATION%"		// interval between timelapse moves
#define TL_IMAGES_VAR			"%TL_IMAGES%"			// number of moves in a timelapse sequence
#define SPEED_VAR					"%SPEED%"				// speed translated form user inputs
#define DIRECTION_VAR			"%DIRECTION%"			// direction of travel (fwd/rev)
#define START_VAR					"%START%"				// running/standby
#define TRAVELED_VAR				"%TRAVELED%"			// distance traveled in last move
#define ELAPSED_VAR				"%ELAPSED%"				// elapsed time of last move
#define MEASURED_VAR				"%MEAS_SPEED%"			// actual speed (vs target)
#define TL_MEAS_DIST_VAR		"%TL_MEAS_DIST%"		// distance so far in this timelapse sequence
#define TL_MOVECOUNT_VAR		"%TL_MOVECOUNT%"		// count of current timelapse moves
#define TL_REMAINING_VAR		"%TL_REMAINING%"		// timelapse moves to go
#define TL_MOVEDIST				"%TL_MOVEDIST%"		// incremental move distance in inches
#define TL_INTERVAL				"%TL_INTERVAL%"		// incremental move interval (including move)
#define TL_COUNT					"%TL_COUNT%"				// current image count

// string objects for (static) filesystem contents
#define VIDEO_BODY_FILE			"/video_body.html"		// html code <body> for video mode
#define TIMELAPSE_BODY_FILE	"/timelapse_body.html"	// html code <body> for timelapse mode
#define DISABLED_BODY_FILE		"/disabled_body.html"	// very short body with just the state mode button
#define CSS_FILE					"/css.html"					// static CSS HTML contents to reduce burden on String objects
#define STRING_MAX				3000							// max String obj length for holding HTML file content
#define STRING_MAX_SHORT		512							// for smaller files

String	videoBodyFile;											// String copy of video body file segment
String	timelapseBodyFile;									// String copy of timelapse body file segment
String	disabledBodyFile;										// String copy of disabled mode body file segment
String	cssFile;													// String copy of CSS file segment

/*
 user actions in HTML request stream
 also includes URI requests that invoke no action but will send the HTML file anyways (always need to reply to client)
 this table is searched to-to-bottom, first match, so ensure that tokens are not substrings of each other or the first one will always be found
*/
#define ACTION_MODE				"MODE_BTN="					// mode button
#define ACTION_ENDSTOP			"ENDSTOP_BTN="				// endstop state change button
#define ACTION_DISTANCE			"DISTANCE="					// input distance to travel
#define ACTION_DURATION			"DURATION="					// input travel duration
#define ACTION_TL_DISTANCE		"TL_DIST="					// input total timelapse travel distance 
#define ACTION_TL_DURATION		"TL_DURN="					// input total timelapse duration in sec
#define ACTION_TL_IMAGES		"TL_IMAGES="				// input total number of images to take
#define ACTION_DIRECTION		"DIRECTION_BTN="			// toggle carriage direction
#define ACTION_START				"START_BTN="				// initiate/stop movement
#define ACTION_REFRESH			"REFRESH_BTN="				// refresh display
#define ACTION_FORGET         "FORGET_BTN="           // clear saved user credentials
#define ACTION_HOME				"HOME_BTN="					// home the carriage
#define ACTION_CALIBRATE      "CALI_BTN="             // calibrate slider length

typedef enum:uint8_t { 
   NULL_ACTION, IGNORE, SLIDER_STATE, ENDSTOP_STATE, SET_DISTANCE, SET_DURATION, SET_TL_DISTANCE,
   SET_TL_DURATION, SET_TL_IMAGES, SET_DIRECTION, START_STATE, HOME_CARRIAGE, CALIBRATE, FORGET, 
} T_Action;

#define ACTION_TABLE_SIZE		16
const struct {	
   char 		action[20];
   T_Action	type;
} actionTable[ACTION_TABLE_SIZE] = {
   {ACTION_MODE, 			SLIDER_STATE},
   {ACTION_ENDSTOP, 		ENDSTOP_STATE},
   {ACTION_DISTANCE,		SET_DISTANCE},
   {ACTION_DURATION,		SET_DURATION},
   {ACTION_TL_DISTANCE,	SET_TL_DISTANCE},
   {ACTION_TL_DURATION,	SET_TL_DURATION},
   {ACTION_TL_IMAGES,	SET_TL_IMAGES},
   {ACTION_DIRECTION,	SET_DIRECTION},
   {ACTION_START,			START_STATE},
   {ACTION_HOME,			HOME_CARRIAGE},
   {ACTION_CALIBRATE,	CALIBRATE},
   {ACTION_FORGET,      FORGET},
   {ACTION_REFRESH,		NULL_ACTION},			// null actions must be at the end so they don't intercept ones above
   {"GET / ",				NULL_ACTION},					
   {"GET /index.html",	NULL_ACTION},
   {"GET /favicon.ico",	IGNORE}					// always comes after another request, so skip it
};

/*
 CSS color substitution variables and color defines
*/
#define MODE_CSS					"%MODE_CSS%"
#define ENDSTOP_CSS				"%ENDSTOP_CSS%"
#define DISTANCE_CSS				"%DISTANCE_CSS%"
#define DURATION_CSS				"%DURATION_CSS%"
#define TL_DISTANCE_CSS			"%TL_DISTANCE_CSS%"
#define TL_DURATION_CSS			"%TL_DURATION_CSS%"
#define TL_IMAGES_CSS			"%TL_IMAGES_CSS%"
#define DIRECTION_CSS			"%DIRECTION_CSS%"
#define START_CSS					"%START_CSS%"

// button background colors
#define CSS_GREEN					"greenbkgd"
#define CSS_RED					"redbkgd"
#define CSS_ORANGE				"orangebkgd"
#define CSS_GREY					"greybkgd"
#define CSS_BLUE					"bluebkgd"
#define CSS_CYAN					"cyanbkgd"
#define CSS_PURPLE				"purplebkgd"
#define CSS_MAGENTA				"magentabkgd"

WiFiServer	server(80);						// web server instance	
WiFiClient	client; 							// client stream
#define     AP_CHANNEL         11      // WiFi channel to use for STA+AP mode

MoveMode sliderMode = MOVE_NOT_SET;	   // current mode

// data for video move mode
struct {
   int	travelDuration;					// holds travel duration in sec until we convert it to speed
   int	travelDistance;					// holds travel distance in inches until we convert it to steps
} video = {0, 0};

// data for timelapse mode
TL_Data timelapse = {false, 0, 0, 0, 0, 0, 0, 0, S_SHUTTER};

// saved state for homing moves
Home_State homeState = { false, STOP_HERE, 0, 0.0 };				// these initial values are not used

bool calibrating = false;                // true when calibrating slider distance


bool userConnected = false;              // true once user is connected

extern void fatalError(const LEDColor color);
extern void timelapseMove(void);

extern uint32_t maxDistance;             // maximum slider travel distance in inches

/*
Create and return a unique WiFi SSID using the ESP8266 WiFi MAC address
Form the SSID as an IP address so the user knows what address to connect to when in AP mode just in case
(Even though the config page comes up automatically without the user having to use a browser)
*/
String createUniqueSSID (void) {
   uint8_t  mac[WL_MAC_ADDR_LENGTH];
   String   uSSID;

   WiFi.softAPmacAddress(mac);
   uSSID = String("WCS ") + String ("10.") + String(mac[WL_MAC_ADDR_LENGTH - 3]) + String(".") + String(mac[WL_MAC_ADDR_LENGTH - 2]) + String(".") + String(mac[WL_MAC_ADDR_LENGTH - 1]);
   INFO(F("Derived SSID"), uSSID);
   return uSSID;
}

/*
Create a unique class A private IP address using the ESP8266 WiFi MAC address
*/
IPAddress createUniqueIP (void) {
   uint8_t   mac[WL_MAC_ADDR_LENGTH];
   IPAddress result;

   WiFi.softAPmacAddress(mac);
   result[0] = 10;
   result[1] = mac[WL_MAC_ADDR_LENGTH - 3];
   result[2] = mac[WL_MAC_ADDR_LENGTH - 2];
   result[3] = mac[WL_MAC_ADDR_LENGTH - 1];
   INFO(F("Derived AP IP"), result);
   return result;
}

/*
Create a unique password, again based on MAC address, but this time using hexadecimal for the octets
Must be 8 characters in length for WiFiManager to accept it
*/
String createUniquePassword (void) {
   uint8_t  mac[WL_MAC_ADDR_LENGTH];
   char     password[10];

   WiFi.softAPmacAddress(mac);
   sprintf(password, "WCS%02x%02x%02x", mac[WL_MAC_ADDR_LENGTH - 3], mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1]);
   INFO(F("Password"), password);
   return String(password);
}

/*
this disables the use of the SSID to broadcast the IP address in station+AP mode and sets STA mode
also enables OTA (over-the-air) firmware updates
*/
void STAMode (void) {
   if (WiFi.getMode() == WIFI_AP_STA) {
      // switch to STA mode
      INFO(F("STA+IP => STA mode and enabling OTA"), ".");
      WiFi.mode(WIFI_STA);
      WiFi.begin();	                      // Ref: https://github.com/esp8266/Arduino/issues/2352

      // set up OTA once we have established STA mode
      ArduinoOTA.setPort(8266);

      // Hostname defaults to esp8266-[ChipID]
      //ArduinoOTA.setHostname("myesp8266");

      // No authentication by default
      // ArduinoOTA.setPassword((const char *)"123");

      ArduinoOTA.onStart([]() {
         LOG(PSTR("OTA Start\n"));
      });
      ArduinoOTA.onEnd([]() {
         LOG(PSTR("\nOTA End. Restarting ...\n"));
         delay(5000);
         ESP.restart();                          // the OTA library calls restart, but we never return, so force it.
      });
      ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
         LOG(PSTR("OTA Progress: %u%%\r"), (progress / (total / 100)));
      });
      ArduinoOTA.onError([](ota_error_t error) {
         LOG(PSTR("OTA Error[%u]: "), error);
         switch (error) {
         case OTA_AUTH_ERROR:
            LOG(PSTR("OTA Auth Failed\n"));
            break;

         case OTA_BEGIN_ERROR:
            LOG(PSTR("OTA Begin Failed\n"));
            break;

         case OTA_CONNECT_ERROR:
            LOG(PSTR("OTA Connect Failed\n"));
            break;

         case OTA_RECEIVE_ERROR:
            LOG(PSTR("OTA Receive Failed\n"));
            break;

         case OTA_END_ERROR:
            LOG(PSTR("OTA End Failed\n"));
            break;

         default:
            LOG(PSTR("OTA Unknown error\n"));
            break;
         }
      });
      ArduinoOTA.begin();
      LOG(PSTR("OTA Ready. IP Address: %s Chip ID %0X\n"), WiFi.localIP().toString().c_str(), ESP.getChipId());
   }
}

void setupWiFi (void) {
   WiFiManager wifiManager;                                     // IP address establishment
   IPAddress   my_IPAddress = createUniqueIP();                 // unique IP address for AP mode to prevent conflicts with multiple devices
   String      my_password = createUniquePassword();

   // first check that there are WiFi networks to possibly connect to
   int netCount = WiFi.scanNetworks();
   bool connectToAP = false;

   if (netCount > 0) {
      // try to connect (saved credentials or manual entry if not) and default to AP mode if this fails
#ifdef DEBUG
      wifiManager.setDebugOutput(true);
#else
      wifiManager.setDebugOutput(false);
#endif

      INFO(F("Network scan count"), netCount);

      wifiManager.setBreakAfterConfig(true);	                       // undocumented function to return if config unsuccessful
      wifiManager.setSaveCredentialsInEEPROM(true, CRED_ADDR);      // [Local mod] forces credentials to be saved in EEPROM also
      wifiManager.setExitButtonLabel("Standalone Mode");            // [Local mod] sets the label on the exit button to clarify the meaning of exiting from the portal

                                                                    //wifiManager.setAPStaticIPConfig(my_IPAddress, my_IPAddress, IPAddress(255, 0, 0, 0));    // use native WiFi class call below instead
      WiFi.softAPConfig(my_IPAddress, my_IPAddress, IPAddress(255, 0, 0, 0));	                   // workaround for callout issue - see above
      WiFi.setAutoConnect(true);

      led.setColor(LEDColor::RED, LEDColor::GREEN);                 // indicate setup mode
      led.setState(LEDState::ALTERNATE, 250);
      if (wifiManager.autoConnect(createUniqueSSID().c_str(), my_password.c_str())) {
         // we are connected as a client and the ESP is in STA mode Ref: https://github.com/esp8266/Arduino/issues/2352
         INFO(F("Connected (STA+IP; local WiFi)"), WiFi.localIP().toString());
         led.setColor(LEDColor::BLUE);                   // indicates STA mode
         led.setState(LEDState::ON);


         /*

         We also need to know the address of the client as it will be running our HTTP server
         The trick is to switch to mixed mode and broadcast the local IP address in the AP SSID name.
         The user can thus find the local address by looking at the scanned SSIDs on their device

         Keep this on until the status is disabled using the HTML "Forget ID" button *** TODO *** needs a better name

         */

         WiFi.mode(WIFI_AP_STA);

         String ssid = String("WCS: ") + WiFi.localIP().toString();
         INFO(F("Local IP as SSID"), ssid);
         WiFi.softAP(ssid.c_str(), "1nfc^Dh3kAz");                // password only to prevent people from connecting by mistake; channel will be the same as STA mode
         WiFi.softAPConfig(my_IPAddress, my_IPAddress, IPAddress(255, 0, 0, 0));

         //WiFi.reconnect();                                        // supposedly required, but does not work if this is called

#if DEBUG >= 4
         Serial.println("AP_STA Diag:");
         WiFi.printDiag(Serial);
#endif

#if DEBUG >= 2
         WiFi.printDiag(Serial);
#endif
      } else {
         /*
         we get here if the credentials on the setup page are incorrect, blank, or the "Exit" button was used
         */
         INFO(F("Did not connect to local WiFi"), ".");
         connectToAP = true;
      }
   } else {
      INFO(F("No WiFI networks"), ".");
      connectToAP = true;
   }

   if (connectToAP) {
      // use AP mode	 - WiFiManager leaves the ESP in AP+STA mode at this point
      INFO(F("Using AP Mode"), my_IPAddress.toString());
      led.setColor(LEDColor::ORANGE);                   // indicates AP mode
      led.setState(LEDState::ON);

      //WiFi.softAP(createUniqueSSID().c_str(), my_password.c_str(), AP_CHANNEL);
      //WiFi.softAPConfig(my_IPAddress, my_IPAddress, IPAddress(255, 0, 0, 0));
      WiFi.mode(WIFI_AP);

#if DEBUG >= 3
      Serial.println(F("AP Diag:"));
      WiFi.printDiag(Serial);
#endif
   }

   // start the server & init the SPIFFS file system
   server.begin();
   if (!SPIFFS.begin()) {
      ERROR(F("Cannot open SPIFFS file system."), ".");
      fatalError(LEDColor::RED);
   }

#if DEBUG >= 2
   Dir dir = SPIFFS.openDir("/");
   Serial.println("SPIFFS directory contents:");
   while (dir.next()) {
      Serial.print(dir.fileName());
      File f = dir.openFile("r");
      Serial.print(": Size: ");
      Serial.println(f.size());
   }
#endif	

   // open the BODY HTML files on the on-board FS and read it into a String (note that this string is never modified)
   File serverFile = SPIFFS.open(VIDEO_BODY_FILE, "r");
   videoBodyFile.reserve(STRING_MAX);
   if (serverFile) {
      if (serverFile.available()) {
         videoBodyFile = serverFile.readString();
      }
      serverFile.close();;
   } else {
      ERROR(F("error opening"), F("Video BODY file"));
      fatalError(LEDColor::MAGENTA);
   }

   serverFile = SPIFFS.open(TIMELAPSE_BODY_FILE, "r");
   timelapseBodyFile.reserve(STRING_MAX);
   if (serverFile) {
      if (serverFile.available()) {
         timelapseBodyFile = serverFile.readString();
      }
      serverFile.close();;
   } else {
      ERROR(F("error opening"), F("Timelapse BODY file"));
      fatalError(LEDColor::PURPLE);
   }

   serverFile = SPIFFS.open(DISABLED_BODY_FILE, "r");
   disabledBodyFile.reserve(STRING_MAX_SHORT);
   if (serverFile) {
      if (serverFile.available()) {
         disabledBodyFile = serverFile.readString();
      }
      serverFile.close();;
   } else {
      ERROR(F("error opening"), F("Disabled BODY file"));
      fatalError(LEDColor::CYAN);
   }

   // open the CSS file on the on-board FS and read it into a String (note that this string is never modified)
   serverFile = SPIFFS.open(CSS_FILE, "r");
   cssFile.reserve(STRING_MAX);
   if (serverFile) {
      if (serverFile.available()) {
         cssFile = serverFile.readString();
      }
      serverFile.close();;
   } else {
      ERROR(F("error opening"), F("CSS file"));
      fatalError(LEDColor::ORANGE);
   }
}

/*
send body string as formatted HTML to client
Because of issues in the String class, we need to output complete HTML file in segments (header, CSS, working HTML code)
*/
void sendHTML (const int code, const char *content_type, const String &body) {
   String title;

   // get correct IP to add to page title
   if (WiFi.getMode() == WIFI_AP) {
      title = String("WiFi CamSlider ") + WiFi.softAPIP().toString();
   } else {
      title = String("WiFi CamSlider ") +  WiFi.localIP().toString();
   }
   INFO(F("Sending web page"), title);

   // header
   client.println(String("HTTP/1.1 ") + String(code) + String(" OK"));
   client.println(String("Content-Type: ") + String(content_type));
   client.println("Connection: close\n");
   client.println(String("<!DOCTYPE HTML> <HTML> <HEAD> <TITLE>") + title + String("</TITLE> </HEAD>"));

   // send CSS file contents
   client.println(cssFile);

   // send body
   client.println(body);
   client.println("</HTML>");
}


/*
clear saved WiFi credentials
*/
void clearCredentials(void) {
   if (EEPROM.read(CRED_ADDR) == EEPROM_KEY) {
      // EEPROM backup saved credentials
      for (int i = 1; i <= sizeof(struct station_config); i++) {
         EEPROM.write(CRED_ADDR + i, 0);
      }
      EEPROM.commit();
   }
   /*
   delete credentials from saved parameter area (last 16KB of flash) using direct ESP SDK library calls
   while we could call WiFi.disconnect(), it will clear the credentials as well as disconnecting from the AP,
   which is undesirable in this case
   */
   struct station_config conf;
   wifi_station_get_config(&conf);
   *conf.ssid = '\0';
   *conf.password = '\0';

   ETS_UART_INTR_DISABLE();
   wifi_station_set_config(&conf);
   ETS_UART_INTR_ENABLE();
}

/*
 depending on what the requested action is, make the appropriate changes to the HTML code stream (video or timelapse mode)
 (e.g. variable substation) and state changes to the main sketch code and send the modified HTML
 stream to the client. This is done simply by copying the string with the base HTML code and making modifications 
 to the copy only.
 
 Color coding:
  * buttons change color when changing state (e.g. run -> standby)
  * text labels will indicated errors in red, "cautions" in yellow (e.g. if parameters were reset to meet min/max limits)
*/
void sendResponse ( const T_Action actionType, const String &url ) {
   String 	indexModified;								// all changes made to this String
   bool		timelapseParamsChanged = false;		// determines if user movement parameters changed

   INFO(F("ACTION"), actionType);

   /*
     Because the CSS colors below must ALWAYS be substituted, we set the normal defaults first
     then change as necessary below
   */
#if DEBUG > 0
   INFO(F("MODE"), sliderMode);
#endif
   // determine what HTML interface file we need based on current mode
   switch ( sliderMode ) {
   case MOVE_VIDEO:
      indexModified = videoBodyFile;
      indexModified.reserve(STRING_MAX);
      break;

   case MOVE_TIMELAPSE:
      indexModified = timelapseBodyFile;
      indexModified.reserve(STRING_MAX);
      break;

   case MOVE_DISABLED:
   default:
      indexModified = disabledBodyFile;
      indexModified.reserve(STRING_MAX_SHORT);
      break;
   }
   
#if DEBUG >= 4
   Serial.println("\nIndex file before modification:");
   Serial.print(indexModified);
#endif
   
   // default colors
   String distanceTextColor = String("white");
   String durationTextColor = String("white");
   
   String totalDistanceTextColor = String("white");
   String totalDurationTextColor = String("white");
   String totalImagesTextColor = String("white");
   
   //process user action
   if ( actionType != IGNORE ) {
      uint8_t  idx;

      switch ( actionType ) {
      /*
       COMMON ACTIONS (implementation may be state-specific)
      */
      case SLIDER_STATE:
          //toggles state to the next state in sequence
         switch ( sliderMode ) {
         case MOVE_DISABLED:
            sliderMode = MOVE_VIDEO;
            video.travelDistance = 0;
            video.travelDuration = 0;
            indexModified = videoBodyFile;
            break;
            
         case MOVE_VIDEO:
            sliderMode = MOVE_TIMELAPSE;
            timelapse.totalDistance = 0;
            timelapse.totalDuration = 0;
            timelapse.totalImages = 0;
            indexModified = timelapseBodyFile;
            break;
            
         case MOVE_TIMELAPSE:
            sliderMode = MOVE_DISABLED;
            indexModified = disabledBodyFile;
            break;
            
         default:
            break;
         }
         break;
         
      case ENDSTOP_STATE:
         // toggle endstop state to next one in sequence
         switch ( endstopAction ) {
         case STOP_HERE:
            endstopAction = REVERSE;
            break;
            
         case REVERSE:
            endstopAction = ONE_CYCLE;
            break;
            
         case ONE_CYCLE:
            endstopAction = STOP_HERE;
            break;
         
         default:
            break;
         }
         break;
         
      case SET_DIRECTION:
         // toggle carriage direction
         clockwise = !clockwise;
         break;
         
      case START_STATE:
         /*
           stop or initiatiate movement
           indicate errors to user if req data is missing
         */
         if ( sliderMode == MOVE_VIDEO ) {
            if ( running ) {
               carriageState = CARRIAGE_STOP;								// state machine will clear running flag
            } else {
               bool conditionsSatisfied = true;
               
               if ( video.travelDistance <= 0 ) {
                     durationTextColor = String("red");
                     conditionsSatisfied = false;
               }
               if ( video.travelDuration <= 0 ) {
                     durationTextColor = String("red");
                     conditionsSatisfied = false;
               }
               
               if ( conditionsSatisfied ) {
                  // travel position & speed set already at input time, so just set the flag for the FSM to start the move
                  newMove = true;
#if DEBUG >= 2
                  Serial.println(String("Move to position: ") + String(targetPosition) + String(" at speed ") + String(targetSpeed));
#endif
               } else {
                  targetSpeed = 0;
               }
            }
         } else if ( sliderMode == MOVE_TIMELAPSE ) {
            if ( timelapse.enabled ) {
               timelapse.enabled = false;
               carriageState = CARRIAGE_STOP;
            } else {
               bool conditionsSatisfied = true;
               
               if ( timelapse.totalDistance <= 0 ) {
                  totalDistanceTextColor = String("red");
                  conditionsSatisfied = false;
               }
               if ( timelapse.totalDuration <= 0 ) {
                  totalDurationTextColor = String("red");
                  conditionsSatisfied = false;
               }
               if ( timelapse.totalImages <= 0 ) {
                  totalImagesTextColor = String("red");
                  conditionsSatisfied = false;
               }

               if ( conditionsSatisfied ) {
                  // sequence move plan params calculated when last user input was received
#if DEBUG >= 2
                  Serial.println(String("Timelapse seq: ") + String(timelapse.totalImages) + String (" images moving ") + String(timelapse.moveDistance) + 
                    String (" in @ interval ") + String(timelapse.moveInterval));
#endif
                  timelapse.imageCount = 0;
                  timelapse.state = S_SHUTTER;
                  timelapse.enabled = true;
                  timelapseMove();
               }
            }
         }
         break;

      case CALIBRATE:
         calibrating = true;
         // fallthru
      case HOME_CARRIAGE:
         // save current stepper params to restore after move is complete
         homeState.homing = true;
         homeState.lastTargetPosition = targetPosition;
         homeState.lastTargetSpeed = targetSpeed;
         homeState.lastEndstopState = endstopAction;
         
         // return the carriage to the home position
         targetPosition = (long)INCHES_TO_STEPS(MAX_TRAVEL_DISTANCE);
         targetSpeed = HS24_MAX_SPEED;
         endstopAction = STOP_HERE;
         clockwise = false;							// towards the motor
         newMove = true;
         break;
      
         
      /*
       VIDEO MODE
       in this mode, move parameters are built as the commands are entered, then we check before initiatiating a move in START_STATE
       calculate speed in each state so data can be entered in any order
       so we can display on the interface, calculate stepper params now and not in START_STATE
      */
      case SET_DISTANCE:
         // get distance to travel in inches
         idx = url.lastIndexOf('=');
         if ( idx ) {
            String value = url.substring(idx+1);
            
            video.travelDistance = constrain(value.toInt(), 1, maxDistance);
            targetPosition = (long)INCHES_TO_STEPS(video.travelDistance);
            if ( video.travelDuration ) {
               targetSpeed = constrain((float)(targetPosition / video.travelDuration), 1.0, HS24_MAX_SPEED);	// steps per second
            } else {
               targetSpeed = 0;
            }
#if DEBUG >= 2
            Serial.println(String("Travel distance: ") + String(video.travelDistance) + String(" inches "));
#endif
         }
         break;
         
      case SET_DURATION:
         //get travel duration
         idx = url.lastIndexOf('=');
         if ( idx ) {
            String value = url.substring(idx+1);
            
            video.travelDuration = constrain(value.toInt(), 1, MAX_TRAVEL_TIME);
            if ( targetPosition > 0 ) {
               targetSpeed = constrain((float)(targetPosition / video.travelDuration), 1.0, HS24_MAX_SPEED);	// steps per second
            } else {
               targetSpeed = 0;
            }
#if DEBUG >= 2
            Serial.println(String("Travel Duration: ") + String(video.travelDuration) + String(" sec "));
#endif
         }
         break;
      
      /*
       TIMELAPSE MODE
       in this mode, individual moves are planned in timelapseMove(), not here, so just get the data we need
      */
      case SET_TL_DISTANCE:
         // distance to move in total
         idx = url.lastIndexOf('=');
         if ( idx ) {
            String value = url.substring(idx+1);
            
            timelapse.totalDistance = constrain(value.toInt(), 1, maxDistance);
#if DEBUG >= 2
            Serial.println(String("Total dist: ") + String(timelapse.totalDistance) + String(" inches "));
#endif
            timelapseParamsChanged = true;						// enable parameter checking below
         }
         break;
         
      case SET_TL_DURATION:
         // total elapsed timrelapse seq time
         idx = url.lastIndexOf('=');
         if ( idx ) {
            String value = url.substring(idx+1);
            
            // carriage needs time to stabalize after a move, so min time is 1 + this delay
            timelapse.totalDuration = constrain(value.toInt(), CARR_SETTLE_SEC, MAX_TRAVEL_TIME);
#if DEBUG >= 2
            Serial.println(String("Total duration: ") + String(timelapse.totalDuration) + String(" sec "));
#endif
            timelapseParamsChanged = true;
         }
         break;
         
      case SET_TL_IMAGES:
         // number of total images to capture
         idx = url.lastIndexOf('=');
         if ( idx ) {
            String value = url.substring(idx+1);
            
            timelapse.totalImages = constrain(value.toInt(), 2, MAX_IMAGES);
#if DEBUG >= 2
            Serial.println(String("Total images: ") + String(timelapse.totalImages));
#endif
            timelapseParamsChanged = true;
         }
         break;

      case FORGET:
         clearCredentials();
         break;
         
      case NULL_ACTION:
      default:
         break;
      }
      
      /*
       substitute current data variables for the placeholders in the base HTML file
       
       COMMON
      */
      switch ( sliderMode ) {
      case MOVE_DISABLED:
         indexModified.replace(String(STATE_VAR), String("Disabled"));
         break;
         
      case MOVE_VIDEO:
         indexModified.replace(String(STATE_VAR), String("Video"));
         break;
         
      case MOVE_TIMELAPSE:
         indexModified.replace(String(STATE_VAR), String("Timelapse"));
         break;
         
      default:
         break;
      }

      if ( (sliderMode != MOVE_DISABLED) && (sliderMode != MOVE_NOT_SET) ) {
         // thiese fields not present on this screen
         switch ( endstopAction ) {
         case STOP_HERE:
            indexModified.replace(String(ENDSTOP_VAR), String("Stop"));
            indexModified.replace(String(ENDSTOP_CSS), String(CSS_RED));
            break;

         case REVERSE:
            indexModified.replace(String(ENDSTOP_VAR), String("Reverse"));
            indexModified.replace(String(ENDSTOP_CSS), String(CSS_PURPLE));
            break;

         case ONE_CYCLE:
            indexModified.replace(String(ENDSTOP_VAR), String("One Cycle"));
            indexModified.replace(String(ENDSTOP_CSS), String(CSS_BLUE));
            break;

         default:
            break;
         }

         indexModified.replace(String(DIRECTION_VAR), clockwise ? String("Away") : String("Towards"));

         // common colors
         if ( clockwise ) {
            indexModified.replace(String(DIRECTION_CSS), String(CSS_BLUE));
         } else {
            indexModified.replace(String(DIRECTION_CSS), String(CSS_ORANGE));
         }
      }

      // mode-specific
      if ( sliderMode == MOVE_VIDEO ) {
         // variables
         indexModified.replace(String(DISTANCE_VAR), String(video.travelDistance));
         indexModified.replace(String(DURATION_VAR), String(video.travelDuration));
         indexModified.replace(String(SPEED_VAR), String((float)(STEPS_TO_INCHES(targetSpeed)), 2));
         indexModified.replace(String(START_VAR), running ? String("Running") : String("Standby"));
         
         // CSS colors
         indexModified.replace(String(MODE_CSS), String(CSS_CYAN));
         indexModified.replace(String(DISTANCE_CSS), distanceTextColor);
         indexModified.replace(String(DURATION_CSS), durationTextColor);
         indexModified.replace(String(START_CSS), running ? String(CSS_GREEN) : String(CSS_RED));
         
         // status section - stepsTaken will either have the running running total or the total from the last run (or 0 if never run, of course)
         indexModified.replace(String(TRAVELED_VAR), String((float)(STEPS_TO_INCHES(stepsTaken)), 2));
         if ( running ) {
            // currently running
            float t_duration = (float)((millis() - travelStart)/1000.0);
            indexModified.replace(String(ELAPSED_VAR), String(t_duration, 2));
            indexModified.replace(String(MEASURED_VAR), String((float)(STEPS_TO_INCHES(stepsTaken)/t_duration), 2));
         } else if ( lastRunDuration ) {
            // previous run
            float t_duration = (float)(lastRunDuration/1000.0);
            indexModified.replace(String(ELAPSED_VAR), String(t_duration, 2));
            indexModified.replace(String(MEASURED_VAR), String((float)(STEPS_TO_INCHES(stepsTaken)/t_duration), 2));
         } else {
            indexModified.replace(String(ELAPSED_VAR), String(" "));
            indexModified.replace(String(MEASURED_VAR), String(" "));
         }
      } else if ( sliderMode == MOVE_TIMELAPSE ) {
         if ( timelapseParamsChanged && ((timelapse.totalDistance > 0) && (timelapse.totalDuration > 0) && (timelapse.totalImages > 0)) ) {
            // pre-calculate & validate the sequence parameters for user display in status: the number of moves (images) and delay between images (moves)
            timelapse.moveDistance = (int)floor(timelapse.totalDistance / (timelapse.totalImages - 1));
            if ( timelapse.moveDistance <= 0 ) {
               // must actually move the stepper for the state machine to function
               timelapse.moveDistance = 1;
               timelapse.totalDistance = timelapse.totalImages - 1;
               totalDistanceTextColor = String("yellow");
            }
            
            timelapse.moveInterval = (int)floor(timelapse.totalDuration / (timelapse.totalImages - 1));		
            int minDelay = (int)ceil(INCHES_TO_STEPS(timelapse.moveDistance) / HS24_MAX_SPEED);			// inches per step / steps per second = seconds
            minDelay += CARR_SETTLE_SEC;
            if ( timelapse.moveInterval < minDelay ) {
               // minimum interval is the carriage move time + stabilization delay
               timelapse.moveInterval = minDelay;
               timelapse.totalDuration = timelapse.moveInterval * (timelapse.totalImages - 1);
               totalDurationTextColor = String("yellow");
            }
            timelapse.imageCount = 0;	
#if DEBUG >= 2
            Serial.println(String("Timelapse seq: ") + String(timelapse.totalImages) + String (" images moving ") + String(timelapse.moveDistance) + 
              String (" in @ interval ") + String(timelapse.moveInterval));
#endif				
         }
         
         // variables
         indexModified.replace(String(TL_DISTANCE_VAR), String(timelapse.totalDistance));
         indexModified.replace(String(TL_DURATION_VAR), String(timelapse.totalDuration));
         indexModified.replace(String(TL_IMAGES_VAR), String(timelapse.totalImages));
         indexModified.replace(String(START_VAR), timelapse.enabled ? String("Running") : String("Standby"));
         // colors
         indexModified.replace(String(MODE_CSS), String(CSS_MAGENTA));
         indexModified.replace(String(TL_DISTANCE_CSS), totalDistanceTextColor);
         indexModified.replace(String(TL_DURATION_CSS), totalDurationTextColor);
         indexModified.replace(String(TL_IMAGES_CSS), totalImagesTextColor);
         indexModified.replace(String(START_CSS), timelapse.enabled ? String(CSS_GREEN) : String(CSS_RED));
         
         // status
         indexModified.replace(String(TL_MOVEDIST), String(timelapse.moveDistance));
         indexModified.replace(String(TL_INTERVAL), String(timelapse.moveInterval));
         indexModified.replace(String(TL_COUNT), String(timelapse.imageCount));
      } else {
         // disabled
         indexModified.replace(String(MODE_CSS), String(CSS_RED));
      }
      

#if DEBUG >= 3
      Serial.println("\n**INDEX FILE MODIFIED**");
      Serial.print(indexModified);
#endif
      // finally, send to the client
      sendHTML(200, "text/html", indexModified);
   }
}

/*
  main WiFi service routine
*/
void WiFiService ( void ) {
   bool responseSent = false;
   
   client = server.available();
   if ( client && client.connected() ) {
      INFO(F("Client connected. Connected flag"), userConnected);
      /*
       If this is the first time we are connected, disable AP mode broadcast of the IP address
       and enable OTA mode
       To be safe, we'll exit now and pick up the user commands on the next iteration
      */
      if (!userConnected) {
         userConnected = true;
         STAMode();
         sliderMode = MOVE_TIMELAPSE;                       // set initial default so LED status light will change
         return;
      }
      // retrieve the URI request from the client stream
      String uri = client.readStringUntil('\r');
      INFO(F("Client URI"), uri);
      client.flush();
      
      // scan the action table to get the action and send appropriate response
      for ( uint8_t i = 0; i < ACTION_TABLE_SIZE; i++ ) {
         if ( uri.indexOf(actionTable[i].action) != -1 ) {
            // done once we found the first match (this is why null actions are the the bottom of the table)
            sendResponse(actionTable[i].type, uri);
            responseSent = true;
            break;
         }
      }
      if ( !responseSent ) {
         // keep the connection alive
         sendResponse(NULL_ACTION, uri);
      }
      client.stop();
   }
}