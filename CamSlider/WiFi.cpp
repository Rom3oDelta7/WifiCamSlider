/*
   TABS=3

	camera slider WiF control interface
	
	Prior to running this sketch, the HTML file must be loaded into the ESP8266 SPIFFS file system.
	Currently, the instructions for installing and running the uplaod tool can be found here: http://esp8266.github.io/Arduino/versions/2.3.0/doc/filesystem.html

   Copyright 2016 Rob Redford
   This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
   
*/

#define DEBUG			2

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
#include "FS.h"
#include "CamSlider.h"

// main sketch externs
extern volatile bool				newMove;					// true when we need to initiate a new move
extern volatile bool				clockwise;
extern volatile EndstopMode	endstopAction;			// action to take when an endstop is hit
extern volatile CarriageMode	carriageState;			// current state of the carriage (for the motion state machine)
extern long							targetPosition;		// inches to travel
extern float						targetSpeed;			// speed in steps/second
extern unsigned long				travelStart;			// start of curent carriage movement
extern int							stepsTaken;				// counts steps actually executed
extern bool							running;					// true iff moving the carriage


// inline variable substitution
#define DIRECTION_VAR			"%DIRECTION%"			// direction of travel (fwd/rev)
#define MOTOR_VAR					"%MOTOR%"				// enabled/disabled
#define DISTANCE_VAR				"%DISTANCE%"			// req distance to travel
#define DURATION_VAR				"%DURATION%"			// req travel time
#define SPEED_VAR					"%SPEED%"				// speed translated form user inputs
#define START_VAR					"%START%"				// running/standby
#define TRAVELED_VAR				"%TRAVELED%"			// distance traveled in last move
#define ELAPSED_VAR				"%ELAPSED%"				// elapsed time of last move


#define	INDEX_FILE				"/index.html"				// root HTML file (SPIFFS) name (static content)


/*
 user actions in HTML request stream
 also includes URI requests that invoke no action but will send the HTML file anyways (always need to reply to client)
*/
#define ACTION_MOTOR				"MOTOR_BTN="				// motor button
#define ACTION_END_STOP			"STOP"						// endstop state to stop (dropdown menu)
#define ACTION_END_REVERSE		"REVERSE"					// endstop state to reverse
#define ACTION_END_CYCLE		"CYCLE"						// endstop state to one cycle
#define ACTION_DISTANCE			"DIST="						// input distance to travel
#define ACTION_DURATION			"DURN="						// input travel duration
#define ACTION_DIRECTION		"DIRECTION_BTN="			// toggle carriage direction
#define ACTION_START				"START_BTN="				// initiate/stop movement
#define ACTION_REFRESH			"REFRESH_BTN="				// refresh display

typedef enum { NULL_ACTION, IGNORE, MOTOR_STATE, SET_STOP, SET_REVERSE, SET_CYCLE, SET_DISTANCE, SET_DURATION, SET_DIRECTION, START_STATE } T_Action;

#define ACTION_TABLE_SIZE		12
const struct {	
	char 		action[20];
	T_Action	type;
} actionTable[ACTION_TABLE_SIZE] = {
	{ACTION_MOTOR, 		MOTOR_STATE},
	{ACTION_END_STOP, 	SET_STOP},
	{ACTION_END_REVERSE,	SET_REVERSE},
	{ACTION_END_CYCLE,	SET_CYCLE},
	{ACTION_DISTANCE,		SET_DISTANCE},
	{ACTION_DURATION,		SET_DURATION},
	{ACTION_DIRECTION,	SET_DIRECTION},
	{ACTION_START,			START_STATE},
	{ACTION_REFRESH,		NULL_ACTION},			// null actions must be at the end so they don't intercept ones above
	{"GET / ",				NULL_ACTION},					
	{"GET /index.html",	NULL_ACTION},
	{"GET /favicon.ico",	IGNORE}				// always comes after another request, so skip it
};


/*
 identity info for each control unit to employ in creating unique network identiies
 capture the last 2 bytes of the ESP8266 MAC address using the MacAddress utility and add here
*/
#define	ID_TABLE_SIZE			2 
const struct {
	char 		ssid[10];					// need char arrays not String objects for function call arguments
	char 		password[12];
	uint8_t 	mac[2];						// last 2 bytes of MAC address
	uint8_t	subnet;						// unique subnet to use for gateway address and DHCP assignment
} ESP_Identifier[ID_TABLE_SIZE] = {
	{"CSPROTO", "camslider", {0xF3, 0xD6}, 30},
	{"CS001",   "camslider", {0xF3, 0x83}, 31}
};

/*
 CSS color substitution variables and color defines
*/
#define MOTOR_CSS					"%MOTOR_CSS%"
#define DIRECTION_CSS			"%DIRECTION_CSS%"
#define DISTANCE_CSS				"%DISTANCE_CSS%"
#define DURATION_CSS				"%DURATION_CSS%"
#define START_CSS					"%START_CSS%"

#define CSS_GREEN					"greenbkgd"
#define CSS_RED					"redbkgd"
#define CSS_ORANGE				"orangebkgd"
#define CSS_GREY					"greybkgd"
#define CSS_BLUE					"bluebkgd"


WiFiServer	server(80);						// web server instance
String		indexFile;						// on-board flash file system in String form
WiFiClient	client; 							// client stream

#define MAX_TRAVEL_DISTANCE	80			// maximum possible travel distance (inches)
#define MAX_TRAVEL_TIME			3600		// maximum possible travel duration (sec)

bool			inputPermitted = true;		// controls input actions (action for motor enable/disable button)
int			travelDuration = 0;			// holds travel duration in sec until we convert it to speed
int			travelDistance = 0;			// holds travel distance in inches until we convert it to steps


/*
  initialization
*/
void setupWiFi ( void ) {
	uint8_t	mac[WL_MAC_ADDR_LENGTH];
	uint8_t	subnet;
	
	// get MAC address and use this to select ID info for WiFi setup
#if DEBUG > 0
	Serial.println("\nConfiguring access point...");
#endif
	
	WiFi.mode(WIFI_AP);
	WiFi.softAPmacAddress(mac);
	uint8_t index = ID_TABLE_SIZE;

	for ( uint8_t i = 0; i < ID_TABLE_SIZE; i++ ) {
		if ( (ESP_Identifier[i].mac[0] == mac[WL_MAC_ADDR_LENGTH - 2]) && (ESP_Identifier[i].mac[1] == mac[WL_MAC_ADDR_LENGTH - 1]) ) {
			index = i;
			break;
		}
	}
	if ( index < ID_TABLE_SIZE ) {
		WiFi.softAP(ESP_Identifier[index].ssid, ESP_Identifier[index].password);
		subnet = ESP_Identifier[index].subnet;
#if DEBUG > 0
		Serial.print("Mapping successful. Using SSID: ");
		Serial.println(ESP_Identifier[index].ssid);
#endif
	} else {
		char 	DefaultSSID[12];

		snprintf(DefaultSSID, 12, "DEFAULT%02X%02X", mac[WL_MAC_ADDR_LENGTH - 2], mac[WL_MAC_ADDR_LENGTH - 1]);
		WiFi.softAP(DefaultSSID);				// no password in the default case
		subnet = ESP_Identifier[ID_TABLE_SIZE-1].subnet+1;
#if DEBUG > 0
		Serial.print("No MAC address match - using default. SSID: ");
		Serial.println(DefaultSSID);
#endif
	}
	delay(500);
	
	// setup the default IP address then remap so each controller has its own subnet address range
	IPAddress myIP = WiFi.softAPIP();
#if DEBUG > 0
	Serial.print("\nInitial AP IP address: ");
	Serial.println(myIP);
#endif

	struct ip_info ipinfo;
	
	if ( wifi_get_ip_info(SOFTAP_IF, &ipinfo) ) {
		wifi_softap_dhcps_stop();
		IP4_ADDR(&ipinfo.ip, 192, 168, subnet, 1);
		IP4_ADDR(&ipinfo.gw, 192, 168, subnet, 1);
		IP4_ADDR(&ipinfo.netmask, 255, 255, 255, 0);
		if ( !wifi_set_ip_info(SOFTAP_IF, &ipinfo) ) {
#if DEBUG > 0
			Serial.println("IP address change failed");
#endif
		} else {
			myIP = WiFi.softAPIP();
#if DEBUG > 0
			Serial.print("AP IP address CHANGED: ");
			Serial.println(myIP);
#endif
		}
		wifi_softap_dhcps_start();
	}
	
	// start the server & init the SPIFFS file system
	server.begin();
	if ( !SPIFFS.begin() ) {
#if DEBUG > 0
		Serial.println("Cannot open SPIFFS file system.");
#endif
		//errorLED(false);  ZZZ 
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

	// open the HTML file on the on-board FS and read it into a String (note that this string is never modified)
	File serverFile = SPIFFS.open(INDEX_FILE, "r");
	if ( serverFile ) {
		while ( serverFile.available() ) {
			indexFile += (char)serverFile.read();
		}
		serverFile.close();;
	} else {
		//errorLED(true);						// blinking ZZZZ add LED to schematic & setup()
	}
}

/*
  send string as formatted HTML to client
*/
void sendHTML ( const int code, const char *content_type, const String &content ) {
	// header
	client.println(String("HTTP/1.1 ") + String(code) + String(" OK"));
	client.println(String("Content-Type: ") + String(content_type));
	client.println("Connection: close\n");
	// send content
	client.println(content);
}

/*
 depending on what the requested action is, make the appropriate changes to the HTML code stream
 (e.g. variable substation) and state changes to the main sketch code (e.g. increasing brightness) and send the modified HTML
 stream to the client
*/
void sendResponse ( const T_Action actionType, const String &url ) {
	String indexModified = indexFile;								// all changes made to this String

#if DEBUG > 0
	Serial.print("ACTION: ");
	Serial.println(actionType);
#endif
#if DEBUG >= 4
	Serial.println("\nIndex file, unmodified:");
	Serial.print(indexModified);
#endif

	/*
	  Because the CSS colors for distance and duration must ALWAYS be substituted, we set the normal defaults first
	  then change as necessary below - cleaner than the alternative of complex logic
	*/
	indexModified.replace(String(DISTANCE_CSS), String("white"));
	indexModified.replace(String(DURATION_CSS), String("white"));
	
	//process user action
	if ( actionType != IGNORE ) {
		uint8_t  idx;

		switch ( actionType ) {
		case MOTOR_STATE:
			/*
			 toggles motor state - enabled/disabled
			 note that this is separate from the controller enable pin setting - it simply allows
			 user actions to be executed or not. Controller is always disabled when it is not moving.
			 This naming is for user interface consistency
			*/
			inputPermitted = !inputPermitted;
			break;
			
		case SET_STOP:
			endstopAction = STOP_HERE;
			break;
			
		case SET_REVERSE:
			endstopAction = REVERSE;
			break;

		case SET_CYCLE:
			endstopAction = ONE_CYCLE;
			break;

			
		case SET_DISTANCE:
			// get distance to travel in inches and convert to steps
			idx = url.lastIndexOf('=');
			if ( idx ) {
				String value = url.substring(idx+1);
				
				travelDistance = constrain(value.toInt(), 1, MAX_TRAVEL_DISTANCE);
				targetPosition = (long)INCHES_TO_STEPS(travelDistance);
				Serial.println(String("Target Position: ") + String(targetPosition) + String(" steps "));
			}
			break;
			
		case SET_DURATION:
			//get travel duration - conversion to speed in START_STATE
			idx = url.lastIndexOf('=');
			if ( idx ) {
				String value = url.substring(idx+1);
				
				travelDuration = constrain(value.toInt(), 1, MAX_TRAVEL_TIME);
				Serial.println(String("Travel Duration: ") + String(travelDuration) + String(" sec "));
			}
			break;
		
		case SET_DIRECTION:
			// toggle carriage direction
			clockwise = !clockwise;
			break;
			
		case START_STATE:
			/*
			  stop or initiatiate movement
			  indicate errors to user if we don't have what we need to initiate a move
			*/
			if ( inputPermitted ) {
				if ( running ) {
					carriageState = CARRIAGE_STOP;
				} else {
					if ( targetPosition > 0 ) {
						if ( travelDuration > 0 ) {
							// all conditions satisfied - plan a new move
							targetSpeed = constrain((float)(targetPosition / travelDuration), 1.0, HS24_MAX_SPEED);	// steps per second
							newMove = true;																									// this flag triggers a new move
						} else {
							indexModified.replace(String(DURATION_CSS), String("red"));
						}
					} else {
						indexModified.replace(String(DISTANCE_CSS), String("red"));
					}
				}
			} else {
				// error indication needed here ?? ZZZ
			}
			break;
			
		case NULL_ACTION:
		default:
			break;
		}
	
		// substitute current data variables for the placeholders in the base HTML file
		indexModified.replace(String(MOTOR_VAR), inputPermitted ? String("Enabled") : String("Disabled"));
		indexModified.replace(String(DIRECTION_VAR), clockwise ? String("Forward") : String("Reverse"));
		indexModified.replace(String(DISTANCE_VAR), String(travelDistance));
		indexModified.replace(String(DURATION_VAR), String(travelDuration));
		indexModified.replace(String(SPEED_VAR), String(targetSpeed));
		indexModified.replace(String(START_VAR), running ? String("Running") : String("Standby"));
		indexModified.replace(String(TRAVELED_VAR), String((float)STEPS_TO_INCHES(stepsTaken), 2));
		indexModified.replace(String(ELAPSED_VAR), String((float)(((travelStart - millis())/1000), 2)));
		
		
		// change REMAINING color placeholders to corresponding CSS to match current state
		if ( clockwise ) {
			indexModified.replace(String(DIRECTION_CSS), String(CSS_BLUE));
		} else {
			indexModified.replace(String(DIRECTION_CSS), String(CSS_ORANGE));
		}
		if ( inputPermitted ) {
			indexModified.replace(String(MOTOR_CSS), String(CSS_GREEN));
		} else {
			indexModified.replace(String(MOTOR_CSS), String(CSS_RED));
		}
		if ( newMove ) {
			indexModified.replace(String(START_CSS), String("green"));
		} else {
			indexModified.replace(String(START_CSS), String("red"));
		}

#if DEBUG >= 3
		Serial.println("\nINDEX FILE **MODIFIED**");
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
#if DEBUG > 0
		Serial.println("Client connected");
#endif
		// retrieve the URI request from the client stream
		String uri = client.readStringUntil('\r');
#if DEBUG > 0
		Serial.print("Client URI: ");
		Serial.println(uri);
#endif
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