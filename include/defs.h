#ifndef DEFS_H
#define DEFS_H

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

String serialNumber = "000001";
String VER_STRING = "v1.0.0";

// ---------- Button Handling ----------
extern bool p1prox1On;
extern bool p1prox2On;
extern bool p2prox1On;
extern bool p2prox2On;
extern bool p1lvlOn = false;
extern bool p2lvlOn = false;
extern unsigned long upLeftStartTime;
extern unsigned long lastMQTTReconnectAttempt;

// --------------- Global WiFi Settings and Declarations ---------------
struct wifiSettings_t {
  String ssid;
  String password;
  String htmlVersion;
  String cssVersion;
  String mqttServerAddress;
  int    mqttPort;
  String updateTopic;
  String baseUpdateUrl;
};

#define WIFI_SETTINGS_FILE "/wifiSettings.json"
extern wifiSettings_t wifiSettings;
#define SETTINGS_FILE "/settings.json"

extern String serialNumber;
extern String VER_STRING;

// ------------------ HiveMQ Cloud MQTT Setup ------------------
// Replace with your HiveMQ Cloud details:
#define HIVEMQ_SERVER "bd58f8878eef4f5eb73ac65312b10130.s1.eu.hivemq.cloud"
#define HIVEMQ_PORT   8883
#define HIVEMQ_USER   "a3Admin"
#define HIVEMQ_PASS   "DontLetMeIn247"

extern WiFiClientSecure wifiClient;
extern PubSubClient mqttClient;
extern AsyncWebServer server;
extern DNSServer dnsServer;
// Analogue inputs
const int supplyVoltPin = 4;
//int rtcVoltPin = 11;

// General variables (declare as extern, define in main.cpp)
extern uint8_t xPos;
extern uint8_t yPos;
extern bool setupMode;
extern uint8_t rootMenuIndex;
extern uint8_t maxRootIndex;
extern bool rootMenuChanged;
extern bool subMenu1;
extern bool retFromSub1;
extern bool adjustMode;
extern uint8_t speedUpCounter;
extern uint8_t speedUpCount;
extern uint8_t speedUpCountFast;
extern int normalSpeedAdj;
extern int fastSpeedAdj;
extern bool runningFast;
extern bool runningFaster;
extern bool clearCursor;
extern int previousIndex;
extern int tankLevelPerc;
extern unsigned long pulseDelay;
extern int pulseCount;
extern int pulseCounter;
extern unsigned long p1lvlPulseStrt;
extern unsigned long p2lvlPulseStrt;
extern bool dontShowTest;
extern bool updateTime;
extern bool updateText;
extern bool updateCycles;
extern bool valueChanged;
extern bool setupExited;
extern int iconFlashOn;
extern bool altIconFlash;
extern bool pump1Error;
extern bool pump2Error;
extern bool toggleError;
extern bool justStarted;
extern bool pump1displayed;
extern bool pump2displayed;
extern bool displayPumpNo;
extern bool sleeping;
extern bool screensaving;
extern uint32_t screenSaveTimeout;
extern uint32_t screenSaveTime;
extern uint32_t screenSaveAnimate;
extern unsigned long prevCurrTstMillis;
extern int testTimeTotal;
extern int testTimeCounter;
extern bool autoMode;

// Global variables for screensaver animation
int sleepX = 0;
int sleepY = 0;
int dx = 1;                   // Horizontal direction: +1/right, -1/left
int dy = 4;                   // Vertical direction: +1/down, -1/up
int speed = 2;                // Horizontal speed in pixels per update

uint16_t pump1Current = 0;
uint16_t pump2Current = 0;
int oldTankLevel = 0;
unsigned long previousMillis = 0; //General timing variable
unsigned long prevVoltMillis = 0; //Timing variable for voltage reading
float supplyVoltage = 0; //Variable to hold the supply voltage reading
float shortCircuitCurrent = 18000.0; //Short circuit current for the pump, set to 15A in milliamps
float openCircuitCurrent = 10.0; //Open circuit current for the pump, set to 10 milliamp

// Alarm control variables, also used for event logging
bool p1activeAlarm = false;
bool p2activeAlarm = false;
bool beAnnoying = false;
#define numFaults 16 //Number of alarms to cater for, change if more get added
int faultStatus[numFaults] = {0}; //Array to hold the individual alarm values, 0 being off, > 0 indicating the number of times each alarm has been triggered
bool hardStop = false; //Stops controller until reset, pump will not run if this is on.

//Individual flag index values with descriptions, 0 to 5 is listed with the proxy and level pin setups
#define p1shrtFault 6 //Fault flag for pump 1 motor short circuit
#define p1ocFault 7 //Fault flag for pump 1 motor open circuit
#define p1blockFault 8 //Fault flag for pump 1 blockage
#define p2shrtFault 9 //Fault flag for pump 2 motor short circuit
#define p2ocFault 10 //Fault flag for pump 2 motor open circuit
#define p2blockFault 11 //Fault flag for pump 2 blockage

// Event log variables
File logFile;

//Individual event codes for the log file entry
// Non critical events
int settingsChange = 101; //Local update
int settingsUpdate = 102; //App update
int forcedPause = 103;    //When enter pressed to pause cycle if in error state
int powerOn = 104;        //Self explanatory
int powerOff = 105;       //Self explanatory
int faultsCleared = 106;  //All faults have been cleared
int fwUpgrade = 107;      //When firmware gets upgraded
// Critical events, count starts at 300 for expansion room for not critical events
int forcedRun = 301;      //Enter pressed when in error state to make motor run again, will usually accompany a current error state
int p1motOc = 302;       //Pump 1 open circuit
int p1motSc = 303;       //Pump 1 short curcuit
int p1motBlock = 304;    //Pump 1 tripped on blockage current
int p2motOc = 305;       //Pump 2 open circuit
int p2motSc = 306;       //Pump 2 short curcuit
int p2motBlock = 307;    //Pump 2 tripped on blockage current
int lampSc = 308;        //Lamp output short circuit
int p1lvlErr = 309;      //Pump 1 level error
int p1proxy1Err = 310;   //Pump 1 Proxy 1 error
int p1proxy2Err = 311;   //Pump 1 Proxy 2 error
int p2proxy1Err = 312;   //Pump 2 Proxy 1 error
int p2proxy2Err = 313;   //Pump 2 Proxy 2 error
int p2lvlErr = 314;      //Pump 2 level error
String verbatimFaultMsg = ""; //String to hold the fault message for reporting

// Header template for log file
String logHeader{
  "event_code,event_datetime,errors,firmware_revision,supply_voltage,"
  "pump1_current,pump2_in_use,pump2_current,"
  "p1_prox1_state,p1_prox2_state,p1_level_state,"
  "p2_prox1_state,p2_prox2_state,p2_level_state,"
  "ignition_state,pump1_state,pump2_state,lamp_state,"
  "p1_pause_time,p1_run_mode,p1_run_time,"
  "p1_prox1_setting,p1_prox1_dwell,p1_prox2_setting,p1_prox2_dwell,p1_vent_time,"
  "p1_level,p1_level_type,"
  "p2_pause_time,p2_run_mode,p2_run_time,"
  "p2_prox1_setting,p2_prox1_dwell,p2_prox2_setting,p2_prox2_dwell,p2_vent_time,"
  "p2_level,p2_level_type,ext_lamp_type,"
  "p1_block_current,p2_block_current,"
  "tot_on_time,tot_p1_run_time,tot_p1_pause_time,tot_p2_run_time,tot_p2_pause_time,tot_err_time,"
  "tot_p1_prox1_err_time,tot_p1_prox2_err_time,tot_p1_level_err_time,tot_p1_short_time,tot_p1_open_time,tot_p1_block_time,"
  "tot_p2_prox1_err_time,tot_p2_prox2_err_time,tot_p2_level_err_time,tot_p2_short_time,tot_p2_open_time,tot_p2_block_time,"
  "tot_lamp_short_time,rtc_voltage,sequence_number"
};

uint32_t pump1OnCounter = 0; //For counting the on time seconds or cycles of the pump when running
uint32_t pump1DwellCounter = 0; //For counting the dwell time when in cycle mode
uint32_t pump1OffCounter = 0; //For counting the pause time in seconds
uint32_t p1proxy1dwellCounter = 0; //For counting the dwell time of proxy1 when in cycle mode and both proxies are in use
uint32_t p1proxy2dwellCounter = 0; //For counting the dwell time of proxy2 when in cycle mode and both proxies are in use
uint32_t pump1CycleCounter = 0; //For Cycle mode, counting cycles
uint32_t pump1PrevPumpSecond = 0;
uint32_t pump2OnCounter = 0; //For counting the on time seconds or cycles of the pump when running
uint32_t pump2DwellCounter = 0; //For counting the dwell time when in cycle mode
uint32_t pump2OffCounter = 0; //For counting the pause time in seconds
uint32_t p2proxy1dwellCounter = 0; //For counting the dwell time of proxy1 when in cycle mode and both proxies are in use
uint32_t p2proxy2dwellCounter = 0; //For counting the dwell time of proxy2 when in cycle mode and both proxies are in use
uint32_t pump2CycleCounter = 0; //For Cycle mode, counting cycles
uint32_t pump2PrevPumpSecond = 0;
unsigned long flashDelay = 700;
unsigned long secondDelay = 1000;
unsigned long prevIconFlashMs = 0;
unsigned long prevSecondMs = 0;
unsigned long prevMainMs = 0;
unsigned long prevAlmMs = 0;

// Define color constants for RGB ;leds
#define RED     128, 0, 0
#define GREEN   0, 128, 0
#define BLUE    0, 0, 126
#define YELLOW  128, 64, 0
#define OFF     0, 0, 0
#define CYAN    0, 128, 128
#define PURPLE  128, 0, 128
#define ORANGE  128, 64, 0

//Input button variables
bool upLeftPressed = false;
bool upLeftLongPressed = false;
bool upLeftReleased = false;
bool dnRightPressed = false;
bool dnRightLongPressed = false;
bool dnRightReleased = false;
bool enterPressed = false;
bool enterLongPressed = false;
bool enterReleased = false;
int dnRight = 7;
int upLeft = 5;
int enter = 6;
bool flashMot = false;  //Boolean to toggle flashing of the motor symbol for test mode
bool flashLamp = false;  //Boolean to toggle flashing of the lamp symbol for test mode
unsigned long upLeftStartTime = 0;
unsigned long dnRightStartTime = 0;
unsigned long enterStartTime = 0;
unsigned long updnLongPress = 800;  // Long press duration in milliseconds
unsigned long debounce = 150;         // Debounce duration in milliseconds
unsigned long longPress = 1500;      // Long press duration for enter key

//Output pin variables
int pump1Out = 13;
bool pump1Running = false;
bool pump1AutoCalibrate = false; // Flag to indicate if Pump 1 is in auto calibration mode
unsigned long pump1StartTime = 0; // Start time for Pump 1 auto calibration
int pump2Out = 14;
bool pump2Running = false;
bool pump2AutoCalibrate = false; // Flag to indicate if Pump 2 is in auto calibration mode
unsigned long pump2StartTime = 0; // Start time for Pump 2 auto calibration
int buzzOut = 40;
bool buzzOn = false;
int lampOut = 48;
bool lampOn = false;
#define LED_PIN 21
#define LED_COUNT 5

//Proxy and level input variables
//Pump1 proxies
int p1prox1In = 39; //Physical input pin
bool p1prox1changed = false; //Flag to check for change in state
int p1prox1state = 0; //Actual input state record
#define p1prox1fault 0 //Fault state array index value

int p1prox2In = 17; //Physical input pin
bool p1prox2changed = false; //Flag to check for change in state
int p1prox2state = 0; //Actual input state record
#define p1prox2fault 1 //Fault state array index value

//Pump1 level
int p1lvlIn = 18;
bool p1lvlChanged = false;
bool p1lvlHiToLow = false;
int p1lvlState = 0;
#define p1lvlfault 2 //Fault state array index value
int p1lvlLowVal = 10;
int p1lvlHallAddr = 8; //I2C address for Hall sensor level measurement

//Pump2 proxies
int p2prox1In = 3;
bool p2prox1changed = false;
int p2prox1state = 0;
#define p2prox1fault 3 //Fault state array index value

int p2prox2In = 46;
bool p2prox2changed = false;
int p2prox2state = 0;
#define p2prox2fault 4 //Fault state array index value

//Pump2 level
int p2lvlIn = 10;
bool p2lvlChanged = false;
bool p2lvlHiToLow = false;
int p2lvlState = 0;
#define p2lvlfault 5 //Fault state array index value
int p2lvlLowVal = 10;
int p2lvlHallAddr = 9; //I2C address for Hall sensor level measurement

int ignIn = 47;

#define HALL_ADDRESS 0x08 //Slave address for Hall sense level measurement
const char* colour = "GRN";

// Global storage variables for settings, to enable after the fact resoration if adjustment values don't get saved.
String oldP1MainMode = "";
String oldP2MainMode = "";
uint32_t oldP1PauseTime = 0;
uint32_t oldP2PauseTime = 0;
String oldP1RunMode = "";
String oldP2RunMode = "";
uint32_t oldP1RunTimeCyc = 0;
uint32_t oldP2RunTimeCyc = 0;
uint32_t oldP1CycTimeout = 0;
uint32_t oldP2CycTimeout = 0;
String oldP1Prox1 = "";
String oldP1Prox2 = "";
uint32_t oldP1Prox1Dwell = 0;
uint32_t oldP1Prox2Dwell = 0;
uint16_t oldP1Prox2Cyc = 0;
uint32_t oldP1VentTime = 0;
String oldP1Lvl = "";
String oldP1LvlType = "";
String oldP1LvlNonC = "";
String oldP2Prox1 = "";
String oldP2Prox2 = "";
uint32_t oldP2Prox1Dwell = 0;
uint32_t oldP2Prox2Dwell = 0;
uint16_t oldP2Prox2Cyc = 0;
uint32_t oldP2VentTime = 0;
String oldP2Lvl = "";
String oldP2LvlType = "";
String oldP2LvlNonC = "";
uint32_t oldP1BlockCurrent = 0;
uint32_t oldP2BlockCurrent = 0;
String oldLmpYN = "";
String oldLmpType = "";
String oldP2InUse = "";

// Struct to hold the settings
struct Settings {
  String P1_MAIN_MODE;  // Main mode of Pump 1 of controller. States: PLS, SLS and DLS
  uint32_t P1_PAUSE_TIME;    // Pause Time for Pump 1 in seconds, convert to hours and minutes, pump off
  String P1_RUN_MODE;   // Run Mode type for Pump 1, Time or Cycle based. States: TIME or CYCLE
  uint32_t P1_RUN_TIME_CYC;  // Run time for Pump 1 in seconds, converts to minutes and seconds in TIME run mode or cycles in CYCLE mode, pump on
  uint32_t P1_CYC_TIMEOUT;   // Timeout in Pump 1 Cycle mode, in seconds, converts to hours and minutes, used as backup if proxy not detected, not used currently
  String P1_PROX1;     // Pump 1 Proxy 1 in use, YES or NO, automatic if Cycle mode is used
  uint32_t P1_PROX1_DWELL;  // Pump 1 Proxy 1 dwell time in seconds, convert to hours and minutes
  String P1_PROX2;     // Pump 1 Proxy 2 in use, YES or NO
  uint32_t P1_PROX2_DWELL;  // Pump 1 Proxy 2 dwell time in seconds, convert to hours and minutes
  uint16_t P1_PROX2_CYC;    // Pump 1 Proxy 2 cycle count if in CYCLE mode
  uint32_t P1_VENT_TIME;  // Vent time for Pump 1 in seconds, convert to hours and minutes
  String P1_LVL;      // Use level detection for Pump 1, YES or NO
  String P1_LVL_TYPE;   // Level type for Pump 1, Low Level only, LLO, Pulsed full, PULF, Pulsed empty, PULE, or Hall sensor level, HEF
  String P1_LVL_NONC;   // If not Hall sensor level type for Pump 1, level NO or NC
  String PUMP2_IN_USE; // Pump 2 in use, YES or NO
  String P2_MAIN_MODE;  // Main mode of Pump 2 of controller. States: PLS, SLS and DLS
  uint32_t P2_PAUSE_TIME;    // Pause Time for Pump 2 in seconds, convert to hours and minutes, pump off
  String P2_RUN_MODE;   // Run Mode type for Pump 2, Time or Cycle based. States: TIME or CYCLE
  uint32_t P2_RUN_TIME_CYC;  // Run time for Pump 2 in seconds, converts to minutes and seconds in TIME run mode or cycles in CYCLE mode, pump on
  uint32_t P2_CYC_TIMEOUT;   // Timeout in Pump 2 Cycle mode, in seconds, converts to hours and minutes, used as backup if proxy not detected, not used currently
  String P2_PROX1;     // Pump 2 Proxy 1 in use, YES or NO, automatic if Cycle mode is used
  uint32_t P2_PROX1_DWELL;  // Pump 2 Proxy 1 dwell time in seconds, convert to hours and minutes
  String P2_PROX2;     // Pump 2 Proxy 2 in use, YES or NO
  uint32_t P2_PROX2_DWELL;  // Pump 2 Proxy 2 dwell time in seconds, convert to hours and minutes
  uint16_t P2_PROX2_CYC;    // Pump 2 Proxy 2 cycle count if in CYCLE mode
  uint32_t P2_VENT_TIME;  // Vent time for Pump 2 in seconds, convert to hours and minutes
  String P2_LVL;      // Use level detection for Pump 2, YES or NO
  String P2_LVL_TYPE;   // Level type for Pump 2, Low Level only, LLO, Pulsed full, PULF, Pulsed empty, PULE, or Hall sensor level, HEF
  String P2_LVL_NONC;   // If not Hall sensor level type for Pump 2, level NO or NC
  String EXT_LAMP;   // External warning lamp attached, YES or NO
  String LAMP_TYP;   // Lamp type, Steady lamp, STE, Flashing, FLA or RGB for Ultraflow RGB lamp
  String P1_CURR_STATE; // When starting up, status of pump 1 when power was removed, RUNNING or STOPPED
  uint32_t P1_CURR_TIME;     // Cycle time or timeout time left when power was switched off for pump 1
  String P2_CURR_STATE; // When starting up, status of pump 2 when power was removed, RUNNING or STOPPED
  uint32_t P2_CURR_TIME;     // Cycle time or timeout time left when power was switched off for pump 2
  String ACTIVE_STATE; // If system was paused or running when power was switched off, YES or NO
  uint32_t P1_BLOCK_CURRENT; // Pump 1 blockage current level in milliamps
  uint32_t P2_BLOCK_CURRENT; // Pump 2 blockage current level in milliamps
  uint8_t P1_PROX1_FLT_CNT; // Fault counter for Pump 1 Proxy 1
  uint8_t P1_PROX2_FLT_CNT; // Fault counter for Pump 1 Proxy 2
  uint8_t P1_LVL_FLT_CNT; // Fault counter for Level
  uint8_t P1_BLK_FLT_CNT; //Fault counter for blockage events
  uint8_t P1_OC_FLT_CNT;  //Fault counter for open circuit faults
  uint8_t P1_SHRT_FLT_CNT; //Fault counter for motor short circuit
  uint8_t P2_PROX1_FLT_CNT; // Fault counter for Pump 2 Proxy 1
  uint8_t P2_PROX2_FLT_CNT; // Fault counter for Pump 2 Proxy 2
  uint8_t P2_LVL_FLT_CNT; // Fault counter for Level
  uint8_t P2_BLK_FLT_CNT; //Fault counter for blockage events
  uint8_t P2_OC_FLT_CNT;  //Fault counter for open circuit faults
  uint8_t P2_SHRT_FLT_CNT; //Fault counter for motor short circuit
  uint32_t TOT_ON_TIME; //Total on time accumulator
  uint32_t TOT_P1_RUN_TIME; //Total Pump 1 run time accumulator
  uint32_t TOT_P1_PSE_TIME; //Total Pump 1 pause time accumulator
  uint32_t TOT_P2_RUN_TIME; //Total Pump 2 run time accumulator
  uint32_t TOT_P2_PSE_TIME; //Total Pump 2 pause time accumulator
  uint32_t TOT_ERR_TIME; //Total error time accumulator
  uint32_t TOT_P1_PROX1_ERR_TIME; //Total Pump1 proxy 1 error time accumulator
  uint32_t TOT_P1_PROX2_ERR_TIME; //Total Pump1 proxy 2 error time accumulator
  uint32_t TOT_P1_LVL_ERR_TIME; //Total Pump1 level error time accumulator
  uint32_t TOT_P1_SC_TIME; //Total Pump1 short circuit time accumulator
  uint32_t TOT_P1_OC_TIME; //Total Pump1 open circuit time accumulator
  uint32_t TOT_P1_BLK_TIME; //Total Pump1 overpressure time accumulator
  uint32_t TOT_P2_PROX1_ERR_TIME; //Total Pump2 proxy 1 error time accumulator
  uint32_t TOT_P2_PROX2_ERR_TIME; //Total Pump2 proxy 2 error time accumulator
  uint32_t TOT_P2_LVL_ERR_TIME; //Total Pump2 level error time accumulator
  uint32_t TOT_P2_SC_TIME; //Total Pump2 short circuit time accumulator
  uint32_t TOT_P2_OC_TIME; //Total Pump2 open circuit time accumulator
  uint32_t TOT_P2_BLK_TIME; //Total Pump2 overpressure time accumulator
  uint32_t TOT_LMP_SC_TIME; //Total error lamp short circuit time
  uint32_t TOT_SEQ_NO; //Maximum sequence number for logging
  String VER_STRING; //Version string, change to match the version of the firmware in use
};

// Default settings
Settings defaultSettings = {
  "PLS",     //P1_MAIN_MODE
  10,         //P1_PAUSE_TIME
  "TIME",    //P1_RUN_MODE
  5,         //Pump 1 P1_RUN_TIME_CYC
  10,         //Pump 1 P1_CYC_TIMEOUT
  "YES",      //P1_PROX1 NOTE: Change back to NO for deployment
  10,         //P1_PROX1_DWELL
  "NO",      //P1_PROX2 
  10,         //P1_PROX2_DWELL
  5,         //P1_PROX2_CYC
  20,         //Pump 1 Vent time
  "YES",      //Pump 1 LEVEL NOTE: Change back to NO for deployment
  "HEF",     //Pump 1 LVL_TYPE NOTE:
  "NO",      //Pump 1 LVL_NONC
  "NO",      //Pump 2 in use, YES or NO
  "PLS",     //P2_MAIN_MODE
  10,         //P2_PAUSE_TIME
  "TIME",    //P2_RUN_MODE
  5,         //Pump 2 P2_RUN_TIME_CYC
  10,         //Pump 2 P2_CYC_TIMEOUT
  "YES",      //P2_PROX1 NOTE: Change back to NO for deployment
  10,         //P2_PROX1_DWELL
  "NO",      //P2_PROX2 
  10,         //P2_PROX2_DWELL
  5,         //P2_PROX2_CYC
  20,         //Pump 2 Vent time
  "YES",      //Pump 2 LEVEL NOTE: Change back to NO for deployment
  "HEF",     //Pump 2 LVL_TYPE NOTE:
  "NO",      //Pump 2 LVL_NONC
  "YES",     //EXT_LAMP NOTE: Change back to NO for deployment
  "RGB",     //LAMP_TYP NOTE: Change back to RGB for deployment
  "STOPPED", //P1_CURR_STATE
  2300,      //P1_CURR_TIME
  "STOPPED", //P2_CURR_STATE
  2300,      //P2_CURR_TIME
  "NO",      //ACTIVE_STATE
  1000,       //Pump 1 P1_BLOCK_CURRENT
  1000,       //Pump 2 P2_BLOCK_CURRENT
  0,          //Fault counter for Pump 1 Proxy 1
  0,          //Fault counter for Pump 1 Proxy 2
  0,          //Fault counter for Pump 1 Level
  0,          //Fault counter for Pump 1 blockage events
  0,          //Fault counter for Pump 1 open circuit faults
  0,          //Fault counter for Pump 1 motor short circuit
  0,          //Fault counter for Pump 2 Proxy 1
  0,          //Fault counter for Pump 2 Proxy 2
  0,          //Fault counter for Pump 2 Level
  0,          //Fault counter for Pump 2 blockage events
  0,          //Fault counter for Pump 2 open circuit faults
  0,          //Fault counter for Pump 2 motor short circuit
  0,          //Total on time accumulator
  0,          //Total Pump 1 run time accumulator
  0,          //Total Pump 1 pause time accumulator
  0,          //Total Pump 2 run time accumulator
  0,          //Total Pump 2 pause time accumulator
  0,          //Total error time accumulator
  0,          //Total Pump1 proxy 1 error time accumulator
  0,          //Total Pump1 proxy 2 error time accumulator
  0,          //Total Pump1 level error time accumulator
  0,          //Total Pump1 short curcuit time accumulator
  0,          //Total Pump1 open circuit time accumulator
  0,          //Total Pump1 overpressure time accumulator
  0,          //Total Pump2 proxy 1 error time accumulator
  0,          //Total Pump2 proxy 2 error time accumulator
  0,          //Total Pump2 level error time accumulator
  0,          //Total Pump2 short curcuit time accumulator
  0,          //Total Pump2 open circuit time accumulator
  0,          //Total Pump2 overpressure time accumulator
  0,          //Total error lamp short circuit time
  0,          //Maximum sequence number for logging
  "0002"      //Version string
};

// Extern declaration for settings
extern Settings settings;

//WiFi variables
// Set your desired AP credentials
extern String ap_ssid_str;
extern const char* ap_ssid;
const char* ap_password = "123456789";
bool ipAcquired = false; // Set flag when IP is acquired
int wifiIPretryCount = 5; // Counter for WiFi connection retries
bool wifiEnabled = true;

#endif
