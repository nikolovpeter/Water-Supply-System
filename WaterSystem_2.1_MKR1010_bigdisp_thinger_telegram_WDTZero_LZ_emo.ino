
//Water Supply System v.2.1.
//Programer: Peter Nikolov.
//Suitable for MKR1010 but could be re-worked for Uno/Nano by changing pin assignments.
//A Water Supply System controller that controls and displays the level of water in the tank, and controls a pump, filling the tank, and a pressure pump that draws water from the tank.
//Using Dallas OneWire thermometer and rotary encoder for menu selector, 4 relays control, opto-isolator adviceable.
//Licensed under Creative Commons.
#undef min
#undef max
#include "arduino_secrets.h"

// ArduinoJson 5
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>

#include <Wire.h>
//#include <rgb_lcd.h>
//#include <WiFiSSLClient.h>
//#include <WiFi101.h>
#include <WiFiNINA.h>
//#include <TelegramBot.h>
#include <UniversalTelegramBot.h>
//#include <ThingerWifi101.h>
#include <ThingerWiFiNINA.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>    // 1-Wire temperature sensor
#include <VL53L1X.h>              // I2C laser distance sensor
//#include <ComponentObject.h>
//#include <RangeSensor.h>
//#include <SparkFun_VL53L1X.h>
//#include <vl53l1x_class.h>
//#include <vl53l1_error_codes.h>
//#include "SparkFun_VL53L1X.h"

#include <DHT.h>                  // DHT-11 temperature and humidity sensor
#include <avr/pgmspace.h>
//#include <Adafruit_SleepyDog.h>
#include <WDTZero.h>
#include <SPI.h>
#include <RTCZero.h>
#include "EmonLib.h"                   // Include Emon Library



const char* WIFI_SSID = SECRET_SSID;    //  your network SSID (name)
const char* WIFI_PASSWORD = SECRET_PSWD;  // your network password
const char BotToken[] = SECRET_BOT_TOKEN;
const char THINGER_USERNAME[] = SECRET_THINGER_USERNAME;
const char THINGER_DEVICE_ID[] = SECRET_THINGER_DEVICE_ID;
const char THINGER_DEVICE_CREDENTIAL[] = SECRET_THINGER_DEVICE_CREDENTIAL;



//Pin assignment
#define flowSensor 0             // Flow sensor (or another input that has interrupt) - do not change - needed for pin interrupt or PCI
#define ONE_WIRE_BUS 1           // OneWire interface (for thermometers) // temp - to be moved to 9
#define AmbientThermometer1Pin 2 // DHT 11 ambient thermometer 1 interface - basement
#define currentManualFillingLED 3 // Filling pump manual control LED indicator
#define overfillEffector 4       // Effector for lowering water level in case of overfill of the tank
#define fillingPump 5            // Filling pump 
#define waterDrawPump 6          // Water draw (pressure) pump permission relay
#define AmbientThermometer2Pin 9 // DHT 11 ambient thermometer 2 interface - house
#define buzzer 10                // Buzzer/beeper
#define fillingPumpACCurrent A0  // Filling pump AC current sensor (A6 is an analog input only pin on Nano)
#define doorSwitch A6            // // temp - to be moved to 2 - Door open switch (or another input that has interrupt) - do not change - needed for pin interrupt or PCI
#define encoderPinA A2           // Rotary encoder CLK 
#define encoderPinB A3           // Rotary encoder DT 
#define selectorButton 7         // Selector/pause/resume/reset button 
#define manualButton 8           // Manual control button 
#define FloatSwitchSensor A1     // Water float switch sensor = backup water level sensor


// Note: On Uno and Nano A4 and A5 are for I2C, and on Leonardo I2C is on 2 and 3.


//Communication over EasyTransferI2C
//#define I2C_SLAVE_ADDRESS 17     // Define I2C slave address on I2C bus of the receiving Arduino (this is the sending Arduino, i.e. master)


#define DHTTYPE DHT11

RTCZero rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display
//rgb_lcd lcd;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature waterThermometer(&oneWire);
DeviceAddress waterThermometerAddress;
VL53L1X laserSensor;
//SFEVL53L1X laserSensor;
DHT ambientThermometer1(AmbientThermometer1Pin, DHTTYPE);
DHT ambientThermometer2(AmbientThermometer2Pin, DHTTYPE);
//ThingerWifi101 thing(THINGER_USERNAME, THINGER_DEVICE_ID, THINGER_DEVICE_CREDENTIAL);
ThingerWiFiNINA thing(THINGER_USERNAME, THINGER_DEVICE_ID, THINGER_DEVICE_CREDENTIAL);
WiFiSSLClient wifisslclient;
//TelegramBot bot (BotToken, wifisslclient);
UniversalTelegramBot bot (BotToken, wifisslclient);
WDTZero Watchdog; // Define WDT
EnergyMonitor emon1;                   // Create an instance


// Define data classes for communication with another Arduino
struct RO_PARAMS_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint8_t currentAutoFilling;
  uint8_t currentManualFilling;
  uint8_t currentPressure;
  uint8_t fillingPumpONDuration; // Number of minutes the filling pump worked during the last totalFillCyclePeriod minutes
  uint8_t lastFillDuration; // in minutes
  uint16_t waterVolume;
  int8_t currentWaterFlow;
  int8_t waterTemperature;
  int8_t ambientTemperature1;
  int8_t ambientHumidity1;
  int8_t ambientTemperature2;
  int8_t ambientHumidity2;
  uint8_t doorSwitchState;
  uint8_t pausedState = false;
  uint8_t faultCode = 0;
  uint16_t controllerUptime; // Controller uptime in minutes
  uint8_t currentTimeHours;
  uint8_t currentTimeMinutes;
  uint8_t currentTimeSeconds;
  uint16_t currentDateYear;
  uint8_t currentDateMonth;
  uint8_t currentDateDay;
};


struct RW_COMMANDS_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint8_t debugLevel = 1;
  uint8_t pressureMode = 1; // Pressure mode = AUTO (0 = OFF; 1 = AUTO)
  uint8_t fillingMode = 1; // Filling mode = AUTO+MANUAL (0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY; 3 = MANUAL ONLY)
  uint8_t autoReset = true; // Auto reset enabled every 24 hours
  uint8_t forceReset = false;  // Reset requested
  uint8_t forcePause = false;  // Pause requested
  uint8_t forceNoChecks = false;  // NoChecks requested for 1 hour
  uint16_t minWaterVolume = 60; // Lower water level limit to start refilling
  uint16_t maxWaterVolume = 270; // Higher water level limit to stop refilling
};


//give a name to the group of data
RO_PARAMS_DATA_STRUCTURE params;
RW_COMMANDS_DATA_STRUCTURE commands, newCommands, prePauseCommands, preErrorCommands;


// General constants declaration
const PROGMEM char softwareVersion[6] = "2.1";


// Display variables declaration
const unsigned int displayDimPeriod = 300; // display dim period in seconds: 0 = no display dimming; 180 seconds = 3 minutes, 300 seconds = 5 minutes
byte currentDisplayCycle = 0;               // Counter for cycling display of different temperatures
unsigned long currentDisplayCycleTimer1;
unsigned long currentDisplayCycleTimer2;
volatile unsigned long displayDimTimer;
char displayLineString [4][21];
char initActionResult[21];
// make a custom character:
uint8_t backslash[8] = {
  0b00000,
  0b10000,
  0b01000,
  0b00100,
  0b00010,
  0b00001,
  0b00000,
  0b00000
};



// Menus, encoder and buttons variables declaration
const int resetButtonDebounceDelay = 10000; // delay to count as reset in milliseconds - 10 seconds
volatile byte selectorButtonLastState = HIGH;      // Selector/pause/resume/reset button current state
volatile byte manualButtonLastState = HIGH;        // Manual control button current state
volatile unsigned long selectorButtonLDT = 0;      // Selector/pause/resume/reset button last debounce time
volatile unsigned long manualButtonLDT = 0;        // Manual control button last debounce time
volatile bool selectorButtonCode = false;          // Selector button state
bool selectorButtonMenu = true;                    // switch case statement to control what each button push does
volatile bool manualButtonCode = false;            // Manual filling turned OFF
unsigned long forceNoChecksStart;
char setParameterNames [9][19];
const PROGMEM char mainMenuTexts [9][17] = {"Pause all procs.", "Water low limit", "Water top limit", "Filling mode   ", "Pressure mode  ",  "Auto reset mode", "No checks 1 h  ", "Factory reset  ", "Go back        "};
const PROGMEM char fillingModeText [4][4] = {"OFF", "A+M", "A", "M"};
volatile int encoderCurrentPosition = 0;
volatile int encoderLowestPosition, encoderHighestPosition, encoderStep;
volatile unsigned long menuTimer;
const unsigned long menuTimerTimeout = 30000; // Menu timer timeout in milliseconds (user menu will be cancelled if no user input for this period)



// Filling and pump variables declaration
const byte pauseBetweenFills = 0; // in minutes: 1 minute - to be changed to 30 minutes
const byte singleFillMaxDuration = 10; // Maximum allowed duration of a single filling cycle in minutes
const byte totalFillMaxDuration = 30;  // Maximum allowed duration of all filling cycles in minutes within the last totalFillCyclePeriod minutes (by default in the last 120 minutes)
const byte totalFillCyclePeriod = 120; // In minutes - cycle period to control maximum allowed filling duration (cannot be more than the bits in historyByMinutes[]) - 2 hours
byte currentHistoryPointer = 0; // Number between 0 and totalFillCyclePeriod - current pointer in the history by minutes
byte historyArrayPointer;
byte historyArrayBitPointer;
unsigned long historyByMinutes[4] = {0, 0, 0, 0}; // Array to store filling pump status by minutes for the last 2 hours (128 minutes)
unsigned long lastFillStartTime;
unsigned long lastFillStopTime;
unsigned long fillStartPausedDuration;
const byte presssureMinWaterVolume = 30; // Lower water level limit to start refilling
const int waterVolumeLimit = 290; // Water level limit to count as overfilling



// Timer and clock variables declaration

unsigned long n;
unsigned long currentMinuteTimer;
//const unsigned int watchdogRestartTime = 120; // Watchdog restart time in seconds; the system will default to maximum 16 seconds, if a larger value is given
const int GMT = 3; // Timezone offset from GMT



// Water level laser distance sensor variables declaration
const int tankCapacity = 300; // Tank capacity in litres
const int tankHeight = 120; // Tank height in centimeters
byte laserDistanceArrayIndex = 0;
byte laserDistanceArraySize = 30;
int laserDistanceArray[30]; // Array of laser distance measurements in cm, size should be = laserDistanceArraySize
int laserDistance_mtbs = 200; //mean time between water level measurements in milliseconds
unsigned long laserDistance_lasttime = 0;   //last time laser distance has been measured


// Flow sensor variables declaration
volatile int pulsecount;         // Variable to count number of pulses from flow sensor
const int noFlowTreshold = 30; // No flow treshold in seconds
unsigned long noFlowTimer = 0;


// Temperature monitoring variables declaration
const int freezeAlarmLimit = 2; // Lowest temperature limit to count for freezing alarm
int temperature_mtbs = 200; //mean time between water level measurements in milliseconds
unsigned long temperature_lasttime = 0;   //last time laser distance has been measured


// AC Current sensor variables
int ACsensor_mtbs = 3000; //mean time between water level measurements in milliseconds
unsigned long ACsensor_lasttime = 0;   //last time laser distance has been measured



// Pause-resume functions variables declaration
volatile unsigned long stopTime = 0;       // Record time program was paused
volatile unsigned long pausedDuration = 0; // Record how long the program was paused for
volatile byte prepausedStatefillingPump;
volatile byte prepausedStatewaterDrawPump;
volatile byte prepausedStatecurrentManualFillingLED;
volatile byte prepausedStateoverfillEffector;


// Communication variables declaration
volatile bool newData = false;
volatile unsigned long dataTransferTimer;
byte forcePeerReset = false;
int WiFiStatus = WL_IDLE_STATUS;     // the Wifi radio's status
bool WiFiConnectionAlive = false;
bool InternetConnectionAlive = false;
bool ThingerConnectionAlive = false;
byte forceAllReset = false;
char lastSystemStatus [200];
char lastOperationStatus [250];
char lastEnvironmentStatus [150];
char lastControlStatus [200];
char lastDateTimeStatus [150];
//String OldChatId = "";
bool localFeedback = true;
int Bot_mtbs = 10000; //mean time between scan messages in milliseconds
unsigned long Bot_sleeptime = 60000; // time to put bot to sleep by increasing Bot_mtbs
unsigned long Bot_lasttime;   //last time messages' scan has been done
unsigned long Bot_lastMessage;   //last time a message has been received
int Thinger_mtbs = 1000; //mean time between scan messages in milliseconds
unsigned long Thinger_sleeptime = 60000; // time to put Thinger to sleep by increasing Thinger_mtbs
unsigned long Thinger_lasttime;   //last time messages' scan has been done
unsigned long Thinger_lastMessage;   //last time a message has been received


// Error reporting variables declaration
const PROGMEM char faultDescriptionArray[8][22] {"Water t-sensor err.", "Amb. t-sensor err.", "LD sensor err.", "No Fpump water flow.", "Freezing risk!", "Overfill!", "Filling too long.", "General error."};
const PROGMEM char faultActionArray[8][4] {"PPP", "PPP", "PFP", "0F0", "0FP", "PF0", "PF0", "PFP"};
bool faultFillingOff = false;
bool faultPressureOff = false;
byte lastFaultCode = 0;
char binaryfaultCodeString[9];
char faultDescription [280] = "";
byte positionCounter = 0;


//Timing actions


void TimeElapsedFunction() { // Time counter
  if ((millis() - currentMinuteTimer) >= 60000) { // Check if a full minute has elapsed
    if (commands.debugLevel > 1) Serial.println(F(" === TimeElapsedFunction started... ==="));
    currentMinuteTimer = millis(); // Initialize minute timer
    historyArrayPointer = currentHistoryPointer / 32;
    historyArrayBitPointer = currentHistoryPointer - (historyArrayPointer * 32);
    bitWrite(historyByMinutes[historyArrayPointer], historyArrayBitPointer, digitalRead (fillingPump));
    if (currentHistoryPointer > totalFillCyclePeriod) currentHistoryPointer = 0;
    historyArrayPointer = 0;
    params.fillingPumpONDuration = 0;
    for (historyArrayPointer = 0; historyArrayPointer < 3; historyArrayPointer++) {
      n = historyByMinutes[historyArrayPointer];
      while (n) {
        n &= (n - 1);
        params.fillingPumpONDuration++;
      }
    }
    currentHistoryPointer++; // Increase history pointer with 1 minute
    checkConnectionToWiFi();
    getEpochFromInternet();
    getDateAndTime();
    generateStatusStrings();
    printStatusStrings();
  }
}



void generateStatusStrings() {
  snprintf_P(lastSystemStatus, sizeof(lastSystemStatus), PSTR(" == Water volume: %d L; Water flow: %d L/min; Water temperature: %d°C; Last fill duration: %d min; Filling pump uptime: %d min/2h. =="), params.waterVolume, params.currentWaterFlow, params.waterTemperature, params.lastFillDuration, params.fillingPumpONDuration);
  snprintf_P(lastEnvironmentStatus, sizeof(lastEnvironmentStatus), PSTR(" == Amb. temp. 1: %d°C; Amb. hum. 1: %d%%; Amb. temp. 2: %d°C; Amb. hum. 2: %d%%. =="), params.ambientTemperature1, params.ambientHumidity1, params.ambientTemperature2, params.ambientHumidity2);
  snprintf_P(lastOperationStatus, sizeof(lastOperationStatus), PSTR(" == Pressure pump status: %d; Current auto filling: %d; Current manual filling: %d; Controller uptime: %d min; Paused state: %d; Mannual override mode: %d; Auto reset mode: %d; Door switch state: %d; Fault code: %d. =="), params.currentPressure, params.currentAutoFilling, params.currentManualFilling, params.controllerUptime, params.pausedState, commands.forceNoChecks, commands.autoReset, params.doorSwitchState, params.faultCode);
  snprintf_P(lastControlStatus, sizeof(lastControlStatus), PSTR(" == Pressure mode: %d; Filling mode: %d; Force pause: %d; Force reset: %d; commands.minWaterVolume: %d L; commands.maxWaterVolume: %d L. =="), commands.pressureMode, commands.fillingMode, commands.forcePause, commands.forceReset, commands.minWaterVolume, commands.maxWaterVolume);
  snprintf_P(lastDateTimeStatus, sizeof(lastDateTimeStatus), PSTR(" == Current date and time: %02d.%02d.%04d - %02d:%02d:%02d. =="), params.currentDateDay, params.currentDateMonth, params.currentDateYear, params.currentTimeHours, params.currentTimeMinutes, params.currentTimeSeconds);
}



void printStatusStrings() {
  Serial.print(F(" == Minutes worked: "));
  Serial.println(params.controllerUptime);
  Serial.println(lastSystemStatus);
  Serial.println(lastEnvironmentStatus);
  Serial.println(lastOperationStatus);
  if (params.faultCode) Serial.println(faultDescription);
  Serial.println(lastControlStatus);
  Serial.println(lastDateTimeStatus);
}


void getDateAndTime() {
  params.currentTimeHours = rtc.getHours();
  params.currentTimeMinutes = rtc.getMinutes();
  params.currentTimeSeconds = rtc.getSeconds();
  params.currentDateYear = rtc.getYear() + 2000;
  params.currentDateMonth = rtc.getMonth();
  params.currentDateDay = rtc.getDay();
}



//User interface actions


void UserMenuFunction() { // User menu
  int menuSelectorValue = 0;
  lcd.backlight();
  Watchdog.clear();
  //  Watchdog.disable();
  //  int setParameterValue = 0;  // Returned parameter value number
  //  int setParameterCount = 0;  // Number of parameter values to choose from (should not be more than 20)
  shortBeep();
  lcd.setCursor(0, 2);
  lcd.print(F("Menu:               "));
  delay (100);
  for (byte i = 0; i < 9; i++) {
    snprintf_P(setParameterNames[i], sizeof(setParameterNames[i]), mainMenuTexts[i]);
  }
  menuSelectorValue = setParameterFunction(9, 8, 0, ">", "");
  strcpy (displayLineString[2], displayLineString[3]);
  lcd.setCursor(0, 2);
  lcd.print(displayLineString[2]);
  switch (menuSelectorValue) {
    case 0: // Start pause
      shortBeep();
      commands.forcePause = true;
      selectorButtonCode = 0;
      return;
      break;
    case 1: // Edit lowest water level limit
      shortBeep();
      commands.minWaterVolume = setDigitalValueFunction(commands.minWaterVolume + 5, commands.maxWaterVolume - 5, commands.minWaterVolume, 5, ">>Water low: ", "L");
      selectorButtonCode = 0;
      Serial.print(F("commands.minWaterVolume set manually to ")); //temp
      Serial.print(commands.minWaterVolume); //temp
      Serial.println("L."); //temp
      return;
      break;
    case 2: // Edit highest water level limit
      shortBeep();
      commands.maxWaterVolume = setDigitalValueFunction(commands.minWaterVolume + 5, waterVolumeLimit - 5, commands.maxWaterVolume, 5, ">>Water high: ", "L");
      selectorButtonCode = 0;
      Serial.print(F("commands.maxWaterVolume set manually to ")); //temp
      Serial.print(commands.maxWaterVolume); //temp
      Serial.println("L."); //temp
      return;
      break;
    case 3: // Edit filling pump mode: 0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("Fill:OFF"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("Fill:AUTO+MANUAL"));
      snprintf_P(setParameterNames [2], sizeof(setParameterNames [2]), PSTR("Fill:AUTO ONLY"));
      snprintf_P(setParameterNames [3], sizeof(setParameterNames [3]), PSTR("Fill:MANUAL ONLY"));
      commands.fillingMode = setParameterFunction(4, commands.fillingMode, commands.fillingMode, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("commands.fillingMode set manually to ")); //temp
      Serial.println(commands.fillingMode); //temp
      return;
      break;
    case 4: // Edit pressure pump mode: 0 = OFF; 1 = AUTO
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("Pressure:OFF"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("Pressure:AUTO"));
      commands.pressureMode = setParameterFunction(2, commands.pressureMode, commands.pressureMode, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("commands.pressureMode set manually to ")); //temp
      Serial.println(commands.pressureMode); //temp
      return;
      break;
    case 5: // Edit auto reset mode: 0 = OFF; 1 = ON = restart every 24 h.
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("commands.autoReset:OFF"));
      //                                                                     01234567890123456789
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("commands.autoReset:every24h"));
      commands.autoReset = setParameterFunction(2, commands.autoReset, commands.autoReset, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("commands.autoReset mode set manually to ")); //temp
      Serial.println(commands.autoReset); //temp
      return;
      break;
    case 6: // Edit force no checks mode: 0 = OFF; 1 = ON = no checks for 1 h.
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("All checks ON"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("No checks for 1 h"));
      menuSelectorValue = setParameterFunction(2, commands.forceNoChecks, commands.forceNoChecks, ">>", "");
      selectorButtonCode = 0;
      if (menuSelectorValue == 1) {
        longBeep();
        commands.forceNoChecks = true;
        return;
      }
      else {
        commands.forceNoChecks = false;
        Serial.print(F("commands.forceNoChecks set manually to ")); //temp
        Serial.println(commands.forceNoChecks); //temp
      }
      break;
    case 7:  // Factory reset.
      longBeep();
      lcd.setCursor(0, 2);
      lcd.print(F("Reset system ? "));
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("No "));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("Yes"));
      menuSelectorValue = setParameterFunction(2, 0, 0, ">> Confirm ?:", "");
      if (menuSelectorValue == 1) {
        veryLongBeep();
        ResetFunction();
      }
    case 8:  // Do nothing and exit menu.
      break;
  }
  selectorButtonCode = 0;
  shortBeep();
}



void startForceNoChecks () {
  params.faultCode = 0;
  commands.forceNoChecks = true;
  forceNoChecksStart = millis();
  Serial.print(F("commands.forceNoChecks set manually to ")); //temp
  Serial.println(commands.forceNoChecks); //temp
}



int setParameterFunction (int parameterCount, int defaultParameterValue, int initialParameterValue, char textBefore[17], char textAfter[17]) {  // Select a menu item - names of values should be pre-set in the string array setParameterNames[]
  selectorButtonCode = false;
  selectorButtonMenu = false;
  encoderLowestPosition = 0;
  encoderHighestPosition = parameterCount - 1;
  encoderCurrentPosition = initialParameterValue;
  encoderStep = 1;
  menuTimer = millis();
  while (!selectorButtonCode && ((millis() - menuTimer) < menuTimerTimeout)) { //  !!!!! potentially blocking cycle, so protected with inactivity timeout
    snprintf_P (displayLineString[3], sizeof(displayLineString[3]), PSTR("%s%s%s"), textBefore, setParameterNames[encoderCurrentPosition], textAfter);
    while (strlen(displayLineString[3]) < 20) strcat (displayLineString[3], " ");
    displayLineString[3][20] = 0;
    displayFillingLine ();
    displayPressureLine ();
    displayLines();
  }
  shortBeep();
  selectorButtonCode = false;
  selectorButtonMenu = true;
  if (millis() - menuTimer >= menuTimerTimeout) return defaultParameterValue; else return encoderCurrentPosition;
}



int setDigitalValueFunction (int minValue, int maxValue, int defaultValue, int valueStep, char textBefore[17], char textAfter[17]) { //
  selectorButtonCode = 0;
  selectorButtonMenu = false;
  encoderLowestPosition = minValue;
  encoderHighestPosition = maxValue;
  encoderCurrentPosition = defaultValue;
  encoderStep = valueStep;
  menuTimer = millis();
  while (!selectorButtonCode && ((millis() - menuTimer) < menuTimerTimeout)) { //  !!!!! potentially blocking cycle, so protected with inactivity timeout
    snprintf_P (displayLineString[3], sizeof(displayLineString[3]), PSTR("%s%d%s"), textBefore, encoderCurrentPosition, textAfter);
    while (strlen(displayLineString[3]) < 20) strcat (displayLineString[3], " ");
    displayLineString[3][20] = 0;
    displayFillingLine ();
    displayPressureLine ();
    displayLines();
  }
  shortBeep();
  selectorButtonCode = false;
  selectorButtonMenu = true;
  if (millis() - menuTimer >= menuTimerTimeout) return defaultValue; else return encoderCurrentPosition;
}



void encoderISR () {
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB)) {
    noInterrupts();
    if ((encoderCurrentPosition + encoderStep) <= encoderHighestPosition) encoderCurrentPosition += encoderStep;
  }
  else {
    if ((encoderCurrentPosition - encoderStep) >= encoderLowestPosition) encoderCurrentPosition -= encoderStep;
  }
  tone (buzzer, 600, 20); // Buzz for 20 milliseconds with frequency 600 Hz
  displayDimTimer = millis();
  menuTimer = millis();
  interrupts();
}



void selectorButtonISR() { // Read buttons and take actions as needed
  bool selectorButtonReading = digitalRead (selectorButton);
  bool manualButtonReading = digitalRead (manualButton);
  if (selectorButtonLastState == LOW && ((millis() - selectorButtonLDT)) >= resetButtonDebounceDelay) { // Selector button held down long enough to count as Reset
    commands.forceReset = true;
    NVIC_SystemReset(); // Called explicitly from here
  }
  if (selectorButtonReading == LOW && manualButtonReading == LOW) commands.forceNoChecks = true;
  //  noInterrupts();
  if (selectorButtonReading != selectorButtonLastState) {
    if (selectorButtonReading == LOW) selectorButtonCode = !selectorButtonCode; // If selector button held down after suffcient time to count as normal press, and then released, increment selectorButtonCode
    selectorButtonLastState = selectorButtonReading;
    selectorButtonLDT = millis();
    displayDimTimer = millis();
    if (selectorButtonCode) tone (buzzer, 600, 80); // Buzz for 80 milliseconds with frequency 600 Hz
  }
  //  interrupts();
}



void manualButtonISR() { // Read buttons and take actions as needed
  bool manualButtonReading = digitalRead (manualButton);
  bool selectorButtonReading = digitalRead (selectorButton);
  if (manualButtonLastState == LOW && ((millis() - manualButtonLDT)) >= resetButtonDebounceDelay) {
    commands.forceReset = true; // Manual button held down long enough to count as Reset
    NVIC_SystemReset(); // Called explicitly from here
  }
  if (selectorButtonReading == LOW && manualButtonReading == LOW) commands.forceNoChecks = true;
  if (manualButtonReading != manualButtonLastState) {
    if (manualButtonReading == LOW) manualButtonCode = !manualButtonCode; // If manual button held down after suffcient time to count as normal press, invert manualButtonCode
    manualButtonLastState = manualButtonReading;
    manualButtonLDT = millis();
    displayDimTimer = millis();
    if (manualButtonCode) {
      tone (buzzer, 700, 250); // Buzz for 250 milliseconds with frequency 700 Hz
      delay (250);
      tone (buzzer, 700, 250); // Buzz for 250 milliseconds with frequency 700 Hz
      delay (250);
      tone (buzzer, 700, 250); // Buzz for 250 milliseconds with frequency 700 Hz
    }
    else {
      tone (buzzer, 700, 80); // Buzz for 80 milliseconds with frequency 700 Hz - new data received from remote controller
      delay (120);
      tone (buzzer, 700, 80); // Buzz for 80 milliseconds with frequency 700 Hz - new data received from remote controller
      delay (120);
      //      params.currentManualFilling = false;
      FillingPumpOFFAction();
      delay (1000);
    }
  }
}



void doorSwitchFunction() { // Read buttons and take actions as needed
  if (digitalRead(doorSwitch) == LOW) {
    params.doorSwitchState = true;
    Serial.println(F("Basement door open!")); //temp
    veryLongBeep();
    shortBeep();
  }
  else params.doorSwitchState = false;
}



//Display actions



void scrollDisplayUp() {
  strcpy (displayLineString[0], displayLineString[1]);
  strcpy (displayLineString[1], displayLineString[2]);
  strcpy (displayLineString[2], displayLineString[3]);
}



void displayLines() {
  lcd.setCursor(0, 0);
  lcd.print(displayLineString[0]);
  lcd.setCursor(0, 1);
  lcd.print(displayLineString[1]);
  lcd.setCursor(0, 2);
  lcd.print(displayLineString[2]);
  lcd.setCursor(0, 3);
  lcd.print(displayLineString[3]);
}



void displayCurrentStatus() {
  if (commands.debugLevel > 1) Serial.println(F(" === displayCurrentStatus started... ==="));
  if ((millis() - displayDimTimer) >= (displayDimPeriod * 1000) && (displayDimPeriod != 0)) lcd.noBacklight(); else lcd.backlight();
  if ((millis() - currentDisplayCycleTimer1) >= 1000) { // Check if a full second has elapsed
    currentDisplayCycleTimer1 = millis();
    strcpy(displayLineString[0], "");
    strcpy(displayLineString[1], "");
    strcpy(displayLineString[2], "");
    strcpy(displayLineString[3], "");
    displayFillingLine ();
    displayPressureLine ();
    switch (currentDisplayCycle) { // Cycle display
      case 0: // Display cycle 0
        if (params.pausedState) snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("PAUSED: key->resume "));
        else if (params.currentAutoFilling || params.currentManualFilling) displayFlowDurationLine(); else displayIdleAnimation();
        displayAmbientLine();
        break;
      case 1: // Display cycle 1
        if (params.pausedState) snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("PAUSED: key->resume "));
        else if (params.currentAutoFilling || params.currentManualFilling) displayFlowDurationLine(); else displayIdleAnimation();
        displayAmbientLine();
        break;
      case 2: // Display cycle 2
        displayIdleAnimation();
        displayWiFiStatus();
        break;
      case 3: // Display cycle 3
        if (params.faultCode != 0) displayFaultCode(); else displayIdleAnimation();
        displayWiFiStatus();
        break;
    }
    displayLines();
    if (!params.currentAutoFilling && !params.currentManualFilling && !params.pausedState && !params.faultCode && (currentDisplayCycle == 3)) {
      lcd.setCursor(0, 2);
      lcd.write(1);
    }
    currentDisplayCycle++;
    if (currentDisplayCycle > 3) currentDisplayCycle = 0;
  }
  if (params.faultCode != 0 && ((millis() - currentDisplayCycleTimer2) >= 200)) {
    strcpy (displayLineString[3], "");
    for (byte i = 0; i < 20; i++) {
      if ((positionCounter + i) < strlen(faultDescription)) {
        displayLineString[3][i] = faultDescription[positionCounter + i];
      }
      else {
        displayLineString[3][i] = ' ';
      }
    }
    displayLineString[3][20] = 0;
    if (positionCounter < strlen(faultDescription)) positionCounter++; else positionCounter = 0;
    lcd.setCursor(0, 3);
    lcd.print (displayLineString[3]);
    currentDisplayCycleTimer2 = millis();
  }
}



void displayFillingLine () { // char displayLineString[0][21];     //Fill:A+M nowOFF 200L
  strcpy_P (displayLineString[0], PSTR("Fill"));
  if (commands.forceNoChecks) strcat_P(displayLineString[0], PSTR("!")); else strcat_P(displayLineString[0], PSTR(":"));
  strcat_P (displayLineString[0], fillingModeText [commands.fillingMode]);
  if (!params.currentAutoFilling && !params.currentManualFilling) {
    strcat_P(displayLineString[0], PSTR("_nowOFF"));
  }
  else {
    if (params.currentAutoFilling)   strcat_P(displayLineString[0], PSTR("_nowONa"));
    if (params.currentManualFilling) strcat_P(displayLineString[0], PSTR("_nowONm"));
  }
  while (strlen(displayLineString[0]) < 16) strcat (displayLineString[0], " ");
  char temporaryString [5] = "";
  snprintf_P(temporaryString, sizeof(temporaryString), PSTR("%3dL\n"), params.waterVolume);
  strcat(displayLineString[0], temporaryString);
  displayLineString[0][20] = 0;
}



void displayPressureLine () { // char displayLineString[0][21];     //Pres:AUTO nowOFF 15o
  strcpy_P (displayLineString[1], PSTR("Pres"));
  if (commands.forceNoChecks) strcat_P(displayLineString[1], PSTR("!")); else strcat_P(displayLineString[1], PSTR(":"));
  if (commands.pressureMode) strcat_P(displayLineString[1], PSTR(("AUTO"))); else strcat_P(displayLineString[1], PSTR("OFF"));
  if (params.currentPressure) strcat_P(displayLineString[1], PSTR("_nowON")); else strcat_P(displayLineString[1], PSTR("_nowOFF"));
  while (strlen(displayLineString[1]) < 17) strcat (displayLineString[1], " ");
  char temporaryString [5] = "";
  snprintf_P(temporaryString, sizeof(temporaryString), PSTR("%2do\n"), params.waterTemperature);
  temporaryString[2] = 223;
  strcat(displayLineString[1], temporaryString);
  displayLineString[1][20] = 0;
}



void displayFlowDurationLine () {  // char displayLineString[3][21];     // mFl:35L/min Dur:38'  or  FaultDescription or Submenu Item
  char temporaryString [17] = "";
  strcpy_P(displayLineString[2], PSTR("Flow"));
  if (commands.forceNoChecks) strcat(displayLineString[2], "!"); else strcat(displayLineString[2], ":");
  unsigned int currentFillDuration = (millis() - lastFillStartTime - (pausedDuration - fillStartPausedDuration)) / 60000;
  snprintf_P(temporaryString, sizeof(temporaryString), PSTR("%2dL/min Dur:%2d'\n"), params.currentWaterFlow,  currentFillDuration);
  strcat(displayLineString[2], temporaryString);
  displayLineString[2][20] = 0;
}


void displayIdleAnimation () {  // char displayLineString[3][21];     // mFl:35L/min Dur:38'  or  FaultDescription or Submenu Item
  byte hours, minutes;
  char timeElapsedString[20];
  hours = params.controllerUptime / 60;
  minutes = params.controllerUptime % 60;
  getDateAndTime ();
  switch (currentDisplayCycle) { // Cycle display
    case 0:
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("|"));
      snprintf_P (timeElapsedString, sizeof(timeElapsedString), PSTR("%02d.%02d.%04d %02d:%02d:%02d"), params.currentDateDay, params.currentDateMonth, params.currentDateYear, params.currentTimeHours, params.currentTimeMinutes, params.currentTimeSeconds);
      strcat (displayLineString[2], timeElapsedString);
      break;
    case 1:
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("/"));
      snprintf_P (timeElapsedString, sizeof(timeElapsedString), PSTR(" Time worked: %02d:%02d "), hours, minutes);
      strcat (displayLineString[2], timeElapsedString);
      break;
    case 2:
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("-"));
      snprintf_P (timeElapsedString, sizeof(timeElapsedString), PSTR("%02d.%02d.%04d %02d:%02d:%02d"), params.currentDateDay, params.currentDateMonth, params.currentDateYear, params.currentTimeHours, params.currentTimeMinutes, params.currentTimeSeconds);
      strcat (displayLineString[2], timeElapsedString);
      break;
    case 3:
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR(" "));
      snprintf_P (timeElapsedString, sizeof(timeElapsedString), PSTR(" Time worked: %02d:%02d "), hours, minutes);
      strcat (displayLineString[2], timeElapsedString);
      break;
  }
  displayLineString[2][20] = 0;
}



void displayAmbientLine () { // char displayLineString[1][21];     // Amb:22/24o Hu:67/43%
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Amb:%2d|%2do Hu:%2d|%2d%%"), params.ambientTemperature1, params.ambientTemperature2, params.ambientHumidity1, params.ambientHumidity2);
  displayLineString[3][9] = 223;
  //  displayLineString[3][19] = 37;
  displayLineString[3][20] = 0;
}



void displayFaultCode () {
  snprintf_P(displayLineString[2], sizeof(displayLineString[2]), PSTR("ERROR %3d:"), params.faultCode);
  strcat (displayLineString[2], binaryfaultCodeString);
  while (strlen(displayLineString[2]) < 20) strcat (displayLineString[2], " ");
  displayLineString[2][20] = 0;
  snprintf(displayLineString[3], sizeof(displayLineString[3]), faultDescription);
  while (strlen(displayLineString[3]) < 20) strcat (displayLineString[3], " ");
  displayLineString[3][20] = 0;
  //  currentDisplayCycleTimer2 = millis();
}



void displayWiFiStatus () {
  char internetAlive[6];
  if (InternetConnectionAlive) snprintf_P(internetAlive, sizeof(internetAlive), PSTR("[OK]")); else snprintf_P(internetAlive, sizeof(internetAlive), PSTR("[NoI]"));
  if (WiFiConnectionAlive == true) {
    snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("WiFi: %s%s"), WiFi.SSID(), internetAlive);
  }
  else {
    snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("WiFi disconnected..."));
  }
  while (strlen(displayLineString[3]) < 20) strcat (displayLineString[3], " ");
  displayLineString[3][20] = 0;
}



void shortBeep() {
  tone (buzzer, 700, 80); // Buzz for 80 milliseconds with frequency 700 Hz - new data received from remote controller
  delay (120);
}


void longBeep() {
  tone (buzzer, 700, 150); // Buzz for 80 milliseconds with frequency 700 Hz - new data received from remote controller
  delay (120);
}


void veryLongBeep() {
  tone (buzzer, 700, 250); // Buzz for 80 milliseconds with frequency 700 Hz - new data received from remote controller
  delay (120);
}


// Sensor reading actions


void initLaserDistanceSensor () { // In case using #include <VL53L1X.h>
  Serial.println(F("##### initLaserDistanceSensor started #####"));
  Wire.begin(); // Initialize I2C as master
  bool initSuccess = false;
  int laserDistance;
  strcpy (initActionResult, "");
  laserSensor.setTimeout(500);
  if (!laserSensor.init()) { //Begin returns 0 on a good init
    delay (100);
    if (!laserSensor.init()) { //Begin returns 0 on a good init - second try
      delay (100);
      if (!laserSensor.init()) { //Begin returns 0 on a good init - third try
        params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
        snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor....[Fail!]"));
        Serial.println(F("##### Failed to init LD sensor after 3 attempts! #####"));
      }
      else initSuccess = true;
    }
    else initSuccess = true;
  }
  else initSuccess = true;
  if (initSuccess) {
    laserSensor.setDistanceMode(VL53L1X::Long);
    laserSensor.setMeasurementTimingBudget(50000); // Measurement timing budget in microseconds
    laserSensor.startContinuous(100); // Interval between measurements in milliseconds
    params.faultCode &= 0b11111011; // Clear bit 3 of faultCode = no laser distance sensor error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor.......[OK]"));
    Serial.println(F("##### LD sensor intialized: OK. Starting array init... #####"));
    for (byte i = 0; i < (laserDistanceArraySize - 1); i++) { // Initialize laser distance array
      laserDistance = readLaserDistance();
      int laserDistance2 = readLaserDistance();
      int laserDistance3 = readLaserDistance();
      if (laserDistance != -1) {
        laserDistanceArray[i] = ((laserDistance + laserDistance2 + laserDistance3) / 3) / 10;
      }
    }
    laserDistance = readLaserDistance();
    if (laserDistance != -1) {
      laserDistanceArray [laserDistanceArrayIndex] = laserDistance / 10;
    }
  }
  Serial.println(F("##### LD sensor and array init finished. #####"));
}



int readLaserDistance() { // In case using #include #include <VL53L1X.h>
  if (commands.debugLevel > 1 || (params.faultCode & 0b00100100) != 0) Serial.println(F("    ====== readLaserDistance started... ======"));
  bool laserDistanceTimeout = false;
  unsigned long measurementTimer = millis();
  if (commands.debugLevel > 2 || (params.faultCode & 0b00100100) != 0) Serial.println(F("    ====== readLaserDistance : checkForDataReady... ======"));
  laserSensor.setTimeout(500);
  while (!laserSensor.dataReady()) {
    delay(1);
    if (millis() - measurementTimer >= 500) {
      laserDistanceTimeout = true;
      break;
    }
  }
  if (laserSensor.dataReady()) {
    if (commands.debugLevel > 2 || (params.faultCode & 0b00100100) != 0) Serial.println(F("    ====== readLaserDistance : start reading... ======"));
    laserSensor.read(false); // Read the sensor in non-blocking mode
    if (!laserSensor.timeoutOccurred()) {
      params.faultCode &= 0b11111011; // Clear bit 2 of faultCode = no laser distance sensor error
      int distance = laserSensor.ranging_data.range_mm; //Get the result of the measurement from the sensor
      if (commands.debugLevel > 2 || (params.faultCode & 0b00100100) != 0) {
        Serial.print (F("    ======                       Laser distance measured: "));
        Serial.print (distance);
        Serial.println (F(" mm         ======"));
      }
      return distance;
    }
    else laserDistanceTimeout = true;
  }
  if (laserDistanceTimeout) {
    params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
    Serial.println(F("    ====== Laser distance sensor error: Timeout occurred. ======"));
    return -1;
  }
}



//void initLaserDistanceSensor () { // In case using #include <SparkFun_VL53L1X.h>
//  Serial.println(F("===initLaserDistanceSensor started==="));
//  Wire.begin(); // Initialize I2C as master
//  bool initSuccess = false;
//  int laserDistance;
//  strcpy (initActionResult, "");
//  if (laserSensor.begin() != 0) { //Begin returns 0 on a good init
//    delay (100);
//    if (laserSensor.begin() != 0) { //Begin returns 0 on a good init - second try
//      delay (100);
//      if (laserSensor.begin() != 0) { //Begin returns 0 on a good init - third try
//        params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
//        snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor....[Fail!]"));
//        Serial.println(F("Failed to init LD sensor after 3 attempts!"));
//      }
//      else initSuccess = true;
//    }
//    else initSuccess = true;
//  }
//  else initSuccess = true;
//  if (initSuccess) {
//    laserSensor.setDistanceModeLong();
//    if (commands.debugLevel>2) Serial.println(laserSensor.getIntermeasurementPeriod());
//    params.faultCode &= 0b11111011; // Clear bit 3 of faultCode = no laser distance sensor error
//    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor.......[OK]"));
//    Serial.println(F("LD sensor intialized: OK."));
//    for (byte i = 0; i < (laserDistanceArraySize - 1); i++) { // Initialize laser distance array
//      laserDistance = readLaserDistance();
//      int laserDistance2 = readLaserDistance();
//      int laserDistance3 = readLaserDistance();
//      if (laserDistance != -1) {
//        laserDistanceArray[i] = ((laserDistance + laserDistance2 + laserDistance3) / 3) / 10;
//      }
//    }
//    laserDistance = readLaserDistance();
//    if (laserDistance != -1) {
//      laserDistanceArray [laserDistanceArrayIndex] = laserDistance / 10;
//    }
//  }
//}
//
//
//
//int readLaserDistance() { // In case using #include <SparkFun_VL53L1X.h>
//  if (commands.debugLevel>1) Serial.println(F("=== readLaserDistance started. ==="));
//  bool laserDistanceTimeout = false;
//  int distance;
//  unsigned long measurementTimer = millis();
//  if (commands.debugLevel>2) Serial.println(F("=== readLaserDistance : startRanging... "));
//  laserSensor.startRanging(); //Write configuration bytes to initiate measurement
//  if (commands.debugLevel>2) Serial.println(F("=== readLaserDistance : checkForDataReady... "));
//  while (!laserSensor.checkForDataReady()) {
//    delay(1);
//    if (millis() - measurementTimer >= 500) {
//      laserDistanceTimeout = true;
//      break;
//    }
//  }
//  if (laserDistanceTimeout) {
//    params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
//    Serial.println(F("Laser distance sensor error: Timeout occurred."));
//    return -1;
//  }
//  else {
//    params.faultCode &= 0b11111011; // Clear bit 2 of faultCode = no laser distance sensor error
//    if (commands.debugLevel>2) Serial.println(F("=== readLaserDistance : getDistance... "));
//    distance = laserSensor.getDistance(); //Get the result of the measurement from the sensor
//    laserSensor.clearInterrupt();
//    laserSensor.stopRanging();
//    if (commands.debugLevel>1) {
//      Serial.print (F("======================= Laser distance measured: "));
//      Serial.print (distance);
//      Serial.println (F(" mm ======================="));
//    }
//  }
//  return distance;
//}



void readWaterVolume() { // Read current water level in litres
  if (millis() > laserDistance_lasttime + laserDistance_mtbs)  {
    if (commands.debugLevel || (params.faultCode & 0b00100100) != 0)  Serial.println(F("   ===== Read water volume started... ====="));
    int currentWaterVolume = 0;
    int laserDistance = readLaserDistance();
    if (laserDistance == 0 || laserDistance == -1) {
      Serial.println(F("   ===== Read water volume error occurred - trying to reset LD sensor... ====="));
      initLaserDistanceSensor();
    }
    else {
      laserDistanceArray[laserDistanceArrayIndex] = laserDistance / 10; // Store distance in centimeters
      laserDistanceArrayIndex++;
      if (laserDistanceArrayIndex > laserDistanceArraySize) laserDistanceArrayIndex = 0;
      for (byte i = 0; i < laserDistanceArraySize - 1; i++) {
        currentWaterVolume += tankCapacity - round((tankCapacity * 1.0) * (laserDistanceArray[i] * 1.0 / tankHeight * 1.0));
      }
      params.waterVolume = (currentWaterVolume / laserDistanceArraySize);
      laserDistance_lasttime = millis();
      if (commands.debugLevel > 1 || (params.faultCode & 0b00100100) != 0)  Serial.println(F("   ===== Read water volume completed successfully. ====="));
    }
  }
}



void initEmonCurrentSensor () {
  Serial.println(F("##### AC current sensor init started... #####"));
  emon1.current(fillingPumpACCurrent, 60.6);             // Current: input pin, calibration.
  //  emon1.current(fillingPumpACCurrent, 111.1);             // Current: input pin, calibration.
  strcpy (initActionResult, "");
  //                                                           01234567890123456789
  snprintf_P(initActionResult, sizeof(initActionResult), PSTR("AC current snsr.[OK]"));
  Serial.println(F("##### AC current sensor intialized. #####"));
}



double readEmonCurrentSensor () {
  if (millis() > ACsensor_lasttime + ACsensor_mtbs)  {
    if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.print(F("   =====                      AC current reading: "));
    double Irms = emon1.calcIrms(1480);  // Calculate Irms only
    //  Irms = Irms * 27.0;
    if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.print(Irms);          // Irms
    if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.print(" Amp; apparent power: ");
    if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.print(Irms * 230.0);       // Apparent power
    if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.println(" W      =====");
    ACsensor_lasttime = millis();
    return Irms;
  }
}




void readWaterFlow() {  // Read water flow sensor and return flow in L/min
  if (commands.debugLevel || (params.faultCode & 0b01001000) != 0) Serial.println(F("   ===== readWaterFlow started ====="));
  pulsecount = 0;
  interrupts();
  delay (500);    //Wait 1/2 second
  noInterrupts();  //Disable the interrupt
  //Half a second is over now and we have the number of pulses in variable 'pulsecount' updated by the ISR
  //Calculating the water flow rate in liters per minute
  params.currentWaterFlow = round((pulsecount * 2 * 10) / 48); //flowRate in L/min (to be calibrated)
  interrupts();
}


void flowSensorISR() { // ISR for input pin 7 interrupt
  pulsecount++;
}



void initWaterTemperatureSensor () {
  Serial.println(F("##### initWaterTemperatureSensor started #####"));
  strcpy (initActionResult, "");
  if (!waterThermometer.isConnected(waterThermometerAddress)) {
    waterThermometer.begin(); // Start initializing water thermometer
    waterThermometer.getAddress(waterThermometerAddress, 0);
    if (!waterThermometer.getAddress(waterThermometerAddress, 0)) { // Water thermometer: Unable to find address
      params.faultCode |= 0b00000001; // Set bit 0 of faultCode = water thermometer error
      snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor..[Fail!]"));
      Serial.println(F("##### Address not found for water temperature sensor - device 0 #####"));
    }
    delay(500); // wait 1/2 seconds for connection:
  }
  if (waterThermometer.isConnected(waterThermometerAddress)) {
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor.....[OK]"));
    Serial.print(F("##### Dallas temperature sensor - found "));
    Serial.print(waterThermometer.getDeviceCount(), DEC);
    Serial.println(F(" device/s. #####"));
    Serial.println(F("##### Dallas water temperature sensor intialized: OK. #####"));
    waterThermometer.setResolution(waterThermometerAddress, 9);
    params.faultCode &= 0b11111110; // Clear bit 0 of faultCode = no water thermometer error
  }
  else {
    params.faultCode |= 0b00000001; // Set bit 0 of faultCode = water thermometer error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor..[Fail!]"));
    Serial.println(F("##### Failed to initialize water temperature sensor! #####"));
  }
}



void initAmbientTemperatureSensors () {
  Serial.println(F("##### initAmbientTemperatureSensors started... #####"));
  strcpy (initActionResult, "");
  ambientThermometer1.begin(); // Initialize DHT ambient thermometer
  ambientThermometer2.begin(); // Initialize DHT ambient thermometer
  //  readTemperatureFunction();
  delay(500); // wait 1/2 second for connection:
  readTemperatureFunction();
  if ((params.faultCode & 0b00000010) == 0) {
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Atmp sensors....[OK]"));
    Serial.println(F("##### Ambient temperature sensors intialized: OK. #####"));
    params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
  }
  else {
    params.faultCode |= 0b00000010; // Set bit 0 of faultCode = ambient thermometer error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Atmp sensors.[Fail!]"));
    Serial.println(F("##### Failed to init an ambient temperature sensor! #####"));
  }
}



void readTemperatureFunction() { // Measure water temperature and ambient temperatures and humidities
  if (millis() > temperature_lasttime + temperature_mtbs)  {
    if (commands.debugLevel || (params.faultCode & 0b00010011) != 0) Serial.println(F("   ===== readTemperatureFunction started - water temp ====="));
    // Read water temperature data
    waterThermometer.requestTemperatures();
    int waterThermometerCheck = waterThermometer.getTempC(waterThermometerAddress);
    params.waterTemperature = waterThermometerCheck;
    if (waterThermometerCheck == DEVICE_DISCONNECTED_C) {
      Serial.println(F("   ===== Read water temperature error occurred - trying to reset water temperature sensor... ====="));
      initWaterTemperatureSensor ();
    }
    else {
      params.faultCode &= 0b11111110; // Clear bit 0 of faultCode = no water thermometer error
    }
    params.waterTemperature = waterThermometerCheck;
    if (commands.debugLevel || (params.faultCode & 0b00010011) != 0) Serial.println(F("   ===== readTemperatureFunction started - ambient temp ====="));
    // Read ambient temperature data - ambient thermometer 1
    float h = ambientThermometer1.readHumidity();
    float t = ambientThermometer1.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println(F("   ===== Read ambient temperature 1 error occurred - trying to reset ambient temperature sensor 1... ====="));
      ambientThermometer1.begin(); // Initialize DHT ambient thermometer 1
      h = ambientThermometer1.readHumidity();
      t = ambientThermometer1.readTemperature();
      if (isnan(h) || isnan(t)) {
        params.faultCode |= 0b00000010; // Set bit 1 of faultCode = ambient thermometer error
      }
      else {
        params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
        Serial.print(F("   ===== DHT11 ambient t-sensor 1 restarted successfully. ====="));
      }
    }
    else {
      params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
    }
    params.ambientTemperature1 = t;
    params.ambientHumidity1 = h;
    // Read ambient temperature data - ambient thermometer 2
    h = ambientThermometer2.readHumidity();
    t = ambientThermometer2.readTemperature();
    if (isnan(h) || isnan(t)) {
      Serial.println(F("   ===== Read ambient temperature 2 error occurred - trying to reset ambient temperature sensor 2... ====="));
      ambientThermometer2.begin(); // Initialize DHT ambient thermometer 2
      h = ambientThermometer2.readHumidity();
      t = ambientThermometer2.readTemperature();
      if (isnan(h) || isnan(t)) {
        params.faultCode |= 0b00000010; // Set bit 1 of faultCode = ambient thermometer error
      }
      else {
        params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
        Serial.print(F("   ===== DHT11 ambient t-sensor 2 restarted successfully. ====="));
      }
    }
    else {
      params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
    }
    params.ambientTemperature2 = t;
    params.ambientHumidity2 = h;
    temperature_lasttime = millis();
  }
}



//Filling and pump actions



void FillingPumpONAction() { // Start filling pump
  if (digitalRead (fillingPump) == LOW) {
    if ((millis() - lastFillStopTime) < (pauseBetweenFills * 60000)) return; // Check pump worked recently and wait
    digitalWrite (fillingPump, HIGH); // Start filling pump
    lastFillStartTime = millis();
    noFlowTimer = millis();
    fillStartPausedDuration = pausedDuration;
    Serial.print(F("    Filling pump started: currentManualFilling = "));
    Serial.print(params.currentManualFilling);
    Serial.print(F(" and currentAutoFilling = "));
    Serial.println(params.currentAutoFilling);
  }
  else {
    if (params.fillingPumpONDuration >= totalFillMaxDuration) { // Check if pump working for too long and wait if needed
      FillingPumpOFFAction();
    };
  }
  TimeElapsedFunction();
}



void FillingPumpOFFAction() { // Stop filling pump
  if (digitalRead (fillingPump) == HIGH) {
    digitalWrite (fillingPump, LOW); // Stop filling pump
    lastFillStopTime = millis ();    // Record time when filling pump stopped
    params.currentManualFilling = false;
    params.currentAutoFilling = false;
    params.lastFillDuration = (millis() - lastFillStartTime - (pausedDuration - fillStartPausedDuration)) / 60000;
    //    TimeElapsedFunction();
    params.currentWaterFlow = 0;
    Serial.print(F("    Filling pump stopped: lastFillDuration = "));
    Serial.print(params.lastFillDuration);
    Serial.print(F(" ' and fillingPumpONDuration = "));
    Serial.println(params.fillingPumpONDuration);
  }
}


// Checks, errors and alarms


void CheckForAlarms() { // Check for filling alarms
  // params.faultCode = 0;
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 1 - read temperatue... ===="));
  readTemperatureFunction();
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 2 - read water volume... ===="));
  readWaterVolume();
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 3 - read AC current... ===="));
  readEmonCurrentSensor ();
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 4 - read water flow... ===="));
  if (params.currentAutoFilling || params.currentManualFilling) readWaterFlow();
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 5 - analyze alarm conditions and set fault codes... ===="));
  if (commands.forceNoChecks && ((millis() - forceNoChecksStart) < 3600000)) { // Check if no checks for 1 h. and return
    params.faultCode = 0;
    params.currentAutoFilling = false;
    if (commands.fillingMode > 0) commands.fillingMode = 3;
    return;
  }
  commands.forceNoChecks = false;
  // Water and ambient temperature checks
  if ((params.waterTemperature <= freezeAlarmLimit) || (params.ambientTemperature1 <= 0)) {
    params.faultCode |= 0b00010000; // Set bit 4 of faultCode = Risk of freezing
  }
  else {
    params.faultCode &= 0b11101111; // Clear bit 4 of faultCode = no risk of freezing
  }
  // Water volume checks
  if (params.waterVolume > waterVolumeLimit) {
    params.faultCode |= 0b00100000; // Set bit 5 of faultCode = Overfill
  }
  else {
    params.faultCode &= 0b11011111; // Clear bit 5 of faultCode = no overfill
  }
  if (digitalRead (fillingPump) == HIGH) {
    if (params.lastFillDuration >= (singleFillMaxDuration )) {
      params.faultCode |= 0b01000000; // Set bit 6 of faultCode = Filling too long, i.e. continuing for more than singleFillMaxDuration minutes
    }
    else {
      params.faultCode &= 0b10111111; // Clear bit 6 of faultCode = filling not too long
    }
    // Water flow checks
    if (params.currentAutoFilling) { // Check for no flow if automatic filling is active
      // Check for flow sensor alarm (only if currently auto filling)
      if (params.currentWaterFlow < 1) {
        if ((millis() - noFlowTimer) >= (noFlowTreshold * 1000)) params.faultCode |= 0b00001000; // Check if no flow for more than 30 seconds and set bit 3 of faultCode = No water flow
      }
      else {
        noFlowTimer = millis();
        params.faultCode &= 0b11110111; // Clear bit 3 of faultCode = water flow is present
      }
    }
  }
  if (params.waterVolume > commands.maxWaterVolume + 3 || ((params.faultCode & 0b10110111) != 0)) { // Do not allow manual filling if tank near to overfill
    manualButtonCode = false;
    params.currentManualFilling = false;
  };
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 6 - check if water tank needs refilling and set flag if necessary... ===="));
  if (!params.pausedState && !params.currentManualFilling && ((commands.fillingMode == 1) || (commands.fillingMode == 2))) { // Check if auto filling is allowed
    if ((params.waterVolume < commands.minWaterVolume) && ((params.faultCode & 0b11110111) == 0)) {  // To be redacted to include also bit 4 = no water flow
      params.currentAutoFilling = true; // Water level low - initiate auto refill
      Serial.println(F("   ==== Water level low - auto refilling. ====")); //temp
    }
    else params.currentAutoFilling = false;
  }
  else params.currentAutoFilling = false;
  if (params.waterVolume > commands.maxWaterVolume) params.currentAutoFilling = false; // Turn off auto filling if tank is full enough
  if ((params.faultCode & 0b10010111) != 0) params.currentPressure = false; // If some errors, pressure pump forbidden
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 7 - call ErrorFunction if faultCode... ===="));
  if (params.faultCode != 0) ErrorFunction();
  if (commands.debugLevel > 2) Serial.println(F("  ==== CheckForAlarms point 8 - end. ===="));
}



void ErrorFunction() { // Error encountered - stop all processes and show fault code
  if (params.faultCode != lastFaultCode) {
    displayDimTimer = millis();
    lcd.backlight();
    if (lastFaultCode == 0) preErrorCommands = commands;
    lastFaultCode = params.faultCode;
    veryLongBeep();
    tone (buzzer, 600, 2000); // Buzz for 3 seconds with frequency 600 Hz
    strcpy_P (faultDescription, PSTR("ERROR:"));
    faultFillingOff = false;
    faultPressureOff = false;
    for (byte i = 0; i < 8; i++) { // Generate fault description string and audible alarm code
      byte faultCodeBit = bitRead(params.faultCode, i);
      if (faultCodeBit == 1) {
        veryLongBeep();
        strcat_P (faultDescription, PSTR(" <"));
        strcat_P (faultDescription, faultDescriptionArray[i]);
        strcat_P (faultDescription, PSTR("> "));
        char faultActionsText[4];
        strcpy_P (faultActionsText, faultActionArray[i]);
        if (faultActionsText[0] == 'P') commands.forcePause = true; // Check first character of fault description and start pause if required
        if (faultActionsText[1] == 'F') faultFillingOff = true;
        if (faultActionsText[2] == 'P') faultPressureOff = true;
        delay (150);
      }
      else {
        shortBeep();
        delay (150);
      }
    }
    if (faultFillingOff) strcat_P (faultDescription, PSTR("<Fpump OFF.> ")); // Check second character of fault description and change filling mode if required
    if (faultPressureOff) strcat_P (faultDescription, PSTR("<Ppump OFF.> ")); // Check third character of fault description and change pressure mode if required
    positionCounter = 0;
    // Create binary string from the error code
    char binaryString[9];
    strcpy (binaryfaultCodeString, "");
    itoa (params.faultCode, binaryString, 2);
    printf ("%s\n", binaryString);
    for (byte i = 0; i < (8 - strlen(binaryString)); i++) strcat (binaryfaultCodeString, "0");
    strcat (binaryfaultCodeString, binaryString);
    displayCurrentStatus();
    errorTelegramNotification ();
  }
  // Take actions according to the error
  if (faultFillingOff) commands.fillingMode = 0; // Check second character of fault description and change filling mode if required
  if (faultPressureOff) commands.pressureMode = 0; // Check third character of fault description and change pressure mode if required
  Serial.print(F("ERROR "));
  Serial.print(params.faultCode);
  Serial.print(F(": "));
  Serial.print(binaryfaultCodeString);
  Serial.print(F(": "));
  Serial.println(faultDescription);
  selectorButtonCode = false;
  manualButtonCode = false;
}



/* Fault codes description:
  ======= Fault Codes ====================
  1   params.faultCode |= 0b00000001; // Set bit 0 of faultCode = water thermometer error
  2   params.faultCode |= 0b00000010; // Set bit 1 of faultCode = ambient thermometer error
  4   params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
  8   params.faultCode |= 0b00001000; // Set bit 3 of faultCode = No water flow
  16  params.faultCode |= 0b00010000; // Set bit 4 of faultCode = Risk of freezing
  32  params.faultCode |= 0b00100000; // Set bit 5 of faultCode = Overfill
  64  params.faultCode |= 0b01000000; // Set bit 6 of faultCode = Filling continuing for more than singleFillMaxDuration minutes
  128 params.faultCode |= 0b10000000; // Set bit 7 of faultCode = General/unknown error
  ========================================
*/
/*
  const String faultDescriptionArray[8][4] {
  {"Water thermometer error.", "P", "0", "P"}, = 1
  {"Ambient thermometer error.", "P", "0", "P"}, = 2
  {"Laser distance sensor error. F. pump OFF.", "P", "F", "P"}, = 4
  {"No water flow while filling. F. pump OFF.", "P", "0", "0"}, = 8
  {"Too cold - risk of freezing! All pumps OFF.", "0", "F", "P"}, = 16
  {"Overfill! F. pump OFF.", "P", "F", "0"}, = 32
  {"Filling longer than allowed time. F. pump OFF.", "P", "0", "0"}, = 64
  {"Unknown error.", "P", "F", "P"} = 128
  };
*/


//void doorSwitchFunction() { // Door swich action
//
//}


// Program flow actions


void InitiateResetFunction () { // Declaration of reset function
  NVIC_SystemReset(); // Declaration of reset function for MKR1000 and MKR1010
}

//void (* InitiateResetFunction) (void) = 0; // Declaration of reset function for Uno, Leonardo, Nano, Yun



void StopAllFunction() {  // Stop all parts of the machine
  FillingPumpOFFAction();
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(currentManualFillingLED, LOW);
  digitalWrite(overfillEffector, LOW);
}



void PausedStateFunction() { // pausedState - stop all processes and raise pausedState flag
  if (!params.pausedState) {
    tone (buzzer, 600, 500); // Buzz for 500 milliseconds with frequency 700 Hz
    Serial.println(F("Paused state started... ")); //temp
    stopTime = millis();
    //    prepausedStatefillingPump = digitalRead(fillingPump);
    //    prepausedStatewaterDrawPump = digitalRead(waterDrawPump);
    //    prepausedStatecurrentManualFillingLED = digitalRead(currentManualFillingLED);
    //    prepausedStateoverfillEffector = digitalRead(overfillEffector);
    prePauseCommands = commands;
    StopAllFunction();
    params.currentAutoFilling = false;
    params.currentManualFilling = false;
    params.currentPressure = false;
    params.pausedState = true;
    selectorButtonMenu = false;
    selectorButtonCode = false;
    manualButtonCode = false;
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print(F("Paused... "));
    //    delay (2000);
  }
}


void ResumeFunction() { // Resume from pause or error
  lcd.clear();
  lcd.setCursor(0, 2);
  lcd.print(F("Resuming...     "));
  Serial.println(F("Resumed from paused state.")); //temp
  tone (buzzer, 800, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 800, 300); // Buzz for 300 milliseconds with frequency 700 Hz
  params.pausedState = false;
  selectorButtonMenu = true;
  selectorButtonCode = false;
  manualButtonCode = false;
  pausedDuration = millis() - stopTime;
  if (params.faultCode) commands = preErrorCommands; else commands = prePauseCommands;
  params.faultCode = 0; // Reset error code
  commands.forcePause = false;
  //  if (prepausedStatefillingPump == HIGH) FillingPumpONAction();
  //  digitalWrite(waterDrawPump, prepausedStatewaterDrawPump);
  //  digitalWrite(currentManualFillingLED, prepausedStatecurrentManualFillingLED);
  //  digitalWrite(overfillEffector, prepausedStateoverfillEffector);
  lcd.clear();
}


void ResetFunction() { // Stop all processes and restart system
  StopAllFunction();  // Stop all devices
  veryLongBeep();
  veryLongBeep();
  lcd.clear();
  lcd.home ();
  lcd.print(F("System reset..."));
  Serial.println(F("Restarting system...")); //temp
  InitiateResetFunction (); // Restart machine
}


void myshutdown() {
  //  initLaserDistanceSensor ();
  Serial.println(F("We gonna shut down ! ..."));
}


// Communication actions



void connectToWiFi(int retryMilliSeconds) {
  strcpy (initActionResult, "");
  if (WiFi.status() == WL_NO_SHIELD) {
    strcpy_P (initActionResult, PSTR("WiFi init....[Fail!]"));
    WiFiConnectionAlive = false;
    InternetConnectionAlive = false;
    ThingerConnectionAlive = false;
    Serial.println(F("##### WiFi shield not present. #####"));
  }
  unsigned long connectWiFiTimer = millis ();
  // attempt to connect to WiFi network:
  while (WiFiStatus != WL_CONNECTED && (millis() < (connectWiFiTimer + retryMilliSeconds))) {
    Serial.print("##### Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network:
    Watchdog.clear();
    //    Watchdog.disable();
    WiFiStatus = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Watchdog.clear();
    //    Watchdog.enable(watchdogRestartTime * 1000);
    delay(4000); // wait 4 seconds for connection:
  }
  if ( WiFiStatus == WL_CONNECTED) {
    strcpy_P (initActionResult, PSTR("WiFi connect....[OK]"));
    WiFiConnectionAlive = true;
    printCurrentNet();
    printWiFiData();
  }
  else {
    strcpy_P (initActionResult, PSTR("WiFi connect.[Fail!]"));
    WiFiConnectionAlive = false;
    InternetConnectionAlive = false;
    ThingerConnectionAlive = false;
  }
}


void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("##### IP Address: ");
  Serial.println(ip);
  if (commands.debugLevel) {
    // print your MAC address:
    byte mac[6];
    WiFi.macAddress(mac);
    if (commands.debugLevel)  Serial.print("##### MAC address: ");
    printMacAddress(mac);
  }
}


void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("##### Connected to the network SSID: ");
  Serial.println(WiFi.SSID());
  if (commands.debugLevel) {
    // print the MAC address of the router you're attached to:
    byte bssid[6];
    WiFi.BSSID(bssid);
    Serial.print("##### BSSID: ");
    printMacAddress(bssid);
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("##### signal strength (RSSI):");
    Serial.println(rssi);
    // print the encryption type:
    byte encryption = WiFi.encryptionType();
    Serial.print("##### Encryption Type:");
    Serial.println(encryption, HEX);
  }
}


void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}



void checkConnectionToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFiConnectionAlive = false;
    connectToWiFi(1000); // Attempt to reconnect to WiFi
    initThingerConfiguration();
  }
  else WiFiConnectionAlive = true;
  getEpochFromInternet();
}



void getEpochFromInternet() {
  //  rtc.begin();
  unsigned long epoch;
  int numberOfTries = 0, maxTries = 6;
  do {
    epoch = WiFi.getTime();
    numberOfTries++;
  }
  while ((epoch == 0) && (numberOfTries < maxTries));
  if (numberOfTries == maxTries) {
    Serial.println(F("NTP unreachable!!"));
    InternetConnectionAlive = false;
    ThingerConnectionAlive = false;
    //    checkConnectionToWiFi();
  }
  else {
    if (commands.debugLevel) Serial.print(F("Epoch received: "));
    if (commands.debugLevel) Serial.println(epoch);
    rtc.setEpoch(epoch + (GMT * 3600));
    InternetConnectionAlive = true;
  }
}



void initThingerConfiguration () {
  getEpochFromInternet();
  if (InternetConnectionAlive) {
    Watchdog.clear();
    //    Watchdog.disable();
    // Output values of params.
    Serial.println (F("##### Starting Thinger configuration... #####"));
    thing.add_wifi(WIFI_SSID, WIFI_PASSWORD);
    Watchdog.clear();
    thing["CA"] >> outputValue(params.currentAutoFilling);
    thing["CM"] >> outputValue(params.currentManualFilling);
    thing["CP"] >> outputValue(params.currentPressure);
    thing["UT"] >> outputValue(params.fillingPumpONDuration);
    thing["LF"] >> outputValue(params.lastFillDuration);
    thing["WV"] >> outputValue(params.waterVolume);
    thing["WF"] >> outputValue(params.currentWaterFlow);
    thing["WT"] >> outputValue(params.waterTemperature);
    thing["T1"] >> outputValue(params.ambientTemperature1);
    thing["H1"] >> outputValue(params.ambientHumidity1);
    thing["T2"] >> outputValue(params.ambientTemperature2);
    thing["H2"] >> outputValue(params.ambientHumidity2);
    thing["DS"] >> outputValue(params.doorSwitchState);
    Watchdog.clear();
    thing["PS"] >> outputValue(params.pausedState);
    thing["FC"] >> outputValue(params.faultCode);
    thing["PM"] >> outputValue(commands.pressureMode);
    thing["FM"] >> outputValue(commands.fillingMode);
    thing["LV"] >> outputValue(commands.minWaterVolume);
    thing["HV"] >> outputValue(commands.maxWaterVolume);
    thing["AR"] >> outputValue(commands.autoReset);
    thing["FR"] >> outputValue(commands.forceReset);
    thing["FP"] >> outputValue(commands.forcePause);
    thing["FN"] >> outputValue(commands.forceNoChecks);
    thing["CU"] >> outputValue(params.controllerUptime);
    thing["TH"] >> outputValue(params.currentTimeHours);
    thing["TM"] >> outputValue(params.currentTimeMinutes);
    thing["TS"] >> outputValue(params.currentTimeSeconds);
    thing["DY"] >> outputValue(params.currentDateYear);
    thing["DM"] >> outputValue(params.currentDateMonth);
    thing["DD"] >> outputValue(params.currentDateDay);
    // Input new data in commands.
    Watchdog.clear();
    thing["nPM"] << inputValue (newCommands.pressureMode, {newData = true;});
    thing["nFM"] << inputValue (newCommands.fillingMode, {newData = true;});
    thing["nLV"] << inputValue (newCommands.minWaterVolume, {newData = true;});
    thing["nHV"] << inputValue (newCommands.maxWaterVolume, {newData = true;});
    thing["nAR"] << inputValue (newCommands.autoReset, {newData = true;});
    thing["nFR"] << inputValue (newCommands.forceReset, {
      newData = true;
    });
    thing["nFP"] << inputValue (newCommands.forcePause, {newData = true;});
    //  Thinger data definitions - end
    Watchdog.clear();
    thing.handle();
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Thinger init..[Done]"));
    Serial.println (F("##### Thinger configuration completed. #####"));
    ThingerConnectionAlive = true;
    Watchdog.clear();
    //    Watchdog.enable(watchdogRestartTime * 1000);
  }
  else {
    ThingerConnectionAlive = false;
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Thinger init.[Fail!]"));
    Serial.println (F("##### Thinger configuration aborted due to no Internet connection! #####"));
  }
}



void communicateWithThinger() {
  if ((millis() - Thinger_lastMessage) >= Thinger_sleeptime) Thinger_mtbs = 500; else Thinger_mtbs = 100;
  if (commands.debugLevel > 1) Serial.print(F(" === communicateWithThinger point 1: milliseconds between Thinger checks: "));
  if (commands.debugLevel > 1) Serial.println(Thinger_mtbs);
  if (millis() > Thinger_lasttime + Thinger_mtbs)  {
    checkConnectionToWiFi();
    if (WiFiConnectionAlive && InternetConnectionAlive && ThingerConnectionAlive) {
      newCommands = commands;
      newData = false;
      //    Watchdog.disable();
      Watchdog.clear();
      thing.handle();
      Watchdog.clear();
      //    Watchdog.enable(watchdogRestartTime * 1000);
      if (commands.debugLevel > 2) Serial.println(F(" === communicateWithThinger point 2: Thinger handling done. ==="));
      dataTransferTimer = millis();
    }
    else {
      checkConnectionToWiFi();
      if (WiFiConnectionAlive) getEpochFromInternet();
      if (InternetConnectionAlive && !ThingerConnectionAlive) initThingerConfiguration ();
      if (commands.debugLevel > 2) Serial.println(F(" === communicateWithThinger point 3: Thinger restart done. ==="));
    }
    if (newData) {
      noInterrupts();
      Thinger_lastMessage = millis();
      commands = newCommands;
      newData = false;
      interrupts();
      if (localFeedback) {
        displayDimTimer = millis();
        shortBeep();
        shortBeep();
        shortBeep();
      }
      Serial.println(F(" === Parameters changed by Thinger IoT cloud. ===")); //temp
    }
    else {
      noInterrupts();
      newCommands = commands;
      interrupts();
    }
    Thinger_lasttime = millis();
  }
  if (commands.debugLevel > 2) Serial.println(F(" === communicateWithThinger point 4: end of Thinger communication..."));
}


void initTelegramBotConfiguration() { // Telegram bot init
  getEpochFromInternet();
  bool botResult = false;
  if (InternetConnectionAlive) {
    //    Watchdog.disable();
    Serial.println (F("##### Starting Telegram bot configuration... #####"));
    //    bot.begin(); // Initialize Telegram bot - not required for UniversalTelegramBot library
    generateStatusStrings();
    int numNewMessages = bot.getUpdates(1);
    Serial.println("##### Telegram initialization... #####");
    botResult = bot.sendMessage(SECRET_BOT_CHATID, "Hello! System now starting at:");
    botResult = bot.sendMessage(SECRET_BOT_CHATID, lastDateTimeStatus);
    if (botResult) {
      Bot_lasttime = millis();
      snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Telegram init.[Done]"));
      Serial.println (F("##### Telegram bot configuration completed. #####"));
      //    ThingerConnectionAlive = true;
      Watchdog.clear();
      //      Watchdog.enable(watchdogRestartTime * 1000);
    }
    else {
      snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Telegram init[Fail!]"));
      Serial.println (F("##### Telegram bot configuration finishing with unknown result. #####"));
    }
  }
  else {
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Telegram init[Fail!]"));
    Serial.println (F("##### Telegram bot configuration aborted due to no Internet connection! #####"));
  }
}


void errorTelegramNotification () { // Telegram bot init
  getEpochFromInternet();
  bool botResult = false;
  if (InternetConnectionAlive) {
    Watchdog.clear();
    Serial.println (F("Starting Telegram bot ERROR notification..."));
    generateStatusStrings();
    int numNewMessages = bot.getUpdates(1);
    bot.sendMessage(SECRET_BOT_CHATID, "An ERROR has occurred:");
    bot.sendMessage(SECRET_BOT_CHATID, faultDescription);
    //    bot.sendMessage(SECRET_BOT_CHATID, "== Minutes worked: " + String(params.controllerUptime) + " ==");
    //    Watchdog.clear();
    //    bot.sendMessage(SECRET_BOT_CHATID, lastSystemStatus);
    //    bot.sendMessage(SECRET_BOT_CHATID, lastEnvironmentStatus);
    //    bot.sendMessage(SECRET_BOT_CHATID, lastOperationStatus);
    //    Watchdog.clear();
    //    bot.sendMessage(SECRET_BOT_CHATID, lastControlStatus);
    //    bot.sendMessage(SECRET_BOT_CHATID, lastDateTimeStatus);
    //    bot.sendMessage(SECRET_BOT_CHATID, "== End of status ==");
    Watchdog.clear();
  }
  else {
    Serial.println (F("Telegram ERROR notification aborted due to no Internet connection!"));
  }
}



void communicateWithTelegram() {
  //  Watchdog.disable();
  Watchdog.clear();
  if ((millis() - Bot_lastMessage) >= Bot_sleeptime) Bot_mtbs = 10000; else Bot_mtbs = 3000;
  if (commands.debugLevel > 1) Serial.print(F(" === communicateWithTelegram point 1: milliseconds between Telegram bot checks: "));
  if (commands.debugLevel > 1) Serial.println(Bot_mtbs);
  if (millis() > Bot_lasttime + Bot_mtbs)  {
    if (commands.debugLevel > 2) Serial.println(F(" === communicateWithTelegram point 2: checking Telegram bot for new messages..."));
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      if (commands.debugLevel > 2) Serial.println(F(" === communicateWithTelegram point 3: analyze and respond to new messages..."));
      Serial.println(F("Received message from Telegram bot:"));
      for (int i = 0; i < numNewMessages; i++) {
        Bot_lastMessage = millis();
        Serial.print(bot.messages[i].chat_id);
        Serial.print(" sent message: ");
        Serial.println(bot.messages[i].text);
        if (bot.messages[i].text == "/feedback OFF" || bot.messages[i].text == "/feedback SILENT" || bot.messages[i].text == "/silent") {
          localFeedback = false;
          bot.sendMessage(bot.messages[i].chat_id, "Local feedback mode changed to OFF.");
        }
        if (localFeedback) {
          displayDimTimer = millis();
          longBeep();
          shortBeep();
          longBeep();
          shortBeep();
          if ((millis() - displayDimTimer) >= (displayDimPeriod * 1000) && (displayDimPeriod != 0)) lcd.noBacklight(); else lcd.backlight();
          String messageReceived = "MSG:" + bot.messages[i].text.substring(0, 16);
          while (messageReceived.length() < 20) messageReceived += " ";
          lcd.setCursor (0, 3);
          lcd.print (messageReceived);
        }
        if (bot.messages[i].text == "/status") {
          generateStatusStrings();
          bot.sendMessage(bot.messages[i].chat_id, "== Minutes worked: " + String(params.controllerUptime) + " ==");
          bot.sendMessage(bot.messages[i].chat_id, lastSystemStatus);
          bot.sendMessage(bot.messages[i].chat_id, lastEnvironmentStatus);
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, lastOperationStatus);
          if (params.faultCode) bot.sendMessage(bot.messages[i].chat_id, faultDescription);
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, lastControlStatus);
          bot.sendMessage(bot.messages[i].chat_id, lastDateTimeStatus);
          bot.sendMessage(bot.messages[i].chat_id, "== End of status ==");
        }
        else if (bot.messages[i].text == "/start") {
          Bot_mtbs = 1000;
          bot.sendMessage(bot.messages[i].chat_id, "Chat mode started - awaiting your command:");
        }
        else if (bot.messages[i].text == "/pressure OFF") {
          commands.pressureMode = 0;
          bot.sendMessage(bot.messages[i].chat_id, "Pressure mode changed to 0 (OFF).");
        }
        else if (bot.messages[i].text == "/pressure ON" || bot.messages[i].text == "/pressure AUTO") {
          commands.pressureMode = 1;
          bot.sendMessage(bot.messages[i].chat_id, "Pressure mode changed to 1 (AUTO).");
        }
        else if (bot.messages[i].text == "/filling OFF") {
          commands.fillingMode = 0;
          bot.sendMessage(bot.messages[i].chat_id, "Filling mode changed to 0 (OFF).");
        }
        else if (bot.messages[i].text == "/filling ALL" || bot.messages[i].text == "/filling A+M") {
          commands.fillingMode = 1;
          bot.sendMessage(bot.messages[i].chat_id, "Filling mode changed to 1 (AUTO+MANUAL).");
        }
        else if (bot.messages[i].text == "/filling AUTO" || bot.messages[i].text == "/filling A") {
          commands.fillingMode = 2;
          bot.sendMessage(bot.messages[i].chat_id, "Filling mode changed to 2 (AUTO ONLY).");
        }
        else if (bot.messages[i].text == "/filling MANUAL" || bot.messages[i].text == "/filling M") {
          commands.fillingMode = 3;
          bot.sendMessage(bot.messages[i].chat_id, "Filling mode changed to 3 (MANUAL ONLY).");
        }
        else if (bot.messages[i].text == "/feedback ON" || bot.messages[i].text == "/feedback VERBOSE" || bot.messages[i].text == "/silent OFF") {
          localFeedback = true;
          bot.sendMessage(bot.messages[i].chat_id, "Local feedback mode changed to ON.");
        }
        else if (bot.messages[i].text.substring(0, 10) == "/minvolume") {
          commands.minWaterVolume = (bot.messages[i].text.substring(11)).toInt();
          bot.sendMessage(bot.messages[i].chat_id, "commands.minWaterVolume changed to " + String(commands.minWaterVolume) + " L.");
        }
        else if (bot.messages[i].text.substring(0, 10) == "/maxvolume") {
          commands.maxWaterVolume = (bot.messages[i].text.substring(11)).toInt();
          bot.sendMessage(bot.messages[i].chat_id, "commands.maxWaterVolume changed to " + String(commands.maxWaterVolume) + " L.");
        }
        else if (bot.messages[i].text.substring(0, 11) == "/commands.debugLevel") {
          commands.debugLevel = (bot.messages[i].text.substring(12)).toInt();
          bot.sendMessage(bot.messages[i].chat_id, "commands.debugLevel changed to " + String(commands.debugLevel));
        }
        else if (bot.messages[i].text == "/pause") {
          commands.forcePause = true;
          bot.sendMessage(bot.messages[i].chat_id, "Force pause mode started.");
        }
        else if (bot.messages[i].text == "/resume") {
          commands.forcePause = false;
          bot.sendMessage(bot.messages[i].chat_id, "Resumed from pause mode.");
        }
        else if (bot.messages[i].text == "/commands.autoReset ON") {
          commands.autoReset = true;
          bot.sendMessage(bot.messages[i].chat_id, "commands.autoReset mode set to: " + String (commands.autoReset));
        }
        else if (bot.messages[i].text == "/commands.autoReset OFF") {
          commands.autoReset = false;
          bot.sendMessage(bot.messages[i].chat_id, "commands.autoReset mode set to: " + String (commands.autoReset));
        }
        else if (bot.messages[i].text == "/reset") {
          commands.forceReset = true;
          bot.sendMessage(bot.messages[i].chat_id, "Force reset initiated.");
        }
        else if (bot.messages[i].text == "/help") {
          generateStatusStrings();
          bot.sendMessage(bot.messages[i].chat_id, "== Supported commands: ==");
          bot.sendMessage(bot.messages[i].chat_id, "/start          Start an active chat with the bot.");
          bot.sendMessage(bot.messages[i].chat_id, "/status         Request actual status report.");
          bot.sendMessage(bot.messages[i].chat_id, "/pressure OFF   Switch pressure pump OFF.");
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, "/pressure ON    Set pressure pump to AUTO mode.");
          bot.sendMessage(bot.messages[i].chat_id, "/pressure AUTO  = /pressure ON");
          bot.sendMessage(bot.messages[i].chat_id, "/filling OFF    Switch filling pump OFF.");
          bot.sendMessage(bot.messages[i].chat_id, "/filling ALL    Set filling pump to AUTO+MANUAL mode.");
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, "/filling A+M    = /filling ALL");
          bot.sendMessage(bot.messages[i].chat_id, "/filling AUTO   Set filling pump to AUTO ONLY mode.");
          bot.sendMessage(bot.messages[i].chat_id, "/filling A      = /filling AUTO");
          bot.sendMessage(bot.messages[i].chat_id, "/filling MANUAL Set filling pump to MANUAL ONLY mode.");
          bot.sendMessage(bot.messages[i].chat_id, "/filling M      = /filling MANUAL");
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, "/minvolume XXX  Set lowest water level to XXX litres.");
          bot.sendMessage(bot.messages[i].chat_id, "/maxvolume XXX  Set highest water level to XXX litres.");
          bot.sendMessage(bot.messages[i].chat_id, "/pause          Force system into pause mode.");
          bot.sendMessage(bot.messages[i].chat_id, "/resume         Resume system from pause mode.");
          Watchdog.clear();
          bot.sendMessage(bot.messages[i].chat_id, "/commands.autoReset ON   Set commands.autoReset mode to ON.");
          bot.sendMessage(bot.messages[i].chat_id, "/commands.autoReset OFF  Set commands.autoReset mode to OFF.");
          bot.sendMessage(bot.messages[i].chat_id, "/reset          Force system to reset.");
          bot.sendMessage(bot.messages[i].chat_id, "== End of list ==");
        }
        else {
          bot.sendMessage(bot.messages[i].chat_id, bot.messages[i].text, ""); // Reply to the same chat with the same text
          bot.sendMessage(bot.messages[i].chat_id, "Waiting your command");
          Serial.println(F(" === Text bounced back to Telegram bot as no command recognized. ==="));
        }
      }
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    Bot_lasttime = millis();
  }
  Watchdog.clear();
  if (commands.debugLevel > 2) Serial.println(F(" === communicateWithTelegram point 4: end of Telegram communication..."));
}



// Main program actions

void checkForResetConditions() {
  if (commands.debugLevel > 1) Serial.println(F(" === checkForResetConditions started... ==="));
  params.controllerUptime = millis() / 60000;
  if ((commands.autoReset == true) && (params.controllerUptime > 1440)) commands.forceReset = true; // Restart system every 24 hours if commands.autoReset=true
  if (selectorButtonLastState == LOW && ((millis() - selectorButtonLDT)) >= resetButtonDebounceDelay) commands.forceReset = true; // Selector button held down long enough to count as Reset
  if (manualButtonLastState == LOW && ((millis() - manualButtonLDT)) >= resetButtonDebounceDelay) commands.forceReset = true; // Manual button held down long enough to count as Reset
  if (commands.forceReset) ResetFunction(); // Reset system
}


void takeActionsPerParams() {
  Watchdog.clear();
  if (commands.debugLevel > 1) Serial.println(F(" === takeActionsPerParams started ==="));
  //  if (commands.debugLevel > 2) Serial.println(F(" === takeActionsPerParams point 1 - pause/resume, buttons check, pump modes... ==="));
  checkForResetConditions();
  //  displayCurrentStatus();
  if (params.pausedState && (selectorButtonCode || manualButtonCode)) {
    commands.forcePause = false;
    selectorButtonMenu = true;
    selectorButtonCode = false;
    manualButtonCode = false;
  }
  if (commands.forcePause) PausedStateFunction();
  if (params.pausedState && !commands.forcePause) {
    ResumeFunction();
    selectorButtonMenu = true;
    selectorButtonCode = false;
    manualButtonCode = false;
  }
  if (selectorButtonCode && selectorButtonMenu) {
    UserMenuFunction();
    Watchdog.clear();
    selectorButtonCode = false;
  }
  if ((commands.fillingMode != 1) && (commands.fillingMode != 3)) manualButtonCode = false;
  params.currentManualFilling = (!commands.forcePause && manualButtonCode && ((commands.fillingMode == 1) || (commands.fillingMode == 3)));
  params.currentPressure = (!commands.forcePause && commands.pressureMode && (params.waterVolume >= presssureMinWaterVolume)); // If water level in tank is too low or if some errors, pressure pump forbidden
  displayCurrentStatus();
  Watchdog.clear();
}




void executeChecksAndCommands() {
  Watchdog.clear();
  if (commands.debugLevel > 1) Serial.println(F(" === executeChecksAndCommands started ==="));
  if (commands.debugLevel > 2) Serial.println(F(" === executeChecksAndCommands point 1 - check for alarms... ==="));
  CheckForAlarms();
  if (commands.debugLevel > 2) Serial.println(F(" === executeChecksAndCommands point 2 - execute pump actions... ==="));
  digitalWrite(currentManualFillingLED, params.currentManualFilling);
  if (params.currentManualFilling || params.currentAutoFilling) FillingPumpONAction(); // Start/maintain active filling pump
  if (!params.currentManualFilling && !params.currentAutoFilling) FillingPumpOFFAction(); // Stop/maintain non-active filling pump
  digitalWrite (waterDrawPump, params.currentPressure); // Actuate pressure pump state
  if (commands.debugLevel > 2) Serial.println(F(" === executeChecksAndCommands point 3 - end. ==="));
  Watchdog.clear();
}


void setup() {
  Wire.begin(); // Initialize I2C as master
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  delay (2000); // Wait 2 seconds
  Serial.println(F("System started."));
  // Init watchdog timer
  if (commands.debugLevel) Serial.println (F("Enabling watchdog timer."));
  Watchdog.attachShutdown(myshutdown);
  Watchdog.setup(WDT_SOFTCYCLE1M);  // initialize WDT-softcounter refesh cycle on 32sec interval
  Watchdog.clear();
  //  if (commands.debugLevel) Serial.print (F("Enabling watchdog timer. Milliseconds left until reset: "));
  //  Serial.println (Watchdog.enable(watchdogRestartTime * 1000)); // Set the watchdog timer to watchdogRestartTime seconds
  // Init pins and pin interrupts
  pinMode(fillingPump, OUTPUT);
  pinMode(waterDrawPump, OUTPUT);
  pinMode(currentManualFillingLED, OUTPUT);
  pinMode(overfillEffector, OUTPUT);
  pinMode(buzzer, OUTPUT);                  // Buzzer
  pinMode(FloatSwitchSensor, INPUT_PULLUP); // Float switch sensor
  pinMode(flowSensor, INPUT_PULLUP);        // Flow sensor or another input that needs interrupt
  pinMode(doorSwitch, INPUT_PULLUP);        // Door switch
  pinMode(fillingPumpACCurrent, INPUT);     // AC current - analogue input
  pinMode(encoderPinA, INPUT_PULLUP);       // Rotary encoder CLK
  pinMode(encoderPinB, INPUT_PULLUP);       // Rotary encoder DT
  pinMode(selectorButton, INPUT_PULLUP);    // Selector button
  pinMode(manualButton, INPUT_PULLUP);      // Manual fill button
  digitalWrite(fillingPump, LOW);
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(overfillEffector, LOW);
  digitalWrite(currentManualFillingLED, LOW);
  // attachInterrupt(digitalPinToInterrupt(doorSwitch), doorSwitchFunctionISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowSensor), flowSensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(selectorButton), selectorButtonISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(manualButton), manualButtonISR, CHANGE);
  shortBeep();
  veryLongBeep();
  // Init display
  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.createChar(1, backslash);
  lcd.clear();
  lcd.home (); // Set cursor to 0, 0
  strcpy_P(displayLineString[0], PSTR("Welcome!"));
  snprintf_P(displayLineString[1], sizeof(displayLineString[1]), PSTR("WaterSystem %s"), softwareVersion);
  // Init LD sensor
  snprintf_P(displayLineString[2], sizeof(displayLineString[2]), PSTR("LD sensor...        "));
  displayLines();
  Watchdog.clear();
  initLaserDistanceSensor ();
  Watchdog.clear();
  snprintf(displayLineString[2], sizeof(displayLineString[2]), initActionResult);
  // Init water temperature sensor
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Wtmp sensor...      "));
  displayLines();
  Watchdog.clear();
  initWaterTemperatureSensor ();
  Watchdog.clear();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  scrollDisplayUp();
  // Init ambient temperature and humidity sensors
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Atmp sensor...      "));
  displayLines();
  Watchdog.clear();
  initAmbientTemperatureSensors ();
  Watchdog.clear();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  scrollDisplayUp();
  // Init AC current sensor
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("AC current snsr...   "));
  displayLines();
  Watchdog.clear();
  initEmonCurrentSensor();
  Watchdog.clear();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  scrollDisplayUp();
  // Connect to WiFi
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("WiFi connect...     "));
  displayLines();
  connectToWiFi(20000); // Try to connect to WiFi for maximum of 20 seconds
  Watchdog.clear();
  // Init Real Time Clock
  rtc.begin();
  getEpochFromInternet();
  getDateAndTime();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  scrollDisplayUp();
  // Init Thinger IoT cloud
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Thinger init...     "));
  displayLines();
  initThingerConfiguration();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  // Init Telegram bot
  scrollDisplayUp();
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Telegram init...    "));
  displayLines();
  initTelegramBotConfiguration ();
  Watchdog.clear();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  // Init general timers
  currentMinuteTimer = millis(); // Initialize current minute timer
  currentDisplayCycleTimer1 = millis(); // Initialize display cycle timer 1
  currentDisplayCycleTimer2 = millis(); // Initialize display cycle timer 2
  displayDimTimer = millis(); // Initialize display dim timer
}


void loop() {
  if (commands.debugLevel) Serial.println(F("== Point 1 - General actions... =="));
  Watchdog.clear(); // If this function is not called within watchdogRestartTime seconds the board will reset itself
  doorSwitchFunction();
  TimeElapsedFunction();
  if ((millis() - displayDimTimer) >= (displayDimPeriod * 1000) && (displayDimPeriod != 0)) lcd.noBacklight(); else lcd.backlight();
  takeActionsPerParams();
  // Thinger communication
  if (commands.debugLevel) Serial.println(F("== Point 2 - Thinger communication... =="));
  communicateWithThinger();
  if (commands.debugLevel) Serial.println(F("== Point 3 - takeActionsPerParams... =="));
  takeActionsPerParams();
  // Telegram bot communication
  if (commands.debugLevel) Serial.println(F("== Point 4 - Telegram communication... =="));
  communicateWithTelegram();
  if (commands.debugLevel) Serial.println(F("== Point 5 - takeActionsPerParams... =="));
  takeActionsPerParams();
  // General actions
  if (commands.debugLevel) Serial.println(F("== Point 6 - executeChecksAndCommands... =="));
  executeChecksAndCommands(); // Execute actual effect after checking for alarms
}
