//Water Supply System v.2.0.
//Programer: Peter Nikolov.
//Suitable for MKR1010 but could be re-worked for Uno/Nano by changing pin assignments.
//A Water Supply System controller that controls and displays the level of water in the tank, and controls a pump, filling the tank, and a pressure pump that draws water from the tank.
//Using Dallas OneWire thermometer and rotary encoder for menu selector, 4 relays control, opto-isolator adviceable.
//Licensed under Creative Commons.
#undef min
#undef max
#include "arduino_secrets.h"

// ArduinoJson
//#include <ArduinoJson.h>
//#include <ArduinoJson.hpp>

#include <Wire.h>
//#include <rgb_lcd.h>
//#include <WiFi101.h>
#include <WiFiNINA.h>
//#include <TelegramBot.h>
//#include <ThingerWifi101.h>
#include <ThingerWiFiNINA.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>    // 1-Wire temperature sensor
#include <VL53L1X.h>              // I2C laser distance sensor
#include <DHT.h>                  // DHT-11 temperature and humidity sensor
#include <avr/pgmspace.h>
#include <Adafruit_SleepyDog.h>



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

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display
//rgb_lcd lcd;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature waterThermometer(&oneWire);
DeviceAddress waterThermometerAddress;
VL53L1X laserSensor;
DHT ambientThermometer1(AmbientThermometer1Pin, DHTTYPE);
DHT ambientThermometer2(AmbientThermometer2Pin, DHTTYPE);
//ThingerWifi101 thing(THINGER_USERNAME, THINGER_DEVICE_ID, THINGER_DEVICE_CREDENTIAL);
ThingerWiFiNINA thing(THINGER_USERNAME, THINGER_DEVICE_ID, THINGER_DEVICE_CREDENTIAL);



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
  uint8_t pressureMode = 1; // Pressure mode = AUTO (0 = OFF; 1 = AUTO)
  uint8_t fillingMode = 1; // Filling mode = AUTO+MANUAL (0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY; 3 = MANUAL ONLY)
  uint8_t autoReset = true; // Auto reset enabled every 24 hours
  uint8_t forceReset = false;  // Reset requested
  uint8_t forcePause = false;  // Pause requested
  uint8_t forceNoChecks = false;  // NoChecks requested for 1 hour
  uint16_t minWaterVolume = 60; // Lower water level limit to start refilling
  uint16_t maxWaterVolume = 270; // Higher water level limit to stop refilling
  uint16_t controllerUptime; // Controller uptime in minutes
  //  uint16_t cycleTime; // Controller last loop cycle time in milliseconds
  //  uint16_t communicationInterval = 50; // Thinger communication interval in milliseconds
};


//give a name to the group of data
RO_PARAMS_DATA_STRUCTURE params;
RO_PARAMS_DATA_STRUCTURE newParams;


// General constants declaration
const PROGMEM char softwareVersion[6] = "2.0";


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
const byte totalFillMaxDuration = 30;  // Maximum allowed duration of all filling cycles in minutes within the last totalFillCyclePeriod minutes
const byte presssureMinWaterVolume = 30; // Lower water level limit to start refilling
const int waterVolumeLimit = 280; // Water level limit to count as overfilling



// Timer variables declaration
unsigned long historyByMinutes[4] = {0, 0, 0, 0}; // Array to store filling pump status by minutes for the last 2 hours (128 minutes)
unsigned long lastFillStartTime;
unsigned long lastFillStopTime;
unsigned long fillStartPausedDuration;
unsigned long n;
unsigned long currentMinuteTimer;
const unsigned int watchdogRestartTime = 120; // Watchdog restart time in seconds; the system will default to maximum 16 seconds, if a larger value is given
const byte totalFillCyclePeriod = 120; // In minutes - cycle period to control maximum allowed filling duration (cannot be more than the bits in historyByMinutes[]) - 2 hours
byte currentHistoryPointer = 0; // Number between 0 and 191 - current pointer in the history by minutes
byte historyArrayPointer;
byte historyArrayBitPointer;


// Water level laser distance sensor variables declaration
const int tankCapacity = 300; // Tank capacity in litres
const int tankHeight = 120; // Tank height in centimeters
byte laserDistanceArrayIndex = 0;
byte laserDistanceArraySize = 30;
int laserDistanceArray[30]; // Array of laser distance measurements in cm, size should be = laserDistanceArraySize


// Flow sensor variables declaration
volatile int pulsecount;         // Variable to count number of pulses from flow sensor
const int noFlowTreshold = 30; // No flow treshold in seconds
unsigned long noFlowTimer = 0;


// Temperature monitoring variables declaration
const int freezeAlarmLimit = 2; // Lowest temperature limit to count for freezing alarm


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
byte forceAllReset = false;


// Error reporting variables declaration
const PROGMEM char faultDescriptionArray[8][22] {"Water t-sensor err.", "Amb. t-sensor err.", "LD sensor err.", "No Fpump water flow.", "Freezing risk!", "Overfill!", "Filling too long.", "Unknown error."};
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
    Serial.print(F(" == Minutes worked: "));
    Serial.println(params.controllerUptime);
    Serial.print(F("CA:"));
    Serial.print(params.currentAutoFilling);
    Serial.print(F(",CM:"));
    Serial.print(params.currentManualFilling);
    Serial.print(F(",CP:"));
    Serial.print(params.currentPressure);
    Serial.print(F(",UP:"));
    Serial.print(params.fillingPumpONDuration);
    Serial.print(F("',LF:"));
    Serial.print(params.lastFillDuration);
    Serial.print(F("',"));
    Serial.print(F(",WV:"));
    Serial.print(params.waterVolume);
    Serial.print(F("L, WF:"));
    Serial.print(params.currentWaterFlow);
    Serial.print(F("L/min, "));
    Serial.print(F(" WT:"));
    Serial.print(params.waterTemperature);
    Serial.print(F("oC, AT1:"));
    Serial.print(params.ambientTemperature1);
    Serial.print(F("oC, AH1:"));
    Serial.print(params.ambientHumidity1);
    Serial.print(F(" %, AT2:"));
    Serial.print(params.ambientTemperature2);
    Serial.print(F("oC, AH2:"));
    Serial.print(params.ambientHumidity2);
    Serial.println(F(" %, DS:"));
    Serial.print(params.doorSwitchState);
    Serial.print(F(",PS:"));
    Serial.print(params.pausedState);
    Serial.print(F(",FC:"));
    Serial.print(params.faultCode);
    Serial.print(F(",PM:"));
    Serial.print(params.pressureMode);
    Serial.print(F(",FM:"));
    Serial.print(params.fillingMode);
    Serial.print(F(",AR:"));
    Serial.print(params.autoReset);
    Serial.print(F(",FP:"));
    Serial.print(params.forcePause);
    Serial.print(F(",NC:"));
    Serial.print(params.forceNoChecks);
    Serial.println();
  }
}


//User interface actions


void UserMenuFunction() { // User menu
  int menuSelectorValue = 0;
  lcd.backlight();
  Watchdog.disable();
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
      params.forcePause = true;
      selectorButtonCode = 0;
      return;
      break;
    case 1: // Edit lowest water level limit
      shortBeep();
      params.minWaterVolume = setDigitalValueFunction(presssureMinWaterVolume + 5, params.maxWaterVolume - 5, params.minWaterVolume, 5, ">>Water low: ", "L");
      selectorButtonCode = 0;
      Serial.print(F("minWaterVolume set manually to ")); //temp
      Serial.print(params.minWaterVolume); //temp
      Serial.println("L."); //temp
      return;
      break;
    case 2: // Edit highest water level limit
      shortBeep();
      params.maxWaterVolume = setDigitalValueFunction(params.minWaterVolume + 5, waterVolumeLimit - 5, params.maxWaterVolume, 5, ">>Water high: ", "L");
      selectorButtonCode = 0;
      Serial.print(F("maxWaterVolume set manually to ")); //temp
      Serial.print(params.maxWaterVolume); //temp
      Serial.println("L."); //temp
      return;
      break;
    case 3: // Edit filling pump mode: 0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("Fill:OFF"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("Fill:AUTO+MANUAL"));
      snprintf_P(setParameterNames [2], sizeof(setParameterNames [2]), PSTR("Fill:AUTO ONLY"));
      snprintf_P(setParameterNames [3], sizeof(setParameterNames [3]), PSTR("Fill:MANUAL ONLY"));
      params.fillingMode = setParameterFunction(4, params.fillingMode, params.fillingMode, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("fillingMode set manually to ")); //temp
      Serial.println(params.fillingMode); //temp
      return;
      break;
    case 4: // Edit pressure pump mode: 0 = OFF; 1 = AUTO
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("Pressure:OFF"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("Pressure:AUTO"));
      params.pressureMode = setParameterFunction(2, params.pressureMode, params.pressureMode, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("pressureMode set manually to ")); //temp
      Serial.println(params.pressureMode); //temp
      return;
      break;
    case 5: // Edit auto reset mode: 0 = OFF; 1 = ON = restart every 24 h.
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("AutoReset:OFF"));
      //                                                                     01234567890123456789
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("AutoReset:every24h"));
      params.autoReset = setParameterFunction(2, params.autoReset, params.autoReset, ">>", "");
      selectorButtonCode = 0;
      Serial.print(F("autoReset mode set manually to ")); //temp
      Serial.println(params.autoReset); //temp
      return;
      break;
    case 6: // Edit force no checks mode: 0 = OFF; 1 = ON = no checks for 1 h.
      shortBeep();
      snprintf_P(setParameterNames [0], sizeof(setParameterNames [0]), PSTR("All checks ON"));
      snprintf_P(setParameterNames [1], sizeof(setParameterNames [1]), PSTR("No checks for 1 h"));
      menuSelectorValue = setParameterFunction(2, params.forceNoChecks, params.forceNoChecks, ">>", "");
      selectorButtonCode = 0;
      if (menuSelectorValue == 1) {
        longBeep();
        startForceNoChecks ();
        return;
      }
      else {
        params.forceNoChecks = false;
        Serial.print(F("forceNoChecks set manually to ")); //temp
        Serial.println(params.forceNoChecks); //temp
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
  params.forceNoChecks = true;
  forceNoChecksStart = millis();
  Serial.print(F("forceNoChecks set manually to ")); //temp
  Serial.println(params.forceNoChecks); //temp
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
    displayLineString[0][20] = 0;
    lcd.setCursor(0, 3);
    lcd.print (displayLineString[3]);
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
    displayLineString[0][20] = 0;
    lcd.setCursor(0, 3);
    lcd.print (displayLineString[3]);
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
    params.forceReset = true;
    NVIC_SystemReset(); // Called explicitly from here
  }
  if (selectorButtonReading == LOW && manualButtonReading == LOW) startForceNoChecks();
  noInterrupts();
  if (selectorButtonReading != selectorButtonLastState) {
    if (selectorButtonReading == LOW) selectorButtonCode = !selectorButtonCode; // If selector button held down after suffcient time to count as normal press, and then released, increment selectorButtonCode
    selectorButtonLastState = selectorButtonReading;
    selectorButtonLDT = millis();
    displayDimTimer = millis();
    if (selectorButtonCode) tone (buzzer, 600, 80); // Buzz for 80 milliseconds with frequency 600 Hz
  }
  interrupts();
}



void manualButtonISR() { // Read buttons and take actions as needed
  bool manualButtonReading = digitalRead (manualButton);
  bool selectorButtonReading = digitalRead (selectorButton);
  if (manualButtonLastState == LOW && ((millis() - manualButtonLDT)) >= resetButtonDebounceDelay) {
    params.forceReset = true; // Manual button held down long enough to count as Reset
    NVIC_SystemReset(); // Called explicitly from here
  }
  if (selectorButtonReading == LOW && manualButtonReading == LOW) startForceNoChecks();
  noInterrupts();
  if (manualButtonReading != manualButtonLastState) {
    if (manualButtonReading == LOW) manualButtonCode = !manualButtonCode; // If manual button held down after suffcient time to count as normal press, invert manualButtonCode
    manualButtonLastState = manualButtonReading;
    manualButtonLDT = millis();
    displayDimTimer = millis();
    if (manualButtonCode) tone (buzzer, 600, 80); // Buzz for 80 milliseconds with frequency 600 Hz
  }
  interrupts();
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
  if ((millis() - displayDimTimer) >= (displayDimPeriod * 1000) && (displayDimPeriod != 0)) lcd.noBacklight(); else lcd.backlight();
  if ((millis() - currentDisplayCycleTimer1) >= 1000) { // Check if a full second has elapsed
    currentDisplayCycleTimer1 = millis();
    strcpy(displayLineString[0], "");
    strcpy(displayLineString[1], "");
    strcpy(displayLineString[2], "");
    strcpy(displayLineString[3], "");
    displayFillingLine ();
    displayPressureLine ();
    if (params.currentAutoFilling || params.currentManualFilling) displayFlowDurationLine(); else displayIdleAnimation();
    if (params.pausedState) snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("PAUSED: key->resume "));
    switch (currentDisplayCycle) { // Cycle display
      case 0: // Display cycle 0
        displayAmbientLine();
        break;
      case 1: // Display cycle 1
        if (params.faultCode != 0) displayFaultCode();
        displayAmbientLine();
        break;
      case 2: // Display cycle 2
        displayWiFiStatus();
        break;
      case 3: // Display cycle 3
        if (params.faultCode != 0) displayFaultCode();
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
  if (params.forceNoChecks) strcat_P(displayLineString[0], PSTR("!")); else strcat_P(displayLineString[0], PSTR(":"));
  strcat_P (displayLineString[0], fillingModeText [params.fillingMode]);
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
  if (params.forceNoChecks) strcat_P(displayLineString[1], PSTR("!")); else strcat_P(displayLineString[1], PSTR(":"));
  if (params.pressureMode) strcat_P(displayLineString[1], PSTR(("AUTO"))); else strcat_P(displayLineString[1], PSTR("OFF"));
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
  if (params.forceNoChecks) strcat(displayLineString[2], "!"); else strcat(displayLineString[2], ":");
  unsigned int currentFillDuration = (millis() - lastFillStartTime - (pausedDuration - fillStartPausedDuration)) / 60000;
  snprintf_P(temporaryString, sizeof(temporaryString), PSTR("%2dL/min Dur:%2d'\n"), params.currentWaterFlow,  currentFillDuration);
  strcat(displayLineString[2], temporaryString);
  displayLineString[2][20] = 0;
}


void displayIdleAnimation () {  // char displayLineString[3][21];     // mFl:35L/min Dur:38'  or  FaultDescription or Submenu Item
  switch (currentDisplayCycle) { // Cycle display
    case 0: // Display cycle 0 - show current mode
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("| "));
      break;
    case 1: // Display cycle 0 - show current mode
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("/ "));
      break;
    case 2: // Display cycle 0 - show current mode
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("- "));
      break;
    case 3: // Display cycle 0 - show current mode
      snprintf_P (displayLineString[2], sizeof(displayLineString[2]), PSTR("  "));
      //      displayLineString[2][0] = 96;
      break;
  }
  byte hours, minutes;
  char timeElapsedString[19];
  hours = params.controllerUptime / 60;
  minutes = params.controllerUptime % 60;
  snprintf_P (timeElapsedString, sizeof(timeElapsedString), PSTR("Time worked: %02d:%02d "), hours, minutes);
  strcat (displayLineString[2], timeElapsedString);
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
  if (WiFiConnectionAlive == true) {
    snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("WiFi: %s"), WiFi.SSID());
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


void readWaterVolume() { // Read current water level in litres
  Serial.println(F("===Read water volume started==="));
  int currentWaterVolume = 0;
  int laserDistance = laserSensor.read();
  if (laserSensor.timeoutOccurred()) {
    params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
    Serial.println(F("Laser distance sensor error: Timeout occurred."));
    initLaserDistanceSensor ();
  }
  else {
    params.faultCode &= 0b11111011; // Clear bit 2 of faultCode = no laser distance sensor error
    laserDistanceArray[laserDistanceArrayIndex] = laserDistance / 10; // Store distance in centimeters
  }
  laserDistanceArrayIndex++;
  if (laserDistanceArrayIndex > laserDistanceArraySize) laserDistanceArrayIndex = 0;
  for (byte i = 0; i < laserDistanceArraySize - 1; i++) {
    currentWaterVolume += tankCapacity - round((tankCapacity * 1.0) * (laserDistanceArray[i] * 1.0 / tankHeight * 1.0));
  }
  params.waterVolume = (currentWaterVolume / laserDistanceArraySize);
}



void initLaserDistanceSensor () {
  Serial.println(F("===initLaserDistanceSensor started==="));
  Wire.begin(); // Initialize I2C as master
  strcpy (initActionResult, "");
  if (!laserSensor.init()) {
    laserSensor.setTimeout(500); // Start initializing laser sensor
    Serial.println(F("Attempting to initialize laser distance sensor."));
    laserSensor.init();
    delay(1000); // wait 1 second for connection:
  }
  if (laserSensor.init()) {
    laserSensor.setDistanceMode (VL53L1X::Long);
    laserSensor.setMeasurementTimingBudget(50000);
    laserSensor.startContinuous(50);
    for (byte i = 0; i < (laserDistanceArraySize - 1); i++) { // Initialize laser distance array
      laserDistanceArray[i] = laserSensor.read() / 10;
    }
    laserDistanceArray [laserDistanceArrayIndex] = laserSensor.readRangeContinuousMillimeters();  //data received in mm  //temporarily removed - to be reused, to be replaced with reading from laser distance sensor and calculation of water volume
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor.......[OK]"));
    Serial.println(F("LD sensor intialized: OK."));
    params.faultCode &= 0b11111011; // Clear bit 3 of faultCode = no laser distance sensor error
  }
  else {
    params.faultCode |= 0b00000100; // Set bit 2 of faultCode = laser distance sensor error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("LD sensor....[Fail!]"));
    Serial.println(F("Failed to init LD sensor!"));
  }
}



void readWaterFlow() {  // Read water flow sensor and return flow in L/min
  Serial.println(F("===readWaterFlow started==="));
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
  Serial.println(F("===initWaterTemperatureSensor started==="));
  strcpy (initActionResult, "");
  if (!waterThermometer.isConnected(waterThermometerAddress)) {
    waterThermometer.begin(); // Start initializing water thermometer
    waterThermometer.getAddress(waterThermometerAddress, 0);
    if (!waterThermometer.getAddress(waterThermometerAddress, 0)) { // Water thermometer: Unable to find address
      params.faultCode |= 0b00000001; // Set bit 0 of faultCode = water thermometer error
      snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor..[Fail!]"));
      Serial.println(F("Address not found for water temperature sensor - device 0"));
    }
    delay(700); // wait 1/2 seconds for connection:
  }
  if (waterThermometer.isConnected(waterThermometerAddress)) {
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor.....[OK]"));
    Serial.print(F("Dallas temperature sensor - found "));
    Serial.print(waterThermometer.getDeviceCount(), DEC);
    Serial.println(F(" device/s."));
    Serial.println(F("Dallas water temperature sensor intialized: OK."));
    waterThermometer.setResolution(waterThermometerAddress, 9);
    params.faultCode &= 0b11111110; // Clear bit 0 of faultCode = no water thermometer error
  }
  else {
    params.faultCode |= 0b00000001; // Set bit 0 of faultCode = water thermometer error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Wtmp sensor..[Fail!]"));
    Serial.println(F("Failed to initialize water temperature sensor!"));
  }
}



void initAmbientTemperatureSensors () {
  Serial.println(F("===initAmbientTemperatureSensors started==="));
  strcpy (initActionResult, "");
  ambientThermometer1.begin(); // Initialize DHT ambient thermometer
  ambientThermometer2.begin(); // Initialize DHT ambient thermometer
  readTemperatureFunction();
  delay(700); // wait 1/2 second for connection:
  readTemperatureFunction();
  if ((params.faultCode & 0b00000010) == 0) {
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Atmp sensors....[OK]"));
    Serial.println(F("Ambient temperature sensors intialized: OK."));
    params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
  }
  else {
    params.faultCode |= 0b00000010; // Set bit 0 of faultCode = ambient thermometer error
    snprintf_P(initActionResult, sizeof(initActionResult), PSTR("Atmp sensors.[Fail!]"));
    Serial.println(F("Failed to init an ambient temperature sensor!"));
  }
}




void readTemperatureFunction() { // Measure water temperature
  Serial.println(F("===readTemperatureFunction started - water temp==="));
  // Read water temperature data
  waterThermometer.requestTemperatures();
  int waterThermometerCheck = waterThermometer.getTempC(waterThermometerAddress);
  params.waterTemperature = waterThermometerCheck;
  if (waterThermometerCheck == DEVICE_DISCONNECTED_C) {
    Serial.println(F("Water thermometer error - retrying"));
    initWaterTemperatureSensor ();
  }
  else {
    params.faultCode &= 0b11111110; // Clear bit 0 of faultCode = no water thermometer error
  }
  params.waterTemperature = waterThermometerCheck;
  Serial.println(F("===readTemperatureFunction started - ambient temp==="));
  // Read ambient temperature data - ambient thermometer 1
  float h = ambientThermometer1.readHumidity();
  float t = ambientThermometer1.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Ambient t-sensor 1 error - retrying."));
    ambientThermometer1.begin(); // Initialize DHT ambient thermometer 1
    h = ambientThermometer1.readHumidity();
    t = ambientThermometer1.readTemperature();
    if (isnan(h) || isnan(t)) {
      params.faultCode |= 0b00000010; // Set bit 1 of faultCode = ambient thermometer error
    }
    else {
      params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
      Serial.print(F("DHT11 ambient t-sensor 1 restarted successfully."));
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
    Serial.println(F("Ambient t-sensor 2 error - retrying."));
    ambientThermometer2.begin(); // Initialize DHT ambient thermometer 2
    h = ambientThermometer2.readHumidity();
    t = ambientThermometer2.readTemperature();
    if (isnan(h) || isnan(t)) {
      params.faultCode |= 0b00000010; // Set bit 1 of faultCode = ambient thermometer error
    }
    else {
      params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
      Serial.print(F("DHT11 ambient t-sensor 2 restarted successfully."));
    }
  }
  else {
    params.faultCode &= 0b11111101; // Clear bit 1 of faultCode = no ambient thermometer error
  }
  params.ambientTemperature2 = t;
  params.ambientHumidity2 = h;
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
    params.lastFillDuration = (millis() - lastFillStartTime - (pausedDuration - fillStartPausedDuration)) / 60000;
    TimeElapsedFunction();
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
  Serial.println(F("===Point 3.1.1==="));
  readTemperatureFunction();
  Serial.println(F("===Point 3.1.2==="));
  readWaterVolume();
  Serial.println(F("===Point 3.1.3==="));
  if (params.currentAutoFilling || params.currentManualFilling) readWaterFlow();
  Serial.println(F("===Point 3.1.4==="));
  if (params.forceNoChecks && ((millis() - forceNoChecksStart) < 3600000)) { // Check if no checks for 1 h. and return
    params.faultCode = 0;
    params.currentAutoFilling = false;
    if (params.fillingMode > 0) params.fillingMode = 3;
    return;
  }
  params.forceNoChecks = false;
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
  if (params.waterVolume > params.maxWaterVolume + 3 || ((params.faultCode & 0b10110111) != 0)) { // Do not allow manual filling if tank near to overfill
    manualButtonCode = false;
    params.currentManualFilling = false;
  };
  if (!params.pausedState && !params.currentManualFilling && ((params.fillingMode == 1) || (params.fillingMode == 2))) { // Check if auto filling is allowed
    if ((params.waterVolume < params.minWaterVolume) && ((params.faultCode & 0b11110111) == 0)) {  // To be redacted to include also bit 4 = no water flow
      params.currentAutoFilling = true; // Water level low - initiate auto refill
      Serial.println(F("   Water level low - auto refilling.")); //temp
    }
    else params.currentAutoFilling = false;
  }
  else params.currentAutoFilling = false;
  if (params.waterVolume > params.maxWaterVolume) params.currentAutoFilling = false; // Turn off auto filling if tank is full enough
  if ((params.faultCode & 0b10010111) != 0) params.currentPressure = false; // If some errors, pressure pump forbidden
  if (params.faultCode != 0) ErrorFunction();
  Serial.println(F("===Point 3.1.5==="));
}



void ErrorFunction() { // Error encountered - stop all processes and show fault code
  if (params.faultCode != lastFaultCode) {
    displayDimTimer = millis();
    lcd.backlight();
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
        if (faultActionsText[0] == 'P') params.forcePause = true; // Check first character of fault description and start pause if required
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
  }
  // Take actions according to the error
  if (faultFillingOff) params.fillingMode = 0; // Check second character of fault description and change filling mode if required
  if (faultPressureOff) params.pressureMode = 0; // Check third character of fault description and change pressure mode if required
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
  128 params.faultCode |= 0b10000000; // Set bit 7 of faultCode = Unknown error
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
  if (params.pausedState == false) {
    tone (buzzer, 600, 500); // Buzz for 500 milliseconds with frequency 700 Hz
    params.pausedState = true;
    selectorButtonMenu = false;
    Serial.println(F("Paused state started... ")); //temp
    stopTime = millis();
    prepausedStatefillingPump = digitalRead(fillingPump);
    prepausedStatewaterDrawPump = digitalRead(waterDrawPump);
    prepausedStatecurrentManualFillingLED = digitalRead(currentManualFillingLED);
    prepausedStateoverfillEffector = digitalRead(overfillEffector);
    StopAllFunction();
    params.currentAutoFilling = false;
    params.currentManualFilling = false;
    params.currentPressure = false;
    lcd.clear();
    lcd.setCursor(0, 2);
    lcd.print(F("Paused... "));
    //    delay (2000);
  }
}


void ResumeFunction() { // Resume wash processes after pausedState
  lcd.clear();
  lcd.setCursor(0, 2);
  lcd.print(F("Resuming...     "));
  Serial.println(F("Resumed from paused state.")); //temp
  tone (buzzer, 800, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 800, 300); // Buzz for 300 milliseconds with frequency 700 Hz
  params.pausedState = false;
  selectorButtonMenu = true;
  params.faultCode = 0; // Reset error code
  pausedDuration = millis() - stopTime;
  if (prepausedStatefillingPump == HIGH) FillingPumpONAction();
  digitalWrite(waterDrawPump, prepausedStatewaterDrawPump);
  digitalWrite(currentManualFillingLED, prepausedStatecurrentManualFillingLED);
  digitalWrite(overfillEffector, prepausedStateoverfillEffector);
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



// Communication actions



void connectToWiFi(int retryMilliSeconds) {
  unsigned long connectWiFiTimer = millis ();
  strcpy (initActionResult, "");
  if (WiFi.status() == WL_NO_SHIELD) {
    //                                 01234567890123456789
    strcpy_P (initActionResult, PSTR("WiFi init....[Fail!]"));
    WiFiConnectionAlive = false;
    Serial.println("WiFi shield not present.");
  }
  // attempt to connect to WiFi network:
  while (WiFiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network:
    WiFiStatus = WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000); // wait 5 seconds for connection:
    if ((millis() - connectWiFiTimer) > (retryMilliSeconds)) {
      //                                 01234567890123456789
      strcpy_P (initActionResult, PSTR("WiFi connect.[Fail!]"));
      WiFiConnectionAlive = false;
      break;
    }
  }
  if ( WiFiStatus == WL_CONNECTED) {
    WiFiConnectionAlive = true;
    //                                 01234567890123456789
    strcpy_P (initActionResult, PSTR("WiFi connect....[OK]"));
    printCurrentNet();
    printWiFiData();
  }
}


void printWiFiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}


void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("Connected to the network SSID: ");
  Serial.println(WiFi.SSID());
  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);
  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
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
    connectToWiFi(500); // Attempt to reconnect to WiFi
  }
  else WiFiConnectionAlive = true;
}


//void communicateParameters() { // Communicate parameters to another Arduino over I2C
//  params.controllerUptime = millis() / 60000;
//  Wire.beginTransmission (I2C_SLAVE_ADDRESS);
//  i2cSimpleWrite (params);
//  Wire.endTransmission ();
//  Serial.println(F("===I2C data sent==="));
//  if (Wire.requestFrom (I2C_SLAVE_ADDRESS, (sizeof (newData) + sizeof (newParams)))) {
//    i2cSimpleRead (newData);
//    i2cSimpleRead (newParams);
//    noInterrupts();
//    dataTransferTimer = millis();
//    interrupts();
//    Serial.println(F("===I2C data received==="));
//  }
//  if (newData) {
//    noInterrupts();
//    params = newParams;
//    interrupts();
//    shortBeep();
//    shortBeep();
//    shortBeep();
//    Serial.println(F("Data rcvd from slave contr. on I2C.")); // temp
//  }
//  if ((millis() - dataTransferTimer) >= 600000) { // Check if no communication for 10 minutes and restart the other Arduino if so
//    resetOtherArduino();
//    dataTransferTimer = millis();
//  }
//}


// Main program actions


void takeActionsPerParams() {
  if (params.forceReset) ResetFunction();
  if (params.pausedState) {
    if (selectorButtonCode || manualButtonCode) {
      params.forcePause = false;
      selectorButtonMenu = true;
    }
  }
  if (params.forcePause) PausedStateFunction();
  if (params.pausedState && !params.forcePause) {
    ResumeFunction();
    selectorButtonCode = false;
    manualButtonCode = false;
    selectorButtonMenu = true;
  }
  if (selectorButtonCode && selectorButtonMenu) {
    UserMenuFunction();
    Watchdog.enable(watchdogRestartTime * 1000); // Set the watchdog timer to watchdogRestartTime seconds
    Watchdog.reset();
    selectorButtonCode = false;
  }
  if ((params.fillingMode != 1) && (params.fillingMode != 3)) manualButtonCode = false;
  params.currentManualFilling = (!params.pausedState && manualButtonCode && ((params.fillingMode == 1) || (params.fillingMode == 3)));
  params.currentPressure = (!params.pausedState && params.pressureMode && (params.waterVolume >= presssureMinWaterVolume)); // If water level in tank is too low or if some errors, pressure pump forbidden
  Serial.println(F("===Point 3.1==="));
  CheckForAlarms();
  Serial.println(F("===Point 3.2==="));
  digitalWrite(currentManualFillingLED, params.currentManualFilling);
  if (params.currentManualFilling || params.currentAutoFilling) FillingPumpONAction(); // Start/maintain active filling pump
  if (!params.currentManualFilling && !params.currentAutoFilling) FillingPumpOFFAction(); // Stop/maintain non-active filling pump
  digitalWrite (waterDrawPump, params.currentPressure); // Actuate pressure pump state
  TimeElapsedFunction();
  Serial.println(F("===Point 3.3==="));
}



void setup() {
  Wire.begin(); // Initialize I2C as master
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  delay (2000); // Wait 2 seconds
  Serial.println(F("System started."));
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
  lcd.init();
  lcd.backlight();
  lcd.begin(20, 4);
  lcd.createChar(1, backslash);
  lcd.clear();
  lcd.home (); // Set cursor to 0, 0
  strcpy_P(displayLineString[0], PSTR("Welcome!"));
  snprintf_P(displayLineString[1], sizeof(displayLineString[1]), PSTR("WaterSystem %s"), softwareVersion);
  snprintf_P(displayLineString[2], sizeof(displayLineString[2]), PSTR("LD sensor...        "));
  displayLines();
  initLaserDistanceSensor ();
  snprintf(displayLineString[2], sizeof(displayLineString[2]), initActionResult);
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Wtmp sensor...      "));
  displayLines();
  initWaterTemperatureSensor ();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  scrollDisplayUp();
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Atmp sensor...      "));
  displayLines();
  initAmbientTemperatureSensors ();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  // Connect to WiFi
  scrollDisplayUp();
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("WiFi connect...     "));
  displayLines();
  connectToWiFi(20000); // Try to connect to WiFi for maximum of 20 seconds
  snprintf(displayLineString[3], sizeof(displayLineString[3]), initActionResult);
  displayLines();
  // Thinger data definitions - begin
  scrollDisplayUp();
  snprintf_P(displayLineString[3], sizeof(displayLineString[3]), PSTR("Thinger init...     "));
  displayLines();
  // Output values of params.
  thing.add_wifi(WIFI_SSID, WIFI_PASSWORD);
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
  thing["PS"] >> outputValue(params.pausedState);
  thing["FC"] >> outputValue(params.faultCode);
  thing["PM"] >> outputValue(params.pressureMode);
  thing["FM"] >> outputValue(params.fillingMode);
  thing["LV"] >> outputValue(params.minWaterVolume);
  thing["HV"] >> outputValue(params.maxWaterVolume);
  thing["AR"] >> outputValue(params.autoReset);
  thing["FR"] >> outputValue(params.forceReset);
  thing["FP"] >> outputValue(params.forcePause);
  thing["FN"] >> outputValue(params.forceNoChecks);
  thing["CU"] >> outputValue(params.controllerUptime);
  // Input new data in commands.
  thing["nPM"] << inputValue (newParams.pressureMode, {newData = true;});
  thing["nFM"] << inputValue (newParams.fillingMode, {newData = true;});
  thing["nLV"] << inputValue (newParams.minWaterVolume, {newData = true;});
  thing["nHV"] << inputValue (newParams.maxWaterVolume, {newData = true;});
  thing["nAR"] << inputValue (newParams.autoReset, {newData = true;});
  thing["nFR"] << inputValue (newParams.forceReset, {
    newData = true;
  });
  thing["nFP"] << inputValue (newParams.forcePause, {newData = true;});
  //  Thinger data definitions - end
  thing.handle();
  snprintf(displayLineString[3], sizeof(displayLineString[3]), "Thinger init..[Done]");
  displayLines();
  currentMinuteTimer = millis(); // Initialize current minute timer
  currentDisplayCycleTimer1 = millis(); // Initialize display cycle timer
  currentDisplayCycleTimer2 = millis();
  displayDimTimer = millis(); // Initialize display dim timer
  Serial.print (F("Enabling watchdog timer. Milliseconds left until reset: "));
  Serial.println (Watchdog.enable(watchdogRestartTime * 1000)); // Set the watchdog timer to watchdogRestartTime seconds
}


void loop() {
  //  Watchdog.enable(watchdogRestartTime * 1000); // Set the watchdog timer to watchdogRestartTime seconds
  Watchdog.reset(); // If this function is not called within watchdogRestartTime seconds the board will reset itself
  Serial.println(F("===Point 1 - Watchdog just reset ==="));
  params.controllerUptime = millis() / 60000;
  if ((params.autoReset == true) && (params.controllerUptime > 1440)) ResetFunction(); // Restart system every 24 hours if autoReset=true
  if (WiFiConnectionAlive) {
    newParams = params;
    newData = false;
    thing.handle();
    dataTransferTimer = millis();
  }
  if (newData) {
    noInterrupts();
    params = newParams;
    newData = false;
    interrupts();
    displayDimTimer = millis();
    shortBeep();
    shortBeep();
    shortBeep();
    Serial.println(F("== Parameters changed by IoT cloud. ==")); //temp
  }
  else {
    noInterrupts();
    newParams = params;
    interrupts();
  }
  Serial.println(F("===Point 2==="));
  //  ButtonsFunction();
  if (selectorButtonLastState == LOW && ((millis() - selectorButtonLDT)) >= resetButtonDebounceDelay) params.forceReset = true; // Selector button held down long enough to count as Reset
  if (manualButtonLastState == LOW && ((millis() - manualButtonLDT)) >= resetButtonDebounceDelay) params.forceReset = true; // Manual button held down long enough to count as Reset
  if (params.forceReset) ResetFunction(); // Called explicitly from here
  doorSwitchFunction();
  Serial.println(F("===Point 3==="));
  takeActionsPerParams();
  Serial.println(F("===Point 4==="));
  displayCurrentStatus();
  Serial.println(F("===Point 5==="));
  if ((millis() - displayDimTimer) >= (displayDimPeriod * 1000) && (displayDimPeriod != 0)) lcd.noBacklight(); else lcd.backlight();
}
