
//Peter Nikolov's Water Supply System v.1.0.
//Programer: Peter Nikolov.
//Based on Rivera_1.0 by Pedro Rivera and the work of steve8428 - Thank you.
//A Water Supply System controller that controls and displays the level of water in the tank, and controls a pump, filling the tank, and a pressure pump that draws water from the tank.
//Using Dallas OneWire thermometer and rotary encoder for menu selector, 2 relays control, opto-isolator adviceable.
//Licensed under Creative Commons.

// #include <LiquidCrystal.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>    // 1-Wire temperature sensor
#include <Protocentral_VL53L0x.h> // Laser distance sensor

//Pin assignment
// #define floodSensor 2 //flood sensor error
#define fillingPump 4 //filling pump 
#define waterDrawPump 5 //water draw (pressure) pump permission relay
#define overfillEffector 6 //effector for lowering water level in case of overfill of the tank
#define waterFlowSensor 7 //filling pump flow sensor (pin 7 on Leonardo, to be put to pin 3 on Nano)
#define fillingPumpLED 8 //filling pump manual control LED indicator
#define fillingPumpACCurrent 9 //filling pump AC current sensor
#define FloatSwitchSensor 10 //water float switch sensor = backup water level sensor
#define ONE_WIRE_BUS 12 // OneWire interface (for thermometers)
#define doorOpenSwitch 12 //door open switch (A5 for Leonardo, 3 for Nano)
#define rotaryEncoderPinA A0 // Rotary Encoder DT
#define rotaryEncoderPinB A1 // Rotary Encoder CLK
#define selectorButton A2 //start/pausedState/resume/reset button
#define buzzer A3 //buzzer/beeper 
#define ManualFillBtn A4 //filling pump manual control button  (A4 for Leonardo, 12 for Nano; on Uno and Nano A4 and A5 are for I2C)

rgb_lcd lcd;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
VL53L0X sensor;

//constants declaration
const String softwareVersion = "1.0";
const int buttonDebounceDelay = 10; // delay to count as button pressed - 5 milliseconds
const int resetButtonDebounceDelay = 5000; // delay to count as reset - 5 seconds
const int overheatLimit = 72; // Temperature limit to count for overheating
const int freezeAlarmLimit = 3; // Lowest temperature limit to count for freezing alarm
const int tankCapacity = 300; // Tank capacity in litres
const int tankHeight = 1200; // Tank height in millimeters


//variables declaration
volatile int doorOpenSwitchState;
volatile byte pausedState = false;
volatile byte currentAutoFilling = false;
volatile byte currentManualFilling = false;
volatile unsigned long stopTime = 0;       // Record time program was paused
volatile unsigned long pausedDuration = 0; // Record how long the program was paused for
volatile byte prepausedStatefillingPump;
volatile byte prepausedStatewaterDrawPump;
volatile byte prepausedStatefillingPumpLED;
volatile byte prepausedStateoverfillEffector;
volatile int faultCode = 0;
volatile int pulsecount; //Variable to count number of pulses from flow sensor
unsigned long totalPeriodStart, totalPeriodDuration;
// unsigned long currentPeriodStart, currentPeriodDuration;
unsigned long currentFillStart = 0, currentFillDuration = 0, totalFillDuration = 0;
unsigned long LastFillTime = 0, lastFillStopTime = 0;
unsigned long FillStartpausedDuration = 0;
unsigned long pauseBetweenFills = 20000; // 20 seconds - to be changed to 120000 = 2 minutes
signed long laserDistance = 0; // Distance measured by laser sensor in mm
unsigned int singleFillMaxDuration = 30; // Maximum allowed duration of a single filling cycle in minutes
unsigned int totalFillMaxDuration = 60;  // Maximum allowed duration of all filling cycles in minutes within the last 3 hours
double tankProportion;
int currentWaterVolume;
int lastWaterVolume = 0;
int presssureMinWaterVolume = 30; // Lower water level limit to start refilling
int minWaterVolume = 40; // Lower water level limit to start refilling
int maxWaterVolume = 80; // Higher water level limit to stop refilling
int waterVolumeLimit = 280; // Water level limit to count as overfilling
int menuSelectorValue = 0;
int selectorDialValue = 0;
int waterFlowSensorState = 0;
//-----On/pausedState/restart/reset Button reference ------
int rotaryEncoderPosition = 0;
int rotaryEncoderPinALast = LOW;
int selectorButtonState = HIGH;
int selectorButtonRecentState = HIGH;
int selectorButtonLastState = HIGH;
unsigned long lastSelectorBtnDbncTime = 0;
int selectorButtonCode = 0;  //switch case statement to control what each button push does
int ManualFillBtnState = HIGH;
int ManualFillBtnRecentState = HIGH;
int ManualFillBtnLastState = HIGH;
unsigned long lastManualDbncTime = 0;


//General actions


void TotalTimeElapsedFunction() {  // Total time counter with countdown time display on screen
  totalPeriodDuration = (millis() - totalPeriodStart - pausedDuration);
  // lcd.setCursor(0, 0);
  // lcd.print(totalPeriodDuration / 60000);
  // lcd.print("' ");
}



void TimeElapsedFunction() { // Time counter
  currentFillDuration = (millis() - currentFillStart - (pausedDuration - FillStartpausedDuration));
  TotalTimeElapsedFunction();
  ButtonsFunction();
}



void WaitXmsecFunction(unsigned long WaitDuration) { // Wait X milliseconds
  unsigned long waitStart = millis();
  while ((millis() - waitStart) < WaitDuration) {
    TimeElapsedFunction();
    TotalTimeElapsedFunction();
    ButtonsFunction();
  }
}




double readTemperaturesFunction() { // Measure water temperature
  double Temp;
  sensors.requestTemperatures();
  Temp = sensors.getTempCByIndex(0);
  return Temp;
}



void DisplayTemperatureFunction() { // Display current temperature on screen
  String currentTemp = "";
  currentTemp = currentTemp + int(readTemperaturesFunction()) + (char)223 + "C";
  while (currentTemp.length() < 4) {
    currentTemp = " " + currentTemp;
  }
  lcd.setCursor(12, 0);
  lcd.print(currentTemp);
}


void CountPulses() { // Increment pulse counter on flow sensor interrupt
  pulsecount++; //Increment the variable on every pulse
}


double readWaterFlow() {
  //  Serial.println(F("  readWaterFlow started.")); //temp
  pulsecount = 0;
  interrupts();    //Enables interrupts
  delay (1000);    //Wait 1 second
  noInterrupts();  //Disable the interrupt
  //One second is over now and we have the number of pulses in variable 'pulsecount'
  //Calculating the water flow rate in Milli Liters per minute
  double flowRate;
  flowRate = (pulsecount * 2.22 * 60) / 1000;
  interrupts();
  Serial.print(F("Flow rate = "));
  Serial.print(flowRate);   //Print milliliters per minute on serial monitor
  Serial.println(F(" L/minute"));
  lcd.setCursor(1, 1);
  lcd.print(">");
  lcd.print(flowRate);
  lcd.print("L/min ");
  return flowRate;
  //  Serial.println(F("  readWaterFlow ended.")); //temp
}



int readWaterVolume() { // Read current water level in litres
  //  Serial.println(F("  readWaterVolume started.")); //temp
  //  laserDistance = sensor.readRangeContinuousMillimeters();  //data received in mm  //temporarily removed - to be reused, to be replaced with reading from laser distance sensor and calculation of water volume
  if (digitalRead (fillingPump) == HIGH && currentAutoFilling == true) {  // temp - to be removed// temp - to be removed
    laserDistance -= 4; // temp - to be removed
    delay (40); // temp - to be removed
  }
  if (digitalRead (waterDrawPump) == HIGH) {  // temp - to be removed// temp - to be removed
    laserDistance += 1;  // temp - to be removed// temp - to be removed
    delay (50); // temp - to be removed
  }
  tankProportion = (laserDistance * 1.0) / (tankHeight * 1.0);
  lastWaterVolume = tankCapacity - (tankCapacity * tankProportion);
  // Serial.print(F("    Current water level: ")); //temp
  // Serial.println(lastWaterVolume); //temp
  String currentVolume = "";
  currentVolume = currentVolume + int(lastWaterVolume) + "L";
  while (currentVolume.length() < 4) {
    currentVolume = " " + currentVolume;
  }
  lcd.setCursor(12, 1);
  lcd.print(currentVolume);
  //  lcd.setCursor(12, 1);
  //  lcd.print(lastWaterVolume);
  //  lcd.print("L   ");
  return lastWaterVolume; // temp to be replaced with code that reads actual value
  //  Serial.println(F("  readWaterVolume ended.")); //temp
}



void CheckFillLevelAction() { // Check current water level
  // Serial.println(F("  CheckFillLevelAction started.")); //temp
  lcd.setCursor(0, 1);
  lcd.print(F("Water level:    "));
  CheckSfcntLevelAction ();
  currentWaterVolume = readWaterVolume(); // temp - to be replaced with reading from laser distance sensor and calculation of water volume
  if (currentWaterVolume < minWaterVolume) {  // Water level low
    CheckSfcntLevelAction ();
    Serial.println(F("    Water level low - refilling.")); //temp
    currentAutoFilling = true;
    FillAction();     // Refill
    currentAutoFilling = false;
    lcd.setCursor(0, 1);
    lcd.print(F("Water level:    "));
  }
  CheckSfcntLevelAction();
  CheckForFillingAlarms();
  //  Serial.println(F("  CheckFillLevelAction ended.")); //temp
}



void FillAction() { // Fill water
  Serial.println(F("    FillAction started.")); //temp
  currentWaterVolume = readWaterVolume(); // temporarily removed - to be reused
  // if (currentWaterVolume < minWaterVolume) {          // Water level low - start filling
  DisplayTemperatureFunction();
  ButtonsFunction();
  CheckSfcntLevelAction();
  Serial.print(F("    Auto filling: ")); //temp
  Serial.print(currentAutoFilling); //temp
  Serial.print(F("; Manual filling: ")); //temp
  Serial.println(currentManualFilling); //temp
  currentFillStart = millis();
  FillStartpausedDuration = pausedDuration;   // Save pausedState time at start of filling
  TimeElapsedFunction();
  currentWaterVolume = readWaterVolume();
  while (((currentWaterVolume < maxWaterVolume) && (currentAutoFilling == true) ) || (currentManualFilling == true) ) { // && digitalRead(FloatSwitchSensor) == LOW - to be added
    currentWaterVolume = readWaterVolume();
    FillingPumpONAction();  // Filling pump ON
    CheckSfcntLevelAction();
    CheckForFillingAlarms();
    TimeElapsedFunction();
    ButtonsFunction();
    DisplayTemperatureFunction();
  }
  FillingPumpOFFAction();     // Filling pump OFF
  totalFillDuration += currentFillDuration; // Increase total fill timer with current fill duration
  Serial.print(F("    Current fill duration: "));
  Serial.print(currentFillDuration / 60000);
  Serial.print(F(" minutes; Total fill duration: "));
  Serial.print(totalFillDuration / 60000);
  Serial.println(F(" minutes."));
  currentFillStart = 0;
  // }
  currentWaterVolume = readWaterVolume();
  CheckSfcntLevelAction();
  DisplayTemperatureFunction();
  ButtonsFunction();
  Serial.println("    FillAction ended.");  //temp
}



void FillingPumpONAction() { // Start filling pump
  ButtonsFunction();
  DisplayTemperatureFunction();
  if (digitalRead (fillingPump) == LOW) {
    while (millis() - lastFillStopTime < pauseBetweenFills) {
      lcd.setCursor(0, 1);
      lcd.print(F("Waiting...  "));
      Serial.print(F("    Waiting time = "));
      Serial.println(millis() - lastFillStopTime );
      delay (1000); // Wait if last filling ended less than 2 minutes ago
    }
    digitalWrite (fillingPump, HIGH); // Start filling pump
    Serial.println(F("     Filling pump started."));
    lcd.setCursor(0, 1);
    lcd.print(F("Filling...  "));
  }
}



void FillingPumpOFFAction() { // Stop filling pump
  ButtonsFunction();
  DisplayTemperatureFunction();
  if (digitalRead (fillingPump) == HIGH) {
    digitalWrite (fillingPump, LOW); // Start filling pump
    lastFillStopTime = millis ();  // Record time when filling pump stopped
    Serial.println(F("     Filling pump stopped."));
    lcd.setCursor(0, 1);
    lcd.print(F("Water level:    "));
    lastFillStopTime = millis();           // Reset fill timer
  }
}


void CheckSfcntLevelAction() { // Check current water level to determine if pressure pump can be activated
  // Serial.println(F("   CheckSfcntLevelAction started.")); //temp
  currentWaterVolume = readWaterVolume(); // temp - to be replaced with reading from laser distance sensor and calculation of water volume
  if (currentWaterVolume < presssureMinWaterVolume)  {
    ForbidPressurePumpAction (); // If water level in tank is less than 20 litres, pressure pump forbidden
  }
  else {
    AllowPressurePumpAction (); // Allow pressure pump
  }
  // Serial.println(F("   CheckSfcntLevelAction ended.")); //temp
}



void AllowPressurePumpAction() { // Allow pressure pump to draw water from tank
  ButtonsFunction();
  DisplayTemperatureFunction();
  if (digitalRead (waterDrawPump) == LOW) {
    digitalWrite (waterDrawPump, HIGH);
    Serial.println(F("     Pressure pump allowed.")); //temp
    lcd.setCursor(0, 0);
    lcd.print(F("Pressure:ON "));
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Pressure:ON "));
}



void ForbidPressurePumpAction() { // Forbid pressure pump to draw water from tank
  ButtonsFunction();
  DisplayTemperatureFunction();
  if (digitalRead(waterDrawPump) == HIGH) {
    digitalWrite(waterDrawPump, LOW);
    Serial.println(F("     Pressure pump forbidden.")); //temp
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Pressure:OFF "));
}


void CheckForFillingAlarms() { // Check for filling alarms
  //  Serial.println(F("  CheckForFillingAlarms started.")); //temp
  if ((totalFillDuration + currentFillDuration) >= (totalFillMaxDuration * 60000)) { // Check if total fill time exceeds totalFillMaxDuration minutes - to be developed further
    FillingPumpOFFAction(); // Filling pump OFF
    faultCode = 50; // Fault code 50: Total filling time exceeds totalFillMaxDuration minutes.
    ErrorFunction();
  }
  if (currentFillDuration >= (singleFillMaxDuration * 60000)) {  // Check if filling continuing for more than singleFillMaxDuration minutes
    FillingPumpOFFAction(); // Filling pump OFF
    faultCode = 51; // Fault code 51: Current filling time exceeds singleFillMaxDuration minutes.
    ErrorFunction();
  }
  currentWaterVolume = readWaterVolume();
  if (currentWaterVolume > waterVolumeLimit) {  // Water level above limit - tank overfilled
    Serial.println(F("    Water level too high - tank overfilled.")); //temp
    OverfillAction();     // Overfill action
  }
  if (currentAutoFilling == true) { // Alarms to check additionally if automatic filling is active
    // Check for flow sensor alarms
    // double currentWaterFlow = readWaterFlow();
    readWaterFlow();
    /* temporarily disabled - to be reused
      if (currentWaterFlow < 0.01) {
      delay (10000); // wait 10 seconds
      currentWaterFlow = readWaterFlow();
      if (currentWaterFlow < 0.01) {
        delay (10000); // wait 10 seconds
        currentWaterFlow = readWaterFlow();
        if (currentWaterFlow < 0.01) { // Error: Water flow is less than 0.01 L/min for 3 consecutive measurements, each 10 seconds apart
          FillingPumpOFFAction(); // Filling pump OFF
          faultCode = 60; // Fault code 60: Water flow is less than 0.01 L/min for 3 consecutive measurements, each 10 seconds apart.
          ErrorFunction();
        }
      }
      }
    */
  }
  if (currentManualFilling == true) {
    lcd.setCursor(0, 1);
    lcd.print(F("MANUAL:ON  "));
  }
  // Check for float switch sensor alarms
  //  Serial.println(F("  CheckForFillingAlarms ended.")); //temp
}




void CheckForHeatAlarmsAction() {  // Check for heat alarms - to be developed
  ButtonsFunction();
  DisplayTemperatureFunction();
  if (readTemperaturesFunction() >= overheatLimit) { // Over heating
    ForbidPressurePumpAction(); // Heater OFF
    faultCode = 20; // Foult code 20: Over-heating.
    ErrorFunction();
  }
  if (readTemperaturesFunction() <= freezeAlarmLimit) { // Risk of freezing
    ForbidPressurePumpAction(); // Heater OFF
    faultCode = 21; // Foult code 21: Risk of freezing.
    ErrorFunction();
  }
  // Code for checking of other temperature alarms may be introduced here
  ButtonsFunction();
  DisplayTemperatureFunction();
}

/*
  int ManualFillBtnState = HIGH;
  int ManualFillBtnRecentState = HIGH;
  int ManualFillBtnLastState = HIGH;
  unsigned long lastManualDbncTime = 0;
*/

void ManualFillFunction() { // Manual control of filling pump
  Serial.println(F("  ManualFillFunction started.")); //temp
  ManualFillBtnState = digitalRead(ManualFillBtn);
  if (ManualFillBtnState != ManualFillBtnLastState) { //meaning button changed state
    lastManualDbncTime = millis();
    Serial.println(F("  control point 1.")); //temp
  }
  if ((ManualFillBtnState == LOW) && ((millis() - lastManualDbncTime) >= resetButtonDebounceDelay)) { // button held down long enough to count as Reset
    ManualFillBtnLastState = ManualFillBtnState;
    selectorButtonCode = 4;
    ResetFunction(); // Reset the machine.
    selectorButtonCode = 0;
    return selectorButtonCode;
  }
  Serial.println(F("  control point 1.5.")); //temp
  if ((millis() - lastManualDbncTime) >= buttonDebounceDelay) { // button held down long enough to count as normal push
    Serial.println(F("  control point 1.7.")); //temp
    if (ManualFillBtnState != ManualFillBtnRecentState) {
      ManualFillBtnRecentState = ManualFillBtnState;
      if (ManualFillBtnRecentState == LOW) { //If button has been pushed for long enough time, invert state of currentManualFilling
        Serial.println(F("  control point 2.")); //temp
        currentManualFilling = !currentManualFilling;
        Serial.print(F("  currentManualFilling changed to: ")); //temp
        Serial.println(currentManualFilling); //temp
        tone (buzzer, 700, 80); // Buzz for 80 milliseconds with frequency 700 Hz
      }
    }
    ManualFillBtnLastState = ManualFillBtnState;
    if (currentAutoFilling == true) { // System currently auto-filling tank - initiate confirmation dialog
      // Code....
      lcd.setCursor(0, 1);
      lcd.print(F("Busy with auto filling!"));
      delay (1000);
      return;
    }
    if (currentManualFilling == true) { // Filling pump currently started manually - turn it off
      currentManualFilling = false;
      currentAutoFilling = false;
      tone (buzzer, 700, 100); // Buzz for 100 milliseconds with frequency 700 Hz
      FillingPumpOFFAction(); // Filling pump OFF
      digitalWrite(fillingPumpLED, false);
      Serial.println(F("  control point 3.")); //temp
    }
    else {
      if (currentManualFilling == false) {  // Turn on manual filling
        Serial.println(F("  control point 4.")); //temp
        lcd.setCursor(0, 1);
        lcd.print(F("MANUAL:ON "));
        tone (buzzer, 700, 100); // Buzz for 100 milliseconds with frequency 700 Hz
        currentAutoFilling = false;
        currentManualFilling = true;
        digitalWrite(fillingPumpLED, true);
        FillAction();
        FillingPumpOFFAction(); // Filling pump OFF
        currentManualFilling = false;
        digitalWrite(fillingPumpLED, false);
      }
    }
    lcd.setCursor(0, 1);
    lcd.print(F("Water level: "));
    Serial.println(F("  ManualFillFunction ended.")); //temp
  }
  Serial.println(F("  control point 5.")); //temp
}



unsigned int EncoderFunction(int LowestPosition, int HighestPosition, int EncoderStep) { // Read rotary encoder
  int rotaryEncoderPinAState = digitalRead(rotaryEncoderPinA);
  if ((rotaryEncoderPinALast == LOW) && (rotaryEncoderPinAState == HIGH)) {
    if (digitalRead(rotaryEncoderPinB) == LOW) {
      rotaryEncoderPosition -= EncoderStep;
    } else {
      rotaryEncoderPosition += EncoderStep;
    }
    tone (buzzer, 600, 7); // Buzz for 7 milliseconds with frequency 600 Hz
  }
  rotaryEncoderPinALast = rotaryEncoderPinAState;
  rotaryEncoderPosition = constrain (rotaryEncoderPosition, LowestPosition, HighestPosition);
  // if (rotaryEncoderPosition < StartPosition) rotaryEncoderPosition = StartPosition;
  // if (rotaryEncoderPosition > EndPosition) rotaryEncoderPosition = EndPosition;
  return rotaryEncoderPosition;
}



int ButtonsFunction() { // Start/Pause/Resume/Reset Button
  if (digitalRead(doorOpenSwitch) == LOW) doorOpenSwitchFunction();
  if (digitalRead(ManualFillBtn) == LOW) ManualFillFunction();
  do { // Start loop to check for "pausedStated" state
    selectorButtonState = digitalRead(selectorButton);
    if (selectorButtonState != selectorButtonLastState) { //meaning button changed state
      lastSelectorBtnDbncTime = millis();
    }
    if ((selectorButtonState == LOW) && ((millis() - lastSelectorBtnDbncTime) >= resetButtonDebounceDelay)) { // button held down long enough to count as Reset
      selectorButtonLastState = selectorButtonState;
      selectorButtonCode = 4;
      ResetFunction(); // Reset the machine.
      selectorButtonCode = 0;
      return selectorButtonCode;
    }
    if ((millis() - lastSelectorBtnDbncTime) >= buttonDebounceDelay) { // button held down long enough to count as normal push
      if (selectorButtonState != selectorButtonRecentState) {
        selectorButtonRecentState = selectorButtonState;
        if (selectorButtonRecentState == HIGH) { //If button has been pushed for long enough time, and then released, increment selectorButtonCode by 1
          selectorButtonCode++;
          tone (buzzer, 700, 100); // Buzz for 100 milliseconds with frequency 700 Hz
        }
      }
    }
    selectorButtonLastState = selectorButtonState;
    switch (selectorButtonCode) { //At startup selectorButtonCode will == 0
      case 1:              //Button pushed so selectorButtonCode == 1, start wash cycle, or continue unchanged if already selectorButtonCode == 1
        return selectorButtonCode;
        break;
      case 2:             //Button pushed again during wash so call pausedState function
        pausedStateFunction();
        break;
      case 3:             // button pushed for 3rd time so call restartFun but take 1 down from selectorButtonCode
        ResumeFunction();
        selectorButtonCode = 0;
        break;
      case 4:             // button pushed for so long as to count as reset
        ResetFunction();
        selectorButtonCode = 0;
    }
  }
  while (selectorButtonCode == 1);
  return selectorButtonCode;
}



// Program flow actions



void (* InitiateResetFunction) (void) = 0; // Declaration of reset function


void doorOpenSwitchFunction() { // Door swich action
  Serial.println(F("doorOpenSwitchFunction started.")); //temp
  Serial.println(F("Basement door open!")); //temp
  // Code to be added here
  Serial.println(F("doorOpenSwitchFunction ended.")); //temp
}

void StopAllFunction() {  // Stop all parts of the machine
  Serial.println(F("stopFun started.")); //temp
  digitalWrite(fillingPump, LOW);
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(fillingPumpLED, LOW);
  digitalWrite(overfillEffector, LOW);
  digitalWrite(buzzer, LOW);
  Serial.println(F("stopFun ended.")); //temp
}



void pausedStateFunction() { // pausedState - stop all wash processes and raise pausedState flag
  //Serial.println("pausedStateFun started."); //temp
  if (pausedState == false) {
    tone (buzzer, 600, 500); // Buzz for 500 milliseconds with frequency 700 Hz
    pausedState = true;
    Serial.println(F("Paused state started... ")); //temp
    stopTime = millis();
    prepausedStatefillingPump = digitalRead(fillingPump);
    prepausedStatewaterDrawPump = digitalRead(waterDrawPump);
    prepausedStatefillingPumpLED = digitalRead(fillingPumpLED);
    prepausedStateoverfillEffector = digitalRead(overfillEffector);
    StopAllFunction();
    lcd.setCursor(0, 1);
    lcd.print(F("Paused... "));
  }
  // Serial.println("pausedStateFun ended."); //temp
}


void ResumeFunction() { // Resume wash processes after pausedState
  Serial.println(F("resumeFun started.")); //temp
  lcd.setCursor(0, 1);
  lcd.print(F("Resuming...     "));
  Serial.println(F("Resumed from paused state.")); //temp
  tone (buzzer, 800, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 800, 300); // Buzz for 300 milliseconds with frequency 700 Hz
  pausedState = false;
  pausedDuration = millis() - stopTime;
  digitalWrite(fillingPump, prepausedStatefillingPump);
  digitalWrite(waterDrawPump, prepausedStatewaterDrawPump);
  digitalWrite(fillingPumpLED, prepausedStatefillingPumpLED);
  digitalWrite(overfillEffector, prepausedStateoverfillEffector);
  lcd.clear();
  Serial.println(F("resumeFun ended.")); //temp
}


void ResetFunction() { // Reset wash, drain washer and restart machine
  Serial.println(F("resetFun started.")); //temp
  lcd.clear();
  lcd.home (); // go home
  lcd.print(F("Machine reset...     "));
  tone (buzzer, 700, 1000); // Buzz for 1000 milliseconds with frequency 700 Hz
  StopAllFunction();  // Stop all devices
  delay(2000); //Wait for 2 seconds
  OverfillAction(); // Drain
  pausedState = false;
  stopTime = 0;
  pausedDuration = 0;
  delay(2000); //Wait for 2 seconds
  // Restart machine
  Serial.println(F("Restarting machine...")); //temp
  InitiateResetFunction (); // Restart machine
}


void floodSensororFunction() { // Error sensor - to be developed
  Serial.println(F("floodSensororFunction started.")); //temp
  cli();
  StopAllFunction();
  lcd.setCursor(0, 0);
  lcd.print(F("ERROR - DISCONNECT"));
  lcd.setCursor(0, 1);
  lcd.print(F("THE DISHWASHER FROM SOCKET"));
  while (true) {};
  Serial.println(F("floodSensororFunction ended.")); //temp
}



void ErrorFunction() { // Error encountered - stop all wash processes and show fault code
  Serial.println(F("ErrorFunction started.")); //temp
  tone (buzzer, 600, 3000); // Buzz for 3 seconds with frequency 600 Hz
  StopAllFunction(); // Stop all wash processes
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ERROR Fault:    "));
  lcd.setCursor(12, 0);
  lcd.print(faultCode);
  String faultDescription;
  switch (faultCode) {
    case 20:
      faultDescription = F("20: Over-heating - temperature exceeds ");
      faultDescription = faultDescription + overheatLimit ;
      faultDescription = faultDescription +  F(" degrees Celsius. All wash processes stopped.");
      break;
    case 22:
      faultDescription = F("22: Overfill of water tank! Filling pump stopped.");
      break;
    case 50:
      faultDescription = F("50: Total filling time exceeds ");
      faultDescription = faultDescription + totalFillMaxDuration ;
      faultDescription = faultDescription +  F(" minutes.");
      break;
    case 51:
      faultDescription = F("51: Current filling time exceeds ");
      faultDescription = faultDescription + singleFillMaxDuration ;
      faultDescription = faultDescription +  F(" minutes.");
      break;
    case 60:
      faultDescription = F("60: Water filling flow is less than 0.01 L/min for 3 consecutive measurements 10 seconds apart. Filling pump turned OFF.");
      break;
    default:
      faultDescription = F(": Unspecified error.");
      faultDescription = faultCode + faultDescription;
      break;
  }
  faultDescription = "ERROR! Fault code = " + faultDescription + "    ";
  Serial.println(faultDescription);
  lcd.setCursor(0, 0);
  lcd.autoscroll();
  lcd.print(faultDescription);
  while (true) {
    for (int positionCounter = 0; positionCounter < faultDescription.length(); positionCounter++) {  // scroll one position left:
      lcd.scrollDisplayLeft();
      delay(300);
    }
    for (int positionCounter = 0; positionCounter < faultDescription.length(); positionCounter++) {  // scroll one position right:
      lcd.scrollDisplayRight();
      delay(300);
    }
  };
  /* Fault codes description:
    ======= Fault Codes ====================

    ========================================
  */
  Serial.println(F("ErrorFunction ended.")); //temp
}


void OverfillAction() { // Emergency action if tank is overfilled - to be developed
  Serial.println(F("OverfillAction started.")); //temp
  lcd.setCursor(0, 1);
  lcd.print(F("Overfill! "));
  FillingPumpOFFAction();
  digitalWrite (overfillEffector, HIGH);
  faultCode = 22; // Foult code 22: Overfill.
  ErrorFunction();
  Serial.println(F("OverfillAction ended.")); //temp
}



void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home (); // go home
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  delay (2000);
  Serial.println(F("setup started.")); //temp
  sensor.init(); // Initialize laser sensor
  sensor.startContinuous(); // Initialize laser sensor
  //tankProportion = Pi * (tankDiameter / 100 / 2) * (tankDiameter / 100 / 2); // Tank coefficient in litres for a cyllindrical tank with measurements in mm
  laserDistance = 1100; // temp - to be removed
  pinMode(fillingPump, OUTPUT);
  pinMode(waterDrawPump, OUTPUT);
  pinMode(overfillEffector, OUTPUT);
  pinMode(fillingPumpLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(waterFlowSensor, INPUT_PULLUP);
  pinMode(FloatSwitchSensor, INPUT_PULLUP);
  pinMode(fillingPumpACCurrent, INPUT);
  pinMode(rotaryEncoderPinA, INPUT); // Rotary Encoder DT
  pinMode(rotaryEncoderPinB, INPUT); // Rotary Encoder CLK
  pinMode(selectorButton, INPUT_PULLUP);
  pinMode(ManualFillBtn, INPUT_PULLUP);
  pinMode(doorOpenSwitch, INPUT_PULLUP);
  // pinMode(floodSensor, INPUT);
  digitalWrite(fillingPump, LOW);
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(overfillEffector, LOW);
  digitalWrite(fillingPumpLED, LOW);
  digitalWrite(buzzer, LOW);
  attachInterrupt(digitalPinToInterrupt(waterFlowSensor), CountPulses, FALLING);
  // attachInterrupt(digitalPinToInterrupt(doorOpenSwitch), actdoorOpenSwitch, FALLING); // Not used at this time - door switch is currently checked via polling together with the On/Off button
  // attachInterrupt(digitalPinToInterrupt(3), floodSensororFunction, FALLING); // For future use
  Serial.println("setup ended."); //temp
}


void loop() {
  Serial.println(F("loop started.")); //temp
  readTemperaturesFunction(); // Initialize temperature measurement
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Welcome! Arduino "));
  lcd.setCursor(0, 1);
  lcd.print(F("Water Control"));
  lcd.print(softwareVersion);
  delay(4000);
  lcd.clear();
  // lastFillStopTime = millis () - 100000; // temp - to be removed
  rotaryEncoderPosition = 0;
  totalPeriodStart = millis();
  stopTime = 0;
  pausedDuration = 0;
  pausedState = false;
  Serial.println(F("System started."));
  delay(300);
  tone (buzzer, 700, 300); // Buzz for 300 milliseconds with frequency 700 Hz - system starting
  while (true) {
    ButtonsFunction();
    DisplayTemperatureFunction();
    CheckForHeatAlarmsAction();
    CheckFillLevelAction();
    TimeElapsedFunction();
    ButtonsFunction();
  }
  Serial.println(F("loop ended.")); //temp
}
