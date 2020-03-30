//Water Supply System v.1.2.
//Programer: Peter Nikolov.
//Suitable for Leonardo and Uno/Nano.
//A Water Supply System controller that controls and displays the level of water in the tank, and controls a pump, filling the tank, and a pressure pump that draws water from the tank.
//Using Dallas OneWire thermometer and rotary encoder for menu selector, 2 relays control, opto-isolator adviceable.
//Uses pins A0-A3 for the encoder and two buttons, pin 12 for PCI interrupt (flow sensor).
//Note: On Uno and Nano A4 and A5 are for I2C, on Leonardo I2C is on 2 and 3.
//Licensed under Creative Commons.

// #include <YunClient.h>
// #include <BridgeUdp.h>
#include <Console.h>
#include <Bridge.h>
// #include <BridgeServer.h>
// #include <Mailbox.h>
// #include <BridgeSSLClient.h>
// #include <HttpClient.h>
// #include <FileIO.h>
// #include <Process.h>
// #include <BridgeClient.h>
// #include <YunServer.h>
#include <Wire.h>
#include <rgb_lcd.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>    // 1-Wire temperature sensor
#include <VL53L1X.h>              // I2C laser distance sensor
// #include <Protocentral_VL53L0x.h> // Laser distance sensor


//Pin assignment
#define fillingPump 4          // Filling pump 
#define waterDrawPump 5        // Water draw (pressure) pump permission relay
#define fillingPumpACCurrent 6 // Filling pump AC current sensor
#define flowSensor 7           // Flow sensor (or another input that needs interrupt) - do not change - needed for pin interrupt  or PCI
#define overfillEffector 8     // Effector for lowering water level in case of overfill of the tank
#define currentManualFillingLED 9  // Filling pump manual control LED indicator
#define FloatSwitchSensor 12   // Water float switch sensor = backup water level sensor
#define buzzer 10              // Buzzer/beeper
#define ONE_WIRE_BUS 11        // OneWire interface (for thermometers)
#define encoderPinA A0         // Rotary encoder CLK 
#define encoderPinB A1         // Rotary encoder DT 
#define selectorButton A2      // Selector/pause/resume/reset button 
#define manualButton A3        // Manual control button 
#define doorSwitch A4          // Door open switch (A4 for Leonardo, 2 for Nano)


rgb_lcd lcd;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
VL53L1X lasersensor;


// General constants declaration
const String softwareVersion = "1.1";

// Encoder and buttons variables declaration
const int buttonDebounceDelay = 3;        // delay to count as button pressed - 3 milliseconds
const int resetButtonDebounceDelay = 5000; // delay to count as reset - 5 seconds
byte encoderPinALastState = HIGH;          // Rotary encoder DT current state
int encoderPosition = 0;
byte selectorButtonLastState = HIGH;       // Selector/pause/resume/reset button current state
byte manualButtonLastState = HIGH;         // Manual control button current state
unsigned long selectorButtonLDT = 0;       // Selector/pause/resume/reset button last debounce time
unsigned long manualButtonLDT = 0;         // Manual control button last debounce time
byte selectorButtonCode = 0;               // switch case statement to control what each button push does
byte selectorButtonMenu = true;            // switch case statement to control what each button push does
byte manualButtonCode = 0;                 // Manual filling turned OFF
int menuSelectorValue = 0;
byte setParameterValue = 0;                // Returned parameter value number
byte setParameterCount = 0;                // Number of parameter values to choose from (should not be more than 20)
String setParameterNames [] = {"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""}; // Initialize paramter value names


// Filling and pump variables declaration
volatile byte currentAutoFilling = false, currentManualFilling = false;
unsigned long historyByMinutes[6] = {0, 0, 0, 0, 0, 0}; // Array to store filling pump status by minutes for the last 3 hours (192 minutes)
unsigned long currentMinuteTimer = 0;
unsigned long lastFillStartTime = 0, lastFillStopTime = 0, lastFillDuration = 0, fillStartPausedDuration = 0;
unsigned long pauseBetweenFills = 20000; // in milliseconds: 20 seconds - to be changed to 120000 = 2 minutes
unsigned int singleFillMaxDuration = 30; // Maximum allowed duration of a single filling cycle in minutes
unsigned int totalFillMaxDuration = 60;  // Maximum allowed duration of all filling cycles in minutes within the last 3 hours
int currentHistoryPointer = 0; // Number between 0 and 191 - current pointer in the history by minutes
int fillingPumpONDuration = 0; // Number of minutes the filling pump worked during the last 3 hours
int presssureMinWaterVolume = 30; // Lower water level limit to start refilling
int minWaterVolume = 60; // Lower water level limit to start refilling
int maxWaterVolume = 200; // Higher water level limit to stop refilling
int waterVolumeLimit = 280; // Water level limit to count as overfilling
byte pressureMode = 1; // Pressure mode = AUTO (0 = OFF; 1 = AUTO)
byte fillingMode = 1; // Filling mode = AUTO+MANUAL (0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY; 3 = MANUAL ONLY)

// Water level laser distance sensor variables declaration
const int tankCapacity = 300; // Tank capacity in litres
const int tankHeight = 1200; // Tank height in millimeters
signed long laserDistance = 0; // Distance measured by laser sensor in mm
double tankProportion;
int currentWaterVolume;
int waterVolume = 0;

// Flow sensor variables declaration
volatile byte flowSensorReading;
volatile byte flowSensorLastState = HIGH;         // Flow sensor current state
volatile int pulsecount = 0;         // Variable to count number of pulses from flow sensor

// Temperature monitoring variables declaration
const int overheatLimit = 72; // Temperature limit to count for overheating
const int freezeAlarmLimit = 2; // Lowest temperature limit to count for freezing alarm

// Pause-resume and error functions variables declaration
volatile byte pausedState = false;
volatile unsigned long stopTime = 0;       // Record time program was paused
volatile unsigned long pausedDuration = 0; // Record how long the program was paused for
volatile byte prepausedStatefillingPump;
volatile byte prepausedStatewaterDrawPump;
volatile byte prepausedStatecurrentManualFillingLED;
volatile byte prepausedStateoverfillEffector;
volatile int faultCode = 0;

// Door open variables declaration
volatile int doorSwitchState;


//Timing actions


void TotalTimeElapsedFunction() {  // Total time counter with countdown time display on screen
  //  totalPeriodDuration = (millis() - totalPeriodStart - pausedDuration);
  if ((millis() - currentMinuteTimer) >= 60000) { // Check if a full minute has elapsed
    currentMinuteTimer = millis(); // Initialize minute timer
    byte pumpValue = digitalRead (fillingPump);
    byte historyArrayPointer = currentHistoryPointer / 32;
    byte historyArrayBitPointer = currentHistoryPointer - (historyArrayPointer * 32);
    bitWrite(historyByMinutes[historyArrayPointer], historyArrayBitPointer, pumpValue);
    if (currentHistoryPointer > 191) currentHistoryPointer = 0;
    historyArrayPointer = 0;
    fillingPumpONDuration = 0;
    unsigned int n;
    for (historyArrayPointer = 0; historyArrayPointer < 6; historyArrayPointer++) {
      n = historyByMinutes[historyArrayPointer];
      while (n) {
        n &= (n - 1);
        fillingPumpONDuration++;
      }
    }
    currentHistoryPointer++; // Increase history pointer with 1 minute
    Console.print(F("Minutes in the last three hours when fill pump was ON: ")); //temp
    Console.println(fillingPumpONDuration); //temp
    /*
      Console.print(F(" / ")); //temp
      Console.print(currentHistoryPointer); //temp
      Console.print(F(" / ")); //temp
      Console.print(historyByMinutes[0], BIN); //temp
      Console.print(historyByMinutes[1], BIN); //temp
      Console.print(historyByMinutes[2], BIN); //temp
      Console.print(historyByMinutes[3], BIN); //temp
      Console.print(historyByMinutes[4], BIN); //temp
      Console.println(historyByMinutes[5], BIN); //temp
    */
  }
  lcd.setCursor(0, 1); // temp
  lcd.print(F("Total:")); // temp
  lcd.print(fillingPumpONDuration); // temp
  lcd.print(F("'   ")); // temp
}


void TimeElapsedFunction() { // Time counter
  lastFillDuration = (millis() - lastFillStartTime - (pausedDuration - fillStartPausedDuration));
  TotalTimeElapsedFunction();
  ButtonsFunction();
}


/*
  void WaitXmsecFunction(unsigned long WaitDuration) { // Wait X milliseconds
  unsigned long waitStart = millis();
  while ((millis() - waitStart) < WaitDuration) {
    TimeElapsedFunction();
    TotalTimeElapsedFunction();
    ButtonsFunction();
  }
  }
*/

//User interface actions


void UserMenuFunction() { // User menu
  Console.println(F("UserMenuFunction started.")); //temp
  lcd.clear();
  tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
  lcd.setCursor(0, 0);
  lcd.print(F("Menu:"));
  delay (100);
  setParameterNames [0] = F("Pause all procs.");
  setParameterNames [1] = F("Water low limit");
  setParameterNames [2] = F("Water top limit");
  setParameterNames [3] = F("Pressure mode");
  setParameterNames [4] = F("Filling mode");
  setParameterNames [5] = F("Factory reset");
  setParameterNames [6] = F("Go back");
  menuSelectorValue = setParameterFunction(7, 0, ">", "");
  switch (menuSelectorValue) {
    case 0: // Start pause
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      lcd.clear();
      selectorButtonCode = 2;
      ButtonsFunction();
      return;
      break;
    case 1: // Edit lowest water level limit
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      minWaterVolume = setDigitalValueFunction(presssureMinWaterVolume + 5, maxWaterVolume - 5, minWaterVolume, 5, "Water low: ", "L");
      lcd.clear();
      selectorButtonCode = 0;
      Console.print(F("minWaterVolume set manually to ")); //temp
      Console.print(minWaterVolume); //temp
      Console.println("L."); //temp
      return;
      break;
    case 2:
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      maxWaterVolume = setDigitalValueFunction(minWaterVolume + 5, waterVolumeLimit - 5, maxWaterVolume, 5, "Water high: ", "L");
      lcd.clear();
      selectorButtonCode = 0;
      Console.print(F("maxWaterVolume set manually to ")); //temp
      Console.print(maxWaterVolume); //temp
      Console.println("L."); //temp
      return;
      break;
    case 3: // Edit pressure pump mode
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      setParameterNames [0] = "Pressure:OFF";
      setParameterNames [1] = "Pressure:AUTO";
      pressureMode = setParameterFunction(2, pressureMode, ">", "");
      lcd.clear();
      selectorButtonCode = 0;
      Console.print(F("pressureMode set manually to ")); //temp
      Console.println(pressureMode); //temp
      return;
      break;
    case 4: // Edit filling pump mode: 0 = OFF; 1 = AUTO+MANUAL; 2 = AUTO ONLY
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      setParameterNames [0] = "Fill:OFF";
      setParameterNames [1] = "Fill:AUTO+MANU";
      setParameterNames [2] = "Fill:AUTO ONLY";
      setParameterNames [3] = "Fill:MANU ONLY";
      fillingMode = setParameterFunction(4, fillingMode, ">", "");
      lcd.clear();
      selectorButtonCode = 0;
      Console.print(F("fillingMode set manually to ")); //temp
      Console.println(fillingMode); //temp
      return;
      break;
    case 5:  // Factory reset.
      lcd.clear();
      tone (buzzer, 700, 150); // Buzz for 50 milliseconds with frequency 700 Hz
      lcd.setCursor(0, 0);
      lcd.print(F("Reset system?"));
      setParameterNames [0] = "No";
      setParameterNames [1] = "Yes";
      menuSelectorValue = setParameterFunction(2, 0, ">Confirm?:", "");
      switch (menuSelectorValue) {
        case 0: // Answer is No - do nothing and return.
          tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
          selectorButtonCode = 0;
          return;
          break;
        case 1: // Answer is Yes - reset system.
          tone (buzzer, 700, 250); // Buzz for 100 milliseconds with frequency 700 Hz
          ResetFunction();
          break;
      }
    case 6:  // Answer is 6 - do nothing and return.
      lcd.clear();
      selectorButtonCode = 0;
      tone (buzzer, 700, 50); // Buzz for 50 milliseconds with frequency 700 Hz
      return;
      break;
  }
  Console.println(F("UserMenuFunction ended.")); //temp
}



byte setParameterFunction (byte parameterCount, byte defaultParameterValue, String textBefore, String textAfter) {  // Select a menu item - names of values should be pre-set in the string array setParameterNames[]
  selectorButtonCode = 0;
  selectorButtonMenu = false;
  encoderPosition = defaultParameterValue;
  byte menuSelectorValue;
  while (ButtonsFunction() != 1) {
    menuSelectorValue = EncoderFunction(0, parameterCount - 1, 1); //read the value from the rotary encoder
    lcd.setCursor(0, 1);
    lcd.print (textBefore);
    lcd.print(setParameterNames[menuSelectorValue]);
    lcd.print (textAfter);
    lcd.print (F("            "));
  }
  selectorButtonCode = 0;
  selectorButtonMenu = true;
  return menuSelectorValue;
}



int setDigitalValueFunction (int minValue, int maxValue, int defaultValue, int valueStep, String textBefore, String textAfter) { //
  selectorButtonCode = 0;
  selectorButtonMenu = false;
  encoderPosition = defaultValue;
  byte menuSelectorValue;
  while (ButtonsFunction() != 1) {
    menuSelectorValue = EncoderFunction(minValue, maxValue, valueStep); //read the value from the rotary encoder
    lcd.setCursor(0, 1);
    lcd.print (textBefore);
    lcd.print (menuSelectorValue);
    lcd.print (textAfter);
    lcd.print (F("            ")); //
  }
  selectorButtonCode = 0;
  selectorButtonMenu = true;
  return menuSelectorValue;
}


int EncoderFunction (int LowestPosition, int HighestPosition, int EncoderStep) { // Read rotary encoder
  int encoderPinAState = digitalRead(encoderPinA);
  if ((encoderPinAState == LOW) && (encoderPinALastState == HIGH)) {
    if (digitalRead(encoderPinB) == LOW) {
      encoderPosition -= EncoderStep;
    } else {
      encoderPosition += EncoderStep;
    }
    tone (buzzer, 600, 7); // Buzz for 7 milliseconds with frequency 600 Hz
    delay (10);
  }
  encoderPinALastState = encoderPinAState;
  encoderPosition = constrain (encoderPosition, LowestPosition, HighestPosition);
  return encoderPosition;
}



int ButtonsFunction() { // Read buttons and take actions as needed
  do { // Start loop to check for "PausedStated" state
    byte selectorButtonReading = digitalRead (selectorButton);
    if (selectorButtonReading != selectorButtonLastState) {
      //    if ((selectorButtonReading == LOW) && ((millis() - selectorButtonLDT) >= buttonDebounceDelay)) selectorButtonCode++; // If selector button held down after suffcient time to count as normal press, and then released, increment selectorButtonCode
      if (selectorButtonReading == LOW)  selectorButtonCode++; // If selector button held down after suffcient time to count as normal press, and then released, increment selectorButtonCode
      selectorButtonLastState = selectorButtonReading;
      selectorButtonLDT = millis();
    }
    if ((selectorButtonLastState == LOW) && ((millis() - selectorButtonLDT) >= resetButtonDebounceDelay)) { // Selector button held down long enough to count as Reset
      ResetFunction(); // Reset the machine.
      return;
    }
    byte manualButtonReading = digitalRead (manualButton);
    if (manualButtonReading != manualButtonLastState) {
      //  if ((manualButtonReading == LOW) && ((millis() - manualButtonLDT) >= buttonDebounceDelay)) manualButtonCode++; // If manual button held down after suffcient time to count as normal press, and then released, increment manualButtonCode
      if (manualButtonReading == LOW) manualButtonCode++; // If manual button held down after suffcient time to count as normal press, and then released, increment manualButtonCode
      manualButtonLastState = manualButtonReading;
      manualButtonLDT = millis();
    }
    if (digitalRead(doorSwitch) == LOW) doorSwitchFunction();
    switch (selectorButtonCode) { // Actions if selectorButton pressed
      case 1:              // Selector button pushed so selectorButtonCode = 1, return selectorButtonCode, or continue unchanged if already selectorButtonCode = 1
        if (selectorButtonMenu == true) {
          UserMenuFunction();
          selectorButtonMenu = true;
          selectorButtonCode = 0;
        }
        return selectorButtonCode;
        break;
      case 2:             //Button pushed again during operation, so call PausedStateFunction() function
        PausedStateFunction();
        break;
      case 3:             // button pushed for 3rd time so call ResumeFunction(), and selectorButtonCode = 0
        ResumeFunction();
        selectorButtonCode = 0;
        break;
    }
    if (selectorButtonCode > 3) selectorButtonCode = 0;
    if (manualButtonCode > 1) manualButtonCode = 0;
  }
  while (selectorButtonCode == 2);
  return selectorButtonCode;
}


void displayFillingMode() {
  String fillingModeText;
  switch (fillingMode) {
    case 0:
      fillingModeText = F("Fill:OFF    ");
      break;
    case 1:
      fillingModeText = F("Fill:AUTO+M ");
      break;
    case 2:
      fillingModeText = F("Fill:AUTO   ");
      break;
    case 3:
      fillingModeText = F("Fill:MANUAL ");
      break;
  }
  if (currentManualFilling) fillingModeText = F("MANUAL: ON  ");
  lcd.setCursor(0, 1);
  lcd.print(fillingModeText);
}


// Sensor reading actions


int readWaterVolume() { // Read current water level in litres
  // Console.println(F("  readWaterVolume started.")); //temp
  // laserDistance = lasersensor.readRangeContinuousMillimeters();  //data received in mm  //temporarily removed - to be reused, to be replaced with reading from laser distance sensor and calculation of water volume
  // if (digitalRead (fillingPump) == HIGH && currentAutoFilling == true) {  // temp - to be removed// temp - to be removed
  if (digitalRead (fillingPump) == HIGH) {  // temp - to be removed// temp - to be removed
    laserDistance -= 4; // temp - to be removed
    // delay (40); // temp - to be removed
  }
  if (digitalRead (waterDrawPump) == HIGH) {  // temp - to be removed// temp - to be removed
    laserDistance += 1;  // temp - to be removed// temp - to be removed
    // delay (50); // temp - to be removed
  }
  tankProportion = (laserDistance * 1.0) / (tankHeight * 1.0);
  waterVolume = tankCapacity - (tankCapacity * tankProportion);
  String currentVolume = "";
  currentVolume = currentVolume + int(waterVolume) + "L";
  while (currentVolume.length() < 4) {
    currentVolume = " " + currentVolume;
  }
  lcd.setCursor(12, 1);
  lcd.print(currentVolume);
  return waterVolume; // temp to be replaced with code that reads actual value
  //  Console.println(F("  readWaterVolume ended.")); //temp
}


double readWaterFlow() {  // Read water flow sensor and return flow in L/min
  pulsecount = 0;
  interrupts();
  delay (500);    //Wait 1/2 second
  noInterrupts();  //Disable the interrupt
  //One second is over now and we have the number of pulses in variable 'pulsecount'
  //Calculating the water flow rate in liters per minute
  double flowRate;
  flowRate = ((pulsecount * 2.0) / 4.8); //flowRate in L/min (to be calibrated)
  interrupts();
  Console.print(F("     Flow rate = "));
  Console.print(flowRate);   //Print milliliters per minute on serial monitor
  Console.println(F(" L/minute"));
  lcd.setCursor(1, 1);
  lcd.print(">");
  lcd.print(flowRate);
  lcd.print("L/min ");
  return flowRate;
}


void flowSensorISR() { // ISR for input pin 7 interrupt
  noInterrupts();
  flowSensorReading = digitalRead (flowSensor);
  if (flowSensorReading != flowSensorLastState) {
    if (flowSensorReading == LOW) pulsecount++;
    flowSensorLastState = flowSensorReading;
  }
  interrupts();
}


double readTemperatureFunction() { // Measure water temperature
  double Temp;
  sensors.requestTemperatures();
  Temp = sensors.getTempCByIndex(0);
  String currentTemp = "";
  currentTemp = currentTemp + int(Temp) + (char)223 + "C";
  while (currentTemp.length() < 4) {
    currentTemp = " " + currentTemp;
  }
  lcd.setCursor(12, 0);
  lcd.print(currentTemp);
  return Temp;
}


//Filling and pump actions


void fillControlAction() { // Activate auto or manual filling if applicable
  //  Console.println(F("  fillControlAction started.")); //temp
  autoPressureAction();
  displayFillingMode();
  currentWaterVolume = readWaterVolume();
  CheckForFillingAlarms();
  ButtonsFunction();
  currentManualFilling = ((fillingMode == 1) || (fillingMode == 3)) && (manualButtonCode == 1);
  if (currentWaterVolume > maxWaterVolume + 3) { // Do not allow manual filling if tank near to overfill
    manualButtonCode = 0;
    currentManualFilling = false;
    lcd.setCursor (0, 1);
    lcd.print (F("Tank too full!    "));
    delay (500);
  };
  ButtonsFunction();
  if ((!currentManualFilling && ((fillingMode == 1)) || (fillingMode == 2))) { // Check if auto filling is allowed
    if (currentWaterVolume < minWaterVolume) { // Water level low - initiate auto refill
      currentAutoFilling = true;
      Console.println(F("     Water level low - auto refilling.")); //temp
    }
  }
  else currentAutoFilling = false;
  if (currentWaterVolume > maxWaterVolume) currentAutoFilling = false; // Turn off auto filling if tank is full enough
  ButtonsFunction();
  displayFillingMode();
  CheckForFillingAlarms();
  autoPressureAction();
  // Print status to console - begin
  String fillControlActionParams = ""; //temp
  Console.print( F("    fillControlAction parameters: currentWaterVolume / fillingMode / currentManualFilling / currentAutoFilling = ")); //temp
  fillControlActionParams += currentWaterVolume; //temp
  fillControlActionParams += F("/"); //temp
  fillControlActionParams += fillingMode; //temp
  fillControlActionParams += F("/"); //temp
  fillControlActionParams += currentManualFilling; //temp
  fillControlActionParams += F("/"); //temp
  fillControlActionParams += currentAutoFilling; //temp
  Console.println(fillControlActionParams); // temp
  // Print status to console - end
  if (currentManualFilling) digitalWrite(currentManualFillingLED, HIGH);
  else digitalWrite(currentManualFillingLED, LOW);
  if (currentManualFilling || currentAutoFilling) FillingPumpONAction(); // Start/maintain active filling pump
  if (!currentManualFilling && !currentAutoFilling) FillingPumpOFFAction(); // Stop/maintain non-active filling pump
  CheckForFillingAlarms();
  autoPressureAction();
  TimeElapsedFunction();
  // Console.println(F("  fillControlAction ended.")); //temp
}



void FillingPumpONAction() { // Start filling pump
  //  Console.println(F("    FillingPumpONAction started."));
  if (digitalRead (fillingPump) == LOW) {
    if (millis() - lastFillStopTime < pauseBetweenFills) {
      lcd.setCursor(0, 1);
      lcd.print(F("Waiting...  "));
      Console.print(F("    Waiting time = "));
      Console.println(millis() - lastFillStopTime );
      return;
    }
    digitalWrite (fillingPump, HIGH); // Start filling pump
    lastFillStartTime = millis();
    fillStartPausedDuration = pausedDuration;
    Console.println(F("     Filling pump started."));
  }
  else {
    while (fillingPumpONDuration >= totalFillMaxDuration) { // Check if pump working for too long and wait if needed
      FillingPumpOFFAction();
      TimeElapsedFunction();
      ButtonsFunction();
      lcd.setCursor(0, 1);
      lcd.print(F("Waiting...  "));
      Console.println(F("    Filling stopped and waiting due to overtime during last 3 hours."));
    };
  }

  //  Console.println(F("    FillingPumpONAction ended."));
}



void FillingPumpOFFAction() { // Stop filling pump
  //  Console.println(F("    FillingPumpOFFAction started."));
  if (digitalRead (fillingPump) == HIGH) {
    digitalWrite (fillingPump, LOW); // Stop filling pump
    lastFillStopTime = millis ();    // Record time when filling pump stopped
    TimeElapsedFunction();
    Console.print(F("    Last fill duration: "));
    Console.print(lastFillDuration / 60000);
    Console.println(F(" minutes."));
    Console.println(F("     Filling pump stopped."));
  }
  //  Console.println(F("    FillingPumpOFFAction ended."));
}


void autoPressureAction() { // Check current water level to determine if pressure pump can be activated
  // Console.println(F("    CheckSfcntLevelAction started.")); //temp
  if (pressureMode == 1) {
    currentWaterVolume = readWaterVolume(); // temp - to be replaced with reading from laser distance sensor and calculation of water volume
    if (currentWaterVolume < presssureMinWaterVolume)  {
      ForbidPressurePumpAction (); // If water level in tank is less than 20 litres, pressure pump forbidden
    }
    else {
      AllowPressurePumpAction (); // Allow pressure pump
    }
  }
  else ForbidPressurePumpAction ();
  // Console.println(F("    CheckSfcntLevelAction ended.")); //temp
}



void AllowPressurePumpAction() { // Allow pressure pump to draw water from tank
  if (digitalRead (waterDrawPump) == LOW) {
    digitalWrite (waterDrawPump, HIGH);
    Console.println(F("     Pressure pump allowed.")); //temp
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Pressure:ON "));
  readTemperatureFunction();
}



void ForbidPressurePumpAction() { // Forbid pressure pump to draw water from tank
  if (digitalRead(waterDrawPump) == HIGH) {
    digitalWrite(waterDrawPump, LOW);
    Console.println(F("     Pressure pump forbidden.")); //temp
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Pressure:OFF"));
  readTemperatureFunction();
}


void CheckForFillingAlarms() { // Check for filling alarms
  //  Console.println(F("  CheckForFillingAlarms started.")); //temp
  currentWaterVolume = readWaterVolume();
  if (currentWaterVolume > waterVolumeLimit) {  // Check for tank overfill
    FillingPumpOFFAction();
    digitalWrite (overfillEffector, HIGH);
    faultCode = 22; // Foult code 22: Overfill.
    ErrorFunction();
  }
  if (digitalRead (fillingPump) == HIGH) {
    if (fillingPumpONDuration > (totalFillMaxDuration + 3)) { // Check if total fill time exceeds totalFillMaxDuration minutes - to be developed further
      FillingPumpOFFAction(); // Filling pump OFF
      faultCode = 50; // Fault code 50: Total filling time exceeds totalFillMaxDuration minutes.
      ErrorFunction();
    }
    if (lastFillDuration >= (singleFillMaxDuration * 60000)) {  // Check if filling continuing for more than singleFillMaxDuration minutes
      FillingPumpOFFAction(); // Filling pump OFF
      faultCode = 51; // Fault code 51: Current filling time exceeds singleFillMaxDuration minutes.
      ErrorFunction();
    }
    if (currentAutoFilling == true) { // Check for no flow if automatic filling is active
      // Check for flow sensor alarm
      double currentWaterFlow = readWaterFlow();
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
  }
  // Check for float switch sensor alarms
  //  Console.println(F("  CheckForFillingAlarms ended.")); //temp
}


void CheckForHeatAlarmsAction() {  // Check for heat alarms - to be developed
  ButtonsFunction();
  readTemperatureFunction();
  if (readTemperatureFunction() >= overheatLimit) { // Over heating
    ForbidPressurePumpAction(); // Heater OFF
    faultCode = 20; // Foult code 20: Over-heating.
    ErrorFunction();
  }
  if (readTemperatureFunction() <= freezeAlarmLimit) { // Risk of freezing
    ForbidPressurePumpAction(); // Heater OFF
    faultCode = 21; // Foult code 21: Risk of freezing.
    ErrorFunction();
  }
  // Code for checking of other temperature alarms may be introduced here
  ButtonsFunction();
  readTemperatureFunction();
}



// Errors and alarms


void doorSwitchFunction() { // Door swich action
  Console.println(F("doorSwitchFunction started.")); //temp
  Console.println(F("Basement door open!")); //temp
  // Code to be added here
  Console.println(F("doorSwitchFunction ended.")); //temp
}



void ErrorFunction() { // Error encountered - stop all wash processes and show fault code
  Console.println(F("ErrorFunction started.")); //temp
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
      faultDescription = faultDescription +  F(" degrees Celsius. All pump processes stopped.");
      break;
    case 22:
      faultDescription = F("22: Overfill of water tank! Filling pump stopped. Overfill effector started.");
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
  Console.println(faultDescription);
  while (true) {
    for (int positionCounter = 1; positionCounter < faultDescription.length() - 1; positionCounter++) { // scroll one position left:
      lcd.setCursor(0, 1);
      lcd.print(faultDescription.substring(positionCounter, positionCounter + 17));
      delay(250);
    }
  };
  /* Fault codes description:
    ======= Fault Codes ====================


    ========================================
  */
  // Console.println(F("ErrorFunction ended.")); //temp
}


// Program flow actions


void (* InitiateResetFunction) (void) = 0; // Declaration of reset function



void StopAllFunction() {  // Stop all parts of the machine
  Console.println(F("stopFun started.")); //temp
  digitalWrite(fillingPump, LOW);
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(currentManualFillingLED, LOW);
  digitalWrite(overfillEffector, LOW);
  digitalWrite(buzzer, LOW);
  Console.println(F("stopFun ended.")); //temp
}



void PausedStateFunction() { // pausedState - stop all processes and raise pausedState flag
  //Console.println("pausedStateFun started."); //temp
  if (pausedState == false) {
    tone (buzzer, 600, 500); // Buzz for 500 milliseconds with frequency 700 Hz
    pausedState = true;
    Console.println(F("Paused state started... ")); //temp
    Console.print(F("selectorButtonCode: ")); //temp
    Console.print(selectorButtonCode);
    Console.print(F("; manualButtonCode: ")); //temp
    Console.print(manualButtonCode);
    Console.print(F("; currentAutoFilling: ")); //temp
    Console.print(currentAutoFilling);
    Console.print(F("; currentAutoFilling: ")); //temp
    Console.println(currentManualFilling);
    stopTime = millis();
    prepausedStatefillingPump = digitalRead(fillingPump);
    prepausedStatewaterDrawPump = digitalRead(waterDrawPump);
    prepausedStatecurrentManualFillingLED = digitalRead(currentManualFillingLED);
    prepausedStateoverfillEffector = digitalRead(overfillEffector);
    StopAllFunction();
    lcd.setCursor(0, 1);
    lcd.print(F("Paused... "));
  }
  // Console.println("pausedStateFun ended."); //temp
}


void ResumeFunction() { // Resume wash processes after pausedState
  Console.println(F("resumeFun started.")); //temp
  lcd.setCursor(0, 1);
  lcd.print(F("Resuming...     "));
  Console.println(F("Resumed from paused state.")); //temp
  tone (buzzer, 800, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 800, 300); // Buzz for 300 milliseconds with frequency 700 Hz
  pausedState = false;
  pausedDuration = millis() - stopTime;
  digitalWrite(fillingPump, prepausedStatefillingPump);
  digitalWrite(waterDrawPump, prepausedStatewaterDrawPump);
  digitalWrite(currentManualFillingLED, prepausedStatecurrentManualFillingLED);
  digitalWrite(overfillEffector, prepausedStateoverfillEffector);
  lcd.clear();
  Console.println(F("resumeFun ended.")); //temp
}


void ResetFunction() { // Reset wash, drain washer and restart machine
  Console.println(F("resetFun started.")); //temp
  lcd.clear();
  lcd.home (); // go home
  lcd.print(F("Machine reset...     "));
  tone (buzzer, 700, 1000); // Buzz for 1000 milliseconds with frequency 700 Hz
  StopAllFunction();  // Stop all devices
  delay(2000); //Wait for 2 seconds
  pausedState = false;
  stopTime = 0;
  pausedDuration = 0;
  delay(2000); //Wait for 2 seconds
  // Restart machine
  Console.println(F("Restarting machine...")); //temp
  InitiateResetFunction (); // Restart machine
}


// Main program actions


void setup() {
  Wire.begin(); // Initialize I2C
  lasersensor.init(); // Initialize laser sensor
  lasersensor.setTimeout(500);
  lasersensor.setDistanceMode (VL53L1X::Long);
  lasersensor.setMeasurementTimingBudget(15000);
  lasersensor.startContinuous(15); // Initialize laser sensor
  /*
    noInterrupts(); // Assign pin change interrupts for pin 12
    PCICR |= 0b00000001;    // turn on port B
    PCMSK0 |= 0b00001000;   // turn on pin on PB0, which is PCINT4, corresponding to physical pin 12
    // PCMSK0 |= 0b00001000;   // turn on PCI on pin 12 only; pins 8-14 are mapped from left to right, i.e. 0b10001000 means interrupts for pins 8 and 12 are allowed
    interrupts();
  */
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home (); // go home
  Bridge.begin();
  Console.begin();
  // Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  delay (2000);
  Console.println(F("setup() running.")); //temp
  pinMode(fillingPump, OUTPUT);
  pinMode(waterDrawPump, OUTPUT);
  pinMode(overfillEffector, OUTPUT);
  pinMode(currentManualFillingLED, OUTPUT);
  pinMode(FloatSwitchSensor, INPUT_PULLUP);
  pinMode(doorSwitch, INPUT_PULLUP);
  pinMode(fillingPumpACCurrent, INPUT);
  pinMode(encoderPinA, INPUT_PULLUP);       // Rotary encoder CLK
  pinMode(encoderPinB, INPUT_PULLUP);       // Rotary encoder DT
  pinMode(selectorButton, INPUT_PULLUP);    // Selector button
  pinMode(manualButton, INPUT_PULLUP);      // Manual fill button
  pinMode(flowSensor, INPUT_PULLUP);        // Flow sensor or another input that needs interrupt
  pinMode(buzzer, OUTPUT);                  // Buzzer
  digitalWrite(buzzer, LOW);
  digitalWrite(fillingPump, LOW);
  digitalWrite(waterDrawPump, LOW);
  digitalWrite(overfillEffector, LOW);
  digitalWrite(currentManualFillingLED, LOW);
  // attachInterrupt(digitalPinToInterrupt(doorSwitch), doorSwitchAction, FALLING);
  attachInterrupt(digitalPinToInterrupt(flowSensor), flowSensorISR, FALLING);
  readTemperatureFunction(); // Initialize temperature measurement
  laserDistance = lasersensor.readRangeContinuousMillimeters();  //data received in mm  //temporarily removed - to be reused, to be replaced with reading from laser distance sensor and calculation of water volume
  laserDistance = 600; // temp - to be removed
  Console.println("setup ended."); //temp
}


void loop() {
  Console.println(F("loop started.")); //temp
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Welcome! Arduino "));
  lcd.setCursor(0, 1);
  lcd.print(F("WaterControl "));
  lcd.print(softwareVersion);
  delay(3000);
  lcd.clear();
  encoderPosition = 0;
  stopTime = 0;
  pausedDuration = 0;
  pausedState = false;
  selectorButtonCode = 0;
  selectorButtonMenu = true;
  manualButtonCode = 0;
  Console.println(F("System started."));
  tone (buzzer, 700, 300); // Buzz for 300 milliseconds with frequency 700 Hz - system starting
  readTemperatureFunction();
  CheckForHeatAlarmsAction();
  autoPressureAction();
  currentHistoryPointer = 0;
  currentMinuteTimer = millis(); // Initialize current minute timer
  while (true) {
    ButtonsFunction();
    readTemperatureFunction();
    CheckForHeatAlarmsAction();
    autoPressureAction();
    ButtonsFunction();
    fillControlAction();
    TimeElapsedFunction();
  }
  Console.println(F("loop ended.")); //temp
}


// Interrupt actions

/*
  ISR(PCINT0_vect) { // Do not rename! ISR for interface input change interrupts - Port B, PCINT0 - PCINT7
  noInterrupts();
  flowSensorReading = digitalRead (flowSensor);
  if (flowSensorReading != flowSensorLastState) {
    if (flowSensorReading == LOW) pulsecount++;
    flowSensorLastState = flowSensorReading;
  }
  interrupts();
  }
*/
