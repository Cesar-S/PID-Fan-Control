/* Sketch for experimenting with tuning a PID loop

This sketch controls a fan blowing into a vertical acrylic tube designed to 
contain a floating a ping pong ball. The control algorithm is supposed to 
maintain the ball at a set height from the bottom of the tube.
- Distance measurement is with a SR-04 ultrasonic sensor located at the open end of 
  the tube.
- The fan is controlled with a PWM signal fed into a L293D motor controller with built
  in diode protection. 
- A 2 line LCD display module and a rotary encoder are used to change the PID constants 
  while tuning the loop. The LCD is conencteed with a Shift Register one-wire backpack;
  other types of LCD will requires changes to the class initialisation parameters
  to suit that hardware type.
- PID parameters can be saved to EEPROM and are relaoded at startup.
- Data is logged to the Serial Monitor to allow it to be graphed externally. Arduino IDE
  Serial Plot can also be used, but the timestamp needs to be removed from the data to 
  make the graph legible.

The sketch accompanies the blog article at 
https://arduinoplusplus.wordpress.com/2017/06/10/pid-control-experiment-making-the-testing-rig/

Copyright (c) Marco Colli 2017. This is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public License as published by 
the Free Software Foundation; either version 2.1 of the License, or(at your option) 
any later version.

This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

External library dependencies:
- NewPing          https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
- PID_v1           http://playground.arduino.cc/Code/PIDLibrary
- LiquidCrystal_SR https://bitbucket.org/fmalpartida/new-liquidcrystal/overview
- MD_REncoder      https://github.com/MajicDesigns/MD_REncoder
- MD_KeySwitch     https://github.com/MajicDesigns/MD_KeySwitch
- MD_Menu          https://github.com/MajicDesigns/MD_Menu

External Applications
- PLX DAQ Excel logging http://forum.arduino.cc/index.php?topic=437398.msg3013761#msg3013761
*/

#include <EEPROM.h>
#include <NewPing.h>
#include <PID_v1.h>
#include <LiquidCrystal_SR.h>
#include <MD_REncoder.h>
#include <MD_KeySwitch.h>
#include <MD_Menu.h>

#define DEBUG_MENU 0    // outputs to the serial monitor to debug menu navigation

// Motor controller definitions -------------------------------------
// Remember motor is polarised, so need to connect I1 to HIGH, I2 to LOW
const uint8_t PIN_E1 = 6; // PWM pin

// LCD with Display with SR one-wire backpack -----------------------
// LCD display definitions
const uint8_t LCD_ROWS = 2;
const uint8_t LCD_COLS = 16;

// LCD pin definitions
const uint8_t LCD_CLK_PIN = 9;  // SR clock pin
const uint8_t LCD_DAT_PIN = 8;  // SR data pin

// Rotary Encoder definitions ---------------------------------------
const uint8_t RE_A_PIN = 2;
const uint8_t RE_B_PIN = 3;
const uint8_t CTL_PIN = 4;

// SR-04 ultrasonic distance sensor definitions ---------------------
const uint8_t PIN_TRIG = 12; // Echo sensor trigger pin
const uint8_t PIN_ECHO = 11; // Echo sensor return pin
const uint16_t SAMPLING_PERIOD = 50; // in milliseconds

// EEPROM base address for config data ------------------------------
const uint16_t EEPROM_ADDRESS = 0;

// PWM Control Parameters -------------------------------------------
const uint8_t PWM_CONTROL_RANGE = 75.0;
const uint8_t PWM_BASE_VALUE = (255 - PWM_CONTROL_RANGE);

// Physical constants for the apparatus -----------------------------
const float BALL_DIAMETER = 4.0; // in cm
const float TUBE_LENGTH = 33.0;   // in cm
const float SENSOR_ZERO = (TUBE_LENGTH - BALL_DIAMETER);

// PID Parameters ---------------------------------------------------
// Define Tuning and and Set Point in a structure we can load and 
// save to EEPROM in one hit
struct PIDCfg_t
{
  uint8_t sig[3];
  double Kp, Ti, Td; // PID constants
  double SetP;       // Set point
} PIDCfg;

// Menu definitions for 2 line LCD module ---------------------------
const uint16_t MENU_TIMEOUT = 10000; // in milliseconds
const bool MENU_AUTOSTART = false;   // autostart or not
const uint8_t SP_MAX = (uint8_t)(TUBE_LENGTH - BALL_DIAMETER); // setpoint max value
const uint8_t SP_MIN = (uint8_t)BALL_DIAMETER; // setpoint min value

// Menu function prototypes
bool display(MD_Menu::userDisplayAction_t, char*);
MD_Menu::userNavAction_t navigation(uint16_t &incDelta);
void *mnuValueRqst(MD_Menu::mnuId_t id, bool bGet);
void *mnuSave(MD_Menu::mnuId_t id, bool bGet);
void *mnuPrint(MD_Menu::mnuId_t id, bool bGet);

// Menu Headers --------
const PROGMEM MD_Menu::mnuHeader_t mnuHdr[] =
{
  { 10, "Menu",   10, 15, 0 },
  { 20, "Edit Param", 20, 22, 0 },
};

// Menu Items ----------
const PROGMEM MD_Menu::mnuItem_t mnuItm[] =
{
  { 10, "Step chg SP", MD_Menu::MNU_INPUT, 10 },
  { 11, "Edit Param",  MD_Menu::MNU_MENU, 20 },
  { 12, "Log Data",    MD_Menu::MNU_INPUT, 30 },
  { 13, "Enable CO",   MD_Menu::MNU_INPUT, 31 },
  { 14, "Print Param", MD_Menu::MNU_INPUT, 20 },
  { 15, "Save Config", MD_Menu::MNU_INPUT, 13 },

  { 20, "Edit Kp", MD_Menu::MNU_INPUT, 40 },
  { 21, "Edit Ti", MD_Menu::MNU_INPUT, 41 },  
  { 22, "Edit Td", MD_Menu::MNU_INPUT, 42 },  
};

// Input Items ---------
enum logType_t 
{ 
  LOG_NONE = 0,
  LOG_SERIAL_MONITOR = 1,    // Arduino IDE serial monitor
  LOG_SERIAL_PLOTTER = 2,    // Arduino IDE srial plotter
  LOG_PLX_DAQ = 3,           // PLX-DAQ Excel real-time logger 
};
const PROGMEM char listLogs[] = { "None|Ser Mon|Ser Plot|PLX DAQ" };

const PROGMEM MD_Menu::mnuInput_t mnuInp[] =
{
  { 10, "SetPoint", MD_Menu::INP_INT8, mnuValueRqst, 2, SP_MIN, SP_MAX, 10, nullptr },
  { 13, "Confirm",  MD_Menu::INP_RUN, mnuSave, 0, 0, 0, 0, nullptr },

  { 20, "Print", MD_Menu::INP_RUN, mnuPrint, 0, 0, 0, 0, nullptr },

  { 30, "Type", MD_Menu::INP_LIST, mnuValueRqst, 8, 0, 0, 0, listLogs },

  { 31, "Enable", MD_Menu::INP_BOOL, mnuValueRqst, 0, 0, 0, 10, nullptr },

  { 40, "Kp", MD_Menu::INP_FLOAT, mnuValueRqst, 6, 0, 10000, 1, nullptr },
  { 41, "Ti", MD_Menu::INP_FLOAT, mnuValueRqst, 6, 0, 10000, 1, nullptr },
  { 42, "Td", MD_Menu::INP_FLOAT, mnuValueRqst, 6, 0, 10000, 1, nullptr },
};

// Global variables -------------------------------------------------
// PID variables and constants
double CV, CO;          // Current Value, Current Output
logType_t logType = LOG_NONE; // logging output
bool enableOutput = true; // enable the output of the PID control
uint32_t logCount;      // data log time stamp

// Global Objects ---------------------------------------------------
PID myPID(&CV, &CO, &PIDCfg.SetP, PIDCfg.Kp, PIDCfg.Ti, PIDCfg.Td, DIRECT);

NewPing sonar(PIN_TRIG, PIN_ECHO, 100);

MD_Menu M(navigation, display,// user navigation and display
  mnuHdr, ARRAY_SIZE(mnuHdr), // menu header data
  mnuItm, ARRAY_SIZE(mnuItm), // menu item data
  mnuInp, ARRAY_SIZE(mnuInp));// menu input data

LiquidCrystal_SR lcd(LCD_DAT_PIN, LCD_CLK_PIN);
MD_REncoder  RE(RE_A_PIN, RE_B_PIN);
MD_KeySwitch swCtl(CTL_PIN);

// Menu callback functions ------------------------------------------
void *mnuValueRqst(MD_Menu::mnuId_t id, bool bGet)
// Value request callback for variables
{
  static uint8_t iTemp;
  static uint32_t fTemp;

  switch (id)
  {
  case 10:
    if (bGet)
    {
      iTemp = PIDCfg.SetP;
      return((void *)&iTemp);
    }
    else
      PIDCfg.SetP = iTemp;
    break;

  case 30:
    if (bGet)
    {
      iTemp = (uint8_t)logType;
      return((void *)&iTemp);
    }
    else
    {
      logType = (logType_t)iTemp;
      if (logType != LOG_NONE)
        logHeader();
    }
    break;

  case 31:
    if (bGet) return((void *)&enableOutput);
    break;

  case 40:
    if (bGet)
    {
      fTemp = (uint32_t)(PIDCfg.Kp * 100.0);
      return((void *)&fTemp);
    }
    else
    {
      PIDCfg.Kp = (float)fTemp / 100.0;
      myPID.SetTunings(PIDCfg.Kp, PIDCfg.Ti, PIDCfg.Td);
    }
    break;

  case 41:
    if (bGet)
    {
      fTemp = (uint32_t)(PIDCfg.Ti * 100.0);
      return((void *)&fTemp);
    }
    else
    {
      PIDCfg.Ti = (float)fTemp / 100.0;
      myPID.SetTunings(PIDCfg.Kp, PIDCfg.Ti, PIDCfg.Td);
    }
    break;

  case 42:
    if (bGet)
    {
      fTemp = (uint32_t)(PIDCfg.Td * 100.0);
      return((void *)&fTemp);
    }
    else
    {
      PIDCfg.Td = (float)fTemp / 100.0;
      myPID.SetTunings(PIDCfg.Kp, PIDCfg.Ti, PIDCfg.Td);
    }
    break;
  }

  return(nullptr);
}

void *mnuSave(MD_Menu::mnuId_t id, bool bGet)
  // Value request callback for run code input
{
  paramSave();
}

void *mnuPrint(MD_Menu::mnuId_t id, bool bGet)
// Value request callback for run code input
{
  Serial.print(F("\nPID Kp="));
  Serial.print(PIDCfg.Kp);
  Serial.print(F(" Ti="));
  Serial.print(PIDCfg.Ti);
  Serial.print(F(" Td="));
  Serial.print(PIDCfg.Td);
  Serial.print(F("\n"));

  return(nullptr);
}

#if DEBUG_MENU
// output to the serial monitor
bool display(MD_Menu::userDisplayAction_t action, char *msg)
{
  switch (action)
  {
  case MD_Menu::DISP_CLEAR:
    Serial.print("\n-> CLS");
    break;

  case MD_Menu::DISP_L0:
    Serial.print("\n0> ");
    Serial.print(msg);
    break;

  case MD_Menu::DISP_L1:
    Serial.print("\n1> ");
    Serial.print(msg);
    break;
  }

  return(true);
}
#else
// output to the LCD display
bool display(MD_Menu::userDisplayAction_t action, char *msg)
{
  static char szLine[LCD_COLS + 1] = { '\0' };

  switch (action)
  {
  case MD_Menu::DISP_CLEAR:
    lcd.clear();
    memset(szLine, ' ', LCD_COLS);
    break;

  case MD_Menu::DISP_L0:
    lcd.setCursor(0, 0);
    lcd.print(szLine);
    lcd.setCursor(0, 0);
    lcd.print(msg);
    break;

  case MD_Menu::DISP_L1:
    lcd.setCursor(0, 1);
    lcd.print(szLine);
    lcd.setCursor(0, 1);
    lcd.print(msg);
    break;
  }

  return(true);
}
#endif

MD_Menu::userNavAction_t navigation(uint16_t &incDelta)
{
  uint8_t re = RE.read();

  if (re != DIR_NONE)
  {
    incDelta = (M.isInEdit() ? (1 << abs(RE.speed() / 10)) : 1);
    return(re == DIR_CCW ? MD_Menu::NAV_DEC : MD_Menu::NAV_INC);
  }

  switch (swCtl.read())
  {
  case MD_KeySwitch::KS_PRESS:     return(MD_Menu::NAV_SEL);
  case MD_KeySwitch::KS_LONGPRESS: return(MD_Menu::NAV_ESC);
  }

  return(MD_Menu::NAV_NULL);
}

// EEPROM load and save ---------------------------------------------
void paramLoad(void)
// The structire has a few signature bytes that allow detection of an
// invalid data structure in EEPROM, as would be found the first run.
{
  uint8_t signature[3] = { 'P', 'I', 'D' };

  EEPROM.get(EEPROM_ADDRESS, PIDCfg);

  if (signature[0] != PIDCfg.sig[0] || 
      signature[1] != PIDCfg.sig[1] ||
      signature[2] != PIDCfg.sig[2])
  {
    PIDCfg.sig[0] = signature[0];
    PIDCfg.sig[1] = signature[1];
    PIDCfg.sig[2] = signature[2];
    PIDCfg.Kp = 0;
    PIDCfg.Ti = 0;
    PIDCfg.Td = 0;
    PIDCfg.SetP = 15.0;
  }
};

void paramSave(void)
{
  EEPROM.put(EEPROM_ADDRESS, PIDCfg);
}

// Data logging functions -------------------------------------------
void logHeader(void)
{
  if (logType == LOG_SERIAL_MONITOR || logType == LOG_SERIAL_PLOTTER)
  {
    if (logType == LOG_SERIAL_MONITOR)
    {
      mnuPrint(0, false);
      Serial.print(F("Time\t"));
    }
    Serial.println(F("SP\tCV\tCO"));
  }
  else if (logType == LOG_PLX_DAQ)
  {
    //Serial.println("CLEARDATA"); // clears sheet starting at row 2
    Serial.println("CLEARSHEET"); // clears sheet starting at row 1

    // define colum headings
    Serial.println("LABEL,Date,Time,DeltaT,SP,CV,CO");
    Serial.println("BEEP");
  }
  logCount = 0;
}

void logData(void)
// log data if enabled
// SP, CV, CO
{
  if (logType == LOG_SERIAL_MONITOR || logType == LOG_SERIAL_PLOTTER)
  {
    if (logType == LOG_SERIAL_MONITOR)
    {
      Serial.print(logCount*SAMPLING_PERIOD);
      Serial.print(F("\t"));
    }
    Serial.print((int8_t)PIDCfg.SetP);
    Serial.print(F("\t"));
    Serial.print((int8_t)CV);
    Serial.print(F("\t"));
    Serial.println(CO);
  }
  else if (logType == LOG_PLX_DAQ)
  {
    Serial.print("DATA,DATE,TIME,");
    Serial.print(logCount*SAMPLING_PERIOD);
    Serial.print(",");
    Serial.print((int8_t)PIDCfg.SetP);
    Serial.print(",");
    Serial.print((int8_t)CV);
    Serial.print(",");
    Serial.print(CO);
    Serial.println(",AUTOSCROLL_20");
  }
}

// Standard Setup and loop ------------------------------------------
void setup(void) 
{
  Serial.begin(57600);

  paramLoad();

  // LCD display
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.noCursor();

  // Menu
  M.begin();
  M.setMenuWrap(true);
  M.setAutoStart(MENU_AUTOSTART);
  M.setTimeout(MENU_TIMEOUT);

  // Rotary Encoder
  RE.begin();
  swCtl.begin();
  swCtl.enableRepeat(false);

  // PWM output
  pinMode(PIN_E1, OUTPUT);

  //turn the PID on
  myPID.SetOutputLimits(0, PWM_CONTROL_RANGE);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(SAMPLING_PERIOD);
  myPID.SetTunings(PIDCfg.Kp, PIDCfg.Ti, PIDCfg.Td);
}

void loop(void) 
{
  static uint32_t timeLastSensor = 0; // sensor update timer
  static uint32_t timeLastUpdate = 0; // display update timer

  if (!MENU_AUTOSTART && !M.isInMenu())
  {
    uint16_t delta;
    MD_Menu::userNavAction_t m = navigation(delta);

    switch (m)
    {
    case MD_Menu::NAV_SEL: M.runMenu(true); break;
    case MD_Menu::NAV_INC: PIDCfg.SetP = ((PIDCfg.SetP + delta) <= SP_MAX ? (PIDCfg.SetP + delta) : SP_MAX); break;
    case MD_Menu::NAV_DEC: PIDCfg.SetP = ((PIDCfg.SetP - delta) >= SP_MIN ? (PIDCfg.SetP - delta) : SP_MIN); break;
    }
  }

  // run the menu
  M.runMenu();

  // read the sensor and do the PID
  // Wait between pings. 29ms should be the shortest delay between pings.
  if (millis() - timeLastSensor >= SAMPLING_PERIOD)
  {
    CV = SENSOR_ZERO - sonar.ping_cm();
    timeLastSensor = millis();

    // do PID calcs and output new control value
    myPID.Compute();

    // output value
    if (enableOutput)
      analogWrite(PIN_E1, (uint8_t)(CO + PWM_BASE_VALUE));
    else
      digitalWrite(PIN_E1, LOW);

    if (logType != LOG_NONE)
    {
      logData(); // log the data if enabled
      logCount++;
    }
  }

  // Update the display if not running menu, but don't overload 
  // the LCD module
  if (!M.isInMenu())
  {
    if (millis() - timeLastUpdate > 300)
    {
      timeLastUpdate = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("SP:"));
      lcd.print((uint8_t)PIDCfg.SetP);
      lcd.setCursor(LCD_COLS/2, 0);
      lcd.print(F("CV:"));
      lcd.print((uint8_t)CV);
      lcd.setCursor(0, 1);
      lcd.print(F("CO:"));
      lcd.print(CO+PWM_BASE_VALUE);
    }
  }
}
