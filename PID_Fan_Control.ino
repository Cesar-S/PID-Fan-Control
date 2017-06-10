/* Sketch for experimenting with tuning a PID loop

This sketch controls a fan blowing into a vertical acrylic tube designed to 
contain a floating a ping pong ball. The control algorithm is supposed to 
maintain the ball at a set height from the bottom of the tube.
- Distance measurement is with a SR-04 ultrasonic sensor located at the open end of 
  the tube.
- The fan is controlled with a PWM signal fed into a L293N morot controller with built
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
https://arduinoplusplus.wordpress.com

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
const uint16_t DISTANCE_SAMPLE_PERIOD = 50; // in milliseconds

// EEPROM base address for config data ------------------------------
const uint16_t EEPROM_ADDRESS = 0;

// PWM Control Parameters -------------------------------------------
const uint8_t PWM_CONTROL_RANGE = 75.0;
const uint8_t PWM_BASE_VALUE = (255 - PWM_CONTROL_RANGE);

// Physical constants for the apparatus -----------------------------
const float BALL_DIAMETER = 4.0; // in cm
const float SENSOR_MAX = 33.0;   // in cm
const float SENSOR_ZERO = (SENSOR_MAX - BALL_DIAMETER);

// PID Parameters ---------------------------------------------------
// Define Tuning and and Set Point in a structure we can load and 
// save to EEPROM in one hit
struct PIDCfg_t
{
  uint8_t sig[3];
  double kP, kI, kD;     // PID constants
  double SetP;    // Set point
} PIDCfg;

// Menu definitions for 2 line LCD module ---------------------------
const uint16_t MENU_TIMEOUT = 5000; // in milliseconds

// Menu function prototypes
bool display(MD_Menu::userDisplayAction_t, char*);
MD_Menu::userNavAction_t navigation(uint16_t &incDelta);
void *mnuValueRqst(MD_Menu::mnuId_t id, bool bGet);
void *mnuSave(MD_Menu::mnuId_t id, bool bGet);
void *mnuKPrint(MD_Menu::mnuId_t id, bool bGet);

// Menu Headers --------
const PROGMEM MD_Menu::mnuHeader_t mnuHdr[] =
{
  { 10, "Menu",   10, 14, 0 },
  { 20, "Edit K", 20, 22, 0 },
};

// Menu Items ----------
const PROGMEM MD_Menu::mnuItem_t mnuItm[] =
{
  { 10, "Change SP",   MD_Menu::MNU_INPUT, 10 },
  { 11, "Log Output",  MD_Menu::MNU_INPUT, 30 },
  { 12, "Edit K",      MD_Menu::MNU_MENU, 20 },
  { 13, "Print K",     MD_Menu::MNU_INPUT, 20 },
  { 14, "Save Config", MD_Menu::MNU_INPUT, 13 },

  { 20, "Edit kP", MD_Menu::MNU_INPUT, 40 },
  { 21, "Edit kI", MD_Menu::MNU_INPUT, 41 },  
  { 22, "Edit kD", MD_Menu::MNU_INPUT, 42 },  
};

// Input Items ---------
const PROGMEM MD_Menu::mnuInput_t mnuInp[] =
{
  { 10, "SetPoint", MD_Menu::INP_INT8, mnuValueRqst, 2, 4, 28, 10, nullptr },
  { 13, "Confirm",  MD_Menu::INP_RUN, mnuSave, 0, 0, 0, 0, nullptr },

  { 20, "Print", MD_Menu::INP_RUN, mnuKPrint, 0, 0, 0, 0, nullptr },

  { 30, "Log", MD_Menu::INP_BOOL, mnuValueRqst, 0, 0, 0, 10, nullptr },

  { 40, "kP", MD_Menu::INP_FLOAT, mnuValueRqst, 5, 0, 1000, 1, nullptr },
  { 41, "kI", MD_Menu::INP_FLOAT, mnuValueRqst, 5, 0, 1000, 1, nullptr },
  { 42, "kD", MD_Menu::INP_FLOAT, mnuValueRqst, 5, 0, 1000, 1, nullptr },
};

// Global variables -------------------------------------------------
// PID variables and constants
double CV, CO;          // Current Value, Current Output
bool enableLog = false; // log to the Serial Monitor for data collection
uint32_t timeLogStart;  // data log time stamp

// Global Objects ---------------------------------------------------
PID myPID(&CV, &CO, &PIDCfg.SetP, PIDCfg.kP, PIDCfg.kI, PIDCfg.kD, DIRECT);

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
  static uint8_t setP;
  static uint32_t f;

  switch (id)
  {
  case 10:
    if (bGet)
    {
      setP = PIDCfg.SetP;
      return((void *)&setP);
    }
    else
      PIDCfg.SetP = setP;
    break;

  case 30:
    if (bGet) 
      return((void *)&enableLog);
    else if (enableLog)
      {
        mnuKPrint(0, false);
        logHeader();
        timeLogStart = millis();
      }
    break;

  case 40:
    if (bGet)
    {
      f = (uint32_t)(PIDCfg.kP * 100.0);
      return((void *)&f);
    }
    else
    {
      PIDCfg.kP = (float)f / 100.0;
      myPID.SetTunings(PIDCfg.kP, PIDCfg.kI, PIDCfg.kD);
    }
    break;

  case 41:
    if (bGet)
    {
      f = (uint32_t)(PIDCfg.kI * 100.0);
      return((void *)&f);
    }
    else
    {
      PIDCfg.kI = (float)f / 100.0;
      myPID.SetTunings(PIDCfg.kP, PIDCfg.kI, PIDCfg.kD);
    }
    break;

  case 42:
    if (bGet)
    {
      f = (uint32_t)(PIDCfg.kD * 100.0);
      return((void *)&f);
    }
    else
    {
      PIDCfg.kD = (float)f / 100.0;
      myPID.SetTunings(PIDCfg.kP, PIDCfg.kI, PIDCfg.kD);
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

void *mnuKPrint(MD_Menu::mnuId_t id, bool bGet)
// Value request callback for run code input
{
  Serial.print(F("\nPID Kp="));
  Serial.print(PIDCfg.kP);
  Serial.print(F(" Ki="));
  Serial.print(PIDCfg.kI);
  Serial.print(F(" Kd="));
  Serial.print(PIDCfg.kD);
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
    if (M.isInEdit()) incDelta = 1 << abs(RE.speed() / 10);
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
    PIDCfg.kP = 0;
    PIDCfg.kI = 0;
    PIDCfg.kD = 0;
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
  Serial.println(F("Time\tSP\tCV\tCO"));
}

void logData(void)
// log data if enabled
// SP, CV, CO
{
  Serial.print(millis() - timeLogStart);
  Serial.print(F("\t"));
  Serial.print((int8_t)PIDCfg.SetP);
  Serial.print(F("\t"));
  Serial.print((int8_t)CV);
  Serial.print(F("\t"));
  Serial.println(CO);
}

// Standard Setup and loop ------------------------------------------
void setup(void) 
{
  Serial.begin(57600);

  paramLoad();

  // Menu
  M.begin();
  M.setMenuWrap(false);
  M.setAutoStart(true);
  M.setTimeout(MENU_TIMEOUT);

  // Rotary Encoder
  RE.begin();
  swCtl.begin();
  swCtl.enableRepeat(false);

  // LCD display
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear();
  lcd.noCursor();

  // PWM output
  pinMode(PIN_E1, OUTPUT);

  //turn the PID on
  myPID.SetOutputLimits(0, PWM_CONTROL_RANGE);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(DISTANCE_SAMPLE_PERIOD);
  myPID.SetTunings(PIDCfg.kP, PIDCfg.kI, PIDCfg.kD);
}

void loop(void) 
{
  static uint32_t timeLastSensor = 0; // sensor update timer
  static uint32_t timeLastUpdate = 0; // display update timer

  // run the menu
  M.runMenu();

  // read the sensor and do the PID
  // Wait between pings. 29ms should be the shortest delay between pings.
  if (millis() - timeLastSensor >= DISTANCE_SAMPLE_PERIOD)
  {
    CV = SENSOR_ZERO - sonar.ping_cm();
    timeLastSensor = millis();

    // do PID calcs and output new control value
    myPID.Compute();

    // output value
    analogWrite(PIN_E1, (uint8_t)(CO + PWM_BASE_VALUE));

    if (enableLog) logData(); // log the data if enabled
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
