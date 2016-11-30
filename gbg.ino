/*
Fri Nov 11, 2016  8:50 AM
Program to operate on Arduino for Go Baby Go

gbg.ino

*/
/*****************************************************************************
 * Program to run the GBG 
 * Includes:
 * Bluetooth interface for remote control of GBG
 * Digital I/O for various interface features
 * Analog input to read a potentiometer
 * Analog input to read range sensor as a curb detector
 * Analog output function of pot input to provide a control voltage
 *   to a speed control
 * Digital output to control a motor relay
 * Digital output to control a direction relay
 * Digital input to control local or remote control
 *
 * This file is used to control the execution of the various other
 * features of the program set.
 ****************************************************************************/

/*****************************************************************************
 * Basic main loop:
 *  Check BT comms
 *   Actions:=
 *     STOP
 *     REVERSE
 *       if(remote)
 *         FASTER
 *         SLOWER
 *  Check curb sensor
 *    Action:
 *     if(curb_sensed)
 *       STOP
 *  if(controlLocal)
 *    check pot
 *  if(local control)  
 *    if(new_pot)
 *    Change speed
 *  else
 *    if(remote control)
 *      check register and change speed
 *  check operator button
 *    if safe go
 *    else beep "not safe"
 *  update configuration switch register  
 *       
****************************************************************************/

#include "gbg.h"
/**************************************************************************
 * Declare variables 
**************************************************************************/

int potValue = 0;
int range_val = 0;
int cycle_time = 0;

/**************************************************************************
 * Setup the i/o structure and initialize some values.
**************************************************************************/

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);
  Serial.println(F("GBG controller"));
  Serial.println(F("--------------"));
    
  setupBT();

  pinMode(REM_LOCAL, INPUT_PULLUP);
  pinMode(REV_RELAY_DRV, OUTPUT);
  pinMode(RUN_RELAY_DRV, OUTPUT);
  pinMode(REV_RELAY_DRV, OUTPUT);
  pinMode(POT_DRV, OUTPUT);
  
    
  analogWrite(POT_DRV,0);
  
  potValue= analogRead(SPEED_POT);
  range_val = analogRead(RANGE_SENSOR);

  cycle_time = micros();
}
/**************************************************************************

**************************************************************************/



