
/*
Fri Nov 11, 2016  8:50 AM
Program to operate on Arduino for Go Baby Go
Program structure.
  Came with the BT demo code
 BluefruitConfig.h
 packetParser.cpp     

  Admin and convenience files 
 stars                          // for comment blocks
 TAGS                           // EMACS tags for finding vars and functions
 README.txt                     // usually notes on the program
 Doxyfile                       // A documentation scheme with cross refs
 docs                           // where the documentation lives
 backlog.tar                    // incremental backup for recovering from
                                // errors
 
 gbg.ino                        // Ardino schema for compiler
 chkPhone.cpp
       This is code provided by Adafruit that operates with the "app" in several
       clever ways.  We are using only the "button" aspect of the code but there
       are several other interesting parts to play with later
 gbg.h                          // header file specific to GBG
 decode.cpp                     // command decoder
 controller.cpp                 // main part of code 

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

#include <string.h>
#include <Arduino.h>

#include "gbg.h"

/**************************************************************************
 * GBG code begins here
 * Declare variables 
**************************************************************************/
void checkPhone(void);
int decode(int);                // declared as external



#define OP_STATUS 1
#define RANGE_STATUS 2
#define REV_STATUS 4
#define REM_STATUS 8
#define RANGETRIP 1024/2        // tbd actual trip value
#define RANGE_MIN 470           // to run from a min max value for the
#define RANGE_MAX 560           // time being

int potValue = 0;
int range_val = 0;
int cycle_time = 0;
int temp_time;
int potVolts;
int localTime;
int buttonStatus = 0x0;
int oldButtonStatus = 0xffff;
int phone_cmd = 0;
int composite = 0;
int range_switch = 0;
int op_switch = 0;

int testmode = RUNMANUAL | TESTPHONE;


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
  pinMode(REM_LOCAL, INPUT);
  pinMode(REV_RELAY_DRV, OUTPUT);
  pinMode(RUN_RELAY_DRV, OUTPUT);
  pinMode(REV_RELAY_DRV, OUTPUT);
  pinMode(POT_DRV, OUTPUT);
  analogWrite(POT_DRV,0);
  potValue= analogRead(SPEED_POT);
  range_val = analogRead(RANGE_SENSOR);
  cycle_time = micros();

  if(testmode & TESTPHONE == TESTPHONE){
      setupBT();
  }

}

/**************************************************************************
 * main loop to read commands and run them.
**************************************************************************/

void loop(void)
{
#if 0    
    Serial.print("\ntestmode: ");
    Serial.println(testmode);
#endif    
    if(testmode & TESTPHONE == TESTPHONE){
        checkPhone();
    }

/**************************************************************************
 * Tue Apr 18, 2017  2:49 PM
 * The button status (global) is returned from the phone.  Right now decode
 * only scans the buttons and prints stuff.  Phone_cmd is actually button
 * status and should be used to act on phone button presses.
**************************************************************************/
    
    if (buttonStatus != oldButtonStatus) {
#if 0
        Serial.print("button status: 0X");
        Serial.println(buttonStatus, HEX);
#endif
        oldButtonStatus = buttonStatus;
        phone_cmd  = decode(buttonStatus);
    }

    localTime = micros();
    temp_time = localTime - cycle_time;
    cycle_time = localTime;

/**************************************************************************
 * TBD
 * check digital inputs
 * act on digital inputs
 * Check analog 5 (is being used as a din)
 * act on A5
 * read range sensor; print data
 * compare to threshold
 * act on data
 * run relays per switches
 *
 * Composite bits are used to build a control word.  Based on this word
 * the rev and run relays are controlled.
 * this is the decider we talked about.
 **************************************************************************/
    
    if((testmode & RUNMANUAL) == RUNMANUAL) {
        delay(100);

        op_switch  = analogRead(OP_1);
#if 0
        Serial.print(F("\nop_switch: "));
        Serial.println(op_switch);
#endif
        if(op_switch > 1024/2) {
            Serial.print("op released ");
            composite &= ~OP_STATUS;
        }
        else {
            Serial.print("op pressed ");
            composite |= OP_STATUS;
        }

/**************************************************************************
 range switch is analog input
 The actual threshold values are tbd
 Current sensor has 5 -18 inches for a
 output of 2.8 to 1.2 volts (count from 580 to 257)
 not much dynamic range for the distances we are going to see
************************************************************************/
        range_switch  = analogRead( RANGE_SENSOR);
        Serial.print(F("range_switch: "));
        Serial.print(range_switch);

        if((range_switch >= RANGE_MIN) && (range_switch <= RANGE_MAX)) {
            composite |= RANGE_STATUS;
        }
        else {
            composite &= ~RANGE_STATUS;
        }
        
        if(digitalRead(REVERSE) == 0) {
            Serial.print(" reverse pressed ");
            composite |= REV_STATUS;
        }
        else {
            Serial.print(" reverse released ");
            composite &= ~REV_STATUS;
        }
        
        if(digitalRead(REM_LOCAL) == 0)
            composite |=  REM_STATUS;
        else
            composite &=  ~REM_STATUS;
        
        Serial.print("composite ");
        Serial.print(composite, HEX);
        Serial.println("");
        
        
// set relays according to the state of phone and switches
// phone_cmd
// composite
/*
  tbd, here is the logic from phone.  this will need to be integrated
  with the composite value.  Interesting homework.
  A nice way to do this might be combine the commands from the phone to
  override the commands from the switches by changing the composite.
  
        if(phone_cmd & ACT_START == ACT_START)
        if(phone_cmd & ACT_REV == ACT_REV)
*/        

           // range status must override OP_STATUS
           // except in reverse
           // just disable op_status here
        if ((composite & (OP_STATUS | RANGE_STATUS)) == (OP_STATUS | RANGE_STATUS)) {
            digitalWrite(RUN_RELAY_DRV, 1);
        }
        else{
            digitalWrite(RUN_RELAY_DRV, 0);
        }
                
        if((composite & REV_STATUS) == REV_STATUS){
            digitalWrite(REV_RELAY_DRV, 1);
            digitalWrite(RUN_RELAY_DRV, 1);
        }
        else {
            digitalWrite(REV_RELAY_DRV, 0);
        }

    } // end testmode == RUNMANUAL

/**************************************************************************
 * some test code to run a relay based on the pot input.  Two switch
 * points to run rev relay and the run relay
 **************************************************************************/

    if((testmode & TESTIO) == TESTIO){
        potVolts = analogRead(SPEED_POT);
        analogWrite(POT_DRV, potVolts / 4);
        Serial.print(F("potVolts: "));
        Serial.println(potVolts);
    }
  
    if((testmode & TESTIO) == TESTIO){
        if(potVolts > 500){
            digitalWrite(REV_RELAY_DRV, 1);
        }
        else {
            digitalWrite(REV_RELAY_DRV, 0);
        }
        if(potVolts > 700){       // test run relay too
            digitalWrite(RUN_RELAY_DRV, 1);
        }
        else {
            digitalWrite(RUN_RELAY_DRV, 0);
        }
    }
  
    
#if 0    
    potValue = analogRead(POT_OUT);
    range_val = analogRead(RANGE_SENSOR);
#endif
        

}
