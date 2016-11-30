/**************************************************************************
 * decode a keyset from the phone
 * input: code representing one or more of pressed keys
 * action: excute command associated with key
**************************************************************************/
#include <string.h>
#include <Arduino.h>

#include "gbg.h"
boolean relayStatus = false;


int decode(int code) 
{
    int rotator = 0;
    int flags = 0;
/**************************************************************************
 * decide what to shut off, if anything on a zero code.
 * tbd this needs to handle combinations of the switches if two or more
 * are simultaneously invoked.
**************************************************************************/
    Serial.println(code, HEX);
    
    if(code == 0){
        if(relayStatus == true){
            relayStatus = false;
            digitalWrite(REV_RELAY_DRV, 0);
            digitalWrite(RUN_RELAY_DRV, 0);
        }
    }
        
    for(rotator = 0; rotator < 8; rotator++) {
        flags = 0xffff & (code & (1 << rotator));
        if(flags != 0) {
            Serial.print("flags are ");
            Serial.println(flags, HEX);
            switch(flags) {
                case 0x02:
                    Serial.println("action = 1");
                    
                    break;
                case 0x04:
                    Serial.println("action = 2");
                    break;
                case 0x08:
                    Serial.println("action = 3");
                    break;
                case 0x10:
                    Serial.println("action = 4");
                    break;
                case 0x20:
                    Serial.println("action = 5");
                    break;
                case 0x40:
                    Serial.println("action = 6");
                    if(relayStatus == false) {
                        relayStatus = true;
                        digitalWrite(REV_RELAY_DRV, 1);
                        digitalWrite(RUN_RELAY_DRV, 1);
                    }
                    break;
                case 0x80:
                    Serial.println("action = 7");
                    break;
                    
                case 0x100:
                    Serial.println("action = 8");
                    break;
                default:
                    break;
            }
            Serial.println("\n");
        }
    }
    Serial.println("\n");
    return (0);
    
}
