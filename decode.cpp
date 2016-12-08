/**************************************************************************
 * decode a keyset from the phone
 * input: code representing one or more of pressed keys
 * action: excute command associated with key
**************************************************************************/
#include <string.h>
#include <Arduino.h>

#include "gbg.h"
boolean relayStatus = false;

/**************************************************************************
 * The code, also transferred from phone as buttonStatus is a 1 bit
 * shifted left by the button number.  This way two or more buttons
 * can be simultaneously transferred.  That works from the phone to the
 * Arduino BT.
 * Since there are 8 buttons we'll define them.  Defs can change anytime.
**************************************************************************/

int decode(int code) 
{
    int rotator = 0;
    int flags = 0;
/**************************************************************************
 * tbd decide what to shut off, if anything on a zero code.
 * tbd this needs to handle combinations of the switches if two or more
 * are simultaneously invoked.
 *
 * The rotator is a 1 shifted left by a counter.  At each count, if the
 * result of 'and' with the code the switch is run.  Switch decodes the
 * particular bit.
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
                    Serial.println(" ACT_FWD ");
                    break;
                case 0x04:
                    Serial.println(" ACT_SLOWER ");
                    break;
                case 0x08:
                    Serial.println(" ACT_STOP ");
                    break;
                case 0x10:
                    Serial.println(" ACT_START ");
                    break;
                case 0x20:
                    Serial.println(" ACT_FASTER ");
                    break;
                case 0x40:
                    Serial.println(" ACT_REV ");
                    if(relayStatus == false) {
                        relayStatus = true;
                        digitalWrite(REV_RELAY_DRV, 1);
                        digitalWrite(RUN_RELAY_DRV, 1);
                    }
                    break;
                case 0x80:
                    Serial.println(" ACT_LIGHT ");
                    break;
                    
                case 0x100:
                    Serial.println(" ACT_HORN ");
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
