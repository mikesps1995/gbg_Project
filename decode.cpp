/**************************************************************************
 * decode a keyset from the phone
 * input: code representing one or more of pressed keys
 * action: excute command associated with key
**************************************************************************/
#include <string.h>
#include <Arduino.h>

#include "gbg.h"

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
 *
 * Actions are performed in the loop to coordinate the phone with the
 * hardware commands, so right now this just prints out a result of a button press.
**************************************************************************/
#if 0
    Serial.print("Phone Code ");
    Serial.print(code, HEX);
#endif
    if(code == 0){
           // tbd, not sure what to do here.
    }
    
    for(rotator = 0; rotator < 8; rotator++) {
        flags = 0xffff & (code & (1 << rotator));
        if(flags != 0) {
            Serial.print(" flags: ");
            Serial.print(flags, HEX);
            switch(flags) {
                case 0x02:
                    Serial.print(" ACT_FWD ");
                    break;
                case 0x04:
                    Serial.print(" ACT_SLOWER ");
                    break;
                case 0x08:
                    Serial.print(" ACT_STOP ");
                    break;
                case 0x10:
                    Serial.print(" ACT_START ");
                    break;
                case 0x20:
                    Serial.print(" ACT_FASTER ");
                    break;
                case 0x40:
                    Serial.print(" ACT_REV ");
                    break;
                case 0x80:
                    Serial.print(" ACT_LIGHT ");
                    break;
                    
                case 0x100:
                    Serial.print(" ACT_HORN ");
                    break;
                default:
                    break;
            } // end switch flags
        }     // end if flags !=0
    }         // end rotator
    Serial.println("");
    return (code);
}
