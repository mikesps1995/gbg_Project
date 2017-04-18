/**************************************************************************
checkPhone:
run various commands from smartphone
**************************************************************************/
#include <string.h>
#include <Arduino.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

extern uint8_t packetbuffer[];
extern float parsefloat(uint8_t *buffer) ;
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);


/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

extern int buttonStatus;

void checkPhone(void) 
{
    
/* Wait for new data to arrive */
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
    if (len == 0) return;

       /* Got a packet! */
       // printHex(packetbuffer, len);

       // Color
    if (packetbuffer[1] == 'C') {
        uint8_t red = packetbuffer[2];
        uint8_t green = packetbuffer[3];
        uint8_t blue = packetbuffer[4];
        Serial.print ("RGB #");
        if (red < 0x10) Serial.print("0");
        Serial.print(red, HEX);
        if (green < 0x10) Serial.print("0");
        Serial.print(green, HEX);
        if (blue < 0x10) Serial.print("0");
        Serial.println(blue, HEX);
    }

/**************************************************************************
 * Buttons
 * There are 8 buttons available on the phone application.  This is what
 * is used to control GBG.  All the rest of the code is left for reference
**************************************************************************/

    if (packetbuffer[1] == 'B') {
        uint8_t buttnum = packetbuffer[2] - '0';
        boolean pressed = packetbuffer[3] - '0';
        Serial.print ("Button "); Serial.print(buttnum, HEX);
        Serial.print(" ");
        
        if (pressed) {
            Serial.print(1 << buttnum, HEX);
            buttonStatus |= (1 << buttnum);
            Serial.print(" pressed  ");
        } else {
            buttonStatus &= (~(1 << buttnum));
            Serial.print((~(1 << buttnum)), HEX);
            Serial.print(" released  ");
        }
        Serial.print(" buttonStatus ");
        Serial.print( buttonStatus, HEX);
    }

       // GPS Location
    if (packetbuffer[1] == 'L') {
        float lat, lon, alt;
        lat = parsefloat(packetbuffer+2);
        lon = parsefloat(packetbuffer+6);
        alt = parsefloat(packetbuffer+10);
        Serial.print("GPS Location\t");
        Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
        Serial.print('\t');
        Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
        Serial.print('\t');
        Serial.print(alt, 4); Serial.println(" meters");
    }

       // Accelerometer
    if (packetbuffer[1] == 'A') {
        float x, y, z;
        x = parsefloat(packetbuffer+2);
        y = parsefloat(packetbuffer+6);
        z = parsefloat(packetbuffer+10);
        Serial.print("Accel\t");
        Serial.print(x); Serial.print('\t');
        Serial.print(y); Serial.print('\t');
        Serial.print(z); Serial.println();
    }

       // Magnetometer
    if (packetbuffer[1] == 'M') {
        float x, y, z;
        x = parsefloat(packetbuffer+2);
        y = parsefloat(packetbuffer+6);
        z = parsefloat(packetbuffer+10);
        Serial.print("Mag\t");
        Serial.print(x); Serial.print('\t');
        Serial.print(y); Serial.print('\t');
        Serial.print(z); Serial.println();
    }

       // Gyroscope
    if (packetbuffer[1] == 'G') {
        float x, y, z;
        x = parsefloat(packetbuffer+2);
        y = parsefloat(packetbuffer+6);
        z = parsefloat(packetbuffer+10);
        Serial.print("Gyro\t");
        Serial.print(x); Serial.print('\t');
        Serial.print(y); Serial.print('\t');
        Serial.print(z); Serial.println();
    }

       // Quaternions
    if (packetbuffer[1] == 'Q') {
        float x, y, z, w;
        x = parsefloat(packetbuffer+2);
        y = parsefloat(packetbuffer+6);
        z = parsefloat(packetbuffer+10);
        w = parsefloat(packetbuffer+14);
        Serial.print("Quat\t");
        Serial.print(x); Serial.print('\t');
        Serial.print(y); Serial.print('\t');
        Serial.print(z); Serial.print('\t');
        Serial.print(w); Serial.println();
    }
}

