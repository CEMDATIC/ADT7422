/**************************************************************************/
/*!
This is a demo for writing and reading ADT7422 temperature registers
*/
/**************************************************************************/

#include <Wire.h>
#include "ADT7422.h"

/******************************************************************************************/
/*!
Register Address                     Description                     Power-On Default
    0x00               Temperature value most significant byte             0x00
    0x01               Temperature value least significant byte            0x00
    0x02               Status                                              0x00
    0x03               Configuration                                       0x00
    0x04               T_HIGH setpoint most significant byte               0x20 (64ºC)
    0x05               T_HIGH setpoint least significant byte              0x00 (64ºC)
    0x06               T_LOW setpoint most significant byte                0x05 (10ºC)
    0x07               T_LOW setpoint least significant byte               0x00 (10ºC)
    0x08               T_CRIT setpoint most significant byte               0x49 (147ºC)
    0x09               T_CRIT setpoint least significant byte              0x80 (147ºC)
    0x0A               T_HYST setpoint                                     0x05 (5ºC)
    0x0B               ID                                                  0xCB
    0x0C               Reserved                                            0xXX
    0x0D               Reserved                                            0xXX
    0x2E               Reserved                                            0xXX
    0x2F               Software reset                                      0xXX
*/
/*****************************************************************************************/

/*************************************************************************************************************************/
/*!
To change the configuration, see page 14 of the specification sheet:
https://www.analog.com/media/en/technical-documentation/data-sheets/ADT7422.pdf

For example: 156 in x1 is 10011100

 Bit                                                    Description
[1:0]              These two bits set the number of undertemperature/overtemperature faults that can occur before
                   setting the INT pin and CT pin. This helps to avoid false triggering due to temperature noise.
                   00 = 1 fault (default).
       
  2                This bit selects the output polarity of the CT pin.
                   1 = active high.
                   
  3                This bit selects the output polarity of the INT pin.
                   1 = active high.
                   
  4                This bit selects between comparator mode and interrupt mode.
                   1 = comparator mode.
                   
[5:6]              These two bits set the operational mode for the ADT7422.
                   00 = continuous conversion (default). When one conversion is finished, the ADT7422 starts another.
                   
  7                This bit sets up the resolution of the ADC when converting.
                   1 = 16-bit resolution. Sign bit + 15 bits gives a temperature resolution of 0.0078°C.
*/
/**********************************************************************************************************************/

// Create the ADT7422 temperature sensor object.
ADT7422 tempsensor = ADT7422();

uint32_t x1 = 156;
uint8_t y1 = 1;

uint32_t x2 = 64;
uint8_t y2 = 2;

uint32_t x3 = 10;
uint8_t y3 = 2;

uint32_t x4 = 147;
uint8_t y4 = 2;

void setup() {
  Serial.begin(115200);
  Serial.println("ADT7422 example, writing and reading registers");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7422!");
    while (1);
  }

  tempsensor.writeConfig(x1, y1);
  tempsensor.writeTempC_HIGH(x2*128, y2);
  tempsensor.writeTempC_LOW(x3*128, y3);
  tempsensor.writeTempC_CRIT(x4*128, y4);
  
  // sensor takes 250 ms to get first readings
  delay(2500);
}

void loop() {
  float r = tempsensor.readConfig();
  Serial.print("Config: "); Serial.println(r);
  
  // Read and print out the temperature, then convert to *F
  float g = tempsensor.readTempC_HIGH();
  float h = g * 9.0 / 5.0 + 32;
  Serial.print("Temp_HIGH: "); Serial.print(g); Serial.print("*C\t"); 
  Serial.print(h); Serial.println("*F");
  
  float i = tempsensor.readTempC_LOW();
  float j = i * 9.0 / 5.0 + 32;
  Serial.print("Temp_LOW: "); Serial.print(i); Serial.print("*C\t"); 
  Serial.print(j); Serial.println("*F");

  float k = tempsensor.readTempC_CRIT();
  float l = k * 9.0 / 5.0 + 32;
  Serial.print("Temp_CRIT: "); Serial.print(k); Serial.print("*C\t"); 
  Serial.print(l); Serial.println("*F");

  delay(1000);

}
