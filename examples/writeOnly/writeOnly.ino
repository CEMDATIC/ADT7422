/**************************************************************************/
/*!
This is a demo for writing ADT7422 registers
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

//Here you enter the address of the register you want to read 
//For example: 0x03 is Configuration
uint16_t addr_Register = 0x03;

//Here you write the value you want to give to the register 
//in the address of the register itself.
uint32_t value = 156;

void setup() {
  Serial.begin(115200);
  Serial.println("ADT7422 example, write only");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7422!");
    while (1);
  }
  
  //The temperature value registers (0x00 and 0x01), the status register (0x02), 
  //and the ID register are read only (0x0B)
  if((addr_Register == 0x02) || (addr_Register == 0x03) || (addr_Register == 0x0A) || (addr_Register == 0x0C) || (addr_Register == 0x0D) || (addr_Register == 0x2E) || (addr_Register == 0x2F)){
  tempsensor.writeRegister(addr_Register, 1, value, 1);
  }else{
  tempsensor.writeRegister(addr_Register, 2, value*128, 2);
  }
 
  // sensor takes 250 ms to get first readings
  delay(2500);
}

void loop() {

}
