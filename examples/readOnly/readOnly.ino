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

// Create the ADT7422 temperature sensor object.
ADT7422 tempsensor = ADT7422();

//Here you enter the address of the register you want to read 
//For example: 0x03 is Configuration
uint16_t addr_Register = 0x03;

void setup() {
  Serial.begin(115200);
  Serial.println("ADT7422 example, read only");
  
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x49) for example
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find ADT7422!");
    while (1);
  }

  // sensor takes 250 ms to get first readings
  delay(2500);
}

void loop() {
  //The software reset (0x2F) register is a write only register. 
  if((addr_Register == 0x03) || (addr_Register == 0x02) || (addr_Register == 0x0B) || (addr_Register == 0x0C) || (addr_Register == 0x0D) || (addr_Register == 0x2E)){
  float c = tempsensor.readRegister(addr_Register, 1);
  Serial.print("Register: "); Serial.println(c); 
  }else if(addr_Register == 0x0A){
  float c = tempsensor.readRegister(addr_Register, 1); 
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("Register: "); Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F"); 
  }else{
  float c = tempsensor.readRegister(addr_Register, 2);
  c /= 128.0;
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("Register: "); Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F");
  }
  delay(1000);
}
