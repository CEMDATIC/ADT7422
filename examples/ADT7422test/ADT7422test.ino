
/**************************************************************************/
/*!
This is a demo for the ADT7422 
*/
/**************************************************************************/

#include <Wire.h>
#include "ADT7422.h"

// Create the ADT7422 temperature sensor object.
ADT7422 tempsensor = ADT7422();

void setup() {
  Serial.begin(115200);
  Serial.println("ADT7422 demo");
  
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
  // Read and print out the temperature, then convert to *F
  float c = tempsensor.readTempC();
  float f = c * 9.0 / 5.0 + 32;
  Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t"); 
  Serial.print(f); Serial.println("*F");

  delay(1000);
}
