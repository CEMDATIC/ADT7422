/*!
 *  @file ADT7422.cpp
 *
 *  @mainpage ADT7422 I2C Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for Analog Devices ADT7422 I2C Temp sensor
 *
 *  @section author Author
 *
 *  Miguel Angel San Jose Alarcon (CEMDATIC)
 *
 * 	@section license License
 *
 * 	Copyright (C) 2021  Miguel Angel San Jose Alarcon (CEMDATIC)
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "ADT7422.h"

/*!
 *    @brief  Instantiates a new ADT7422 class
 */
ADT7422::ADT7422(void) {}

/*!
 *    @brief  Setups the HW
 *    @param  addr The I2C address, defaults to 0x48
 *    @param  wire The I2C interface, pointer to a TwoWire, defaults to WIre
 *    @return True if initialization was successful, otherwise false.
 */
bool ADT7422::begin(uint8_t addr, TwoWire *wire) {
	
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(addr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  // Check connection
  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, ADT7422_REG__ADT7422_ID, 1);

  // make sure we're talking to the right chip
  if ((chip_id.read() & 0xF8) != 0xC8) {
    // Not detected ... return false
    return false;
  }

  // Soft reset. If this function is used, each register is reset to its default value.
  // reset();

  return true;
}

/*!
 *   @brief  Perform a soft reset
 *   @return True on success
 */
bool ADT7422::reset(void) {
  uint8_t cmd = ADT7422_REG__ADT7422_SWRST;

  if (!i2c_dev->write(&cmd, 1)) {
    return false;
  }
  delay(10);
  return true;
}

/*!
 *   @brief  Writes the requested register
 */
void ADT7422::writeRegister(uint16_t addr_Register, uint8_t width, uint32_t value, uint8_t numbytes) {
Adafruit_BusIO_Register write_reg = Adafruit_BusIO_Register(
      i2c_dev, addr_Register, width, MSBFIRST);	  
	  write_reg.write(value,numbytes);

  return ;
}

/*!
 *   @brief  Reads the register and returns the value 
 *           of the address of the requested register.
 *   @return Value of the address of the requested register.
 */
float ADT7422::readRegister(uint16_t addr_Register, uint8_t width ) {
Adafruit_BusIO_Register read_reg = Adafruit_BusIO_Register(
      i2c_dev, addr_Register, width, MSBFIRST); 
	  
  uint16_t t = read_reg.read();
  
  float temp = (int16_t)t;

  return temp;
}

/*!
 *   @brief  Reads the 16-bit temperature register and returns the Centigrade
 *           temperature as a float.
 *   @return Temperature in Centigrade.
 */
float ADT7422::readTempC() {
Adafruit_BusIO_Register temp_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB, 2, MSBFIRST); 
	  
  uint16_t t = temp_reg.read();
  
  //Serial.println(t);
  float temp = (int16_t)t;
  temp /= 128.0;

  return temp;
}

/*!
 *   @brief  Writes the 16-bit high setpoint temperature register
 */
void ADT7422::writeTempC_HIGH(uint32_t value, uint8_t numbytes) {
Adafruit_BusIO_Register writeHIGH_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_HIGH, 2, MSBFIRST);	  
	  writeHIGH_reg.write(value,numbytes);

  return ;
}

/*!
 *   @brief  Reads the 16-bit high setpoint temperature register and returns the Centigrade
 *           temperature as a float.
 *   @return High setpoint temperature in Centigrade.
 */
float ADT7422::readTempC_HIGH() {
 Adafruit_BusIO_Register readHIGH_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_HIGH, 2, MSBFIRST);	  
  uint16_t t = readHIGH_reg.read();
  
  float temp = (int16_t)t;
  temp /= 128.0;

  return temp;
}

/*!
 *   @brief  Writes the 16-bit low setpoint temperature register
 */
void ADT7422::writeTempC_LOW(uint32_t value, uint8_t numbytes) {
Adafruit_BusIO_Register writeLOW_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_LOW, 2, MSBFIRST);	  
	  writeLOW_reg.write(value,numbytes);

  return ;
}

/*!
 *   @brief  Reads the 16-bit low setpoint temperature register and returns the Centigrade
 *           temperature as a float.
 *   @return Low setpoint temperature in Centigrade.
 */
float ADT7422::readTempC_LOW() {
Adafruit_BusIO_Register readLOW_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_LOW, 2, MSBFIRST);	  
  uint16_t t = readLOW_reg.read();
  
  float temp = (int16_t)t;
  temp /= 128.0;

  return temp;
}

/*!
 *   @brief  Writes the 16-bit critical temperature register
 */
void ADT7422::writeTempC_CRIT(uint32_t value, uint8_t numbytes) {
 Adafruit_BusIO_Register writeCRIT_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_CRIT, 2, MSBFIRST);	  
	  writeCRIT_reg.write(value,numbytes);

  return ;
}

/*!
 *   @brief  Reads the 16-bit critical temperature register and returns the Centigrade
 *           temperature as a float.
 *   @return Critical temperature in Centigrade.
 */
float ADT7422::readTempC_CRIT() {
 Adafruit_BusIO_Register readCRIT_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_TEMPMSB_CRIT, 2, MSBFIRST);	  
  uint16_t t = readCRIT_reg.read();
 
  float temp = (int16_t)t;
  temp /= 128.0;

  return temp;
}

/*!
 *   @brief  Writes the 8-bit configuration register
 */
void ADT7422::writeConfig(uint32_t value, uint8_t numbytes) {
 Adafruit_BusIO_Register writeCONFIG_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_CONFIG, 1, MSBFIRST);	  
	  writeCONFIG_reg.write(value,numbytes);

  return ;
}

/*!
 *   @brief  Reads the 8-bit configuration register and returns the configuration
 *           register as a float.
 *   @return Configuration register in decimal.
 */
float ADT7422::readConfig() {
Adafruit_BusIO_Register readCONFIG_reg = Adafruit_BusIO_Register(
      i2c_dev, ADT7422_REG__ADT7422_CONFIG, 1, MSBFIRST);	  
  uint16_t t = readCONFIG_reg.read();
  
  float temp = (int16_t)t;

  return temp;
}
