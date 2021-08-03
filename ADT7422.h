/*!
 *  @file ADT7422.h
 *
 * 	I2C Driver for Analog Devices ADT7422 I2C Temp sensor
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

#ifndef _ADT7422_H
#define _ADT7422_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

#define ADT7422_I2CADDR_DEFAULT 0x48 ///< I2C address

#define ADT7422_REG__ADT7422_TEMPMSB 0x00 ///< Temp. value MSB
#define ADT7422_REG__ADT7422_TEMPLSB 0x01 ///< Temp. value LSB
#define ADT7422_REG__ADT7422_STATUS 0x02  ///< Status register
#define ADT7422_REG__ADT7422_CONFIG 0x03  ///< Configuration register
#define ADT7422_REG__ADT7422_TEMPMSB_HIGH 0x04 ///< Temp. value MSB_HIGH
#define ADT7422_REG__ADT7422_TEMPLSB_HIGH 0x05 ///< Temp. value LSB_HIGH
#define ADT7422_REG__ADT7422_TEMPMSB_LOW 0x06 ///< Temp. value MSB_LOW
#define ADT7422_REG__ADT7422_TEMPLSB_LOW 0x07 ///< Temp. value LSB_LOW
#define ADT7422_REG__ADT7422_TEMPMSB_CRIT 0x08 ///< Temp. value MSB_CRIT
#define ADT7422_REG__ADT7422_TEMPLSB_CRIT 0x09 ///< Temp. value LSB_CRIT
#define ADT7422_REG__ADT7422_THYST 0x0A      ///< Temp. HYST
#define ADT7422_REG__ADT7422_ID 0x0B      ///< Manufacturer identification
#define ADT7422_REG__ADT7422_RSVD1 0x0C      ///< Reserved
#define ADT7422_REG__ADT7422_RSVD2 0x0D      ///< Reserved
#define ADT7422_REG__ADT7422_RSVD3 0x2E      ///< Reserved
#define ADT7422_REG__ADT7422_SWRST 0x2F  ///< Software reset

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            ADT7422 Temp Sensor
 */
class ADT7422 {
public:
  ADT7422();
  bool begin(uint8_t a = ADT7422_I2CADDR_DEFAULT, TwoWire *wire = &Wire);
  bool reset(void);
  float readRegister(uint16_t addr_Register, uint8_t width );
  float readTempC();
  float readTempC_HIGH();
  float readTempC_LOW();
  float readTempC_CRIT();
  float readConfig();
  
  void writeRegister(uint16_t addr_Register, uint8_t width, uint32_t value, uint8_t numbytes); 
  void writeTempC_HIGH(uint32_t value, uint8_t numbytes);
  void writeTempC_LOW(uint32_t value, uint8_t numbytes);
  void writeTempC_CRIT(uint32_t value, uint8_t numbytes);
  void writeConfig(uint32_t value, uint8_t numbytes);
 
private:
  int32_t _sensorID = 7422;
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
};

#endif
