/*!
 * @file  DFRobot_LarkWeatherStation.h
 * @brief SMesh Wind Sensor - AS5600 + PCNT implementation
 * @copyright	Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    Modified for SMesh Wind Sensor (AS5600 + GPIO PCNT)
 * @version   V2.0
 * @date      2024-11-24
 * @url       https://github.com/meshtastic/DFRobot_LarkWeatherStation
 */
#ifndef _DFROBOT_LARKWEATHERSTATION_H_
#define _DFROBOT_LARKWEATHERSTATION_H_

#include "Arduino.h"
#include "Wire.h"

#ifdef ARCH_ESP32
#include "driver/pcnt.h"
#endif

#include <Adafruit_AS5600.h>

//#define ENABLE_DBG ///< Enable this macro to see the detailed execution process of the program
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

// AS5600 I2C address (fixed)
#define AS5600_I2C_ADDR 0x36

// AS5600 register addresses for angle reading
#define AS5600_RAW_ANGLE_H 0x0C  // Raw angle high byte (bits 11:8 in bits 3:0)
#define AS5600_RAW_ANGLE_L 0x0D  // Raw angle low byte (bits 7:0)

class DFRobot_LarkWeatherStation{
public:

  /**
   * @fn DFRobot_LarkWeatherStation
   * @brief SMesh Wind Sensor Class Constructor.
   */
  DFRobot_LarkWeatherStation();

  /**
   * @fn  ~DFRobot_LarkWeatherStation
   * @brief SMesh Wind Sensor Class Destructor.
   */
  ~DFRobot_LarkWeatherStation();

  /**
   * @fn begin
   * @brief Initialize the SMesh Wind Sensor (AS5600 + PCNT)
   *
   * @param freq Set I2C communication frequency (default 100kHz)
   * @return int Init status
   * @n       0  Init successful
   * @n      -1  The communication interface class & object are not passed in
   * @n      -2  Check if the hardware connection is correct
   */
  int begin(uint32_t freq = 100000);

  /**
   * @brief Get sensor data
   *
   * @param keys Data to be obtained ("Speed" for wind speed, "Dir" for direction)
   * @return String Returns the acquired data
   */
  String getValue(const char *keys);

  /**
   * @brief Get data unit
   *
   * @param keys Data for which units need to be obtained
   * @return String Returns the obtained units
   */
  String getUnit(const char *keys);

  /**
   * @brief Get wind speed from PCNT counter
   *
   * @return float Wind speed in pulses per second
   */
  float getWindSpeed();

  /**
   * @brief Get wind direction from AS5600
   *
   * @return uint16_t Raw 12-bit angle value (0-4095)
   */
  uint16_t getWindDirection();

protected:
  /**
   * @fn init
   * @brief Pure virtual function, interface init
   *
   * @param freq     Communication frequency
   * @return Init status
   * @n       0    Init succeeded
   * @n      -1    Interface object is null pointer
   * @n      -2    Device does not exist
   */
  virtual int init(uint32_t freq) = 0;

  Adafruit_AS5600 as5600;        ///< AS5600 magnetic encoder instance
  int16_t lastCount = 0;         ///< Last PCNT counter value
  uint32_t lastMillis = 0;       ///< Last millis() timestamp
  float windSpeed = 0.0;         ///< Cached wind speed value
};

class DFRobot_LarkWeatherStation_I2C:public DFRobot_LarkWeatherStation {

public:
  /**
   * @brief Initialize SMesh Wind Sensor I2C object
   *
   * @param addr AS5600 I2C address (fixed at 0x36)
   * @param pWire I2C object (default &Wire)
   */
  DFRobot_LarkWeatherStation_I2C(uint8_t addr = AS5600_I2C_ADDR, TwoWire *pWire = &Wire);
  ~DFRobot_LarkWeatherStation_I2C();

protected:
  /**
   * @fn init
   * @brief Initialize I2C interface
   *
   * @param freq Set I2C communication frequency
   * @return int Init status
   * @n       0  Init successful
   * @n      -1  The communication interface class & object are not passed in
   * @n      -2  Check if the hardware connection is correct
   */
  int init(uint32_t freq);

private:
  TwoWire *_pWire;
  uint8_t _addr;
};

// Type alias for compatibility
typedef DFRobot_LarkWeatherStation_I2C SMesh_Wind_Sensor_I2C;

#endif
