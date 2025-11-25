/*!
 * @file  DFRobot_LarkWeatherStation.cpp
 * @brief SMesh Wind Sensor - AS5600 + PCNT implementation
 *
 * @copyright	Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license   The MIT License (MIT)
 * @author    Modified for SMesh Wind Sensor (AS5600 + GPIO PCNT)
 * @version   V2.0
 * @date      2024-11-24
 * @url       https://github.com/meshtastic/DFRobot_LarkWeatherStation
 */
#include "DFRobot_LarkWeatherStation.h"

DFRobot_LarkWeatherStation::DFRobot_LarkWeatherStation()
{
    lastCount = 0;
    lastMillis = 0;
    windSpeed = 0.0;
}

DFRobot_LarkWeatherStation::~DFRobot_LarkWeatherStation()
{
}

int DFRobot_LarkWeatherStation::begin(uint32_t freq)
{
    return init(freq);
}

String DFRobot_LarkWeatherStation::getValue(const char *keys)
{
    if (keys == NULL) return "";

    String key = String(keys);

    if (key.equalsIgnoreCase("Speed")) {
        // Return wind speed in pulses/second
        return String(getWindSpeed(), 2);
    } else if (key.equalsIgnoreCase("Dir")) {
        // Return wind direction as raw 12-bit value (0-4095)
        return String(getWindDirection());
    }

    return "";
}

String DFRobot_LarkWeatherStation::getUnit(const char *keys)
{
    if (keys == NULL) return "";

    String key = String(keys);

    if (key.equalsIgnoreCase("Speed")) {
        return "pulses/sec";
    } else if (key.equalsIgnoreCase("Dir")) {
        return "raw";
    }

    return "";
}

float DFRobot_LarkWeatherStation::getWindSpeed()
{
#ifdef ARCH_ESP32
    uint32_t currentMillis = millis();
    uint32_t deltaMillis = currentMillis - lastMillis;

    // Only update if more than 1000ms have passed
    if (deltaMillis >= 1000) {
        int16_t currentCount = 0;

        // Read PCNT counter value
        pcnt_get_counter_value(PCNT_UNIT_0, &currentCount);

        // Calculate delta (handle overflow/underflow)
        int16_t deltaCount = currentCount - lastCount;

        // Calculate pulses per second
        if (deltaMillis > 0) {
            windSpeed = (float)deltaCount * 1000.0 / (float)deltaMillis;
        }

        // Clear counter and record new timestamp
        pcnt_counter_clear(PCNT_UNIT_0);
        lastCount = 0;  // Reset since we cleared the counter
        lastMillis = currentMillis;

        DBG("Wind Speed: deltaCount=" + String(deltaCount) + " deltaMillis=" + String(deltaMillis) + " speed=" + String(windSpeed));
    }
#endif

    return windSpeed;
}

uint16_t DFRobot_LarkWeatherStation::getWindDirection()
{
    // Read raw angle from AS5600 (0-4095, 12-bit)
    uint16_t rawAngle = as5600.getRawAngle();

    DBG("Wind Direction: rawAngle=" + String(rawAngle));

    return rawAngle;
}

// ========== I2C Implementation ==========

DFRobot_LarkWeatherStation_I2C::DFRobot_LarkWeatherStation_I2C(uint8_t addr, TwoWire *pWire)
  : DFRobot_LarkWeatherStation(), _pWire(pWire), _addr(addr)
{
}

DFRobot_LarkWeatherStation_I2C::~DFRobot_LarkWeatherStation_I2C()
{
}

int DFRobot_LarkWeatherStation_I2C::init(uint32_t freq)
{
    if (_pWire == NULL) {
        DBG("I2C Wire object is NULL");
        return -1;
    }

    // Initialize I2C bus
    _pWire->begin();
    _pWire->setClock(freq);

    // Initialize AS5600 with the I2C bus
    // AS5600 has a fixed I2C address of 0x36
    if (!as5600.begin(AS5600_I2C_ADDR, _pWire)) {
        DBG("AS5600 initialization failed");
        return -2;
    }

    // Check if AS5600 magnet is detected
    if (!as5600.isMagnetDetected()) {
        DBG("AS5600 magnet not detected - sensor may not be properly positioned");
        // Don't return error - magnet might not be present during init
    }

    DBG("AS5600 initialized successfully");

    // Initialize timestamp for wind speed calculation
    lastMillis = millis();

    return 0;
}
