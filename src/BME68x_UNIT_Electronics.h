/**
 * BME68x UNIT Electronics Library
 * 
 * Main header file for BME68x sensor library optimized for UNIT Electronics boards.
 * This is a convenience header that includes the main BME68x library.
 * 
 * Compatible Boards:
 * - UNIT TouchDot S3 (ESP32-S3)
 * - UNIT Pulsar ESP32-C6
 * - UNIT DualMCU (ESP32 + RP2040)
 * - Arduino Uno/Mega/Nano
 * - Other Arduino-compatible boards
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Based on original Bosch Sensortec BME68x library
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BME68X_UNIT_ELECTRONICS_H
#define BME68X_UNIT_ELECTRONICS_H

// Include the main BME68x library
#include "bme68xLibrary.h"

// Board-specific pin definitions for UNIT Electronics boards
#if defined(ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    // UNIT TouchDot S3
    #define UNIT_I2C_SDA 9
    #define UNIT_I2C_SCL 8
    #define UNIT_STATUS_LED 25  // NeoPixel
  #elif defined(CONFIG_IDF_TARGET_ESP32C6)
    // UNIT Pulsar ESP32-C6
    #define UNIT_I2C_SDA 6
    #define UNIT_I2C_SCL 7
    #define UNIT_STATUS_LED 15
  #else
    // UNIT DualMCU or generic ESP32
    #define UNIT_I2C_SDA 21
    #define UNIT_I2C_SCL 22
    #define UNIT_STATUS_LED 2
  #endif
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  // Arduino Uno/Nano
  #define UNIT_I2C_SDA A4
  #define UNIT_I2C_SCL A5
  #define UNIT_STATUS_LED 13
#elif defined(ARDUINO_AVR_MEGA2560)
  // Arduino Mega
  #define UNIT_I2C_SDA 20
  #define UNIT_I2C_SCL 21
  #define UNIT_STATUS_LED 13
#endif

// Convenience function declarations
bool beginUNITBME68x(Bme68x& sensor);
void setupUNITBoard();

#endif // BME68X_UNIT_ELECTRONICS_H
