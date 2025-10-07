/**
 * BME68x Basic Example for UNIT TouchDot S3
 * 
 * This example demonstrates how to use BME680/BME688 sensor with the
 * UNIT TouchDot S3 board featuring ESP32-S3 microcontroller.
 * 
 * UNIT TouchDot S3 Features:
 * - ESP32-S3 microcontroller with WiFi & Bluetooth
 * - Circular wearable design (Ø 55mm)
 * - Built-in NeoPixel RGB LED (Pin 25)
 * - QWIIC connector for easy I2C connections
 * - Power switch and LiPo battery support
 * 
 * Hardware Connections:
 * BME68x    TouchDot S3
 * VCC   →   3.3V
 * GND   →   GND
 * SCL   →   GPIO 8 (I2C)
 * SDA   →   GPIO 9 (I2C)
 * 
 * Or use QWIIC connector for plug-and-play connection
 * 
 * More info: https://uelectronics.com/producto/unit-touchdot-s3/
 * GitHub: https://github.com/UNIT-Electronics-MX/unit_touchdot_s3
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Based on original Bosch Sensortec library
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"
#include "Adafruit_NeoPixel.h"

// UNIT TouchDot S3 specific pin definitions
#define I2C_SDA 5
#define I2C_SCL 6
#define NEOPIXEL_PIN 4
#define NEOPIXEL_COUNT 1

// Initialize BME68x sensor and NeoPixel
Bme68x bme;
Adafruit_NeoPixel pixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Colors for air quality indication
#define COLOR_EXCELLENT   pixel.Color(0, 255, 0)    // Green
#define COLOR_GOOD        pixel.Color(100, 255, 0)  // Yellow-Green  
#define COLOR_MODERATE    pixel.Color(255, 255, 0)  // Yellow
#define COLOR_POOR        pixel.Color(255, 100, 0)  // Orange
#define COLOR_UNHEALTHY   pixel.Color(255, 0, 0)    // Red
#define COLOR_INIT        pixel.Color(0, 0, 255)    // Blue (initializing)
#define COLOR_ERROR       pixel.Color(255, 0, 255)  // Magenta (error)

// Calibration variables for IAQ
float gas_baseline = 0;
float hum_baseline = 40.0;
int calibration_count = 0;
bool is_calibrated = false;

/**
 * @brief Calculate Indoor Air Quality index
 */
float calculateIAQ(float gas_resistance, float humidity) {
    if (!is_calibrated) return 50; // Neutral during calibration
    
    float gas_score = (gas_resistance / gas_baseline) * 100.0;
    gas_score = constrain(gas_score, 0, 100);
    
    float hum_score = 100;
    if (humidity < 38 || humidity > 42) {
        hum_score = (humidity < 38) ? 100 - (38 - humidity) * 5 : 100 - (humidity - 42) * 5;
    }
    hum_score = constrain(hum_score, 0, 100);
    
    return (100 - gas_score) * 0.7 + (100 - hum_score) * 0.3;
}

/**
 * @brief Calibrate gas baseline for IAQ calculation
 */
void calibrateGas(float gas_resistance) {
    if (calibration_count < 100) {
        if (calibration_count == 0) {
            gas_baseline = gas_resistance;
        } else {
            gas_baseline = (gas_baseline + gas_resistance) / 2.0;
        }
        calibration_count++;
    } else {
        is_calibrated = true;
    }
}

/**
 * @brief Update NeoPixel color based on IAQ
 */
void updateAirQualityLED(float iaq) {
    uint32_t color;
    
    if (!is_calibrated) {
        color = COLOR_INIT;
    } else if (iaq <= 50) {
        color = COLOR_EXCELLENT;
    } else if (iaq <= 100) {
        color = COLOR_GOOD;
    } else if (iaq <= 150) {
        color = COLOR_MODERATE;
    } else if (iaq <= 200) {
        color = COLOR_POOR;
    } else {
        color = COLOR_UNHEALTHY;
    }
    
    pixel.setPixelColor(0, color);
    pixel.show();
}

/**
 * @brief Get air quality description
 */
String getAirQualityLevel(float iaq) {
    if (!is_calibrated) return "Calibrating";
    if (iaq <= 50) return "Excellent";
    if (iaq <= 100) return "Good";
    if (iaq <= 150) return "Moderate";
    if (iaq <= 200) return "Poor";
    return "Unhealthy";
}

/**
 * @brief Rainbow effect for startup
 */
void rainbowStartup() {
    for (int i = 0; i < 255; i += 15) {
        pixel.setPixelColor(0, pixel.ColorHSV(i * 256));
        pixel.show();
        delay(50);
    }
}

/**
 * @brief Setup function
 */
void setup(void)
{
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {
        delay(10);
    }
    
    Serial.println("========================================");
    Serial.println("BME68x Sensor - UNIT TouchDot S3");
    Serial.println("Wearable Air Quality Monitor");
    Serial.println("========================================");
    
    // Initialize NeoPixel
    pixel.begin();
    pixel.setBrightness(50); // Reduce brightness for wearable use
    rainbowStartup();
    
    // Initialize I2C with TouchDot S3 specific pins
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // 400kHz I2C speed
    
    Serial.println("Initializing BME68x sensor...");
    
    // Initialize sensor
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("ERROR: BME68x sensor not found!");
        Serial.println("Check connections:");
        Serial.println("  VCC → 3.3V");
        Serial.println("  GND → GND");
        Serial.println("  SDA → GPIO 5");
        Serial.println("  SCL → GPIO 6");
        Serial.println("Or use QWIIC connector");
        
        // Error indication with red LED
        while (1) {
            pixel.setPixelColor(0, COLOR_ERROR);
            pixel.show();
            delay(500);
            pixel.clear();
            pixel.show();
            delay(500);
        }
    }
    
    // Check sensor status
    if (bme.checkStatus()) {
        if (bme.checkStatus() == BME68X_ERROR) {
            Serial.println("Sensor error: " + bme.statusString());
            return;
        }
        else if (bme.checkStatus() == BME68X_WARNING) {
            Serial.println("Sensor warning: " + bme.statusString());
        }
    }
    
    Serial.println("Sensor initialized successfully!");
    
    // Configure sensor for wearable application
    bme.setTPH();
    // Lower heater temperature for wearable use (less heat, better battery life)
    bme.setHeaterProf(280, 80);
    
    Serial.println("\nStarting calibration (100 readings)...");
    Serial.println("Keep the device in clean air for best calibration");
    Serial.println("\nOutput format:");
    Serial.println("Time(ms), Temp(°C), Press(hPa), Hum(%), Gas(Ohms), IAQ, Status");
    Serial.println("================================================================");
    
    // Initial LED color
    updateAirQualityLED(0);
}

/**
 * @brief Main loop
 */
void loop(void)
{
    bme68xData data;
    static unsigned long lastReading = 0;
    
    // Read sensor every 3 seconds (good for wearable battery life)
    if (millis() - lastReading >= 3000) {
        lastReading = millis();
        
        // Configure forced mode
        bme.setOpMode(BME68X_FORCED_MODE);
        uint16_t delayPeriod = bme.getMeasDur(BME68X_FORCED_MODE);
        delay(delayPeriod);
        
        // Read data
        if (bme.fetchData()) {
            uint8_t nFieldsLeft = 0;
            
            do {
                nFieldsLeft = bme.getData(data);
                
                if (data.status & BME68X_NEW_DATA_MSK) {
                    // Calibrate gas baseline
                    if (!is_calibrated) {
                        calibrateGas(data.gas_resistance);
                    }
                    
                    // Calculate IAQ
                    float iaq = calculateIAQ(data.gas_resistance, data.humidity);
                    
                    // Update LED based on air quality
                    updateAirQualityLED(iaq);
                    
                    // Print data
                    Serial.print(millis());
                    Serial.print(", ");
                    Serial.print(data.temperature, 2);
                    Serial.print(", ");
                    Serial.print(data.pressure / 100.0, 2);
                    Serial.print(", ");
                    Serial.print(data.humidity, 2);
                    Serial.print(", ");
                    Serial.print(data.gas_resistance);
                    Serial.print(", ");
                    
                    if (is_calibrated) {
                        Serial.print(iaq, 1);
                        Serial.print(", ");
                        Serial.print(getAirQualityLevel(iaq));
                    } else {
                        Serial.print("Calibrating (");
                        Serial.print(calibration_count);
                        Serial.print("/100)");
                    }
                    
                    if (data.status & BME68X_GASM_VALID_MSK) {
                        Serial.print(" [Gas Valid]");
                    } else {
                        Serial.print(" [Heating]");
                    }
                    
                    Serial.println();
                }
            } while (nFieldsLeft);
        } else {
            Serial.println("Failed to fetch data from sensor");
            pixel.setPixelColor(0, COLOR_ERROR);
            pixel.show();
        }
    }
    
    // Small delay for system stability
    delay(10);
}
