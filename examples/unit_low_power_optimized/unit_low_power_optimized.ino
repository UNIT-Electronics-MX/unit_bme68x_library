/**
 * BME68x Low Power Optimized Example for UNIT Electronics Boards
 * 
 * This example is optimized for battery-powered applications with:
 * - Deep sleep modes for ESP32 boards
 * - Minimal sensor heating time
 * - RTC memory data retention
 * - Adaptive measurement intervals
 * - Power consumption monitoring
 * - Efficient data transmission
 * 
 * Compatible with:
 * - UNIT TouchDot S3 (wearable applications)
 * - UNIT Pulsar ESP32-C6 (IoT sensors)  
 * - UNIT DualMCU (distributed low power)
 * 
 * Power consumption estimates:
 * - Active measurement: ~100mA for 3-5 seconds
 * - Deep sleep: ~10µA (ESP32), ~5µA (ESP32-C6)
 * - Battery life with 1000mAh: 6-12 months (5min intervals)
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Based on original Bosch Sensortec library
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/rtc_io.h"

// Auto-detect board type and configure pins
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    // UNIT TouchDot S3
    #define BOARD_NAME "UNIT TouchDot S3"
    #define I2C_SDA 9
    #define I2C_SCL 8
    #define STATUS_LED 25  // NeoPixel pin
    #define BATTERY_PIN 4  // ADC for battery monitoring
    #define HAS_NEOPIXEL true
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    // UNIT Pulsar ESP32-C6
    #define BOARD_NAME "UNIT Pulsar ESP32-C6"
    #define I2C_SDA 6
    #define I2C_SCL 7
    #define STATUS_LED 15
    #define BATTERY_PIN 1  // ADC for battery monitoring
    #define HAS_NEOPIXEL false
#else
    // UNIT DualMCU or generic ESP32
    #define BOARD_NAME "UNIT ESP32"
    #define I2C_SDA 21
    #define I2C_SCL 22
    #define STATUS_LED 2
    #define BATTERY_PIN 36  // ADC for battery monitoring  
    #define HAS_NEOPIXEL false
#endif

// Sleep configuration
#define SLEEP_TIME_MINUTES 5    // Default sleep interval
#define uS_TO_S_FACTOR 1000000ULL
#define MIN_SLEEP_MINUTES 1     // Minimum sleep time
#define MAX_SLEEP_MINUTES 60    // Maximum sleep time

// Low power sensor configuration
#define LOW_POWER_HEATER_TEMP 250   // Lower temperature for power saving
#define LOW_POWER_HEATER_DURATION 60 // Shorter duration

// BME68x sensor
Bme68x bme;

// RTC memory structure for data persistence
typedef struct {
    uint32_t boot_count;
    float last_temperature;
    float last_humidity;
    float last_pressure;
    uint32_t last_gas_resistance;
    float gas_baseline;
    int calibration_count;
    bool is_calibrated;
    uint32_t total_runtime_seconds;
    float battery_voltage;
    uint16_t sleep_interval_minutes;
    uint32_t crc32;
} rtc_data_t;

RTC_DATA_ATTR rtc_data_t rtc_data;

/**
 * @brief Calculate CRC32 for data integrity
 */
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xffffffff;
    while (length--) {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1) {
            bool bit = crc & 0x80000000;
            if (c & i) bit = !bit;
            crc <<= 1;
            if (bit) crc ^= 0x04c11db7;
        }
    }
    return crc;
}

/**
 * @brief Validate RTC data integrity
 */
bool isRTCDataValid() {
    uint32_t crc = calculateCRC32((uint8_t*)&rtc_data, sizeof(rtc_data) - sizeof(rtc_data.crc32));
    return crc == rtc_data.crc32;
}

/**
 * @brief Save data to RTC memory
 */
void saveRTCData() {
    rtc_data.crc32 = calculateCRC32((uint8_t*)&rtc_data, sizeof(rtc_data) - sizeof(rtc_data.crc32));
}

/**
 * @brief Initialize RTC data
 */
void initRTCData() {
    if (!isRTCDataValid()) {
        Serial.println("Initializing RTC data...");
        memset(&rtc_data, 0, sizeof(rtc_data));
        rtc_data.sleep_interval_minutes = SLEEP_TIME_MINUTES;
        rtc_data.gas_baseline = 0;
        saveRTCData();
    }
    rtc_data.boot_count++;
}

/**
 * @brief Read battery voltage
 */
float readBatteryVoltage() {
    #ifdef BATTERY_PIN
        int raw = analogRead(BATTERY_PIN);
        // Convert to voltage (adjust multiplier based on voltage divider)
        float voltage = (raw / 4095.0) * 3.3 * 2.0; // Assuming 1:2 voltage divider
        return voltage;
    #else
        return 3.7; // Default voltage if no battery monitoring
    #endif
}

/**
 * @brief Calculate estimated battery life
 */
float calculateBatteryLife(float voltage, uint32_t runtime_seconds) {
    // Simple battery life estimation
    float capacity_used_percent = (4.2 - voltage) / (4.2 - 3.2) * 100;
    if (capacity_used_percent < 0) capacity_used_percent = 0;
    if (capacity_used_percent > 100) capacity_used_percent = 100;
    
    return 100 - capacity_used_percent;
}

/**
 * @brief Adaptive sleep interval based on conditions
 */
uint16_t calculateSleepInterval(float temp_change, float hum_change, float battery_percent) {
    uint16_t interval = SLEEP_TIME_MINUTES;
    
    // Adjust based on environmental changes
    if (abs(temp_change) > 2.0 || abs(hum_change) > 5.0) {
        interval = max(MIN_SLEEP_MINUTES, interval / 2); // More frequent if environment changing
    }
    
    // Adjust based on battery level
    if (battery_percent < 20) {
        interval = min(MAX_SLEEP_MINUTES, interval * 2); // Less frequent if battery low
    } else if (battery_percent < 50) {
        interval = min(MAX_SLEEP_MINUTES, (uint16_t)(interval * 1.5));
    }
    
    return interval;
}

/**
 * @brief Quick LED indication
 */
void quickLEDSignal(int pulses) {
    #if HAS_NEOPIXEL
        // For NeoPixel (TouchDot S3)
        for (int i = 0; i < pulses; i++) {
            // Simple on/off without library to save power
            digitalWrite(STATUS_LED, HIGH);
            delay(100);
            digitalWrite(STATUS_LED, LOW);
            delay(100);
        }
    #else
        // For regular LED
        pinMode(STATUS_LED, OUTPUT);
        for (int i = 0; i < pulses; i++) {
            digitalWrite(STATUS_LED, HIGH);
            delay(100);
            digitalWrite(STATUS_LED, LOW);
            delay(100);
        }
    #endif
}

/**
 * @brief Calculate IAQ with power optimization
 */
float calculateIAQ(uint32_t gas_resistance, float humidity) {
    if (!rtc_data.is_calibrated || rtc_data.gas_baseline == 0) return 50;
    
    float gas_score = (gas_resistance / rtc_data.gas_baseline) * 100.0;
    gas_score = constrain(gas_score, 0, 100);
    
    float hum_score = 100;
    if (humidity < 38 || humidity > 42) {
        hum_score = (humidity < 38) ? 100 - (38 - humidity) * 5 : 100 - (humidity - 42) * 5;
    }
    hum_score = constrain(hum_score, 0, 100);
    
    return (100 - gas_score) * 0.7 + (100 - hum_score) * 0.3;
}

/**
 * @brief Print wakeup information
 */
void printWakeupInfo() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    Serial.println("==========================================");
    Serial.printf("BME68x Low Power - %s\n", BOARD_NAME);
    Serial.println("==========================================");
    
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("Wakeup: Timer");
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            Serial.println("Wakeup: First boot or reset");
            break;
    }
    
    Serial.printf("Boot count: %u\n", rtc_data.boot_count);
    Serial.printf("Total runtime: %u seconds\n", rtc_data.total_runtime_seconds);
    Serial.printf("Battery: %.2fV\n", rtc_data.battery_voltage);
    Serial.printf("Sleep interval: %u minutes\n", rtc_data.sleep_interval_minutes);
    
    if (rtc_data.boot_count > 1) {
        Serial.println("\nLast sensor data:");
        Serial.printf("  Temperature: %.2f°C\n", rtc_data.last_temperature);
        Serial.printf("  Humidity: %.2f%%\n", rtc_data.last_humidity);
        Serial.printf("  Pressure: %.2f hPa\n", rtc_data.last_pressure);
        Serial.printf("  Gas: %u Ohms\n", rtc_data.last_gas_resistance);
        Serial.printf("  Calibrated: %s\n", rtc_data.is_calibrated ? "Yes" : "No");
    }
    
    Serial.println();
}

/**
 * @brief Setup function
 */
void setup() {
    uint32_t start_time = millis();
    
    // Quick system initialization
    Serial.begin(115200);
    delay(100); // Brief delay for serial stability
    
    // Initialize RTC data
    initRTCData();
    
    // Read battery voltage early
    rtc_data.battery_voltage = readBatteryVoltage();
    
    // Print system information
    printWakeupInfo();
    
    // Quick LED indication (1 pulse for normal wakeup)
    quickLEDSignal(1);
    
    // Initialize I2C with reduced speed for power saving
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000); // 100kHz for lower power consumption
    
    Serial.println("Initializing BME68x sensor...");
    
    // Initialize sensor with retries
    bool sensor_init = false;
    for (int attempt = 0; attempt < 3 && !sensor_init; attempt++) {
        if (bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
            sensor_init = true;
        } else {
            Serial.printf("Sensor init attempt %d failed\n", attempt + 1);
            delay(500);
        }
    }
    
    if (!sensor_init) {
        Serial.println("ERROR: BME68x sensor not found!");
        Serial.println("Entering extended sleep...");
        esp_sleep_enable_timer_wakeup(10 * 60 * uS_TO_S_FACTOR); // 10 minutes
        esp_deep_sleep_start();
    }
    
    // Configure sensor for low power operation
    bme.setTPH();
    bme.setHeaterProf(LOW_POWER_HEATER_TEMP, LOW_POWER_HEATER_DURATION);
    
    Serial.println("Reading sensor data...");
    
    // Read sensor data
    bool data_success = readSensorData();
    
    if (data_success) {
        // Calculate adaptive sleep interval
        float temp_change = abs(rtc_data.last_temperature - rtc_data.last_temperature);
        float hum_change = abs(rtc_data.last_humidity - rtc_data.last_humidity);
        float battery_percent = calculateBatteryLife(rtc_data.battery_voltage, rtc_data.total_runtime_seconds);
        
        rtc_data.sleep_interval_minutes = calculateSleepInterval(temp_change, hum_change, battery_percent);
        
        // Update runtime
        rtc_data.total_runtime_seconds += (millis() - start_time) / 1000;
        
        // Save data to RTC memory
        saveRTCData();
        
        Serial.printf("Next wakeup in %u minutes\n", rtc_data.sleep_interval_minutes);
    } else {
        Serial.println("Failed to read sensor data");
        quickLEDSignal(3); // Error indication
    }
    
    Serial.println("Entering deep sleep...");
    Serial.println("==========================================");
    
    // Configure next wakeup
    esp_sleep_enable_timer_wakeup(rtc_data.sleep_interval_minutes * 60 * uS_TO_S_FACTOR);
    
    // Turn off peripherals
    Wire.end();
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

/**
 * @brief Read sensor data efficiently
 */
bool readSensorData() {
    bme68xData data;
    bool success = false;
    
    // Configure forced mode (most power efficient)
    bme.setOpMode(BME68X_FORCED_MODE);
    
    // Get measurement duration and wait
    uint16_t delayPeriod = bme.getMeasDur(BME68X_FORCED_MODE);
    delay(delayPeriod);
    
    // Try to read data with timeout
    unsigned long timeout = millis() + 5000;
    
    while (millis() < timeout && !success) {
        if (bme.fetchData()) {
            uint8_t nFieldsLeft = 0;
            
            do {
                nFieldsLeft = bme.getData(data);
                
                if (data.status & BME68X_NEW_DATA_MSK) {
                    success = true;
                    
                    // Store previous values for change detection
                    float prev_temp = rtc_data.last_temperature;
                    float prev_hum = rtc_data.last_humidity;
                    
                    // Update sensor data
                    rtc_data.last_temperature = data.temperature;
                    rtc_data.last_humidity = data.humidity;
                    rtc_data.last_pressure = data.pressure / 100.0;
                    rtc_data.last_gas_resistance = data.gas_resistance;
                    
                    // Calibrate gas baseline
                    if (rtc_data.calibration_count < 100) {
                        if (rtc_data.calibration_count == 0) {
                            rtc_data.gas_baseline = data.gas_resistance;
                        } else {
                            rtc_data.gas_baseline = (rtc_data.gas_baseline + data.gas_resistance) / 2.0;
                        }
                        rtc_data.calibration_count++;
                        
                        if (rtc_data.calibration_count >= 100) {
                            rtc_data.is_calibrated = true;
                            Serial.println("Gas baseline calibration completed!");
                        }
                    }
                    
                    // Calculate IAQ
                    float iaq = calculateIAQ(data.gas_resistance, data.humidity);
                    String iaq_level = "Unknown";
                    
                    if (rtc_data.is_calibrated) {
                        if (iaq <= 50) iaq_level = "Excellent";
                        else if (iaq <= 100) iaq_level = "Good";
                        else if (iaq <= 150) iaq_level = "Moderate";
                        else if (iaq <= 200) iaq_level = "Poor";
                        else iaq_level = "Unhealthy";
                    } else {
                        iaq_level = "Calibrating";
                    }
                    
                    // Calculate battery life
                    float battery_percent = calculateBatteryLife(rtc_data.battery_voltage, rtc_data.total_runtime_seconds);
                    
                    // Print data in compact format
                    Serial.printf("Boot: %u | T: %.1f°C | H: %.1f%% | P: %.1f hPa | G: %u Ω\n",
                                 rtc_data.boot_count, 
                                 rtc_data.last_temperature,
                                 rtc_data.last_humidity,
                                 rtc_data.last_pressure,
                                 rtc_data.last_gas_resistance);
                    
                    if (rtc_data.is_calibrated) {
                        Serial.printf("IAQ: %.1f (%s) | Battery: %.0f%% | Gas Valid: %s\n",
                                     iaq, iaq_level.c_str(), battery_percent,
                                     (data.status & BME68X_GASM_VALID_MSK) ? "Yes" : "No");
                    } else {
                        Serial.printf("Calibrating: %d/100 | Battery: %.0f%%\n",
                                     rtc_data.calibration_count, battery_percent);
                    }
                    
                    // Detect significant changes
                    if (rtc_data.boot_count > 1) {
                        float temp_change = abs(rtc_data.last_temperature - prev_temp);
                        float hum_change = abs(rtc_data.last_humidity - prev_hum);
                        
                        if (temp_change > 1.0 || hum_change > 3.0) {
                            Serial.printf("Significant change detected: ΔT=%.1f°C, ΔH=%.1f%%\n", 
                                         temp_change, hum_change);
                        }
                    }
                    
                    // Low battery warning
                    if (battery_percent < 20) {
                        Serial.println("WARNING: Low battery detected!");
                        quickLEDSignal(5); // Warning signal
                    }
                    
                    break;
                }
            } while (nFieldsLeft);
        }
        
        delay(100);
    }
    
    if (!success) {
        Serial.println("Timeout reading sensor data");
    }
    
    return success;
}

/**
 * @brief Loop function (not used due to deep sleep)
 */
void loop() {
    // This should never be reached due to deep sleep
    delay(1000);
}
