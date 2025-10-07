/**
 * BME68x Dual-Core Example for UNIT DualMCU ESP32 + RP2040
 * 
 * This example demonstrates how to use both MCUs in the UNIT DualMCU board:
 * - ESP32: Handles WiFi communication and sensor data processing
 * - RP2040: Manages sensor reading and data collection
 * - Inter-MCU communication via I2C or UART
 * - Distributed processing for enhanced performance
 * 
 * UNIT DualMCU Features:
 * - ESP32-WROOM-32E (WiFi & Bluetooth)
 * - Raspberry Pi RP2040 (Dual-core ARM Cortex-M0+)
 * - Both MCUs can run independently
 * - Shared I2C, SPI, UART interfaces
 * - Multiple power options
 * 
 * This example runs on the ESP32 side and communicates with RP2040
 * 
 * Hardware Connections (ESP32 side):
 * BME68x    ESP32
 * VCC   →   3.3V
 * GND   →   GND
 * SCL   →   GPIO 22 (I2C)
 * SDA   →   GPIO 21 (I2C)
 * 
 * Inter-MCU Communication:
 * ESP32     RP2040
 * GPIO 16 → GPIO 0  (UART TX)
 * GPIO 17 → GPIO 1  (UART RX)
 * 
 * More info: 
 * - https://uelectronics.com/producto/unit-dualmcu-esp32-rp2040-tarjeta-de-desarrollo/
 * - https://uelectronics.com/producto/unit-dualmcu-one-esp32-rp2040/
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Based on original Bosch Sensortec library
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"
#include "WiFi.h"
#include "WebServer.h"
#include "ArduinoJson.h"
#include "HardwareSerial.h"

// UNIT DualMCU ESP32 pin definitions
#define I2C_SDA 21
#define I2C_SCL 22
#define STATUS_LED 2
#define INTER_MCU_TX 16
#define INTER_MCU_RX 17

// WiFi Configuration
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Web server
WebServer server(80);

// Inter-MCU communication
HardwareSerial McuSerial(2); // Use Serial2 for inter-MCU comm

// BME68x sensor
Bme68x bme;

// Data structures
struct SensorReading {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    uint32_t timestamp;
    bool valid;
};

struct ProcessedData {
    SensorReading raw;
    float iaq;
    String iaq_level;
    float dewPoint;
    float absoluteHumidity;
    float heatIndex;
    bool calibrated;
};

ProcessedData current_data;
float gas_baseline = 0;
int calibration_count = 0;

/**
 * @brief Calculate Dew Point
 */
float calculateDewPoint(float temp, float humidity) {
    float a = 17.27;
    float b = 237.7;
    float alpha = ((a * temp) / (b + temp)) + log(humidity / 100.0);
    return (b * alpha) / (a - alpha);
}

/**
 * @brief Calculate Absolute Humidity
 */
float calculateAbsoluteHumidity(float temp, float humidity) {
    float absHum = (6.112 * exp((17.67 * temp) / (temp + 243.5)) * humidity * 2.1674) / (273.15 + temp);
    return absHum;
}

/**
 * @brief Calculate Heat Index
 */
float calculateHeatIndex(float temp, float humidity) {
    if (temp < 27.0) return temp; // Heat index not applicable below 27°C
    
    float hi = -8.784695 + 1.61139411 * temp + 2.338549 * humidity;
    hi += -0.14611605 * temp * humidity - 0.012308094 * temp * temp;
    hi += -0.016424828 * humidity * humidity + 0.002211732 * temp * temp * humidity;
    hi += 0.00072546 * temp * humidity * humidity - 0.000003582 * temp * temp * humidity * humidity;
    
    return hi;
}

/**
 * @brief Calculate IAQ
 */
float calculateIAQ(uint32_t gas_resistance, float humidity) {
    if (gas_baseline == 0) return 50;
    
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
 * @brief Get IAQ level description
 */
String getIAQLevel(float iaq) {
    if (gas_baseline == 0) return "Calibrating";
    if (iaq <= 50) return "Excellent";
    if (iaq <= 100) return "Good";
    if (iaq <= 150) return "Moderate";
    if (iaq <= 200) return "Poor";
    return "Unhealthy";
}

/**
 * @brief Send command to RP2040
 */
void sendCommandToRP2040(String command) {
    McuSerial.println(command);
}

/**
 * @brief Receive data from RP2040
 */
String receiveFromRP2040() {
    String received = "";
    unsigned long timeout = millis() + 1000; // 1 second timeout
    
    while (millis() < timeout) {
        if (McuSerial.available()) {
            received = McuSerial.readStringUntil('\n');
            break;
        }
        delay(10);
    }
    
    return received;
}

/**
 * @brief Initialize WiFi
 */
void initWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi");
    }
}

/**
 * @brief Setup web server
 */
void setupWebServer() {
    server.on("/", []() {
        String html = "<!DOCTYPE html><html><head><title>UNIT DualMCU Air Monitor</title>";
        html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}";
        html += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
        html += ".mcu-section{border:2px solid #007bff;margin:10px;padding:15px;border-radius:8px}";
        html += ".esp32{border-color:#ff6b35}.rp2040{border-color:#28a745}";
        html += ".data-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin:10px 0}";
        html += ".data-item{background:#f8f9fa;padding:10px;border-radius:5px;text-align:center}";
        html += ".value{font-size:1.5em;font-weight:bold;color:#007bff}";
        html += ".label{color:#666;font-size:0.9em}";
        html += ".status{padding:10px;border-radius:5px;text-align:center;margin:10px 0}";
        html += ".good{background:#d4edda;color:#155724}.moderate{background:#fff3cd;color:#856404}.poor{background:#f8d7da;color:#721c24}";
        html += "</style></head><body>";
        html += "<div class='container'>";
        html += "<h1>🤖 UNIT DualMCU Air Quality Monitor</h1>";
        html += "<p><strong>ESP32 + RP2040 Dual Processing System</strong></p>";
        
        html += "<div class='mcu-section esp32'>";
        html += "<h3>📡 ESP32 (WiFi & Processing)</h3>";
        html += "<div class='data-grid'>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.iaq, 1) + "</div><div class='label'>Air Quality Index</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.dewPoint, 1) + "°C</div><div class='label'>Dew Point</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.absoluteHumidity, 1) + " g/m³</div><div class='label'>Absolute Humidity</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.heatIndex, 1) + "°C</div><div class='label'>Heat Index</div></div>";
        html += "</div></div>";
        
        html += "<div class='mcu-section rp2040'>";
        html += "<h3>🎯 RP2040 (Sensor Reading)</h3>";
        html += "<div class='data-grid'>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.raw.temperature, 1) + "°C</div><div class='label'>Temperature</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.raw.humidity, 1) + "%</div><div class='label'>Humidity</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.raw.pressure, 1) + " hPa</div><div class='label'>Pressure</div></div>";
        html += "<div class='data-item'><div class='value'>" + String(current_data.raw.gas_resistance) + " Ω</div><div class='label'>Gas Resistance</div></div>";
        html += "</div></div>";
        
        String statusClass = "good";
        if (current_data.iaq > 100) statusClass = "moderate";
        if (current_data.iaq > 200) statusClass = "poor";
        
        html += "<div class='status " + statusClass + "'>Air Quality: " + current_data.iaq_level + "</div>";
        html += "<p><em>Auto-refresh every 10 seconds</em></p>";
        html += "</div>";
        html += "<script>setTimeout(function(){location.reload()},10000)</script>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
    });
    
    server.on("/api/data", []() {
        DynamicJsonDocument doc(1024);
        doc["board"] = "UNIT_DualMCU";
        doc["esp32"]["iaq"] = current_data.iaq;
        doc["esp32"]["iaq_level"] = current_data.iaq_level;
        doc["esp32"]["dew_point"] = current_data.dewPoint;
        doc["esp32"]["absolute_humidity"] = current_data.absoluteHumidity;
        doc["esp32"]["heat_index"] = current_data.heatIndex;
        doc["esp32"]["calibrated"] = current_data.calibrated;
        
        doc["rp2040"]["temperature"] = current_data.raw.temperature;
        doc["rp2040"]["humidity"] = current_data.raw.humidity;
        doc["rp2040"]["pressure"] = current_data.raw.pressure;
        doc["rp2040"]["gas_resistance"] = current_data.raw.gas_resistance;
        doc["rp2040"]["timestamp"] = current_data.raw.timestamp;
        doc["rp2040"]["valid"] = current_data.raw.valid;
        
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    server.begin();
}

/**
 * @brief Read sensor data (ESP32 handles this, can also delegate to RP2040)
 */
bool readSensorData() {
    bme68xData data;
    
    bme.setOpMode(BME68X_FORCED_MODE);
    uint16_t delayPeriod = bme.getMeasDur(BME68X_FORCED_MODE);
    delay(delayPeriod);
    
    if (bme.fetchData()) {
        uint8_t nFieldsLeft = 0;
        
        do {
            nFieldsLeft = bme.getData(data);
            
            if (data.status & BME68X_NEW_DATA_MSK) {
                // Update raw sensor data
                current_data.raw.temperature = data.temperature;
                current_data.raw.humidity = data.humidity;
                current_data.raw.pressure = data.pressure / 100.0;
                current_data.raw.gas_resistance = data.gas_resistance;
                current_data.raw.timestamp = millis();
                current_data.raw.valid = (data.status & BME68X_GASM_VALID_MSK) ? true : false;
                
                // Calibrate gas baseline
                if (calibration_count < 100) {
                    if (calibration_count == 0) {
                        gas_baseline = data.gas_resistance;
                    } else {
                        gas_baseline = (gas_baseline + data.gas_resistance) / 2.0;
                    }
                    calibration_count++;
                    current_data.calibrated = false;
                } else {
                    current_data.calibrated = true;
                }
                
                // Calculate derived values (ESP32 processing power)
                current_data.iaq = calculateIAQ(data.gas_resistance, data.humidity);
                current_data.iaq_level = getIAQLevel(current_data.iaq);
                current_data.dewPoint = calculateDewPoint(data.temperature, data.humidity);
                current_data.absoluteHumidity = calculateAbsoluteHumidity(data.temperature, data.humidity);
                current_data.heatIndex = calculateHeatIndex(data.temperature, data.humidity);
                
                return true;
            }
        } while (nFieldsLeft);
    }
    
    return false;
}

/**
 * @brief Setup function
 */
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);
    
    Serial.println("==================================================");
    Serial.println("BME68x Dual-Core Monitor - UNIT DualMCU ESP32+RP2040");
    Serial.println("Distributed Processing Air Quality Monitor");
    Serial.println("==================================================");
    
    // Initialize status LED
    pinMode(STATUS_LED, OUTPUT);
    
    // Initialize inter-MCU communication
    McuSerial.begin(115200, SERIAL_8N1, INTER_MCU_RX, INTER_MCU_TX);
    Serial.println("Inter-MCU communication initialized");
    
    // Send initialization command to RP2040
    sendCommandToRP2040("INIT_BME68X");
    String rp2040_response = receiveFromRP2040();
    Serial.println("RP2040 Response: " + rp2040_response);
    
    // Initialize I2C and sensor on ESP32 side
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    
    Serial.println("Initializing BME68x on ESP32...");
    
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("WARNING: BME68x not found on ESP32, will rely on RP2040");
    } else {
        bme.setTPH();
        bme.setHeaterProf(320, 150);
        Serial.println("BME68x initialized on ESP32");
    }
    
    // Initialize WiFi
    if (strlen(ssid) > 0) {
        initWiFi();
        
        if (WiFi.status() == WL_CONNECTED) {
            setupWebServer();
            Serial.println("Web server: http://" + WiFi.localIP().toString());
        }
    }
    
    Serial.println("System ready - ESP32 handling WiFi & processing, RP2040 handling sensors");
    Serial.println("Calibrating...");
    
    digitalWrite(STATUS_LED, HIGH);
}

/**
 * @brief Main loop
 */
void loop() {
    static unsigned long lastReading = 0;
    
    if (millis() - lastReading >= 5000) { // Read every 5 seconds
        lastReading = millis();
        
        // Try to read from local sensor first
        bool localSuccess = readSensorData();
        
        if (!localSuccess) {
            // If local sensor fails, request data from RP2040
            sendCommandToRP2040("READ_BME68X");
            String rp2040_data = receiveFromRP2040();
            
            if (rp2040_data.length() > 0) {
                // Parse JSON response from RP2040
                DynamicJsonDocument doc(512);
                if (deserializeJson(doc, rp2040_data) == DeserializationError::Ok) {
                    current_data.raw.temperature = doc["temperature"];
                    current_data.raw.humidity = doc["humidity"];
                    current_data.raw.pressure = doc["pressure"];
                    current_data.raw.gas_resistance = doc["gas_resistance"];
                    current_data.raw.valid = doc["valid"];
                    current_data.raw.timestamp = millis();
                    
                    // Process data on ESP32
                    if (calibration_count < 100) {
                        if (calibration_count == 0) {
                            gas_baseline = current_data.raw.gas_resistance;
                        } else {
                            gas_baseline = (gas_baseline + current_data.raw.gas_resistance) / 2.0;
                        }
                        calibration_count++;
                        current_data.calibrated = false;
                    } else {
                        current_data.calibrated = true;
                    }
                    
                    current_data.iaq = calculateIAQ(current_data.raw.gas_resistance, current_data.raw.humidity);
                    current_data.iaq_level = getIAQLevel(current_data.iaq);
                    current_data.dewPoint = calculateDewPoint(current_data.raw.temperature, current_data.raw.humidity);
                    current_data.absoluteHumidity = calculateAbsoluteHumidity(current_data.raw.temperature, current_data.raw.humidity);
                    current_data.heatIndex = calculateHeatIndex(current_data.raw.temperature, current_data.raw.humidity);
                    
                    localSuccess = true;
                }
            }
        }
        
        if (localSuccess) {
            // Print comprehensive data
            DynamicJsonDocument output(1024);
            output["mcu"] = "ESP32";
            output["timestamp"] = current_data.raw.timestamp;
            output["sensor"]["temperature"] = current_data.raw.temperature;
            output["sensor"]["humidity"] = current_data.raw.humidity;
            output["sensor"]["pressure"] = current_data.raw.pressure;
            output["sensor"]["gas_resistance"] = current_data.raw.gas_resistance;
            output["processed"]["iaq"] = current_data.iaq;
            output["processed"]["iaq_level"] = current_data.iaq_level;
            output["processed"]["dew_point"] = current_data.dewPoint;
            output["processed"]["absolute_humidity"] = current_data.absoluteHumidity;
            output["processed"]["heat_index"] = current_data.heatIndex;
            output["calibrated"] = current_data.calibrated;
            
            if (!current_data.calibrated) {
                output["calibration_progress"] = (calibration_count * 100) / 100;
            }
            
            String jsonOutput;
            serializeJson(output, jsonOutput);
            Serial.println(jsonOutput);
            
            // Send processed data back to RP2040 if needed
            sendCommandToRP2040("PROCESSED_DATA:" + jsonOutput);
            
            // Update LED based on air quality
            if (current_data.calibrated) {
                if (current_data.iaq <= 100) {
                    digitalWrite(STATUS_LED, HIGH);
                } else {
                    static unsigned long lastBlink = 0;
                    if (millis() - lastBlink >= 500) {
                        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
                        lastBlink = millis();
                    }
                }
            }
        } else {
            Serial.println("{\"error\":\"Failed to read sensor data from both MCUs\"}");
            digitalWrite(STATUS_LED, LOW);
        }
    }
    
    // Handle web server
    server.handleClient();
    
    // Check for commands from RP2040
    if (McuSerial.available()) {
        String command = McuSerial.readStringUntil('\n');
        Serial.println("Command from RP2040: " + command);
        
        // Process inter-MCU commands here
        if (command.startsWith("STATUS")) {
            McuSerial.println("ESP32_OK");
        }
    }
    
    delay(10);
}
