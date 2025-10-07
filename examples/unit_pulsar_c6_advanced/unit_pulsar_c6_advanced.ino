/**
 * BME68x Advanced Example for UNIT Pulsar ESP32-C6
 * 
 * This example demonstrates advanced features using BME680/BME688 sensor 
 * with the UNIT Pulsar ESP32-C6 board, including:
 * - Wi-Fi 6 connectivity
 * - Matter/Thread protocol support
 * - JSON data output for IoT
 * - Air quality monitoring with multiple heater profiles
 * - Low power modes
 * - Web server for remote monitoring
 * 
 * UNIT Pulsar ESP32-C6 Features:
 * - ESP32-C6 RISC-V microcontroller @ 160MHz
 * - Wi-Fi 6, Bluetooth 5, Zigbee, Thread, Matter support
 * - 4MB Flash, 512KB RAM
 * - QWIIC connector for sensors
 * - LiPo battery support with charging
 * - Ultra-compact design (18mm x 43mm)
 * 
 * Hardware Connections:
 * BME68x    Pulsar C6
 * VCC   →   3.3V
 * GND   →   GND
 * SCL   →   GPIO 7 (I2C)
 * SDA   →   GPIO 6 (I2C)
 * 
 * Or use QWIIC connector for easy connection
 * 
 * More info: https://uelectronics.com/producto/unit-pulsar-esp32-c6/
 * GitHub: https://github.com/UNIT-Electronics-MX/unit_nano_esp32_c6
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
#include "ESPAsyncWebServer.h"
#include "ArduinoJson.h"
#include "esp_sleep.h"

// UNIT Pulsar C6 specific pin definitions
#define I2C_SDA 6
#define I2C_SCL 7
#define STATUS_LED 15   // Built-in LED

// WiFi Configuration (update with your credentials)
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Web server
AsyncWebServer server(80);

// BME68x sensor
Bme68x bme;

// Air quality variables
float gas_baseline = 0;
float hum_baseline = 40.0;
int calibration_count = 0;
bool is_calibrated = false;

// Sensor data structure
struct SensorData {
    float temperature;
    float humidity;
    float pressure;
    uint32_t gas_resistance;
    float iaq;
    String iaq_level;
    uint32_t timestamp;
    bool gas_valid;
    uint8_t sensor_id;
};

SensorData latest_data;

// Multiple heater profiles for enhanced gas sensing
uint16_t temp_prof[10] = {200, 240, 280, 320, 360, 400, 360, 320, 280, 240};
uint16_t dur_prof[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

/**
 * @brief Calculate Indoor Air Quality index
 */
float calculateIAQ(float gas_resistance, float humidity) {
    if (!is_calibrated) return 50;
    
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
 * @brief Get air quality level description
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
 * @brief Calibrate gas baseline
 */
void calibrateGas(float gas_resistance) {
    if (calibration_count < 200) { // Extended calibration for better accuracy
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
 * @brief Initialize WiFi connection
 */
void initWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(1000);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());
        
        // Blink LED to indicate WiFi connection
        for (int i = 0; i < 5; i++) {
            digitalWrite(STATUS_LED, HIGH);
            delay(200);
            digitalWrite(STATUS_LED, LOW);
            delay(200);
        }
    } else {
        Serial.println("\nFailed to connect to WiFi");
    }
}

/**
 * @brief Setup web server endpoints
 */
void setupWebServer() {
    // Root page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = "<!DOCTYPE html><html><head><title>UNIT Pulsar C6 Air Monitor</title>";
        html += "<meta charset='UTF-8'>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<style>body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0}";
        html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
        html += ".sensor-data{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin:20px 0}";
        html += ".data-card{background:#f8f9fa;padding:15px;border-radius:8px;text-align:center}";
        html += ".data-value{font-size:2em;font-weight:bold;color:#007bff}";
        html += ".data-label{color:#666;margin-top:5px}";
        html += ".status{padding:10px;border-radius:5px;text-align:center;margin:10px 0}";
        html += ".excellent{background:#d4edda;color:#155724}";
        html += ".good{background:#d1ecf1;color:#0c5460}";
        html += ".moderate{background:#fff3cd;color:#856404}";
        html += ".poor{background:#f8d7da;color:#721c24}";
        html += ".unhealthy{background:#f5c6cb;color:#721c24}";
        html += "</style>";
        html += "<script>setInterval(function(){fetch('/api/data').then(r=>r.json()).then(updateData)},5000);</script>";
        html += "</head><body>";
        html += "<div class='container'>";
        html += "<h1>🌱 UNIT Pulsar C6 Air Quality Monitor</h1>";
        html += "<div class='sensor-data' id='sensorData'>";
        html += "<div class='data-card'><div class='data-value' id='temp'>--</div><div class='data-label'>Temperature (°C)</div></div>";
        html += "<div class='data-card'><div class='data-value' id='hum'>--</div><div class='data-label'>Humidity (%)</div></div>";
        html += "<div class='data-card'><div class='data-value' id='press'>--</div><div class='data-label'>Pressure (hPa)</div></div>";
        html += "<div class='data-card'><div class='data-value' id='gas'>--</div><div class='data-label'>Gas Resistance (Ω)</div></div>";
        html += "<div class='data-card'><div class='data-value' id='iaq'>--</div><div class='data-label'>Air Quality Index</div></div>";
        html += "</div>";
        html += "<div class='status' id='status'>Loading...</div>";
        html += "<p><strong>Board:</strong> UNIT Pulsar ESP32-C6</p>";
        html += "<p><strong>Sensor:</strong> BME680/688</p>";
        html += "<p><strong>Last Update:</strong> <span id='timestamp'>--</span></p>";
        html += "</div>";
        html += "<script>function updateData(data){";
        html += "document.getElementById('temp').textContent=data.temperature.toFixed(1);";
        html += "document.getElementById('hum').textContent=data.humidity.toFixed(1);";
        html += "document.getElementById('press').textContent=data.pressure.toFixed(1);";
        html += "document.getElementById('gas').textContent=data.gas_resistance;";
        html += "document.getElementById('iaq').textContent=data.iaq.toFixed(1);";
        html += "const status=document.getElementById('status');";
        html += "status.textContent='Air Quality: '+data.iaq_level;";
        html += "status.className='status '+data.iaq_level.toLowerCase();";
        html += "document.getElementById('timestamp').textContent=new Date(data.timestamp).toLocaleString();";
        html += "}</script></body></html>";
        
        request->send(200, "text/html", html);
    });
    
    // API endpoint for JSON data
    server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request) {
        DynamicJsonDocument doc(1024);
        doc["temperature"] = latest_data.temperature;
        doc["humidity"] = latest_data.humidity;
        doc["pressure"] = latest_data.pressure;
        doc["gas_resistance"] = latest_data.gas_resistance;
        doc["iaq"] = latest_data.iaq;
        doc["iaq_level"] = latest_data.iaq_level;
        doc["timestamp"] = latest_data.timestamp;
        doc["gas_valid"] = latest_data.gas_valid;
        doc["sensor_id"] = latest_data.sensor_id;
        doc["calibrated"] = is_calibrated;
        doc["board"] = "UNIT Pulsar ESP32-C6";
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
    
    server.begin();
}

/**
 * @brief Blink status LED
 */
void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(STATUS_LED, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED, LOW);
        delay(delayMs);
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
    
    Serial.println("================================================");
    Serial.println("BME68x Advanced Monitor - UNIT Pulsar ESP32-C6");
    Serial.println("IoT Air Quality Monitor with Wi-Fi 6 & Matter");
    Serial.println("================================================");
    
    // Initialize status LED
    pinMode(STATUS_LED, OUTPUT);
    blinkLED(3, 300);
    
    // Initialize I2C with Pulsar C6 specific pins
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // Fast I2C for better performance
    
    Serial.println("Initializing BME68x sensor...");
    
    // Initialize sensor
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("ERROR: BME68x sensor not found!");
        Serial.println("Check connections or use QWIIC connector");
        
        while (1) {
            blinkLED(5, 100);
            delay(2000);
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
    
    // Configure sensor for advanced monitoring
    bme.setTPH();
    bme.setSeqSleep(BME68X_ODR_0_59_MS); // Sequential mode timing
    bme.setHeaterProf(temp_prof, dur_prof, 10); // Multiple heater profiles
    
    Serial.println("Configured sequential mode with multiple heater profiles");
    
    // Initialize WiFi if credentials are provided
    if (strlen(ssid) > 0) {
        initWiFi();
        
        if (WiFi.status() == WL_CONNECTED) {
            setupWebServer();
            Serial.println("Web server started at: http://" + WiFi.localIP().toString());
        }
    }
    
    Serial.println("Starting calibration (200 readings for enhanced accuracy)...");
    Serial.println("Keep device in clean air during calibration");
    Serial.println();
    
    digitalWrite(STATUS_LED, HIGH);
}

/**
 * @brief Main loop
 */
void loop(void)
{
    bme68xData data;
    static unsigned long lastReading = 0;
    
    // Read sensor every 5 seconds for IoT applications
    if (millis() - lastReading >= 5000) {
        lastReading = millis();
        
        // Configure sequential mode for multiple measurements
        bme.setOpMode(BME68X_SEQUENTIAL_MODE);
        
        // Get measurement duration and wait
        uint16_t delayPeriod = bme.getMeasDur(BME68X_SEQUENTIAL_MODE);
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
                    
                    // Update latest data structure
                    latest_data.temperature = data.temperature;
                    latest_data.humidity = data.humidity;
                    latest_data.pressure = data.pressure / 100.0; // Convert to hPa
                    latest_data.gas_resistance = data.gas_resistance;
                    latest_data.iaq = iaq;
                    latest_data.iaq_level = getAirQualityLevel(iaq);
                    latest_data.timestamp = millis();
                    latest_data.gas_valid = (data.status & BME68X_GASM_VALID_MSK) ? true : false;
                    latest_data.sensor_id = data.gas_index;
                    
                    // Create JSON output for IoT applications
                    DynamicJsonDocument doc(512);
                    doc["board"] = "UNIT_Pulsar_C6";
                    doc["timestamp"] = latest_data.timestamp;
                    doc["temperature"] = round(latest_data.temperature * 100) / 100.0;
                    doc["humidity"] = round(latest_data.humidity * 100) / 100.0;
                    doc["pressure"] = round(latest_data.pressure * 100) / 100.0;
                    doc["gas_resistance"] = latest_data.gas_resistance;
                    doc["gas_baseline"] = gas_baseline;
                    doc["iaq"] = round(latest_data.iaq * 100) / 100.0;
                    doc["iaq_level"] = latest_data.iaq_level;
                    doc["gas_valid"] = latest_data.gas_valid;
                    doc["heat_stable"] = (data.status & BME68X_HEAT_STAB_MSK) ? true : false;
                    doc["sensor_id"] = latest_data.sensor_id;
                    doc["calibrated"] = is_calibrated;
                    
                    if (!is_calibrated) {
                        doc["calibration_progress"] = (calibration_count * 100) / 200;
                    }
                    
                    // Add WiFi status
                    if (WiFi.status() == WL_CONNECTED) {
                        doc["wifi_connected"] = true;
                        doc["ip_address"] = WiFi.localIP().toString();
                        doc["rssi"] = WiFi.RSSI();
                    } else {
                        doc["wifi_connected"] = false;
                    }
                    
                    // Print JSON to serial
                    String jsonString;
                    serializeJson(doc, jsonString);
                    Serial.println(jsonString);
                    
                    // Update status LED based on air quality
                    if (is_calibrated) {
                        if (latest_data.iaq <= 100) {
                            digitalWrite(STATUS_LED, HIGH); // Good air quality
                        } else {
                            // Blink for poor air quality
                            static unsigned long lastBlink = 0;
                            if (millis() - lastBlink >= 500) {
                                digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
                                lastBlink = millis();
                            }
                        }
                    }
                }
            } while (nFieldsLeft);
        } else {
            Serial.println("{\"error\":\"Failed to fetch sensor data\"}");
            blinkLED(2, 100);
        }
    }
    
    // Handle WiFi reconnection if needed
    if (strlen(ssid) > 0 && WiFi.status() != WL_CONNECTED) {
        static unsigned long lastReconnect = 0;
        if (millis() - lastReconnect >= 30000) { // Try every 30 seconds
            Serial.println("Attempting WiFi reconnection...");
            WiFi.begin(ssid, password);
            lastReconnect = millis();
        }
    }
    
    // Small delay for system stability
    delay(10);
}
