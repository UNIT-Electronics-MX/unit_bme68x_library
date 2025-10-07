/**
 * BME68x Advanced Example for UNIT Electronics ESP32-S3 DevKit
 * 
 * Ejemplo avanzado que incluye:
 * - Modo secuencial con múltiples perfiles de calentador
 * - Cálculo del índice de calidad del aire (IAQ)
 * - Salida formateada JSON
 * - Calibración automática
 * - WiFi opcional para IoT
 * 
 * Conexiones recomendadas:
 * BME68x    ESP32-S3 (UNIT Electronics)
 * VCC   →   3.3V
 * GND   →   GND
 * SCL   →   GPIO 8 (I2C)
 * SDA   →   GPIO 9 (I2C)
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Basado en la librería original de Bosch Sensortec
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"
#include <ArduinoJson.h>

// Definir pines I2C para UNIT Electronics ESP32-S3
#define I2C_SDA 9
#define I2C_SCL 8

// LED indicador (ESP32-S3 built-in)
#define STATUS_LED 2

// Configuración WiFi (opcional)
#define ENABLE_WIFI false
#define WIFI_SSID "TU_WIFI"
#define WIFI_PASSWORD "TU_PASSWORD"

// Crear instancia del sensor
Bme68x bme;

// Variables para calibración IAQ
float gas_baseline = 0;
float hum_baseline = 40.0; // Humedad base en %
int getgasreference_count = 0;
boolean calibrated = false;

// Perfiles de calentador para modo secuencial
uint16_t tempProf[10] = {200, 240, 280, 320, 360, 360, 320, 280, 240, 200};
uint16_t durProf[10]  = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

/**
 * @brief Calcula el índice de calidad del aire
 */
float calculateIAQ(float gas_resistance, float humidity) {
    float gas_score = 0;
    float hum_score = 0;
    
    // Calcular score del gas (0-100)
    if (calibrated && gas_baseline > 0) {
        gas_score = (gas_resistance / gas_baseline) * 100.0;
        if (gas_score > 100) gas_score = 100;
        if (gas_score < 0) gas_score = 0;
    } else {
        gas_score = 50; // Valor neutral durante calibración
    }
    
    // Calcular score de humedad (0-100)
    if (humidity >= 38 && humidity <= 42) {
        hum_score = 100; // Humedad óptima
    } else {
        if (humidity < 38) {
            hum_score = 100 - (38 - humidity) * 5;
        } else {
            hum_score = 100 - (humidity - 42) * 5;
        }
        if (hum_score < 0) hum_score = 0;
        if (hum_score > 100) hum_score = 100;
    }
    
    // IAQ combinado (menor es mejor)
    float iaq = (100 - gas_score) * 0.7 + (100 - hum_score) * 0.3;
    return iaq;
}

/**
 * @brief Establece la línea base del gas para calibración
 */
void calibrateGasBaseline(float gas_resistance) {
    if (getgasreference_count <= 500) {
        if (gas_resistance > 0) {
            if (getgasreference_count == 0) {
                gas_baseline = gas_resistance;
            } else {
                gas_baseline = (gas_baseline + gas_resistance) / 2.0;
            }
        }
        getgasreference_count++;
    } else {
        calibrated = true;
    }
}

/**
 * @brief Obtiene la calificación textual del IAQ
 */
String getIAQLevel(float iaq) {
    if (iaq <= 50) return "Excelente";
    else if (iaq <= 100) return "Bueno";
    else if (iaq <= 150) return "Ligeramente contaminado";
    else if (iaq <= 200) return "Moderadamente contaminado";
    else if (iaq <= 300) return "Muy contaminado";
    else return "Severamente contaminado";
}

/**
 * @brief Parpadea el LED indicador
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
 * @brief Inicialización del sistema
 */
void setup(void)
{
    // Configurar LED de status
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    
    // Inicializar comunicación serie
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("====================================================");
    Serial.println("BME68x Advanced Sensor - UNIT Electronics ESP32-S3");
    Serial.println("====================================================");
    
    // Indicar inicio con LED
    blinkLED(3, 200);
    
    // Inicializar I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); // I2C a 400kHz para mejor rendimiento
    
    Serial.println("Inicializando sensor BME68x...");
    
    // Inicializar sensor
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("ERROR: No se pudo inicializar el sensor BME68x");
        while (1) {
            blinkLED(5, 100); // Error signal
            delay(2000);
        }
    }
    
    // Verificar status
    if (bme.checkStatus()) {
        if (bme.checkStatus() == BME68X_ERROR) {
            Serial.println("Error del sensor: " + bme.statusString());
            return;
        }
        else if (bme.checkStatus() == BME68X_WARNING) {
            Serial.println("Advertencia: " + bme.statusString());
        }
    }
    
    Serial.println("Sensor inicializado correctamente!");
    
    // Configurar TPH
    bme.setTPH();
    
    // Configurar perfiles de calentador para modo secuencial
    bme.setSeqSleep(BME68X_ODR_0_59_MS);
    bme.setHeaterProf(tempProf, durProf, 10);
    
    Serial.println("Configuración completada!");
    Serial.println("Iniciando calibración (primeras 500 lecturas)...");
    Serial.println();
    
    // Indicar listo
    digitalWrite(STATUS_LED, HIGH);
}

/**
 * @brief Bucle principal
 */
void loop(void)
{
    bme68xData data;
    uint8_t nFieldsLeft = 0;
    static unsigned long lastReading = 0;
    
    // Leer cada 3 segundos
    if (millis() - lastReading >= 3000) {
        lastReading = millis();
        
        // Configurar modo secuencial
        bme.setOpMode(BME68X_SEQUENTIAL_MODE);
        
        // Obtener duración de medición
        uint16_t delayPeriod = bme.getMeasDur(BME68X_SEQUENTIAL_MODE);
        delay(delayPeriod);
        
        // Leer datos
        if (bme.fetchData()) {
            do {
                nFieldsLeft = bme.getData(data);
                
                if (data.status & BME68X_NEW_DATA_MSK) {
                    // Calibrar línea base del gas
                    if (!calibrated) {
                        calibrateGasBaseline(data.gas_resistance);
                        Serial.printf("Calibrando... %d/500 (%.1f%%)\n", 
                                    getgasreference_count, 
                                    (getgasreference_count / 500.0) * 100);
                        continue;
                    }
                    
                    // Calcular IAQ
                    float iaq = calculateIAQ(data.gas_resistance, data.humidity);
                    
                    // Crear JSON con los datos
                    DynamicJsonDocument doc(1024);
                    doc["timestamp"] = millis();
                    doc["temperature"] = round(data.temperature * 100) / 100.0;
                    doc["pressure"] = round(data.pressure / 100.0 * 100) / 100.0;
                    doc["humidity"] = round(data.humidity * 100) / 100.0;
                    doc["gas_resistance"] = data.gas_resistance;
                    doc["gas_baseline"] = gas_baseline;
                    doc["iaq"] = round(iaq * 100) / 100.0;
                    doc["iaq_level"] = getIAQLevel(iaq);
                    doc["gas_valid"] = (data.status & BME68X_GASM_VALID_MSK) ? true : false;
                    doc["heat_stable"] = (data.status & BME68X_HEAT_STAB_MSK) ? true : false;
                    doc["sensor_id"] = data.gas_index;
                    
                    // Mostrar JSON
                    String jsonString;
                    serializeJson(doc, jsonString);
                    Serial.println(jsonString);
                    
                    // Control del LED según calidad del aire
                    if (iaq <= 50) {
                        digitalWrite(STATUS_LED, HIGH); // Aire excelente - LED encendido
                    } else if (iaq <= 100) {
                        // Aire bueno - parpadeo lento
                        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
                    } else {
                        // Aire contaminado - parpadeo rápido
                        static unsigned long lastBlink = 0;
                        if (millis() - lastBlink >= 200) {
                            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
                            lastBlink = millis();
                        }
                    }
                }
            } while (nFieldsLeft);
        }
    }
    
    // Pequeña pausa para no saturar el sistema
    delay(10);
}
