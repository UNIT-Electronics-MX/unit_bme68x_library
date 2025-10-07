/**
 * BME68x Basic Example for UNIT Electronics ESP32 DevKit
 * 
 * Este ejemplo muestra cómo usar el sensor BME680/BME688 con las tarjetas
 * de desarrollo ESP32 de UNIT Electronics.
 * 
 * Conexiones recomendadas:
 * BME68x    ESP32 (UNIT Electronics)
 * VCC   →   3.3V
 * GND   →   GND
 * SCL   →   GPIO 22 (I2C)
 * SDA   →   GPIO 21 (I2C)
 * 
 * Para conexión SPI (opcional):
 * SCK   →   GPIO 18
 * MISO  →   GPIO 19
 * MOSI  →   GPIO 23
 * CS    →   GPIO 5
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Basado en la librería original de Bosch Sensortec
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"
#include "Wire.h"

// Definir pines I2C para UNIT Electronics ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// Crear instancia del sensor
Bme68x bme;

/**
 * @brief Inicializa el sensor y la configuración de hardware
 */
void setup(void)
{
    // Inicializar comunicación serie
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("=== BME68x Sensor - UNIT Electronics ESP32 ===");
    Serial.println("Inicializando sensor...");
    
    // Inicializar I2C con pines específicos
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Inicializar sensor usando I2C
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("ERROR: No se pudo inicializar el sensor BME68x");
        Serial.println("Verifica las conexiones y reinicia");
        while (1) {
            delay(1000);
        }
    }
    
    // Verificar status del sensor
    if (bme.checkStatus()) {
        if (bme.checkStatus() == BME68X_ERROR) {
            Serial.println("Error del sensor: " + bme.statusString());
            return;
        }
        else if (bme.checkStatus() == BME68X_WARNING) {
            Serial.println("Advertencia del sensor: " + bme.statusString());
        }
    }
    
    Serial.println("Sensor inicializado correctamente!");
    
    // Configurar parámetros TPH (Temperature, Pressure, Humidity)
    bme.setTPH();
    
    // Configurar perfil del calentador: 300°C por 100ms para modo forzado
    bme.setHeaterProf(300, 100);
    
    Serial.println("\nFormato de salida:");
    Serial.println("Tiempo(ms), Temp(°C), Presión(hPa), Humedad(%), Gas(Ohms), Status");
    Serial.println("========================================================================");
}

/**
 * @brief Bucle principal - lee datos del sensor cada segundo
 */
void loop(void)
{
    bme68xData data;
    uint8_t nFieldsLeft = 0;
    
    // Configurar modo forzado y obtener duración de medición
    bme.setOpMode(BME68X_FORCED_MODE);
    uint16_t delayPeriod = bme.getMeasDur(BME68X_FORCED_MODE);
    
    // Esperar el tiempo necesario para la medición
    delay(delayPeriod);
    
    // Obtener datos del sensor
    if (bme.fetchData()) {
        do {
            nFieldsLeft = bme.getData(data);
            
            if (data.status & BME68X_NEW_DATA_MSK) {
                // Mostrar timestamp
                Serial.print(String(millis()));
                Serial.print(", ");
                
                // Mostrar temperatura
                Serial.print(String(data.temperature, 2));
                Serial.print(", ");
                
                // Mostrar presión en hPa
                Serial.print(String(data.pressure / 100.0, 2));
                Serial.print(", ");
                
                // Mostrar humedad
                Serial.print(String(data.humidity, 2));
                Serial.print(", ");
                
                // Mostrar resistencia del gas
                Serial.print(String(data.gas_resistance));
                Serial.print(", ");
                
                // Mostrar status en hexadecimal
                Serial.print("0x");
                Serial.print(data.status, HEX);
                
                // Información adicional sobre la calidad del gas
                if (data.status & BME68X_GASM_VALID_MSK) {
                    Serial.print(" (Gas válido)");
                } else {
                    Serial.print(" (Gas calentando)");
                }
                
                Serial.println();
            }
        } while (nFieldsLeft);
    }
    
    // Esperar 1 segundo antes de la siguiente lectura
    delay(1000);
}
