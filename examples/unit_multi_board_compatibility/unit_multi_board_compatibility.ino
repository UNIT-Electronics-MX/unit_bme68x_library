/**
 * BME68x Multi-Board Compatibility Example
 * 
 * Este ejemplo funciona automáticamente en diferentes tarjetas:
 * - ESP32 DevKit (UNIT Electronics)
 * - ESP32-S3 DevKit (UNIT Electronics)
 * - ESP32-C3 DevKit (UNIT Electronics)
 * - Arduino Uno/Nano/Mega
 * - Otras tarjetas compatibles
 * 
 * El código detecta automáticamente el tipo de tarjeta y configura
 * los pines correspondientes.
 * 
 * Copyright (C) 2025 UNIT Electronics
 * Basado en la librería original de Bosch Sensortec
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Arduino.h"
#include "bme68xLibrary.h"

// Auto-detection of UNIT Electronics boards and pin configuration
#if defined(ESP32)
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    // UNIT TouchDot S3 (ESP32-S3)
    #define BOARD_NAME "UNIT TouchDot S3"
    #define I2C_SDA 9
    #define I2C_SCL 8
    #define STATUS_LED 25  // NeoPixel LED
    #define HAS_WIFI true
    #define HAS_NEOPIXEL true
    #define QWIIC_CONNECTOR true
    #include "Wire.h"
  #elif defined(CONFIG_IDF_TARGET_ESP32C6)
    // UNIT Pulsar ESP32-C6
    #define BOARD_NAME "UNIT Pulsar ESP32-C6"
    #define I2C_SDA 6
    #define I2C_SCL 7
    #define STATUS_LED 15
    #define HAS_WIFI true
    #define HAS_MATTER true
    #define QWIIC_CONNECTOR true
    #include "Wire.h"
  #elif defined(CONFIG_IDF_TARGET_ESP32C3)
    // ESP32-C3 generic
    #define BOARD_NAME "ESP32-C3"
    #define I2C_SDA 4
    #define I2C_SCL 5
    #define STATUS_LED 8
    #define HAS_WIFI true
    #include "Wire.h"
  #else
    // UNIT DualMCU or generic ESP32
    #define BOARD_NAME "UNIT ESP32 DualMCU"
    #define I2C_SDA 21
    #define I2C_SCL 22
    #define STATUS_LED 2
    #define HAS_WIFI true
    #define HAS_DUAL_MCU true
    #include "Wire.h"
  #endif
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI)
  // Arduino Uno/Nano/Mini
  #define BOARD_NAME "Arduino Uno/Nano/Mini"
  #define I2C_SDA A4
  #define I2C_SCL A5
  #define STATUS_LED 13
  #define HAS_WIFI false
  #include "Wire.h"
#elif defined(ARDUINO_AVR_MEGA2560)
  // Arduino Mega
  #define BOARD_NAME "Arduino Mega 2560"
  #define I2C_SDA 20
  #define I2C_SCL 21
  #define STATUS_LED 13
  #define HAS_WIFI false
  #include "Wire.h"
#else
  // Tarjeta genérica - usar pines por defecto
  #define BOARD_NAME "Tarjeta Genérica"
  #define STATUS_LED LED_BUILTIN
  #define HAS_WIFI false
  #include "Wire.h"
  // No definir pines específicos, usar Wire.begin() por defecto
#endif

// Crear instancia del sensor
Bme68x bme;

// Variables de configuración según la tarjeta
struct BoardConfig {
    String name;
    bool hasCustomPins;
    bool hasWiFi;
    bool hasLowPower;
    uint32_t serialSpeed;
    uint32_t i2cSpeed;
} boardConfig;

/**
 * @brief Configura parámetros específicos de la tarjeta
 */
void setupBoardConfig() {
    boardConfig.name = BOARD_NAME;
    boardConfig.hasWiFi = HAS_WIFI;
    
#if defined(ESP32)
    boardConfig.hasCustomPins = true;
    boardConfig.hasLowPower = true;
    boardConfig.serialSpeed = 115200;
    boardConfig.i2cSpeed = 400000; // ESP32 puede manejar I2C más rápido
#else
    #if defined(I2C_SDA) && defined(I2C_SCL)
        boardConfig.hasCustomPins = true;
    #else
        boardConfig.hasCustomPins = false;
    #endif
    boardConfig.hasLowPower = false;
    boardConfig.serialSpeed = 9600; // Más conservador para Arduino
    boardConfig.i2cSpeed = 100000; // I2C estándar para Arduino
#endif
}

/**
 * @brief Inicializa I2C según la tarjeta
 */
void setupI2C() {
#if defined(ESP32) && defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(boardConfig.i2cSpeed);
    Serial.printf("I2C inicializado - SDA: %d, SCL: %d, Velocidad: %d Hz\n", 
                  I2C_SDA, I2C_SCL, boardConfig.i2cSpeed);
#elif defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin();
    Wire.setClock(boardConfig.i2cSpeed);
    Serial.print("I2C inicializado - SDA: ");
    Serial.print(I2C_SDA);
    Serial.print(", SCL: ");
    Serial.print(I2C_SCL);
    Serial.print(", Velocidad: ");
    Serial.print(boardConfig.i2cSpeed);
    Serial.println(" Hz");
#else
    Wire.begin();
    Wire.setClock(boardConfig.i2cSpeed);
    Serial.print("I2C inicializado con pines por defecto, Velocidad: ");
    Serial.print(boardConfig.i2cSpeed);
    Serial.println(" Hz");
#endif
}

/**
 * @brief Parpadea el LED de estado
 */
void blinkStatusLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(STATUS_LED, HIGH);
        delay(delayMs);
        digitalWrite(STATUS_LED, LOW);
        delay(delayMs);
    }
}

/**
 * @brief Muestra información de la tarjeta detectada
 */
void printBoardInfo() {
    Serial.println("===============================================");
    Serial.println("BME68x Multi-Board Compatibility");
    Serial.println("UNIT Electronics - Universal Version");
    Serial.println("===============================================");
    Serial.print("Board detected: ");
    Serial.println(boardConfig.name);
    Serial.print("Serial speed: ");
    Serial.print(boardConfig.serialSpeed);
    Serial.println(" bps");
    Serial.print("I2C speed: ");
    Serial.print(boardConfig.i2cSpeed);
    Serial.println(" Hz");
    Serial.print("Custom I2C pins: ");
    Serial.println(boardConfig.hasCustomPins ? "Yes" : "No (default)");
    Serial.print("WiFi available: ");
    Serial.println(boardConfig.hasWiFi ? "Yes" : "No");
    Serial.print("Low power mode: ");
    Serial.println(boardConfig.hasLowPower ? "Supported" : "Not supported");
    
    // Additional board-specific features
    #ifdef HAS_NEOPIXEL
    Serial.println("Special features: NeoPixel LED");
    #endif
    #ifdef HAS_MATTER
    Serial.println("Special features: Matter/Thread support");
    #endif
    #ifdef HAS_DUAL_MCU
    Serial.println("Special features: Dual MCU (ESP32 + RP2040)");
    #endif
    #ifdef QWIIC_CONNECTOR
    Serial.println("Hardware: QWIIC connector available");
    #endif
    
#ifdef STATUS_LED
    Serial.print("LED de estado: Pin ");
    Serial.println(STATUS_LED);
#endif
    
    Serial.println("===============================================");
}

/**
 * @brief Obtiene configuración optimizada del sensor según la tarjeta
 */
void configureSensorForBoard() {
    // Configuración TPH estándar
    bme.setTPH();
    
    // Configuración del calentador según capacidades de la tarjeta
    if (boardConfig.hasLowPower) {
        // Para tarjetas con capacidades de bajo consumo, usar perfil eficiente
        bme.setHeaterProf(280, 90);
        Serial.println("Configuración: Perfil de bajo consumo (280°C x 90ms)");
    } else {
        // Para Arduino, usar perfil estándar
        bme.setHeaterProf(320, 150);
        Serial.println("Configuración: Perfil estándar (320°C x 150ms)");
    }
}

/**
 * @brief Formatea y muestra datos según capacidades de la tarjeta
 */
void displaySensorData(bme68xData &data) {
    if (boardConfig.hasWiFi) {
        // Para tarjetas con WiFi, mostrar formato JSON para IoT
        Serial.print("{");
        Serial.print("\"timestamp\":");
        Serial.print(millis());
        Serial.print(",\"board\":\"");
        Serial.print(boardConfig.name);
        Serial.print("\",\"temperature\":");
        Serial.print(data.temperature, 2);
        Serial.print(",\"humidity\":");
        Serial.print(data.humidity, 2);
        Serial.print(",\"pressure\":");
        Serial.print(data.pressure / 100.0, 2);
        Serial.print(",\"gas_resistance\":");
        Serial.print(data.gas_resistance);
        Serial.print(",\"gas_valid\":");
        Serial.print((data.status & BME68X_GASM_VALID_MSK) ? "true" : "false");
        Serial.println("}");
    } else {
        // Para Arduino, formato legible simple
        Serial.print("Temp: ");
        Serial.print(data.temperature, 1);
        Serial.print("°C | Hum: ");
        Serial.print(data.humidity, 1);
        Serial.print("% | Press: ");
        Serial.print(data.pressure / 100.0, 1);
        Serial.print(" hPa | Gas: ");
        Serial.print(data.gas_resistance);
        Serial.print(" Ohms");
        if (data.status & BME68X_GASM_VALID_MSK) {
            Serial.println(" [Gas OK]");
        } else {
            Serial.println(" [Calentando]");
        }
    }
}

/**
 * @brief Configuración inicial
 */
void setup() {
    // Configurar parámetros de la tarjeta
    setupBoardConfig();
    
    // Configurar LED de estado si está disponible
#ifdef STATUS_LED
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
#endif
    
    // Inicializar comunicación serie
    Serial.begin(boardConfig.serialSpeed);
    
    // Para ESP32, esperar conexión serie
#if defined(ESP32)
    while (!Serial && millis() < 5000) {
        delay(10);
    }
#else
    // Para Arduino, pausa breve
    delay(2000);
#endif
    
    // Mostrar información de la tarjeta
    printBoardInfo();
    
    // Señal de inicio
    blinkStatusLED(3, 200);
    
    // Inicializar I2C
    setupI2C();
    
    Serial.println("Inicializando sensor BME68x...");
    
    // Intentar inicializar sensor con diferentes direcciones I2C
    bool sensorFound = false;
    uint8_t addresses[] = {BME68X_I2C_ADDR_LOW, BME68X_I2C_ADDR_HIGH};
    
    for (int i = 0; i < 2 && !sensorFound; i++) {
        Serial.print("Probando dirección I2C: 0x");
        Serial.println(addresses[i], HEX);
        
        if (bme.begin(addresses[i], Wire)) {
            sensorFound = true;
            Serial.print("Sensor encontrado en dirección: 0x");
            Serial.println(addresses[i], HEX);
        }
    }
    
    if (!sensorFound) {
        Serial.println("ERROR: Sensor BME68x no encontrado");
        Serial.println("Verifica las conexiones:");
        if (boardConfig.hasCustomPins) {
            Serial.print("  SDA -> Pin ");
            Serial.println(I2C_SDA);
            Serial.print("  SCL -> Pin ");
            Serial.println(I2C_SCL);
        } else {
            Serial.println("  Usa pines I2C por defecto de la tarjeta");
        }
        Serial.println("  VCC -> 3.3V");
        Serial.println("  GND -> GND");
        
        while (1) {
            blinkStatusLED(5, 100);
            delay(2000);
        }
    }
    
    // Verificar estado del sensor
    if (bme.checkStatus()) {
        if (bme.checkStatus() == BME68X_ERROR) {
            Serial.println("Error del sensor: " + bme.statusString());
        } else if (bme.checkStatus() == BME68X_WARNING) {
            Serial.println("Advertencia: " + bme.statusString());
        }
    }
    
    // Configurar sensor según la tarjeta
    configureSensorForBoard();
    
    Serial.println("Configuración completada!");
    Serial.println();
    
    // Encender LED para indicar funcionamiento
#ifdef STATUS_LED
    digitalWrite(STATUS_LED, HIGH);
#endif
}

/**
 * @brief Bucle principal
 */
void loop() {
    bme68xData data;
    uint8_t nFieldsLeft = 0;
    
    // Configurar modo forzado
    bme.setOpMode(BME68X_FORCED_MODE);
    
    // Obtener tiempo de medición y esperar
    uint16_t delayPeriod = bme.getMeasDur(BME68X_FORCED_MODE);
    delay(delayPeriod);
    
    // Leer datos
    if (bme.fetchData()) {
        do {
            nFieldsLeft = bme.getData(data);
            
            if (data.status & BME68X_NEW_DATA_MSK) {
                displaySensorData(data);
                
                // Parpadeo del LED para indicar lectura exitosa
#ifdef STATUS_LED
                digitalWrite(STATUS_LED, LOW);
                delay(50);
                digitalWrite(STATUS_LED, HIGH);
#endif
            }
        } while (nFieldsLeft);
    }
    
    // Intervalo entre lecturas según capacidades de la tarjeta
    if (boardConfig.hasLowPower) {
        delay(2000); // ESP32 - lecturas más frecuentes
    } else {
        delay(5000); // Arduino - intervalos más largos para no saturar
    }
}
