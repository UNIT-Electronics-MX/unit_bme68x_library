# BME68x Library for UNIT Electronics

This Arduino library wraps the [BME68x Sensor API](https://github.com/BoschSensortec/BME68x-Sensor-API) to provide a simpler experience when using BME680 or BME688 sensors from Bosch Sensortec.

**This version is optimized for UNIT Electronics development boards.**

## Fork Information

This library is based on the [official Bosch-BME68x-Library fork](https://github.com/boschsensortec/Bosch-BME68x-Library) and has been adapted to improve compatibility with UNIT Electronics development boards.

## Supported Sensors

- [BME680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
- [BME688](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688/)

## Compatible Boards

### UNIT Electronics Boards
- [**UNIT TouchDot S3**](https://uelectronics.com/producto/unit-touchdot-s3/) - ESP32-S3 based, circular wearable design with NeoPixel RGB LED
- [**UNIT Pulsar ESP32-C6**](https://uelectronics.com/producto/unit-pulsar-esp32-c6/) - ESP32-C6 RISC-V based with Wi-Fi 6, Bluetooth 5, Matter, Thread support
- [**UNIT DualMCU ONE**](https://uelectronics.com/producto/unit-dualmcu-one-esp32-rp2040/) - Dual MCU ESP32 + RP2040 
- [**UNIT DualMCU**](https://uelectronics.com/producto/unit-dualmcu-esp32-rp2040-tarjeta-de-desarrollo/) - ESP32 + RP2040 Development Board

### Other Compatible Boards
- Arduino Uno/Nano/Mega
- Generic ESP32/ESP32-S3/ESP32-C3/ESP32-C6 boards
- Other Arduino-compatible boards

## Installation

### From Arduino IDE
1. Open Arduino IDE
2. Go to **Sketch** → **Include Library** → **Add .ZIP Library**
3. Select the ZIP file of this library
4. Restart Arduino IDE

### From PlatformIO
Add this line to your `platformio.ini`:
```ini
lib_deps = 
    https://github.com/UNIT-Electronics-MX/unit_bm68x_library
```

## Examples Included

### For UNIT Electronics Boards
- `unit_touchdot_s3_basic` - Basic example for UNIT TouchDot S3
- `unit_pulsar_c6_advanced` - Advanced example for UNIT Pulsar ESP32-C6 with IoT features
- `unit_dualmcu_dual_core` - Dual-core example for UNIT DualMCU boards
- `unit_low_power_optimized` - Low power consumption example for battery-powered projects

### Original Examples
- `forced_mode` - Basic forced mode
- `sequential_mode` - Sequential mode  
- `parallel_mode` - Parallel mode
- `bme688_dev_kit` - For BME688 development kit

## Hardware Connections

### For UNIT TouchDot S3 (ESP32-S3)
```
BME68x    TouchDot S3
VCC   →   3.3V
GND   →   GND
SCL   →   GPIO 8 (I2C) 
SDA   →   GPIO 9 (I2C)
```
*Note: TouchDot S3 has QWIIC connector for easy I2C connection*

### For UNIT Pulsar ESP32-C6
```
BME68x    Pulsar C6
VCC   →   3.3V
GND   →   GND
SCL   →   GPIO 7 (I2C)
SDA   →   GPIO 6 (I2C)
```
*Note: Pulsar C6 has QWIIC connector for plug-and-play sensor connection*

### For UNIT DualMCU (ESP32 + RP2040)
**ESP32 Side:**
```
BME68x    ESP32
VCC   →   3.3V
GND   →   GND
SCL   →   GPIO 22 (I2C)
SDA   →   GPIO 21 (I2C)
```

**RP2040 Side:**
```
BME68x    RP2040
VCC   →   3.3V
GND   →   GND
SCL   →   GPIO 5 (I2C)
SDA   →   GPIO 4 (I2C)
```

## Basic Usage

### For UNIT TouchDot S3
```cpp
#include "bme68xLibrary.h"
#include "Wire.h"

Bme68x bme;

void setup() {
    Serial.begin(115200);
    Wire.begin(9, 8); // SDA=9, SCL=8 for TouchDot S3
    
    // Initialize sensor (I2C)
    bme.begin(BME68X_I2C_ADDR_LOW, Wire);
    
    // Configure TPH (Temperature, Pressure, Humidity)
    bme.setTPH();
    
    // Configure heater (300°C for 100ms)
    bme.setHeaterProf(300, 100);
}

void loop() {
    bme68xData data;
    bme.setOpMode(BME68X_FORCED_MODE);
    delay(bme.getMeasDur(BME68X_FORCED_MODE));
    
    if (bme.fetchData() && bme.getData(data)) {
        Serial.println("Temp: " + String(data.temperature) + "°C");
        Serial.println("Hum: " + String(data.humidity) + "%");
        Serial.println("Press: " + String(data.pressure / 100.0) + " hPa");
        Serial.println("Gas: " + String(data.gas_resistance) + " Ohms");
    }
    
    delay(1000);
}
```

### For UNIT Pulsar ESP32-C6
```cpp
#include "bme68xLibrary.h"
#include "Wire.h"

Bme68x bme;

void setup() {
    Serial.begin(115200);
    Wire.begin(6, 7); // SDA=6, SCL=7 for Pulsar C6
    
    if (!bme.begin(BME68X_I2C_ADDR_LOW, Wire)) {
        Serial.println("BME68x sensor not found!");
        while(1);
    }
    
    bme.setTPH();
    bme.setHeaterProf(320, 150); // Optimized for C6
}

void loop() {
    bme68xData data;
    bme.setOpMode(BME68X_FORCED_MODE);
    delay(bme.getMeasDur(BME68X_FORCED_MODE));
    
    if (bme.fetchData() && bme.getData(data)) {
        // JSON output for IoT applications
        Serial.printf("{\"temp\":%.2f,\"hum\":%.2f,\"press\":%.2f,\"gas\":%u}\n",
                     data.temperature, data.humidity, 
                     data.pressure/100.0, data.gas_resistance);
    }
    
    delay(2000);
}
```

## Board Detection

The library includes automatic board detection:

```cpp
#include "bme68xLibrary.h"

void setup() {
    // Auto-detect and configure for UNIT Electronics boards
    Bme68x bme;
    
    if (bme.beginAutoDetect()) {
        Serial.println("BME68x initialized for UNIT board!");
    }
}
```

## Contributing

Found a bug or want to improve the library?

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

Distributed under the BSD-3-Clause License. See `LICENSE` for more information.

---
### Original Copyright
Copyright (c) 2021 Bosch Sensortec GmbH

### UNIT Electronics Adaptations
Copyright (c) 2025 UNIT Electronics