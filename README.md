# MS5607 SPI STM32 HAL Driver

C driver library for the MS5607-02 high-resolution barometric pressure and temperature sensor.  
Designed for STM32 microcontrollers using the STM32 HAL library and SPI interface.  
Based on the original MS5607-02 SPI library for ARM STM32F103xx (MIT License).

---

## **Features**

- Sensor reset and initialization  
- Read PROM calibration coefficients  
- Initiate uncompensated pressure (D1) and temperature (D2) conversions  
- Read 24-bit ADC results  
- Convert raw data to compensated pressure and temperature  
- SPI communication with chip select control  
- Basic HAL error handling and status reporting  

---

## **Requirements**

- STM32 MCU with STM32 HAL library  
- SPI hardware interface connected to MS5607 sensor  
- Basic experience with embedded C and STM32CubeIDE or similar  

---

**Note:**  
This driver includes `#include "stm32f4xx_hal.h"` in the header files and is configured for the STM32F4 series.  
If you intend to reuse the code for other STM32 families, please adjust the HAL include and any hardware-specific settings accordingly.

## **Quick Start**

1. Add driver files to your project

Include `MS5607SPI.c` and `MS5607SPI.h` in your STM32 project source folder.

2. Configure your hardware

Set up SPI peripheral and GPIO pins for SPI and Chip Select (CS) according to your board schematic.

3. Initialize driver handle structure

```c
MS5607_HW_InitTypeDef MS5607_Handle = {
    .SPIhandler = &hspi1,             // Your SPI handle configured in your project
    .CS_GPIOport = GPIOA,             // GPIO port of CS pin
    .CS_GPIOpin = GPIO_PIN_4,         // GPIO pin of CS
    .SPI_Timeout = 10                  // SPI timeout in milliseconds
};
```

4. Initialize the sensor

```c
if (MS5607_Init(&MS5607_Handle) != MS5607_STATE_READY) {
    // Handle error: sensor not detected or failed initialization
}
```

5. Start pressure and temperature conversions

```c
MS5607_Pressure_Conversion(&MS5607_Handle, MS5607_OSR_4096);
MS5607_Temperature_Conversion(&MS5607_Handle, MS5607_OSR_4096);
```

6. Read raw ADC data

```c
MS5607_Raw_Data_TypeDef raw_data;
MS5607_ADC_Read(&MS5607_Handle, &raw_data.pressure);
MS5607_ADC_Read(&MS5607_Handle, &raw_data.temperature);
```

7. Convert to compensated values

```c
MS5607_Converted_Data_TypeDef sensor_values;
MS5607_Data_Convert(&raw_data, &sensor_values);

int32_t pressure = sensor_values.pressure;       // Compensated pressure
int32_t temperature = sensor_values.temperature; // Compensated temperature
```

---

## **API Overview**

- `MS5607_Init()` — Initialize sensor and read PROM  
- `MS5607PromRead()` — Read calibration coefficients from PROM  
- `MS5607_Pressure_Conversion()` — Start uncompensated pressure conversion  
- `MS5607_Temperature_Conversion()` — Start uncompensated temperature conversion  
- `MS5607_ADC_Read()` — Read raw 24-bit ADC value  
- `MS5607_Data_Convert()` — Convert raw ADC to compensated pressure and temperature  
- `enableCS_MS5607()` / `disableCS_MS5607()` — Control SPI chip select  

---

## **License**

This project is licensed under the MIT License. See the LICENSE file for details.  
Based on the original MS5607-02 SPI library (MIT License) by João Pedro Vilas.

---

## **Author**

Nathan Netzel  
Electrical Engineering Student - Londrina State University

