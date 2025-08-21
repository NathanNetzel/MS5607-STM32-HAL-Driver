/*
 * MS5607SPI.h
 *
 * Based on:
 *   MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
 *   05/01/2020 by João Pedro Vilas <joaopedrovbs@gmail.com>
 *
 * Changelog:
 *   2012-05-23 - Initial release by João Pedro Vilas.
 *   2025-07-28 - Adapted by Nathan Netzel for STM32F4 series,
 *                refactored API to use MS5607_HW_InitTypeDef structure,
 *                added HAL error handling on SPI communication,
 *                split functionality into smaller modular functions,
 *                improved code readability and added Doxygen-style comments.
 *
 * ============================================================================================
 * MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
 * Copyright (c) 2020 João Pedro Vilas Boas
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ============================================================================================
 */

#ifndef _MS5607SPI_H_
#define _MS5607SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// --- MS5607 SPI Commands ---
#define RESET_COMMAND                 0x1E
#define PROM_READ(address)            (0xA0 | ((address) << 1))  	/**< Macro to access PROM addresses (0 to 7) */
#define CONVERT_D1_COMMAND            0x40                      	/**< Command to start pressure conversion */
#define CONVERT_D2_COMMAND            0x50                      	/**< Command to start temperature conversion */
#define READ_ADC_COMMAND              0x00                      	/**< Command to read ADC result */

// --- MS5607 Oversampling Ratio (OSR) Values ---
#define MS5607_OSR_256		0x00
#define MS5607_OSR_512		0x02
#define MS5607_OSR_1024		0x04
#define MS5607_OSR_2048		0x06
#define MS5607_OSR_4096		0x08

// --- MS5607 System States ---
typedef enum MS5607States {
  MS5607_STATE_FAILED,   /**< Sensor communication failed or invalid data */
  MS5607_STATE_READY,    /**< Sensor ready for data acquisition */
  MS5607_STATE_BUSY,     /**< Sensor busy performing conversion */
  MS5607_HAL_ERROR       /**< HAL SPI communication error */
} MS5607StateTypeDef;

// --- MS5607 PROM Data Structure ---
struct promData{
  uint16_t reserved;	/**< Reserved PROM register */
  uint16_t sens;		/**< Sensitivity coefficient */
  uint16_t off;			/**< Offset coefficient */
  uint16_t tcs;			/**< Temperature coefficient of sensitivity */
  uint16_t tco;			/**< Temperature coefficient of offset */
  uint16_t tref;		/**< Reference temperature */
  uint16_t tempsens;	/**< Temperature sensitivity */
  uint16_t crc;			/**< CRC for PROM data integrity */
};

// --- Raw uncompensated sensor data ---
typedef struct MS5607UncompensatedValues{
  uint32_t  pressure;
  uint32_t  temperature;
} MS5607_Raw_Data_TypeDef;

// --- Converted sensor readings (compensated) ---
typedef struct MS5607Readings{
  int32_t  pressure;
  int32_t  temperature;
} MS5607_Converted_Data_TypeDef;

// --- Hardware initialization structure ---
typedef struct {
  SPI_HandleTypeDef *SPIhandler;    /**< Pointer to SPI handle */
  GPIO_TypeDef *CS_GPIOport;        /**< GPIO port for chip select */
  uint16_t CS_GPIOpin;              /**< GPIO pin number for chip select */
  uint8_t SPI_Timeout;              /**< Timeout duration for SPI communication */
} MS5607_HW_InitTypeDef;

/**
 * @brief Initialize MS5607 sensor and read PROM calibration data
 * @param MS5607_Handler Pointer to hardware initialization structure
 * @return MS5607StateTypeDef Initialization status (READY, FAILED, HAL_ERROR)
 */
MS5607StateTypeDef MS5607_Init(MS5607_HW_InitTypeDef *);

/**
 * @brief Read the PROM content of the MS5607 sensor
 * @note This function should be called only once during sensor initialization
 * @param MS5607_Handler Pointer to hardware initialization structure
 * @param prom Pointer to promData structure to store PROM values
 * @return MS5607StateTypeDef Status of PROM read operation
 */
MS5607StateTypeDef MS5607PromRead(MS5607_HW_InitTypeDef *, struct promData *prom);

/**
 * @brief Start uncompensated pressure (D1) conversion with specified oversampling ratio
 * @param MS5607_Handler Pointer to hardware initialization structure
 * @param MS5607_Press_OSR Oversampling ratio value (use MS5607_OSR_XXX macros)
 * @return MS5607StateTypeDef Status of pressure conversion command
 */
MS5607StateTypeDef MS5607_Pressure_Conversion(MS5607_HW_InitTypeDef *, uint8_t);

/**
 * @brief Start uncompensated temperature (D2) conversion with specified oversampling ratio
 * @param MS5607_Handler Pointer to hardware initialization structure
 * @param MS5607_Temp_OSR Oversampling ratio value (use MS5607_OSR_XXX macros)
 * @return MS5607StateTypeDef Status of temperature conversion command
 */
MS5607StateTypeDef MS5607_Temperature_Conversion(MS5607_HW_InitTypeDef *, uint8_t);

/**
 * @brief Initiate ADC read command to retrieve conversion result
 * @param MS5607_Handler Pointer to hardware initialization structure
 * @param raw_data Pointer to store 24-bit ADC raw data
 * @return MS5607StateTypeDef Status of ADC read operation
 */
MS5607StateTypeDef MS5607_ADC_Read(MS5607_HW_InitTypeDef *, uint32_t *);

/**
 * @brief Convert raw uncompensated pressure and temperature values to compensated values
 * @param sample Pointer to raw data structure containing uncompensated values
 * @param value Pointer to structure where compensated values will be stored
 */
void MS5607_Data_Convert(MS5607_Raw_Data_TypeDef *, MS5607_Converted_Data_TypeDef *);

/**
 * @brief Enable the Chip Select (CS) pin (active low)
 * @param CS_GPIOport GPIO port for CS pin
 * @param CS_GPIOpin GPIO pin number for CS pin
 */
void enableCS_MS5607(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin);

/**
 * @brief Disable the Chip Select (CS) pin (inactive high)
 * @param CS_GPIOport GPIO port for CS pin
 * @param CS_GPIOpin GPIO pin number for CS pin
 */
void disableCS_MS5607(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin);

#endif /* _MS5607SPI_H_ */
