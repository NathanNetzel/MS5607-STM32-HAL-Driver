/*
 * MS5607SPI.c
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

#include <MS5607SPI.h>

/* PROM data structure */
static struct promData promData;

/**
 * @brief  Resets the sensor and initializes PROM values
 * @note   Performs device reset and reads calibration PROM data
 * @param  MS5607_Handler Pointer to the MS5607 hardware init structure
 * @retval MS5607StateTypeDef Status of the initialization
 */
MS5607StateTypeDef MS5607_Init(MS5607_HW_InitTypeDef *MS5607_Handler) {

	uint8_t SPITransmitData;

	enableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
	SPITransmitData = RESET_COMMAND;

	if(HAL_SPI_Transmit(MS5607_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
		{
		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
			return MS5607_HAL_ERROR;
	}

	HAL_Delay(3);
	disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

	MS5607PromRead(MS5607_Handler, &promData);

	if (promData.off == 0x00 || promData.tref == 0xff)
		return MS5607_STATE_FAILED;
	else
		return MS5607_STATE_READY;
}

/**
 * @brief  Reads the calibration PROM from the sensor
 * @param  MS5607_Handler Pointer to the MS5607 hardware init structure
 * @param  prom Pointer to the promData structure to fill
 * @retval MS5607StateTypeDef Status of the PROM read
 */
MS5607StateTypeDef MS5607PromRead(MS5607_HW_InitTypeDef *MS5607_Handler, struct promData *prom){
	uint8_t   address;
	uint16_t  *structPointer;
	uint8_t SPITransmitData;

	structPointer = (uint16_t *) prom;

	for (address = 0; address < 8; address++) {
		SPITransmitData = PROM_READ(address);
		enableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

		if(HAL_SPI_Transmit(MS5607_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
		{
			disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
			return MS5607_HAL_ERROR;
		}

		/* Receive two bytes and store directly in the structure */
		if(HAL_SPI_Receive(MS5607_Handler->SPIhandler, structPointer, 2, 10) != HAL_OK)
		{
			disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
			return MS5607_HAL_ERROR;
		}

		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
		structPointer++;
	}

	structPointer = (uint16_t *) prom;
	for (address = 0; address < 8; address++) {
		uint8_t   *toSwap = (uint8_t *) structPointer;
		uint8_t secondByte = toSwap[0];
		toSwap[0] = toSwap[1];
		toSwap[1] = secondByte;
		structPointer++;
	}

	return MS5607_STATE_READY;
}

/**
 * @brief  Starts an uncompensated pressure conversion (D1)
 * @param  MS5607_Handler Pointer to the MS5607 hardware init structure
 * @param  MS5607_Press_OSR Oversampling ratio for pressure conversion
 * @retval MS5607StateTypeDef Status of the command
 */
MS5607StateTypeDef MS5607_Pressure_Conversion(MS5607_HW_InitTypeDef *MS5607_Handler, uint8_t MS5607_Press_OSR){

	uint8_t SPITransmitData;
	enableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
	SPITransmitData = CONVERT_D1_COMMAND | MS5607_Press_OSR;
	if(HAL_SPI_Transmit(MS5607_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
		return MS5607_HAL_ERROR;
	}
	disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

	return MS5607_STATE_BUSY;
}

/**
 * @brief  Starts an uncompensated temperature conversion (D2)
 * @param  MS5607_Handler Pointer to the MS5607 hardware init structure
 * @param  MS5607_Temp_OSR Oversampling ratio for temperature conversion
 * @retval MS5607StateTypeDef Status of the command
 */
MS5607StateTypeDef MS5607_Temperature_Conversion(MS5607_HW_InitTypeDef *MS5607_Handler, uint8_t MS5607_Temp_OSR){

	uint8_t SPITransmitData;

	enableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
	SPITransmitData = CONVERT_D2_COMMAND | MS5607_Temp_OSR;
	if(HAL_SPI_Transmit(MS5607_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
		return MS5607_HAL_ERROR;
	}
  	disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

  	return MS5607_STATE_BUSY;
}

/**
 * @brief  Reads the ADC result from the sensor
 * @param  MS5607_Handler Pointer to the MS5607 hardware init structure
 * @param  raw_data Pointer to store the 24-bit ADC raw data
 * @retval MS5607StateTypeDef Status of the read operation
 */
MS5607StateTypeDef MS5607_ADC_Read(MS5607_HW_InitTypeDef *MS5607_Handler, uint32_t *raw_data){

	uint8_t SPITransmitData;
	uint8_t reply[3]; 

	enableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

	SPITransmitData = READ_ADC_COMMAND;
	if(HAL_SPI_Transmit(MS5607_Handler->SPIhandler, &SPITransmitData, 1, 10) != HAL_OK)
	{
		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
		return MS5607_HAL_ERROR;
	}
	if(HAL_SPI_Receive(MS5607_Handler->SPIhandler, reply, 3, 10) != HAL_OK)
	{
		disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);
		return MS5607_HAL_ERROR;
	}
	disableCS_MS5607(MS5607_Handler->CS_GPIOport, MS5607_Handler->CS_GPIOpin);

	*raw_data = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

	return MS5607_STATE_READY;
}


/**
 * @brief  Converts raw sensor data into compensated pressure and temperature values
 * @param  sample Pointer to raw data structure containing pressure and temperature
 * @param  value Pointer to structure to store converted pressure and temperature
 * @retval None
 */
void MS5607_Data_Convert(MS5607_Raw_Data_TypeDef *sample, MS5607_Converted_Data_TypeDef *value){
	int32_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;

	dT = sample->temperature - ((int32_t) (promData.tref << 8));

	TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);

	OFF = ((int64_t) promData.off << 17) + (((int64_t) promData.tco * dT) >> 6);
	SENS = ((int64_t) promData.sens << 16) + (((int64_t) promData.tcs * dT) >> 7);


	if (TEMP < 2000) {
		int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
		int32_t TEMPM = TEMP - 2000;
		int64_t OFF2 = (61 * (int64_t) TEMPM * (int64_t) TEMPM) >> 4;
		int64_t SENS2 = (2 * (int64_t) TEMPM * (int64_t) TEMPM);
		if (TEMP < -1500) {
			int32_t TEMPP = TEMP + 1500;
			int32_t TEMPP2 = TEMPP * TEMPP;
			OFF2 = OFF2 + (int64_t) 15 * TEMPP2;
			SENS2 = SENS2 + (((int64_t) 8 * TEMPP2));
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}

	value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
	value->temperature = TEMP;

}

/**
 * @brief  Enables the CS pin (sets it LOW)
 * @param  CS_GPIOport GPIO port of the CS pin
 * @param  CS_GPIOpin GPIO pin number of the CS pin
 * @retval None
 */
void enableCS_MS5607(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_RESET);
}

/**
 * @brief  Disables the CS pin (sets it HIGH)
 * @param  CS_GPIOport GPIO port of the CS pin
 * @param  CS_GPIOpin GPIO pin number of the CS pin
 * @retval None
 */
void disableCS_MS5607(GPIO_TypeDef *CS_GPIOport, uint16_t CS_GPIOpin){
  HAL_GPIO_WritePin(CS_GPIOport, CS_GPIOpin, GPIO_PIN_SET);
}

