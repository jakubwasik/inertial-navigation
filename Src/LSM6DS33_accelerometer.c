/*
 * LSM6DS33_accelerometer.c
 *
 *  Created on: 30.10.2017
 *      Author: Micha³
 */

#include "LSM6DS33_accelerometer.h"
#include "limits.h"

/*************** constants ***************/

const uint32_t kI2CTimeout = 100;

/*************** end of constants ***************/

void checkI2CConnection(I2C_HandleTypeDef *hi2c) {
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ACC_ADDRESS, LSM6DS33_WHO_AM_I, 1, &data, 1,
			kI2CTimeout);
	if (data != LSM6DS33_WHO_AM_I_OK) {
		while (1) {
			// wrong communication -> we have a problem
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			HAL_Delay(100);
		}
	}
}

void initializeI2C(I2C_HandleTypeDef *hi2c) {
	uint8_t settings = LSM6DS33_ACC_104HZ;

	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ACC_ADDRESS, LSM6DS33_CTRL1_XL, 1, &settings,
			1, kI2CTimeout);
}

int16_t accelerometerReadAxisValue(I2C_HandleTypeDef *hi2c, uint16_t axis) {
	uint8_t data[2];
	uint16_t temp;
	int16_t axis_value;

	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ACC_ADDRESS, axis, 1, data, 2,
			kI2CTimeout);

	temp = (data[1] << 8) + data[0];
	axis_value = temp > INT16_MAX ? temp + 2 * INT16_MIN : temp;

	return axis_value;
}

