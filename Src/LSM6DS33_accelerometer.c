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
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_WHO_AM_I, 1, &data, 1,
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
	uint8_t acc_settings = LSM6DS33_ACC_104HZ;
	uint8_t gyro_settings = LSM6DS33_GYRO_104HZ;
	uint8_t gyro_orientation = LSM6DS33_GYRO_ORIENT;

	// write accelerometer settings
	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_CTRL1_XL, 1, &acc_settings,
			1, kI2CTimeout);

	HAL_Delay(50);

	// write gyroscope settings
	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G, 1, &gyro_settings,
			1, kI2CTimeout);

	HAL_Delay(50);

	// change gyroscope orientation from (X,Y,Z) to (Y,X,Z) (because in (X,Y,Z) X is not a pitch)
	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_ORIENT_CFG_G, 1,
			&gyro_orientation, 1, kI2CTimeout);
}

void accelerometerReadAllAxis(I2C_HandleTypeDef *hi2c, int16_t *acc_x,
		int16_t *acc_y, int16_t *acc_z) {
	uint8_t data[6];
	uint16_t temp_x, temp_y, temp_z;

	// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_XL
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_XL, 1, data,
			6, kI2CTimeout);

	// decode the data
	temp_x = (data[1] << 8) + data[0];
	temp_y = (data[3] << 8) + data[2];
	temp_z = (data[5] << 8) + data[4];

	// convert data to signed 16-bit integer
	*acc_x = temp_x > INT16_MAX ? temp_x + 2 * INT16_MIN : temp_x;
	*acc_y = temp_y > INT16_MAX ? temp_y + 2 * INT16_MIN : temp_y;
	*acc_z = temp_z > INT16_MAX ? temp_z + 2 * INT16_MIN : temp_z;
}

void gyroscopeReadAllAxis(I2C_HandleTypeDef *hi2c, int16_t *gyro_x,
		int16_t *gyro_y, int16_t *gyro_z) {
	uint8_t data[6];
	uint16_t temp_x, temp_y, temp_z;

	// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_g
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_G, 1, data, 6,
			kI2CTimeout);

	// decode the data
	temp_x = (data[1] << 8) + data[0];
	temp_y = (data[3] << 8) + data[2];
	temp_z = (data[5] << 8) + data[4];

	// convert data to signed 16-bit integer
	*gyro_x = temp_x > INT16_MAX ? temp_x + 2 * INT16_MIN : temp_x;
	*gyro_y = temp_y > INT16_MAX ? temp_y + 2 * INT16_MIN : temp_y;
	*gyro_z = temp_z > INT16_MAX ? temp_z + 2 * INT16_MIN : temp_z;
}

