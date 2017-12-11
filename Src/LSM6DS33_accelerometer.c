/*
 * LSM6DS33_accelerometer.c
 *
 *  Created on: 30.10.2017
 *      Author: Micha³
 */

#include "LSM6DS33_accelerometer.h"
#include "limits.h"

/*************** constants ***************/

const uint32_t kLSM6DS33Timeout = 100;

/*************** end of constants ***************/

void checkLSM6DS33Connection(I2C_HandleTypeDef *hi2c) {
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_WHO_AM_I, 1, &data, 1,
			kLSM6DS33Timeout);
	if (data != LSM6DS33_WHO_AM_I_OK) {
		while (1) {
			// wrong communication -> we have a problem
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			HAL_Delay(100);
		}
	}
}

void initializeLSM6DS33(I2C_HandleTypeDef *hi2c) {
	uint8_t acc_settings = LSM6DS33_ACC_416HZ;
	uint8_t gyro_settings = LSM6DS33_GYRO_416HZ;
	uint8_t gyro_orientation = LSM6DS33_GYRO_ORIENT;

	// write accelerometer settings
	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_CTRL1_XL, 1, &acc_settings,
			1, kLSM6DS33Timeout);

	HAL_Delay(50);

	// write gyroscope settings
	HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_CTRL2_G, 1, &gyro_settings,
			1, kLSM6DS33Timeout);

	HAL_Delay(50);

	// change gyroscope orientation from (X,Y,Z) to (Y,X,Z) (because in (X,Y,Z) X is not a pitch)
	// EDIT: above line is wrong -> orientation is OK
	//HAL_I2C_Mem_Write(hi2c, LSM6DS33_ADDRESS, LSM6DS33_ORIENT_CFG_G, 1,
	//		&gyro_orientation, 1, kLSM6DS33Timeout);
}

void accelerometerReadAllAxis(I2C_HandleTypeDef *hi2c, float *acc_x,
		float *acc_y, float *acc_z) {
	uint8_t data[6];
	int16_t temp_x, temp_y, temp_z;
	float offset[] = { 0.004, 0, 0.021 };
	float gain[] = { 0.975, 0.99, 0.97 };

	// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_XL
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_XL, 1, data,
			6,
			kLSM6DS33Timeout);

	// decode the data
	temp_x = (data[1] << 8) + data[0];
	temp_y = (data[3] << 8) + data[2];
	temp_z = (data[5] << 8) + data[4];

	// convert data to g
	*acc_x = (((float) (temp_x * LSM6DS33_ACC_RESOLUTION) / (float) INT16_MAX)
			- offset[0]) * gain[0];
	*acc_y = (((float) (temp_y * LSM6DS33_ACC_RESOLUTION) / (float) INT16_MAX)
			- offset[1]) * gain[1];
	*acc_z = (((float) (temp_z * LSM6DS33_ACC_RESOLUTION) / (float) INT16_MAX)
			- offset[2]) * gain[2];
}


void gyroscopeReadAllAxis(I2C_HandleTypeDef *hi2c, float *gyro_x, float *gyro_y,
		float *gyro_z) {
	uint8_t data[6];
	int16_t temp_x, temp_y, temp_z;

	// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_g
	HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_G, 1, data, 6,
			kLSM6DS33Timeout);

	// decode the data
	temp_x = (data[1] << 8) + data[0];
	temp_y = (data[3] << 8) + data[2];
	temp_z = (data[5] << 8) + data[4];

	// convert data to dps
	*gyro_x = (float) (temp_x * LSM6DS33_GYRO_RESOLUTION) / (float) INT16_MAX;
	*gyro_y = (float) (temp_y * LSM6DS33_GYRO_RESOLUTION) / (float) INT16_MAX;
	*gyro_z = (float) (temp_z * LSM6DS33_GYRO_RESOLUTION) / (float) INT16_MAX;
}

void readDataFromLSM6DS33(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	if (data != NULL) {
		// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_g
		HAL_I2C_Mem_Read(hi2c, LSM6DS33_ADDRESS, LSM6DS33_OUTX_L_G, 1, data, 12,
				kLSM6DS33Timeout);
	}

	/* data[0] - gyro_x LB
	 * data[1] - gyro_x HB
	 * data[2] - gyro_y LB
	 * data[3] - gyro_y HB
	 * data[4] - gyro_z LB
	 * data[5] - gyro_z HB
	 * data[6] - acc_x LB
	 * data[7] - acc_x HB
	 * data[8] - acc_y LB
	 * data[9] - acc_y HB
	 * data[10] - acc_z LB
	 * data[11] - acc_z HB
	 */
}

