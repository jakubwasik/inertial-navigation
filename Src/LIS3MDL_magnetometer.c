/*
 * LIS3MDL_accelerometer.c
 *
 *  Created on: 30.10.2017
 *      Author: Micha³
 */

#include "LIS3MDL_magnetometer.h"
#include "limits.h"

/*************** constants ***************/

const uint32_t kLIS3MDLTimeout = 100;

/*************** end of constants ***************/

void checkLIS3MDLConnection(I2C_HandleTypeDef *hi2c) {
	uint8_t data;
	HAL_I2C_Mem_Read(hi2c, LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I, 1, &data, 1,
			kLIS3MDLTimeout);
	if (data != LIS3MDL_WHO_AM_I_OK) {
		while (1) {
			// wrong communication -> we have a problem
			HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
			HAL_Delay(100);
		}
	}
}

void initializeLIS3MDL(I2C_HandleTypeDef *hi2c) {
	uint8_t mag_ctrl1_settings = LIS3MDL_MAG_OM_XY + LIS3MDL_MAG_80HZ;
	uint8_t mag_ctrl3_settings = LIS3MDL_MAG_CONTI_CONV_MODE;
	uint8_t mag_ctrl4_settings = LIS3MDL_MAG_OM_Z;

	// write settings to magnetometer control register 1
	HAL_I2C_Mem_Write(hi2c, LIS3MDL_ADDRESS, LIS3MDL_CTRL1, 1,
			&mag_ctrl1_settings,
			1, kLIS3MDLTimeout);

	HAL_Delay(50);

	// write settings to magnetometer control register 3
	HAL_I2C_Mem_Write(hi2c, LIS3MDL_ADDRESS, LIS3MDL_CTRL3, 1,
			&mag_ctrl3_settings,
			1, kLIS3MDLTimeout);

	HAL_Delay(50);

	// write settings to magnetometer control register 4
	HAL_I2C_Mem_Write(hi2c, LIS3MDL_ADDRESS, LIS3MDL_CTRL4, 1,
			&mag_ctrl4_settings, 1, kLIS3MDLTimeout);

	HAL_Delay(50);

}

void magnetometerReadAllAxis(I2C_HandleTypeDef *hi2c, float *mag_x,
		float *mag_y, float *mag_z) {
	uint8_t data[6];
	int16_t temp_x, temp_y, temp_z;
	float offset[] = { 3.48, -0.162, 0.177 };
	float gain[] = { 1.17, 1.133, 1.14 };

	// read high and low byte from all axis -> start address is LSM6DS33_OUTX_L_g
	HAL_I2C_Mem_Read(hi2c, LIS3MDL_ADDRESS, LIS3MDL_OUTX_L, 1, data, 6,
			kLIS3MDLTimeout);

	// decode the data
	temp_x = (data[1] << 8) + data[0];
	temp_y = (data[3] << 8) + data[2];
	temp_z = (data[5] << 8) + data[4];

	// convert data to gauss
	*mag_x = (((float) (temp_x * LIS3MDL_MAG_RESOLUTION) / (float) INT16_MAX)
			- offset[0]) * gain[0];
	*mag_y = (((float) (temp_y * LIS3MDL_MAG_RESOLUTION) / (float) INT16_MAX)
			- offset[1]) * gain[1];
	*mag_z = (((float) (temp_z * LIS3MDL_MAG_RESOLUTION) / (float) INT16_MAX)
			- offset[2]) * gain[2];
}

void readDataFromLIS3MDL(I2C_HandleTypeDef *hi2c, uint8_t *data) {
	if (data != NULL) {
	// read high and low byte from all axis -> start address is LIS3MDL_OUTX_L
	HAL_I2C_Mem_Read(hi2c, LIS3MDL_ADDRESS, LIS3MDL_OUTX_L, 1, data, 6,
				kLIS3MDLTimeout);
	}

/* data[0] - mag_x LB
 * data[1] - mag_x HB
 * data[2] - mag_y LB
 * data[3] - mag_y HB
 * data[4] - mag_z LB
	 * data[5] - mag_z HB
	 */
}

