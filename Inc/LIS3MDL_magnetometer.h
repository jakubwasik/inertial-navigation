/*
 * LIS3MDL_accelerometer.h
 *
 *  Created on: 29.10.2017
 *      Author: Micha³
 */

#ifndef LIS3MDL_MAGNETOMETER_H_
#define LIS3MDL_MAGNETOMETER_H_

#include "stm32f4xx_hal.h"

/*************** registry addresses ***************/

// Who am I
#define LIS3MDL_WHO_AM_I 0x0F

// Magnetometer control register 1
// CTRL1 [TEMP_EN][OM1][OM0][DO2][DO1][DO0][FAST_ODR][ST]
#define LIS3MDL_CTRL1 0x20

// Magnetometer control register 2
// CTRL2 [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
#define LIS3MDL_CTRL2 0x21

// Magnetometer control register 3
// CTRL3 [0][0][LP][0][0][SIM][MD1][MD0]
#define LIS3MDL_CTRL3 0x22

// Magnetometer control register 4
// CTRL4 [0][0][0][0][OMZ1][OMZ0][BLE][0]
#define LIS3MDL_CTRL4 0x23

// Magnetometer data registers
#define LIS3MDL_OUTX_L 0x28
#define LIS3MDL_OUTX_H 0x29
#define LIS3MDL_OUTY_L 0x2A
#define LIS3MDL_OUTY_H 0x2B
#define LIS3MDL_OUTZ_L 0x2C
#define LIS3MDL_OUTZ_H 0x2D

/*************** end of registry addresses ***************/

// This value indicates correct communication
#define LIS3MDL_WHO_AM_I_OK 0x3D

// LIS3MDL accelerometer & gyroscope address expressed as a 8 bit number
#define LIS3MDL_ADDRESS (0x1E << 1)

// OM[1:0] - Operating mode for X,Y axes -> high-performance
#define LIS3MDL_MAG_OM_XY (0x10 << 5)

// OMZ[1:0] - Operating mode for Z axis -> high-performance
#define LIS3MDL_MAG_OM_Z (0x10 << 2)

// ODR_XL[2:0] - Output Data Rate [Hz]
#define LIS3MDL_MAG_10HZ (0x04 << 2)
#define LIS3MDL_MAG_20HZ (0x05 << 2)
#define LIS3MDL_MAG_40HZ (0x06 << 2)
#define LIS3MDL_MAG_80HZ (0x07 << 2)

// MD[1:0] - Power mode selection - default 0x11 power-down
#define LIS3MDL_MAG_CONTI_CONV_MODE 0x00

// Magnetometer full-scale selection [gauss] FS_XL[1:0] (default +- 4 gauss)
#define LIS3MDL_MAG_RESOLUTION 4.0

void checkLIS3MDLConnection(I2C_HandleTypeDef *hi2c);
void initializeLIS3MDL(I2C_HandleTypeDef *hi2c);

void magnetometerReadAllAxis(I2C_HandleTypeDef *hi2c, float *mag_x,
		float *mag_y, float *mag_z);

// function read all data from Magnetometer
// Data, which is stored in uint8_t vector, is used by USB communication
void readDataFromLIS3MDL(I2C_HandleTypeDef *hi2c, uint8_t *data);

#endif /* LIS3MDL_MAGNETOMETER_H_ */
