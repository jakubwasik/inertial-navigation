/*
 * LSM6DS33_accelerometer.h
 *
 *  Created on: 29.10.2017
 *      Author: Micha³
 */

#ifndef LSM6DS33_ACCELEROMETER_H_
#define LSM6DS33_ACCELEROMETER_H_

#include "stm32f4xx_hal.h"

/*************** registry addresses ***************/

// Who am I
#define LSM6DS33_WHO_AM_I 0x0F

// control register 1
// CTRL1_XL [ODR_XL3][ODR_XL2][ODR_XL1][ODR_XL0][FS_XL1][FS_XL0][BW_XL1][BW_XL0]
#define LSM6DS33_CTRL1_XL 0x10

// Accelerometer data registers
#define LSM6DS33_OUTX_L_XL 0x28
#define LSM6DS33_OUTX_H_XL 0x29
#define LSM6DS33_OUTY_L_XL 0x2A
#define LSM6DS33_OUTY_H_XL 0x2B
#define LSM6DS33_OUTZ_L_XL 0x2C
#define LSM6DS33_OUTZ_H_XL 0x2D

/*************** end of registry addresses ***************/

// accelerometer address expressed as a 8 bit number
#define LSM6DS33_ACC_ADDRESS (0x6B << 1)

// ODR = 0b0100, 104HZ HIGH PERFORMANCE with XL_HM_MODE = 0
#define LSM6DS33_ACC_104HZ 0x40

// Maximum geforce value [g]
#define LSM6DS33_ACC_RESOLUTION 2.0

// This value indicates correct communication
#define LSM6DS33_WHO_AM_I_OK 0x69

void checkI2CConnection(I2C_HandleTypeDef *hi2c);
void initializeI2C(I2C_HandleTypeDef *hi2c);
int16_t accelerometerReadAxisValue(I2C_HandleTypeDef *hi2c, uint16_t axis);

#endif /* LSM6DS33_ACCELEROMETER_H_ */
