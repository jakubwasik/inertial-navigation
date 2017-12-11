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

// Angular rate sensor sign and orientation register
#define LSM6DS33_ORIENT_CFG_G 0x0B

// Linear acceleration control register 1
// CTRL1_XL [ODR_XL3][ODR_XL2][ODR_XL1][ODR_XL0][FS_XL1][FS_XL0][BW_XL1][BW_XL0]
#define LSM6DS33_CTRL1_XL 0x10

// Angular rate control register 2
// CTRL1_XL [ODR_G3][ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][FS_125][0]
#define LSM6DS33_CTRL2_G 0x11

// Accelerometer data registers
#define LSM6DS33_OUTX_L_XL 0x28
#define LSM6DS33_OUTX_H_XL 0x29
#define LSM6DS33_OUTY_L_XL 0x2A
#define LSM6DS33_OUTY_H_XL 0x2B
#define LSM6DS33_OUTZ_L_XL 0x2C
#define LSM6DS33_OUTZ_H_XL 0x2D

// Gyroscope data registers
#define LSM6DS33_OUTX_L_G 0x22
#define LSM6DS33_OUTX_H_G 0x23
#define LSM6DS33_OUTY_L_G 0x24
#define LSM6DS33_OUTY_H_G 0x25
#define LSM6DS33_OUTZ_L_G 0x26
#define LSM6DS33_OUTZ_H_G 0x27

/*************** end of registry addresses ***************/

// This value indicates correct communication
#define LSM6DS33_WHO_AM_I_OK 0x69

// Gyroscope orientation Orient[2:0] in ORIENT_CFG_G -> in order to achieve proper roll & pitch & yaw
#define LSM6DS33_GYRO_ORIENT 0x02

// LSM6DS33 accelerometer & gyroscope address expressed as a 8 bit number
#define LSM6DS33_ADDRESS (0x6B << 1)

// ODR_XL[3:0] = 0b0100, 104HZ HIGH PERFORMANCE with XL_HM_MODE = 0 (default)
#define LSM6DS33_ACC_104HZ 0x40
#define LSM6DS33_ACC_208HZ 0x50
#define LSM6DS33_ACC_416HZ 0x60 // read every 0.0025s

// Accelerometer full-scale selection [g] FS_XL[1:0]
#define LSM6DS33_ACC_RESOLUTION 2.0

// ODR_G[3:0] = 0b0100, 104HZ HIGH PERFORMANCE with XL_HM_MODE = 0 (default)
#define LSM6DS33_GYRO_104HZ 0x40
#define LSM6DS33_GYRO_208HZ 0x50
#define LSM6DS33_GYRO_416HZ 0x60

// Gyroscope full-scale selection [dps] FS_G[1:0]
#define LSM6DS33_GYRO_RESOLUTION 245.0

void checkLSM6DS33Connection(I2C_HandleTypeDef *hi2c);
void initializeLSM6DS33(I2C_HandleTypeDef *hi2c);
void accelerometerReadAllAxis(I2C_HandleTypeDef *hi2c, float *acc_x,
		float *acc_y, float *acc_z);
void gyroscopeReadAllAxis(I2C_HandleTypeDef *hi2c, float *gyro_x, float *gyro_y,
		float *gyro_z);

// function read all data from Accelerometer and Gyroscope
// Data, which is stored in uint8_t vector, is used by USB communication
void readDataFromLSM6DS33(I2C_HandleTypeDef *hi2c, uint8_t *data);

#endif /* LSM6DS33_ACCELEROMETER_H_ */
