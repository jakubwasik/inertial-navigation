/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "matrix.h"
#include "usbd_cdc_if.h"
#include "LSM6DS33_accelerometer.h"
#include "LIS3MDL_magnetometer.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// variables for storing accelerometer, gyroscope and magnetometer data
float acc_x_g = 0.0, acc_y_g = 0.0, acc_z_g = 0.0;
float gyro_x_dps = 0.0, gyro_y_dps = 0.0, gyro_z_dps = 0.0;
float mag_x_gauss = 0.0, mag_y_gauss = 0.0, mag_z_gauss = 0.0;

float roll = 0.0, pitch = 0.0, yaw = 0.0;
float roll_estimate = 0.0, pitch_estimate = 0.0, yaw_estimate = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
float kalmanFilter(float u, float y, float *A, float *B, float *C, float *Q,
		float R, float *x_corr, float *P_corr);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
	const int kLSM6DS33MessageLength = 12; // 12 - accelerometer 6 bytes, gyroscope 6 bytes
	const int kLIS3MDLMessageLength = 6;   // magnetometer 6 bytes
	const int kMessageLength = kLSM6DS33MessageLength + kLIS3MDLMessageLength;

	uint8_t data_to_send[kMessageLength];

	// Kalman filter variables
	float dt = 0.001;
	float A[] = { 1.0, -dt, 0.0, 1.0 };
	float B[] = { dt, 0.0 };
	float C[] = { 1.0, 0.0 };
	float R = 1;
	float q = 0.0001;
	float Q[] = { q, 0.0, 0.0, q };

	float u_roll = 0.0, u_pitch = 0.0, u_yaw = 0.0;
	float x_corr_roll[] = { 0.0, 0.0 }, x_corr_pitch[] = { 0.0, 0.0 },
			x_corr_yaw[] = { 0.0, 0.0 };
	float P_corr_roll[] = { q, 0.0, 0.0, q }, P_corr_pitch[] = { q, 0.0, 0.0, q },
			P_corr_yaw[] = { q, 0.0, 0.0, q };

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	MX_I2C3_Init();

  /* USER CODE BEGIN 2 */

	checkLIS3MDLConnection(&hi2c3);
	checkLSM6DS33Connection(&hi2c3);
	initializeLIS3MDL(&hi2c3);
	initializeLSM6DS33(&hi2c3);

	// Get new data
	accelerometerReadAllAxis(&hi2c3, &acc_x_g, &acc_y_g, &acc_z_g);
	gyroscopeReadAllAxis(&hi2c3, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
	magnetometerReadAllAxis(&hi2c3, &mag_x_gauss, &mag_y_gauss, &mag_z_gauss);

	roll = ((float) atan2(acc_z_g, acc_y_g) * (180.0 / 3.1415926) - 88.9494);
	pitch = ((float) atan2(acc_z_g, acc_x_g) * (180.0 / 3.1415926) - 91.139);
	yaw = ((float) atan2(mag_x_gauss, mag_y_gauss) * (180.0 / 3.1415926));

	u_roll = gyro_x_dps;
	u_pitch = gyro_y_dps;
	u_yaw = gyro_z_dps;

	x_corr_roll[0] = roll;
	x_corr_pitch[0] = pitch;
	x_corr_yaw[0] = yaw;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

		// Calculate new roll & pitch & yaw estimate
		roll_estimate = kalmanFilter(u_roll, roll, A, B, C, Q, R, x_corr_roll,
				P_corr_roll);
		pitch_estimate = kalmanFilter(u_pitch, pitch, A, B, C, Q, R, x_corr_pitch,
				P_corr_pitch);
		yaw_estimate = kalmanFilter(u_yaw, yaw, A, B, C, Q, R, x_corr_yaw,
				P_corr_yaw);

		//readDataFromLSM6DS33(&hi2c3, data_to_send);
		//readDataFromLIS3MDL(&hi2c3, data_to_send + kLSM6DS33MessageLength);
		//CDC_Transmit_FS(data_to_send, kMessageLength);

		HAL_Delay(1);

		// Get new data from IMU-10
		accelerometerReadAllAxis(&hi2c3, &acc_x_g, &acc_y_g, &acc_z_g);
		gyroscopeReadAllAxis(&hi2c3, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);
		magnetometerReadAllAxis(&hi2c3, &mag_x_gauss, &mag_y_gauss, &mag_z_gauss);

		// Calculate angles
		roll = ((float) atan2(acc_z_g, acc_y_g) * (180.0 / 3.1415926) - 88.9494);
		pitch = ((float) atan2(acc_z_g, acc_x_g) * (180.0 / 3.1415926) - 91.139);
		yaw = ((float) atan2(mag_x_gauss, mag_y_gauss) * (180.0 / 3.1415926));

		u_roll = gyro_x_dps;
		u_pitch = gyro_y_dps;
		u_yaw = gyro_z_dps;

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
float kalmanFilter(float u, float y, float *A, float *B, float *C, float *Q,
		float R, float *x_corr, float *P_corr) {
	float xk_pred[] = { 0.0, 0.0 };
	float Pk_pred[] = { 0.0, 0.0, 0.0, 0.0 };

	// temporary matrixes for multiplication purposes
	float Ax[] = { 0.0, 0.0 };
	float Bu[] = { 0.0, 0.0 };

	float AP[] = { 0.0, 0.0, 0.0, 0.0 };
	float AT[] = { 0.0, 0.0, 0.0, 0.0 };
	float APAT[] = { 0.0, 0.0, 0.0, 0.0 };

	float PCT[] = { 0.0, 0.0 };
	float CP[] = { 0.0, 0.0 };
	float K[] = { 0.0, 0.0 };
	float Ke[] = { 0.0, 0.0 };
	float KC[] = { 0.0, 0.0, 0.0, 0.0 };
	float eye[] = { 1.0, 0.0, 0.0, 1.0 };
	float eye_minus_KC[] = { 0.0, 0.0, 0.0, 0.0 };

	float CPCT = 0.0;
	float S = 0.0;
	float Cx = 0.0;
	float error = 0.0;

	// xk_pred = A*xk_corr + B*u
	matrix_2x2_mul_2x1(A, x_corr, Ax);
	matrix_2x1_mul_1x1(B, &u, Bu);
	matrix_2x1_add_2x1(Ax, Bu, xk_pred);

	// Pk_pred = A*Pk_corr*transpose(A) + Q
	matrix_2x2_mul_2x2(A, P_corr, AP);
	matrix_2x2_trans(A, AT);
	matrix_2x2_mul_2x2(AP, AT, APAT);
	matrix_2x2_add_2x2(APAT, Q, Pk_pred);

	// error = y - C*xk_pred
	matrix_1x2_mul_2x1(C, xk_pred, &Cx);
	error = y - Cx;

	// K = (Pk_pred*transpose(C)) / (C*Pk_pred*transpose(C) + R)
	matrix_2x2_mul_2x1(Pk_pred, C, PCT);
	matrix_1x2_mul_2x2(C, Pk_pred, CP);
	matrix_1x2_mul_2x1(CP, C, &CPCT);
	S = 1.0 / (float) (CPCT + R);
	matrix_2x1_mul_1x1(PCT, &S, K);

	// xk_corr = xk_pred + K*error
	matrix_2x1_mul_1x1(K, &error, Ke);
	matrix_2x1_add_2x1(xk_pred, Ke, x_corr);

	// Pk_corr = (eye(2) - K*C) / Pk_pred
	matrix_2x1_mul_1x2(K, C, KC);
	matrix_2x2_sub_2x2(eye, KC, eye_minus_KC);
	matrix_2x2_mul_2x2(eye_minus_KC, Pk_pred, P_corr);

	// We have calculated new estimate
	return x_corr[0];
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
