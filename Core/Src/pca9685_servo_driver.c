/*
 * pca9685_servo_driver.c
 *
 *  Created on: Feb 5, 2025
 *      Author: Muhtasim Rahman
 */

#include <assert.h>
#include <math.h>
#include <pca9685_servo_driver.h>
#include <stdbool.h>

#include <stm32f4xx_hal.h>

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;

uint8_t buffer[48];

/**
 * @brief  Initializes PWM Driver
 * @retval void
 */
void init()
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c3, PCA9685_I2C_ADDRESS<<1, 1, HAL_MAX_DELAY);

	if(ret == HAL_OK){
		strcpy((char*)buffer, "DEVICE STATUS:\tREADY\r\n");
	}else{
		strcpy((char*)buffer, "DEVICE STATUS:\tNOT READY\r\n");
	}
}

//  Read from I2C address
