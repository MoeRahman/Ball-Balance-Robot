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
#include <stdint.h>
#include <string.h>

#include <stm32f4xx_hal.h>

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart1;

uint8_t buffer[48];


/**
 * @brief Set the Bit object
 * 
 * @param Register 
 * @param Bit 
 * @param Value 
 */
void setBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    // Read specified register
    HAL_I2C_Mem_Read(&hi2c3, PCA9685_I2C_ADDRESS, Register, 1, &readValue, 1, HAL_MAX_DELAY);
    if(Value == 0) readValue &= ~(1<<Bit);
    else readValue |= (1<<Bit);
    HAL_I2C_Mem_Write(&hi2c3, PCA9685_I2C_ADDRESS, Register, 1, &readValue, 1, HAL_MAX_DELAY);
    HAL_Delay(1);
}


/**
 * @brief Set the PWM frequency
 * 
 * @param frequency 
 */
void setPWMFreq(uint16_t frequency)
{
  uint8_t prescale;
  if(frequency >= 1526) prescale = 0x03;
  else if(frequency <= 24) prescale = 0xFF;
  // internal 25 MHz oscillator
  else prescale = 25000000 / (4096 * frequency);
  // prescale changes 3 to 255 for 1526Hz to 24Hz
  setBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
  HAL_I2C_Mem_Write(&hi2c3, PCA9685_I2C_ADDRESS, PCA9685_PRESCALE, 1, &prescale, 1, HAL_MAX_DELAY);
  setBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
  setBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}


/**
 * @brief  Initializes PWM Driver
 * @retval void
 */ 
void servo_setup(uint16_t frequency)
{
    HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c3, PCA9685_I2C_ADDRESS<<1, 1, HAL_MAX_DELAY);

	if(ret == HAL_OK){
		strcpy((char*)buffer, "DEVICE STATUS:\tREADY\r\n");
	}else{
		strcpy((char*)buffer, "DEVICE STATUS:\tNOT READY\r\n");
	}
    HAL_UART_Transmit(&huart1, buffer, strlen((char*)buffer), HAL_MAX_DELAY);

    setPWMFreq(frequency);
    setBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
    strcpy((char*)buffer, "Initialization complete\n");
    HAL_UART_Transmit(&huart1, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}


/**
 * @brief Set PWM Duty Cycle by controlling the on time and off time
 * 
 * @param Channel 
 * @param OnTime 
 * @param OffTime 
 */
void setServoPWM(uint16_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress;
    uint8_t PWM[4];
    registerAddress = PCA9685_LED0_ON_L + (4 * Channel);

    PWM[0] = OnTime & 0xFF;
    PWM[1] = OnTime>>8;
    PWM[2] = OffTime & 0xFF;
    PWM[3] = OffTime>>8;

    HAL_I2C_Mem_Write(&hi2c3, PCA9685_I2C_ADDRESS, registerAddress, 1, PWM, 4, HAL_MAX_DELAY);
}


/**
 * @brief Set the Angle of Servo [deg]
 * 
 * @param Channel 
 * @param Angle 
 */
void setServoAngle(uint8_t Channel, float Angle)
{
    float Value;

    // 12 bit resolution @PWM frequency 50Hz == 20ms Period
    Value = 4095 * (((Angle/180) + 1)/20);
    setServoPWM(Channel, 0, (uint16_t)Value);
}

void ServoEaseMultiple(ServoEase servos[], uint8_t numServos)
{
    uint32_t globalStartTime = HAL_GetTick();
    uint32_t elapsedTime = 0;

    // Set individual start times
    for (uint8_t i = 0; i < numServos; i++) {
        servos[i].startTime = globalStartTime;
    }

    while (elapsedTime <= servos[0].duration) {
        elapsedTime = HAL_GetTick() - globalStartTime;

        for (uint8_t i = 0; i < numServos; i++) {
            float t = (float)elapsedTime;
            float duration = servos[i].duration;

            // Ensure the easing function only runs within duration limits
            if (t <= duration) {
                float easedAngle = ServoEaseTo(servos[i].startAngle, servos[i].endAngle, duration, t);
                setServoAngle(servos[i].channel, easedAngle);
            } else {
                setServoAngle(servos[i].channel, servos[i].endAngle); // Ensure it reaches final position
            }
        }

        HAL_Delay(10); // Smooth movement update
    }
}

float ServoEaseTo(float theta_s, float theta_e, float tf, float t) 
{
    // Cubic polynomial coefficients
    float a0 = theta_s;
    float a1 = 0;
    float a2 = 3 / (tf * tf) * (theta_e - theta_s);
    float a3 = -2 / (tf * tf * tf) * (theta_e - theta_s);

    // Ensure `t` does not exceed `tf`
    if (t > tf) t = tf;

    // Compute eased angle
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
}
