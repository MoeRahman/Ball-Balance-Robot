/*
 * pca9685_servo_driver.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Muhtasim Rahman
 */ 

#include <stdint.h>

#ifndef INC_PCA9685_SERVO_DRIVER_H_
#define INC_PCA9685_SERVO_DRIVER_H_

#ifdef __cplusplus
extern "C"{
#endif

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x80      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

/**
 * @brief Structure to define servo movement parameters.
 * 
 * This struct holds the necessary parameters for easing a servo
 * from a start angle to an end angle over a given duration.
 */
typedef struct {
    uint8_t channel;       /** PCA9685 Channel the servo is connected to */
    float startAngle;      /** Starting angle of the servo [degrees] */
    float endAngle;        /** Target angle of the servo [degrees] */
    float duration;        /** Time to complete movement [milliseconds] */
    uint32_t startTime;    /** Time when movement started [milliseconds] */
} ServoEase;

// Setup driver
void servo_setup(uint16_t frequency);

// Update & Read PWM Frequency
void setPWMFreq(uint16_t frequncy);

// Update  PWM signal
void setServoPWM(uint16_t channel, uint16_t onTime, uint16_t offTime);

// Update servo angle
void setServoAngle(uint8_t channel, float angle);

// Using interpolation to smooth servo movements
float ServoEaseTo(float theta_s, float theta_e, float tf, float t);

// Control Multiple Servo in parallel
void ServoEaseMultiple(ServoEase servos[], uint8_t numServos);

#ifdef __cplusplus
}
#endif

#endif /* INC_PCA9685_SERVO_DRIVER_H_ */
