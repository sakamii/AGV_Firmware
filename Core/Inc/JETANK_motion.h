#ifndef __JETANK_MOTION_H__
#define __JETANK_MOTION_H__

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>


#define PCA9685_ADDR 0xC0
#define PWMA_REG_ADDR 0x26 		//LED8 , start at LED_ON_L
#define PWMB_REG_ADDR 0x3A 		//LED13
#define AIN1_REG_ADDR 0x2E		//LED10
#define AIN2_REG_ADDR 0x2A	 	//LED9
#define BIN1_REG_ADDR 0x32		//LED11
#define BIN2_REG_ADDR 0x36		//LED12
#define MODE1_REG_ADDR 0x00		//SLEEP_BIT : 4
#define MODE2_REG_ADDR 0x01
#define PRESCALER_REG_ADDR 0xFE
#define SLEEP_BIT 4
#define ALL_LED_ON_L 0xFA

#define _MOTOR_A 0xA
#define _MOTOR_B 0xB
#define _RIGHT_MOTOR _MOTOR_B
#define _LEFT_MOTOR _MOTOR_A
#define CW 1 // back
#define CCW 2 //go
#define _SET 1
#define _CLEAR 0
#define _MOTOR_STOP_BOUND 20

#define HIGH_LEVEL 1
#define LOW_LEVEL 0

bool pca9685_init(I2C_HandleTypeDef* hi2c);
void pin_state(I2C_HandleTypeDef* hi2c, const uint8_t PIN_ADDR, const int _STATE);
void set_motor_dir(I2C_HandleTypeDef* hi2c, const int _MOTOR, int _DIR);
bool set_motor_power(I2C_HandleTypeDef* hi2c, const int _MOTOR, double power);
void turnRight(I2C_HandleTypeDef* hi2c, double diff);
void turnLeft(I2C_HandleTypeDef* hi2c, double diff);
void stop(I2C_HandleTypeDef* hi2c);
void move_go(I2C_HandleTypeDef* hi2c, double power, double diff);
void move_back(I2C_HandleTypeDef* hi2c, double power, double diff);
void channel_to_motion(I2C_HandleTypeDef* hi2c, uint32_t ch1, uint32_t ch2);
double map(double x, double in_min, double in_max, double out_min, double out_max);


#endif
