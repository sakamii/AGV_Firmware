#ifndef __MANIPULATOR_H__
#define __MANIPULATOR_H__

#include "stm32f1xx_hal.h"

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
//Instruction for DXL Protocol
#define INST_PING 		 1
#define INST_READ 		 2
#define INST_WRITE 		 3
#define INST_REG_WRITE 	 4
#define INST_ACTION 	 5
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_READ 	0x82

#define POSITION_REG_ADDR_H 0x2A

#define DEFALUT_RUNNING_TIME 0x0000
#define DEFAULT_RUNNING_SPEED 0x03E8 //1000

#define MIN_DEGREE 0
#define MAX_DEGREE 200
#define MIN_POSITION 0
#define MAX_POSITION 1023


double map(double x, double in_min, double in_max, double out_min, double out_max);
void servo_posintion_write(UART_HandleTypeDef* huart, uint8_t ID, double degree);
void channel_to_manipulator(UART_HandleTypeDef* huart, uint32_t ch3, uint32_t ch4, uint32_t ch5);
void manipulator_position_init(UART_HandleTypeDef* huart);
void set_manipulator_vertical_motion(UART_HandleTypeDef* huart, uint32_t motion);
void set_manipulator_gripper(UART_HandleTypeDef* huart, double degree);
void set_manipulator_horizon_motion(UART_HandleTypeDef* huart, double degree);

#endif
