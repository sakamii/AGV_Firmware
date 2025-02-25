#ifndef __BATTERY_STATE_H__
#define __BATTERY_STATE_H__

#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define INA219_DEVICE_ADDR 0x41 << 1
#define CONFIG_REG_ADDR 0x00
#define SHUNT_VOLTAGE_REG_ADDR 0x01
#define BUS_VOLTAGE_REG_ADDR 0x02
#define POWER_REG_ADDR 0x03
#define CURRENT_REG_ADDR 0x04
#define CAL_REG_ADDR 0x05
#define TO_MICRO 0.000001
#define TO_MILI 0.001


typedef struct ina219{
	I2C_HandleTypeDef* hi2c;
	double expected_max_current;
	double shunt_resistor;
	double Current_LSB;
	double Power_LSB;
	uint16_t Cali_Register;
}ina219_handle_t;

typedef struct display{
	GPIO_TypeDef* LED1_GPIO;
	uint16_t LED1_pin;
	GPIO_TypeDef* LED2_GPIO;
	uint16_t LED2_pin;
	GPIO_TypeDef* LED3_GPIO;
	uint16_t LED3_pin;
}LED_handler_t;

bool INA219_init(ina219_handle_t* handle);
double get_bus_voltage(ina219_handle_t* handle);
double get_shunt_voltage(ina219_handle_t* handle);
double get_current(ina219_handle_t* handle);
void charge_display(double voltage, LED_handler_t* LED_handler);



#endif
