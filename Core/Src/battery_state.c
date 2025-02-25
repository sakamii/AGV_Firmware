#include "battery_state.h"

bool INA219_init(ina219_handle_t* handle){
	bool success = true;

	handle->Current_LSB = handle->expected_max_current /32768; //2^15 = 32,768
	handle->Power_LSB = 20 * handle->Current_LSB;
	handle->Cali_Register = trunc(0.04096 / (handle->Current_LSB * handle->shunt_resistor));
	//handle->Cali_Register = 0;

	uint16_t data = handle->Cali_Register << 1;
	uint8_t data_H = data<<8;
	uint8_t data_L = data&0xFF;

	uint8_t transmit[] = {CAL_REG_ADDR, data_H, data_L};
	success &= (HAL_I2C_Master_Transmit(handle->hi2c, INA219_DEVICE_ADDR , transmit, sizeof(transmit), 10) == HAL_OK);

	printf("cur_LSB : %f \r\n", handle->Current_LSB);

	return success;
}

double get_bus_voltage(ina219_handle_t* handle){
	uint8_t target_reg = BUS_VOLTAGE_REG_ADDR;
	if(HAL_I2C_Master_Transmit(handle->hi2c, INA219_DEVICE_ADDR , &target_reg, 1, 10) != HAL_OK){
		printf("write error\r\n");
	}

	uint8_t receive_data[2] = {0, };
	if(HAL_I2C_Master_Receive(handle->hi2c, INA219_DEVICE_ADDR , receive_data, sizeof(receive_data), 10) != HAL_OK){
		printf("receive error\r\n");
	}

	uint16_t data = (receive_data[0]<<8) + (receive_data[1]&0xFF);
	data = data >> 3;

	double ret = (double)data * 4 * TO_MILI;


	return ret;
}

double get_shunt_voltage(ina219_handle_t* handle){
	uint8_t target_reg = SHUNT_VOLTAGE_REG_ADDR;
	if(HAL_I2C_Master_Transmit(handle->hi2c, INA219_DEVICE_ADDR , &target_reg, 1, 10) != HAL_OK){
		printf("write error\r\n");
	}

	uint8_t receive_data[2] = {0, };
	if(HAL_I2C_Master_Receive(handle->hi2c, INA219_DEVICE_ADDR , receive_data, sizeof(receive_data), 10) != HAL_OK){
		printf("receive error\r\n");
	}
	uint16_t data = (receive_data[0]<<8) + (receive_data[1]&0xFF);
	double ret = (double)data * 10 * TO_MICRO;

	return ret;
}

double get_current(ina219_handle_t* handle){
	uint8_t target_reg = CURRENT_REG_ADDR;
	if(HAL_I2C_Master_Transmit(handle->hi2c, INA219_DEVICE_ADDR , &target_reg, 1, 10) != HAL_OK){
		printf("write error\r\n");
	}

	uint8_t receive_data[2] = {0, };
	if(HAL_I2C_Master_Receive(handle->hi2c, INA219_DEVICE_ADDR , receive_data, sizeof(receive_data), 10) != HAL_OK){
		printf("receive error\r\n");
	}

	uint16_t data = ((uint16_t)receive_data[0]<<8) + ((uint16_t)receive_data[1]&0xFF);

	printf("%d %d\r\n",receive_data[0], receive_data[1]);

	double ret = (double)data * (handle->Current_LSB);


	return ret;
}


void charge_display(double voltage, LED_handler_t* LED_handler){
	if(voltage <= 10.0f){
		HAL_GPIO_WritePin(LED_handler->LED1_GPIO,LED_handler->LED1_pin, 0);
		HAL_GPIO_WritePin(LED_handler->LED2_GPIO,LED_handler->LED2_pin, 0);
		HAL_GPIO_WritePin(LED_handler->LED3_GPIO,LED_handler->LED3_pin, 0);
	}
	else if(voltage > 10.0f && voltage <= 10.7f){
		HAL_GPIO_WritePin(LED_handler->LED1_GPIO,LED_handler->LED1_pin, 1);
		HAL_GPIO_WritePin(LED_handler->LED2_GPIO,LED_handler->LED2_pin, 0);
		HAL_GPIO_WritePin(LED_handler->LED3_GPIO,LED_handler->LED3_pin, 0);
	}
	else if(voltage > 10.7f && voltage < 11.5f){
		HAL_GPIO_WritePin(LED_handler->LED1_GPIO,LED_handler->LED1_pin, 1);
		HAL_GPIO_WritePin(LED_handler->LED2_GPIO,LED_handler->LED2_pin, 1);
		HAL_GPIO_WritePin(LED_handler->LED3_GPIO,LED_handler->LED3_pin, 0);
	}
	else if(voltage >= 11.5f){
		HAL_GPIO_WritePin(LED_handler->LED1_GPIO,LED_handler->LED1_pin, 1);
		HAL_GPIO_WritePin(LED_handler->LED2_GPIO,LED_handler->LED2_pin, 1);
		HAL_GPIO_WritePin(LED_handler->LED3_GPIO,LED_handler->LED3_pin, 1);
	}
}
