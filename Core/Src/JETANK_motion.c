#include "JETANK_motion.h"

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool pca9685_init(I2C_HandleTypeDef* hi2c){
	assert(hi2c != NULL);
	//hi2c_handle = hi2c;
	bool success = true;
	// Set mode registers to default values (Auto-Increment, Sleep, Open-Drain).
	// totem pole
	uint8_t mode1_reg_default_value = 0b00110000u;
	uint8_t mode2_reg_default_value = 0b00000100u;

	// frequency : 1kHz
	//double frequency = 1000;
	//uint8_t prescaler_defalut_value = (uint8_t)roundf(25000000.0f / (4096 * frequency)) - 1;
	//uint8_t prescaler_defalut_value = 0;// f = max
	uint8_t prescaler_defalut_value = 121; // f = 50Hz

	uint8_t transmit1[] = {MODE1_REG_ADDR, mode1_reg_default_value};
	success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, transmit1, 2, 10) == HAL_OK);

	uint8_t transmit2[] = {MODE2_REG_ADDR, mode2_reg_default_value};
	success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, transmit2, 2, 10) == HAL_OK);

	uint8_t transmit3[] = {PRESCALER_REG_ADDR, prescaler_defalut_value};
	success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, transmit3, 2, 10) == HAL_OK);

	mode1_reg_default_value &= ~(0x01 << SLEEP_BIT);
	uint8_t transmit4[] = {MODE1_REG_ADDR, mode1_reg_default_value};
	success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, transmit4, 2, 10) == HAL_OK);


	HAL_Delay(10);
	//all LED pin off
	uint8_t data[] = {ALL_LED_ON_L, 0x00, 0x00, 0x00, 0x10};
	success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, data, 5, 10) == HAL_OK);

	return success;
}

void pin_state(I2C_HandleTypeDef* hi2c, const uint8_t PIN_ADDR, const int _STATE){
	uint8_t on_data[] = {PIN_ADDR, 0x00, 0x10, 0x00, 0x00};
	uint8_t off_data[] = {PIN_ADDR, 0x00, 0x00, 0x00, 0x10};

	if(_STATE == _SET){
		if(HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, on_data, 5, 100) != HAL_OK){
			printf("transmit error1 \r\n");
		}
	}
	else if(_STATE == _CLEAR){
		if(HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, off_data, 5, 100) != HAL_OK){
			printf("transmit error3 \r\n");
		}
	}
}

void set_motor_dir(I2C_HandleTypeDef* hi2c, const int _MOTOR, int _DIR){
	if(_MOTOR == _MOTOR_A){
		if(_DIR == CW){//AIN1_REG_ADDR : 1 , AIN2_REG_ADDR : 0
			pin_state(hi2c, AIN1_REG_ADDR, _SET);
			pin_state(hi2c, AIN2_REG_ADDR, _CLEAR);
		}
		else if(_DIR == CCW){ //AIN1_REG_ADDR : 0 , AIN2_REG_ADDR : 1
			pin_state(hi2c, AIN1_REG_ADDR, _CLEAR);
			pin_state(hi2c, AIN2_REG_ADDR, _SET);
		}

	}
	if(_MOTOR == _MOTOR_B){
		if(_DIR == CW){//BIN1_REG_ADDR : 1 , BIN2_REG_ADDR : 0
			pin_state(hi2c, BIN1_REG_ADDR, _SET);
			pin_state(hi2c, BIN2_REG_ADDR, _CLEAR);
		}
		else if(_DIR == CCW){ //BIN1_REG_ADDR : 0 , BIN2_REG_ADDR : 1
			pin_state(hi2c, BIN1_REG_ADDR, _CLEAR);
			pin_state(hi2c, BIN2_REG_ADDR, _SET);
		}
	}
}

bool set_motor_power(I2C_HandleTypeDef* hi2c, const int _MOTOR, double power){
	if(power > 100 || power < 0 ) return false;
	bool success = true;

	//power: 0 ~ 100 => duty ratio 0 ~ 20% (0~819)
	uint16_t duty = (uint16_t)round((power/100.0f)*4095);
	uint8_t duty_L = duty & (uint8_t)0xFF;
	uint8_t duty_H = duty >> 8;

	if(_MOTOR == _MOTOR_A){ // PWMA_REG_ADDR
		uint16_t on_time = 0;
		uint16_t off_time = on_time + duty;
		uint8_t data[] = {PWMA_REG_ADDR, on_time&0xFF, (on_time>>8) , off_time&0xFF, (off_time>>8)};
		success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, data, 5, 10) == HAL_OK);
	}

	if(_MOTOR == _MOTOR_B){ // PWMB_REG_ADDR
		uint16_t on_time = 0;
		uint16_t off_time = on_time + duty;
		uint8_t data[] = {PWMB_REG_ADDR, on_time&0xFF, (on_time>>8) , off_time&0xFF, (off_time>>8)};
		success &= (HAL_I2C_Master_Transmit(hi2c, PCA9685_ADDR, data, 5, 10) == HAL_OK);
	}

	return success;
}



void turnRight(I2C_HandleTypeDef* hi2c, double diff){
	double power = abs(diff);
	set_motor_dir(hi2c, _RIGHT_MOTOR, CW);
	set_motor_dir(hi2c, _LEFT_MOTOR, CCW);
	set_motor_power(hi2c, _RIGHT_MOTOR, power);
	set_motor_power(hi2c, _LEFT_MOTOR, power);
}
void turnLeft(I2C_HandleTypeDef* hi2c, double diff){
	double power = abs(diff);
	set_motor_dir(hi2c, _RIGHT_MOTOR, CCW);
	set_motor_dir(hi2c, _LEFT_MOTOR, CW);
	set_motor_power(hi2c, _RIGHT_MOTOR, power);
	set_motor_power(hi2c, _LEFT_MOTOR, power);
}
void stop(I2C_HandleTypeDef* hi2c){
	set_motor_power(hi2c, _RIGHT_MOTOR, 0);
	set_motor_power(hi2c, _LEFT_MOTOR, 0);
}

void move_go(I2C_HandleTypeDef* hi2c, double power, double diff){
	double right_power = power - (diff * 0.2f);
	double left_power = power + (diff * 0.2f);

	if(right_power > 100) right_power = 100;
	if(right_power < 0) right_power = 0;
	if(left_power > 100) left_power = 100;
	if(left_power < 0) left_power = 0;

	set_motor_dir(hi2c, _RIGHT_MOTOR, CCW);
	set_motor_dir(hi2c, _LEFT_MOTOR, CCW);
	set_motor_power(hi2c, _RIGHT_MOTOR, right_power);
	set_motor_power(hi2c, _LEFT_MOTOR, left_power);
}

void move_back(I2C_HandleTypeDef* hi2c, double power, double diff){
	double right_power = power - (diff * 0.2f);
	double left_power = power + (diff * 0.2f);

	if(right_power > 100) right_power = 100;
	if(right_power < 0) right_power = 0;
	if(left_power > 100) left_power = 100;
	if(left_power < 0) left_power = 0;

	set_motor_dir(hi2c, _RIGHT_MOTOR, CW);
	set_motor_dir(hi2c, _LEFT_MOTOR, CW);
	set_motor_power(hi2c, _RIGHT_MOTOR, right_power);
	set_motor_power(hi2c, _LEFT_MOTOR, left_power);
}

void channel_to_motion(I2C_HandleTypeDef* hi2c, uint32_t ch1, uint32_t ch2){
	//channel exception handling
	if(ch1 > 1600) ch1 = 1600;
	else if(ch1 < 600) ch1 = 600;
	if(ch2 > 1600) ch2 = 1600;
	else if(ch2 < 600) ch2 = 600;

	//channel to power range : 600us ~ 1600us  => -100 ~ 100
	//channel to diff range : 500us ~ 1600us => 0 ~ 100
	double power = map((double)ch2, 600, 1600, -100, 100);
	double diff = map((double)ch1, 600, 1600, -100, 100);

	//stop state
	if(power < _MOTOR_STOP_BOUND && power > -_MOTOR_STOP_BOUND
			&& diff < _MOTOR_STOP_BOUND && diff > -_MOTOR_STOP_BOUND){
		stop(hi2c);
		return;
	}

	// rotation state
	if(power < _MOTOR_STOP_BOUND && power > -_MOTOR_STOP_BOUND ){
		if(diff > _MOTOR_STOP_BOUND){
			turnRight(hi2c, diff);
		}
		else if(diff < -_MOTOR_STOP_BOUND){
			turnLeft(hi2c, diff);
		}

		return;
	}

	//go or back
	if(power > 0) move_go(hi2c, power, diff);
	else if(power < 0) move_back(hi2c, abs(power), diff);

}
