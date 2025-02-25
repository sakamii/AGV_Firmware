#include "manipulator.h"


void channel_to_manipulator(UART_HandleTypeDef* huart, uint32_t ch3, uint32_t ch4, uint32_t ch5){
	//ch4 -> horizon
	//ch3 -> vertical
	//ch5 -> grip
	uint32_t motion = (uint32_t)map(ch3, 600, 1600, 0, 100);
	set_manipulator_vertical_motion(huart, motion);

	double horizon_deg = map(ch4, 600, 1600, 0, 180);
	set_manipulator_horizon_motion(huart, horizon_deg);

	double grip_deg = map(ch5, 600, 1600, 0, 180);
	set_manipulator_gripper(huart,grip_deg);
}

void manipulator_position_init(UART_HandleTypeDef* huart){
	set_manipulator_horizon_motion(huart, 90);
	set_manipulator_gripper(huart, 0);

	servo_posintion_write(huart, MOTOR2, 30);
	servo_posintion_write(huart, MOTOR3, 30);

}

void set_manipulator_vertical_motion(UART_HandleTypeDef* huart, uint32_t motion){
	//motion : 0~100

	//초기값 :motor2: 30도, motor3 : 30도?
	//motion : 0~50
	//1단계 : motor2: 130도 , motor3: 130도?
	if(motion >= 0 && motion < 50){
		double deg = map((double)motion, 0, 50, 30, 130);
		servo_posintion_write(huart, MOTOR2, deg);
		servo_posintion_write(huart, MOTOR3, deg);
		return;
	}

	//motion : 51~80
	//2단계 : motor2만 180도까지 증가
	if(motion >= 50 && motion < 80){
		double deg = map((double)motion, 50,80, 130, 180);
		servo_posintion_write(huart, MOTOR2, deg);
		servo_posintion_write(huart, MOTOR3, 130);
		return;
	}


	//motion : 80~100
	//3단계 : motor3만 130도에서 80도까지 감소
	if(motion >= 80 && motion <=100){
		double deg = map((double)motion, 80,100, 130, 80);
		servo_posintion_write(huart, MOTOR2, 180);
		servo_posintion_write(huart, MOTOR3, deg);
		return;
	}

}

void set_manipulator_gripper(UART_HandleTypeDef* huart, double degree){
	servo_posintion_write(huart,MOTOR4, degree);
}

void set_manipulator_horizon_motion(UART_HandleTypeDef* huart, double degree){
	servo_posintion_write(huart, MOTOR1, degree);
}

/////////////////////////////////////




void servo_posintion_write(UART_HandleTypeDef* huart, uint8_t ID, double degree){
	//position range : 0~1023 =? 0 degree ~ 200 degree
	if(degree < 0 || degree > 200) return;

	uint16_t pos = (uint16_t)map(degree, MIN_DEGREE, MAX_DEGREE, MIN_POSITION, MAX_POSITION);

	const uint8_t LEN = 0x09;

	uint16_t time = DEFALUT_RUNNING_TIME;
	uint16_t speed = DEFAULT_RUNNING_SPEED;

	uint8_t time_L = time&0xFF;
	uint8_t time_H = time>>8;
	uint8_t speed_L = speed&0xFF;
	uint8_t speed_H = speed>>8;

	uint8_t pos_L = pos&0xFF;
	uint8_t pos_H = pos>>8;

	uint8_t tmp = ID + LEN + INST_WRITE + POSITION_REG_ADDR_H + pos_L + pos_H + time_L + time_H + speed_L + speed_H;
	uint8_t check_sum = ~tmp;

	uint8_t transmit_data[] = {0xFF, 0xFF, ID, LEN, INST_WRITE, POSITION_REG_ADDR_H, pos_H, pos_L, time_H, time_L, speed_H, speed_L, check_sum};
	uint8_t receive_data[] = {0x99, 0x99,0x99,0x99, 0x99, 0x99};


	HAL_HalfDuplex_EnableTransmitter(huart);
	if(HAL_UART_Transmit(huart, transmit_data, sizeof(transmit_data), 10) != HAL_OK){
		printf("uart transmit error\r\n");
	}
	HAL_HalfDuplex_EnableReceiver(huart);
	if(HAL_UART_Receive(huart, receive_data, sizeof(receive_data), 10) != HAL_OK){
		//printf("uart receive error\r\n");
	}
	else{
		//printf("receive_data : %d\r\n", receive_data[4]);
	}
}
