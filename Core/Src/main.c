/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "JETANK_motion.h"
#include "battery_state.h"
#include "manipulator.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

////////// MODE SETTING /////////
#define RC_TANK_CONTROL_ON 			1
#define MANIPULATOR_CONTROL_ON 		1
#define BATTERY_STATE_DISPLAY_ON 	1

/////////////////////////////////







/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define TIMER_MAX 65535
#define RTSR_OFFSET 0x08
#define FTSR_OFFSET 0x0C


typedef struct channel{
    int ch1;
    int ch2;
    int ch3;
    int ch4;
    int ch5;
    int ch6;
    int ch7;
    int ch8;
    int dummy;
}_channel;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Create the handle for the driver.

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _read(int file, char *ptr, int len);
int _write(int file, char *ptr, int len);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


_channel channel_data = {0, };
uint32_t t1 = 0;
uint32_t t2 = 0;
uint32_t time_interval = 0;
uint32_t rising_cnt = 0;
uint32_t falling_cnt = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

    if (GPIO_Pin == PPM_Pin){

        if(( EXTI->RTSR >> 4) & 0x01){
            t1= TIM2->CNT;

            rising_cnt++;
            EXTI->RTSR &= ~((uint32_t)0x01 << 4);
            EXTI->FTSR |= ((uint32_t)0x01 << 4);
        }
        else if(( EXTI->FTSR >> 4) & 0x01){
            t2 = TIM2->CNT;

            if( t1 > t2) time_interval = TIMER_MAX - t1 + t2;
            else time_interval = t2-t1;

            falling_cnt++;
            EXTI->RTSR |= ((uint32_t)0x01 << 4);
            EXTI->FTSR &= ~((uint32_t)0x01 << 4);
        }

        if(time_interval > 4000){
            rising_cnt = 9;
            falling_cnt = 9;
        }
        switch(rising_cnt + falling_cnt){
        case 2:
            channel_data.ch1 = time_interval;
            break;
        case 4:
            channel_data.ch2 = time_interval;
            break;
        case 6:
            channel_data.ch3 = time_interval;
            break;
        case 8:
            channel_data.ch4 = time_interval;
            break;
        case 10:
            channel_data.ch5 = time_interval;
            break;
        case 12:
            channel_data.ch6 = time_interval;
            break;
        case 14:
            channel_data.ch7 = time_interval;
            break;
        case 16:
            channel_data.ch8 = time_interval;
            break;
        case 18:
            channel_data.dummy = time_interval;
            rising_cnt = 0;
            falling_cnt = 0;
            break;
        default:
            //Nothing to do
            break;
        }
    }
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  ina219_handle_t ina219_handler = {
		  .hi2c = &hi2c1,
		  .expected_max_current = 3.2,
		  .shunt_resistor = 0.1
  };


  LED_handler_t LED_handler ={
		  .LED1_GPIO = LED1_GPIO_Port,
		  .LED2_GPIO = LED2_GPIO_Port,
		  .LED3_GPIO = LED3_GPIO_Port,
		  .LED1_pin = LED1_Pin,
		  .LED2_pin = LED2_Pin,
		  .LED3_pin = LED3_Pin
  };

  if(INA219_init(&ina219_handler)){
	  printf("ina219 init success\r\n");
  }
  else{
	  printf("ina219 init error\r\n");
  }

  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(40);

  if (pca9685_init(&hi2c1)){
	  printf("pca9685 init success\r\n");
  }
  else{
	  printf("pca9685 init error\r\n");
  }

  manipulator_position_init(&huart3);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if RC_TANK_CONTROL_ON
	  channel_to_motion(&hi2c1, channel_data.ch1, channel_data.ch2);
#endif

#if MANIPULATOR_CONTROL_ON

	  channel_to_manipulator(&huart3, channel_data.ch3, channel_data.ch4, channel_data.ch5);

	  HAL_Delay(100);
#endif


#if BATTERY_STATE_DISPLAY_ON
	  double vol = get_bus_voltage(&ina219_handler);
	  charge_display(vol, &LED_handler);
#endif





  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _read(int file, char *ptr, int len){
	HAL_UART_Receive(&huart2, (uint8_t*)ptr, 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, 1, 100);
	return 1;
}

int _write(int file, char *ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
	return len;
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
