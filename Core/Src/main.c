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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc_sr04.h"
#include <stdlib.h>
#include "string.h"
#include "stdio.h"
#include "servo.h"
#include "pid_controller.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct us_sensor_str distance_sensor;
volatile int d = 0;
pid_str pid;
float kp = 5.2f;
float ki = 2.0f;
float kd = 12.0f;
float fs = 16;
float anti_windup_limit = 10.0f;
float y_ref = 20.0f;
float previous_distance = 5.0f;
int counter = 0;
float mean_distance = 0.0f;

uint8_t tx_buffer[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void read_distance(TIM_HandleTypeDef *htim){
	uint32_t echo_us;

	echo_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	distance_sensor.distance_cm = hc_sr04_convert_us_to_cm(echo_us);
	if (distance_sensor.distance_cm > 40 || distance_sensor.distance_cm < 4){
		distance_sensor.distance_cm = previous_distance;
			}
	previous_distance = distance_sensor.distance_cm;
}

void generate_control(){
	int u = pid_calculate(&pid, y_ref, distance_sensor.distance_cm);
	servo_set_angle(u);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM4 == htim->Instance)
	{
		read_distance(htim);
		generate_control();

		d = distance_sensor.distance_cm * 10;


		uint8_t tx_buffer[32];
		int tx_msg_len = sprintf((char*)tx_buffer, "%02u.%01u\r", d / 10, d % 10);
		HAL_UART_Transmit(&huart3, tx_buffer, tx_msg_len, 100);

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
      {

    	int value = strtol((char *)&tx_buffer[2], NULL, 10);

    	if (strncmp((char *)tx_buffer, "kp", 2) == 0){
    		kp = value;
    	}
    	if (strncmp((char *)tx_buffer, "ki", 2) == 0){
    	    		ki = value;
    	    	}
    	if (strncmp((char *)tx_buffer, "kd", 2) == 0){
    	    		kd = value;
    	    	}
    	if (strncmp((char *)tx_buffer, "yr", 2) == 0){
    		if (value > 5 && value < 25)
    	    		y_ref = value;
    	    	}



    }
    HAL_UART_Receive_IT(&huart3, tx_buffer, 4);



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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  hc_sr04_init(&distance_sensor, &htim4, &htim2, TIM_CHANNEL_4);
  servo_init(&htim1, TIM_CHANNEL_1);
  pid_init(&pid, kp, ki, kd, fs, anti_windup_limit);
  int a = -90;
  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//servo_set_angle(a);
	//HAL_Delay(20);
	//a += 1;
	//if(a == 90){
		//	a = -90;
	//}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
