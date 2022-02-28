/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <Library/AMT21.h>
#include <Library/ARMsProtocol.h>
#include <Library/Trajectory.h>
#include <Library/KalmanFilter.h>
#include <Library/Kinematic.h>
#include <Library/Motor.h>
#include <Library/PID.h>
#include "stdio.h"
#include "math.h"
#include "stm32h7xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KALMAN_Q 10
#define KALMAN_R 0.01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * Struct
 */
AMT21 encoders[5];
Stepper_Motor steppers[5];
Servo_Motor servo_motor;
PIDController position_pid_controller[5];
PIDController velocity_pid_controller[5];
KalmanFilter kalman_filter[5];
QuinticTrajectory quintic_trajectory[5];
/*
 * ARMsProtocol Var
 */
extern int32_t encoder_config[5];
extern double delta_khe[5];
extern double desired_position[5];
extern double max_desired_position[5];
extern double min_desired_position[5];
extern ARMsProtocol_HandleTypedef ARMsProtocol_h1;
extern ARMsProtocol_DATA ARMsProtocol_Data;

extern uint8_t servo_flag;
extern double servo_degree;
/*
 * Main Var
 */
double cascade_out[5] = { 0 };
double motor_vel[5] = { 0 };
double desired_velocity[5] = { 0 };
double motor_config[5] = { 0 };
double prev_motor_config[5] = { 0 };
double desired_motor_position[5] = { 0 };
double checking_value = 0.0;

double delta_khe[5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM23_Init();
  MX_CRC_Init();
  MX_TIM24_Init();
  /* USER CODE BEGIN 2 */
	ARMsProtocol_FUNC_Init();

	/*
	 * Servo Initialise
	 */
	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, 1);
	servo_initialise(&servo_motor, &htim17, TIM_CHANNEL_1);
	/*
	 * Encoder Initialise
	 */
	AMT21_initialise(&encoders[0], &huart2, 0x2C, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[1], &huart2, 0x70, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[2], &huart2, 0x54, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[4], &huart2, 0xE8, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	AMT21_initialise(&encoders[3], &huart2, 0xB4, USART2_DE_GPIO_Port,
	USART2_DE_Pin);
	/*
	 * Stepper Initialise
	 */
	stepper_initialise(&steppers[0], &htim1, TIM_CHANNEL_1, DIR1_GPIO_Port,
	DIR1_Pin, 1); //swap 0 -> 1
	stepper_initialise(&steppers[1], &htim2, TIM_CHANNEL_1, DIR2_GPIO_Port,
	DIR2_Pin, 1);
	stepper_initialise(&steppers[2], &htim3, TIM_CHANNEL_1, DIR3_GPIO_Port,
	DIR3_Pin, 0); //swap 1 -> 0
	stepper_initialise(&steppers[4], &htim4, TIM_CHANNEL_1, DIR4_GPIO_Port,
	DIR4_Pin, 0);
	stepper_initialise(&steppers[3], &htim15, TIM_CHANNEL_1, DIR5_GPIO_Port,
	DIR5_Pin, 1);
	/*
	 * Kalman Filter Initialise
	 */
	KalmanFilter_initialise(&kalman_filter[0], 0, 0, 1, 0, 0, 1, KALMAN_R,
	KALMAN_Q);
	KalmanFilter_initialise(&kalman_filter[1], 0, 0, 1, 0, 0, 1, KALMAN_R,
	KALMAN_Q);
	KalmanFilter_initialise(&kalman_filter[2], 0, 0, 1, 0, 0, 1, KALMAN_R,
	KALMAN_Q);
	KalmanFilter_initialise(&kalman_filter[3], 0, 0, 1, 0, 0, 1, KALMAN_R,
	KALMAN_Q);
	KalmanFilter_initialise(&kalman_filter[4], 0, 0, 1, 0, 0, 1, KALMAN_R,
	KALMAN_Q);
	/*
	 * Position Pid Initialise
	 */
	/*
	 * Feed-Forward Control
	 */
//	PIDController_initialise(&position_pid_controller[0], 40, 0, 0);
////	PIDController_initialise(&position_pid_controller[1], 42.5, 0, 5);
//	PIDController_initialise(&position_pid_controller[1], 40, 0, 5);
//	PIDController_initialise(&position_pid_controller[2], 20, 0, 5);
//	PIDController_initialise(&position_pid_controller[3], 40, 0, 0);
//	PIDController_initialise(&position_pid_controller[4], 40, 0, 0);
	/*
	 * Cascade Control
	 */
	PIDController_initialise(&position_pid_controller[0], 100, 0, 2.5);
	PIDController_initialise(&position_pid_controller[1], 40, 0, 2.5);
	PIDController_initialise(&position_pid_controller[2], 40, 0, 0);
	PIDController_initialise(&position_pid_controller[3], 40, 0, 2.5);
	PIDController_initialise(&position_pid_controller[4], 40, 0, 2.5);
	/*
	 * Velocity Pid Initialise
	 */
	/*
	 * Feed-Forward Control
	 */
//	PIDController_initialise(&velocity_pid_controller[0], 300, 0, 0); // 0.05
//	PIDController_initialise(&velocity_pid_controller[1], 100, 0, 0); // 0.001
//	PIDController_initialise(&velocity_pid_controller[2], 250, 0, 0); // 0.1
//	PIDController_initialise(&velocity_pid_controller[3], 75, 0.005, 0);
//	PIDController_initialise(&velocity_pid_controller[4], 75, 0.005, 0);
	/*
	 * Cascade Control
	 */
	PIDController_initialise(&velocity_pid_controller[0], 20, 0, 0); //5
	PIDController_initialise(&velocity_pid_controller[1], 20, 0, 0); //1
	PIDController_initialise(&velocity_pid_controller[2], 10, 0, 0); //0.5
	PIDController_initialise(&velocity_pid_controller[3], 20, 0, 0); //1
	PIDController_initialise(&velocity_pid_controller[4], 20, 0, 0); //1

	PIDController_set_limit(&velocity_pid_controller[0], 500, 2000);
	PIDController_set_limit(&velocity_pid_controller[1], 500, 2000);
	PIDController_set_limit(&velocity_pid_controller[2], 500, 2000);
	PIDController_set_limit(&velocity_pid_controller[3], 500, 2000);
	PIDController_set_limit(&velocity_pid_controller[4], 500, 2000);
	/*
	 * Quintic Trajectory Following Initialise
	 */
	for (int i = 0; i < 5; i++) {
		QuinticTrajectory_initialise(&quintic_trajectory[i], 0.01);
	}
	int8_t j_num = 0;
	uint32_t timestamp1 = 0;
	uint32_t timestamp2 = 0;
	uint32_t timestamp3 = 0;
	HAL_StatusTypeDef rep = HAL_ERROR;
	int32_t encoder_unwrap_value[5] = { 0 };
	double q0[5] = {-2, 8, -10, 4, 4};
	double q1[5] = {2, -16, 8, -2.5, -2.5};
	double v0[5] = {0};
	double v1[5] = {0};
	double ac0[5] = {0};
	double ac1[5] = {0};
	double tf = 6;
	int8_t traj_buf = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if ((delta_khe[0] != 0)||
				(delta_khe[1] != 0)||
				(delta_khe[2] != 0)||
				(delta_khe[3] != 0)||
				(delta_khe[4] != 0)){
			/*
			 * Tai-Ban Cartesian Jog
			 */
			double joint_config[5] = {0};
			double delta_q[5] = {0};
			joint_config[0] = desired_position[0] * (9.0/25.0);
			joint_config[1] = desired_position[1] / 27.0;
			joint_config[2] = asin(desired_position[2]/22.5);
			joint_config[3] = (desired_position[3] + desired_position[4]) * 0.1125;
			joint_config[4] = (desired_position[3] - desired_position[4])/8.0;
			IVK(joint_config, delta_khe, delta_q);
			for (int i = 0; i < 5; i++) {
				delta_khe[i] = 0;
				desired_position[i] += delta_q[i];
			}
		}
		if (HAL_GetTick() - timestamp1 >= 100) {
			/*
			 * UART
			 */
			timestamp1 = HAL_GetTick();
			ARMsProtocol_FUNC_Interface();
			if (servo_flag){
				/*
				 * Set Servo Degree
				 */
				servo_set_degree(&servo_motor, servo_degree);
				servo_flag = 0;
			}
		}
		if (HAL_GetTick() - timestamp2 >= 2) {
			/*
			 * Read Encoders
			 */
			timestamp2 = HAL_GetTick();
			rep = HAL_ERROR;
			encoder_unwrap_value[j_num] = 0;
			while (1) {
				AMT21_read_value(&(encoders[j_num]));
				rep = AMT21_check_value(&(encoders[j_num]));
				if (rep == HAL_OK) {
					encoder_unwrap_value[j_num] = AMT21_unwrap(
							(int32_t) encoders[j_num].position,
							(int32_t) encoders[j_num].prev_position);
					encoders[j_num].prev_position = encoders[j_num].position;
					break;
				}
			}
			if ((j_num == 0) | (j_num == 2) | (j_num == 3)) {
				encoder_unwrap_value[j_num] = encoder_unwrap_value[j_num] * -1;
			}
			encoder_config[j_num] = encoder_config[j_num]
					+ encoder_unwrap_value[j_num];
			j_num++;
			if (j_num == 5) {
				j_num = 0;
			}
		}
		if (HAL_GetTick() - timestamp3 >= 10) {
			/*
			 * Control Loop
			 */
			timestamp3 = HAL_GetTick();
			if (quintic_trajectory[0].is_end) {
				if (traj_buf == 0) {
//					QuinticTrajectory_set_param(&(quintic_trajectory[0]), -2,
//							0, 0, 0.094814814814815, -0.018962962962963, 0.001011358024691, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[1]), -20,
//							0, 0, 0.545185185185185, -0.109037037037037, 0.005815308641975, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[2]), -2.5,
//							0, 0, 0.201481481481481, -0.040296296296296, 0.002149135802469, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[3]), -2.75, 0,
//							0, 0.130370370370370, -0.026074074074074, 0.001390617283951, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[4]), -2.75, 0,
//							0, 0.130370370370370, -0.026074074074074, 0.001390617283951, 7.5);
					for (int i =0; i <5 ;i ++){
						QuinticTrajectory_cal_and_set_coeff(&(quintic_trajectory[i]), q0[i], q1[i], v0[i], v1[i], ac0[i], ac1[i], tf);
					}
					traj_buf = 1;
				} else if (traj_buf == 1) {
//					QuinticTrajectory_set_param(&(quintic_trajectory[0]), 2,
//							0, 0, -0.094814814814815, 0.018962962962963, -0.001011358024691, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[1]), 3, 0,
//							0, -0.545185185185185, 0.109037037037037, -0.005815308641975, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[2]), 6, 0,
//							0, -0.201481481481481, 0.040296296296296, -0.002149135802469, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[3]), 2.75, 0,
//							0, -0.130370370370370, 0.026074074074074, -0.001390617283951, 7.5);
//					QuinticTrajectory_set_param(&(quintic_trajectory[4]), 2.75, 0,
//							0, -0.130370370370370, 0.026074074074074, -0.001390617283951, 7.5);
					for (int i =0; i <5 ;i ++){
						QuinticTrajectory_cal_and_set_coeff(&(quintic_trajectory[i]), q1[i], q0[i], v0[i], v1[i], ac0[i], ac1[i], tf);
					}
					traj_buf = 0;
				}
			}
			motor_config[0] = ((2 * M_PI * encoder_config[0]) / 16384.0)
					* (25.0 / 9.0);
			motor_config[1] = ((2 * M_PI * encoder_config[1]) / 16384.0) * 27.0;
			motor_config[2] = 22.5
					* sin((2 * M_PI * encoder_config[2]) / 16384.0);
			motor_config[3] = (2 * M_PI * encoder_config[3]) / 16384.0; //checked
			motor_config[4] = (2 * M_PI * encoder_config[4]) / 16384.0; //checked
			/*
			 * Trajectory Update
			 */
			for (int i = 0; i <5; i++){
				QuinticTrajectory_update(&(quintic_trajectory[i]));
				desired_position[i] = quintic_trajectory[i].pos_out;
				desired_velocity[i] = quintic_trajectory[i].vel_out;
				KalmanFilter_Update(&(kalman_filter[i]), motor_config[i]);
			}
			/*
			 * Joint Limit
			 */
			for (int i = 0; i < 5; i++) {
				if (desired_position[i] >= max_desired_position[i]) {
					desired_position[i] = max_desired_position[i];
				}
				if (desired_position[i] <= min_desired_position[i]) {
					desired_position[i] = min_desired_position[i];
				}
			}
			/*
			 * Cascade Controller
			 */
			for (int i = 0; i < 5; i++) {
				cascade_out[i] = Cascade_PIDController_update(&(position_pid_controller[i]),
						&(velocity_pid_controller[i]),
						&(kalman_filter[i]), desired_position[i],
						desired_velocity[i]);
			}
			for (int i = 0; i < 5; i++) {
				stepper_set_speed(&steppers[i], cascade_out[i]);
			}
		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 15;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
	while (1) {
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

