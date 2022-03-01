/*
 * ARMs_Protocol.c
 *
 *  Created on: Jan 21, 2022
 *      Author: Natmatee
 */
/*
 ******************************************************************************
 * @file    ARMs_Protocol.c
 * @brief   This file provides code for Interface UART and Communication
 * 			with Manipulator.
 * 			License by Narwhal & Zhu Corp
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <Library/ARMsProtocol.h>
#include <Library/Kinematic.h>
#include "usart.h"
#include "crc.h"
#include "math.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
double desired_position[5] = {0};
double min_desired_position[5] = { -2.3, -35.0, -15.0, -4, -4 };
double max_desired_position[5] = { 2.3, 15.0, 11.0, 4, 4 };
uint8_t servo_flag = 0;
double servo_degree = 0;
int32_t encoder_config[5] = { 0 };
/* Private user code ---------------------------------------------------------*/
ARMsProtocol_DATA ARMsProtocol_Data;
ARMsProtocol_HandleTypedef ARMsProtocol_h1;


/*=============================================================================*/
/**
 * @brief	Initial Function : Initial the parameters that use in this library
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Init(void){
	// setting ARMsProtocol_h1
	ARMsProtocol_h1.handle = &huart3;
	ARMsProtocol_h1.Instance = USART3;
	ARMsProtocol_h1.slave_id = 0;

	//setting ARMsProtocol_Data
	ARMsProtocol_Data.Code = 0;
	ARMsProtocol_Data.State = 0;
	ARMsProtocol_Data.Rx_count = 0;

	// Enable UART IT
	HAL_UART_Receive_IT(ARMsProtocol_h1.handle, &ARMsProtocol_Data.Rx_reg, 1);
}


/*=============================================================================*/
/**
 * @brief	Interface Function : Data frame checking and state machine
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Interface(void){
	ARMsProtocol_Data.Flag = 1;
	if(ARMsProtocol_Data.Flag  == 1){
		//check header
		if(ARMsProtocol_Data.State == 0){
			ARMsProtocol_Data.Header = ARMsProtocol_Data.Rx_buf[0];
			if(ARMsProtocol_Data.Header == ARMsProtocol_HEADER){
				ARMsProtocol_Data.State = 1;
			}
		}
		//check slave id
		if(ARMsProtocol_Data.State == 1){
			ARMsProtocol_Data.Id = ARMsProtocol_Data.Rx_buf[1];
			if(ARMsProtocol_Data.Id == ARMsProtocol_h1.slave_id){
				ARMsProtocol_Data.Instruction = ARMsProtocol_Data.Rx_buf[2];
				ARMsProtocol_Data.Length = ARMsProtocol_Data.Rx_buf[3];
				ARMsProtocol_Data._CRC = ARMsProtocol_Data.Rx_buf[3 + ARMsProtocol_Data.Length];
				ARMsProtocol_Data.State = 2;
			}
		}
		// check crc
		if(ARMsProtocol_Data.State == 2){
			ARMsProtocol_CALC_CRC((uint32_t *) &ARMsProtocol_Data.Rx_buf[2], ARMsProtocol_Data.Length + 1);
			if(ARMsProtocol_Data._CRC == ARMsProtocol_Data.CRC_CAL){
				for(int i = 0;i <= ARMsProtocol_Data.Length - 2;i++){
					ARMsProtocol_Data.Data_buf[i] = ARMsProtocol_Data.Rx_buf[i+4];
				}
				ARMsProtocol_Data.State = 3;
			}
			else{
				ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ILLEGALCRC);
			}
			ARMsProtocol_FUNC_Rx_Clrbuf(ARMsProtocol_Data.Rx_count);
			ARMsProtocol_Data.Rx_count = 0;
		}
		if(ARMsProtocol_Data.State == 3){
			switch(ARMsProtocol_Data.Instruction){
			/* USER CODE BEGIN 0 */
			case ARMsProtocol_ADDR_JOINTJOG:
				ARMsProtocol_FUNC_Jointjog();
				break;
			case ARMsProtocol_ADDR_CATESIANJOG:
				ARMsProtocol_FUNC_Catesianjog();
				break;
			case ARMsProtocol_ADDR_TRAJECTORY:
				ARMsProtocol_FUNC_Trajectory();
				break;
			case ARMsProtocol_ADDR_PATHWAY:
				ARMsProtocol_FUNC_Pathway();
				break;
			case ARMsProtocol_ADDR_GRIPPER:
				ARMsProtocol_FUNC_Gripper();
				break;
			case ARMsProtocol_ADDR_JOTNTFEEDBACK:
				ARMsProtocol_FUNC_Jointfeedback(ARMsProtocol_h1.handle);
				break;
			case ARMsProtocol_ADDR_BOARDFEEDBACK:
				ARMsProtocol_FUNC_Boardfeedback(ARMsProtocol_h1.handle);
				break;
			/* USER CODE END 0 */
			default:
				ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ILLEGALFUNC);
			}
		}
		ARMsProtocol_Data.Flag  = 0;
		ARMsProtocol_Data.State = 0;
		ARMsProtocol_Data.Rx_count = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Recieve Callback Function
 * @param  	huart : UART_TypeDef of USART
 * @retval 	None
 */
void ARMsProtocol_FUNC_Rx_Callback(UART_HandleTypeDef *huart){
	if (huart->Instance == ARMsProtocol_h1.Instance) {
		ARMsProtocol_Data.Rx_buf[ARMsProtocol_Data.Rx_count++] = ARMsProtocol_Data.Rx_reg;
			if (ARMsProtocol_Data.Rx_count >= sizeof(ARMsProtocol_Data.Rx_buf)) {
				ARMsProtocol_Data.Rx_count = 0;
			}
		HAL_UART_Receive_IT(huart, &ARMsProtocol_Data.Rx_reg, 1);
	}
}


/*=============================================================================*/
/**
 * @brief	Transmit Callback Function
 * @param  	huart : UART_TypeDef of USART
 * @retval 	None
 */
void ARMsProtocol_FUNC_Tx_Callback(UART_HandleTypeDef *huart){
	if (huart->Instance == ARMsProtocol_h1.Instance){
			HAL_UART_Transmit_IT(huart, &ARMsProtocol_Data.Tx_buf[0], ARMsProtocol_Data.Tx_count);
			while(ARMsProtocol_Data.Tx_flag);
			//ARMsProtocol_Data.Tx_count = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Exception Response function
 * @param  	huart : UART_TypeDef of USART
 * 			code  : Report Code -> 	0x01 = ARMsProtocol_TRANSMIT_ILLEGALFUNC
 * 									0x02 = ARMsProtocol_TRANSMIT_ILLEGALCRC
 * 									0x03 = ARMsProtocol_TRANSMIT_ACKNOWLEDGE
 * 									0x04 = ARMsProtocol_TRANSMIT_DONE
 * @retval 	None
 */
void ARMsProtocol_EXCEPTION_Response(UART_HandleTypeDef *huart, uint8_t code){
	ARMsProtocol_Data.Tx_count = 4;
	ARMsProtocol_Data.Tx_buf[0] = 0xFF;
	ARMsProtocol_Data.Tx_buf[1] = ARMsProtocol_h1.slave_id;
	ARMsProtocol_Data.Tx_buf[2] = code;
	ARMsProtocol_CALC_CRC((uint32_t*) &ARMsProtocol_Data.Tx_buf, ARMsProtocol_Data.Tx_count - 1);
	ARMsProtocol_Data.Tx_buf[3] = ARMsProtocol_Data.CRC_CAL;
	ARMsProtocol_Data.Tx_flag = 1;
	ARMsProtocol_FUNC_Tx_Callback(huart);
}


/*=============================================================================*/
/**
 * @brief	CRC Calculation Function
 * @param  	*nData  : Data
 * 			wLength : Length of Data
 * @retval 	None
 */
void ARMsProtocol_CALC_CRC (uint32_t *pBuffer, uint32_t BufferLength)
{
	ARMsProtocol_Data.CRC_CAL = HAL_CRC_Calculate(&hcrc, pBuffer, BufferLength) ^ 0xFF;
}


/*=============================================================================*/
/**
 * @brief	Clear Recieve Buffer
 * @param  	count : amount of data in Rx_buf
 * @retval 	None
 */
void ARMsProtocol_FUNC_Rx_Clrbuf(uint8_t count){
	for(int i =0; i <= count; i++){
		ARMsProtocol_Data.Rx_buf[i] = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Clear Data Buffer
 * @param  	count : amount of data in Data_buf
 * @retval 	None
 */
void ARMsProtocol_FUNC_Data_Clrbuf(){
	for(int i =0; i <= ARMsProtocol_Data.Length; i++){
		ARMsProtocol_Data.Data_buf[i] = 0;
	}
}


/*=============================================================================*/
/**
 * @brief	Sethome Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Pathway(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	ARMsProtocol_Data.Pathway_flag = 1;
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Jointjog Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Jointjog(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	ARMsProtocol_Data.Jointjog_flag = 1;
	double data_buf = 0.0;
	/* USER CODE BEGIN 3 */
	for(int i = 0;i < 5;i++){
		data_buf = (int16_t)((ARMsProtocol_Data.Data_buf[i*2] << 8) + ARMsProtocol_Data.Data_buf[(i*2)+1]);
		data_buf = data_buf/1000.0;
		desired_position[i] += data_buf;
		if (i == 0){
			data_buf = data_buf / (9.0 / 25.0);
			desired_position[0] += data_buf;
		}
		else if (i == 1){
			data_buf = data_buf * 27.0;
			desired_position[1] += data_buf;
		}
		else if (i == 2){
			data_buf = 22.5 * sin(data_buf);
			desired_position[2] += data_buf;
		}
		else if (i ==3){
			desired_position[3] += (40.0 * data_buf)/9.0;
			desired_position[4] += (40.0 * data_buf)/9.0;
		}
		else if (i ==4){
			desired_position[3] += (4 * data_buf);
			desired_position[4] -= (4 * data_buf);
		}
		else {
			desired_position[i] += data_buf;
		}
	}
	/* USER CODE END 3 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Catesianjog Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Catesianjog(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	ARMsProtocol_Data.Catesian_flag = 1;
	/* USER CODE BEGIN 4 */
	double joint_config[5] = {0};
	double delta_q[5] = {0};
	double delta_khe[5] = {0};
	for (int i = 0; i <5; i++){
		delta_khe[i] = (int16_t)((ARMsProtocol_Data.Data_buf[i*2] << 8) + ARMsProtocol_Data.Data_buf[(i*2)+1]);
		delta_khe[i] = delta_khe[i]/1000.0;
//		if (i == 0){
//			delta_khe[i] = (delta_khe[i]/1000.0f)/10;
//		}
//		if (i == 1){
//			delta_khe[i] = (delta_khe[i]/1000.0f)/10;
//		}
//		else {
//			delta_khe[i] = 5*(delta_khe[i]/1000.0f);
//		}
	}
//	joint_config[0] = (2*M_PI * encoder_config[0])/16384.0f;
//	joint_config[1] = (2*M_PI * encoder_config[1])/16384.0f;
//	joint_config[2] = (2*M_PI * encoder_config[2])/16384.0f;
//	double m4 = (2*M_PI * encoder_config[3])/16384.0f;
//	double m5 =  (2*M_PI * encoder_config[4])/16384.0f;
//	joint_config[3] = (m4 + m5) * 0.1125;
//	joint_config[4] = (m4 - m5)/8.0;
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
	/* USER CODE END 4 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 *
 * @brief	Recievetrajectory Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Trajectory(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	ARMsProtocol_Data.Trajectory_flag = 1;
	/* USER CODE BEGIN 5 */

	/* USER CODE END 5 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Controlgripper Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Gripper(void){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 6 */
	servo_flag = 1;
	servo_degree = ARMsProtocol_Data.Data_buf[0];
	/* USER CODE END 6 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


/*=============================================================================*/
/**
 * @brief	Setzeroencoder Function
 * @param  	None
 * @retval 	None
 */
void ARMsProtocol_FUNC_Jointfeedback(UART_HandleTypeDef *huart){
	// Acknowledge Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 7 */
	double joint_config[5] = {0};
	joint_config[0] = (2*M_PI * encoder_config[0])/16384.0f;
	joint_config[1] = (2*M_PI * encoder_config[1])/16384.0f;
	joint_config[2] = (2*M_PI * encoder_config[2])/16384.0f;
	double m4 = (2*M_PI * encoder_config[3])/16384.0f;
	double m5 =  (2*M_PI * encoder_config[4])/16384.0f;
	joint_config[3] = (m4 + m5) * 0.1125;
	joint_config[4] = (m4 - m5)/8.0;
	joint_config[0] = joint_config[0]*1000.0;
	joint_config[1] = joint_config[1]*1000.0;
	joint_config[2] = joint_config[2]*1000.0;
	joint_config[3] = joint_config[3]*1000.0;
	joint_config[4] = joint_config[4]*1000.0;
	ARMsProtocol_Data.Tx_count = 15;
	ARMsProtocol_Data.Tx_buf[0] = 0xFF;
	ARMsProtocol_Data.Tx_buf[1] = ARMsProtocol_h1.slave_id;
	ARMsProtocol_Data.Tx_buf[2] = ARMsProtocol_ADDR_JOTNTFEEDBACK;
	ARMsProtocol_Data.Tx_buf[3] = 11;
	ARMsProtocol_Data.Tx_buf[4] = ((int16_t)joint_config[0]) >> 8; //J1_H
	ARMsProtocol_Data.Tx_buf[5] = ((int16_t)joint_config[0]);//J1_L
	ARMsProtocol_Data.Tx_buf[6] = ((int16_t)joint_config[1]) >> 8; //J2_H
	ARMsProtocol_Data.Tx_buf[7] = ((int16_t)joint_config[1]); //J2_L
	ARMsProtocol_Data.Tx_buf[8] = ((int16_t)joint_config[2]) >> 8; //J3_H
	ARMsProtocol_Data.Tx_buf[9] = ((int16_t)joint_config[2]); //J3_L
	ARMsProtocol_Data.Tx_buf[10] = ((int16_t)joint_config[3]) >> 8;//J4_H
	ARMsProtocol_Data.Tx_buf[11] = ((int16_t)joint_config[3]);//J4_L
	ARMsProtocol_Data.Tx_buf[12] = ((int16_t)joint_config[4]) >> 8;//J5_H
	ARMsProtocol_Data.Tx_buf[13] = ((int16_t)joint_config[4]);//J6_L
	ARMsProtocol_CALC_CRC((uint32_t*) &ARMsProtocol_Data.Tx_buf[2], 12);
	ARMsProtocol_Data.Tx_buf[14] = ARMsProtocol_Data.CRC_CAL;
	ARMsProtocol_Data.Tx_flag = 1;
	ARMsProtocol_FUNC_Tx_Callback(huart);
	/* USER CODE END 7 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


void ARMsProtocol_FUNC_Boardfeedback(UART_HandleTypeDef *huart){
	// Acknowledge Func
	ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_ACKNOWLEDGE);
	/* USER CODE BEGIN 8 */
	ARMsProtocol_Data.Tx_count = 7;
	ARMsProtocol_Data.Tx_buf[0] = 0xFF;
	ARMsProtocol_Data.Tx_buf[1] = ARMsProtocol_h1.slave_id;
	ARMsProtocol_Data.Tx_buf[2] = ARMsProtocol_ADDR_BOARDFEEDBACK;
	ARMsProtocol_Data.Tx_buf[3] = 11;
	ARMsProtocol_Data.Tx_buf[4] = 0;
	ARMsProtocol_Data.Tx_buf[5] = 0;
	ARMsProtocol_CALC_CRC((uint32_t*) &ARMsProtocol_Data.Tx_buf, ARMsProtocol_Data.Tx_count - 1);
	ARMsProtocol_Data.Tx_buf[6] = ARMsProtocol_Data.CRC_CAL;
	ARMsProtocol_Data.Tx_flag = 1;
	ARMsProtocol_FUNC_Tx_Callback(huart);
	/* USER CODE END 8 */

	// Clear Data
	ARMsProtocol_FUNC_Data_Clrbuf();
	// Done Func
	//ARMsProtocol_EXCEPTION_Response(ARMsProtocol_h1.handle, ARMsProtocol_TRANSMIT_DONE);
}


void ARMsProtocol_FUNC_Clrflag(void){
	ARMsProtocol_Data.Jointjog_flag = 0;
	ARMsProtocol_Data.Catesian_flag = 0;
	ARMsProtocol_Data.Trajectory_flag = 0;
	ARMsProtocol_Data.Pathway_flag = 0;
}
