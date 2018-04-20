/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "canpie.h"
#include "cp_msg.h"
#include "cp_core.h"
#include "printf.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#if 0
CanTxMsgTypeDef TxM;
CanRxMsgTypeDef RxM;
CAN_FilterConfTypeDef sFilterConfig;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t canpie_rx_handler(CpCanMsg_ts *ptsCanMsgV, uint8_t ubBufferIdxV);
uint8_t canpie_tx_handler(CpCanMsg_ts *ptsCanMsgV, uint8_t ubBufferIdxV);
uint8_t canpie_err_handler(CpState_ts *state);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	uint32_t pclk;

	CpPort_ts can_port_main;
#if CP_FIFO
	CpFifo_ts tsFifoRcvS;
	CpCanMsg_ts atsCanMsgRcvS[32];
	CpCanMsg_ts tsCanMsgReadT;
	uint32_t ulMsgCntT;
#endif
	CpStatus_tv tvResultT;

	uint8_t tx_data[8];
	tx_data[0] = 'C';
	tx_data[1] = 'A';
	tx_data[2] = 'N';
	tx_data[3] = 'P';
	tx_data[4] = 'I';
	tx_data[5] = 'E';
	tx_data[6] = 'F';
	tx_data[7] = 'D';

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART6_UART_Init();

	/* USER CODE BEGIN 2 */
	pclk = HAL_RCC_GetPCLK1Freq();

#if 0
	hcan1.pTxMsg = &TxM;
	hcan1.pRxMsg = &RxM;

	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x02 << 5; // page 10-66
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	hcan1.pTxMsg->StdId = 0x1f3;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->DLC = 8;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);

	hcan1.pTxMsg->Data[0] = 0x04;
	hcan1.pTxMsg->Data[1] = 0x55;
	hcan1.pTxMsg->Data[2] = 0x4B;
	hcan1.pTxMsg->Data[3] = 0x57;
	hcan1.pTxMsg->Data[4] = 0x4E;
	hcan1.pTxMsg->Data[5] = 0x98;
	hcan1.pTxMsg->Data[6] = 0x96;
	hcan1.pTxMsg->Data[7] = 0x7F;

	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_SET);

	HAL_CAN_Transmit(&hcan1, 10);
#else

	// start initialization
	printf("\ncanpie-fd demo for stm32f4xx");

	//----------------------------------------------------------------
	// valid initialization
	//
	memset(&can_port_main, 0, sizeof(CpPort_ts));
	tvResultT = CpCoreDriverInit(eCP_CHANNEL_1, &can_port_main, 0);
	CpCoreIntFunctions(&can_port_main, canpie_rx_handler, canpie_tx_handler, canpie_err_handler);

	CpCoreBitrate(&can_port_main, eCP_BITRATE_250K, eCP_BITRATE_250K);
	CpCoreCanMode(&can_port_main, eCP_MODE_START);

	// some rx buffer
	CpCoreBufferConfig(&can_port_main, eCP_BUFFER_7, 0x001, 0x001, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_RCV);
	CpCoreBufferConfig(&can_port_main, eCP_BUFFER_8, 0x002, 0x002, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_RCV);

#if CP_FIFO
	CpCoreBufferConfig(&can_port_main, eCP_BUFFER_9, 0x00 , 0x003, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_RCV);
	CpFifoInit(&tsFifoRcvS, &atsCanMsgRcvS[0], 32);
	CpCoreFifoConfig(&can_port_main, eCP_BUFFER_9, &tsFifoRcvS);
#endif
#endif

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		/* USER CODE END WHILE */
		pclk = HAL_RCC_GetPCLK1Freq();
		printf("\nPCLK: %ld", pclk);

		/* USER CODE BEGIN 3 */
#if 0
		HAL_CAN_Transmit(&hcan, 1);
#else
		CpStatus_tv ret = eCP_ERR_TRM_FULL;

		CpCoreBufferConfig(&can_port_main, eCP_BUFFER_4, 0x004, 2047, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_TRM);
		CpCoreBufferSetData(&can_port_main, eCP_BUFFER_4, tx_data, 0, 1);
		CpCoreBufferSetDlc(&can_port_main, eCP_BUFFER_4, 1);
		ret = CpCoreBufferSend(&can_port_main, eCP_BUFFER_4);
		HAL_Delay(1);
		printf("\nret: %d", ret);

		CpCoreBufferConfig(&can_port_main, eCP_BUFFER_5, 0x12030, 2047, CP_MSG_FORMAT_CEFF, eCP_BUFFER_DIR_TRM);
		CpCoreBufferSetData(&can_port_main, eCP_BUFFER_5, tx_data, 0, 3);
		CpCoreBufferSetDlc(&can_port_main, eCP_BUFFER_5, 3);
		ret = CpCoreBufferSend(&can_port_main, eCP_BUFFER_5);
		HAL_Delay(1);
		printf("\nret: %d", ret);

		CpCoreBufferConfig(&can_port_main, eCP_BUFFER_10, 0x010, 2047, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_TRM);
		CpCoreBufferSetData(&can_port_main, eCP_BUFFER_10, tx_data, 0, 4);
		CpCoreBufferSetDlc(&can_port_main, eCP_BUFFER_10, 4);
		ret = CpCoreBufferSend(&can_port_main, eCP_BUFFER_10);
		HAL_Delay(1);
		printf("\nret: %d", ret);

		CpCoreBufferConfig(&can_port_main, eCP_BUFFER_3, 0x003, 2047, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_TRM);
		CpCoreBufferSetData(&can_port_main, eCP_BUFFER_3, tx_data, 0, 6);
		CpCoreBufferSetDlc(&can_port_main, eCP_BUFFER_3, 6);
		ret = CpCoreBufferSend(&can_port_main, eCP_BUFFER_3);
		HAL_Delay(1);
		printf("\nret: %d", ret);

		CpCoreBufferConfig(&can_port_main, eCP_BUFFER_2, 0x002, 2047, CP_MSG_FORMAT_CBFF, eCP_BUFFER_DIR_TRM);
		CpCoreBufferSetData(&can_port_main, eCP_BUFFER_2, tx_data, 0, 8);
		CpCoreBufferSetDlc(&can_port_main, eCP_BUFFER_2, 8);
		ret = CpCoreBufferSend(&can_port_main, eCP_BUFFER_2);
		HAL_Delay(1000);
		printf("\nret: %d", ret);

		printf("\n\n");

#if CP_FIFO
		ulMsgCntT = 1;
		ret = eCP_ERR_NONE;
		while (ret != eCP_ERR_FIFO_EMPTY)
		{
			ret = CpCoreFifoRead(&can_port_main, eCP_BUFFER_9, &tsCanMsgReadT, &ulMsgCntT);
			if (ret != eCP_ERR_FIFO_EMPTY)
			{
				canpie_rx_handler(&tsCanMsgReadT, eCP_BUFFER_9);
			}
		}
#endif
		if (tvResultT != eCP_ERR_NONE)
		{
			printf("\ncanpie-fd error: %d", tvResultT);
		}
#endif
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 160;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 16;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_3TQ;
	hcan1.Init.BS2 = CAN_BS2_5TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD12 PD13 PD14 PD15 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
#if 1
/**
 * @return current APB1 clk frequency in HZ
 */
uint32_t McCpuGetClockAPB1(void)
{
	uint32_t pclk;
	pclk = HAL_RCC_GetPCLK1Freq();
	return pclk;
}
#endif

uint8_t canpie_tx_handler(CpCanMsg_ts *ptsCanMsgV, uint8_t ubBufferIdxV)
{
	uint32_t canid = CpMsgGetStdId(ptsCanMsgV);
	printf("\ntx_handler: %d, id: 0x%lx", ubBufferIdxV, canid);
	return 0;
}

uint8_t canpie_rx_handler(CpCanMsg_ts *ptsCanMsgV, uint8_t ubBufferIdxV)
{
	uint32_t canid = CpMsgGetStdId(ptsCanMsgV);
	printf("\nrx: %d, id: 0x%lx, usr:%ld", ubBufferIdxV, canid, ptsCanMsgV->ulMsgUser);
	return 0;
}

uint8_t canpie_err_handler(CpState_ts *state)
{
	printf("\ncanpie_err_handler ... ");
	return 0;
}

void _putchar(char character)
{
	char nl = '\n';
	char cr = '\r';

	if (character == nl)
	{
		HAL_UART_Transmit(&huart6, (uint8_t *) &nl, 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart6, (uint8_t *) &cr, 1, HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&huart6, (uint8_t *) &character, 1, HAL_MAX_DELAY);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
