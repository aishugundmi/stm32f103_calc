/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fifo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

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

uint8_t bufbyte;

void uprintf(uint8_t Value);
void uart_send_string(char *p);

void UART1_PutChar(uint8_t data);
void UART1_PutStr(char *string);

int s_i(uint8_t *str);
void i_s(int num);
void r_str(char *str);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char s1[100], fnum[100];
char s2[100], s3[100], op, ch;
uint16_t a, b, c;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */

	/* USER CODE END USART1_IRQn 0 */
	/* USER CODE BEGIN USART1_IRQn 1 */
	if (LL_USART_IsActiveFlag_RXNE(USART1)
			&& LL_USART_IsEnabledIT_RXNE(USART1)) {
		bufbyte = LL_USART_ReceiveData8(USART1);
		fifo_write_byte(bufbyte);
	};
	ch = 1;

	/* USER CODE END USART1_IRQn 1 */
}

void uprintf(uint8_t Value) {
	//	LL_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value);
	// LL_USART_ReceiveData8(USART_TypeDef *USARTx)

	LL_USART_TransmitData8(USART1, Value);
}

void uart_send_string(char *p) {
	while (*p) {
		LL_USART_TransmitData8(USART1, *p++);
	}
}

void UART1_PutChar(uint8_t data) {
	LL_USART_TransmitData8(USART1, data);
	/* Wait for TXE flag to be raised */
	while (!LL_USART_IsActiveFlag_TXE(USART1))
		;
}

void UART1_PutStr(char *string) {
	while (*string != '\0')
		UART1_PutChar(*string++);
}

int s_i(uint8_t *str) {
	uint16_t num = 0;

	for (int i = 0; i < strlen((char *)str); i++) {
		num = num * (10);
		num += str[i] - '0';
	}
	//     printf("%d\n", num);


	return num;
}

void i_s(int num) {
	char str[100];
	int i = 0;
	while (num > 0) {
		str[i] = (num % 10) + '0';
		num /= 10;
		i++;
	}
	str[i] = '\0';
	r_str(str);
}

void r_str(char *str) {
	char t;
	int len = (strlen(str) - 1);
	for (int i = 0; i < len; i++) {
		t = str[i];
		s1[i] = str[len];
		s1[len] = t;

		len--;
	}
}

int main(void) {
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	LL_USART_EnableIT_RXNE(USART1);
	/* USER CODE END 2 */


	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		memset(s1, 0, sizeof(s1));
		memset(s2, 0, sizeof(s2));
		memset(s3, 0, sizeof(s3));

		while (!fifo_data_available()) {
			; //wait loop for first byte of data to get filled in the fifo.
		}

		//now we reached here because some bytes are received in fifo.
		//Let us give some time for the fifo to get filled with the full string.
		HAL_Delay(300);

		//Collect the data from fifo to s1 buffer
		uint8_t n = 0;
		while (fifo_data_available()) {
			s1[n] = fifo_read_byte();
			n++;
		}

		//validation after 300 mS of wait...
		if (n) {
			if (s1[n - 1] == '\n') {
				//Valid data available in buffer...
			} else {
				//flush fifo for receiving new data string
				while (fifo_data_available()) {
					fifo_read_byte();
				}
				continue;// We couldn't find \n at the end of string even after 300 ms of waiting. Invalid packet. Go back to main while on top...
			}
		} else {
			continue; //no data available because n is zero! go to main while on top ...
		}

		///we are here because we received a valid string with proper \n ending. We will process the sting now.


		 int i = 0, j = 0;
         while((s1[i] > 47) && (s1[i] < 58))
		 {
		 s2[i] = s1[i];
		 i++;
		 }
		 a = s_i(s2);
		 char adata[20];
		 sprintf(adata, "a = %d\n", a);
		 UART1_PutStr(adata);

		 op = s1[i];
		 UART1_PutChar(op);
		 UART1_PutStr("\n");

		 i++;

		 while(s1[i] != '\n')
		 {
		 s3[j] = s1[i];
		 i++;
		 j++;
		 }
		 b = s_i(s3);
		 char bdata[20];
		 sprintf(bdata, "b = %d\n", b);
		 UART1_PutStr(bdata);

		switch (op) {
		char cdata[20];
		case '+':
			c = a + b;

			sprintf(cdata, "sum = %d\n", c);
			UART1_PutStr(cdata);

			break;
		case '-':
			c = a - b;
			sprintf(cdata, "difference = %d\n", c);
			UART1_PutStr(cdata);

			break;
		case '*':
			c = a * b;
			sprintf(cdata, "product = %d\n", c);
			UART1_PutStr(cdata);

			break;
		case '/':
			c = a / b;
			sprintf(cdata, "quotient = %d\n", c);
			UART1_PutStr(cdata);

			break;
		default:
			break;
		}

		UART1_PutStr("\n");

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	LL_USART_InitTypeDef USART_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	/**USART1 GPIO Configuration
	 PA9   ------> USART1_TX
	 PA10   ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 interrupt Init */
	NVIC_SetPriority(USART1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USART1_IRQn);

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);
	LL_USART_ConfigAsyncMode(USART1);
	LL_USART_Enable(USART1);
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : led_Pin */
	GPIO_InitStruct.Pin = led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
