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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t adcResult = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
inline uint32_t get_us() {
	return 1000 * HAL_GetTick(); // ДУже грубо, а що зробиш?
}
#define LOOP_FREQ (SystemCoreClock/4000000)
#define TRESHOLD                            3000

inline void udelay_asm(uint32_t useconds) {
	useconds *= LOOP_FREQ;

	asm volatile("   mov r0, %[useconds]    \n\t"
			"1: subs r0, #1            \n\t"
			"   bhi 1b                 \n\t"
			:
			: [useconds] "r" (useconds)
			: "r0");
}

/* USER CODE END 0 */
unsigned char i = 0;
unsigned int hasRequest = 0;

int led_status = 0;
void EXTI0_1_IRQHandler(void) {
	//Handler for interruptions on external button and motion sensor
	//We light the led and notify all systems about that
	EXTI->PR |= 1;
	if (GPIOC->ODR & 1 << 12) {
		GPIOC->ODR &= ~(1 << 12);
	} else {
		GPIOC->ODR |= (1 << 12);

	}
}

void USART1_IRQHandler(void) {
	uint8_t chartoreceive = 0;
	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chartoreceive = (uint8_t) (USART1->RDR);
		switch (chartoreceive) {
		case 'O':
			GPIOC->ODR |= (1 << 6);
			hasRequest = 1;
			break;
		case 'F':
			GPIOC->ODR &= ~(1 << 6);
			hasRequest = 0;
			break;
		default:
			break;
		}
	} else {
		NVIC_DisableIRQ(USART1_IRQn);
	}
}
void USART2_IRQHandler(void) {
	uint8_t chartoreceive = 0;
	if ((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		chartoreceive = (uint8_t) (USART2->RDR);
		switch (chartoreceive) {
		case 'O':
			GPIOC->ODR |= (1 << 12);
			break;
		case 'B':
			GPIOC->ODR &= ~(1 << 12);
			break;
		default:
			break;
		}
	} else {
		NVIC_DisableIRQ(USART2_IRQn);
	}
}

void init_PA0_interrupt() {
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_PA;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->RTSR |= EXTI_RTSR_TR0;
	NVIC_SetPriority(EXTI0_1_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void init_USART_1() {
	//Configures for USART line 1
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH1 |
	GPIO_AFRH_AFRH2)) | (1 << (1 * 4)) | (1 << (2 * 4));

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->BRR = 480000 / 96;
	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
	while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
		;
	USART1->ICR |= USART_ICR_TCCF;
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
}
void init_USART_2() {
	//Configures for USART line 2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE |
	USART_CR1_RXNEIE;
	USART2->BRR = 480000 / 96;
	while ((USART2->ISR & USART_ISR_TC) != USART_ISR_TC)
		;
	USART2->ICR |= USART_ICR_TCCF;
}

int main(void) {

	/* USER CODE BEGIN 1 */
	init_PA0_interrupt();

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
	GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	GPIOA->MODER = (GPIOA->MODER
			& ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10 | GPIO_MODER_MODER14
					| GPIO_MODER_MODER15))
			| (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1 | GPIO_MODER_MODER14_1
					| GPIO_MODER_MODER15_1);
	GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFRH1 |
	GPIO_AFRH_AFRH2)) | (1 << (1 * 4)) | (1 << (2 * 4));

	init_USART_1();

	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);

	GPIOC->MODER |= GPIO_MODER_MODER12_0;

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
	MX_ADC_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;

	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 |
	GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	//Setting AF for USART 2

	GPIOA->MODER = (GPIOA->MODER
			& ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15
					| GPIO_MODER_MODER7))\

			| (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1
					| GPIO_MODER_MODER7_1);

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOC->MODER |= (1 << 15);
	GPIOC->MODER &= ~(1 << 14);

	TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; /* (1) */
	TIM3->PSC = 0; /* (2) */
	TIM3->ARR = 999; /* (3) */
	TIM3->CCR2 = 0;
	; /* (4) */
	TIM3->CCER |= (TIM_CCER_CC2E); // Capture/Compare 2 output enable -- PA7
	TIM3->CCER &= ~TIM_CCER_CC2P;   //

	TIM3->CR1 |= TIM_CR1_CEN; //Turn on

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		int t_state = HAL_GPIO_ReadPin(Trig_GPIO_Port, Trig_Pin);
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
//		udelay_asm(16);
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
		if (HAL_GPIO_ReadPin(Trig_GPIO_Port, Trig_Pin) != GPIO_PIN_RESET) {
			HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
			HAL_Delay(300);
			continue;
		}
		if (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET) {
			HAL_Delay(300);
			continue;
		}
		uint32_t watchdog_begin = get_us();
		int didnt_had_1_at_echo = 0;
		while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_RESET) {
			if (get_us() - watchdog_begin > 500000) {
				didnt_had_1_at_echo = 1;
				break;
			}
		}
		if (didnt_had_1_at_echo) {
			HAL_Delay(300);
			continue;
		}

		uint32_t before = get_us();
		int didnt_had_0_at_echo = 0;
		while (HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin) == GPIO_PIN_SET) {

			if (get_us() - watchdog_begin > 500000) {
				didnt_had_0_at_echo = 1;
				break;
			}

		}
		if (didnt_had_0_at_echo) {
			HAL_Delay(300);
			continue;
		}

		uint32_t pulse_time = get_us() - before;
		uint32_t distance = pulse_time / 58;

		int water_available = 1;
		if (distance > 50) {
			water_available = 0;
		}

		HAL_ADC_Start(&hadc);

		HAL_ADC_PollForConversion(&hadc, 100);

		adcResult = HAL_ADC_GetValue(&hadc);

		HAL_ADC_Stop(&hadc);

		if ((adcResult >= TRESHOLD || hasRequest == 1)
				&& water_available == 1) {
			TIM3->CCR2 = 999;
			hasRequest = 0;
			HAL_Delay(5000);
			TIM3->CCR2 = 0;
			HAL_Delay(15000);
		}

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
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

/* ADC init function */
static void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA2   ------> TSC_G1_IO3
 PA3   ------> TSC_G1_IO4
 PA6   ------> TSC_G2_IO3
 PA7   ------> TSC_G2_IO4
 PB0   ------> TSC_G3_IO2
 PB1   ------> TSC_G3_IO3
 PB10   ------> I2C2_SCL
 PB11   ------> I2C2_SDA
 PB13   ------> SPI2_SCK
 PB14   ------> SPI2_MISO
 PB15   ------> SPI2_MOSI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
	NCS_MEMS_SPI_Pin | EXT_RESET_Pin | LD3_Pin | LD6_Pin | LD4_Pin | LD5_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
	 LD4_Pin LD5_Pin */
	GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin | EXT_RESET_Pin | LD3_Pin | LD6_Pin
			| LD4_Pin | LD5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = MEMS_INT1_Pin | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : I2C2_SCL_Pin I2C2_SDA_Pin */
	GPIO_InitStruct.Pin = I2C2_SCL_Pin | I2C2_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_SCK_Pin SPI2_MISO_Pin SPI2_MOSI_Pin */
	GPIO_InitStruct.Pin = SPI2_SCK_Pin | SPI2_MISO_Pin | SPI2_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Echo_Pin */
	GPIO_InitStruct.Pin = Echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Echo_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Trig_Pin */
	GPIO_InitStruct.Pin = Trig_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Trig_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
