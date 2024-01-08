/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UPDATE_PERIOD			0.005		/* Chu ki cap nhat: 5ms = ngat timer 4 */
#define PULSES_IN_ROUND		249.6		/* So xung tren 1 vong cua dong co = 13 xung x ti so truyen 19.2 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t buffer[64];
uint8_t run;
int16_t DesiredSpeed;
float DesiredVel;
char Rx_data[2];
char Rx_Buffer[20];
char Rx_indx;

int16_t pwm1;
int16_t pwm2;

/* Parameter of PIController */
float 		Kp 							= 0.321949254;				/* Ti le */
float 		Ki 							= 5.970149254;				/* Tich phan */
float 		Kd 							= 0.0;								/* Vi phan */
float			Kb 							= 18.54375863;				/* Dung cho Anti-windup */

int32_t 	UHiLim 					= 100;								/* Gioi han tren pwm */
int32_t 	ULoLim 					= 0;									/* Gioi han duoi pwm */

uint8_t 	PreviousState 	= 0;							/* Trang thai encoder chu ki truoc */					
int16_t 	CountValue  		= 0;							/* Bien dem encoder => tinh vi tri */
int32_t 	PosCnt  				= 0;							/* Bien dem so vong quay => tinh vi tri */
uint16_t 	VelCnt      		= 0;							/* Bien dem encoder => tinh van toc */
uint16_t 	VelTemp      		= 0;							/* Bien dem encoder => tinh van toc */

/* Parameter of HAL_TIM_PeriodElapsedCallback */
float 		CurrVel 				= 0;	 		/* Van toc hien tai (Rad/s) */
int16_t 	CurrSpeed	 			= 0;	 		/* Van toc hien tai (RPM) */
uint8_t 	dir							= 0;			/* Chieu quay banh trai */
uint16_t 	tick							= 0;			/* Dung de dem cho timer 5 */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ======================================================================================= USB printf ======================================================================================= */
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* FILE is typedef'd in stdio.h. */
FILE __stdout;
int fputc(int ch, FILE *f)
{	
    while(!(CDC_Transmit_FS((uint8_t*)&ch, 1) == USBD_OK))
        ;
    return ch;
}
/* ======================================================================================= End USB printf ======================================================================================= */

/* ================================================================================ PI Controller +(Anti-windup) ================================================================================ */
int32_t PIController(float desired_val, float real_val, float gain, float Ki, float Kb, int uHiLim, int uLoLim)
{
		/* Controller values */
		static float err_reset;
		static float uI_p;
		float err, uP, uI;
		int uOut;
		
		/* Algorithm */
		err = (desired_val - real_val);
			/* Proportional */
		uP = gain * err;
			/* Integral (with Anti-windup) */
		uI = uI_p + (Ki * UPDATE_PERIOD *err) + Kb * UPDATE_PERIOD * err_reset;
			/* Integral part */
		uI_p = uI;
			/* Output control */
		uOut = (int)(uP + uI);
			/* Output saturation */
		if(uOut >= uHiLim)
		{
			err_reset = uHiLim - uOut;
			uOut = uHiLim;
		}
		else if(uOut <= uLoLim)
		{
			err_reset = uLoLim - uOut;
			uOut = uLoLim;
		}
		return uOut;
}
/* ============================================================================== End PI Controller +(Anti-windup) ============================================================================== */
 
/* ================================================================================ Motor Control ================================================================================ */
void MotorRot(int32_t pwmSet)
{
	if (pwmSet > 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwmSet);
	}
	else if (pwmSet < 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,-pwmSet);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 0);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	}
	
}
/* ============================================================================== End Motor Control ============================================================================== */

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// khoi tao timer 1 	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	// khoi tao timer 1
	HAL_TIM_Base_Start_IT(&htim3);						// khoi tao timer 3
	HAL_TIM_Base_Start_IT(&htim4);						// khoi tao timer 4

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 11;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ========================================================================= Read Encoder ========================================================================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{		
	/* ENC_CHA RIGHT MOTOR */
	if(GPIO_Pin == GPIO_PIN_4)
	{
		unsigned char StateA;
		StateA = (StateA<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		StateA = (StateA<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
		StateA = StateA&0x03;
		switch (StateA) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
			break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
			break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
			break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
			break;
			}
		PreviousState = StateA;
		VelCnt++;
		if (CountValue>=4*PULSES_IN_ROUND) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-4*PULSES_IN_ROUND) {
			CountValue = 0;
			PosCnt--;
		}
	}
	
	/* ENC_CHB RIGHT MOTOR */
	if(GPIO_Pin == GPIO_PIN_6)
	{
		unsigned char StateB;
		StateB = (StateB<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		StateB = (StateB<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
		StateB = StateB&0x03;
		switch (StateB) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
			break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
			break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
			break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
			break;
			}
		PreviousState = StateB;
		VelCnt++;
		if (CountValue>=4*PULSES_IN_ROUND) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-4*PULSES_IN_ROUND) {
			CountValue = 0;
			PosCnt--;
		}
	}
}
/* ========================================================================= End Read Encoder ========================================================================= */

/* ====================================================================== Timer Interrupt ====================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	

	if(htim->Instance==TIM3)																												/* Ngat timer 3: 5ms */
	{		
		
		VelTemp		 	= VelCnt;																													/* Gan bien dem van toc vao bien tam */		
		VelCnt 			= 0;																															/* Reset bien dem van toc */	
		CurrSpeed 	= 60*VelTemp/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																			/* RPM */				
		CurrVel 		= 2*3.1415*VelTemp/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																/* rad/s */	
			
		pwm1 = PIController(DesiredVel, CurrVel, Kp, Ki, Kb, UHiLim, ULoLim);
		
		if (run ==true) {
			MotorRot(pwm1);
		}
		
		return;
	}
	
	if(htim->Instance==TIM4)																												/* Ngat timer 4: 10ms */
	{
		tick++;	
		if (tick==5){																									/* 10ms x 5 */
			tick = 0;
			printf("V%d\r \n",CurrSpeed);
		}
	}	
}
/* ====================================================================== End Timer Interrupt ====================================================================== */
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
