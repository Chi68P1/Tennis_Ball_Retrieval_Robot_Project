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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t buffer[64];
uint8_t run;
int16_t DesiredSpeedL;
int16_t DesiredSpeedR;
float DesiredVelL;
float DesiredVelR;
char Rx_data[2];
char Rx_Buffer[20];
char Rx_indx;

int16_t pwmL;
int16_t pwmR;


/* Parameter of PIController */
float 		Kp 							= 1.609746269;				/* Ti le */
float 		Ki 							= 29.85074627;				/* Tich phan */
float 		Kd 							= 0.0;								/* Vi phan */
float			Kb 							= 18.54375863;				/* Dung cho Anti-windup */

int32_t 	UHiLim 					= 100;								/* Gioi han tren pwm */
int32_t 	ULoLim 					= -100;									/* Gioi han duoi pwm */

/* Parameter of HAL_TIM_PeriodElapsedCallback */
int16_t 	VelCntL     		= 0;			/* Bien dem encoder => tinh van toc */
float 		CurrVelL 				= 0;	 		/* Van toc hien tai (Rad/s) */
int16_t 	CurrSpeedL	 			= 0;	 		/* Van toc hien tai (RPM) */

int16_t 	VelCntR      		= 0;			/* Bien dem encoder => tinh van toc */
float 		CurrVelR 				= 0;	 		/* Van toc hien tai (Rad/s) */
int16_t 	CurrSpeedR	 			= 0;	 		/* Van toc hien tai (RPM) */

uint16_t 	tick						= 0;			/* Dung de dem cho timer 5 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
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
int32_t PIControllerL(float desired_val, float real_val, float gain, float Ki, float Kb, int uHiLim, int uLoLim)
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

int32_t PIControllerR(float desired_val, float real_val, float gain, float Ki, float Kb, int uHiLim, int uLoLim)
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
void MotorLeftRot(int32_t pwmSet)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
	if (pwmSet > 0) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwmSet);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
	}
	else if (pwmSet < 0) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, -pwmSet);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	}
}
void MotorRightRot(int32_t pwmSet)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
	if (pwmSet > 0) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmSet);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
	}
	else if (pwmSet < 0) {
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, -pwmSet);
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// start  timer 1 	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	// start 	timer 1
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// start  timer 1 	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);	// start 	timer 1
	HAL_TIM_Base_Start_IT(&htim3);						// start  timer 3
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // start encode mode
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // start encode mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
			//MotorRightRot(50);
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 11;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ====================================================================== Timer Interrupt ====================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	

	if(htim->Instance==TIM3)																												/* Ngat timer 3: 5ms */
	{		
		DesiredVelL = DesiredSpeedL*2*3.1415/60;
		DesiredVelR = DesiredSpeedR*2*3.1415/60;
		
		VelCntL			=__HAL_TIM_GET_COUNTER(&htim1);														/* Lay gia tri count velocity */		
		__HAL_TIM_SET_COUNTER(&htim1, 0);                                 		/* Reset cnt */      
		CurrSpeedL 	= 60*VelCntL/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																			/* RPM */				
		CurrVelL 		= 2*3.1415*VelCntL/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																/* rad/s */	
		pwmL = PIControllerL(DesiredVelL, CurrVelL, Kp, Ki, Kb, UHiLim, ULoLim);
		MotorLeftRot(pwmL);
		
		VelCntR			=__HAL_TIM_GET_COUNTER(&htim2);														/* Lay gia tri count velocity */		
		__HAL_TIM_SET_COUNTER(&htim2, 0);                                 		/* Reset cnt */      
		CurrSpeedR 	= 60*VelCntR/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																			/* RPM */				
		CurrVelR 		= 2*3.1415*VelCntR/(4*PULSES_IN_ROUND*UPDATE_PERIOD);																/* rad/s */
		pwmR = PIControllerR(DesiredVelR, CurrVelR, Kp, Ki, Kb, UHiLim, ULoLim);
		MotorRightRot(pwmR);
		
		if (run ==true) {
			tick++;	
			if (tick==5){																									/* 10ms x 5 */
				tick = 0;
				printf("V%d\r \n",CurrSpeedR);
			}
		}
		
		return;
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
