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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t time){
 
	__HAL_TIM_SET_COUNTER(&htim6, 0);

	while ((__HAL_TIM_GET_COUNTER(&htim6))<time);

	}
	

void Set_Pin_Output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin){
  
   GPIO_InitTypeDef DHT11_DATA={0};
  
   DHT11_DATA.Pin=GPIO_Pin;
   DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
	 DHT11_DATA.Pull = GPIO_NOPULL;
   DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOx,&DHT11_DATA);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin){

 GPIO_InitTypeDef DHT11_DATA={0};
  
   DHT11_DATA.Pin=GPIO_Pin;
   DHT11_DATA.Mode=GPIO_MODE_INPUT;
   DHT11_DATA.Pull=GPIO_NOPULL;
   DHT11_DATA.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOx,&DHT11_DATA);
 
}

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_2

///////////******  DHT11 FUNCTIONS    ******///////

uint8_t durum=0,Humidity=0,Temperature=0;
uint16_t tempVal=0,humVal=0;
uint8_t dhtVal[2];
uint8_t mData[40]; 
uint16_t mTime1 = 0, mTime2 = 0;
uint16_t mbit = 0;
uint8_t  parityVal = 0, genParity = 0;

uint8_t DHT11_Read (void){
	
  for(int a=0;a<40;a++) mData[a]=0;	
   mTime1 = 0, mTime2 = 0, durum=0, tempVal=0, humVal=0, parityVal = 0, genParity = 0,  mbit = 0;
	
    Set_Pin_Output(DHT11_PORT,DHT11_PIN);
	 HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
    delay(18000);
   	Set_Pin_Input(DHT11_PORT,DHT11_PIN);
	 
	 __HAL_TIM_SET_COUNTER(&htim6, 0);				//set timer counter to zero
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	
	__HAL_TIM_SET_COUNTER(&htim6, 0);	
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
    mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
   
  
	if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	{
	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);	
		return 0;
	}

 	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
   	

  uint8_t i,j;
	for(int j = 0; j < 40; j++)
	{
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		__HAL_TIM_SET_COUNTER(&htim6, 0);
		while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
		mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

		//check pass time in high state
		//if pass time 25uS set as LOW
		if(mTime1 > 20 && mTime1 < 30)
		{
			mbit = 0;
		}
		else if(mTime1 > 60 && mTime1 < 80) //if pass time 70 uS set as HIGH
		{
			 mbit = 1;
		}

		//set i th data in data buffer
		mData[j] = mbit;

	}

	//get hum value from data buffer
	for(int i = 0; i < 8; i++)
	{
		humVal += mData[i];
		humVal = humVal << 1;
	}

	//get temp value from data buffer
	for(int i = 16; i < 24; i++)
	{
		tempVal += mData[i];
		tempVal = tempVal << 1;
	}

	//get parity value from data buffer
	for(int i = 32; i < 40; i++)
	{
		parityVal += mData[i];
		parityVal = parityVal << 1;
	}

	parityVal = parityVal >> 1;
	humVal = humVal >> 1;
	tempVal = tempVal >> 1;

	genParity = humVal + tempVal;
 if(genParity == parityVal)
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_RESET);

	dhtVal[0]= tempVal;
	dhtVal[1] = humVal;

	return 1;
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start(&htim6);
 Set_Pin_Output(GPIOD,GPIO_PIN_13);
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		  durum=DHT11_Read();
if (durum ==1)		Temperature=tempVal; Humidity=humVal; 
	
   HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
  
		HAL_Delay(3000);
	
	 
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
