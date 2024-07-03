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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdbool.h>
#include<stdlib.h>
#include<stdio.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
bool continueIt = false;	// Variable for Blocking in Delay-Function
bool eyesInverted = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void DelayTime(uint32_t);
void Hug();
void EatCookie();
void BeUncomfortable();
void FlashEyesSynchronous(int);
void FlashEyesAsynchronous(int, int);
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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

// Hardcoded Prescaler, Autoreload and Capturecompare Registervalues for TIM1 and TIM3
	TIM1->PSC = 240;
	TIM1->ARR = 400;
	TIM1->CCR4 = 100;
	TIM3->PSC = 240;
	TIM3->ARR = 400;
	TIM3->CCR4 = 100;

// Start Timers and PWM Signal of TIM1 and TIM3
	HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

// Addressvariables for I2C-Commuication
	uint16_t gyroAddress = 0x21<<1;
	uint16_t gyroRegister = 0x01;
	uint16_t gyroControllRegister = 0x13;
	uint16_t magAddress = 0x1F<<1;
	uint16_t magRegister = 0x33;
	uint16_t magControllRegister = 0x2A;
	uint16_t magMagControllRegister = 0x5B;

// Datavariables for I2C-Communication
	uint8_t gyroData[6] = {0,0,0,0,0,0};									// Data-Buffer Gyroscope
	uint8_t gyroControllData[2] = {gyroControllRegister,0b00000010};		// Data to be sent to Gyro CTRL_REG1

	uint8_t magData[6] = {0,0,0,0,0,0};										// Data-Buffer Magnetometer
	uint8_t magControllData[2] = {magControllRegister,0b00000001};			// Data to be sent to Magnetometer CTRL_REG1
	uint8_t magMagControllData[2] = {magMagControllRegister,0b00000001};	// Data to be sent to Magnetometer M_CTRL_REG1

	HAL_StatusTypeDef statusGyro;
	HAL_StatusTypeDef statusMag;

	//int16_t gyroX;
	int16_t gyroY;
	//int16_t gyroZ;

	int16_t magX;
	int16_t magY;
	int16_t magZ;
	uint16_t magAbs;

// Configure Gyroscope and Magnetometer via I2C-Communication
	statusGyro = HAL_I2C_Master_Transmit(&hi2c1, gyroAddress, &gyroControllData[0], 2, 100);
	statusMag = HAL_I2C_Master_Transmit(&hi2c1, magAddress, &magControllData[0], 2, 100);
	statusMag = HAL_I2C_Master_Transmit(&hi2c1, magAddress, &magMagControllData[0], 2, 100);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	// Read Status from Gyroscope and Magnetometer
		statusGyro = HAL_I2C_IsDeviceReady(&hi2c1, gyroAddress, 1, 100);
		statusMag = HAL_I2C_IsDeviceReady(&hi2c1, magAddress, 1, 100);

	// If Status OK read AngularRate Data from Gyroscope DataOutputRRegisters
		if(statusGyro == HAL_OK){
			HAL_I2C_Mem_Read(&hi2c1, gyroAddress, gyroRegister, 1, &gyroData[0], 6, 100);
			//gyroX = (gyroData[0]<<8)+gyroData[1];

		// Convert 8MSB and 8LSB to 16 bit signed integer
			gyroY = (gyroData[2]<<8)+gyroData[3];
			//gyroZ = (gyroData[4]<<8)+gyroData[5];

		// React if agularRate is higher than threshhold
			if(abs(gyroY)>2000){
				BeUncomfortable();
			}
		}

	// If Status OK read Magnetometer Data from DataOutputRRegisters
		if(statusMag == HAL_OK){
			HAL_I2C_Mem_Read(&hi2c1, magAddress, magRegister, 1, &magData[0], 6, 100);

		// Convert 8MSB and 8LSB to 16 bit signed integer
			magX = (magData[0]<<8)+magData[1];
			magY = (magData[2]<<8)+magData[3];
			magZ = (magData[4]<<8)+magData[5];

		// React if magnetic field is stronger than threshhold
			magAbs = (uint16_t)(abs(magX)+abs(magY)+abs(magZ));
			if(magAbs>5000){
				EatCookie();
			}
		}

	// Delay to reduce workload on sensors and I2C-Bus
		DelayTime(500);
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	htim1.Init.Prescaler = 48000;
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
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 4800;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2499;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4800;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BUTTON_1_Pin */
	GPIO_InitStruct.Pin = BUTTON_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_BLUE_Pin LED_GREEN_Pin */
	GPIO_InitStruct.Pin = LED_BLUE_Pin|LED_GREEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

// Disable Interrupts on line EXTIO_1
	HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);

// React to button press
	Hug();

// Clear pending interrupts and Enable Interrupts on line EXTIO_1 again
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
// turn Off Green LED on falling Edge of PWM signal (TIM1)
	if(htim == &htim1){

			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET);
	} else if(htim == &htim3){


// turn Off Blue LED on falling Edge of PWM signal (TIM3) (turn on if in eyesInverted mode)
		if(!eyesInverted){

			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_RESET);
		} else {

			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_SET);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
// turn On Green LED on rising Edge of PWM signal (TIM1)
	if (htim == &htim1){

		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	} else if(htim == &htim3){

// turn On Blue LED on rising Edge of PWM signal (TIM3) (turn off if in eyesInverted mode)
		if(!eyesInverted){

			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_SET);
		} else {

			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_RESET);
		}
// set Flag to exit infinite loop in Delay function of rising Edge of PWM-Signal (TIM2)
	} else if (htim == &htim2){

		continueIt = true;
	}
}

void DelayTime(uint32_t delay)
{
	continueIt = false;
// Configure and Start Timer (TIM2) for Delay-Function
	TIM2->PSC = 4800;
	TIM2->ARR = delay*10;
	TIM2->CCR1 = delay*10;
	TIM2->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim2);

// Enter infinite loop for blocking
	while(!continueIt){

	}

// Stop Timer (TIM2)
	HAL_TIM_Base_Stop_IT(&htim2);
}

void Hug(){
// Stop TIMERs and PWM-Signals TIM1 and TIM3 to fully turn on/off the Green and Blue LED
	HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim3);

// Ensure that Green and Blue LEDs are on
	if (HAL_GPIO_ReadPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin,GPIO_PIN_SET);
	}
	if (HAL_GPIO_ReadPin(LED_GREEN_GPIO_Port, LED_GREEN_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
	}

// Wait
	DelayTime(500);

// Start Timers and PWM-Signals TIM1 and TIM3 again
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

}

void EatCookie(){

	HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);

	FlashEyesSynchronous(2500);

	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_1_Pin);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void BeUncomfortable(){

	HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
	eyesInverted = true;

	FlashEyesAsynchronous(4000,3);

	eyesInverted = false;

	__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_1_Pin);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void FlashEyesSynchronous(int duration){

	int initialCCR = TIM1->CCR4;
	int maximumCCR = TIM1->ARR;
	int stepCount = (maximumCCR-initialCCR)/10;
	int delayBetweenSteps = duration / stepCount / 2;

	for (int i = 0; i < stepCount; i++){
		TIM1->CCR4 = initialCCR + i*10;
		TIM3->CCR4 = TIM1->CCR4;
		DelayTime(delayBetweenSteps);
	}

	for (int i = 0; i < stepCount; i++){
		TIM1->CCR4 = maximumCCR-i*10;
		TIM3->CCR4 = TIM1->CCR4;
		DelayTime(delayBetweenSteps);
	}

	TIM1->CCR4 = initialCCR;
	TIM3->CCR4 = TIM1->CCR4;
}

void FlashEyesAsynchronous(int duration, int flashCount){
	int initialCCR1 = TIM1->CCR4;
	int maximumCCR = TIM1->ARR;

	int initialCCR3 = maximumCCR-initialCCR1;

	int medianCCR = maximumCCR/2;
	int stepCountIni = (medianCCR - initialCCR1)/10;
	int stepCount = medianCCR / 10;
	int delayBetweenSteps = duration / (2*stepCountIni + 2*stepCount + 4*stepCount*flashCount);

	for(int i = 0; i<stepCountIni; i++){
		TIM1->CCR4 = initialCCR1+i*10;
		TIM3->CCR4 = initialCCR3-i*10;
		DelayTime(delayBetweenSteps);
	}

	for(int i = 0; i<stepCount; i++){
		TIM1->CCR4 = medianCCR-i*10;
		TIM3->CCR4 = medianCCR-i*10;
		DelayTime(delayBetweenSteps);
	}

	for (int c = 0; c < flashCount; c++){
		for(int i = 0; i<2*stepCount; i++){
			if(i == 0){
				TIM1->CCR4 = 1;
				TIM3->CCR4 = 1;
			} else {
				TIM1->CCR4 = i*10;
				TIM3->CCR4 = i*10;
			}
			DelayTime(delayBetweenSteps);
		}

		for(int i = 0; i<2*stepCount; i++){
			TIM1->CCR4 = maximumCCR - i*10;
			TIM3->CCR4 = maximumCCR - i*10;
			DelayTime(delayBetweenSteps);
		}
	}

	for(int i = 0; i<stepCount; i++){
		if(i == 0){
			TIM1->CCR4 = 1;
			TIM3->CCR4 = 1;
		} else {
			TIM1->CCR4 = i*10;
			TIM3->CCR4 = i*10;
		}
		DelayTime(delayBetweenSteps);
	}

	for(int i = 0; i<stepCountIni; i++){
		TIM1->CCR4 = medianCCR-i*10;
		TIM3->CCR4 = medianCCR+i*10;
		DelayTime(delayBetweenSteps);
	}

	TIM1->CCR4 = initialCCR1;
	TIM3->CCR4 = initialCCR1;
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
