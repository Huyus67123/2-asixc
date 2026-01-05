/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "math.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
volatile uint16_t x_value = 0;
volatile uint16_t y_value = 0;
volatile uint16_t x_go = 0;
volatile uint16_t y_go = 0;
volatile uint8_t x_done = 0;
volatile uint8_t y_done = 0;
volatile float x_pre = 0;
volatile float y_pre = 0;
char rxByte=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PWM_SetARR_Pulse(TIM_HandleTypeDef *htim, uint32_t arr, uint32_t channel);
void GoHome();
void GoLinerX(float y);
void GoLinerY(float y);
void GoDiagonal(float x, float y);
void DrawSquar(float a);
void DrawRectangle(float a, float b);
void DrawCircle(float a);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);

  void printf_uart(const char *str);
//  DrawRectangle(2, 1);
//  GoDiagonal(4, 4);
//  DrawSquar(4);

//  GoHome();
//  DrawStar();
//  DrawCircle(2);
  GoDiagonal(2.0,1.0);
//  GoLinerY(-7.5);
//  GoLinerX(-7.5);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //DrawSquar(5);
//	  if(done==1){
//		  printf_uart("chon hinh\n");
//		  printf_uart("1.vuong\n");
//		  printf_uart("2.chu nhat\n");
//		  printf_uart("3.ngoi sao\n");
//	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR2_Pin|ENA2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENA1_Pin|DIR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR2_Pin ENA2_Pin */
  GPIO_InitStruct.Pin = DIR2_Pin|ENA2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA1_Pin DIR1_Pin */
  GPIO_InitStruct.Pin = ENA1_Pin|DIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void WaitX() {
	while (!x_done){
		HAL_Delay(1);
	}
	x_done = 0;
}

void WaitY() {
	while (!y_done){
		HAL_Delay(1);
	}
	y_done = 0;
}

void WaitXint(int step){
	while (x_go!=step){
		HAL_Delay(1);
	}
}

void WaitYint(int step){
	while (y_go!=step){
		HAL_Delay(1);
	}
}

void printf_uart(const char *mess) {
    HAL_UART_Transmit(&huart1, (uint8_t *)mess, strlen(mess), 500);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	 if (huart->Instance == USART1){

		 HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxByte, 1);
	 }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==GPIO_PIN_6){
		HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_SET);
	}
	if(GPIO_Pin==GPIO_PIN_7){
		HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_SET);
	}
}

//void PWM_SetARR_Pulse(TIM_HandleTypeDef *htim,uint32_t arr,uint32_t channel)
//{
//	uint32_t pulse=arr/2;
//    __HAL_TIM_SET_AUTORELOAD(htim, arr);   // thay đổi ARR (chu kỳ PWM)
//    __HAL_TIM_SET_COMPARE(htim, channel, pulse); // thay đổi Pulse (duty)
//
//    htim->Instance->EGR = TIM_EGR_UG;       // cập nhật ngay lập tức
//}

void PWM_SetARR_Pulse(TIM_HandleTypeDef *htim, uint32_t arr, uint32_t channel)
{
    if (htim == NULL || arr < 20) return;

    __HAL_TIM_DISABLE(htim);

    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2);

    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG;   // 🔥 CỰC KỲ QUAN TRỌNG

    __HAL_TIM_ENABLE(htim);
}




void GoLinerX(float x){
	if(fabs(x)<=7.5){
		x_go=0;
		x_done = 0;
		if(x-x_pre>=0){
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,SET);
		}
		else if(x-x_pre<0){
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin,RESET);
		}
		x_value=fabs((x-x_pre)*14000/7.5);
		x_pre=x;
		HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	}
}

void GoLinerY(float y){
	if(fabs(y)<=7.5){
		y_go=0;
		y_done = 0;
		if(y-y_pre>0){
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin,SET);
		}
		else if(y-y_pre<0){
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin,RESET);
		}
		y_value=fabs((y-y_pre)*14000/7.5);
		y_pre=y;
		HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin, GPIO_PIN_RESET);
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	}
}

//void GoDiagonal(float dx, float dy)
//{
//    if (fabs(dx) < 0.0001f) { GoLinerY(dy); return; }
//    if (fabs(dy) < 0.0001f) { GoLinerX(dx); return; }
//
//    float sx = fabs(dx);
//    float sy = fabs(dy);
//    float sum = sx + sy;
//
//    float ratioX = sx / sum;
//    float ratioY = sy / sum;
//
//    uint32_t baseARR = 1000 - 1;   // ARR gốc (đang dùng)
//    uint32_t arrX = baseARR / ratioX;
//    uint32_t arrY = baseARR / ratioY;
//
//    if (arrX < 100) arrX = 100;    // chống quá nhanh
//    if (arrY < 100) arrY = 100;
//    if (arrX > 5000) arrX = 5000;  // chống quá chậm
//    if (arrY > 5000) arrY = 5000;
//
//    PWM_SetARR_Pulse(&htim1, arrX, TIM_CHANNEL_1);
//    PWM_SetARR_Pulse(&htim2, arrY, TIM_CHANNEL_1);
//
//    GoLinerX(dx);
//    GoLinerY(dy);
//
//    WaitX();
//    WaitY();
//
//    // trả về ARR gốc
//    PWM_SetARR_Pulse(&htim1, baseARR, TIM_CHANNEL_1);
//    PWM_SetARR_Pulse(&htim2, baseARR, TIM_CHANNEL_1);
//}
void GoDiagonal(float x, float y)
{
    float dx = x - x_pre;
    float dy = y - y_pre;

    if (fabs(dx) < 0.0001f && fabs(dy) < 0.0001f) return;

    // 🔒 XỬ LÝ TRƯỜNG HỢP 1 TRỤC
    if (fabs(dx) < 0.0001f) {
        GoLinerY(y);
        WaitY();
        return;
    }

    if (fabs(dy) < 0.0001f) {
        GoLinerX(x);
        WaitX();
        return;
    }

    float sx = fabs(dx);
    float sy = fabs(dy);

    uint32_t baseARR = 1000;
    uint32_t arrX = baseARR;
    uint32_t arrY = baseARR;

    float maxS = (sx > sy) ? sx : sy;
    arrX = (uint32_t)(baseARR * maxS / sx);
    arrY = (uint32_t)(baseARR * maxS / sy);

    // Giới hạn
    if (arrX < 200) arrX = 200;
    if (arrY < 200) arrY = 200;
    if (arrX > 5000) arrX = 5000;
    if (arrY > 5000) arrY = 5000;

    PWM_SetARR_Pulse(&htim1, arrX, TIM_CHANNEL_1);
    PWM_SetARR_Pulse(&htim2, arrY, TIM_CHANNEL_1);
    HAL_Delay(1);

    GoLinerX(x);
    GoLinerY(y);

    WaitX();
    WaitY();
}




void GoHome(){
	GoLinerX(0);
	GoLinerY(0);
	WaitX();
	WaitY();
//	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_2);
}

void DrawSquar(float a){
    // Cạnh 1
    GoLinerX(a);
    WaitX();          // CHỜ X CHẠY XONG

    // Cạnh 2
    GoLinerY(a);
    WaitY();          // CHỜ Y CHẠY XONG

    // Cạnh 3
    GoLinerX(0);
    WaitX();

    // Cạnh 4
    GoLinerY(0);
    WaitY();
}

void DrawRectangle(float a, float b){
	GoLinerX(a);
	WaitX();

	GoLinerY(b);
	WaitY();

	GoLinerX(-a);
	WaitX();

	GoLinerY(-b);
	WaitY();
}
void DrawCircle(float r)
{
    float cx = 4.0f;
    float cy = 4.0f;

    const int N = 360;          // số đoạn
    const float dT = M_PI / N;

    GoDiagonal(cx + r, cy);
    WaitX();
    WaitY();

    for (int i = 1; i <= N; i++)
    {
        float t = i * dT;
        float x = cx + r * cosf(t);
        float y = cy + r * sinf(t);

//        x = round2(x);
//        y = round2(y);

        GoDiagonal(x, y);
    }

    GoDiagonal(cx + r, cy);   // 🔒 đóng vòng tuyệt đối
}


//void DrawCircle(float r){
//  // vi du tam duong tron (4, 4) cm ban kinh 1 cm
//  float Ix = 4; // toa do x tam duong tron
//  float Iy = 4; // toa do y tam duong tron
//  float radius = r; // ban kinh
//
//	GoLinerX(4.0);
//	GoLinerY(4.0);
//	WaitX();
//
//  float minX = Ix - radius;
//  float maxX = Ix + radius;
//  const float step = 0.01;
//  const float a = 1;
//  float constant = Ix*Ix + Iy*Iy - radius*radius;
//  float currX = minX;
//
//  while (currX < maxX) {
//    float b = -2*Iy;
//    float c = currX*currX - 2*Ix*currX + constant;
//    float delta = b*b - 4*a*c;
//    float y1 = (-b + sqrtf(delta)) / 2*a;
//    GoLinerX(currX);
//    GoLinerY(y1);
//    WaitX();
//    WaitY();
//    currX += step;
//  }
//
//  currX = minX;
//
//  while (currX < maxX) {
//    float b = -2*Iy;
//    float c = currX*currX - 2*Ix*currX + constant;
//    float delta = b*b - 4*a*c;
//    float y2 = (-b - sqrtf(delta)) / 2*a;
//    GoLinerX(currX);
//    GoLinerY(y2);
//    WaitX();
//    WaitY();
//    currX += step;
//  }
//}

void DrawStar(){
	GoDiagonal(2,4);

	GoDiagonal(2,-4);

	GoDiagonal(-4,2);

	GoLinerX(4);
	WaitX();
	GoDiagonal(-4,-2);

}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1 ){
		x_go++;
		if(x_go==x_value){
			HAL_GPIO_WritePin(ENA1_GPIO_Port, ENA1_Pin,SET);
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			//PWM_SetPSC(8,&htim1);
			PWM_SetARR_Pulse(&htim1, 1000, TIM_CHANNEL_1);

			x_done = 1;
		}
	}
	if(htim->Instance == TIM2 ){
		y_go++;
		if(y_go==y_value){
			HAL_GPIO_WritePin(ENA2_GPIO_Port, ENA2_Pin,SET);
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
			//PWM_SetPSC(8,&htim2);
			PWM_SetARR_Pulse(&htim2, 1000, TIM_CHANNEL_1);
			y_done = 1;
		}
	}
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
