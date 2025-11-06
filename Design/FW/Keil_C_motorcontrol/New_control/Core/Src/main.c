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
#include "Motor_control.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE 100

uint8_t uart_rx_data;                   // biến nhận từng byte UART
char uart_cmd_buffer[UART_BUFFER_SIZE]; // buffer lưu chuỗi nhận được
uint16_t uart_cmd_index = 0;            // chỉ số lưu vị trí hiện tại trong buffer
volatile uint8_t uart_data_ready = 0;   // cờ báo nhận xong 1 chuỗi dữ liệu
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float toc_do;
uint8_t uart_flag_tx;
volatile uint32_t pulse_count = 0;
//int8_t direction;
uint8_t flag_pulse;
uint8_t flag_timer_100ms;
uint8_t motor_on = 0;
uint8_t controller_type = 0;
uint8_t input_type = 0;
float amp_max = 0, amp_min = 0;
float scaled_speedmras_set;
float scaled_speedmras_speed;
float recovered_speedmras;
uint8_t flag_nguoc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Ngat ngoai dem xung
{
    if (GPIO_Pin == GPIO_PIN_10) // PA10 = EXTI10
    {
        flag_pulse = 1; // Set co bao co xung
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Ngat 100ms tinh toc do
{
    if (htim->Instance == TIM1) //Ngat 100ms tinh toc do
    {
        flag_timer_100ms = 1;
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
							// Danh dau da den chu ky 100ms
    }
		if (htim->Instance == TIM3)		//Ngat 0.2s truyen uart
		{
		uart_flag_tx=1;
		}
}
//Code truyen Uart
void send_uart_3float(UART_HandleTypeDef *huart, float var1, float var2, float var3)
{
    char uart_buffer[100];
    int len;

    // Dinh dang chuoi: "var1,var2,var3\r\n"
    len = sprintf(uart_buffer, "%.2f,%.2f,%.2f\r\n", var1, var2, var3);

    // Truyen qua UART
    HAL_UART_Transmit(huart, (uint8_t*)uart_buffer, len, HAL_MAX_DELAY);
}

void Parse_UART_Command(char* str) {
    char* token;
    uint8_t field = 0;

    token = strtok(str, ",");
    while (token != NULL) {
        switch (field) {
            case 0: motor_on = atoi(token); break;
            case 1: controller_type = atoi(token); break;
            case 2: input_type = atoi(token); break;
            case 3: kp = atof(token); break;
            case 4: ki = atof(token); break;
            case 5: gp = atof(token); break;
            case 6: gi = atof(token); break;
            case 7: amp_max = atof(token); break;
            case 8: amp_min = atof(token); break;
            case 9: set_point = atof(token); break;
            default: break;
        }
				if (set_point <0)
				{
					flag_nguoc =1;
				}
        token = strtok(NULL, ",");
        field++;
    }
}
//Ham ngat nhan Uart - ket thuc bang R --> truyen chuoi nhan dc cho ham xu ly
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  // Đảm bảo là đúng UART bạn đang dùng
    {
        if (uart_rx_data == 'R')  // Gặp ký tự kết thúc chuỗi
        {
            uart_cmd_buffer[uart_cmd_index] = '\0';  // Kết thúc chuỗi
            uart_data_ready = 1;  // Báo hiệu có chuỗi sẵn sàng xử lý
            uart_cmd_index = 0;   // Reset buffer index để nhận chuỗi mới
        }
        else
        {
            if (uart_cmd_index < UART_BUFFER_SIZE - 1)
            {
                uart_cmd_buffer[uart_cmd_index++] = uart_rx_data;
            }
            else
            {
                uart_cmd_index = 0;  // Nếu quá dài, reset tránh tràn
            }
        }

        // Tiếp tục nhận UART (1 byte)
        HAL_UART_Receive_IT(huart, &uart_rx_data, 1);
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile float encoder_count = 0;   //Dem xung
// Ham tinh toc do dong co RPM
float calculate_motor_speed_rpm() {
    const float pulses_per_encoder_rev = 11.0f;
    const float gear_ratio = 21.3f;
    const float sampling_time_sec = 0.1; // 100ms
   
    float encoder_rps = encoder_count / pulses_per_encoder_rev / sampling_time_sec;

    // Toc do truc dau ra
    float motor_rpm = encoder_rps * 60.0f / gear_ratio;

    // Reset encoder count sau m?i chu k?
    encoder_count = 0;

    return motor_rpm;
}
//Ham tu tin hieu dieu khien sang duty
uint16_t percent_to_duty(double input_percent)
{
    uint16_t min_duty = 200;
    uint16_t max_duty = 1000;

		double scaled_duty = min_duty + ((double)input_percent * (max_duty - min_duty) / 100.0);

    return (uint16_t)round(scaled_duty);
}
//Cac ham scale
float scale_to_0_1(float x, float x_min, float x_max)
{
    if (x_max == x_min) return 0.0f; // tránh chia 0
    return (x - x_min) / (x_max - x_min);
}
float scale_to_50_250(float x)
{
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;
    return x * (250.0f - 50.0f) + 50.0f; // = x * 200 + 50
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	Motor_control_initialize();
	HAL_TIM_Base_Start_IT(&htim1); //Tim1 ngat tinh toc do
	HAL_TIM_Base_Start_IT(&htim3); //Tim3 ngat truyen uart
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Khoi dong PWM PA0
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Ngat canh len - dem xung encoder - xac dinh chieu quay
	  if (flag_pulse == 1) 
        {
            flag_pulse = 0;
//		if (HAL_GPIO_ReadPin(GPIOB,ENCODER_PULSE_READ_B) == GPIO_PIN_SET)
//            direction = 1;  // Thu?n
//        else
//            direction = -1; // Ngu?c   //Code phat hien chieu quay
		pulse_count += 1;
				}
	//Ngat timer 100ms - tinh toc do dong co
		if (flag_timer_100ms == 1)
        {
          encoder_count = pulse_count; //So xung sau 0.1s
					toc_do =	calculate_motor_speed_rpm(); // tinh toc do
					pulse_count =0; //Reset xung
					flag_timer_100ms = 0;
        }
	//Ham xu ly uart
	if(uart_data_ready)
{
    Parse_UART_Command(uart_cmd_buffer); // hàm bạn tự định nghĩa để tách chuỗi và xử lý
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    uart_data_ready = 0;                 // reset cờ để tiếp nhận chuỗi mới
}
if (motor_on==1)   //Bat dong co
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		
		if(set_point<0)
		{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
		}
		else{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		//Chon kieu input setpoint
		if(input_type==0){
	set_point=set_point;
	}
else if(input_type==1){
set_point=set_point_sine;
}
else if(input_type==2){
	static uint16_t pulse_counter = 0;
        if (pulse_counter < 15000)
        {
            set_point = amp_max;
        }
        else
        {
            set_point = amp_min;			//Tao Pulse
        }

        pulse_counter++;
        if (pulse_counter >= 30000) 
        {
            pulse_counter = 0;
        }
}
//		if (set_point < 50) {
//    set_point = 50;
//		}			
//		else if (set_point > 250) {
//    set_point = 250;
//		}  //Gioi han set_point
	current_speed = toc_do;
	scaled_speedmras_set = scale_to_0_1(set_point,50,250);
	scaled_speedmras_speed=scale_to_0_1(current_speed,50,250);
  //kp = 0.5;
	//ki = 0.001; //day la thong so PI tunning

	Motor_control_step(); //goi mo hinh simulink tinh toan u dieu khien
		if(controller_type==0)				// Chon dieu khien signal_control - PI hoac signal_controlPImras - PI_MRAS
																	// Doi tin hieu dieu khien sang gia tri nap vao thanh ghi CCR1
{
htim2.Instance->CCR1 = 	percent_to_duty(signal_control);

}
else{
	
	set_point_mras=scaled_speedmras_set;
	current_speed_PImras=scaled_speedmras_speed;
	htim2.Instance->CCR1 = 	percent_to_duty(signal_controlPImras);
}

		
		//////////////////////////////////////////////////////////
	//Gui du lieu qua uart
		if(uart_flag_tx){	
			if(controller_type==0){
		send_uart_3float(&huart1,set_point,current_speed,signal_control);		//PI thuong 
				uart_flag_tx=0;
			}
			else{
			recovered_speedmras = scale_to_50_250(current_speed_model);
			send_uart_3float(&huart1,recovered_speedmras,current_speed,signal_controlPImras); //PImras
				uart_flag_tx=0;
			}
		}
	}
	else
	{
	htim2.Instance->CCR1 = 0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);			//Tat dong co
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
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
  htim3.Init.Prescaler = 799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
