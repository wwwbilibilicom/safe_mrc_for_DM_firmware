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
#include "adc.h"
#include "dma.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_mrc.h"
#include "stdio.h"
#include "string.h"
#include "flash.h"
#include "mrc_debugcli.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Device_MRC_t MRC;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  //SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  //SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  MRC_Init((uint8_t *)"safeMRC", &MRC, 1);
  // 初始化UART4调试指令集
  MRC_DebugCLI_Init(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(MRC.control_loop_flag == 1) // 1kHz contorl loop
    {
      MRC.control_loop_flag = 0;
      MRC_StateMachine_Task1ms(&MRC.statemachine);
      // printf("Actual coil current, filtered coil current: %.3f, %.3f\n", MRC.actual_coil_current, MRC.filtered_coil_current);
      //      MRC.print_count++;
      //      if(MRC.print_count == 1000)
      //      {
        //        MRC.print_count = 0;
        //        printf("angle1, angle2, freq: %.6f, %.6f, %.1f\n", MRC.Encoder.raw_continuous_angle, MRC.Encoder.filtered_angle, MRC.Encoder.real_freq);
        //      }
        // printf("Coil current, AnglarVelocity, Encoder postion(deg) and filtered_angle(deg) with period and high_time: %.3f, %.3f, %.3f, %.3f, %d, %d\n", MRC.actual_coil_current, MRC.Encoder.AngularVelocity, MRC.Encoder.raw_angle, MRC.Encoder.filtered_angle, MRC.Encoder.Encoder_Duty.Period, MRC.Encoder.Encoder_Duty.HighTime);
        MRC_CoilCurrentControl_Update(&MRC);
      }
    if(MRC.coil_current_update_flag == 1) // 10kHZ coil current update
    {
      // if(MRC.Encoder.Encoder_Duty.CapFlag == 1)
      // {
      //   Encoder_Calibrate_n_Filter(&MRC.Encoder);
      // }
      
      MRC_Com_Process(&MRC);
      Encoder_Calibrate_n_Filter(&MRC.Encoder);
      //MRC_collision_detect(&MRC);
      MRC.filtered_coil_current = MRC_Update_Coil_Current(&MRC);
      MRC.coil_current_update_flag = 0;
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{
			CaptureDutyCycle(&MRC.Encoder);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
        if (MRC.control_loop_flag == 0)
        {
          MRC.control_loop_flag = 1;
        }
    }
  if (htim->Instance == TIM4)
  {
    MRC.coil_current_update_flag = 1;
  }

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    MRC_Com_Rx_Enable();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    RS485_HandleUartError(huart, MRC.com.cmd_msg_buffer, MRC.com.cmd_buffer_len);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    /* Clear ORE/FE first */
    // if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    // {
    //   __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
    // }
    // if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET)
    // {
    //   __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_FEF);
    // }

    if(huart->RxEventType == HAL_UART_RXEVENT_IDLE)
    {
      MRC.com.rx_time = getHighResTime_ns();

      /* Only accept complete 8-byte command for this device with valid CRC */
      if (Size == MRC.com.RxLen &&
          MRC.com.cmd_msg_buffer[0] == 0xFE &&
          MRC.com.cmd_msg_buffer[1] == 0xEE &&
          MRC.com.cmd_msg_buffer[2] == MRC.com.id)
      {
        memcpy(&MRC.com.cmd_msg, MRC.com.cmd_msg_buffer, MRC.com.RxLen);
        uint16_t crc_calculated = crc_ccitt(0xFFFF, (uint8_t *)&MRC.com.cmd_msg, MRC.com.RxLen - 2);
        uint16_t crc_received = MRC.com.cmd_msg.CRC16Data;
        if (crc_calculated == crc_received)
        {
          MRC.com.RxFlag = 1;
          MRC_send_data(&MRC);
        }
      }

      /* Smooth re-arm: if RXNE active, delay a short while to avoid mid-frame */
      if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
      {
        for (volatile int i = 0; i < 800; ++i) { __NOP(); }
      }

      /* Rearm ReceiveToIdle DMA for next frames */
      memset(MRC.com.cmd_msg_buffer, 0, MRC_CMD_MSG_BUFFER_SIZE);
      if(HAL_UARTEx_ReceiveToIdle_DMA(MRC.com.mrc_huart, MRC.com.cmd_msg_buffer, MRC_CMD_MSG_BUFFER_SIZE) != HAL_OK)
      {
        printf("ReceiveToIdle DMA failed!\n");
      }
      __HAL_DMA_DISABLE_IT(MRC.com.mrc_huart->hdmarx, DMA_IT_HT);
    }
  }
}

int fputc(int ch, FILE *f)
{

	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);

	return ch;
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
