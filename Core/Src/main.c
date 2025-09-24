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
#include <string.h> // memset, strlen 등 사용을 위해 추가
#include <stdio.h>  // sscanf 사용을 위해 추가
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
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t CANRxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t CANTxData[8];

uint8_t LPUARTRxData;

#define UART_RX_BUFFER_SIZE 100
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint16_t uart_rx_index = 0;
volatile uint8_t command_received_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);
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
  MX_FDCAN1_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  FDCAN_Config();
  HAL_UART_Receive_IT(&hlpuart1, &LPUARTRxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (command_received_flag)
    {
      command_received_flag = 0; // 플래그 초기화

      unsigned int id, dlc;
      unsigned int data[8] = {0};

      // sscanf를 사용하여 문자열 파싱
      // 예: "1A0 8 11 22 33 44 55 66 77 88"
      int parsed_count = sscanf((char*)uart_rx_buffer, "%x %u %x %x %x %x %x %x %x %x",
                                &id, &dlc, &data[0], &data[1], &data[2], &data[3],
                                &data[4], &data[5], &data[6], &data[7]);

      // 최소 ID와 DLC는 파싱되어야 함 (parsed_count >= 2)
      if (parsed_count >= 2 && dlc <= 8)
      {
        // FDCAN TxHeader 설정
        TxHeader.Identifier = id;
        TxHeader.IdType = FDCAN_STANDARD_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        
        // DLC 값에 따라 DataLength 설정
        switch(dlc) {
            case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
            case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
            case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
            case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
            case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
            case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
            case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
            case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
            case 8: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
            default: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
        }

        // 전송 데이터(CANTxData) 준비
        for (int i = 0; i < dlc; i++)
        {
          CANTxData[i] = (uint8_t)data[i];
        }

        // FDCAN 메시지 전송
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, CANTxData) == HAL_OK)
        {
          // 성공 시 LD2 LED 토글 (전송 확인용)
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(100);
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
      }
      
      // 다음 명령어를 위해 버퍼 초기화
      memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief FDCAN1 설정
 */
static void FDCAN_Config(void) {
    FDCAN_FilterTypeDef sFilterConfig;

    /*Rx Filter*/
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterID1 = 0xD4; // 제공받은 모터 CAN ID: 212 = 0xD4
    sFilterConfig.FilterID2 = 0x7FF;

    /*Error Handle*/
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();

    /*TxHeader*/
    TxHeader.Identifier = 0x01; // 제어장치 ID: 1
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
}

/**
 * @brief  FDCAN DataLength 코드를 실제 바이트 수로 변환
 * @param  DataLength FDCAN_RxHeaderTypeDef의 DataLength 필드
 * @retval uint8_t 실제 데이터 바이트 수 (0-8)
 */
static uint8_t DLC_decoder(uint32_t DataLength)
{
    switch(DataLength)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}

/**
 * @brief FDCAN1 인터럽트 수신받은 걸 LPUART로 전송
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CANRxData) != HAL_OK)
        {
            Error_Handler();
        }

        static char tx_buf[100]; // UART 전송을 위한 버퍼
        int len = 0; // 버퍼에 쓰여진 길이
        uint8_t dlc = DLC_decoder(RxHeader.DataLength); // DLC 값을 바이트 수로 변환

        // "ID: 0x123, DLC: 8, Data: " 부분 생성
        len += sprintf(tx_buf + len, "ID: %#04lX, DLC: %u, Data: ",
                       (unsigned long)RxHeader.Identifier, (unsigned int)dlc);

        // 수신된 데이터를 16진수 문자열로 변환
        for (int i = 0; i < dlc; i++)
        {
            len += sprintf(tx_buf + len, "%02X ", CANRxData[i]);
        }

        len += sprintf(tx_buf + len, "\r\n"); // 줄바꿈 추가

        // 3. UART 전송 1회만 호출
        HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)tx_buf, len);
    }
}

/**
  * @brief  UART 수신 완료 콜백
  * @param  huart UART 핸들
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1)
  {
    // --- 수정된 에코 및 줄바꿈 로직 ---
    if (LPUARTRxData == '\r') // Enter(CR) 키를 수신한 경우
    {
        // 터미널에 CR+LF(\r\n)를 보내 커서를 다음 줄 맨 앞으로 이동
        char newline[] = "\r\n";
        HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)newline, 2);
    }
    else if (LPUARTRxData != '\n') // Enter(LF) 키는 무시하고, 나머지 문자들은 에코
    {
        HAL_UART_Transmit_IT(&hlpuart1, &LPUARTRxData, 1);
    }

    // --- 명령어 버퍼링 및 플래그 처리 로직 ---
    if (LPUARTRxData == '\r' || LPUARTRxData == '\n') // Enter 키(CR 또는 LF)를 누르면
    {
        if (uart_rx_index > 0) // 버퍼에 내용이 있으면 명령어 처리 플래그 설정
        {
            uart_rx_buffer[uart_rx_index] = '\0';
            command_received_flag = 1;
        }
        uart_rx_index = 0; // 버퍼 내용 유무와 상관없이 인덱스는 항상 초기화
    }
    else if (LPUARTRxData == '\b' || LPUARTRxData == 127) // 백스페이스를 누르면
    {
        if (uart_rx_index > 0)
        {
            uart_rx_index--; // 버퍼 인덱스 감소
        }
    }
    else if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) // 그 외 문자들은 버퍼에 저장
    {
      uart_rx_buffer[uart_rx_index++] = LPUARTRxData;
    }

    // 다시 1바이트 수신을 위해 인터럽트 활성화
    HAL_UART_Receive_IT(&hlpuart1, &LPUARTRxData, 1);
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
#ifdef USE_FULL_ASSERT
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
