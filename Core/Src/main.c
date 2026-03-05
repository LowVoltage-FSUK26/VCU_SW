/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Master file – STM32CubeIDE managed.
  *                   FSUK VCU – FreeRTOS task creation, peripheral init, ISRs.
  *
  *  This file is intentionally minimal. All task implementations live in:
  *    can_dti.c      – DTI HV500 CAN decoder task.
  *    imd_monitor.c  – Bender IMD PWM monitor task.
  *    vcu_safety.c   – APPS safety, brake light, and R2D supervisor tasks.
  *
  *  DO NOT move peripheral init (MX_*) functions out of this file.
  *  STM32CubeIDE re-generates them on every .ioc save; they must stay here.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "can_dti.h"      /* CAN_Msg_t, DtiData_t, vCanDecodeTask()          */
#include "imd_monitor.h"  /* ImdDebugStatus_t, IsrBuffer_t, StartPwmTask()   */
#include "vcu_safety.h"   /* VcuSensorData_t, vBrakeLightTask(), vR2DLogicTask() */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* All typedefs moved to their respective module headers.                     */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* All safety #defines moved to vcu_safety.h.                                */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* ============================================================================
 * FREERTOS HANDLES
 * All handles declared here (single owner). Modules access them via extern.
 * ========================================================================= */
QueueHandle_t xCanRxQueue     = NULL;  /**< CAN RX FIFO queue (depth 10).    */
TaskHandle_t  xCanTaskHandle  = NULL;  /**< vCanDecodeTask handle.            */
TaskHandle_t  xPwmTaskHandle  = NULL;  /**< StartPwmTask handle (IMD notify). */
TaskHandle_t  xBrakeTaskHandle= NULL;  /**< vBrakeLightTask (I2C DMA notify). */
TaskHandle_t  xR2DTaskHandle  = NULL;  /**< vR2DLogicTask handle.             */

/* ============================================================================
 * ADC DMA BUFFER
 * Scan order matches ADC channel rank configuration in MX_ADC1_Init():
 *   [0] = APPS1      (Rank 1, CH2)
 *   [1] = APPS2      (Rank 2, CH3)
 *   [2] = Brake Pres (Rank 3, CH4)
 * ========================================================================= */
volatile uint16_t adc_dma_buffer[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Task prototypes are declared in their respective headers.                  */
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* ==========================================================================
   * 1. CREATE FREERTOS PRIMITIVES
   * ======================================================================== */

  /* CAN RX queue – depth 10 keeps the ISR from stalling under burst traffic. */
  xCanRxQueue = xQueueCreate(10, sizeof(CAN_Msg_t));
  if (xCanRxQueue == NULL) { Error_Handler(); }

  /* ==========================================================================
   * 2. CREATE FREERTOS TASKS
   *
   *  Priority policy:
   *    3 = Safety-critical real-time (brake light, R2D supervisor).
   *    2 = Monitoring / decoding (CAN, IMD PWM).
   *    1 = Background (telemetry, logging – not yet implemented).
   *
   *  Stack sizes:
   *    512 words for CAN decoder (larger switch-case → more stack usage).
   *    256 words for all others (empirically verified with uxTaskGetStackHighWaterMark).
   * ======================================================================== */

  /* CAN decoder task – drains xCanRxQueue and populates dti_data struct. */
  xTaskCreate(vCanDecodeTask,  "DTI_Dec",    512, NULL, 2, &xCanTaskHandle);

  /*
   * PWM task – xPwmTaskHandle MUST be captured so the TIM2 ISR can notify it.
   * Create BEFORE enabling TIM2 interrupts below.
   */
  xTaskCreate(StartPwmTask,    "PWM_Proc",   256, NULL, 2, &xPwmTaskHandle);

  /*
   * Brake light task – xBrakeTaskHandle MUST be captured so the I2C DMA
   * completion ISR can notify it. Create BEFORE starting ADC DMA below.
   */
  xTaskCreate(vBrakeLightTask, "BrakeLight", 256, NULL, 3, &xBrakeTaskHandle);

  /* R2D supervisor – same priority as brake light (both safety critical). */
  xTaskCreate(vR2DLogicTask,   "R2D_Logic",  256, NULL, 3, &xR2DTaskHandle);

  /* ==========================================================================
   * 3. START HARDWARE
   * ======================================================================== */

  /* CAN peripheral – must be started before enabling RX interrupt. */
  if (HAL_CAN_Start(&hcan) != HAL_OK) { Error_Handler(); }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * TIM2 Input Capture – IMD PWM measurement.
   * Priority 6: below FreeRTOS kernel (5) so task notifications work correctly
   * (configMAX_SYSCALL_INTERRUPT_PRIORITY on STM32 = priority 5).
   */
  HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  /*
   * TIM3 PWM output – PA6 / CH1 / RTD buzzer.
   * Started here at 0% duty. vR2DLogicTask is the sole controller of
   * the compare register (__HAL_TIM_SET_COMPARE) and auto-reload.
   */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* MPU-6050 boot initialisation (blocking I2C writes – must be before RTOS). */
  MPU6050_Init();

  /* ADC DMA continuous scan: APPS1, APPS2, brake pressure → adc_dma_buffer. */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 3);

  /* ==========================================================================
   * 4. START SCHEDULER (never returns)
   * ======================================================================== */
  vTaskStartScheduler();

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* Pass-all filter: mask = 0, ID = 0 → all IDs accepted into FIFO0. */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation     = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank           = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh         = 0;
  canfilterconfig.FilterIdLow          = 0;
  canfilterconfig.FilterMaskIdHigh     = 0;
  canfilterconfig.FilterMaskIdLow      = 0;
  canfilterconfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) { Error_Handler(); }

  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(CAN_HEART_LED_GPIO_Port, CAN_HEART_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_Pin|DRIVE_ENABLE_Pin|FAN_BATTERY_Pin|FAN_MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_HEART_LED_Pin */
  GPIO_InitStruct.Pin = CAN_HEART_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_HEART_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FAULT_LED_Pin */
  GPIO_InitStruct.Pin = FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FAULT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMD_OKHS_Pin */
  GPIO_InitStruct.Pin = IMD_OKHS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMD_OKHS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRAKE_LIGHT_Pin DRIVE_ENABLE_Pin FAN_BATTERY_Pin FAN_MOTOR_Pin */
  GPIO_InitStruct.Pin = BRAKE_LIGHT_Pin|DRIVE_ENABLE_Pin|FAN_BATTERY_Pin|FAN_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RTD_BUTTON_Pin */
  GPIO_InitStruct.Pin = RTD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RTD_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDC_SENSE_Pin */
  GPIO_InitStruct.Pin = SDC_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SDC_SENSE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ============================================================================
 * HARDWARE INTERRUPT CALLBACKS
 *
 * These callbacks are called from within HAL ISR handlers. They run at
 * interrupt priority and must be kept short. Their only job is to pass data
 * to FreeRTOS primitives and yield if a higher-priority task was woken.
 *
 * They live in main.c (not in module .c files) because:
 *   a) HAL weak-symbol overrides must exist in exactly one translation unit.
 *   b) They reference multiple module globals (xCanRxQueue, xPwmTaskHandle,
 *      xBrakeTaskHandle) which are all owned here.
 * ========================================================================= */

/**
 * @brief  CAN RX FIFO0 message pending callback.
 *
 * Called by USB_LP_CAN1_RX0_IRQHandler → HAL_CAN_IRQHandler whenever a new
 * frame arrives in FIFO0. Reads the frame and enqueues it for vCanDecodeTask.
 *
 * xQueueSendFromISR + portYIELD_FROM_ISR ensures that if vCanDecodeTask is
 * higher priority than the currently running task, a context switch happens
 * immediately on ISR exit rather than waiting for the next tick.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_local)
{
    CAN_RxHeaderTypeDef RxHeader;
    CAN_Msg_t msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (HAL_CAN_GetRxMessage(hcan_local, CAN_RX_FIFO0, &RxHeader, msg.Data) == HAL_OK)
    {
        msg.StdId = RxHeader.StdId;
        xQueueSendFromISR(xCanRxQueue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  TIM input capture callback – TIM2 PWM period/pulse measurement.
 *
 * Called when TIM2 CH1 captures a rising edge (slave-reset fires).
 * At this point CCR1 holds the full period and CCR2 holds the pulse width
 * (both updated atomically by hardware before the interrupt fires).
 *
 * Notifies StartPwmTask via direct-to-task notification (binary semaphore
 * behaviour: one notification per capture pair, task processes one at a time).
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        /* Snapshot CCR registers – written atomically by TIM2 hardware. */
        isr_buffer.period = TIM2->CCR1;
        isr_buffer.pulse  = TIM2->CCR2;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xPwmTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  I2C memory read DMA completion callback – MPU-6050 data ready.
 *
 * Called by DMA1_Channel7_IRQHandler → HAL_DMA_IRQHandler when the 14-byte
 * accelerometer read initiated by vBrakeLightTask completes.
 *
 * Notifies vBrakeLightTask, which then processes the data and re-arms the
 * next DMA transfer at the end of its 10 ms loop iteration.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c_local)
{
    if (hi2c_local->Instance == I2C1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xBrakeTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
