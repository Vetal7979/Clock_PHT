#include "main.h"
#include "fatfs.h"
#include "GUI.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "delay.h"
#include "bmp280.h"

extern uint8_t aaa;
extern u8 event_SD;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana16;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana23;

extern uint8_t log_flag;
extern log_t Log;
log_t Log_R[20];

extern uint16_t timeout_wifi,timeout_1pps;
extern uint32_t reset_wifi;

uint32_t start_delay;

CRC_HandleTypeDef hcrc;
//SAI_HandleTypeDef hsai_BlockA1;
//SAI_HandleTypeDef hsai_BlockB1;
DCMI_HandleTypeDef hdcmi;
SD_HandleTypeDef hsd2;
RTC_HandleTypeDef RtcHandle;
SRAM_HandleTypeDef hsram1;
DMA_HandleTypeDef hdma_dcmi;
LPTIM_HandleTypeDef hlptim1;
UART_HandleTypeDef huart1, huart2,huart5, huart7;
SPI_HandleTypeDef hspi2;
IWDG_HandleTypeDef hiwdg1;

extern RTC_TimeTypeDef stimestructureget;
extern RTC_DateTypeDef sdatestructureget;
volatile extern char /*time_set,*/ date_set, date_set2, gps_valid;
uint8_t RxByte, RxByte_U5, RxByte_LoRa, WiFi_Rx;
uint8_t WiFi_Buff[15],LoRa_Buff[10];
uint8_t index_buff, prev_byte, prev_byte_lora, index_lora;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SPI2_Init(void);
static void MX_FMC_Init(void);
static void MX_DCMI_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_IWDG1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_UART_Init(void);
static void MX_UART7_UART_Init(void);
//static void MX_SDMMC2_SD_Init(void);
static void RTC_Init(void);




void board_init(void)
{
  SystemClock_Config();
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_FMC_Init();
  lcd_pwm_init();//
  bright_lcd_set(256);
  __HAL_RCC_DMA1_CLK_ENABLE();
  // MX_DCMI_Init();
  
  /* Initialize GUI */
  GUI_Init();
  GUI_SetFont(&GUI_FontVerdana16);
  LCD_WR_REG(0x36);   //Memory Access Control
  LCD_WR_DATA(0xa0); //       Развернем экран
  GUI_Clear();
  initI2C();
  init_bmp280();
  HAL_Delay(100);
  check_bmp280();
  
  
  
/*  
  MX_SDMMC2_SD_Init();
  MX_FATFS_Init();
*/
  RTC_Init();
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  MX_SPI2_Init();
  MX_LPTIM1_Init();
  MX_USART1_UART_Init();
//  MX_USART2_UART_Init();
//  MX_UART5_UART_Init();
//  MX_UART7_UART_Init();
  
  DWT_Init();
  
  //  Erase_Log(); while(1);

  MX_IWDG1_Init();
}

static void MX_LPTIM1_Init(void)
{


  RCC_PeriphCLKInitTypeDef        RCC_PeriphCLKInitStruct;
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }


  HAL_LPTIM_TimeOut_Start(&hlptim1,65535,65535);




}


static void MX_DCMI_Init(void)
{

  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;

  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }

}

void Sound_Out(void)
    {
       SAI_Out_Init(22050,2);
       InitCodec(1);
    }

void Sound_In(uint8_t st_mo,uint16_t freq, uint16_t sample_per_fr)
    {
       SAI_Out_Init(freq,st_mo);
       SAI_In_Init(st_mo);
       InitCodec(st_mo);
       audio_rec_start(sample_per_fr);
    }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Supply configuration update enable
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY)
  {

  }
  /**Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 336;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 7;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

static void MX_SPI2_Init(void)
{


  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }


}
/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
/*
static void MX_SDMMC2_SD_Init(void)
{


  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 4;

}
*/
static void MX_FMC_Init(void)
{

 FMC_NORSRAM_TimingTypeDef Timing;
  FMC_NORSRAM_TimingTypeDef ExtTiming;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
 /* Timing */
  Timing.AddressSetupTime = 0x8;
  Timing.AddressHoldTime = 0;
  Timing.DataSetupTime = 0x08;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 0;
  Timing.DataLatency = 0;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 8;
  ExtTiming.AddressHoldTime = 0;
  ExtTiming.DataSetupTime = 8;
  ExtTiming.BusTurnAroundDuration = 0;
  ExtTiming.CLKDivision = 0;
  ExtTiming.DataLatency = 0;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SetFMCMemorySwappingConfig(FMC_SWAPBMAP_SDRAM_SRAM);
}

/**
  * @brief IWDG1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG1_Init(void)
{
/* LSI = 32 кГц
   Reload = (LSI*T)/Prescaler, где Т - в секундах
   Например для IWDG 250 ms Reload = (32 000 * 0,25)/256 = 31,25 -> 31
*/
  

  hiwdg1.Instance = IWDG1;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg1.Init.Window = 125;
  hiwdg1.Init.Reload = 125;
  if (HAL_IWDG_Init(&hiwdg1) != HAL_OK)
  {
    Error_Handler();
  }


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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  
  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Detect_SDIO_Pin */
  GPIO_InitStruct.Pin = sd_present_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(sd_present_GPIO_Port, &GPIO_InitStruct);



    /*Configure GPIO pin : Detect_SDIO_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

 
  GPIO_InitStruct.Pin = cs_spi_flash_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(cs_spi_flash_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(cs_spi_flash_GPIO_Port,cs_spi_flash_Pin,GPIO_PIN_SET);
/*
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/
  GPIO_InitStruct.Pin = GPIO_PIN_0 /*| GPIO_PIN_4 | GPIO_PIN_5*/;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5; // кнопки
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5,GPIO_PIN_SET);
  

    /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 9, 0);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 8, 0); //
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  GPIO_InitStruct.Pin = PPS_GPIO_Pin | PPS2_GPIO_Pin; // GPIOB1 - 1PPS
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PPS_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0); // 8
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0); // 8
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
   

  GPIO_InitStruct.Pin = WiFi_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WiFi_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(WiFi_Port, WiFi_Pin, GPIO_PIN_RESET);
  
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

   HAL_UART_Receive_IT(&huart1,&RxByte,1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

   HAL_UART_Receive_IT(&huart2,&RxByte,1);

}

void MX_UART5_UART_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }

   HAL_UART_Receive_IT(&huart5,&RxByte_U5,1); 
}

void MX_UART7_UART_Init(void)
{
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.Mode = UART_MODE_RX;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }

   HAL_UART_Receive_IT(&huart7,&RxByte_LoRa,1); 
}
void RTC_Init(void)
{
  __HAL_RTC_RESET_HANDLE_STATE(&RtcHandle);
  RtcHandle.Instance            = RTC;
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  
}

static const uint16_t wCRCTable[] = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

// подсчет црц

uint16_t CRC16(uint8_t *nData, int wLength)
{
  

  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;

  while (wLength--)
  {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}


//------------------------------------------------------------------------------
uint32_t tickstart_1pps = 0;
uint32_t pps1_time, pps2_time;
int diff_pps;
//uint32_t 
uint16_t milliseconds;
uint8_t ms_flag = 0;
uint8_t pps_flag=0;
extern char calib_1st;


//#define T_DEFAULT    1800 
//#define T_MAX       3600


#define T_DEFAULT    3600       // период между подводкой часов (сек)
#define T_MAX        24 * 3600  // максимальное время между калибровками


volatile uint8_t rtc_calib_ok, need_rtc_calib=0;

uint16_t start,stop,diff;
uint8_t Nizm=0;
uint32_t Summ=0;
uint32_t Freq, Freq_S;
int16_t CALM; 
uint32_t CALP;
uint32_t T = T_DEFAULT; 
time_t current_ttime, ttime_to_calib;
uint8_t synchro_rtc=0;
int delta,  delta_ms;
struct tm current_gps_time, current_gps_time2, current_wifi_time, current_lora_time;
time_t gps_t, gps_t2;
time_t wifi_t, lora_t;

volatile extern char gps_valid, date_set;
extern struct tm gps_time, gps_time2;

struct tm wifi_time;
uint16_t wifi_delay, lora_delay;
uint8_t wifi_stat, wifi_valid_data, lora_valid_data;
int delta_wifi=0, delta_lora = 0;

struct tm lora_time;

extern char no_pps;

extern uint8_t show_time_flag;
extern uint8_t alarm_set;

volatile uint32_t delta_summ;
extern uint16_t timeout_1pps;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == PPS2_GPIO_Pin)
  {
    pps2_time = DWT_CYCCNT;
    if (pps1_time>pps2_time)
      diff_pps = abs(pps1_time - pps2_time);
    else
      diff_pps = abs(pps2_time - pps1_time);
  }
  else if (GPIO_Pin == PPS_GPIO_Pin)
  {
    pps1_time = DWT_CYCCNT;
    
    start_delay = HAL_GetTick(); // начинаем считать задержку  
    
    HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN); // время читаем по сигналу 1PPS
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
    current_ttime = rtc_to_timestamp(&sdatestructureget, &stimestructureget); 
    
    current_gps_time.tm_hour = gps_time.tm_hour;
    current_gps_time.tm_min = gps_time.tm_min;
    current_gps_time.tm_sec = gps_time.tm_sec;
    current_gps_time.tm_mday = gps_time.tm_mday;
    current_gps_time.tm_mon = gps_time.tm_mon-1;
    current_gps_time.tm_year = gps_time.tm_year+100;   
    gps_t = mktime(&current_gps_time) + 1 + 3600*3; //
    
    current_gps_time2.tm_hour = gps_time2.tm_hour;
    current_gps_time2.tm_min = gps_time2.tm_min;
    current_gps_time2.tm_sec = gps_time2.tm_sec;
    current_gps_time2.tm_mday = gps_time2.tm_mday;
    current_gps_time2.tm_mon = gps_time2.tm_mon-1;
    current_gps_time2.tm_year = gps_time2.tm_year+100;
    gps_t2 = mktime(&current_gps_time2) + 1 + 3600*3; //

    
    delta = current_ttime - gps_t; // дельта c UTC
    delta_ms = 1000*(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds)/(RTC_SYNCH_PREDIV+1); // текущее значение миллисекунд
    if (delta<0 && delta_ms!=0) { delta_ms = -1*(1000-delta_ms); delta+=1; }
    if (delta>255) delta = 255; else if (delta<-255) delta = -255;
    
    // считаем сколько миллисекунд между двумя сигналами 1PPS
    if (milliseconds == 0 )
    {
      tickstart_1pps = HAL_GetTick();   milliseconds = 1;
    }
    else
    {
      milliseconds = HAL_GetTick() - tickstart_1pps;
      tickstart_1pps = HAL_GetTick();
    }
    
    // калибровка часового кварца
    if (need_rtc_calib == 1)
    {
      if (pps_flag == 0)
      {
        start = HAL_LPTIM_ReadCounter(&hlptim1);
        pps_flag = 1;  
      }
      else
      {
        if(milliseconds > 1050) // 1pps fail 
        {
          pps_flag = 0 ;  Nizm=0; Summ=0;
        }
        
        stop = HAL_LPTIM_ReadCounter(&hlptim1);
        diff = stop-start;
        Summ+=diff; 
        Nizm++;
        
        if (Nizm>=32)
        {
          Freq=Summ/Nizm; Freq_S=Summ;
          Nizm=0; Summ=0;
          CALM = (Freq_S - 0x100000);
          if ( CALM < 0 ) { CALM+=512; CALP = RTC_SMOOTHCALIB_PLUSPULSES_SET; } 
          else { CALP = RTC_SMOOTHCALIB_PLUSPULSES_RESET; }
          if (CALM>=0 && CALM<=0x1FF) 
          {  
            HAL_LPTIM_TimeOut_Stop(&hlptim1);
            HAL_RTCEx_SetSmoothCalib(&RtcHandle, RTC_SMOOTHCALIB_PERIOD_32SEC, CALP, CALM);                   
            HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_RESET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds));
            rtc_calib_ok = 1;
            //  RTC_BKP_DR2 = ((FL[15] 0 0 0 CALP_FL[11] 0 0 CALM[8:0])
            uint16_t DR2 = (uint16_t)( 0x8000 | ((CALP == RTC_SMOOTHCALIB_PLUSPULSES_SET)?0x0800:0x0000) | CALM);
            HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR2, DR2);
            need_rtc_calib = 0;        
          }
          else 
          {
            pps_flag = 0 ;  Nizm=0; Summ=0; // calibration fail
          }
        }
        start=stop;     
      }
    }// калибровка кварца
    
    if(synchro_rtc == 1)
    {  
      if (date_set)
      {     
        if (abs(delta) < 1) // если дельта меньше секунды то Т оставляем как было
        { 
          if (delta_ms>=0) 
          { //Спешим?
            HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_RESET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds));//HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_RESET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds));
          }
          else
          { //Отстаем?
            HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_SET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds)); //HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_SET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds));
          }
        } 
        else // уход больше секунды - вычисляем Тновое по алгоритму
        {
          // алгоритм!!      
          delta_summ = abs(delta)*1000 + abs(delta_ms);
          if (delta_summ!=0) T = (uint32_t)((T*1000)/abs(delta_summ)); else T = T_MAX;
          
          if (T<T_DEFAULT) T=T_DEFAULT;  
          if (T>T_MAX) T=T_MAX;
          
          RTC_CalendarConfig(current_gps_time);
          HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN); 
          HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);  
          HAL_RTCEx_SetSynchroShift(&RtcHandle,RTC_SHIFTADD1S_RESET,(RTC_SYNCH_PREDIV-stimestructureget.SubSeconds));
        }
        
        current_ttime = rtc_to_timestamp(&sdatestructureget, &stimestructureget);
        
        /* в этом месте мы вычисляем новое Т первый раз после включения, определяя перед этим уход за 1 час.  */
        if (calib_1st!=0)
        {
          if (calib_1st == 1) { T = T_DEFAULT; calib_1st = 2; }   // следующая калибровка после включения через час
          else if (calib_1st == 2)                                // вторая калибровка, вычисляем Т_новое
          {
           delta_summ = abs(delta)*1000 + abs(delta_ms);
           if (delta_summ!=0) T = (uint32_t)((T*1000)/abs(delta_summ)); else T = T_MAX;
           if (T<T_DEFAULT) T=T_DEFAULT; else if (T>T_MAX) T=T_MAX;      
           calib_1st = 0;                                        // дальше калибровка только по алгоритму
          }  
        }
        // log
        Log.delta_ms = 1000 * delta + delta_ms;
        
      }// date_set
      else  // если нет GPS, то перекалибруемся через час 
      { 
        T = T_DEFAULT;
        Log.delta_ms = 0xFFFF;  // флаг неудачной калибровки
      }  
      
      // взводим будильник на следующую калибровку
      HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN); 
      HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN); 
      current_ttime = rtc_to_timestamp(&sdatestructureget, &stimestructureget);
      RTC_AlarmConfig((time_t)(current_ttime + (T-1))); // -1 т.к. прибор проснется, а калибранется в следующую секунду

      alarm_set = 1;
      synchro_rtc = 0; 
      
      
      
      // log!!!   
      Log.start_calib = current_ttime; // во сколько калибровались по RTC (уже подведенному по GPS)
      Log.T_new = T;
      log_flag=1; // записать лог
      
    }
    
    show_time_flag=1;
  } 
  else if(GPIO_Pin == GPIO_PIN_6)
  {
    /*
    event_SD = 1;
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    */
  }
}

/**
  * @brief  Alarm callback
  * @param  hrtc : RTC handle
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Turn LED2 on: Alarm generation */
  synchro_rtc = 1;
  if ( no_pps == 1)
  {
    HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN); 
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
    current_ttime = rtc_to_timestamp(&sdatestructureget, &stimestructureget); 
    RTC_AlarmConfig((time_t)(current_ttime + (T_DEFAULT-1))); // -1 т.к. прибор проснется, а калибранется в следующую секунду
    alarm_set = 1;
    synchro_rtc = 0;     
    Log.start_calib = current_ttime;
    Log.stop_calib = 0;
    Log.gps_time = 0;
    Log.delta_ms = 0;
    Log.T_new = T_DEFAULT;
    log_flag = 1;
  }

}
extern uint32_t timeout_lora_start;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  if (UartHandle->Instance == USART1)
  {
    gps_received(RxByte);
    HAL_UART_Receive_IT(&huart1,&RxByte,1);
  }
  else if (UartHandle->Instance == USART2)
  {

  }
  else if(UartHandle->Instance == UART5)
  {

  }
  else if(UartHandle->Instance == UART7)
  {
    
  }
}






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
