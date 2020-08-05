

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"

extern LPTIM_HandleTypeDef hlptim1;
extern UART_HandleTypeDef huart1, huart2,huart5, huart7;
extern RTC_HandleTypeDef RtcHandle;

//extern SAI_HandleTypeDef hsai_BlockA1;
//extern SAI_HandleTypeDef hsai_BlockB1;
extern __IO int32_t OS_TimeMS;
extern int st_t;
extern volatile int timee;
extern DCMI_HandleTypeDef hdcmi;
extern  int full,half;
extern uint16_t date_set_timeout; //, timeout_2, timeout_wifi, timeout_1pps ;
extern uint32_t bmp_timer, bmp_period;
extern uint8_t bmp_enable;
extern uint32_t reset_wifi;
extern uint8_t view_log;
extern uint16_t timeout_view;

extern uint8_t JpegBuffer[2][JpegBufferLen];//32768/2
uint8_t  fifo_buf [FIFO_LEN];
volatile uint32_t fifo_IN, fifo_OUT;
volatile static uint16_t i=0;

void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
     OS_TimeMS ++;
     if (view_log) timeout_view++; else timeout_view=0;
   if (date_set_timeout<=gps_timeout) date_set_timeout++;  
  if (bmp_timer<bmp_period)  
    bmp_timer++;
  else
  {
    bmp_enable=1;
  }


  /* USER CODE END SysTick_IRQn 1 */
}

extern DMA_HandleTypeDef hdma_dcmi;

void DMA1_Stream0_IRQHandler(void)
{
 // HAL_DMA_IRQHandler(hsai_BlockA1.hdmatx);


  HAL_DMA_IRQHandler(&hdma_dcmi);
}

void EXTI0_IRQHandler(void)
{
HAL_GPIO_EXTI_IRQHandler(PPS2_GPIO_Pin);
}

void EXTI1_IRQHandler(void)
{
HAL_GPIO_EXTI_IRQHandler(PPS_GPIO_Pin);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}
void UART7_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart7);
}

void DCMI_IRQHandler(void)
{
  HAL_DCMI_IRQHandler(&hdcmi);
}

void LPTIM1_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim1);
}

void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&RtcHandle);
}