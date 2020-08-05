#include "main.h"



#define  PERIOD_VALUE       (uint32_t)(500 - 1)  /* Period Value  */
#define  PULSE1_VALUE       (uint32_t)(PERIOD_VALUE/8)        /* Capture Compare 1 Value       (PERIOD_VALUE/8)           */
 extern uint16_t ee_time_from_sat,ee_bright,ee_volume,ee_utc,ee_sound_onoff;
TIM_HandleTypeDef    PWMTimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;



void bright_lcd_set(u16 divider)
    {
      if (divider ==0) divider = 1;
    PWMTimHandle.Instance = TIMx;

  PWMTimHandle.Init.Prescaler         = uhPrescalerValue;
  PWMTimHandle.Init.Period            = PERIOD_VALUE ;
  PWMTimHandle.Init.ClockDivision     = 0;
  PWMTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  PWMTimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&PWMTimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;

  /* Set the pulse value for channel 1 */
 // sConfig.Pulse = PULSE1_VALUE;

        sConfig.Pulse = (uint32_t)(divider);
  if (HAL_TIM_PWM_ConfigChannel(&PWMTimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
   if (HAL_TIM_PWM_Start(&PWMTimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

    }





void fade_bright(void)
{
  char z;
  for (z = ee_bright;z <64 ; z++)
  {
    bright_lcd_set(z);
    HAL_Delay(30);
  }
}

void lcd_pwm_init(void)
{
 volatile uint32_t pres;
// Compute the prescaler value to have TIM1 counter clock equal to 20000000 Hz

uhPrescalerValue = (uint32_t)(SystemCoreClock / (8500000)) - 1;


  PWMTimHandle.Instance = TIMx;

  PWMTimHandle.Init.Prescaler         = uhPrescalerValue;
  PWMTimHandle.Init.Period            = PERIOD_VALUE ;
  PWMTimHandle.Init.ClockDivision     = 0;
  PWMTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  PWMTimHandle.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&PWMTimHandle) != HAL_OK)
  {
    //Initialization Error
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/
  /* Common configuration for all channels */
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
 // pres = (uint32_t) (499/a);
  /* Set the pulse value for channel 1 */
  sConfig.Pulse = PULSE1_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&PWMTimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-3- Start PWM signals generation #######################################*/
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&PWMTimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }


}

  void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef   GPIO_InitStruct;
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIMx_CLK_ENABLE();

  /* Enable all GPIO Channels Clock requested */
  TIMx_CHANNEL_GPIO_PORT();
       /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
   __HAL_RCC_TIM2_CLK_ENABLE();

  /* Enable all GPIO Channels Clock requested */
    __HAL_RCC_GPIOA_CLK_ENABLE();
  /* Configure PA.08  (TIM1_Channel1) in output, push-pull, alternate function mode
  */
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = TIMx_GPIO_AF_CHANNEL1;
  GPIO_InitStruct.Pin = TIMx_GPIO_PIN_CHANNEL1;
  HAL_GPIO_Init(TIMx_GPIO_PORT_CHANNEL1, &GPIO_InitStruct);




  /* Configure PE.09 (pin 38 in CN14 connector) (TIM1_Channel1), PE.11 (pin 36 in CN14 connector) (TIM1_Channel2), PE.13 (pin 32 in CN14 connector) (TIM1_Channel3),
     PE.14 (pin 35 in CN14 connector) (TIM1_Channel4) in output, push-pull, alternate function mode
  */
  /* Common configuration for all channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}