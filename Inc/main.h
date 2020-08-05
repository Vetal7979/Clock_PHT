

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "ili9806.h"
#include "gps.h"
#include "calendar.h"
#include "log.h"
  

#define gps_timeout 1500
#define RTC_CLOCK_SOURCE_LSE
  
  typedef char* string;
  
  #define bool u8

#define FALSE  0
#define TRUE  !FALSE
  
#define key1_port       GPIOC
#define key2_port       GPIOC
#define key1_pin        GPIO_PIN_4
#define key2_pin        GPIO_PIN_5
  
#define WiFi_Port      GPIOC
#define WiFi_Pin       GPIO_PIN_10
#define WiFi_ON        HAL_GPIO_WritePin(WiFi_Port, WiFi_Pin, GPIO_PIN_SET);
#define WiFi_OFF       HAL_GPIO_WritePin(WiFi_Port, WiFi_Pin, GPIO_PIN_RESET);
  
  #define row_on_page     17
  

#define key1_press      HAL_GPIO_ReadPin(key1_port,key1_pin)==GPIO_PIN_RESET
#define key2_press      HAL_GPIO_ReadPin(key2_port,key2_pin)==GPIO_PIN_RESET 
  
#define full_len 32768
#define JpegBufferLen full_len/2
#define FIFO_LEN 65536
#define FIFO_MASK (FIFO_LEN-1)
  
  #define    DWT_CYCCNT    *(volatile uint32_t*)0xE0001004
  #define    DWT_CONTROL   *(volatile uint32_t*)0xE0001000
  #define    SCB_DEMCR     *(volatile uint32_t*)0xE000EDFC
  
#define cs_spi_flash_GPIO_Port  GPIOB
#define cs_spi_flash_Pin        GPIO_PIN_11
  

/* Exported constants --------------------------------------------------------*/
typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF,
  BUFFER_OFFSET_FULL,
}BUFFER_StateTypeDef;

#define PLAY_BUFF_SIZE     2048

typedef struct WAVHEADER_t
{
	// идентификатор формата файла
	char riff[4];
	// общий размер файла
	long filesize;
	// тип данных riff
	char rifftype[4];
	// идентификатор блока описания формата
	char chunk_id1[4];
	// размер блока описания формата
	long chunksize1;
	// идентификатор формата данных
	short wFormatTag;
	// число каналов
	short nChannels;
	// число сэмплов в секунду
	long nSamplesPerSec;
	// среднее число байт/сек
	long nAvgBytesPerSec;
	// размер одного блока (число каналов)*(число байтов на канал)
	short nBlockAlign;
	// число битов на один сэмпл
	short wBitsPerSample;
	// идентификатор области аудиоданных
	char chunk_id2[4];
	// длина области аудиоданных
	long chunksize2;
} WAVHEADER_t;


typedef struct
{
  uint32_t   ChunkID;       /* 0 */
  uint32_t   FileSize;      /* 4 */
  uint32_t   FileFormat;    /* 8 */
  uint32_t   SubChunk1ID;   /* 12 */
  uint32_t   SubChunk1Size; /* 16*/
  uint16_t   AudioFormat;   /* 20 */
  uint16_t   NbrChannels;   /* 22 */
  uint32_t   SampleRate;    /* 24 */

  uint32_t   ByteRate;      /* 28 */
  uint16_t   BlockAlign;    /* 32 */
  uint16_t   BitPerSample;  /* 34 */
  uint32_t   SubChunk2ID;   /* 36 */
  uint32_t   SubChunk2Size; /* 40 */

}WAVE_FormatTypeDef;


typedef struct {
  int32_t offset;
  uint32_t fptr;
}Audio_BufferTypeDef;

typedef struct {
  uint32_t header;
  uint32_t start_calib;
  uint32_t stop_calib;
  uint32_t gps_time;
  uint32_t delta_ms;
  uint32_t T_new;
}log_t;

#define RTC_BKP_ADR         RTC_BKP_DR5

struct Controller_ControlFrame {
    long dt, lat, lon, alt, course, speed; // 6*4 bytes
};



/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
  #define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define lcd_pwm_Pin GPIO_PIN_12
#define lcd_pwm_GPIO_Port GPIOD
#define rst_lcd_Pin GPIO_PIN_13
#define rst_lcd_GPIO_Port GPIOD
#define sd_present_Pin GPIO_PIN_6
#define sd_present_GPIO_Port GPIOD
#define PPS_GPIO_PORT   GPIOB
#define PPS_GPIO_Pin    GPIO_PIN_1
#define PPS2_GPIO_PORT   GPIOB
#define PPS2_GPIO_Pin   GPIO_PIN_0
/* USER CODE BEGIN Private defines */

#define RTC_ASYNCH_PREDIV  0x0F    // 15
#define RTC_SYNCH_PREDIV   0x07FF  // 2047




#define TIMx                           TIM4
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM4_CLK_ENABLE()
/* Definition for TIMx Channel Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOD_CLK_ENABLE();
#define TIMx_GPIO_PORT_CHANNEL1        GPIOD
#define TIMx_GPIO_PIN_CHANNEL1         GPIO_PIN_12
#define TIMx_GPIO_AF_CHANNEL1          GPIO_AF2_TIM4


#define    maxbright 499
#define    minbright 50
/* USER CODE END Private defines */
 void SAI_Out_Init(uint16_t frequency,uint8_t mo_st);
 void SAI_In_Init(uint8_t mo_st);
 void Sound_In(uint8_t st_mo,uint16_t freq, uint16_t sample_per_fr);
 void InitCodec(uint8_t st_mo);
 void audio_rec_start(uint16_t sample_per_fr);
 char init_mp3_encoder_engine(int sample_rate, int num_channels, int bitrate);


 void board_init(void);
 

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
