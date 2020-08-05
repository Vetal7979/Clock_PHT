#include "main.h"
#include "fatfs.h"

extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;
__IO BUFFER_StateTypeDef buffer_offset;
__IO int16_t                 UpdatePointer = -1;
uint16_t     PlayBuff[PLAY_BUFF_SIZE];

uint16_t     RecBuff[4608];

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{

  buffer_offset = BUFFER_OFFSET_FULL;
  UpdatePointer = PLAY_BUFF_SIZE/2;

}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  buffer_offset = BUFFER_OFFSET_HALF;
  UpdatePointer = 0;
}



void audio_stop(void)
{
  HAL_SAI_DMAStop(&hsai_BlockA1);
}

void audio_start(void)
{
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)PlayBuff, PLAY_BUFF_SIZE/2);
}


void audio_rec_start(uint16_t sample_per_fr)
{
   SCB_CleanDCache();
  HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *)RecBuff, sample_per_fr);
}