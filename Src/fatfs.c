

#include "fatfs.h"
#include <string.h>
#include <stdio.h>
#include "GUI.h"

  extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana16;
  extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana23;

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
//FIL SDFile;       /* File object for SD */
  u8 card_present = 0;
  extern char str[];
  u8 mount_str[30];
  u8 card_str[30];
  uint32_t capacity;
  uint32_t free_mem_SD;
  volatile uint32_t   tot_sect = 0;
  FIL file;
  UINT nRead, nWritten;
  
  uint8_t isInitialized = 0;
/*  
  void Write_Log(const char *str)
  {
    FIL fil;
    FRESULT res;
    char tmp[10];
    sprintf(tmp,"%slog.txt",SDPath);
    res = f_open(&fil,tmp,FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK)
    {
      res = f_write(&fil, (uint8_t *)str, strlen(str), &nWritten); 
      f_close(&fil);
    }
  }
*/
void SD_Initialize(void)
{
  if (!BSP_SD_DeInit())
  {
  if (isInitialized == 0)
  {
    if (BSP_SD_Init() == MSD_OK)
    {
      isInitialized = 1;
   //   printf ("SD init OK \r\n");
    }


  }
  }
}

uint32_t k_StorageGetFree (void)
{
  uint32_t   fre_clust = 0;
  FATFS *fs;
  FRESULT res = FR_INT_ERR;


    fs = &SDFatFS;
    res = f_getfree("0:", (DWORD *)&fre_clust, &fs);

  if(res == FR_OK)
  {
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    return (fre_clust * fs->csize);

  }
  else
  {
    return 0;
  }
}

void MX_FATFS_Init(void)
{
  if (HAL_GPIO_ReadPin(sd_present_GPIO_Port, sd_present_Pin)==GPIO_PIN_RESET)
    {
      card_present = 1;
      SD_Initialize();
     if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
      {
        if(f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) == FR_OK)
        {
         sprintf(str, "Свободно:   [%lu MB]", k_StorageGetFree ()/ (2 * 1024));
         GUI_DispStringAt((char *)str, 5, 20);
         sprintf(str, "Ёмкость:      [%lu MB]", tot_sect / (2 * 1024));
         GUI_DispStringAt((char *)str, 5, 0);
        }
      }
    }
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
