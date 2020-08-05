#include "main.h"
#include "GUI.h"
//#include "w25qxx.h"


extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana32;
extern log_t Log;
extern log_t Log_R[];
extern uint32_t records;
extern uint32_t adr_page_log;
extern RTC_HandleTypeDef RtcHandle;
extern IWDG_HandleTypeDef hiwdg1;

uint32_t Check_Addr(void)
{
  uint32_t adr_page=0;
  
  while(adr_page<0xFFFF)
  {
    HAL_IWDG_Refresh(&hiwdg1);
  log_t Tmp={0};
  W25qxx_ReadPage(&Tmp,adr_page,0,sizeof(Tmp));
  if (Tmp.header==0xA5A5A5A5) adr_page++;
  else break;
  }
  return adr_page;
}

void Write_Log(uint32_t adr_page)
{
  Log.header = 0xA5A5A5A5;
  W25qxx_WritePage(&Log,adr_page,0,sizeof(Log));
  // все обнулим
  Log.start_calib = 0;
  Log.stop_calib = 0;
  Log.gps_time = 0;
  Log.delta_ms = 0;
  Log.T_new = 0;
}

void Read_Log(uint16_t page, uint8_t rec)
{
if (rec>20) return;
  uint8_t i = 0;
  while (i<rec)
  {
  W25qxx_ReadPage(&Log_R[i],(page*row_on_page+i),0,sizeof(Log));
  i++;
  }
}

void Erase_Log(void)
{
  W25qxx_EraseChip();
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_ADR,0);
  adr_page_log = 0;
  GUI_SetFont(&GUI_FontVerdana32);
  GUI_DispStringAt("erase ok",50, 50);
 // while(1);
    
}