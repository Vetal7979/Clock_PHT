 /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "GUI.h"
#include <stdio.h>
#include <math.h> 
#include <time.h>
#include "log.h"
#include "delay.h"
#include "bmp280.h"
#include "w25qxx.h"


u8 event_SD;

int Frtc;

RTC_TimeTypeDef stimestructureget;
RTC_DateTypeDef sdatestructureget;

uint16_t date_set_timeout; //, timeout_2, timeout_wifi, timeout_1pps;
uint32_t timeout_lora_start;
uint32_t reset_wifi;
char no_pps = 0;
char calib_1st = 1;     // 1-я калибровка после включения должна быть через 1 час, потом по алгоритму

uint32_t bmp_timer, bmp_period;
uint8_t bmp_enable=1;
char BMP_fail;
extern float pressure, temperature, humidity;

extern UART_HandleTypeDef huart1, huart2, huart5;
extern RTC_HandleTypeDef RtcHandle;
extern IWDG_HandleTypeDef hiwdg1;

extern const string WeekDayNames[];
extern const   uint8_t MonthNames[];
uint8_t need_set_time_date = 0;
uint8_t alarm_set = 0;

extern struct tm gps_time, gps_time2;
extern struct tm wifi_time;
extern struct tm lora_time;

extern uint16_t wifi_delay, lora_delay;
extern int delta_wifi, delta_lora;
extern uint8_t wifi_stat, wifi_valid_data ,lora_valid_data;
//extern uint16_t CRC;

uint8_t show_time_flag=0;
uint8_t psim_ok=0, psim_ok2, psim_count=0, psim_fail=0;
extern int16_t CALM;
extern uint32_t CALP;
extern uint32_t T;
extern uint8_t synchro_rtc;
volatile extern uint8_t rtc_calib_ok, need_rtc_calib;
extern  time_t current_ttime, ttime_to_calib;
extern struct tm current_gps_time, current_gps_time2, current_wifi_time, current_lora_time;
volatile extern char time_set, date_set, gps_valid;
volatile extern char time_set2, date_set2, gps_valid2;
extern int delta, old_delta, delta_ms, old_delta_ms;
extern uint32_t delta_summ;
extern time_t gps_t, gps_t2, wifi_t, lora_t;
extern uint32_t pps1_time, pps2_time;
extern int diff_pps;
extern uint8_t WiFi_Buff[];

extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana16;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana23;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana32;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana220;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana71;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana82;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana96;
extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana45_rus;
extern GUI_CONST_STORAGE GUI_BITMAP bmGPS_WIFI;
extern GUI_CONST_STORAGE GUI_BITMAP bmgps_icon;
extern GUI_CONST_STORAGE GUI_BITMAP bmclock_icon;
extern GUI_CONST_STORAGE GUI_BITMAP bmwire_gps;
extern GUI_CONST_STORAGE GUI_BITMAP bmlora_gps;
static char TempStr[50];


uint8_t log_flag=0;
log_t Log;

uint32_t adr_page_log = 0, records = 0; // свободный адрес и кол-во записей
uint16_t page_num; // кол-во  страниц для вывода

volatile time_t ct = {0};

#define M_PI    (float)     3.14159265f
int crc;
int tX,tY;
volatile int timee;
char str[100];
volatile int st_t = 0;

        FATFS fs;
	FRESULT res;
	DIR dirs;
  	FILINFO finfo;
      //  extern  FIL file;
        FIL tile_f;
        FIL filew;

      extern   UINT nRead, nWritten;


uint16_t DR2_test;
extern uint32_t Freq;


int main(void)
{
  int xpos,ypos,ystep, yspace;
  HAL_Init();
  board_init();
  HAL_IWDG_Refresh(&hiwdg1);
  //  HAL_Delay(1000);
  
  
  char data[50];  
  /* 
  while (!psim_ok)
  {
  HAL_UART_Transmit(&huart1,data,sprintf(data,"$PSIMPPS,W,4,100,0*36\r\n"),100);
  HAL_Delay(300);
  psim_count++;
  if (psim_count>10)
  { psim_fail=1; break; }
}
  */
  //  
  
  
  
  
  
  if (W25qxx_Init() != 1) Error_Handler();
  
 
  
  adr_page_log = Check_Addr();
  
  
  
  if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) != 0x32F2) need_set_time_date = 1;
  
  //  RTC_BKP_DR2 = ((FLAG[15] 0 0 0 CALP_FL[11] 0 0 CALM[8:0])
  uint16_t DR2 = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR2);
  
  if (DR2 !=0x0000) 
  {   
    CALM = (uint16_t)(DR2 & 0x01FF);
    if ((uint16_t)(DR2 & 0x0800)) { CALP = RTC_SMOOTHCALIB_PLUSPULSES_SET; } 
    else { CALP = RTC_SMOOTHCALIB_PLUSPULSES_RESET; }
    HAL_RTCEx_SetSmoothCalib(&RtcHandle, RTC_SMOOTHCALIB_PERIOD_32SEC, CALP, CALM); 
    rtc_calib_ok=1;
    
    Frtc = (0x100000 - CALM)/32; 
    if (CALP == RTC_SMOOTHCALIB_PLUSPULSES_SET) Frtc = (0x100000 - CALM - 512)/32; 
  }
  else 
  {
    rtc_calib_ok=0;
  }
  
  
  
  need_set_time_date = 1;
  
  while (1)
  {
    if (key1_press && !key2_press)
    {
      HAL_Delay(50);
      HAL_IWDG_Refresh(&hiwdg1);
      if (key1_press && !key2_press) 
      {
        MainTask(); // отображаем лог
        bmp_enable=1; bmp_timer=0; // после выхода из просмотра перечитаем P H T и выведем ниже
      }
    }
    
    if (date_set_timeout >= gps_timeout) { date_set = 0; }
    
    
    
    xpos=120;
    ypos = 50;
    ystep = 50;
    yspace = 60;
    
    /*    
    // в теории это здесь не нужно, т.к. будильник взводится в 1 ппс
    if (!alarm_set)
    {
    if (need_set_time_date == 0 && rtc_calib_ok == 1)
    {
    HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
    current_ttime = rtc_to_timestamp(&sdatestructureget, &stimestructureget);
    RTC_AlarmConfig((time_t)(current_ttime + T-1));
    alarm_set = 1;
  }
  }
    */    
    
    if (show_time_flag) 
    {
      HAL_IWDG_Refresh(&hiwdg1);
      if(date_set)
      {
        GUI_DrawBitmap(&bmgps_icon, 760, 0);
        if (need_set_time_date)
        {
          synchro_rtc = 1;
          //   RTC_CalendarConfig(current_gps_time); // это здесь нахуя? оно в прерывании сработает
          need_set_time_date = 0;
        }
        if (rtc_calib_ok == 0 && need_rtc_calib == 0) need_rtc_calib=1; // это будет правильно если после калибровки ставить битик в качестве флага
      }
      else GUI_ClearRect(760,0,854,50);
      
      
      
      //RTC  
      GUI_SetFont(&GUI_FontVerdana45_rus);
      sprintf(TempStr," %02d ",  sdatestructureget.Date);//      Дата
      sprintf(data,"%c%c%c",  MonthNames[((sdatestructureget.Month-1) * 3)], MonthNames[((sdatestructureget.Month-1) * 3)+1],MonthNames[((sdatestructureget.Month-1) * 3)+2]);   //   Месяц
      strcat(TempStr,data);
      sprintf(data," %d, ",  sdatestructureget.Year+2000);                                                                                                                                                                                                                                                //   Год
      strcat(TempStr,data);
      sprintf((char *)data, WeekDayNames[DayOfWeek(sdatestructureget.Year, sdatestructureget.Month,sdatestructureget.Date)]);    //      День недели
      strcat(TempStr,data);
      GUI_DispStringHCenterAt((char *)TempStr,854/2, ypos);
      ypos+=ystep;
      GUI_SetFont(&GUI_FontVerdana220);
      sprintf((char *)TempStr, "%02d:%02d", stimestructureget.Hours , stimestructureget.Minutes);
      GUI_DispStringAt((char *)TempStr,xpos, ypos);
      GUI_SetFont(&GUI_FontVerdana96);
      sprintf((char *)TempStr, "%02d",stimestructureget.Seconds);
      GUI_DispStringAt((char *)TempStr,xpos+570, ypos+30);
      
      HAL_IWDG_Refresh(&hiwdg1);
      
      if (bmp_enable ==1)
      {
        HAL_IWDG_Refresh(&hiwdg1);
        init_bmp280();
        HAL_Delay(100);       
        if (BMP_fail == 0)
        {
          bmp_period = 30000;
          check_bmp280();
          sprintf(TempStr,"Температура: %2.1f град.  Влажность: %2.1f % ",  temperature,humidity);
          GUI_SetFont(&GUI_FontVerdana32);
          GUI_DispStringHCenterAt((char *)TempStr,854/2, 350);
          sprintf(TempStr,"Давление: %3.1f мм.рт.ст.",pressure/133.333);
          GUI_DispStringHCenterAt((char *)TempStr,854/2, 390);
        }
        else 
        {
          initI2C();  
          //init_bmp280();
          GUI_ClearRect(0,300,854,480);
          bmp_period = 5000;
        }
        bmp_timer=0; bmp_enable=0;
      }
  /*    
      if (date_set)
      {
        GUI_SetFont(&GUI_FontVerdana16);
        sprintf(TempStr,"Дельта, = %2d сек %3d мс",delta, delta_ms);
        GUI_DispStringAt((char *)TempStr,50, 440);
      }
      else GUI_ClearRect(50,440,150,480);
    */  
      show_time_flag=0;
    }  
HAL_IWDG_Refresh(&hiwdg1);    
    //---- Запись лога  ----------------------
    if (log_flag ==1 && adr_page_log<1000) // ограничение в 1000 записей
    {
      Write_Log(adr_page_log);
      adr_page_log++;
      log_flag = 0;
    } 
    
  }
  
}







