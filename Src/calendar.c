#include "main.h"
#include "GUI.h"
#include "calendar.h"
#include <string.h>
#include <stdio.h>

extern RTC_HandleTypeDef RtcHandle;

extern struct tm gps_time2;


extern uint8_t log_flag;


const string WeekDayNames[7] =
  {"вс" ,
   "пн",
   "вт",
   "ср",
   "чт",
   "пт",
   "сб"};


const   uint8_t MonthNames[] = {'€', 'н', 'в', 'ф', 'е', 'в', 'м', 'а', 'р', 'а', 'п', 'р',
                                  'м', 'а', 'й', 'и', 'ю', 'н', 'и', 'ю', 'л', 'а', 'в', 'г',
                                  'с', 'е', 'н', 'о', 'к', 'т', 'н', 'о', '€', 'д', 'е', 'к'};

bool IsLeapYear(u16 nYear)
{
  if (nYear % 4 != 0) return FALSE;
  if (nYear % 100 != 0) return TRUE;
  return (bool)(nYear % 400 == 0);
}

uint16_t CountOfFeb29(uint16_t nYear)
{
  uint16_t nCount = 0;

  if (nYear > 0)
  {
    nCount = 1; /* Year 0 is a leap year */
    nYear--;    /* Year nYear is not in the period */
  }
  nCount += nYear / 4 - nYear / 100 + nYear / 400;
  return nCount;
}

uint16_t DayOfWeek(uint16_t nYear, uint16_t nMonth, uint16_t nDay)
{
  uint16_t nDayOfWeek = 0;
  uint16_t pnDaysBeforeMonth[14] = {0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};

  /* The day of Jan 1, nYear */
  nDayOfWeek = 6 + nYear % 7 + CountOfFeb29(nYear) % 7 + 14; /* + 14 : makes nDayOfWeek >= 0 */

  /* The day of nMonth 1, nYear */
  nDayOfWeek += pnDaysBeforeMonth[ nMonth ];
  if ( nMonth > 2 && IsLeapYear(nYear))
  {
    nDayOfWeek++;
  }

  /* The day of nMonth nDay, nYear */
  nDayOfWeek += nDay - 1;
  nDayOfWeek %= 7;
  /* return the number of the day in the week */
  return nDayOfWeek;
}





void RTC_CalendarConfig(struct tm set_time)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

   struct tm rtc = { 0 };
      rtc.tm_hour = set_time.tm_hour;
      rtc.tm_min = set_time.tm_min;
      rtc.tm_sec = set_time.tm_sec;
      rtc.tm_mday = set_time.tm_mday;
      rtc.tm_mon = set_time.tm_mon;//-1; -1 и +100 уже сделаны в прерывании 1PPS
      rtc.tm_year = set_time.tm_year; //+100; // а также прибавлен часовой по€с и 1 сек
     time_t tt = mktime(&rtc);
     tt+=3*3600+1; // UTC + 1 sec
     struct tm *out = localtime(&tt); 
     
  sdatestructure.Year = out->tm_year-100; //
  sdatestructure.Month = out->tm_mon+1; //
  sdatestructure.Date = out->tm_mday; // 
  sdatestructure.WeekDay = out->tm_wday; //

  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  
  stimestructure.Hours = out->tm_hour;
  stimestructure.Minutes = out->tm_min;
  stimestructure.Seconds = out->tm_sec;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  stimestructure.SubSeconds = 2047;

    if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-3- Writes a data in a RTC Backup data Register1 #######################*/
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x32F2);
}

time_t rtc_to_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
  struct tm rtc = { 0 };
      rtc.tm_hour = time->Hours;
      rtc.tm_min = time->Minutes;
      rtc.tm_sec = time->Seconds;
      rtc.tm_mday = date->Date;
      rtc.tm_mon = date->Month-1;
      rtc.tm_year = date->Year+100;
     return mktime(&rtc); 
}
void timestamp_to_rtc(time_t ttime, RTC_DateTypeDef *date, RTC_TimeTypeDef *time )
{
  struct tm *out = localtime(&ttime); 
  date->Date = out->tm_mday;
  date->Month = out->tm_mon+1;
  date->Year = out->tm_year-100;
  time->Hours = out->tm_hour;
  time->Minutes = out->tm_min;
  time->Seconds = out->tm_sec;
}
void RTC_AlarmConfig(time_t al_time)
{  
  struct tm *out = localtime(&al_time); 
  RTC_AlarmTypeDef salarmstructure;

  /*RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
  salarmstructure.Alarm = RTC_ALARM_A;
  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY; // Date/day donТt care in Alarm A comparison
  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  salarmstructure.AlarmTime.Hours = out->tm_hour;//.Hours;
  salarmstructure.AlarmTime.Minutes = out->tm_min;//.Minutes;
  salarmstructure.AlarmTime.Seconds = out->tm_sec;//.Seconds+30;
  salarmstructure.AlarmTime.SubSeconds = 0;
  
  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }

  
}