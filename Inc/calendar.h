#include <time.h>
void RTC_CalendarConfig(struct tm set_time);
time_t rtc_to_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);
void RTC_AlarmConfig(time_t al_time);