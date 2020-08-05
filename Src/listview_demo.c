/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2012  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************
** emWin V5.14 - Graphical user interface for embedded applications **
All  Intellectual Property rights  in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with the following terms:
The software has been licensed to Energy Micro AS whose registered office
is situated at  Sandakerveien 118, N-0484 Oslo, NORWAY solely
for  the  purposes  of  creating  libraries  for Energy Micros ARM Cortex-M3, M4F
processor-based  devices,  sublicensed  and distributed  under the terms and
conditions  of  the   End  User  License Agreement supplied by Energy Micro AS. 
Full source code is available at: www.segger.com
We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : WIDGET_ListView.c
Purpose     : Demonstrates the use of header widgets
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "GUI.h"
#include "LISTVIEW.h"
#include "FRAMEWIN.h"
#include "main.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

#define SPEED 650

#define MSG_CHANGE_MAIN_TEXT (WM_USER + 0)
#define MSG_CHANGE_INFO_TEXT (WM_USER + 1)

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/

extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana23;
extern IWDG_HandleTypeDef hiwdg1;

extern uint32_t records,page_num;
extern log_t Log_R[];
extern uint32_t adr_page_log;
extern uint8_t log_flag;
uint8_t view_log;
uint16_t timeout_view;

static LISTVIEW_Handle _hListView;



/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*******************************************************************
*
*       _ChangeMainText
*
*  Sends a message to the background window and invalidate it, so
*  the callback of the background window display the new text.
*/


/*******************************************************************
*
*       _cbBkWindow
*/
static void _cbBkWindow(WM_MESSAGE* pMsg) {
  switch (pMsg->MsgId) {
    case MSG_CHANGE_MAIN_TEXT:

      break;
    case MSG_CHANGE_INFO_TEXT:

      break;
    case WM_PAINT:
      //GUI_SetBkColor(GUI_BLACK);
      GUI_Clear();

   //   LISTVIEW_SetBkColor(_hListView,LISTVIEW_CI_UNSEL,GUI_BLACK);
      
      break;
       case WM_CREATE:

         break;
    default:
      WM_DefaultProc(pMsg);
  }
}
void Show_Rows(uint8_t n_rows, uint16_t page)
{
  LISTVIEW_DeleteAllRows(_hListView);
  GUI_Exec();
  char item1[20], item2[20],item3[20],item4[20]/*,item5[20]*/;
  time_t next_time;
  GUI_ConstString _aTable_1[] = {item1,item2,item3, item4/*,item5*/ };
  for (int i = 0; i < n_rows; i++) {
    
    struct tm *out = localtime(&Log_R[i].start_calib);
    sprintf(item1,"%02d.%02d.%02d %02d:%02d:%02d", out->tm_mday, out->tm_mon+1, out->tm_year-100, out->tm_hour, out->tm_min, out->tm_sec);
    /*
    if (Log_R[i].stop_calib!=0)
    {
      out = localtime(&Log_R[i].stop_calib);
      sprintf(item2,"%02d.%02d.%02d %02d:%02d:%02d", out->tm_mday, out->tm_mon+1, out->tm_year-100, out->tm_hour, out->tm_min, out->tm_sec);
    }
    else sprintf(item2,"--.--.-- --:--:--");
    if (Log_R[i].gps_time!=0)
    {
      out = localtime(&Log_R[i].gps_time);
      sprintf(item3,"%02d.%02d.%02d %02d:%02d:%02d", out->tm_mday, out->tm_mon+1, out->tm_year-100, out->tm_hour, out->tm_min, out->tm_sec);   
    }
    else sprintf(item3,"--.--.-- --:--:--");
*/
    if (Log_R[i].delta_ms!=0xFFFF)  sprintf(item2,"%d", Log_R[i].delta_ms);
    else  sprintf(item2,"---");

    sprintf(item3,"%d ч  %02d мин",(Log_R[i].T_new/3600),(Log_R[i].T_new%3600)/60);
    next_time = Log_R[i].start_calib+Log_R[i].T_new;
    out = localtime(&next_time);
    sprintf(item4,"%02d.%02d.%02d %02d:%02d:%02d", out->tm_mday, out->tm_mon+1, out->tm_year-100, out->tm_hour, out->tm_min, out->tm_sec);
    LISTVIEW_AddRow(_hListView, _aTable_1);
  }
   sprintf(item1, "Страница %d из %d",page+1, page_num); 
 //  sprintf(item2, "из %d",page_num); 
   sprintf(item2,""); sprintf(item3,""); sprintf(item4,"");
   LISTVIEW_AddRow(_hListView, _aTable_1);
  GUI_Exec(); 
}

void Show_Page(uint16_t page)
{

  if (page!=page_num-1)
  {
    Read_Log(page,row_on_page);
    Show_Rows(row_on_page, page);
  }
  else  // вывод последней страницы (возможно неполной)
  {
    uint8_t num = records-(page_num-1)*row_on_page;
    Read_Log(page,num);
    Show_Rows(num, page);
  }
  
}
/*********************************************************************
*
*       _Demo
*/
static void _Demo(void) {

WM_SetFocus(_hListView);
  //char acInfoText[] = "-- sec to play with header control";
  HEADER_Handle hHeader;
  hHeader = LISTVIEW_GetHeader(_hListView);
  HEADER_SetFont(hHeader,&GUI_FontVerdana23);

  LISTVIEW_AddColumn(_hListView, 250, " Время калибровки ",         GUI_TA_CENTER);
  //LISTVIEW_AddColumn(_hListView, 200, " Время кон. калибр. ", GUI_TA_CENTER);
  //LISTVIEW_AddColumn(_hListView, 200, " GPS время ", GUI_TA_CENTER);
  LISTVIEW_AddColumn(_hListView, 0, " Дельта, ms", GUI_TA_CENTER);
  LISTVIEW_AddColumn(_hListView, 0, "   Т новое   ", GUI_TA_CENTER);
  LISTVIEW_AddColumn(_hListView, 0, " Следующая калибровка ", GUI_TA_CENTER);
  GUI_Exec();
  

  uint16_t curr_page=0;
  Show_Page(curr_page);
  uint16_t count_exit=0;
  view_log = 1;
  while(1)
  {
    HAL_IWDG_Refresh(&hiwdg1);
    if (timeout_view>30000)
    {
      view_log = 0; return;
    }
    // выход
    if (key1_press && key2_press)
    {
      HAL_Delay(100);
      count_exit++;
      if(count_exit>20) { view_log = 0; return; }      
    } 
    else
    if (key1_press || key2_press)
    {
      timeout_view = 0;
      HAL_Delay(50);
      if (key2_press)
      {
        if (curr_page ==  0) curr_page = (page_num-1);
        else curr_page--;
        Show_Page(curr_page);
      }
      else if (key1_press)
      {
        curr_page++; if (curr_page>(page_num-1)) curr_page=0; 
        Show_Page(curr_page);
      }
    }
    // чтобы продолжало писать лог
      if (log_flag ==1 && adr_page_log<1000)  // ограничение в 1000 записей
      {
        Write_Log(adr_page_log);
        adr_page_log++;
        log_flag = 0;
      }  
    
  }

    
}

/*********************************************************************
*
*       _DemoListView
*/
static void _DemoListView(void) {
  
  LISTVIEW_SetDefaultTextColor(0,GUI_WHITE);
  LISTVIEW_SetDefaultBkColor(LISTVIEW_CI_UNSEL,GUI_BLACK);
  LISTVIEW_SetDefaultFont(&GUI_FontVerdana23);
  _hListView = LISTVIEW_Create(10, 10, 840, 470, 0, 1234, WM_CF_SHOW, 0);
  GUI_Exec();
  _Demo();
  WM_DeleteWindow(_hListView);
  GUI_Exec();
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/

void MainTask(void) {
  
  records = adr_page_log;
  page_num = records/row_on_page;
  if (records == 0) return;
  if ((records - page_num*row_on_page)!=0) page_num++; // есть еще одна неполная страница
  WM_EnableMemdev(WM_HBKWIN);
  WM_SetCreateFlags(WM_CF_MEMDEV);
  WM_SetCallback(WM_HBKWIN, _cbBkWindow);
  _DemoListView();

 // }
}

/*************************** End of file ****************************/