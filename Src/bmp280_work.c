#include "main.h"
//#include "GUI.h"
/* USER CODE BEGIN Includes */
#include "bmp280.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

BMP280_HandleTypedef bmp280;
//  extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana16;
 // extern GUI_CONST_STORAGE GUI_FONT GUI_FontVerdana23;
float pressure, temperature, humidity;

uint16_t size;
char Data[256];
char bme280p;
extern char BMP_fail;

void init_bmp280(void) {

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	//bmp280.i2c = &hi2c1;

//	while (!bmp280_init(&bmp280, &bmp280.params)) {
        if  (!bmp280_init(&bmp280, &bmp280.params)) {
		sprintf((char *)Data, "BMP280 initialization failed\n");
                BMP_fail = 1;
                return;

	}
        else BMP_fail = 0;
        bme280p = bmp280.id == BME280_CHIP_ID;
	sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

}

void check_bmp280(void) {


		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
			//sprintf((char *)Data,"Temperature/pressure reading failed\n");
		HAL_Delay(100);
		}
                /*
                  GUI_SetFont(&GUI_FontVerdana23);
		sprintf(Data,"Давл: %.1f мм ",	(float) pressure/133.333);
		GUI_DispStringAt((char *)Data, 10, 178);

		sprintf(Data,"Темп: %.1f C ", temperature);
		GUI_DispStringAt((char *)Data, 10, 200);
		sprintf(Data,"Влаж: %.1f",	humidity);
		GUI_DispStringAt((char *)Data, 10, 220);*/
	}