
/* Includes ------------------------------------------------------------------*/
#include "dcmi_OV2640.h"
#include "DCMI_OV2640_INITTABLE.h"
#include "main.h"



void OV2640_Init(void)
{
  OV2640_Reset();//ПИН СБРОСА НЕ ЗАДЕЙСТВОВАН
  HAL_Delay(1000);
}


void OV2640_Reset(void)
{
  DCMI_SingleRandomWrite(OV2640_DSP_RA_DLMT, 0x01);
  DCMI_SingleRandomWrite(OV2640_SENSOR_COM7, 0x80);
}
/**
  * @brief  Read the OV2640 Manufacturer identifier.
  * @param  OV2640ID: pointer to the OV2640 Manufacturer identifier.
  * @retval None
  */
uint8_t DCMI_OV2640_ReadID(OV2640_IDTypeDef* OV2640ID)
{
	uint8_t temp;
	DCMI_SingleRandomWrite(OV2640_DSP_RA_DLMT, 0x01);
	if(DCMI_SingleRandomRead(OV2640_SENSOR_MIDH,&temp)!=0)
		return 0xff;
	OV2640ID->Manufacturer_ID1 = temp;
	if(DCMI_SingleRandomRead(OV2640_SENSOR_MIDL,&temp)!=0)
		return 0xff;
	OV2640ID->Manufacturer_ID2 = temp;
	if(DCMI_SingleRandomRead(OV2640_SENSOR_PIDH,&temp)!=0)
		return 0xff;
	OV2640ID->Version = temp;
	if(DCMI_SingleRandomRead(OV2640_SENSOR_PIDL,&temp)!=0)
		return 0xff;
	OV2640ID->PID = temp;

	return 0;
}



/**
  * @brief  Configures the OV2640 camera in JPEG mode.
  * @param  JPEGImageSize: JPEG image size
  * @retval None
  */
void OV2640_JPEGConfig(ImageFormat_TypeDef ImageFormat)
{
  uint32_t i;

  OV2640_Reset();
  HAL_Delay(200);


  for(i=0; i<(sizeof(OV2640_JPEG_INIT)/2); i++)
  {
    DCMI_SingleRandomWrite(OV2640_JPEG_INIT[i][0], OV2640_JPEG_INIT[i][1]);
		HAL_Delay(1);
  }


  for(i=0; i<(sizeof(OV2640_YUV422)/2); i++)
  {
    DCMI_SingleRandomWrite(OV2640_YUV422[i][0], OV2640_YUV422[i][1]);
		HAL_Delay(1);
  }

  DCMI_SingleRandomWrite(0xff, 0x01);
  DCMI_SingleRandomWrite(0x15, 0x00);


  for(i=0; i<(sizeof(OV2640_JPEG)/2); i++)
  {
    DCMI_SingleRandomWrite(OV2640_JPEG[i][0], OV2640_JPEG[i][1]);
		HAL_Delay(1);
  }

  HAL_Delay(100);

  switch(ImageFormat)
  {
    case JPEG_160x120:
    {
      for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
      {
        DCMI_SingleRandomWrite(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
		HAL_Delay(1);
      }
      break;
    }
    case JPEG_176x144:
    {
      for(i=0; i<(sizeof(OV2640_176x144_JPEG)/2); i++)
      {
        DCMI_SingleRandomWrite(OV2640_176x144_JPEG[i][0], OV2640_176x144_JPEG[i][1]);
      }
      break;
    }
    case JPEG_320x240:
    {
      for(i=0; i<(sizeof(OV2640_320x240_JPEG)/2); i++)
			{
				DCMI_SingleRandomWrite(OV2640_320x240_JPEG[i][0], OV2640_320x240_JPEG[i][1]);
				HAL_Delay(1);
			}
      break;
    }
    case JPEG_352x288:
    {
      for(i=0; i<(sizeof(OV2640_352x288_JPEG)/2); i++)
      {
        DCMI_SingleRandomWrite(OV2640_352x288_JPEG[i][0], OV2640_352x288_JPEG[i][1]);
      }
      break;
    }
    default:
    {
      for(i=0; i<(sizeof(OV2640_160x120_JPEG)/2); i++)
      {
        DCMI_SingleRandomWrite(OV2640_160x120_JPEG[i][0], OV2640_160x120_JPEG[i][1]);
      }
      break;
    }
  }
}

void SCCB_WriteRegs(const uint8_t* pbuf)
{
	while(1)
	{
		if((*pbuf == 0) && (*(pbuf + 1) == 0))
		{
			break;
		}
		else
		{
			DCMI_SingleRandomWrite(*pbuf++, *pbuf++);//ПРОВЕРИТЬ?
		}
	}
}


const static uint8_t OV2640_AUTOEXPOSURE_LEVEL0[]=// NOT USING !!!
{
	0xFF,	0x01,	0xff,
	0x24,	0x20,	0xff,
	0x25,	0x18,	0xff,
	0x26,	0x60,	0xff,
	0x00,	0x00,	0x00
};

const static uint8_t OV2640_AUTOEXPOSURE_LEVEL1[]=// NOT USING !!!
{
	0xFF,	0x01,	0xff,
	0x24,	0x34,	0xff,
	0x25,	0x1c,	0xff,
	0x26,	0x70,	0xff,
	0x00,	0x00,	0x00
};
const static uint8_t OV2640_AUTOEXPOSURE_LEVEL2[]=
{
	0xFF,	0x01,	0xff,
	0x24,	0x3e,	0xff,
	0x25,	0x38,	0xff,
	0x26,	0x81,	0xff,
	0x00,	0x00,	0x00
};
const static uint8_t OV2640_AUTOEXPOSURE_LEVEL3[]=// NOT USING !!!
{
	0xFF,	0x01,	0xff,
	0x24,	0x48,	0xff,
	0x25,	0x40,	0xff,
	0x26,	0x81,	0xff,
	0x00,	0x00,	0x00
};
const static uint8_t OV2640_AUTOEXPOSURE_LEVEL4[]=// NOT USING !!!
{
	0xFF,	0x01,	0xff,
	0x24,	0x58,	0xff,
	0x25,	0x50,	0xff,
	0x26,	0x92,	0xff,
	0x00,	0x00,	0x00
};

void OV2640_AutoExposure(uint8_t level)
{
	switch(level)
	{
		case 0:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL0);
			break;
		case 1:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL1);
			break;
		case 2:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL2);//320_240
			break;
		case 3:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL3);
			break;
		case 4:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL4);
			break;
		default:
			SCCB_WriteRegs(OV2640_AUTOEXPOSURE_LEVEL0);
			break;
	}

}


void OV2640_BrightnessConfig(uint8_t Brightness)
{
  DCMI_SingleRandomWrite(0xff, 0x00);
  DCMI_SingleRandomWrite(0x7c, 0x00);
  DCMI_SingleRandomWrite(0x7d, 0x04);
  DCMI_SingleRandomWrite(0x7c, 0x09);
  DCMI_SingleRandomWrite(0x7d, Brightness);
  //DCMI_SingleRandomWrite(0x7d, 0x00);
}




/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
