
/* Includes ------------------------------------------------------------------*/
#include "SCCB.h"
#include "i2c_routines.h"

//extern I2C_HandleTypeDef hi2c2;
__IO uint32_t  DCMI_TIMEOUT_MAX = SCCB_Open407V_FLAG_TIMEOUT;

/*******************************************************************************
* Function Name  : SCCB_GPIO_Config
* Description    :
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

/**
  * @brief  Writes a byte at a specific Camera register
  * @param  Device: OV2640 write address.
  * @param  Addr: OV2640 register address.
  * @param  Data: data to be written to the specific register
  * @retval 0x00 if write operation is OK.
  *         0xFF if timeout condition occured (device not connected or bus error).
  */
uint8_t DCMI_SingleRandomWrite(uint8_t Reg, uint8_t Data)
{
   uint8_t err=0;
	 __disable_irq();




  i2c_start(); // I2C_GenerateSTART
  i2c_write(OV2640_DEVICE_WRITE_ADDRESS); //I2C_Send7bitAddress
  wait_ack ();

  i2c_write(Reg);  //Reg
  wait_ack ();

  i2c_write(Data);  //data
  wait_ack ();

  /* Send STOP condition */
  i2c_stop();

//		if( HAL_I2C_Master_Transmit(&hi2c2, OV2640_DEVICE_WRITE_ADDRESS, data,sizeof(data), SCCB_Open407V_FLAG_TIMEOUT)!= HAL_OK)
  //  err=0xFF;
 __enable_irq();
	 return err;
}

/**
  * @brief  Reads a byte from a specific Camera register
  * @param  Device: OV2640 write address.
  * @param  Addr: OV2640 register address.
  * @retval data read from the specific register or 0xFF if timeout condition
  *         occured.
  */
uint8_t DCMI_SingleRandomRead(uint8_t Reg, uint8_t *Data)
{
	  uint8_t err=0;
    __disable_irq();


     /* Send START condition */
  i2c_start();
  /* Send EEPROM address for write */
  i2c_write(OV2640_DEVICE_WRITE_ADDRESS);
  wait_ack ();
  /* Send the internal address to write to */
   i2c_write(Reg);
   wait_ack ();

  /* Send STRAT condition a second time */
  i2c_start(); // повторный старт
  i2c_write(OV2640_DEVICE_READ_ADDRESS);
  wait_ack ();

  *Data=i2c_read(nack);
     i2c_stop();



   // if((HAL_I2C_Master_Transmit(&hi2c2, OV2640_DEVICE_WRITE_ADDRESS, &Reg, 1, SCCB_Open407V_FLAG_TIMEOUT) != HAL_OK)
   // || (HAL_I2C_Master_Receive(&hi2c2, OV2640_DEVICE_READ_ADDRESS, Data, 1, SCCB_Open407V_FLAG_TIMEOUT) != HAL_OK)) {
   //     err=0xFF;
   // }




__enable_irq();
return err;


}
