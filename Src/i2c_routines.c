

#include "i2c_routines.h"

u8 error;
//extern void delay_us(__IO uint32_t nTime);

void delay_uus(volatile int nTime){
    while(nTime--);
}

void initI2C()
{
    sda_direct_out();
    scl_direct_out();
    Set_sdio; // sda = 1;

    Set_sclk; //scl = 1;
    

}

void sda_direct_in(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  
    
      GPIO_InitStruct.Pin = SDA_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
      HAL_GPIO_Init (I2C_DIR, &GPIO_InitStruct); 
}

void sda_direct_out(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; //GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
  HAL_GPIO_Init (I2C_DIR, &GPIO_InitStruct); 
}

void scl_direct_out(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  
    GPIO_InitStruct.Pin = SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  
    HAL_GPIO_Init (I2C_DIR, &GPIO_InitStruct); 
}

void i2c_start(void){
    sda_direct_out();
    /***************************************************

    START: make sure here SDIO_DIR =OUT, SCLK = 1,        SDIO = 1

     ****************************************************/
    Set_sdio; //SDIO = 1;
    delay_uus(i2c_delay);
    Set_sclk; //SCLK = 1;

    delay_uus(i2c_delay); // DELAY(DURATION_START_1);
    Clr_sdio; // SDIO = 0;
    delay_uus(i2c_delay); // DELAY(DURATION_START_2);
    Clr_sclk; // SCLK = 0;
    delay_uus(i2c_delay); // DELAY(DURATION_START_3);

}

void i2c_stop(void){
    sda_direct_out(); // SDIO_DIR = OUT;
    Clr_sdio; //SDIO = 0;
    delay_uus(i2c_delay); //
    Set_sclk; // SCLK = 1;
    delay_uus(i2c_delay); //
    Set_sdio; // SDIO = 1;
    delay_uus(i2c_delay); //
}

u8 wait_ack(void){
    u8 kg;
    Clr_sdio; //
    error = 0;
    delay_uus(i2c_delay);
   // delay_uus(350);
    sda_direct_in(); // порт на вход
    delay_uus(i2c_delay); // задержка
    Set_sclk; // дернем ножкой SCLK
    delay_uus(i2c_delay); // задержка
    kg = HAL_GPIO_ReadPin(I2C_DIR, SDI); //GPIO_ReadInputDataBit(SDIO_DIR, SDI);
    if(kg != 0) error = 1; // собственно провер€ем ответ от микры( она должна выдать в линию 0)
    // 0 не получили - выставили бит ошибки
    Clr_sclk; //
    delay_uus(i2c_delay);
    return error;

}
//-----------------------подпрограмма «јѕ»—и-------------------------------------------------------

void i2c_write(unsigned char valu){

    register union{
        unsigned char all;

        struct{
            unsigned char unuse:7;
            unsigned char msb:1;
        };
    } data;
    data.all = valu;
    char i;
    i = 8;
    sda_direct_out();
    while(i--)
        // цикл на байт
    {
        if(data.msb & 0x01){
            Set_sdio;
        } else{
            Clr_sdio;
        } // выдвинем бит в SDIO
        delay_uus(i2c_delay); // задержка
        Set_sclk; // SCLK = 1;                              // дернем ножкой SCLK
        delay_uus(i2c_delay); // задержка
        Clr_sclk; // SCLK = 0;                              // опустим SCLK
        delay_uus(i2c_delay); // задержка
        data.all <<= 1;
    }
    delay_uus(i2c_delay);
    //Set_sclk; // SCLK = 1;                            
    //delay_uus(330); // задержка

}

u8 i2c_read(u8 is_ack) // будем указывать отправл€ть микре 0 аск или 1 нет
{
    u8 data;
    u8 x = 8;
    sda_direct_in(); //sda input
    while(x--){
        Set_sclk;
        delay_uus(i2c_delay);
        data <<= 1;
        data |= HAL_GPIO_ReadPin(I2C_DIR, SDI); //GPIO_ReadInputDataBit(SDIO_DIR, SDI);
        Clr_sclk;
        delay_uus(i2c_delay);
    }
    sda_direct_out();
    if(is_ack == 0){
        // отправим ACK
        //delay_uus(60);  
        Set_sdio;
        Set_sclk;
        delay_uus(i2c_delay);
        Clr_sclk;
        delay_uus(i2c_delay);
        Clr_sdio;
    } else{
        Clr_sdio;
        Set_sclk;
        delay_uus(i2c_delay);
        Clr_sclk;
        delay_uus(i2c_delay);
    }
    return data;
}



