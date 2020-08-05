

#include "main.h"

//#define u8 char



void sda_direct_in (void);
void sda_direct_out (void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char valu);
void scl_direct_out (void);
void reset_out (void);
u8 i2c_read(u8 is_ack);
void initI2C();
u8 wait_ack (void);

#define i2c_delay 500

#define ack 1
#define nack 0

#define SCL_PIN            GPIO_PIN_6
#define SCL                GPIO_PIN_6
#define SCLK               GPIO_PIN_6

#define SDA_PIN            GPIO_PIN_9
#define SDI                GPIO_PIN_9

#define I2C_DIR            GPIOB

#define Set_sclk       HAL_GPIO_WritePin(I2C_DIR, SCL, GPIO_PIN_SET)    //(I2C_DIR->BSRRL = SCLK)  //
#define Clr_sclk       HAL_GPIO_WritePin(I2C_DIR, SCL, GPIO_PIN_RESET)  //(I2C_DIR->BSRRH = SCLK) //

#define Set_sdio       HAL_GPIO_WritePin(I2C_DIR, SDI, GPIO_PIN_SET)    //(I2C_DIR->BSRRL = SDI)
#define Clr_sdio       HAL_GPIO_WritePin(I2C_DIR, SDI, GPIO_PIN_RESET)  //(I2C_DIR->BSRRH = SDI)




#define READ 1
#define WRITE 0





