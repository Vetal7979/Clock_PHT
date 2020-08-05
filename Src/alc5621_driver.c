#include "main.h"



#include "alc5621.h"
#include "i2c_routines.h"


#define device_read  0x35
#define device_write 0x34

void I2C1WriteBuffer(uint8_t Addr, uint16_t Reg, uint8_t *pBuffer, uint16_t Length)
{
    uint8_t adr_high;
    uint8_t adr_low;

     adr_high  = (Reg>>8);
     adr_low   = Reg;
  i2c_start(); // I2C_GenerateSTART
  i2c_write(Addr); //I2C_Send7bitAddress
  wait_ack ();

  i2c_write(adr_high);  //старший байт
  wait_ack ();

  i2c_write(adr_low);  //младший байт
  wait_ack ();

    while(Length--)
  {
    /* Send the current byte */
    i2c_write(*pBuffer);
    wait_ack ();
    pBuffer++;
  }
  /* Send STOP condition */
  i2c_stop();

}

void I2C1ReadBuffer(uint8_t Addr, uint16_t Reg,  uint8_t *pBuffer, uint16_t Length)
{
  u8 adr_high,adr_low;
  adr_high = Reg>>8;
  adr_low = Reg;


  /* Send START condition */
  i2c_start();
  /* Send EEPROM address for write */
  i2c_write(Addr);
  wait_ack ();
  /* Send the EEPROM's internal address to write to */
   i2c_write(adr_high);
   wait_ack ();
   i2c_write(adr_low);
   wait_ack ();
  /* Send STRAT condition a second time */
  i2c_start(); // повторный старт
  i2c_write(device_read);
  wait_ack ();

    while(Length)
  {
    if(Length == 1)
    {
     *pBuffer=i2c_read(nack);
     i2c_stop();
     break;
    }
      /* Read a byte from the EEPROM */
      *pBuffer = i2c_read(1);
      /* Point to the next location where the byte read will be saved */
      pBuffer++;
      /* Decrement the read bytes counter */
      Length--;
  }

}


/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address
  * @retval Data to be read
  */
uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint16_t Read_Value = 0;
  uint8_t Read_buf[2];
  I2C1ReadBuffer((uint16_t) Addr, (uint16_t) Reg, Read_buf, 2);
  Read_Value = Read_buf[0];
   Read_Value =  Read_Value <<8;
   Read_Value =  Read_Value + Read_buf[1];
  return Read_Value;
}

void writeCodec(uint8_t addr,uint16_t data)
{
  uint8_t Valuehigh;
  uint8_t Valuelow;



   Valuehigh =  data>>8;
   Valuelow =   data;

   i2c_start();      // I2C_GenerateSTART
   i2c_write(0x34);  //I2C_Send7bitAddress
   wait_ack ();
   i2c_write(addr);    //старший байт
   wait_ack ();


  i2c_write(Valuehigh);  //старший байт
  wait_ack ();

  i2c_write(Valuelow);  //младший байт
  wait_ack ();
  i2c_stop();
}



/*
  #define ALC5623_ADC_REC_MIXER			0x14

#define	ALC5623_ADCMIX_MUTE_MONOMIX_R		        (1<<0)
#define	ALC5623_ADCMIX_MUTE_SPKMIX_R		        (1<<1)
#define	ALC5623_ADCMIX_MUTE_HPMIX_R			(1<<2)
#define	ALC5623_ADCMIX_MUTE_AUXIN_R			(1<<3)
#define	ALC5623_ADCMIX_MUTE_LINEIN_R		        (1<<4)
#define	ALC5623_ADCMIX_MUTE_MIC2_R			(1<<5)
#define	ALC5623_ADCMIX_MUTE_MIC1_R			(1<<6)

#define	ALC5623_ADCMIX_MUTE_MONOMIX_L		        (1<<8)
#define	ALC5623_ADCMIX_MUTE_SPKMIX_L		        (1<<9)
#define	ALC5623_ADCMIX_MUTE_HPMIX_L			(1<<10)
#define	ALC5623_ADCMIX_MUTE_AUXIN_L			(1<<11)
#define	ALC5623_ADCMIX_MUTE_LINEIN_L		        (1<<12)
#define	ALC5623_ADCMIX_MUTE_MIC2_L			(1<<13)
#define	ALC5623_ADCMIX_MUTE_MIC1_L			(1<<14)
*/


void InitCodec(uint8_t st_mo)
{
	writeCodec(ALC5623_RESET,0x0000);
        writeCodec(ALC5623_GLOBAL_CLK_CTRL_REG,ALC5623_GBL_CLK_SYS_SOUR_SEL_MCLK);
	writeCodec(ALC5623_SPK_OUT_VOL,0x0a0a);
        writeCodec(ALC5623_HP_OUT_VOL,0x1010);//0x1c1c);
        writeCodec(ALC5623_STEREO_DAC_VOL,0x0505);
        writeCodec(ALC5623_LINE_IN_VOL,0x6202);
	                writeCodec(ALC5623_OUTPUT_MIXER_CTRL,//0x0d40
			ALC5623_OUTPUT_MIXER_CTRL_AUXOUT_MONOMIX
			| ALC5623_OUTPUT_MIXER_CTRL_HP_R_VOL_ENABLE
			| ALC5623_OUTPUT_MIXER_CTRL_HP_L_VOL_ENABLE
			| ALC5623_OUTPUT_MIXER_CTRL_SPKOUT_SPKMIX
			//| ALC5623_OUTPUT_MIXER_CTRL_SPKOUTN_RP_LR // stereo out
			| ALC5623_OUTPUT_MIXER_CTRL_SPKOUTN_LN_MLR
			| ALC5623_OUTPUT_MIXER_CTRL_SPKOUT_CLASSAB
			| ALC5623_OUTPUT_MIXER_CTRL_SPKOUT_STRONG

	);//0fc0



        writeCodec(ALC5623_ADC_REC_MIXER , 	ALC5623_ADCMIX_MUTE_MIC1_L |
                                                ALC5623_ADCMIX_MUTE_MIC2_L |
                                                ALC5623_ADCMIX_MUTE_AUXIN_L |
                                                ALC5623_ADCMIX_MUTE_HPMIX_L |
                                                ALC5623_ADCMIX_MUTE_SPKMIX_L |
                                                ALC5623_ADCMIX_MUTE_MONOMIX_L |
                                                ALC5623_ADCMIX_MUTE_MIC1_R |
                                                ALC5623_ADCMIX_MUTE_MIC2_R |
                                                ALC5623_ADCMIX_MUTE_AUXIN_R |
                                                ALC5623_ADCMIX_MUTE_HPMIX_R |
                                                ALC5623_ADCMIX_MUTE_SPKMIX_R |
                                                ALC5623_ADCMIX_MUTE_MONOMIX_R);


	writeCodec(ALC5623_DAI_CONTROL,ALC5623_DAI_SDP_SLAVE_MODE | ALC5623_DAI_I2S_PCM_MODE);
	writeCodec(ALC5623_PWR_MANAG_ADD1,
			ALC5623_PWR_ADD1_MAIN_I2S_EN//|ALC5623_PWR_ADD1_SOFTGEN_EN
			|ALC5623_PWR_ADD1_HP_OUT_AMP|ALC5623_PWR_ADD1_HP_OUT_ENH_AMP
			|ALC5623_PWR_ADD1_AUX_OUT_AMP|ALC5623_PWR_ADD1_AUX_OUT_ENH_AMP
	);

	writeCodec(ALC5623_PWR_MANAG_ADD2,
			ALC5623_PWR_ADD2_LINEOUT |
//			ALC5623_PWR_ADD2_CLASS_D |
			ALC5623_PWR_ADD2_VREF |
			//		ALC5623_PWR_ADD2_PLL |
			ALC5623_PWR_ADD2_DAC_REF_CIR|
			ALC5623_PWR_ADD2_L_DAC_CLK	|
			ALC5623_PWR_ADD2_R_DAC_CLK	|
			ALC5623_PWR_ADD2_L_ADC_CLK_GAIN	|
			ALC5623_PWR_ADD2_R_ADC_CLK_GAIN	|
			ALC5623_PWR_ADD2_L_HP_MIXER	|
			ALC5623_PWR_ADD2_R_HP_MIXER	|
			ALC5623_PWR_ADD2_SPK_MIXER	|
			ALC5623_PWR_ADD2_MONO_MIXER	|
			ALC5623_PWR_ADD2_L_ADC_REC_MIXER	|
			ALC5623_PWR_ADD2_R_ADC_REC_MIXER

	);
	writeCodec(ALC5623_PWR_MANAG_ADD3,
			ALC5623_PWR_ADD3_MAIN_BIAS
			|	ALC5623_PWR_ADD3_AUXOUT_L_VOL_AMP
			|	ALC5623_PWR_ADD3_AUXOUT_R_VOL_AMP
			|	ALC5623_PWR_ADD3_SPK_OUT
			|	ALC5623_PWR_ADD3_HP_L_OUT_VOL
			|	ALC5623_PWR_ADD3_HP_R_OUT_VOL
			|	ALC5623_PWR_ADD3_LINEIN_L_VOL
			|	ALC5623_PWR_ADD3_LINEIN_R_VOL
			|	ALC5623_PWR_ADD3_AUXIN_L_VOL
			|	ALC5623_PWR_ADD3_AUXIN_R_VOL
			|	ALC5623_PWR_ADD3_MIC1_FUN_CTRL
			|	ALC5623_PWR_ADD3_MIC2_FUN_CTRL
			|	ALC5623_PWR_ADD3_MIC1_BOOST_AD
			|	ALC5623_PWR_ADD3_MIC2_BOOST_AD
	);


	writeCodec(ALC5623_STEREO_AD_DA_CLK_CTRL,
			ALC5623_STEREO_AD_DA_CLK_CTRL_I2S_PREDIV_1 	|
			ALC5623_STEREO_AD_DA_CLK_CTRL_BCLK_DIV_8  	|
			ALC5623_STEREO_AD_DA_CLK_CTRL_WCLK_DIVPRE_4 |
			ALC5623_STEREO_AD_DA_CLK_CTRL_WCLK_DIV_8 |
			ALC5623_STEREO_AD_DA_CLK_CTRL_ADDA_FILTER_256FS|
			ALC5623_STEREO_AD_DA_CLK_CTRL_ADDA_OSR_HIGH
			//ALC5623_STEREO_AD_DA_CLK_CTRL_ADDA_OSR_LOW
	);//2a69 24k 16b //266d 24k 1500k

	writeCodec(ALC5623_ADD_CTRL_REG,
	 ALC5623_ADD_CTRL_REG_AUXOUT_DIF
	|ALC5623_ADD_CTRL_REG_CLASSAB_VMID_1_50VDD
	|ALC5623_ADD_CTRL_REG_CLASSD_VMID_1_50VDD
	|ALC5623_ADD_CTRL_REG_DAC_HPF_EN
	|ALC5623_ADD_CTRL_REG_ADC_HPF_EN
	|ALC5623_ADD_CTRL_REG_CLASSAB_MODEBTL
	);


	return ;
}