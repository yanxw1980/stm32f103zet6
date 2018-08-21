/*******************************************************************************
 * @file    Adc7689.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-11-28
 * @brief   ADC7689 驱动相关文件 
 ******************************************************************************
 * @attention  
 ******************************************************************************
 */
  
// Define to prevent recursive inclusion 
#ifndef _ADC_7689_SPI_H_
#define _ADC_7689_SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"
#include "BoardIoDef.h"

/* Types    ---------------------------------------------------------------------*/
#define ADC_CHN_0    0
#define ADC_CHN_1    1
#define ADC_CHN_MAX  8

#define ADC_BUFF_LEN 5

/* Macros -----------------------------------------------------------------------*/	 
#define  sADC_SPI                SPI1

#define  sADC_DUMMY_BYTE         0xa5
	 
#define  CFG_OVERWRITE           (0x01ul<<13)	 
	 
#define  INCC_BIPOLAR            (0x02ul<<10)	 	
#define  INCC_TEMP_SENSOR        (0x03ul<<10)	 	 
#define  INCC_UNIPOLAR_GND       (0x04ul<<10)	 	
#define  INCC_UNIPOLAR_COM       (0x06ul<<10)	 	
#define  INCC_UNIPOLAR_SINGLE    (0x07ul<<10)	 

#define  INN_CHN_IN0             (0x00ul<<7)
#define  INN_CHN_IN1             (0x01ul<<7)
#define  INN_CHN_IN2             (0x02ul<<7)
#define  INN_CHN_IN3             (0x03ul<<7)
#define  INN_CHN_IN4             (0x04ul<<7)
#define  INN_CHN_IN5             (0x05ul<<7)
#define  INN_CHN_IN6             (0x06ul<<7)
#define  INN_CHN_IN7             (0x07ul<<7)

#define  BW_FULL                 (0x01ul<<6)

#define  REF_INT_2500V           (0x00ul<<3)
#define  REF_INT_4096V           (0x01ul<<3)
#define  REF_EXT_TEMP            (0x02ul<<3)
#define  REF_EXT_BUFF_TEMP       (0x03ul<<3)
#define  REF_EXT                 (0x06ul<<3)
#define  REF_EXT_BUFF            (0x07ul<<3)

#define  SEQ_DISABLE_SEQ         (0x00ul<<1)
#define  SEQ_UPDATE_DURING_SEQ   (0x01ul<<1)
#define  SEQ_SCAN_THEN_TEMP      (0x02ul<<1)
#define  SEQ_SCAN                (0x03ul<<1)

#define  RB_NOT_READ_BACK        (0x01ul<<0)

/* Structure definition----------------------------------------------------------*/	

/* Functions --------------------------------------------------------------------*/	 
	 
void    ADC7689_Init(void);	
uint8_t ADC7689_RecvData(uint8_t *RecvBuf, uint8_t DatLen);
void    ADC7689_ConvAndRecv(uint8_t chn, uint16_t SetValue, uint16_t adcv[]);
void    ADC7689_CfgSettings(uint16_t SetValue);

void    ADC_SetValue(uint8_t chn, uint16_t value);
uint16_t ADC_GetValue(uint8_t chn);
#ifdef __cplusplus
}
#endif
  
#endif // __BSP_H_ 	 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/ 
