/*******************************************************************************
 * @file    SpiDrv.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-07
 * @brief   定义SPI函数接口定义 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_DRV_H
#define __SPI_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"
#include "BoardIoDef.h"

/* Macros ----------------------------------------------------------------------*/
#define sFLASH_DUMMY_BYTE 0x55	 
	 
typedef enum
{
	SPI_NO_1     = 0,
	SPI_NO_2        ,
	SPI_NO_MAX      ,
}emSpiNo;	

/* Types    --------------------------------------------------------------------*/	 
	 
/* Structure definition---------------------------------------------------------*/



/* Functions -------------------------------------------------------------------*/
void SPI_ParaInit(BYTE nIdx);
u16  SPI_SendReadNumByte(u8 spiPort, const u8 *pSndBuf, u16 sndNum, u8 *pReadBuf, u16 ReadNum, u32 TimeOut);

#ifdef __cplusplus
}
#endif

#endif // __STM32_EVAL_SPI_FLASH_H 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
