/**
  ******************************************************************************
  * @file    Adc7689.c
  * @author  
  * @version V1.0.0
  * @date    2017-11-27
  * @brief   
  ******************************************************************************
  * @attention  
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "Adc7689.h"
#include "string.h"
#include "SpiDrv.h"

#include "stdio.h"
#include "BoardIoDef.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint16_t  AdcValueArr[ADC_CHN_MAX][ADC_BUFF_LEN];
static uint8_t   AdcValIndex[ADC_CHN_MAX];

extern SPI_InitTypeDef  stAdc7689SpiSets;

/* Private function prototypes ------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 
void ADC_SetValue(uint8_t chn, uint16_t value);

/**
  * @brief  Initializes the peripherals used by the SPI FLASH driver.
  * @param  None
  * @retval None
  */
void ADC7689_Init(void)
{
	SPI_ParaInit(SPI_NO_1);
	
	memset(AdcValIndex, 0x00, ADC_CHN_MAX);
}

/*
**********************************************************************************************
*                                                  ADC7689 Convert
*
* Description : Get a memory block from a partition
*
* Arguments   : Sends a byte through the SPI interface 
*
*
* Returns     : A pointer to a memory block if no error is detected
*               A pointer to NULL if an error is detected
************************************************************************************************
*/
static uint16_t ADC7689_ReadWriteByte(uint16_t WriteData)
{
	// Loop while DR register in not emplty 
	while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte through the SPI peripheral 
	SPI_I2S_SendData(sADC_SPI, WriteData);		
		
	// Wait to receive a byte 
	while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_RXNE) == RESET);	
	
	// Return the byte read from the SPI bus 
	return SPI_I2S_ReceiveData(sADC_SPI);
}

/*******************************************************************************
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
__INLINE void ADC7689_ConvNoneBusy(void)
{
	SPI_Cmd(SPI1, DISABLE);
	
	stAdc7689SpiSets.SPI_CPOL = SPI_CPOL_Low;
	stAdc7689SpiSets.SPI_CPHA = SPI_CPHA_1Edge;
	stAdc7689SpiSets.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

	SPI_Init(SPI1, &stAdc7689SpiSets);
	
	SPI_Cmd(SPI1, ENABLE);
}

/*******************************************************************************
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
__INLINE void ADC7689_ConvWithBusy(void)
{
	SPI_Cmd(SPI1, DISABLE);
	
	stAdc7689SpiSets.SPI_CPOL = SPI_CPOL_High;
	stAdc7689SpiSets.SPI_CPHA = SPI_CPHA_2Edge;
	stAdc7689SpiSets.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

	SPI_Init(SPI1, &stAdc7689SpiSets);
	
	SPI_Cmd(SPI1, ENABLE);
}

/*
**********************************************************************************************
*                                                  ADC7689 Convert
*
* Description : Get a memory block from a partition
*
* Arguments   : Sends a byte through the SPI interface 
*
*
* Returns     : A pointer to a memory block if no error is detected
*               A pointer to NULL if an error is detected
************************************************************************************************
*/
#define CONV_WITH_BUSY

#ifdef CONV_WITH_BUSY
#if 1    // set then covert and receive data. it is ok, but need 25us
void ADC7689_ConvAndRecv(uint8_t chn, uint16_t SetValue, u16 AdcValue[])
{
	u8  idx;
	u16 RecvData;
	u32 WaitTime;

	// set spi 
	ADC7689_ConvWithBusy();	
	
	for( idx=0; idx<(chn+2); idx++ )
	{	
		sADC_CONV_HIGH();
			
		while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_TXE) == RESET);
		// Send byte through the SPI peripheral 
		if( 0==idx )
			SPI_I2S_SendData(sADC_SPI, SetValue);	
		else
			SPI_I2S_SendData(sADC_SPI, 0x00);
			
		sADC_CONV_LOW();	
			
		// Wait to receive a byte 
		while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_RXNE) == RESET);			
		// Return the byte read from the SPI bus 
		RecvData = SPI_I2S_ReceiveData(sADC_SPI);		
			
		WaitTime = 10;
		while( IsConverting() && WaitTime ) 
		{
			WaitTime--;
		}
		
		if( 0==idx )
			RecvData = ADC7689_ReadWriteByte(SetValue);	
		else
			RecvData = ADC7689_ReadWriteByte(0x00);
		
		if( idx>1 )
		{
	AdcValue[idx-2] = RecvData;
			
			ADC_SetValue(idx-2, RecvData);	
		}
	}
}

#else

// set at first, just convert and receive data
void ADC7689_ConvAndRecv(uint8_t chn, uint16_t SetValue, u16 AdcValue[])
{
	u8  idx;
	u16 RecvData;
	u32 WaitTime;

	// set spi configuration
	ADC7689_ConvWithBusy();		
	
	for( idx=0; idx<chn; idx++ )
	{	
		sADC_CONV_HIGH();
			
		while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_TXE) == RESET);
		// Send byte through the SPI peripheral 
		SPI_I2S_SendData(sADC_SPI, 0x00);	
			
		sADC_CONV_LOW();	
			
		// Wait to receive a byte 
		while (SPI_I2S_GetFlagStatus(sADC_SPI, SPI_I2S_FLAG_RXNE) == RESET);			
		// Return the byte read from the SPI bus 
		RecvData = SPI_I2S_ReceiveData(sADC_SPI);		
			
		WaitTime = 10;
		while( IsConverting() && WaitTime ) 
		{
			WaitTime--;
		}
			
		RecvData = ADC7689_ReadWriteByte(0x00);
AdcValue[idx] = RecvData;
		
		ADC_SetValue(idx, RecvData);	
	}	

}
#endif

#else

void ADC7689_ConvAndRecv(uint8_t chn, uint16_t SetValue, u16 AdcValue[])
{
	u8  idx; 
	u16 RecvData;
	u32 WaitTime;
	
	// set spi for write command
	ADC7689_ConvNoneBusy();
	
	for( idx=0; idx<(chn+2); idx++ )
	{	
		sADC_CONV_HIGH();
		WaitTime = 20;
		while( WaitTime ) 
		{
			WaitTime--;
		}
		sADC_CONV_LOW();		

		
		// set spi for read data

	//	if( WaitTime>0 )
		{
			if( 0==idx )
				RecvData = ADC7689_ReadWriteByte(SetValue);
			else
				RecvData = ADC7689_ReadWriteByte(0x00);
			
			if( idx>1 )
				AdcValue[idx-2] = RecvData;	
		}
	}
}
#endif

/*
**********************************************************************************************
*                                                  ADC7689 Convert
*
* Description : Get a memory block from a partition
*
* Arguments   : Sends a byte through the SPI interface 
*
*
* Returns     : A pointer to a memory block if no error is detected
*               A pointer to NULL if an error is detected
************************************************************************************************
*/
void ADC_SetValue(uint8_t chn, uint16_t value)
{
	if( chn<ADC_CHN_MAX )
	{
		AdcValueArr[chn][AdcValIndex[chn]] = value;
		
		AdcValIndex[chn] += 1;
		
		if( AdcValIndex[chn] >= ADC_BUFF_LEN )
			AdcValIndex[chn] = 0;
	}

}

/*
**********************************************************************************************
*                                                  ADC7689 Convert
*
* Description : Get a memory block from a partition
*
* Arguments   : Sends a byte through the SPI interface 
*
*
* Returns     : A pointer to a memory block if no error is detected
*               A pointer to NULL if an error is detected
************************************************************************************************
*/
uint16_t ADC_GetValue(uint8_t chn)
{
	uint8_t idx;
	
	if( chn<ADC_CHN_MAX )
	{
		if( AdcValIndex[chn] == 0 )
			idx = ADC_BUFF_LEN - 1;
		else
			idx = AdcValIndex[chn] - 1;
		
		return AdcValueArr[chn][idx];
	}
	
	return 0;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
