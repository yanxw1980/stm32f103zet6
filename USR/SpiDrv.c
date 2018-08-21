/*******************************************************************************
 * @file    SpiDrv.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-06
 * @brief   SPI FLASH 读写驱动
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes ------------------------------------------------------------------*/ 
#include <bsp.h>
#include "SpiDrv.h"
#include <string.h>

/* Private typedef ------------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro --------------------------------------------------------------*/
#define SPI_BUFF_MAX       256

/* Private variables -----------------------------------------------------------*/
static u8   spiBuff[SPI_NO_MAX][SPI_BUFF_MAX];

static u16  spiWriteNum[SPI_NO_MAX];        // 写的总数
static u16  spiWriteIdx[SPI_NO_MAX];        // 写的当前编号
static u16  spiReadNum[SPI_NO_MAX];         // 读的总数
static u16  spiReadIdx[SPI_NO_MAX];         // 读的当前编号

OS_SEM semSpiMsg[SPI_NO_MAX];

SPI_InitTypeDef  stAdc7689SpiSets;

/* Private function prototypes -------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 

/*******************************************************************************
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void BSP_SPI_1_ISR_Handler (void)
{
	OS_ERR  err;
	u8      tmp;
	SPI_TypeDef *pSpiHandle;
	OS_SEM      *pSemMsg;
	u8           SpiNo;
	
	SpiNo      = SPI_NO_1;
	pSpiHandle = SPI1;
	
	pSemMsg    = &semSpiMsg[SpiNo];
	if( SET==SPI_I2S_GetFlagStatus(pSpiHandle, SPI_I2S_FLAG_RXNE) )
	{
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_RXNE, DISABLE);
		
		tmp = SPI_I2S_ReceiveData(pSpiHandle);
		
		if( spiWriteIdx[SpiNo]>spiWriteNum[SpiNo] )
		{
			spiBuff[SpiNo][spiReadIdx[SpiNo]] = tmp;
			spiReadIdx[SpiNo] += 1;
						
			if( spiReadIdx[SpiNo]>=spiReadNum[SpiNo] )
			{
				SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, DISABLE);
				OSSemPost(pSemMsg, OS_OPT_POST_1, &err); 
			}
			else
			{
				SPI_I2S_SendData(pSpiHandle, sFLASH_DUMMY_BYTE);
				spiWriteIdx[SpiNo] += 1;
			}
		}		
		
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE, ENABLE);
	}
	
	if( SET==SPI_I2S_GetFlagStatus(pSpiHandle, SPI_I2S_FLAG_TXE) )
	{
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE, DISABLE);
		
		if( spiWriteIdx[SpiNo]<spiWriteNum[SpiNo] )
		{
			SPI_I2S_SendData(pSpiHandle, spiBuff[SpiNo][spiWriteIdx[SpiNo]]);
			spiWriteIdx[SpiNo] += 1;
		}
		else if( spiWriteIdx[SpiNo]==spiWriteNum[SpiNo] )
		{
			if( spiReadNum[SpiNo]>0 )
			{
				SPI_I2S_SendData(pSpiHandle, sFLASH_DUMMY_BYTE);
				spiWriteIdx[SpiNo] += 1;
			}
			else
			{
				SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, DISABLE);
					
				OSSemPost(pSemMsg, OS_OPT_POST_1, &err); 
			}
		}
		
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_RXNE, ENABLE);
	}	
}

/*******************************************************************************
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void BSP_SPI_2_ISR_Handler (void)
{
	OS_ERR  err;
	u8      tmp;
	SPI_TypeDef *pSpiHandle;
	OS_SEM      *pSemMsg;
	u8           SpiNo;
	u8           bDone;
	
	bDone      = FALSE;
	
	SpiNo      = SPI_NO_2;
	pSpiHandle = SPI2;
	
	pSemMsg    = &semSpiMsg[SpiNo];
	if( SET==SPI_I2S_GetFlagStatus(pSpiHandle, SPI_I2S_FLAG_RXNE) )
	{
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_RXNE, DISABLE);
		
		tmp = SPI_I2S_ReceiveData(pSpiHandle);
		
		if( spiWriteIdx[SpiNo]>spiWriteNum[SpiNo] )
		{
			spiBuff[SpiNo][spiReadIdx[SpiNo]] = tmp;
			spiReadIdx[SpiNo] += 1;
						
			if( spiReadIdx[SpiNo]>=spiReadNum[SpiNo] )
			{
				SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, DISABLE);
				OSSemPost(pSemMsg, OS_OPT_POST_1, &err); 
				
				bDone = TRUE;
			}
			else
			{
				SPI_I2S_SendData(pSpiHandle, sFLASH_DUMMY_BYTE);
				spiWriteIdx[SpiNo] += 1;
			}
		}		
		
		if( FALSE==bDone )
			SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE, ENABLE);
	}	
	else if( SET==SPI_I2S_GetFlagStatus(pSpiHandle, SPI_I2S_FLAG_TXE) )
	{
		SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE, DISABLE);
		
		if( spiWriteIdx[SpiNo]<spiWriteNum[SpiNo] )
		{
			SPI_I2S_SendData(pSpiHandle, spiBuff[SpiNo][spiWriteIdx[SpiNo]]);
			spiWriteIdx[SpiNo] += 1;
		}
		else if( spiWriteIdx[SpiNo]==spiWriteNum[SpiNo] )
		{
			if( spiReadNum[SpiNo]>0 )
			{
				SPI_I2S_SendData(pSpiHandle, sFLASH_DUMMY_BYTE);
				spiWriteIdx[SpiNo] += 1;
			}
			else
			{
				SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE|SPI_I2S_IT_ERR, DISABLE);
					
				OSSemPost(pSemMsg, OS_OPT_POST_1, &err); 
				
				bDone = TRUE;
			}
		}
		
		if( FALSE==bDone )
			SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_RXNE, ENABLE);
	}	
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
void SPI_ParaInit(BYTE nIdx)
{
	SPI_InitTypeDef  SPI_InitStructure;

	if( SPI_NO_1==nIdx )
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		
		// SPI configuration 
		stAdc7689SpiSets.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		stAdc7689SpiSets.SPI_Mode = SPI_Mode_Master;
		stAdc7689SpiSets.SPI_DataSize = SPI_DataSize_16b;
		stAdc7689SpiSets.SPI_CPOL = SPI_CPOL_Low;
		stAdc7689SpiSets.SPI_CPHA = SPI_CPHA_1Edge;
		stAdc7689SpiSets.SPI_NSS = SPI_NSS_Soft;
		stAdc7689SpiSets.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;

		stAdc7689SpiSets.SPI_FirstBit = SPI_FirstBit_MSB;
		stAdc7689SpiSets.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &stAdc7689SpiSets);
		
		BSP_IntVectSet(BSP_INT_ID_SPI1, BSP_SPI_1_ISR_Handler);
//		BSP_IntEn(BSP_INT_ID_SPI1);
		BSP_IntDis(BSP_INT_ID_SPI1);
		
//		BSP_OS_SemCreate(&semSpiMsg[SPI_NO_1], 0, (CPU_CHAR *)"Spi1Msg");
		
		// Enable the sFLASH_SPI  
		SPI_Cmd(SPI1, ENABLE);
	}
	else if( SPI_NO_2==nIdx )
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		
		// SPI configuration 
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;

		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI2, &SPI_InitStructure);
		
		BSP_IntVectSet(BSP_INT_ID_SPI2, BSP_SPI_2_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_SPI2);
		
		BSP_OS_SemCreate(&semSpiMsg[SPI_NO_2], 0, (CPU_CHAR *)"Spi2Msg");
		
		// Enable the sFLASH_SPI  
		SPI_Cmd(SPI2, ENABLE);
	}
}


/*******************************************************************************
 * @brief  Sends bytes through the SPI interface and return the byte received
 *         from the SPI bus.
 * @input  byte: byte to send.
 * @return The number of the received byte.
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
u16 SPI_SendReadNumByte(u8 spiPort, const u8 *pSndBuf, u16 sndNum, u8 *pReadBuf, u16 ReadNum, u32 TimeOut)
{
	OS_ERR   err;
	SPI_TypeDef *pSpiHandle;	
	
	if( spiPort>SPI_NO_MAX )
		return 0;
	
	if( (0==sndNum)&&(0==ReadNum) )
		return 0;
	
	if( (NULL==pReadBuf)&&(NULL==pSndBuf) )
		return 0;
	
	if( (sndNum>SPI_BUFF_MAX)||(ReadNum>SPI_BUFF_MAX) )
		return 0;	
	
	spiWriteNum[spiPort] = sndNum;        // 写的总数
	spiWriteIdx[spiPort] = 0;             // 写的当前编号
	spiReadNum[spiPort]  = ReadNum;       // 读的总数
	spiReadIdx[spiPort]  = 0;             // 读的当前编号
	memcpy(&spiBuff[spiPort][0], pSndBuf, spiWriteNum[spiPort]);	

	if( SPI_NO_2 == spiPort )
		pSpiHandle = SPI2;
	else if( SPI_NO_1 == spiPort )
		pSpiHandle = SPI1;
	else
		return 0;
	
	OSSemSet(&semSpiMsg[spiPort], 0, &err);
	
	SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE, ENABLE);
	 
	OSSemPend(&semSpiMsg[spiPort], TimeOut, OS_OPT_PEND_BLOCKING, NULL, &err); 
	
	// Turn off interrupt
	SPI_I2S_ITConfig(pSpiHandle, SPI_I2S_IT_TXE|SPI_I2S_IT_RXNE, DISABLE);	

	if( spiReadNum>0 )
	{
		memcpy(pReadBuf, spiBuff[spiPort], spiReadIdx[spiPort]);		
	}
	
	return spiReadIdx[spiPort];
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
