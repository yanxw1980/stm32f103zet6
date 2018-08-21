/*******************************************************************************
 * @file    SpiFlash.c    
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
#include "SpiFlash.h"
#include <string.h>

/* Private typedef ------------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro --------------------------------------------------------------*/

/* Private variables -----------------------------------------------------------*/
static uint8_t  SectorBuf[sFLASH_SECTOR_SIZE];	

/* Private function prototypes -------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 
static void sFLASH_WaitForWriteEnd(void);
static void sFLASH_WriteEnable(void);
static void sFLASH_WriteDisable(void);

/*******************************************************************************
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @input  None.
 * @return None.
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void sFLASH_Init(void)
{
	SPI_ParaInit(SPI_NO_2);
}

/*******************************************************************************
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @input  None.
 * @return None.
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
static void sFLASH_WaitForWriteEnd(void)
{
	u8   SndBuf[2];
	u8   ReadBuf[2];
	u16  len;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();

	// Send "Read Status Register" instruction 
	SndBuf[0] = sFLASH_CMD_RDSR;
	
	len = 0;
	while( 1 )
	{
		len = SPI_SendReadNumByte(SPI_NO_2, SndBuf, 1, ReadBuf, 1, 10);
		
		if( (len>0) && (sFLASH_WIP_FLAG!=(ReadBuf[0] & sFLASH_WIP_FLAG)))
			break;
	}

	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();
}

/*******************************************************************************
 * @brief  Reads FLASH identification.
 * @input  None.
 * @return FLASH identification.
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
uint32_t sFLASH_ReadID(void)
{
	uint8_t SndBuf[2];
	uint8_t ReadBuf[4];
	uint16_t len;
	
	SndBuf[0] = 0x9F;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	len = SPI_SendReadNumByte(SPI_NO_2, SndBuf, 1, ReadBuf, 3, 100);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();
	
	if( len>=3 )
		return (ReadBuf[0] << 16) | (ReadBuf[1] << 8) | ReadBuf[2];
	else
		return 0x00;
}

/*******************************************************************************
 * @brief  Erases the specified FLASH sector.
 * @input  SectorAddr: address of the sector to erase.
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void sFLASH_EraseSector(uint32_t SectorAddr)
{
	uint8_t SndBuf[5];
	
	// Send write enable instruction 
	sFLASH_WriteEnable();

	SndBuf[0] = sFLASH_CMD_SE;
	SndBuf[1] = (SectorAddr & 0xFF0000) >> 16;
	SndBuf[2] = (SectorAddr & 0xFF00) >> 8;
	SndBuf[3] = SectorAddr & 0xFF;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	SPI_SendReadNumByte(SPI_NO_2, SndBuf, 4, NULL, 0, 10);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();

	// Wait the end of Flash writing 
	sFLASH_WaitForWriteEnd();
}

/*******************************************************************************
 * @brief  Erases the entire FLASH.
 * @input  None
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void sFLASH_EraseBulk(void)
{
	uint8_t SndBuf[2];
	
	// Send write enable instruction
	sFLASH_WriteEnable();

	// Bulk Erase 
	SndBuf[0] = sFLASH_CMD_BE;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	SPI_SendReadNumByte(SPI_NO_2, SndBuf, 1, NULL, 0, 10);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();

	// Wait the end of Flash writing 
	sFLASH_WaitForWriteEnd();
}

/*******************************************************************************
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle 
 *         (Page WRITE sequence).
 * @input  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 *         WriteAddr: FLASH's internal address to write to.
 *         NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
#ifndef W25Q16 
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint8_t  SndBuf[520];		
	uint32_t SectorBase;
	uint16_t Offset;
	
	if( NumByteToWrite>sFLASH_SPI_PAGESIZE )
		NumByteToWrite = sFLASH_SPI_PAGESIZE;
	
	// 读出扇区数据
	SectorBase = WriteAddr/sFLASH_SPI_PAGESIZE;
	SectorBase = SectorBase * sFLASH_SPI_PAGESIZE;
	sFLASH_ReadBuffer(SndBuf+4, SectorBase, sFLASH_SPI_PAGESIZE);
	
	// 擦除扇区数据
	sFLASH_EraseSector( SectorBase );
	
	///////////
	// 读出扇区数据
	SectorBase = WriteAddr/sFLASH_SPI_PAGESIZE;
	SectorBase = SectorBase * sFLASH_SPI_PAGESIZE;
	sFLASH_ReadBuffer(SndBuf+4, SectorBase, sFLASH_SPI_PAGESIZE);
	////////////////
	
	// 更新数据
	Offset = WriteAddr%sFLASH_SPI_PAGESIZE;
	if( (Offset+NumByteToWrite)<=sFLASH_SPI_PAGESIZE )
		memcpy( SndBuf + 4 + Offset, pBuffer, NumByteToWrite );
	else
		memcpy( SndBuf + 4 + Offset, pBuffer, sFLASH_SPI_PAGESIZE - Offset );

	
	// 写回数据
	// Enable the write access to the FLASH 
	sFLASH_WriteEnable();
	
	SndBuf[0] = sFLASH_CMD_WRITE;
	SndBuf[1] = (WriteAddr & 0xFF0000) >> 16;
	SndBuf[2] = (WriteAddr& 0xFF00) >> 8;
	SndBuf[3] = WriteAddr & 0xFF;	

	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	SPI_SendReadNumByte(SPI_NO_2, SndBuf, sFLASH_SPI_PAGESIZE, NULL, 0, 100);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();

	// Wait the end of Flash writing 
	sFLASH_WaitForWriteEnd();
	
	/////////
	// 读出扇区数据
	memset(SndBuf+4, 0x00, sFLASH_SPI_PAGESIZE);
	SectorBase = WriteAddr/sFLASH_SPI_PAGESIZE;
	SectorBase = SectorBase * sFLASH_SPI_PAGESIZE;
	sFLASH_ReadBuffer(SndBuf+4, SectorBase, sFLASH_SPI_PAGESIZE);
}
#endif

/*******************************************************************************
 * @brief  Writes sector of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @input  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 *         WriteAddr: FLASH's internal address to write to.
 *         NumByteToWrite: number of bytes to write to the FLASH.
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
//暂时不考虑跨扇区写
#ifdef W25Q16
void sFLASH_SectorWrite(uint8_t* pBuffer, uint32_t SectorBase)
{
	uint8_t  SndBuf[4];	
	uint8_t  idx;
	uint8_t *pDatBuff;
	
	pDatBuff = pBuffer;
	
	// 写回数据
	// Enable the write access to the FLASH 
	sFLASH_WriteEnable();
	
	for(idx=0; idx<(sFLASH_SECTOR_SIZE/sFLASH_PAGE_SIZE); idx++)
	{			
		SndBuf[0] = sFLASH_CMD_WRITE;
		SndBuf[1] = (SectorBase & 0xFF0000) >> 16;
		SndBuf[2] = (SectorBase& 0xFF00) >> 8;
		SndBuf[3] = SectorBase & 0xFF;	

		// Select the FLASH: Chip Select low 
		sFLASH_CS_LOW();
		
		SPI_SendReadNumByte(SPI_NO_2, SndBuf, 4, NULL, 0, 10);
		
		SPI_SendReadNumByte(SPI_NO_2, pDatBuff+SectorBase, sFLASH_PAGE_SIZE, NULL, 0, 10);
		
		// Deselect the FLASH: Chip Select high 
		sFLASH_CS_HIGH();

		// Wait the end of Flash writing 
		sFLASH_WaitForWriteEnd();
		
		SectorBase += sFLASH_PAGE_SIZE;
		pDatBuff   += sFLASH_PAGE_SIZE;
	}	
	
	sFLASH_WriteDisable( );
}
#endif


/*******************************************************************************
 * @brief  Read sector of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @input  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 *         WriteAddr: FLASH's internal address to write to.
 *         NumByteToWrite: number of bytes to write to the FLASH.
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
#ifdef W25Q16
void sFLASH_SectorRead(uint8_t* pBuffer, uint32_t SectorBase)
{
	uint8_t  idx;	
	uint8_t *pDatBuff = pBuffer;
	
	for(idx=0; idx<(sFLASH_SECTOR_SIZE/sFLASH_PAGE_SIZE); idx++)
	{	
		sFLASH_ReadBuffer(pDatBuff, SectorBase, sFLASH_PAGE_SIZE);
		
		pDatBuff += sFLASH_PAGE_SIZE;
		SectorBase += sFLASH_PAGE_SIZE;
	}
}
#endif

/*******************************************************************************
 * @brief  Writes sector of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @input  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 *         WriteAddr: FLASH's internal address to write to.
 *         NumByteToWrite: number of bytes to write to the FLASH.
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
//暂时不考虑跨扇区写
#ifdef W25Q16
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint32_t SectorBase;
	uint16_t Offset;
	
	if( NumByteToWrite>sFLASH_SECTOR_SIZE )
		NumByteToWrite = sFLASH_SECTOR_SIZE;	
	
	SectorBase = WriteAddr/sFLASH_SECTOR_SIZE;
	SectorBase = SectorBase * sFLASH_SECTOR_SIZE;
	
	// 读出扇区数据
	sFLASH_SectorRead(SectorBuf, SectorBase);
	
	// 擦除扇区数据
	sFLASH_EraseSector( SectorBase );	
	
//	sFLASH_SectorRead(SectorBuf, SectorBase);

	// 更新数据
	Offset = WriteAddr%sFLASH_SECTOR_SIZE;
	if( (Offset+NumByteToWrite)<=sFLASH_SECTOR_SIZE )
		memcpy( SectorBuf + Offset, pBuffer, NumByteToWrite );
	else
		memcpy( SectorBuf + Offset, pBuffer, sFLASH_SECTOR_SIZE - Offset );

	sFLASH_SectorWrite(SectorBuf, SectorBase);
	
//	sFLASH_SectorRead(SectorBuf, SectorBase);
}

#else
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

	Addr = WriteAddr % sFLASH_SPI_PAGESIZE;
	count = sFLASH_SPI_PAGESIZE - Addr;
	NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
	NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

	if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
	{
		if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
		{
			sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
		}
		else /*!< NumByteToWrite > sFLASH_PAGESIZE */
		{
			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
				WriteAddr +=  sFLASH_SPI_PAGESIZE;
				pBuffer += sFLASH_SPI_PAGESIZE;
			}

			sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
		}
	}
	else /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
	{	
		if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
		{
			if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
			{
				temp = NumOfSingle - count;

				sFLASH_WritePage(pBuffer, WriteAddr, count);
				WriteAddr +=  count;
				pBuffer += count;

				sFLASH_WritePage(pBuffer, WriteAddr, temp);
			}
			else
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
			}
		}
		else /*!< NumByteToWrite > sFLASH_PAGESIZE */
		{
			NumByteToWrite -= count;
			NumOfPage =  NumByteToWrite / sFLASH_SPI_PAGESIZE;
			NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

			sFLASH_WritePage(pBuffer, WriteAddr, count);
			WriteAddr +=  count;
			pBuffer += count;

			while (NumOfPage--)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
				WriteAddr +=  sFLASH_SPI_PAGESIZE;
				pBuffer += sFLASH_SPI_PAGESIZE;
			}

			if (NumOfSingle != 0)
			{
				sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
			}
		}
	}
}
#endif

/*******************************************************************************
 * @brief  Reads a block of data from the FLASH.
 * @input  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 *         ReadAddr: FLASH's internal address to read from.
 *         NumByteToRead: number of bytes to read from the FLASH.
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
//  /*!< Select the FLASH: Chip Select low */
//  sFLASH_CS_LOW();

//  /*!< Send "Read from Memory " instruction */
//  sFLASH_SendByte(sFLASH_CMD_READ);

//  /*!< Send ReadAddr high nibble address byte to read from */
//  sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//  /*!< Send ReadAddr medium nibble address byte to read from */
//  sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//  /*!< Send ReadAddr low nibble address byte to read from */
//  sFLASH_SendByte(ReadAddr & 0xFF);

//  while (NumByteToRead--) /*!< while there is data to be read */
//  {
//    /*!< Read a byte from the FLASH */
//    *pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
//    /*!< Point to the next location where the byte read will be saved */
//    pBuffer++;
//  }

//  /*!< Deselect the FLASH: Chip Select high */
//  sFLASH_CS_HIGH();
  
  ///////////
	uint8_t SndBuf[5];
	uint16_t len;
	
	SndBuf[0] = sFLASH_CMD_READ;
	SndBuf[1] = (ReadAddr & 0xFF0000) >> 16;
	SndBuf[2] = (ReadAddr& 0xFF00) >> 8;
	SndBuf[3] = ReadAddr & 0xFF;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	len = SPI_SendReadNumByte(SPI_NO_2, SndBuf, 4, pBuffer, NumByteToRead, 100);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();

	len = len;
}

/*******************************************************************************
 * @brief  Enables the write access to the FLASH.
 * @input  None
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
static void sFLASH_WriteEnable(void)
{	
	uint8_t SndBuf[2];

	SndBuf[0] = sFLASH_CMD_WREN;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	SPI_SendReadNumByte(SPI_NO_2, SndBuf, 1, NULL, 0, 10);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();	
}

/*******************************************************************************
 * @brief  Disables the write access to the FLASH.
 * @input  None
 * @return None
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/ 
static void sFLASH_WriteDisable(void)
{	
	uint8_t SndBuf[2];

	SndBuf[0] = sFLASH_CMD_WRDIS;
	
	// Select the FLASH: Chip Select low 
	sFLASH_CS_LOW();
	
	SPI_SendReadNumByte(SPI_NO_2, SndBuf, 1, NULL, 0, 10);
	
	// Deselect the FLASH: Chip Select high 
	sFLASH_CS_HIGH();	
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
