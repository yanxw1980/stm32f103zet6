/*******************************************************************************
 * @file    MotorDrv.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-23
 * @brief   定义电机运动函数接口定义 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _W25Q16_FLASH_H
#define _W25Q16_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"
#include "BoardIoDef.h"

/* Macros ----------------------------------------------------------------------*/	 
////  M25P SPI Flash supported commands
//#define sFLASH_CMD_WRITE          0x02  /*!< Write to Memory instruction */
//#define sFLASH_CMD_WRSR           0x01  /*!< Write Status Register instruction */
//#define sFLASH_CMD_WREN           0x06  /*!< Write enable instruction */
//#define sFLASH_CMD_READ           0x03  /*!< Read from Memory instruction */
//#define sFLASH_CMD_RDSR           0x05  /*!< Read Status Register instruction  */
//#define sFLASH_CMD_RDID           0x9F  /*!< Read identification */
//#define sFLASH_CMD_SE             0xD8  /*!< Sector Erase instruction */
//#define sFLASH_CMD_BE             0xC7  /*!< Bulk Erase instruction */

//#define sFLASH_WIP_FLAG           0x01  /*!< Write In Progress (WIP) flag */

//#define sFLASH_SPI_PAGESIZE       0x100
//#define sFLASH_SPI_SECTORSIZE     0x200

//  W25Q16 SPI Flash supported commands
#define W25Q16

#define sFLASH_CMD_WRITE          0x02  /*!< Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /*!< Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /*!< Write enable instruction */
#define sFLASH_CMD_WRDIS          0x04  /*!< Write disable instruction */
#define sFLASH_CMD_READ           0x03  /*!< Read from Memory instruction */
#define sFLASH_CMD_READ_FAST      0x0B  /*!< Fast read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /*!< Read Status Register instruction  */

#define sFLASH_CMD_RDID           0x9F  /*!< Read identification */
#define sFLASH_CMD_SE             0x20  /*!< Sector Erase instruction */
#define sFLASH_CMD_BE             0xD8  /*!< Block Erase instruction */
#define sFLASH_CMD_CE             0xC7  /*!< Chip Erase instruction */

#define sFLASH_CMD_POWER_DOWN     0xB9  /*!< Power down instruction */

#define sFLASH_WIP_FLAG           0x01  /*!< Write In Progress (WIP) flag */

#define sFLASH_PAGE_SIZE          0x100   // 256 bytes
#define sFLASH_SECTOR_SIZE        0x1000   // 4096 bytes

// memory allocate
#define FLASH_MOTOR_ADDR_BASE     0x200

/* Types    --------------------------------------------------------------------*/	 
	 
/* Structure definition---------------------------------------------------------*/



/* Functions -------------------------------------------------------------------*/
// High layer functions
void sFLASH_Init(void);
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t sFLASH_ReadID(void);

#ifdef __cplusplus
}
#endif

#endif // __STM32_EVAL_SPI_FLASH_H 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
