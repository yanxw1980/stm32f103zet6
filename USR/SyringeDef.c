/*******************************************************************************
 * @file    SyringeDef.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-11-23
 * @brief   旋转阀的控制定义
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes -----------------------------------------------------------------*/ 
#include <string.h>
#include <stdio.h>
#include "SysError.h"
#include "UartDrv.h" 
#include "SyringeDef.h" 

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/



/* Private function prototypes ------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 


/*******************************************************************************
 * @brief  initial system
 * @input  2F 31 5A 52 0D 0A
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE SYR_InitSystem( BYTE addr )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;
	u8 len;
	
	pSndBuff = aSndBuff;
	
	if( addr>16 )
		return FALSE;
	
	*pSndBuff++ = HEADER;
	*pSndBuff++ = 0x30 + addr;
	*pSndBuff++ = 0x5A;
	*pSndBuff++ = 0x52;
	*pSndBuff++ = CR;
	*pSndBuff++ = LF;
	
	len = pSndBuff - aSndBuff;
	
	return UART_SendNumChar( UART_SYR, aSndBuff, len );
}


/*******************************************************************************
 * @brief  query status
 * @input  2F 31 51 0D 0A
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE SYR_QueryStatus( BYTE addr )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;
	u8 len;
	
	pSndBuff = aSndBuff;
	
	if( addr>16 )
		return FALSE;
	
	*pSndBuff++ = HEADER;
	*pSndBuff++ = 0x30 + addr;
	*pSndBuff++ = 0x51;
	*pSndBuff++ = CR;
	*pSndBuff++ = LF;
	
	len = pSndBuff - aSndBuff;
	
	return UART_SendNumChar( UART_SYR, aSndBuff, len );
}

/*******************************************************************************
 * @brief  commands the valve to the home position
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/


/*******************************************************************************
 * @brief  reads firmware revision
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/





