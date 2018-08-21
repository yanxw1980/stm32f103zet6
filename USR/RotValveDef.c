/*******************************************************************************
 * @file    RotateValveDef.c    
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
#include "RotValveDef.h" 
#include "BoardIoDef.h" 
#include "HexProtocol.h" 

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static u8  RotValveStat  = MODULE_STAT_DISCONNECT;
static u16 RotValveErrNo = SYS_ERR_NONE;

/* Private function prototypes ------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 
extern uint8_t UART_SendHexToRValve(uint8_t *p_str, uint8_t len);

/*******************************************************************************
 * @brief  设置旋转阀的控制状态 
 * @input  stat,状态
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void RV_SetCurrStat(u8 stat)
{
	// 未联接的情况下设置超时则认为通信问题
	if( MODULE_STAT_DISCONNECT == RotValveStat )
	{
		if( (MODULE_STAT_ERR_TIMEOUT == stat) || (MODULE_STAT_ERR_PARA == stat) )
			RotValveStat = MODULE_STAT_ERR_COMM;
		else
			RotValveStat = stat;
	}
	else
	{
		RotValveStat = stat;
	}
}

/*******************************************************************************
 * @brief  获取旋转阀的控制状态 
 * @input  
 * @return 当前状态
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 RV_GetCurrStat(void)
{
	return RotValveStat;
}

/*******************************************************************************
 * @brief  设置旋转阀的错误码 
 * @input  err,错误码
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void RV_SetErrorNumber(u16 err)
{
	RotValveErrNo = err;	
}

/*******************************************************************************
 * @brief  返回旋转阀的错误码 
 * @input  
 * @return 错误码
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 RV_GetErrorNumber(void)
{
	return RotValveErrNo;	
}

/*******************************************************************************
 * @brief  To move the valve to position #n
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE RV_MoveToPosition( BYTE ValvePos )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;
	u8 bytehigh;
	u8 bytelow;
	
	pSndBuff = aSndBuff;
	
	bytehigh = (ValvePos>>4)&0x0F;
	if( bytehigh<10 )
		bytehigh += '0';
	else
		bytehigh += 'A';
	
	bytelow = ValvePos&0x0F;
	if( bytelow<10 )
		bytelow += '0';
	else
		bytelow += 'A';
	
	*pSndBuff++ = 'P';
	*pSndBuff++ = bytehigh;
	*pSndBuff++ = bytelow;
	*pSndBuff++ = 0x0D;
	
	return UART_SendHexToRValve (aSndBuff, 4);
}


/*******************************************************************************
 * @brief  Requests vavle status
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE RV_QueryStatus( void )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;

	pSndBuff = aSndBuff;
	
	*pSndBuff++ = 'S';
	*pSndBuff++ = 0x0D;
	
	return UART_SendHexToRValve (aSndBuff, 2);
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
BYTE RV_Home( void )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;

	pSndBuff = aSndBuff;
	
	*pSndBuff++ = 'M';
	*pSndBuff++ = 0x0D;
	
	return UART_SendHexToRValve (aSndBuff, 2);
}

/*******************************************************************************
 * @brief  reads firmware revision
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE RV_QueryVersion( void )
{	
	u8 aSndBuff[10];
	u8 *pSndBuff;

	pSndBuff = aSndBuff;
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	return UART_SendHexToRValve (aSndBuff, 2);
}




