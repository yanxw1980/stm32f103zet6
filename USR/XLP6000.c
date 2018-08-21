/*******************************************************************************
 * @file    XLP6000.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-08
 * @brief   注射泵的控制定义
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes -----------------------------------------------------------------*/ 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "SysError.h"
#include "XLP6000.h" 
#include "BoardIoDef.h" 
#include "HexProtocol.h" 

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define  MODULE_ADDR       '1'

/* Private variables ---------------------------------------------------------*/
static u8  SyrPumpStat  = MODULE_STAT_DISCONNECT;
static u16 SyrPumpErrNo = SYS_ERR_NONE;

/* Private function prototypes ------------------------------------------------*/
static u16 XLP_VolumeToPulse(u16 VolumeUl);
static u16 XLP_SpeedToMaxFreq(u8 Speed);

/* Private functions -----------------------------------------------------------*/ 
extern uint8_t UART_SendHexToXLP6000(uint8_t *p_str, uint8_t len);

/*******************************************************************************
 * @brief  设置注射泵的控制状态 
 * @input  stat,状态
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void  XLP_SetCurrStat(u8 stat)
{
	// 未联接的情况下设置超时则认为通信问题
	if( MODULE_STAT_DISCONNECT == SyrPumpStat )
	{
		if( (MODULE_STAT_ERR_TIMEOUT == stat) || (MODULE_STAT_ERR_PARA == stat) )
			SyrPumpStat = MODULE_STAT_ERR_COMM;
		else
			SyrPumpStat = stat;
	}
	else
	{
		SyrPumpStat = stat;
	}
}

/*******************************************************************************
 * @brief  获取注射泵的控制状态 
 * @input  
 * @return 当前状态
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8  XLP_GetCurrStat(void)
{
	return SyrPumpStat;
}

/*******************************************************************************
 * @brief  设置注射泵的错误码 
 * @input  err,错误码
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void XLP_SetErrorNumber(u16 err)
{
	SyrPumpErrNo = err;	
}

/*******************************************************************************
 * @brief  返回注射泵的错误码 
 * @input  
 * @return 错误码
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 XLP_GetErrorNumber(void)
{
	return SyrPumpErrNo;	
}


/*******************************************************************************
 * @brief  
 * @input  
 *         VolUl: volume(ul)
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static u8 XLP_GetValveString(u8 *pstrBuf, u8 ValveNo)
{	
	u8  Index;

	Index = 1;
	
	switch(ValveNo)
	{
		case SYR_VAL_CHN_1_SYR:
			*pstrBuf++ = 'O';
			break;
		case SYR_VAL_CHN_2_SYR:
			*pstrBuf++ = 'I';
			break;
		case SYR_VAL_CHN_1_2:
			*pstrBuf++ = 'B';
			break;
		default:
			Index = 0;
			break;
	}	
	
	return Index;
}

/*******************************************************************************
 * @brief  pump home
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 XLP_Home(u8 *pstrBuf, u32 VolxUl, u8 InValveNo, u8 OutValveNo)
{	
	u8 aSndBuff[20];
	u8 *pSndBuff;
	u8 Index;
	
	InValveNo  = InValveNo;
	OutValveNo = OutValveNo;
	
	pSndBuff = aSndBuff;	
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	*pSndBuff++ = 'Z';
	Index = 3;		
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}

//u8 XLP_Home(u8 *pstrBuf, u32 VolxUl, u8 InValveNo, u8 OutValveNo)
//{	
//	u8 aSndBuff[20];
//	u8 *pSndBuff;
//	u8 Index;
//	
//	pSndBuff = aSndBuff;
//	
//	Index = 0;
//	
//	*pSndBuff++ = '/';
//	*pSndBuff++ = MODULE_ADDR;
//	*pSndBuff++ = 'Z';
//	Index = 3;
//	
//	if( VolxUl>=1000 )
//	{
//		*pSndBuff++ = '0';
//		
//		Index += 1;
//	}
//	else if( (250==VolxUl)||(500==VolxUl) )
//	{
//		*pSndBuff++ = '1';
//		
//		Index += 1;
//	}
//	else if( (50==VolxUl)||(100==VolxUl) )
//	{
//		*pSndBuff++ = '2';
//		
//		Index += 1;
//	}
//	
//	// port 1-input, 2-bypass, 3-output
//	if( (InValveNo>0) && (InValveNo<=SYR_VAL_CHN_MAX) )
//	{
//		*pSndBuff++ = ',';
//		*pSndBuff++ = 0x30 + InValveNo;
//		*pSndBuff++ = ',';
//		Index += 3; 
//	}
//	else
//	{
//		*pSndBuff++ = ',';
//		*pSndBuff++ = ',';
//		Index += 2; 
//	}
//	
//	if( (OutValveNo>0) && (OutValveNo<=SYR_VAL_CHN_MAX) )
//	{
//		*pSndBuff++ = 0x30 + OutValveNo;
//		Index += 1; 
//	}
//	
//	*pSndBuff++ = 'R';
//	*pSndBuff++ = 0x0D;
//	
//	Index += 2;	
//	
//	if( NULL != pstrBuf )
//	{
//		memcpy(pstrBuf, pSndBuff, Index);
//		
//		return Index;
//	}
//	else
//	{
//		return UART_SendHexToXLP6000(aSndBuff, Index);
//	}
//}


/*******************************************************************************
 * @brief  Requests vavle status
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
uint8_t XLP_ValveOn(uint8_t *pstrBuf, uint8_t ValveNo)
{	
	u8 aSndBuff[20];
	u8 *pSndBuff;
	u8 Index;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;
	
	if( XLP_GetValveString(pSndBuff, ValveNo) )
	{
		pSndBuff++;
		Index += 1;
	}	
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}

/*******************************************************************************
 * @brief  绝对位置运动
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static u16 XLP_VolumeToPulse(u16 VolumeUl)
{
	u16 Pulse;
	
	if( SYS_TUBE_NUMBER > 1 )
		VolumeUl = (VolumeUl / SYS_TUBE_NUMBER) + ( (0==(VolumeUl%SYS_TUBE_NUMBER))?0:1 );
		
	if( VolumeUl > SYS_TUBE_VOLUME )	
		VolumeUl = SYS_TUBE_VOLUME;
	
	Pulse = (u32)VolumeUl * 6000 / SYS_TUBE_VOLUME;
	
	return Pulse;
}

/*******************************************************************************
 * @brief  绝对位置运动
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static u16 XLP_SpeedToMaxFreq(u8 Speed)
{
	u16 MaxFreq;
	
	if( SYS_TUBE_NUMBER > 1 )
		Speed = (Speed / SYS_TUBE_NUMBER) + ( (0==(Speed%SYS_TUBE_NUMBER))?0:1 );
	
	MaxFreq = (u32)Speed * 6000 / SYS_TUBE_VOLUME;
	
	return MaxFreq;
}

/*******************************************************************************
 * @brief  绝对位置运动
 * @input  need to make sure inhale or drain
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 XLP_AbsMove(u8 *pstrBuf, u8 ValveNo, u8 Model, u16 Pulse)
{	
	u8 strPulse[6];
	u8 aSndBuff[20];	
	u8 *pSndBuff;
	u8 Index;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;
	
	if( XLP_GetValveString(pSndBuff, ValveNo) )
	{
		pSndBuff++;
		Index += 1;
	}	
		
	if( Model==MTR_MODEL_NORM )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '0';	
		Index += 2;		
	}
	else if( Model==MTR_MODEL_MICRO )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '1';	
		Index += 2;		
	}
	
	*pSndBuff++ = 'A';	
	Index += 1;		
	
	// 8细分
	if( (Model==MTR_MODEL_MICRO)&&(Pulse<48000)&&( (Pulse<<3)<=48000 ) )
		Pulse <<= 3;
	
	sprintf((char *)strPulse,"%d", Pulse);
	memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
	Index += strlen((char *)strPulse);
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}

/*******************************************************************************
 * @brief  吸液
 * @input  
 *         VolUl: volume(ul)
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 XLP_RelInhale(u8 *pstrBuf, u8 ValveNo, u8 Model, u8 Speed, u16 VolUl)
{	
	u8  strPulse[6];
	u8  aSndBuff[20];	
	u8  *pSndBuff;
	u8  Index;
	u16 MaxSpeed;
	u16 Pulse;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;
	
//	if( XLP_GetValveString(pSndBuff, ValveNo) )
//	{
//		pSndBuff++;
//		Index += 1;
//	}
	*pSndBuff++ = 'O';	
	Index += 1;	
		
	if( Model==MTR_MODEL_NORM )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '0';	
		Index += 2;		
	}
	else if( Model==MTR_MODEL_MICRO )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '1';	
		Index += 2;		
	}
	
	// Max speed
	MaxSpeed = XLP_SpeedToMaxFreq( Speed );  //Speed * 6000 / 1000;
//	if( MaxSpeed<50 )
//		MaxSpeed = 50;	
		
	*pSndBuff++ = 'V';	
	Index += 1;	
	
	sprintf((char *)strPulse, "%d", MaxSpeed);
	memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
	Index += strlen((char *)strPulse);	
	pSndBuff += strlen((char *)strPulse);
	/////////
	
	
	*pSndBuff++ = 'P';	
	Index += 1;		
	
	Pulse = XLP_VolumeToPulse(VolUl);
	
	// 8细分
	if( Model==MTR_MODEL_MICRO )
		Pulse <<= 3;
	
	sprintf((char *)strPulse,"%d", Pulse);
	memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
	Index += strlen((char *)strPulse);
	pSndBuff += strlen((char *)strPulse);
	
	// turn off valve
	*pSndBuff++ = 'I';	
	Index += 1;	
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}


/*******************************************************************************
 * @brief  排液
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
uint8_t XLP_RelDrain(uint8_t *pstrBuf, u8 ValveNo, uint8_t Model, u8 Speed, uint16_t VolUl)
{	
	u8  strPulse[6];
	u8  aSndBuff[20];	
	u8  *pSndBuff;
	u8  Index;
	u16 MaxSpeed;
	u16 Pulse;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;
	
//	if( XLP_GetValveString(pSndBuff, ValveNo) )
//	{
//		pSndBuff++;
//		Index += 1;
//	}
	// turn off valve
	*pSndBuff++ = 'I';	
	Index += 1;	
		
	if( Model==MTR_MODEL_NORM )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '0';	
		Index += 2;		
	}
	else if( Model==MTR_MODEL_MICRO )
	{
		*pSndBuff++ = 'N';	
		*pSndBuff++ = '1';	
		Index += 2;		
	}

	// Max speed
	MaxSpeed = XLP_SpeedToMaxFreq( Speed );  //Speed * 6000 / 1000;
//	if( MaxSpeed<50 )
//		MaxSpeed = 50;
	
	*pSndBuff++ = 'V';	
	Index += 1;	
	
	sprintf((char *)strPulse,"%d", MaxSpeed);
	memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
	Index += strlen((char *)strPulse);	
	pSndBuff += strlen((char *)strPulse);
	/////////
	
	*pSndBuff++ = 'D';	
	Index += 1;		
	
	
	Pulse = XLP_VolumeToPulse(VolUl);
	
	// 8细分
	if( Model==MTR_MODEL_MICRO )
		Pulse <<= 3;
	
	sprintf((char *)strPulse,"%d", Pulse);
	memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
	Index += strlen((char *)strPulse);
	pSndBuff += strlen((char *)strPulse);
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}

/*******************************************************************************
 * @brief  速度设置
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
uint8_t XLP_SetSpeed(uint8_t *pstrBuf, uint16_t StartSpeed, uint16_t MaxSpeed, uint16_t StopSpeed)
{	
	u8 strPulse[6];
	u8 aSndBuff[30];	
	u8 *pSndBuff;
	u8 Index;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;
		
	if( (StartSpeed>=SPEED_START_MIN)&&(StartSpeed<=SPEED_START_MAX) )
	{
		*pSndBuff++ = 'v';	
		Index += 1;	
		
		sprintf((char *)strPulse, "%d", StartSpeed);
		memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
		Index += strlen((char *)strPulse);
	}
	
	if( (MaxSpeed>=SPEED_MAX_MIN)&&(MaxSpeed<=SPEED_MAX_MAX) )
	{
		*pSndBuff++ = 'V';	
		Index += 1;	
		
		sprintf((char *)strPulse, "%d", MaxSpeed);
		memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
		Index += strlen((char *)strPulse);
	}
	
	if( (StopSpeed>=SPEED_STOP_MIN)&&(StopSpeed<=SPEED_STOP_MAX) )
	{
		*pSndBuff++ = 'c';	
		Index += 1;	
		
		sprintf((char *)strPulse, "%d", StopSpeed);
		memcpy(pSndBuff, strPulse, strlen((char *)strPulse));
	
		Index += strlen((char *)strPulse);
	}
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}

/*******************************************************************************
 * @brief  状态查询
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
uint8_t XLP_QueryStatus(uint8_t *pstrBuf)
{	
	u8 aSndBuff[10];	
	u8 *pSndBuff;
	u8 Index;
	
	pSndBuff = aSndBuff;
	
	Index = 0;
	
	*pSndBuff++ = '/';
	*pSndBuff++ = MODULE_ADDR;
	
	Index = 2;		
	
	*pSndBuff++ = 'Q';
	
	Index += 1;	
	
	*pSndBuff++ = 'R';
	*pSndBuff++ = 0x0D;
	
	Index += 2;	
	
	if( NULL != pstrBuf )
	{
		memcpy(pstrBuf, pSndBuff, Index);
		
		return Index;
	}
	else
	{
		return UART_SendHexToXLP6000(aSndBuff, Index);
	}
}


