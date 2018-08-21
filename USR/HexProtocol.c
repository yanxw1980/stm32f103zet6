/*******************************************************************************
 * @file    Protocol.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-22
 * @brief   ���ڶ˵�Ӧ�ó���,��Ҫ�����ϲ������ͨ��
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes --------------------------------------------------------------------*/ 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "BoardIoDef.h"
#include "HexProtocol.h"
#include "Bsp.h"
#include "MotorBsp.h"
#include "MotorDrv.h"
#include "RotValveDef.h"
#include "XLP6000.h"
#include "SysError.h"

#ifndef PR_MODBUS

/* Private typedef -----------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static u8 cSysCurrOper;

extern OS_SEM semMtr1Ctrl;
#if defined(MOTOR_DEF_2)
extern OS_SEM semMtr2Ctrl;
#endif 

extern OS_SEM semExecMtrStat[MOTOR_MAX];              // �������״̬
extern OS_SEM semExecRoValStat;                       // ��ת������״̬
extern OS_SEM semExecSyrPumpStat;                     // ע�������״̬


//const TIMING_STEP CleanTiming[CLEAN_TIMING_STEP] = 
//{
//	0, 1,  40,  1000ul, 0, 0, 0,
//	1, 2,  40,  1000ul, 0, 0, 0,
//	2, 3,  40,  1000ul, 0, 0, 0,
//	3, 4,  40,  1000ul, 0, 250, 0xFFFF,
//	4, 5,  40,  1000ul, 0, 0, 0,
//	5, 6,  40,  1000ul, 0, 0, 0,
//	6, 7,  40,  1000ul, 0, 0, 0,
//	7, 23, 40,  1000ul, 0, 250, 0xFFFF,
//	8, 1,  0,   0,      0, 0, 0,
//};

//const TIMING_STEP PrimeTiming[PRIME_TIMING_STEP] = 
//{
//	0, 1,  20,  200ul, 0, 0, 0,
//	1, 2,  20,  200ul, 0, 0, 0,
//	2, 3,  20,  200ul, 0, 0, 0,
//	3, 4,  20,  200ul, 0, 0, 0,
//	4, 5,  20,  200ul, 0, 0, 0,
//	5, 6,  20,  200ul, 0, 0, 0,
//	6, 7,  20,  200ul, 0, 250, 0xFFFF,
//    7, 8,  40,  1000ul,0, 0, 0,
//	8, 23, 40,  1000ul,0, 250, 0xFFFF,
//	9, 1,  0,   0,     0, 0, 0,
//};

//const TIMING_STEP LoadTiming[LOAD_TIMING_STEP] = 
//{
//	0, 1,  20,  1100ul, 0,    0,   0,          // DNB
//	1, 2,  20,  200ul,  3600, 250, 0xFFFF,     // DRB
//	2, 2,  20,  2000ul, 0,    250, 0xFFFF,     // DRB
//	3, 3,  20,  2000ul, 180,  250, 0xFFFF,     // DCB
//	4, 4,  20,  2000ul, 0,    250, 0xFFFF,     // REB
//	5, 5,  20,  2000ul, 480,  250, 0xFFFF,     // PWB
//	6, 3,  20,  2000ul, 180,  250, 0xFFFF,     // DCB
//	7, 4,  20,  2000ul, 0,    250, 0xFFFF,     // REB
//	8, 6,  20,  2000ul, 0,    250, 0xFFFF,     // PAW
//	9, 7,  20,  1500ul, 1200, 250, 0xFFFF,     // PRIMER
//	10,4,  20,  2000ul, 0,    250, 0xFFFF,     // REB
//};

const TIMING_STEP CleanTiming[CLEAN_TIMING_STEP] = 
{
	0, "WATER",   1,  {1, 0,  1, 0,  1, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	1, "WATER",   2,  {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	2, "WATER",   3,  {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	3, "WATER",   4,  {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {2, 0,  2, 0,  2, 0}, 0,  250, 0xFFFF,
	4, "WATER",   5,  {1, 0,  1, 0,  1, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	5, "WATER",   6,  {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	6, "WATER",   7,  {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	7, "WATER",   23, {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {2, 0,  2, 0,  2, 0}, 0,  250, 0xFFFF,
	8, "AIR",     1,  {0, 0,  0, 0,  0, 0}, 0,  0,   0,      0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
};

const TIMING_STEP PrimeTiming[PRIME_TIMING_STEP] = 
{
	0, "WATER",   1,  {1, 0,  1, 0,  1, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	1, "WATER",   2,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	2, "WATER",   3,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	3, "WATER",   4,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	4, "WATER",   5,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	5, "WATER",   6,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	6, "WATER",   7,  {0, 0,  0, 0,  0, 0}, 0,  20,  200ul,  0, {2, 0,  2, 0,  2, 0}, 0,  250, 0xFFFF,
    7, "WATER",   8,  {1, 0,  1, 0,  1, 0}, 0,  40,  1000ul, 0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
	8, "WATER",   23, {0, 0,  0, 0,  0, 0}, 0,  40,  1000ul, 0, {2, 0,  2, 0,  2, 0}, 0,  250, 0xFFFF,
	9, "AIR",     1,  {0, 0,  0, 0,  0, 0}, 0,  0,   0,      0, {0, 0,  0, 0,  0, 0}, 0,  0,   0,
};

const TIMING_STEP LoadTiming[LOAD_TIMING_STEP] = 
{
	0,  "DNB",    1,  {0, 0,  0, 0,  0, 0},  0,  20,  1100ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  0,   0,          // DNB
	1,  "DRB",    2,  {0, 0,  0, 0,  0, 0},  0,  20,  200ul,  3600, {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // DRB
	2,  "DRB",    2,  {1, 50, 1, 50, 1, 50}, 0,  20,  2000ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // DRB
	3,  "DCB",    3,  {1, 50, 1, 50, 1, 50}, 0,  20,  2000ul, 180,  {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // DCB
	4,  "REB",    4,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // REB
	5,  "PWB",    5,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 480,  {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // PWB
	6,  "DCB",    3,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 180,  {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // DCB
	7,  "REB",    4,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // REB
	8,  "PAW",    6,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // PAW
	9,  "PRIMER", 7,  {0, 0,  0, 0,  0, 0},  0,  20,  1500ul, 1200, {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // PRIMER
	10, "REB",    4,  {0, 0,  0, 0,  0, 0},  0,  20,  2000ul, 0,    {0, 0,  0, 0,  0, 0}, 0,  250, 0xFFFF,     // REB
};

/* Private function prototypes -----------------------------------------------*/
static void EndianSwap16(BYTE *Buff);
static void AckUpLayerComm( u16 Index, u8 Order, u8 stat );
//static void ResultUpLayerComm( u16 Index, u8 Order, u8 dat[], u16 len );
static u8   UpLayerMsgParsing( P_FRM_PROC pstUpLayerFrm, u16 DatLen );

/* Private functions ---------------------------------------------------------*/ 
/*******************************************************************************
 * @brief ʱ��ת����ָ�� 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 TimingToCommand(u8 *pDatBuf, u8 TimingType)
{
	P_TIMING_STEP psTimingStep;
	u16  DatLen = 0;
	u8   MaxStepNum;
	u8   idx;
	u32  tmp32;
		
	DatLen = 0;
	MaxStepNum = 0;
	psTimingStep = NULL;
	
	switch( TimingType )
	{
		case TIMING_TYPE_CLEAN:
		{
			psTimingStep = (P_TIMING_STEP)CleanTiming;
		
			MaxStepNum = CLEAN_TIMING_STEP;
			break;
		}
		case TIMING_TYPE_PRIME:
		{
			psTimingStep = (P_TIMING_STEP)PrimeTiming;
		
			MaxStepNum = PRIME_TIMING_STEP;
			break;
		}
		case TIMING_TYPE_COVER:
		{
			psTimingStep = (P_TIMING_STEP)NULL;		
			MaxStepNum = 0;
		
			// ѹ�����ת5ת
			*pDatBuf++ = COVER_MTR_POSITION;
			*pDatBuf++ = 50;
			*pDatBuf++ = MTR_POS_TIDE;
			DatLen += 3;	

			tmp32 = 1000;
			*pDatBuf++ = SYSTEM_DELAY_MS;
			*pDatBuf++ = tmp32 >> 0;
			*pDatBuf++ = tmp32 >> 8;
			*pDatBuf++ = tmp32 >> 16;
			*pDatBuf++ = tmp32 >> 24;
			
			DatLen += 5;				
			
			return DatLen;
		}
		case TIMING_TYPE_LOADING:
		{
			psTimingStep = (P_TIMING_STEP)LoadTiming;
		
			MaxStepNum = LOAD_TIMING_STEP;
			
			// init syringe after covering
			*pDatBuf++ = SYR_PUMP_INIT;
			*pDatBuf++ = psTimingStep->SyrAsprSpeed;
				
			DatLen += 2;
			
			break;
		}
		default:
		{
			break;
		}
	}
	
	if( (NULL == psTimingStep) || (0 == MaxStepNum) || (NULL == pDatBuf) )
		return 0;
	
//	// turn on valves
//	if( (TIMING_TYPE_PRIME == TimingType) || (TIMING_TYPE_CLEAN == TimingType) )
//	{
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_1;
//		*pDatBuf++ = 0x01;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		
//		DatLen += 5;
//		
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_2;
//		*pDatBuf++ = 0x01;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		
//		DatLen += 5;
//		
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_3;
//		*pDatBuf++ = 0x01;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//			
//		DatLen += 5;			
//	}
	
	for( idx=0; idx<MaxStepNum; idx++ )
	{		
		// rotate valve control
		if( (psTimingStep->RotValvePos > 0) && (psTimingStep->RotValvePos <= RV_MAX_PORT) )
		{
			*pDatBuf++ = RO_VALVE_CTRL;
			*pDatBuf++ = psTimingStep->RotValvePos;
			
			DatLen += 2;			
		}
		
		// tee valve 1 control
		if( psTimingStep->stTeeValAspr[0].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValAspr[0].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_1;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValAspr[0].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValAspr[0].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_1;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// tee valve 2 control
		if( psTimingStep->stTeeValAspr[1].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValAspr[1].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_2;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValAspr[1].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValAspr[1].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_2;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// tee valve 3 control
		if( psTimingStep->stTeeValAspr[2].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValAspr[2].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_3;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValAspr[2].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValAspr[2].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_3;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// Delay for valve (ms)
		if( psTimingStep->DelayForValve1 > 0) 
		{
			tmp32 =  psTimingStep->DelayForValve1 * 100;
			
			*pDatBuf++ = SYSTEM_DELAY_MS;
			*pDatBuf++ = tmp32 >> 0;
			*pDatBuf++ = tmp32 >> 8;
			*pDatBuf++ = tmp32 >> 16;
			*pDatBuf++ = tmp32 >> 24;
			
			DatLen += 5;			
		}
		
		// pump in control
		if( (psTimingStep->SyrAsprSpeed > 0) && (psTimingStep->SyrAsprVolume > 0) )
		{
			*pDatBuf++ = SYR_PUMP_IN;
			*pDatBuf++ = psTimingStep->SyrAsprSpeed;
			*pDatBuf++ = psTimingStep->SyrAsprVolume >> 0;
			*pDatBuf++ = psTimingStep->SyrAsprVolume >> 8;
			
			DatLen += 4;			
		}
		
		// turn on valve 1 after load dnb
		if( (TIMING_TYPE_LOADING == TimingType) && (1 == idx) )
		{
			*pDatBuf++ = VALVE_CTRL;
			*pDatBuf++ = VALVE_NO_1;
			*pDatBuf++ = 0x01;
			*pDatBuf++ = 0;
			*pDatBuf++ = 0;
			
			DatLen += 5;
		}
		
		// Delay ms
		if( psTimingStep->DelaySecond > 0) 
		{
			tmp32 =  psTimingStep->DelaySecond * 1000;
			
			*pDatBuf++ = SYSTEM_DELAY_MS;
			*pDatBuf++ = tmp32 >> 0;
			*pDatBuf++ = tmp32 >> 8;
			*pDatBuf++ = tmp32 >> 16;
			*pDatBuf++ = tmp32 >> 24;
			
			DatLen += 5;			
		}
		
		// turn on valve 1 after load dnb
		if( (TIMING_TYPE_LOADING == TimingType) && (1 == idx) )
		{
			*pDatBuf++ = VALVE_CTRL;
			*pDatBuf++ = VALVE_NO_1;
			*pDatBuf++ = 0x00;
			*pDatBuf++ = 0;
			*pDatBuf++ = 0;
			
			DatLen += 5;
		}
		
		// tee valve 1 control
		if( psTimingStep->stTeeValDrain[0].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValDrain[0].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_1;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValDrain[0].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValDrain[0].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_1;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// tee valve 2 control
		if( psTimingStep->stTeeValDrain[1].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValDrain[1].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_2;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValDrain[1].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValDrain[1].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_2;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// tee valve 3 control
		if( psTimingStep->stTeeValDrain[2].stat > 0 )
		{
			if( 1 == psTimingStep->stTeeValDrain[2].stat )
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_3;
				*pDatBuf++ = 0x01;
				*pDatBuf++ = psTimingStep->stTeeValDrain[2].T100ms;
				*pDatBuf++ = psTimingStep->stTeeValDrain[2].T100ms >> 8;
			}
			else
			{
				*pDatBuf++ = VALVE_CTRL;
				*pDatBuf++ = VALVE_NO_3;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
				*pDatBuf++ = 0x00;
			}
			
			DatLen += 5;
		}
		
		// Delay for valve (ms)
		if( psTimingStep->DelayForValve2 > 0) 
		{
			tmp32 =  psTimingStep->DelayForValve2 * 100;
			
			*pDatBuf++ = SYSTEM_DELAY_MS;
			*pDatBuf++ = tmp32 >> 0;
			*pDatBuf++ = tmp32 >> 8;
			*pDatBuf++ = tmp32 >> 16;
			*pDatBuf++ = tmp32 >> 24;
			
			DatLen += 5;			
		}
		
		// pump out control
		if( (psTimingStep->SyrDrainSpeed > 0) && (psTimingStep->SyrDrainVolume > 0) )
		{
			if( psTimingStep->SyrDrainVolume > 2500 )
			{
				*pDatBuf++ = SYR_PUMP_INIT;
				*pDatBuf++ = psTimingStep->SyrDrainSpeed;
				
				DatLen += 2;
			}
			else
			{				
				*pDatBuf++ = SYR_PUMP_OUT;
				*pDatBuf++ = psTimingStep->SyrDrainSpeed;
				*pDatBuf++ = psTimingStep->SyrDrainVolume >> 0;
				*pDatBuf++ = psTimingStep->SyrDrainVolume >> 8;
				
				DatLen += 4;		
			}			
		}
		
		psTimingStep++;
	}
	
//	// turn off valves
//	if( (TIMING_TYPE_PRIME == TimingType) || (TIMING_TYPE_CLEAN == TimingType) )
//	{
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_1;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		
//		DatLen += 5;
//		
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_2;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		
//		DatLen += 5;
//		
//		*pDatBuf++ = VALVE_CTRL;
//		*pDatBuf++ = VALVE_NO_3;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//		*pDatBuf++ = 0x00;
//			
//		DatLen += 5;	
//	}
	
	return DatLen;
}

/*******************************************************************************
 * @brief  �ַ�ת����
 * @input  Buff, ���ݻ��棬2���ֽڣ�16λ�����ݣ�
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 Ascii2Hex(u8 tmp)
{
	if( (tmp >= '0') && (tmp <= '9') )
		return (tmp - '0');
	else if( (tmp >= 'A') && (tmp <= 'F') )
		return (tmp - 'A');
	else if( (tmp >= 'a') && (tmp <= 'f') )
		return (tmp - 'a');
	else
		return 0;
}

/*******************************************************************************
 * @brief  16λ������, ��8λ���8λ�Ե� 
 * @input  Buff, ���ݻ��棬2���ֽڣ�16λ�����ݣ�
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void EndianSwap16(BYTE *Buff)
{
	BYTE tmp;
	
	tmp = Buff[0];
	Buff[0] = Buff[1];
	Buff[1] = tmp;
}

/*******************************************************************************
 * @brief  У�����㷨 
 * @input  Buff, ���ݻ���
 *       len, ���ݳ���
 * @return 16λ��У����
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 CRC_GetValue(BYTE *Buff, u16 Len)
{
// CRC-16 (Modbus, init=0xFFFF; IBM, init=0x0000 ) 
	const u16 CRC_QUANTIC = 0xA001;
	
	u16 crc = 0x0000; //0xFFFF;
    u16 i   = 0;
    u8  j   = 0;

	for(i=0; i<Len;i++)
    {
		crc = crc^(*Buff);
		Buff++ ;

		for(j=0; j<8; j++)
		{
			if((crc&0x0001)== 1)
			{
				crc = crc>>1;
				crc = crc ^ CRC_QUANTIC;
			}
			else
			{
				crc = crc >> 1;
			}
		}           
	}  

    return (crc);	
}

/*******************************************************************************
 * @brief ���õ�ǰϵͳ״̬ 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void SetSysCurrOperation(u8 stat)
{
	cSysCurrOper = stat;
}

/*******************************************************************************
 * @brief ��ȡ��ǰϵͳ״̬ 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 GetSysCurrOperation(void)
{
	return cSysCurrOper;
}


/*******************************************************************************
 * @brief   ��ȡ��λ��ָ�� 
 * @input  
 * @return 0-����, �����쳣 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/

u8 HEXPR_FrameDecode( u8 *pRecvBuff, u16 DatLen )
{
	u16  idx;
	u16  len;
	u16  crc;
	u8  *pDatBuf;	
	P_FRM_PROC pstRecvFrm;
	
	if( DatLen<4 )
		return FRM_RECV_LEN_ERROR;
	
	pDatBuf = pRecvBuff;
	
	idx = 1;
	while( idx<DatLen )
	{
		// �ж�֡ͷ	
		if( FRM_HEADER==(((INT16U)pDatBuf[idx-1]<<8)+pDatBuf[idx]) )					
		{
			break;
		}
					
		idx += 1;
	}
				
	// ��ǰ������
	if( idx>1 )
	{
		if( (DatLen-idx)>8 )
		{
			memcpy( pDatBuf, pDatBuf+idx-1, DatLen-idx+1 );
			DatLen -= (idx-1);
		}
		else
		{	
			return FRM_RECV_LEN_ERROR;
		}
	}
	
	pstRecvFrm = (P_FRM_PROC)pDatBuf;

	len = ((u16)pDatBuf[3]<<8) + pDatBuf[2];	
	
	// �ж����ݳ���
	if( len != (DatLen-6) )
	{
		AckUpLayerComm( pstRecvFrm->Header.Struct.Index, pstRecvFrm->Header.Struct.Order, FRM_ACK_ERR_LEN );			
		return FRM_RECV_LEN_ERROR;
	}
	
	// �ж�У����
	crc = CRC_GetValue((u8 *)&pDatBuf[4], len);
	
	if( crc != (((u16)pDatBuf[DatLen-1]<<8) + pDatBuf[DatLen-2]) ) 
	{
		AckUpLayerComm( pstRecvFrm->Header.Struct.Index, pstRecvFrm->Header.Struct.Order, FRM_ACK_ERR_CRC );			
		return FRM_ACK_ERR_CRC;
	}
	
	// �жϵ�ַ
	if( !IsSendToMe(pstRecvFrm->Header.Struct.Addr) )
	{
		AckUpLayerComm( pstRecvFrm->Header.Struct.Index, pstRecvFrm->Header.Struct.Order, FRM_ACK_ERR_ADDR );			
		return FRM_ACK_ERR_ADDR;
	}
	
	// �ж��ظ�
//	if( )
//	{
//		AckUpLayerComm( FRM_ACK_REPEAT );			
//		return FRM_ACK_REPEAT;
//	}
	
	AckUpLayerComm( pstRecvFrm->Header.Struct.Index, pstRecvFrm->Header.Struct.Order, FRM_ACK_OK );	
	
	return UpLayerMsgParsing( pstRecvFrm, DatLen );
}

/*******************************************************************************
 * @brief  Ӧ����λ��ָ�� 
 * @input  stat,  ״̬λ����
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void AckUpLayerComm( u16 Index, u8 Order, u8 stat )
{
	BYTE   buff[20];
	INT16U crc;
	P_FRM_PROC pSendBuf = (P_FRM_PROC)buff;
	
	pSendBuf->Header.Struct.Header = FRM_HEADER;
	pSendBuf->Header.Struct.Length = 5+1;                                  // ���ݳ���,  
	pSendBuf->Header.Struct.Index  = Index;
	pSendBuf->Header.Struct.Type   = FRM_TYPE_ACK;                         // ����, 0x01-��ҪӦ��ͽ��֡,0x04-��ҪӦ��;0x02-Ӧ��֡,0x03-���֡
	pSendBuf->Header.Struct.Addr   = TARGET_ADDR;                          // Ŀ���ַ
	pSendBuf->Header.Struct.Order  = Order;   // ����

	// Endian little to big
	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Header );
//	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Index );
//	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Length );
	
	pSendBuf->DatBuf[0] = stat;	
	
	// ccrc
	crc = CRC_GetValue(buff+4, pSendBuf->Header.Struct.Length); 
	pSendBuf->DatBuf[1] = crc>>8;
	pSendBuf->DatBuf[2] = crc;

	BSP_Ser_WrHex (UART_PC, (CPU_CHAR *)buff, pSendBuf->Header.Struct.Length+6);
}

/*******************************************************************************
 * @brief  ���ؽ��ָ�� 
 * @input  Index,     ֡���
 *         MainOrder, ������
 *         SubOrder,  ������
 *         dat,       ��������
 *         len,       ���ݳ���
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void ResultUpLayerComm( u16 Index, u8 Order, u8 dat[], u16 len )
{
	BYTE   buff[100];
	INT16U crc;
	P_FRM_PROC pSendBuf = (P_FRM_PROC)buff;
	
	pSendBuf->Header.Struct.Header = FRM_HEADER;
	pSendBuf->Header.Struct.Length = 5+len;                               // ���ݳ���, DatBuf size  
	pSendBuf->Header.Struct.Type   = FRM_TYPE_RES;                        // ����, 0x01-��ҪӦ��ͽ��֡,0x04-��ҪӦ��;0x02-Ӧ��֡,0x03-���֡
	pSendBuf->Header.Struct.Index  = Index;
	pSendBuf->Header.Struct.Addr   = TARGET_ADDR;                         // Ŀ���ַ
	pSendBuf->Header.Struct.Order  = Order;                               // ����

	// Endian little to big
	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Header );
//	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Index );
//	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Length );
	
	memcpy(pSendBuf->DatBuf, dat, len);
	
	// ccrc
	crc = CRC_GetValue(buff+4, 5+len); 
	pSendBuf->DatBuf[len]   = crc>>8;
	pSendBuf->DatBuf[len+1] = crc;
	
	BSP_Ser_WrHex(UART_PC, (CPU_CHAR *)buff, 5+len+6);
}

/*******************************************************************************
 * @brief  ���ָ���������䴫��
 * @input  
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 OrderMakeToFrame( const u8 *pSrcBuf, u16 SrcLen, u8 *pDstBuf )
{
	u16 crc;
	P_FRM_PROC pSendBuf = (P_FRM_PROC)pDstBuf;
	
	if( (NULL==pSrcBuf)||(NULL==pDstBuf)||(SrcLen<1) )
		return 0;
	
	pSendBuf->Header.Struct.Header = FRM_HEADER;
	pSendBuf->Header.Struct.Length = 4+SrcLen;                             // ���ݳ���,  
	pSendBuf->Header.Struct.Index  = 0x0;
	pSendBuf->Header.Struct.Type   = FRM_TYPE_2;                           // ����, 0x01-��ҪӦ��ͽ��֡,0x04-��ҪӦ��;0x02-Ӧ��֡,0x03-���֡
	pSendBuf->Header.Struct.Addr   = LOCAL_ADDR;                           // Ŀ���ַ
	pSendBuf->Header.Struct.Order  = pSrcBuf[0];                           // ����

	memcpy( pSendBuf->DatBuf, pSrcBuf+1, SrcLen-1 );
	
	// Endian little to big
	EndianSwap16( (BYTE *)&pSendBuf->Header.Struct.Header );	
	
	// ccrc
	crc = CRC_GetValue(pDstBuf+4, 4+SrcLen); 
	
	pSendBuf->DatBuf[SrcLen-1] = crc>>8;
	pSendBuf->DatBuf[SrcLen-0] = crc;
	
	return (4 + 4 + SrcLen + 2);
}

/*******************************************************************************
 * @brief  ��λ��ָ�� 
 * @input  
 * @return 0-����, �����쳣 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static u8 UpLayerMsgParsing( P_FRM_PROC pstUpLayerFrm, u16 DatLen )
{	
	OS_ERR err;
	u8  aSndBuff[30];
	u8 *pSndBuff;
	u8  SndBuffIdx;
	u8  tmp8;
	u16 CommPackLen;
	u16 ValCtrlTime[VALVE_NO_M];
	
	if( NULL==pstUpLayerFrm )
		return FALSE;
	
	pSndBuff = aSndBuff;	
		
	SndBuffIdx = 0;
	switch( pstUpLayerFrm->Header.Struct.Order )
	{
	case QUERY_SOFT_VERSION:
		*pSndBuff++ = FRM_EXEC_SUCCESS;
		SndBuffIdx = 1;
		
		sprintf((char *)pSndBuff, "%s%s", SYSTEM_VERSION, PROTOCOL_VERSION);
		SndBuffIdx += sizeof(SYSTEM_VERSION);
		SndBuffIdx += sizeof(PROTOCOL_VERSION);		      
		break;
	case QUERY_HARD_VERSION:
		*pSndBuff++ = FRM_EXEC_SUCCESS;
		SndBuffIdx = 1;

		sprintf((char *)pSndBuff, "%s", PCBA_VERSION);
		SndBuffIdx += sizeof(PCBA_VERSION);
		break;
	case UPDATE_SYSTEM:
		break;
	case SET_PARA:               //���ò���
		break;
	case QUERY_PARA:             //��ѯ����
		break;	
	
	case RO_VALVE_PASS:          //��ת��͸��
	case RO_VALVE_CTRL:          //��ת��ָ��	
	{	
		// ��ִ�й�ע,��ϴ,���Թ����в��ܵ�������
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{	
			if( MODULE_STAT_EXECUTING == RV_GetCurrStat() )
			{
				*pSndBuff++ = FRM_EXEC_MODULE_RUNNING;
				SndBuffIdx = 1;
			}
			else
			{
				stMsgHandle[TASK_MSG_RV].Type  = MSG_TYPE_PROTOCOL;
				stMsgHandle[TASK_MSG_RV].DatLen= DatLen;
				memcpy(stMsgHandle[TASK_MSG_RV].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_RV].DatLen); 
				OSQPost((OS_Q *)&MsgQueue[TASK_MSG_RV], &stMsgHandle[TASK_MSG_RV], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
			}
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case RO_VALVE_QUERY:	     //��ѯ��ת��״̬
	{	
		stMsgHandle[TASK_MSG_RV].Type  = MSG_TYPE_PROTOCOL;
		stMsgHandle[TASK_MSG_RV].DatLen= DatLen;
		memcpy(stMsgHandle[TASK_MSG_RV].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_RV].DatLen); 
		OSQPost((OS_Q *)&MsgQueue[TASK_MSG_RV], &stMsgHandle[TASK_MSG_RV], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);			
		
		break;
	}
	case SYR_PUMP_PASS:          //ע���͸��		
	case SYR_PUMP_INIT:          //ע��ø�λ
	case SYR_PUMP_IN:            //ע�����Һ	
	case SYR_PUMP_OUT:           //ע�����Һ
	{	
		// ��ִ�й�ע,��ϴ,���Թ����в��ܵ�������
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{	
			if( MODULE_STAT_EXECUTING == XLP_GetCurrStat() )
			{
				*pSndBuff++ = FRM_EXEC_MODULE_RUNNING;
				SndBuffIdx = 1;
			}
			else
			{					
				stMsgHandle[TASK_MSG_SYR].Type  = MSG_TYPE_PROTOCOL;
				stMsgHandle[TASK_MSG_SYR].DatLen= DatLen;
				memcpy(stMsgHandle[TASK_MSG_SYR].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_SYR].DatLen); 
				OSQPost((OS_Q *)&MsgQueue[TASK_MSG_SYR], &stMsgHandle[TASK_MSG_SYR], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
			}
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case COVER_MTR_INIT:         //�Ӹǵ����λ,    ����ٶ�
	case COVER_MTR_ABS_DIST:     //���ǵ�������˶�,����ٶ�+����
	case COVER_MTR_REL_DIST:     //���ǵ�������˶�,����ٶ�+ָ������+����
	case COVER_MTR_POSITION:     //���ǵ��ָ��λ��,����ٶ�+λ�ñ��
	{	
		// ��ִ�й�ע,��ϴ,���Թ����в��ܵ�������
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{	
			if( MODULE_STAT_EXECUTING==MOTOR_GetCurrStat(MOTOR_1) )
			{
				*pSndBuff++ = FRM_EXEC_MODULE_RUNNING;
				SndBuffIdx = 1;
			}
			else
			{
				stMsgHandle[TASK_MSG_MTR1].Type  = MSG_TYPE_PROTOCOL;
				stMsgHandle[TASK_MSG_MTR1].DatLen= DatLen;
				memcpy(stMsgHandle[TASK_MSG_MTR1].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_MTR1].DatLen); 
				OSQPost((OS_Q *)&MsgQueue[TASK_MSG_MTR1], &stMsgHandle[TASK_MSG_MTR1], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
			}
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case TEMP_CTRL_ON:           //�¶ȿ��ƿ�, Ŀ���¶�(x100)+У׼�¶�(x100)	
		break;
	case TEMP_CTRL_OFF:          //�¶ȿ��ƹ�
		break;
	case TEMP_CTRL_QUERY:        //�¶Ȳ�ѯ
		break;
	case PRESS_QUERY:            //ѹ����ѯ
		break;
	
	case PUMP_CTRL:              //�ÿ��ƿ�,   �ñ��
		break;
	case PUMP_STAT_QUERY:        //��״̬��ѯ, �ñ��
		break;
	
	case VALVE_CTRL:             //�����ƿ�,   �����
	{
		if( DatLen < (sizeof(HEADER) + 4 + 2) )
		{
			*pSndBuff++ = FRM_RECV_PARA_LEN_ERR;
			SndBuffIdx = 1;
		}
		else 
		{
			CommPackLen = DatLen - sizeof(HEADER) - 2;
			if( 0 != (CommPackLen % 4) )
			{
				*pSndBuff++ = FRM_RECV_PARA_CON_ERR;
				SndBuffIdx  = 1;
			}
			else
			{	
				// ����ʵ������
				memset((u8 *)ValCtrlTime, 0x00, VALVE_NO_M<<1);
				
				tmp8 = 0;
				while( 1 )
				{					
					if( TRUE == ValveCtrlFromProl(&pstUpLayerFrm->DatBuf[tmp8], 4, &ValCtrlTime[*((u8 *)pstUpLayerFrm->DatBuf + tmp8) - 1]) )
					{	
						CommPackLen -= 4;
						
						if( CommPackLen<4 )
						{
							*pSndBuff++ = FRM_EXEC_SUCCESS;
							SndBuffIdx = 1;
							break;
						}
						else
						{
							tmp8 += 4;
						}
					}
					else
					{
						*pSndBuff++ = FRM_RECV_PARA_CON_ERR;
						SndBuffIdx  = 1;
						
						break;
					}
				}
			}
		}
		break;
	}
	case VALVE_STAT_QUERY:       //��״̬��ѯ, �����
		*pSndBuff++ = FRM_EXEC_SUCCESS;
		SndBuffIdx = 1;
	
		*pSndBuff++ = IsValve1On()?1:0;
		*pSndBuff++ = IsValve2On()?1:0;
		*pSndBuff++ = IsValve3On()?1:0;
		SndBuffIdx += 3;
	
		break;
	  
	case QUERY_DIODE_STAT:       //��ѯ��ź��ƽ, ��ź���
		*pSndBuff++ = FRM_EXEC_SUCCESS;
		SndBuffIdx = 1;
	
		*pSndBuff++ = IsMotorHomeValid(MOTOR_1)?1:0;
		*pSndBuff++ = IsMotorLimitValid(MOTOR_1)?1:0;
		SndBuffIdx += 2;

#if defined(MOTOR_DEF_2)	
		*pSndBuff++ = IsMotorHomeValid(MOTOR_2)?1:0;
		*pSndBuff++ = IsMotorLimitValid(MOTOR_2)?1:0;
		SndBuffIdx += 2;
#endif
		break;		  

	case FLOW_PRIME_DOWNLOAD:    //���ع�עʱ���
		break;
	case FLOW_PRIME_QUERY:       //��ѯ��עʱ���
		break;
	case FLOW_PRIME_START:       //��ʼ��ע����
	{
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{
			stMsgHandle[TASK_MSG_TIMING].Type  = MSG_TYPE_PROTOCOL;
			stMsgHandle[TASK_MSG_TIMING].DatLen= DatLen;
			memcpy(stMsgHandle[TASK_MSG_TIMING].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_TIMING].DatLen); 
			OSQPost((OS_Q *)&MsgQueue[TASK_MSG_TIMING], &stMsgHandle[TASK_MSG_TIMING], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);			
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case FLOW_PRIME_PAUSE:       //��ͣ��ע����
		break;
	case FLOW_PRIME_STOP:        //ֹͣ��ע����
		break;
	case FLOW_PRIME_STAT_QUERY:  //��ѯ��ע״̬
		break;
	
	case FLOW_CLEAN_DOWNLOAD:    //���ع�עʱ���
		break;
	case FLOW_CLEAN_QUERY:       //��ѯ��עʱ���
		break;
	case FLOW_CLEAN_START:       //��ʼ��ע����
	{
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{
			stMsgHandle[TASK_MSG_TIMING].Type  = MSG_TYPE_PROTOCOL;
			stMsgHandle[TASK_MSG_TIMING].DatLen= DatLen;
			memcpy(stMsgHandle[TASK_MSG_TIMING].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_TIMING].DatLen); 
			OSQPost((OS_Q *)&MsgQueue[TASK_MSG_TIMING], &stMsgHandle[TASK_MSG_TIMING], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);			
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case FLOW_CLEAN_PAUSE:       //��ͣ��ע����
		break;
	case FLOW_CLEAN_STOP:        //ֹͣ��ע����
		break;
	case FLOW_CLEAN_STAT_QUERY:  //��ѯ��ע״̬
		break;
	

	case FLOW_COVER_DOWNLOAD:    //���������Լ��ʱ���
		break;
	case FLOW_COVER_QUERY:       //��ѯ�����Լ��ʱ���
		break;
	case FLOW_COVER_START:       //��ʼ�����Լ������
	{
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{
			stMsgHandle[TASK_MSG_TIMING].Type  = MSG_TYPE_PROTOCOL;
			stMsgHandle[TASK_MSG_TIMING].DatLen= DatLen;
			memcpy(stMsgHandle[TASK_MSG_TIMING].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_TIMING].DatLen); 
			OSQPost((OS_Q *)&MsgQueue[TASK_MSG_TIMING], &stMsgHandle[TASK_MSG_TIMING], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);			
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case FLOW_COVER_PAUSE:       //��ͣ�����Լ������
		break;
	case FLOW_COVER_STOP:        //ֹͣ�����Լ������
		break;
	case FLOW_COVER_STAT_QUERY:  //��ѯ�����Լ��״̬
		break;		
	
	
	case FLOW_LOAD_DOWNLOAD:     //����Loadingʱ���
		break;
	case FLOW_LOAD_QUERY:        //��ѯLoadingʱ���
		break;
	case FLOW_LOAD_START:        //��ʼLoading����
	{
		if( SYS_CURR_OP_IDLE==(tmp8 = GetSysCurrOperation()) )
		{
			stMsgHandle[TASK_MSG_TIMING].Type  = MSG_TYPE_PROTOCOL;
			stMsgHandle[TASK_MSG_TIMING].DatLen= DatLen;
			memcpy(stMsgHandle[TASK_MSG_TIMING].DatBuf, (u8 *)pstUpLayerFrm, stMsgHandle[TASK_MSG_TIMING].DatLen); 
			OSQPost((OS_Q *)&MsgQueue[TASK_MSG_TIMING], &stMsgHandle[TASK_MSG_TIMING], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);			
		}
		else
		{
			*pSndBuff++ = FRM_EXEC_FORBID_PRIMING + tmp8;
			SndBuffIdx = 1;
		}
		
		break;
	}
	case FLOW_LOAD_PAUSE:       //��ͣLoading����
		break;
	case FLOW_LOAD_STOP:        //ֹͣLoading����
		break;
	case FLOW_LOAD_STAT_QUERY:  //��ѯLoading״̬
		break;	
	
	default:
		*pSndBuff++ = FRM_RECV_NON_ORDER;
		SndBuffIdx = 1;
		
		break;
	}	

	// ��Ҫ�ظ����֡
	if( FRM_TYPE_1==pstUpLayerFrm->Header.Struct.Type )
	{		
		if( SndBuffIdx>0 )
		{
			if( (FRM_RECV_PARA_LEN_ERR == aSndBuff[0])||(FRM_RECV_PARA_CON_ERR == aSndBuff[0]) )
			{
				memcpy(pSndBuff, (u8 *)pstUpLayerFrm, DatLen);
				SndBuffIdx += DatLen;
			}
			
			ResultUpLayerComm(pstUpLayerFrm->Header.Struct.Index, 
		                      pstUpLayerFrm->Header.Struct.Order,		
		                      aSndBuff, 
		                      SndBuffIdx);
		}
	}

	return TRUE;
}

/*******************************************************************************
 * @brief  ���������������
 * @input  
 * @return 0-����, �����쳣 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void HEX_MotorCtrlTask( u8 *pRecvBuff )
{
	OS_ERR     err;
	u8    DatBuf[64];
	u8    MsgType;
	u8    DatLen;
	u8    bTastStat;
	u8    bRunStat;
	u8    HomeAgain;
	u32   TimeOut;
	P_FRM_PROC  pstFrameBuf;
	
	MsgType= pRecvBuff[0];	
	DatLen = pRecvBuff[1];
	
	pstFrameBuf = (P_FRM_PROC)&pRecvBuff[2];	
	
	// reset error number
	MOTOR_SetCurrError(MOTOR_1, SYS_ERR_NONE);
	
	TimeOut = MAX_MTR_RUN_TIME_MS;	 
	
	HomeAgain = FALSE;
			
	bTastStat = DEF_TRUE;
	switch( pstFrameBuf->Header.Struct.Order )
	{	
		case COVER_MTR_INIT: //
		{		
			// ���ݳ����ж�
			if( (DatLen<(4+8)) || (pstFrameBuf->DatBuf[0]>MOTOR_SPEED_MAX) )			
			{
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_PARA);
				
				bTastStat = DEF_FALSE;
			}			
				
			if( DEF_TRUE==bTastStat )
			{
				if( IsMotorHomeValid(MOTOR_1) )
				{
					// �������ź
					bRunStat = MOTOR_MoveRelative(MOTOR_1, ((u32)M1_PULSE_ORIGIN<<2), MOTOR_DIR_GO_MAX, 30);
					
					HomeAgain = TRUE;
				}
				else
				{
					bRunStat = MOTOR_Home(MOTOR_1, pstFrameBuf->DatBuf[0]);  
				}
			}			
			break;
		}
		case COVER_MTR_ABS_DIST:					
		{	
			// ���ݳ����ж�
			if( (DatLen<(4+10)) || (pstFrameBuf->DatBuf[0]>MOTOR_SPEED_MAX) )			
			{
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_PARA);
				
				bTastStat = DEF_FALSE;			
			}		
			
			if( DEF_TRUE==bTastStat )
				bRunStat = MOTOR_MoveAbsolute(MOTOR_1, 
			                               ((u32)pstFrameBuf->DatBuf[2]<<8) + pstFrameBuf->DatBuf[1], 
			                               pstFrameBuf->DatBuf[0]);		
			break;
		}
		case COVER_MTR_REL_DIST:
		{
			// ���ݳ����ж�
			if( (DatLen<(4+11)) || (pstFrameBuf->DatBuf[3]>MOTOR_DIR_GO_MAX) ||(pstFrameBuf->DatBuf[0]>MOTOR_SPEED_MAX) )	
			{
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_PARA);	
				
				bTastStat = DEF_FALSE;
			}			
				
			if( DEF_TRUE==bTastStat )
				bRunStat = MOTOR_MoveRelative(MOTOR_1, 
			                               ((u32)pstFrameBuf->DatBuf[2]<<8) + pstFrameBuf->DatBuf[1], 
			                               pstFrameBuf->DatBuf[3],                                     
			                               pstFrameBuf->DatBuf[0]);			
			break;
		}
		case COVER_MTR_POSITION:
		{
			// ���ݳ����ж�
			if( (DatLen<(2+11)) || (pstFrameBuf->DatBuf[0]>MOTOR_SPEED_MAX) )	
			{
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_PARA);	
				
				bTastStat = DEF_FALSE;
			}
			
			if( MTR_POS_ZERO==pstFrameBuf->DatBuf[1] )
			{				
				bRunStat = MOTOR_MoveAbsolute(MOTOR_1, stMtrPosPara[MOTOR_1].PosZero, pstFrameBuf->DatBuf[0]);
			}
			else if( MTR_POS_TIDE==pstFrameBuf->DatBuf[1] )
			{
				bRunStat = MOTOR_MoveAbsolute(MOTOR_1, stMtrPosPara[MOTOR_1].PosTide, pstFrameBuf->DatBuf[0]);
			}
			else
			{
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_PARA);
				
				bTastStat = DEF_FALSE;
			}
			
			break;
		}	
		default:
		{
			MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_COMM);
			
			bTastStat = DEF_FALSE; 
			break;
		}
	}
	
	if( DEF_TRUE != bTastStat )
	{
		// �����쳣״̬
		MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_NORMAL);
		
		// ���ش����� + ָ��֡
		DatBuf[0] = 0x01;
		DatBuf[1] = MOTOR_GetCurrError(MOTOR_1);
		DatBuf[2] = MOTOR_GetCurrError(MOTOR_1)>>8;
		memcpy( DatBuf+3, pRecvBuff+2, DatLen );
		
		if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
			ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, DatBuf, 3 + DatLen );	
		else 
			// ����ִ�е��ź�
			OSSemPost(&semExecMtrStat[MOTOR_1], OS_OPT_POST_1, &err);		
		
		APP_TRACE_INFO(("parameter error for motor 1 msg\n\r"));	

		return ;
	}	
	
	// �ȴ�ִ�����
	DatLen = 0;
	while( DEF_TRUE==bTastStat )
	{
		OSSemPend(&semMtr1Ctrl, TimeOut, OS_OPT_PEND_BLOCKING, NULL, &err); 
			
		// ��ʽ��ԭ��
		if( HomeAgain )
		{
			HomeAgain = FALSE;
				
			// Delay 500ms
			BSP_OS_TimeDlyMs(500);
				
			if( IsMotorHomeValid(MOTOR_1) )
			{
				// �����쳣״̬
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_NON_ORIGIN);
				
				MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_ERR_MACHINE);
					
				// ���ش�����
				DatBuf[0] = 0x01;
				DatBuf[1] = MOTOR_GetCurrError(MOTOR_1);
				DatBuf[2] = MOTOR_GetCurrError(MOTOR_1)>>8;
			
				DatLen = 3;	
					
				APP_TRACE_INFO(("motor 1 can not leave home optical\n\r"));	
					
				// exit while
				bTastStat = DEF_FALSE;
				break;
			}
			else
			{
				OSSemSet(&semMtr1Ctrl, 0, &err);
				
				bRunStat = MOTOR_Home(MOTOR_1, pstFrameBuf->DatBuf[0]);  		
				if( DEF_TRUE != bRunStat )
				{						
					// �����쳣״̬
					MOTOR_SetCurrError(MOTOR_1, bRunStat);
					
					MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_ERR_PARA);
						
					// ���ش����� + ָ��֡
					DatBuf[0] = 0x01;
					DatBuf[1] = MOTOR_GetCurrError(MOTOR_1);
					DatBuf[2] = MOTOR_GetCurrError(MOTOR_1)>>8;
						
					memcpy( DatBuf+3, pRecvBuff+2, DatLen );
					
					DatLen = 3 + 2;	
						
					APP_TRACE_INFO(("parameter error for motor 1 msg\n\r"));
		
					// exit while
					bTastStat = DEF_FALSE;
					break;
				}						
			}
		}
		else
		{
			if( OS_ERR_NONE == err )
			{
				// ��������״̬
				MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_NORMAL);
					
				if( SYS_ERR_NONE == MOTOR_GetCurrError(MOTOR_1) )
				{					
					DatBuf[0] = 0x00;	
					DatBuf[1] = MOTOR_GetAxisPulse(MOTOR_1);
					DatBuf[2] = MOTOR_GetAxisPulse(MOTOR_1)>>8;
				}
				else
				{
					DatBuf[0] = 0x01;	
					DatBuf[1] = MOTOR_GetCurrError(MOTOR_1);
					DatBuf[2] = MOTOR_GetCurrError(MOTOR_1)>>8;
				}
					
				DatLen = 3;								
			}							
			else
			{
				// �����쳣״̬
				MOTOR_SetCurrError(MOTOR_1, SYS_ERR_MTR1_TIMEOUT);
				
				MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_ERR_TIMEOUT);
				
				// ���ش�����
				DatBuf[0] = 0x01;
				DatBuf[1] = MOTOR_GetCurrError(MOTOR_1);
				DatBuf[2] = MOTOR_GetCurrError(MOTOR_1)>>8;
				
				DatLen = 3;
						
				
				APP_TRACE_INFO(("motor 1 run timeout\n\r"));	
			}
				
			// exit while
			bTastStat = DEF_FALSE;
			break;
		}
	}
	
	// return 
	if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
		ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, DatBuf, DatLen );
	else 
		OSSemPost(&semExecMtrStat[MOTOR_1], OS_OPT_POST_1, &err);		
}

/*******************************************************************************
 * @brief  ��ת����������
 * @input  
 * @return 0-����, �����쳣 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void HEX_RoValveTask( u8 *pRecvBuff )
{
	CPU_CHAR  *pMsg;
	OS_ERR     err;
	u8    SndBuf[10];
	u8    MsgType;
	u8    DatLen;
	u8    bStat;
	u32   TimeOut;
	TASK_MSG    stOrderMsg; 
	P_FRM_PROC  pstFrameBuf;
	P_TASK_MSG  pstTaskMsg;
	
	memcpy( (u8 *)&stOrderMsg, pRecvBuff, sizeof(TASK_MSG) );
	
	pstTaskMsg = (P_TASK_MSG)&stOrderMsg;
	
	MsgType= pstTaskMsg->Type;	
	DatLen = pstTaskMsg->DatLen;
	
	pstFrameBuf = (P_FRM_PROC)pstTaskMsg->DatBuf;	
	
	// reset error number
	RV_SetErrorNumber(SYS_ERR_NONE);
		
	TimeOut = 6000;
			
	bStat = DEF_TRUE;
	switch( pstFrameBuf->Header.Struct.Order )
	{
		case RO_VALVE_PASS: //��ת��͸��
		{
			BSP_Ser_WrHex (UART_RV, (CPU_CHAR *)pstFrameBuf->DatBuf, DatLen - sizeof(HEADER) - 2 );	
			break;
		}
		case RO_VALVE_CTRL:					
		{
			if( 'H' == pstFrameBuf->DatBuf[0] ) 
			{
				RV_Home();
			}			
			else if( 'Q' == pstFrameBuf->DatBuf[0] )
			{
				RV_QueryStatus( );
				
				TimeOut = 1000;
			}
			else if( 'V' == pstFrameBuf->DatBuf[0] )
			{
				RV_QueryVersion( );
				
				TimeOut = 1000;
			}
			else if( (pstFrameBuf->DatBuf[0]>0) && (pstFrameBuf->DatBuf[0]<=RV_MAX_PORT) )
			{
				RV_MoveToPosition( pstFrameBuf->DatBuf[0] );
			}
			else
			{
				RV_SetErrorNumber(SYS_ERR_RV_PARA);
				
				bStat = DEF_FALSE;      
			}
			break;
		}
		case RO_VALVE_QUERY:					
		{
			RV_QueryStatus( );
				
			TimeOut = 1000;			
			break;
		}
		default:
		{
			RV_SetErrorNumber(SYS_ERR_RV_COMM);
			
			bStat = DEF_FALSE; 
			break;
		}
	}
	
	// �ȴ��ӻ���Ӧ
	if( DEF_FALSE == bStat )
	{
		// �쳣ִ�е�״̬
		RV_SetCurrStat(MODULE_STAT_NORMAL);
		
		// ���ش����� + ָ��֡
		SndBuf[0] = 0x01;
		SndBuf[1] = LOCAL_ADDR;
		SndBuf[2] = RV_GetErrorNumber();
		memcpy( SndBuf+3, pRecvBuff+2, DatLen );		
		
		if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
		{
			ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, SndBuf, 3 + DatLen );
		}
		else if( MSG_TYPE_FUNCTION==MsgType )
		{			
			OSSemPost(&semExecRoValStat, OS_OPT_POST_1, &err);				
		}
		
		APP_TRACE_INFO(("parameter error for valve msg\n\r"));	
			
		return ;
	}		
	
	// waiting for finishing
	pMsg = (CPU_CHAR *)OSQPend ((OS_Q         *)&MsgQueue[TASK_MSG_RV], 
								(OS_TICK       )TimeOut,
								(OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
								(OS_MSG_SIZE  *)sizeof(TASK_MSG),
								(CPU_TS       *)0,
								(OS_ERR       *)&err);	
	if( OS_ERR_NONE == err )
	{
		pstTaskMsg = (P_TASK_MSG)pMsg;		
					
		SndBuf[0] = 0x00;
		DatLen    = 1;
			
		if(RO_VALVE_PASS==pstFrameBuf->Header.Struct.Order )
		{
			memcpy( SndBuf + 1, pstTaskMsg->DatBuf, pstTaskMsg->DatLen);
				
			DatLen += pstTaskMsg->DatLen;
		}
		else if( 'V' == pstFrameBuf->DatBuf[0] )	
		{
			// ȥ�� 0x0D
			if( pstTaskMsg->DatLen>0 )
			{
				memcpy( SndBuf + 1, pstTaskMsg->DatBuf, pstTaskMsg->DatLen - 1);
				
				DatLen += (pstTaskMsg->DatLen - 1);
			}								
		}
		else if( (RO_VALVE_QUERY ==  pstFrameBuf->Header.Struct.Order) || ('Q' == pstFrameBuf->DatBuf[0]) )	
		{
			// ȥ�� 0x0D
			if( pstTaskMsg->DatLen>1 )
			{
				SndBuf[0] = 0x00;
				SndBuf[1] = (Ascii2Hex(pstTaskMsg->DatBuf[0])<<4) | (Ascii2Hex(pstTaskMsg->DatBuf[1])<<0);
				
				DatLen = 2;
			}								
		}
		else if( pstTaskMsg->DatLen>=3 )
		{
			SndBuf[1] = (Ascii2Hex(pstTaskMsg->DatBuf[0])<<4) | (Ascii2Hex(pstTaskMsg->DatBuf[1])<<0);			
			
			RV_SetErrorNumber( SYS_ERR_RV_INTERNAL + SndBuf[1] - RV_ERR_DATA_CRC );
				
			// ���ش�����
			SndBuf[0] = 0x01;
			SndBuf[1] = LOCAL_ADDR;
			SndBuf[2] = RV_GetErrorNumber();	
			DatLen = 3;
		}
		else
		{
			// ���ش�����
			SndBuf[0] = 0x00;				
			DatLen = 1;
		}	

		// ����ִ�е�״̬
		RV_SetCurrStat(MODULE_STAT_NORMAL);		
	}							
	else
	{	
		// �쳣ִ�е�״̬
		RV_SetErrorNumber(SYS_ERR_RV_TIMEOUT);
		
		RV_SetCurrStat(MODULE_STAT_ERR_TIMEOUT);
			
		// ���ش�����
		SndBuf[0] = 0x01;
		SndBuf[1] = LOCAL_ADDR;
		SndBuf[2] = RV_GetErrorNumber();			
	
		DatLen = 3;
			
		APP_TRACE_INFO(("Ro valve run timeout\n\r"));			
	}
	
	if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
		ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, SndBuf, DatLen );	
	else			
		OSSemPost(&semExecRoValStat, OS_OPT_POST_1, &err);		
}

/*******************************************************************************
 * @brief  ע��õ�������
 * @input  
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void HEX_SyrPumpTask( u8 *pRecvBuff )
{
	CPU_CHAR  *pMsg;
	OS_ERR     err;
	u8    DatBuf[20];
	u8    SndLen;
	u8    MsgType;
	u8    DatLen;
	u8    bStat;
	u32   TimeOut;
	u32   WholeTime;
	TASK_MSG    stOrderMsg; 
	P_FRM_PROC  pstFrameBuf;
	P_TASK_MSG  pstTaskMsg;
	
	memcpy( (u8 *)&stOrderMsg, pRecvBuff, sizeof(TASK_MSG) );
	
	pstTaskMsg = (P_TASK_MSG)&stOrderMsg;
	
	MsgType= pstTaskMsg->Type;	
	DatLen = pstTaskMsg->DatLen;
	
	pstFrameBuf = (P_FRM_PROC)pstTaskMsg->DatBuf;	
	
	// reset error number
	XLP_SetErrorNumber(SYS_ERR_NONE);
	
	TimeOut = 500;
	WholeTime = 0;
	
	bStat = DEF_TRUE;
	switch( pstFrameBuf->Header.Struct.Order )
	{
		case SYR_PUMP_PASS:      //ע���͸��
		{
			BSP_Ser_WrHex (UART_SYR, (CPU_CHAR *)pstFrameBuf->DatBuf, DatLen - sizeof(HEADER) - 2 );	
			break;
		}
		case SYR_PUMP_INIT:      //ע��ø�λ
		{
			if( DatLen == ( sizeof(HEADER) + 3 + 2) )
				XLP_Home( NULL, 1000, pstFrameBuf->DatBuf[1], pstFrameBuf->DatBuf[2] );
			else
				XLP_Home( NULL, 1000, 3, 3 );
			break;
		}
		case SYR_PUMP_IN:        //ע�����Һ
		{
			if( DatLen != ( sizeof(HEADER) + 3 + 2) )
			{
				XLP_SetErrorNumber(SYS_ERR_XLP_PARA);
				
				bStat = DEF_FALSE; 
			}
			else
			{
				XLP_RelInhale(NULL, SYR_VAL_CHN_1_SYR, MTR_MODEL_NORM, pstFrameBuf->DatBuf[0], pstFrameBuf->DatBuf[1] + ((u16)pstFrameBuf->DatBuf[2]<<8));  //MTR_MODEL_NORM
			}
			break;
		}
		case SYR_PUMP_OUT:       //ע�����Һ:					
		{
			if( DatLen != ( sizeof(HEADER) + 3 + 2) )
			{
				XLP_SetErrorNumber(SYS_ERR_XLP_PARA);
				
				bStat = DEF_FALSE; 
			}
			else
			{
				XLP_RelDrain(NULL, SYR_VAL_CHN_2_SYR, MTR_MODEL_NORM, pstFrameBuf->DatBuf[0], pstFrameBuf->DatBuf[1] + ((u16)pstFrameBuf->DatBuf[2]<<8));
			}
			break;
		}
		default:
		{
			XLP_SetErrorNumber(SYS_ERR_XLP_COMM);
			
			bStat = DEF_FALSE; 
			break;
		}
	}	
	
	// command or parameter error, set error number and set error status
	if( DEF_TRUE!=bStat )
	{
		// ���ش����� + ָ��֡
		DatBuf[0] = 0x01;
		DatBuf[1] = LOCAL_ADDR;
		DatBuf[2] = XLP_GetErrorNumber();
		memcpy( DatBuf+3, pRecvBuff+2, DatLen );
		
		if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
		{
			ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, DatBuf, 3 + DatLen );
		}
		else
		{
			OSSemPost(&semExecSyrPumpStat, OS_OPT_POST_1, &err);	
		}
		
		APP_TRACE_INFO(("parameter error for syringe pump msg\n\r"));		
		
		// �쳣ִ�е�״̬
		XLP_SetCurrStat(MODULE_STAT_ERR_PARA);
		
		return ;
	}
	
	// �ȴ��ӻ���Ӧ
	while( DEF_TRUE )
	{
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q         *)&MsgQueue[TASK_MSG_SYR], 
									(OS_TICK       )TimeOut,
									(OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
									(OS_MSG_SIZE  *)sizeof(TASK_MSG),
									(CPU_TS       *)0,
									(OS_ERR       *)&err);	
		if( OS_ERR_NONE == err )
		{
			pstTaskMsg = (P_TASK_MSG)pMsg;
			
			if( ('/'==pstTaskMsg->DatBuf[1])&&(XLP_COMM_IDLE==pstTaskMsg->DatBuf[3]) )
			{
				// ����ִ�е�״̬
				XLP_SetErrorNumber(SYS_ERR_NONE);
				
				XLP_SetCurrStat(MODULE_STAT_NORMAL);
				
				DatBuf[0] = 0x00;
				SndLen = 1;
				
				if( SYR_PUMP_PASS == pstFrameBuf->Header.Struct.Order )
				{
					memcpy( DatBuf + 1, pstTaskMsg->DatBuf, pstTaskMsg->DatLen);	
					SndLen += pstTaskMsg->DatLen;		
				}				
					
				APP_TRACE_INFO(("Syringe pump run ok\n\r"));				
				
				// exit while
				break;
			}
			else if( ('/'==pstTaskMsg->DatBuf[1])&&(XLP_COMM_BUSY==pstTaskMsg->DatBuf[3]) )
			{
				BSP_OS_TimeDlyMs(TimeOut);
				
				WholeTime += TimeOut;
				
				if( WholeTime>MAX_SYR_ASPR_TIME_MS )
				{
					// �쳣ִ�е�״̬
					XLP_SetErrorNumber(SYS_ERR_XLP_TIMEOUT);
					
					XLP_SetCurrStat(MODULE_STAT_ERR_TIMEOUT);
					
					// ���ش�����
					DatBuf[0] = 0x01;
					DatBuf[1] = LOCAL_ADDR;
					DatBuf[2] = XLP_GetErrorNumber();
			
					SndLen = 3;	
					
					APP_TRACE_INFO(("Syringe pump run timeout F1\n\r"));	
										
					// exit while
					break;
				}
				else
				{
					// Query XLP current status
					XLP_QueryStatus(NULL);
					
					APP_TRACE_INFO(("XLP_QueryStatus\n\r"));	
				}
			}			
			else if( ('/'==pstTaskMsg->DatBuf[1])&&(pstTaskMsg->DatBuf[3]>0x40)&&(pstTaskMsg->DatBuf[3]<=0x4F) )
			{
				// �쳣ִ�е�״̬
				XLP_SetErrorNumber( SYS_ERR_XLP_INTERNAL + pstTaskMsg->DatBuf[2] - 0x41 );
					
				XLP_SetCurrStat(MODULE_STAT_NORMAL);
				
				// ���ش�����
				DatBuf[0] = 0x01;
				DatBuf[1] = LOCAL_ADDR;
				DatBuf[2] = XLP_GetErrorNumber();
			
				SndLen = 3;	
					
				APP_TRACE_INFO(("Syringe pump run timeout F2\n\r"));	
			}
			else
			{
				// �쳣ִ�е�״̬
				XLP_SetErrorNumber(SYS_ERR_XLP_RETURN_UNKOW);
				
				XLP_SetCurrStat(MODULE_STAT_NORMAL);
				
				// ���ش�����
				DatBuf[0] = 0x01;
				DatBuf[1] = LOCAL_ADDR;
				DatBuf[2] = XLP_GetErrorNumber();
			
				SndLen = 3;	
				
				APP_TRACE_INFO(("Syringe pump error response\n\r"));					
					
				// exit while
				break;
			}
		}							
		else     
		{	
			// �쳣ִ�е�״̬
			XLP_SetErrorNumber(SYS_ERR_XLP_TIMEOUT);
			
			XLP_SetCurrStat(MODULE_STAT_ERR_TIMEOUT);
			
			// ���ش�����
			DatBuf[0] = 0x01;
			DatBuf[1] = LOCAL_ADDR;
			DatBuf[2] = XLP_GetErrorNumber();	
			
			SndLen = 3;		
			
			APP_TRACE_INFO(("Syringe pump run timeout F0\n\r"));	
			
			// exit while
			break;
		}  // if( OS_ERR_NONE == err )
	}  // while( DEF_TRUE )	

	// return 
	if( (MSG_TYPE_PROTOCOL==MsgType)&&(FRM_TYPE_1==pstFrameBuf->Header.Struct.Type) )
		ResultUpLayerComm( pstFrameBuf->Header.Struct.Index, pstFrameBuf->Header.Struct.Order, DatBuf, SndLen );	
	else
		OSSemPost(&semExecSyrPumpStat, OS_OPT_POST_1, &err);
}
#endif // #ifdef PR_MODBUS
