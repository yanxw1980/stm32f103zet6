/*******************************************************************************
 * @file    MotorDrv.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-22
 * @brief   串口端的应用程序,主要是与上层机器的通信
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes -----------------------------------------------------------------*/ 
#include <stdio.h>
#include <string.h>
#include  <bsp.h>
#include "bsp_ser.h"
#include "MotorDrv.H"
#include "HexProtocol.h"
#include "SysError.h"
#include "SpiFlash.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables -----------------------------------------------------------*/
static u8  MotorStat[MOTOR_MAX];

// 系统设定的电机台阶数和每个台阶的步数
static MTR_STEP_PARA stMtrAccPara[MOTOR_MAX];		   
static MTR_STEP_PARA stMtrDecPara[MOTOR_MAX];

// 电机控制参数  
static CTRL_TYPE_STAT stMtrCtrlInfo[MOTOR_MAX];
static RUN_STAT_PARA  stMtrRunPara[MOTOR_MAX];

MTR_POS_PARA  stMtrPosPara[MOTOR_MAX];

/* Private function prototypes --------------------------------------------------*/
static void MOTOR_FreqControl(BYTE nMtrNo, P_RUN_STAT_PARA pstMtrRunStat);
static BYTE MOTOR_CalcAccDecPara(const P_MTR_STEP_PARA pstAccStep, const P_MTR_STEP_PARA pstDecStep, P_RUN_STAT_PARA pstMtrRunStat);

OS_SEM semMtr1Ctrl;
#if defined(MOTOR_DEF_2)
OS_SEM semMtr2Ctrl;
#endif

/* Private functions -----------------------------------------------------------*/ 

/*******************************************************************************
 * @brief  设置注射泵的控制状态 
 * @input  stat,状态
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MOTOR_SetCurrStat(u8 MtrNo, u8 stat)
{
	if( MtrNo >= MOTOR_MAX )
		return ;
	
	// 未联接的情况下设置超时则认为通信问题
	if( MODULE_STAT_DISCONNECT == MotorStat[MtrNo] )
	{
		if( (MODULE_STAT_ERR_TIMEOUT == stat) || (MODULE_STAT_ERR_PARA == stat) )
			MotorStat[MtrNo] = MODULE_STAT_ERR_COMM;
		else
			MotorStat[MtrNo] = stat;
	}
	else
	{
		MotorStat[MtrNo] = stat;
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
u8 MOTOR_GetCurrStat(u8 MtrNo)
{ 
	if( MtrNo < MOTOR_MAX )
		return MotorStat[MtrNo];
	else
		return 0;
}

/*******************************************************************************
 * @brief  运动坐标更新
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE MOTOR_GetCurrCtrlType(BYTE nMtrNo)
{
	return stMtrCtrlInfo[nMtrNo].ucMtrCtrl;
}

/*******************************************************************************
 * @brief  运动坐标更新
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE MOTOR_GetDivision(BYTE nMtrNo)
{
	BYTE nDiv;
	
	nDiv = 0;
	if( MOTOR_1==nMtrNo )
		nDiv = M1_DIVISION;
#if defined(MOTOR_DEF_2)
	else if( MOTOR_2==nMtrNo )
		nDiv = M2_DIVISION;
#endif
	return nDiv;
}

/*******************************************************************************
 * @brief  获取电机控制参数 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u32 MOTOR_GetPosPara(void)
{ 
	u8 DatBuff[64];
	P_MTR_POS_PARA pstMtrPosPara;
	
	// read parameters from flash
	sFLASH_ReadBuffer(DatBuff, FLASH_MOTOR_ADDR_BASE, sizeof(MTR_POS_PARA));
	
	pstMtrPosPara = (P_MTR_POS_PARA)DatBuff;

//pstMtrPosPara->PosTide = 0xBB80;
stMtrPosPara[MOTOR_1].PosTide = 0xD800;   //0xC880
	
	// check length
	if( (pstMtrPosPara->sDatLen == 0) || (pstMtrPosPara->sDatLen > sizeof(MTR_POS_PARA)) )
	{	
		return DEF_FALSE;
	}
	
	// check crc	
	if( pstMtrPosPara->sCRC != CRC_GetValue( (u8 *)&pstMtrPosPara->PosZero, sizeof(MTR_POS_PARA) - 4) )
		return DEF_FALSE;
	
	// copy data
	memcpy( stMtrPosPara, (u8 *)&pstMtrPosPara->PosZero, sizeof(MTR_POS_PARA) - 4 );

	return DEF_TRUE;
}

/*******************************************************************************
 * @brief  获取电机控制参数 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u32 MOTOR_SavePosPara(u8 ParaBuff[], u16 DatLength, u8 bWriteToFlash)
{ 
	u8 DatBuff[64];
	P_MTR_POS_PARA pstMtrPosPara;
	
	if( DatLength < sizeof(MTR_POS_PARA) )
		return DEF_FALSE;
	
	// copy data
	memcpy( stMtrPosPara, ParaBuff, DatLength );
	
	if( bWriteToFlash )
	{
		pstMtrPosPara = (P_MTR_POS_PARA)DatBuff;
		
		memcpy( DatBuff + 4, ParaBuff, DatLength );
		
		pstMtrPosPara->sDatLen = sizeof(MTR_POS_PARA);
		pstMtrPosPara->sCRC    = CRC_GetValue(ParaBuff, DatLength);
		
		sFLASH_WriteBuffer(DatBuff, FLASH_MOTOR_ADDR_BASE, sizeof(MTR_POS_PARA));
	}
	
	return DEF_TRUE;
}

/*******************************************************************************
 * @brief  运动坐标更新
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MOTOR_PulseUpdate(BYTE nMtrNo, BYTE bPulseLow)
{
	P_RUN_STAT_PARA  pstMtrRunStat;
	P_CTRL_TYPE_STAT pstMtrCtrlInfo;	
	BYTE bCurrHomeStat;
	
	if( nMtrNo>=MOTOR_MAX )
		return ;
	
	TIM_ControlMotor(nMtrNo, OFF);
	
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[nMtrNo];
	pstMtrCtrlInfo= (P_CTRL_TYPE_STAT)&stMtrCtrlInfo[nMtrNo];	

	if( TRUE==bPulseLow )
	{
		// 更新运动距离
		if( pstMtrRunStat->MovePulse>0 )
		{
			pstMtrRunStat->MovePulse -= 1;
		}		
		
		// 运动计数
		pstMtrRunStat->PulseCount += 1;		
		
		// 更新坐标
		if( MOTOR_DIR_TO_ORIG==pstMtrRunStat->ucMoveDir )
		{
			if( pstMtrRunStat->AxisPulse>0 )
			{
				pstMtrRunStat->AxisPulse -= 1;
			}
		}
		else
		{
			if( pstMtrCtrlInfo->fcIsLimitValid(nMtrNo) )
			{
				// stop motor	
				(*pstMtrCtrlInfo->fcMtrStop)(nMtrNo);
					
				pstMtrRunStat->ucMoveStat   = RUN_STAT_ERROR;	
				MOTOR_SetCurrError(nMtrNo, SYS_ERR_MTR1_LOW_LIMIT);	
				
				// send semaphore to task
				if( MOTOR_1==nMtrNo )
					BSP_OS_SemPost(&semMtr1Ctrl);
#if defined(MOTOR_DEF_2)				
				else if( MOTOR_2==nMtrNo )
					BSP_OS_SemPost(&semMtr2Ctrl);
#endif				
				return ;
			}
			else
			{
				pstMtrRunStat->AxisPulse += 1;
				
				if( pstMtrRunStat->AxisPulse>=pstMtrCtrlInfo->MaxPulse )
				{
					// stop motor	
					(*pstMtrCtrlInfo->fcMtrStop)(nMtrNo);
					
					pstMtrRunStat->ucMoveStat   = RUN_STAT_ERROR;						
					MOTOR_SetCurrError(nMtrNo, SYS_ERR_MTR1_OVER_RUN);	
					
					// send semaphore to task
					if( MOTOR_1==nMtrNo )
						BSP_OS_SemPost(&semMtr1Ctrl);
#if defined(MOTOR_DEF_2)					
					else if( MOTOR_2==nMtrNo )
						BSP_OS_SemPost(&semMtr2Ctrl);
#endif
					
					return ;
				}
			}
		}
		
		// 回原点
		if( CTRL_TYPE_INIT==MOTOR_GetCurrCtrlType(nMtrNo) )
		{
			bCurrHomeStat = pstMtrCtrlInfo->fcIsHomeValid(nMtrNo);
			if( bCurrHomeStat && (pstMtrRunStat->bHomePrevStat != bCurrHomeStat) )
			{
				if( 0==pstMtrCtrlInfo->usDitoOrigPulse )
				{
					pstMtrRunStat->MovePulse  = 0;
				}
				else
				{
					pstMtrRunStat->MovePulse  = pstMtrCtrlInfo->usDitoOrigPulse;
					pstMtrRunStat->ucDecGrdIdx= pstMtrRunStat->ucAccGrdIdx;
					pstMtrRunStat->ucMoveStat = RUN_STAT_DEC;
				}					
			}
			
			pstMtrRunStat->bHomePrevStat = bCurrHomeStat;
		}
		
		// stop motor
		if( 0==pstMtrRunStat->MovePulse )
		{
			(*pstMtrCtrlInfo->fcMtrStop)(nMtrNo);
			pstMtrRunStat->ucMoveStat = RUN_STAT_DONE;
			
			MOTOR_SetCurrError(nMtrNo, SYS_ERR_NONE);
			
			// update origin mark
			if( CTRL_TYPE_INIT==MOTOR_GetCurrCtrlType(nMtrNo) )
				pstMtrRunStat->bOriginProc = TRUE;
			
			// send semaphore to task			
			if( MOTOR_1==nMtrNo )
				BSP_OS_SemPost(&semMtr1Ctrl);
#if defined(MOTOR_DEF_2)			
			else if( MOTOR_2==nMtrNo )
				BSP_OS_SemPost(&semMtr2Ctrl);
#endif			
		}
		else
		{
			// 设置频率
			MOTOR_FreqControl(nMtrNo, (P_RUN_STAT_PARA)&stMtrRunPara[nMtrNo]);
			
			TIM_ControlMotor(nMtrNo, ON);
		}
	}		
}

/**********************************************************************************
 * Func. Name: MOTOR_ParameterInit
 *Discription: 电机参数初始化
 *     Inputs:     
 *    Outputs:
 *     Remask:
 *********************************************************************************/	
BYTE MOTOR_ParameterInit(BYTE nMtrNo)
{
	P_CTRL_TYPE_STAT pstMtrCtrlInfo;
	P_RUN_STAT_PARA  pstMtrRunStat;
	u32 MtrBaseFreq[MOTOR_MAX];
	u16 StepPulse[MOTOR_MAX];
	s16 idx;	
	u32 StepFreq[MOTOR_MAX];
	
	if( nMtrNo>=MOTOR_MAX )
	{
		return FALSE;
	}	
	
	StepFreq[MOTOR_1]  = MOTOR_CYCLE_TO_PULSE(1, M1_DIVISION);
	StepPulse[MOTOR_1] = (M1_DIVISION<8) ? 2 : (M1_DIVISION>>1); 

#if defined(MOTOR_DEF_2)	
	StepFreq[MOTOR_2]  = MOTOR_CYCLE_TO_PULSE(1, M2_DIVISION);	
	StepPulse[MOTOR_2] = (M2_DIVISION<8) ? 2 : (M2_DIVISION>>1);
#endif
	
	// 初始化电机加速台阶
	for(idx = 0; idx < MOTOR_ACC_GRADE; idx++)
	{
		if( 0==idx )
		{
			MtrBaseFreq[MOTOR_1] = MOTOR_CYCLE_TO_PULSE(MOTOR_SPEED_MIN, M1_DIVISION);  
			
#if defined(MOTOR_DEF_2)			
			MtrBaseFreq[MOTOR_2] = MOTOR_CYCLE_TO_PULSE(MOTOR_SPEED_MIN, M2_DIVISION);
#endif
		}
		
		// 加速控制参数
		if( idx>0 )
			stMtrAccPara[MOTOR_1].sPulse[idx]  = StepPulse[MOTOR_1] + stMtrAccPara[MOTOR_1].sPulse[idx-1];
		else
			stMtrAccPara[MOTOR_1].sPulse[idx]  = StepPulse[MOTOR_1];
		stMtrAccPara[MOTOR_1].lFreq[idx]   = MtrBaseFreq[MOTOR_1] + (idx * StepFreq[MOTOR_1] );
		
		// 减速控制参数
		if( idx>0 )
			stMtrDecPara[MOTOR_1].sPulse[idx]  = (StepPulse[MOTOR_1]<<1) + stMtrDecPara[MOTOR_1].sPulse[idx-1];
		else
			stMtrDecPara[MOTOR_1].sPulse[idx]  = (StepPulse[MOTOR_1]<<1);
		stMtrDecPara[MOTOR_1].lFreq[idx]   = MtrBaseFreq[MOTOR_1] + (idx * StepFreq[MOTOR_1] );

		// 加速控制参数
#if defined(MOTOR_DEF_2)		
		if( idx>0 )
			stMtrAccPara[MOTOR_2].sPulse[idx]  = StepPulse[MOTOR_2] + stMtrAccPara[MOTOR_2].sPulse[idx-1];
		else
			stMtrAccPara[MOTOR_2].sPulse[idx]  = StepPulse[MOTOR_2];
		stMtrAccPara[MOTOR_2].lFreq[idx]   = MtrBaseFreq[MOTOR_2] + (idx * StepFreq[MOTOR_2]);
		
		// 减速控制参数
		if( idx>0 )
			stMtrDecPara[MOTOR_2].sPulse[idx]  = (StepPulse[MOTOR_2]<<1) + stMtrDecPara[MOTOR_2].sPulse[idx-1];
		else
			stMtrDecPara[MOTOR_2].sPulse[idx]  = (StepPulse[MOTOR_2]<<1);
		stMtrDecPara[MOTOR_2].lFreq[idx]   = MtrBaseFreq[MOTOR_2] + (idx * StepFreq[MOTOR_2] );
#endif		
	}
	
	if( MOTOR_1==nMtrNo )
	{
		// 初始化控制参数		
		pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MOTOR_1]; 

		pstMtrRunStat->bOriginProc = FALSE;            // 是否完成初始化操作 
		
		pstMtrRunStat->AxisPulse   = 0;                // 当前坐标脉冲
		pstMtrRunStat->MovePulse   = 0;                // 运动距离脉冲
		pstMtrRunStat->PulseCount  = 0;                // 运动脉冲计数
		
		pstMtrRunStat->ulDrvFreq   = 0;                // 准备起动速度
		pstMtrRunStat->ulMaxFreq   = 0;                // 准备最大速度	
			
		pstMtrRunStat->ucAccGrdNum = 0;		           // 当前距离对应的加速台阶级数
		pstMtrRunStat->ucAccGrdIdx = 0;
		pstMtrRunStat->ucDecGrdNum = 0;	                // 当前距离对应的减速台阶级数
		pstMtrRunStat->ucDecGrdIdx = 0;	
		
		pstMtrRunStat->pstAccPara  = (P_MTR_STEP_PARA)&stMtrAccPara[MOTOR_1];
		pstMtrRunStat->pstDecPara  = (P_MTR_STEP_PARA)&stMtrDecPara[MOTOR_1];
		pstMtrRunStat->ucMoveDir   = MOTOR_DIR_TO_ORIG;        // 运动方向: 向光藕, 离光藕
		pstMtrRunStat->ucMoveStat  = RUN_STAT_WAIT;            // 运动状态: 匀速, 加速, 高速, 减速	

		pstMtrCtrlInfo = (P_CTRL_TYPE_STAT)&stMtrCtrlInfo[MOTOR_1]; 
		pstMtrCtrlInfo->ucMtrCtrl      = CTRL_TYPE_IDLE;              
		pstMtrCtrlInfo->MaxPulse       = M1_MAX_RUN_PULSE;                       // 最大行程脉冲数
		pstMtrCtrlInfo->usDitoOrigPulse= M1_PULSE_ORIGIN;                        // 光藕到原点距离
		pstMtrCtrlInfo->Division       = M1_DIVISION;                            // 细分
		pstMtrCtrlInfo->DrvPPS         = M1_DRV_PPS;                             // 起跳速度(整步)
		pstMtrCtrlInfo->CurrErrorNo    = SYS_ERR_NONE;
		
		pstMtrCtrlInfo->fcMtrStop      = MOTOR_MotorStop;
		pstMtrCtrlInfo->fcIsHomeValid  = IsMotorHomeValid;    // 光藕挡住	
		pstMtrCtrlInfo->fcIsLimitValid = IsMotorLimitValid;
		pstMtrRunStat->bHomePrevStat   = pstMtrCtrlInfo->fcIsHomeValid(MOTOR_1);      

		BSP_OS_SemCreate(&semMtr1Ctrl, 0, (CPU_CHAR *)"Motor1Status");
		
		stMtrPosPara[MOTOR_1].PosZero  = 0;
		stMtrPosPara[MOTOR_1].PosTide  = (M1_MAX_RUN_PULSE>>1);
	}

#if defined(MOTOR_DEF_2)
	else if( MOTOR_2==nMtrNo )
	{
	   // -----------------MOTOR 2----------------------------   
		pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MOTOR_2]; 

		pstMtrRunStat->bOriginProc = FALSE;            // 是否完成初始化操作 
		
		pstMtrRunStat->AxisPulse   = 0;                // 当前坐标脉冲
		pstMtrRunStat->MovePulse   = 0;                // 运动距离脉冲
		pstMtrRunStat->PulseCount  = 0;                // 运动脉冲计数
		
		pstMtrRunStat->ulDrvFreq   = 0;                // 准备起动速度
		pstMtrRunStat->ulMaxFreq   = 0;                // 准备最大速度
			
		pstMtrRunStat->ucAccGrdNum = 0;		           // 当前距离对应的加速台阶级数
		pstMtrRunStat->ucAccGrdIdx = 0;
		pstMtrRunStat->ucDecGrdNum = 0;	               // 当前距离对应的减速台阶级数
		pstMtrRunStat->ucDecGrdIdx = 0;	
		
		pstMtrRunStat->pstAccPara  = (P_MTR_STEP_PARA)&stMtrAccPara[MOTOR_2];
		pstMtrRunStat->pstDecPara  = (P_MTR_STEP_PARA)&stMtrDecPara[MOTOR_2];
		pstMtrRunStat->ucMoveDir   = MOTOR_DIR_TO_ORIG;        // 运动方向: 向光藕, 离光藕
		pstMtrRunStat->ucMoveStat  = RUN_STAT_WAIT;            // 运动状态: 匀速, 加速, 高速, 减速	

		pstMtrCtrlInfo = (P_CTRL_TYPE_STAT)&stMtrCtrlInfo[MOTOR_2]; 
		pstMtrCtrlInfo->ucMtrCtrl      = CTRL_TYPE_IDLE;              
		pstMtrCtrlInfo->MaxPulse       = M2_MAX_RUN_PULSE;                       // 最大行程脉冲数
		pstMtrCtrlInfo->usDitoOrigPulse= M2_PULSE_ORIGIN;                        // 光藕到原点距离
		pstMtrCtrlInfo->Division       = M2_DIVISION;                            // 细分
		pstMtrCtrlInfo->DrvPPS         = M2_DRV_PPS;                             // 起跳速度(整步)
		pstMtrCtrlInfo->CurrErrorNo    = SYS_ERR_NONE;
		
		pstMtrCtrlInfo->fcMtrStop      = MOTOR_MotorStop;
		pstMtrCtrlInfo->fcIsHomeValid  = IsMotorHomeValid;    // 光藕挡住	
		pstMtrCtrlInfo->fcIsLimitValid = IsMotorLimitValid;
		pstMtrRunStat->bHomePrevStat   = pstMtrCtrlInfo->fcIsHomeValid(MOTOR_2);   
		
		BSP_OS_SemCreate(&semMtr2Ctrl, 0, (CPU_CHAR *)"Motor2Status");
		
		stMtrPosPara[MOTOR_2].PosZero  = 0;
		stMtrPosPara[MOTOR_2].PosTide  = (M2_MAX_RUN_PULSE>>2);
	}
#endif	

	return TRUE;
}

/*******************************************************************************
 * @brief  
 * @input  : motor index: 0/1
 *         : 
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MOTOR_SetCurrError(u8 MtrNo, u8 err)
{
	if( MtrNo < MOTOR_MAX )
	{
		if( SYS_ERR_NONE == err )
			stMtrCtrlInfo[MtrNo].CurrErrorNo = SYS_ERR_NONE;
		else
			stMtrCtrlInfo[MtrNo].CurrErrorNo = ((u16)GetLocalAddr()<<8) | ( (MOTOR_1==MtrNo)?(err):(err+(SYS_ERR_BASE_MTR2-SYS_ERR_BASE_MTR1)) );
	}
}

/*******************************************************************************
 * @brief  
 * @input  : motor index: 0/1
 *         : 
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 MOTOR_GetCurrError(u8 MtrNo)
{
	if( MtrNo>=MOTOR_MAX )
		return 0;
	
	return stMtrCtrlInfo[MtrNo].CurrErrorNo;
}

/*******************************************************************************
 * @brief  电机控制参数初始化
 * @input  : motor index: 0/1
 *         : max frequence index of motor rps 
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 MOTOR_Home(u8 MtrNo, u8 MaxRpsX10)          
{
	P_MTR_STEP_PARA pstAccStep;
	P_MTR_STEP_PARA pstDecStep;
	P_RUN_STAT_PARA pstMtrRunStat;
	OS_ERR err;
	
	if( MtrNo>=MOTOR_MAX )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}
	
	// 最小0.5转每秒
	if( MaxRpsX10<5 )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}
	
	pstAccStep = (P_MTR_STEP_PARA)&stMtrAccPara[MtrNo];
	pstDecStep = (P_MTR_STEP_PARA)&stMtrDecPara[MtrNo];
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MtrNo];	
	
	if( (stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_AVG)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_ACC)	
	  ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_MAX)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_DEC) )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_RUNNING);
		
		return DEF_FALSE;
	}
	
	// 计算最大速度
	stMtrRunPara[MtrNo].ulMaxFreq = MaxRpsX10 * MOTOR_GetDivision(MtrNo) * 20ul;	
		
	stMtrCtrlInfo[MtrNo].ucMtrCtrl = CTRL_TYPE_INIT;	   
	stMtrRunPara[MtrNo].ucMoveStat = RUN_STAT_WAIT;
	
	// 记录光藕状态
	stMtrRunPara[MtrNo].bHomePrevStat = stMtrCtrlInfo[MtrNo].fcIsHomeValid(MtrNo);	

	stMtrRunPara[MtrNo].MovePulse = stMtrCtrlInfo[MtrNo].MaxPulse;    
	stMtrRunPara[MtrNo].ucMoveDir = MOTOR_DIR_TO_ORIG;   
	
	// 运动计数
	stMtrRunPara[MtrNo].PulseCount  = 0;
	stMtrRunPara[MtrNo].ucAccGrdIdx = 0;
	stMtrRunPara[MtrNo].ucDecGrdIdx = 0; 
	
	// 清除错误	
	MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
	
	// 计算加减速台阶
	if( DEF_TRUE == MOTOR_CalcAccDecPara(pstAccStep, pstDecStep, pstMtrRunStat) )
	{		
		MOTOR_SetMotorDirt(MtrNo, pstMtrRunStat->ucMoveDir);
				
		//Start motor	
		TIM_InitMotorClock(MtrNo, pstMtrRunStat->pstAccPara->lFreq[0]);
						
		MOTOR_MotorRun(MtrNo);		
	
		// clear semaphore
		if( MOTOR_1==MtrNo )
			OSSemSet(&semMtr1Ctrl, 0, &err);  
#if defined(MOTOR_DEF_2)
		else if( MOTOR_2==MtrNo )
			OSSemSet(&semMtr2Ctrl, 0, &err);  
#endif		
		
		MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
		
		return DEF_TRUE;
	}
	else
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
	
		return DEF_FALSE;
	}
}


/*******************************************************************************
 * @brief  电机控制参数初始化
 * @input  : MtrNo, motor index: 0/1
 *         : MovePulse, move distance in frequnce
 *         : MaxRpsX10, max frequence index of motor rps
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 MOTOR_MoveAbsolute(u8 MtrNo, u32 MovePulse, u8 MaxRpsX10)           
{
	P_MTR_STEP_PARA pstAccStep;
	P_MTR_STEP_PARA pstDecStep;
	P_RUN_STAT_PARA pstMtrRunStat;
	OS_ERR err;
	
	
	if( MtrNo>=MOTOR_MAX )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}
	
	// 最小0.5转每秒
	if( MaxRpsX10<5 )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}
	
	pstAccStep = (P_MTR_STEP_PARA)&stMtrAccPara[MtrNo];
	pstDecStep = (P_MTR_STEP_PARA)&stMtrDecPara[MtrNo];
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MtrNo];	
	
	// 系统未复位
	if( TRUE!=pstMtrRunStat->bOriginProc )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_NO_RESET);
		
		return DEF_FALSE;
	}
	
	if( (stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_AVG)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_ACC)	
	  ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_MAX)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_DEC) )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_RUNNING);
		
		return DEF_FALSE;
	}
	
	// 计算最大速度
	stMtrRunPara[MtrNo].ulMaxFreq = MaxRpsX10 * MOTOR_GetDivision(MtrNo) * 20ul;	
	
	stMtrCtrlInfo[MtrNo].ucMtrCtrl = CTRL_TYPE_MOVE_ABS;	   
	stMtrRunPara[MtrNo].ucMoveStat = RUN_STAT_WAIT;
	
	// 运动到指定位置
	if( stMtrRunPara[MtrNo].AxisPulse>MovePulse )
	{
		stMtrRunPara[MtrNo].MovePulse = stMtrRunPara[MtrNo].AxisPulse - MovePulse;       
		stMtrRunPara[MtrNo].ucMoveDir = MOTOR_DIR_TO_ORIG;   	
	}
	else if( stMtrRunPara[MtrNo].AxisPulse<MovePulse )
	{
		stMtrRunPara[MtrNo].MovePulse = MovePulse - stMtrRunPara[MtrNo].AxisPulse;       
		stMtrRunPara[MtrNo].ucMoveDir = MOTOR_DIR_GO_MAX;   	
	}
	else
	{
		stMtrRunPara[MtrNo].MovePulse = 0;       
		stMtrRunPara[MtrNo].ucMoveDir = MOTOR_DIR_TO_ORIG; 
		
		MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
		
		return DEF_TRUE;
	}		
	
	// 运动计数
	stMtrRunPara[MtrNo].PulseCount  = 0;
	stMtrRunPara[MtrNo].ucAccGrdIdx = 0;
	stMtrRunPara[MtrNo].ucDecGrdIdx = 0; 
	
	// 清除错误	
	MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
	
	// 计算加减速台阶
	if( DEF_TRUE == MOTOR_CalcAccDecPara(pstAccStep, pstDecStep, pstMtrRunStat) )
	{		
		MOTOR_SetMotorDirt(MtrNo, pstMtrRunStat->ucMoveDir);
				
		//Start motor	
		TIM_InitMotorClock(MtrNo, pstMtrRunStat->pstAccPara->lFreq[0]);
						
		MOTOR_MotorRun(MtrNo);		
	
		// clear semaphore
		if( MOTOR_1==MtrNo )
			OSSemSet(&semMtr1Ctrl, 0, &err);  
#if defined(MOTOR_DEF_2)
		else if( MOTOR_2==MtrNo )
			OSSemSet(&semMtr2Ctrl, 0, &err);  
#endif		
		
		MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
		
		return DEF_TRUE;
	}
	else
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;
	}
}

/*******************************************************************************
 * @brief  电机按照设定的距离和方向,进行相对运动
 * @input  : MtrNo, motor index: 0/1
 *         : MovePulse, move distance in frequnce
 *         : MoveDir, move direction, move to origin diode, or off origin diode
 *         : MaxRpsX10, max frequence index of motor rps      
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u8 MOTOR_MoveRelative(u8 MtrNo, u32 MovePulse, u8 MoveDir, u8 MaxRpsX10)
{
	P_MTR_STEP_PARA pstAccStep;
	P_MTR_STEP_PARA pstDecStep;
	P_RUN_STAT_PARA pstMtrRunStat;
	OS_ERR err;
			
	if( MtrNo>=MOTOR_MAX )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}
	
	if( 0==MovePulse )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
		
		return DEF_TRUE;
	}
	
	// 最小0.5转每秒
	if( MaxRpsX10<5 )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;	
	}		
	
	pstAccStep = (P_MTR_STEP_PARA)&stMtrAccPara[MtrNo];
	pstDecStep = (P_MTR_STEP_PARA)&stMtrDecPara[MtrNo];
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MtrNo];	
	
	if( (stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_AVG)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_ACC)	
	  ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_MAX)
      ||(stMtrRunPara[MtrNo].ucMoveStat==RUN_STAT_DEC) )
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_RUNNING);
		
		return DEF_FALSE;
	}
	
	// 计算最大速度
	stMtrRunPara[MtrNo].ulMaxFreq = MaxRpsX10 * MOTOR_GetDivision(MtrNo) * 20ul;	
	
	stMtrCtrlInfo[MtrNo].ucMtrCtrl = CTRL_TYPE_MOVE_REL;	   
	stMtrRunPara[MtrNo].ucMoveStat = RUN_STAT_WAIT;
	
	// 记录光藕状态
	stMtrRunPara[MtrNo].bHomePrevStat = stMtrCtrlInfo[MtrNo].fcIsHomeValid(MtrNo);	

	// 运动相对距离
	stMtrRunPara[MtrNo].MovePulse = MovePulse;       
	stMtrRunPara[MtrNo].ucMoveDir = MoveDir;  
		
	// 运动计数
	stMtrRunPara[MtrNo].PulseCount  = 0;
	stMtrRunPara[MtrNo].ucAccGrdIdx = 0;
	stMtrRunPara[MtrNo].ucDecGrdIdx = 0; 
	
	// 清除错误	
	MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
	
	// 计算加减速台阶
	if( DEF_TRUE == MOTOR_CalcAccDecPara(pstAccStep, pstDecStep, pstMtrRunStat) )
	{		
		MOTOR_SetMotorDirt(MtrNo, pstMtrRunStat->ucMoveDir);
				
		//Start motor	
		TIM_InitMotorClock(MtrNo, pstMtrRunStat->pstAccPara->lFreq[0]);
						
		MOTOR_MotorRun(MtrNo);		
	
		// clear semaphore
		if( MOTOR_1==MtrNo )
			OSSemSet(&semMtr1Ctrl, 0, &err);  
#if defined(MOTOR_DEF_2)
		else if( MOTOR_2==MtrNo )
			OSSemSet(&semMtr2Ctrl, 0, &err);  
#endif		
		
		MOTOR_SetCurrError(MtrNo, SYS_ERR_NONE);
		
		return DEF_TRUE;
	}
	else
	{
		MOTOR_SetCurrError(MtrNo, SYS_ERR_MTR1_PARA);
		
		return DEF_FALSE;
	}
}

/*******************************************************************************
 * @brief  获取电机当前位置
 * @input  MtrNo, 电机编号
 * @return 返回坐标值
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
u16 MOTOR_GetAxisPulse(BYTE MtrNo)
{
	P_RUN_STAT_PARA  pstMtrRunStat;
	if( MtrNo>=MOTOR_MAX )
		return 0;
	
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MtrNo];
	
	return pstMtrRunStat->AxisPulse;
}

/*******************************************************************************
 * @brief  获取电机运行状态
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE MOTOR_GetRunStatus(BYTE MtrNo)
{
	P_RUN_STAT_PARA  pstMtrRunStat;
	if( MtrNo>=MOTOR_MAX )
		return 0;
	
	pstMtrRunStat = (P_RUN_STAT_PARA)&stMtrRunPara[MtrNo];
	
	return pstMtrRunStat->ucMoveStat;
}

/*******************************************************************************
 * @brief  计算加减速台阶
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static BYTE MOTOR_CalcAccDecPara(const P_MTR_STEP_PARA pstAccStep, const P_MTR_STEP_PARA pstDecStep, P_RUN_STAT_PARA pstMtrRunStat)
{
	INT32U MaxPulse;
	INT8U  idx;
	
#ifdef _DEBUG_
	char  str[20];
#endif

	pstMtrRunStat->ucAccGrdNum = 0;
	pstMtrRunStat->ucDecGrdNum = 0;	
	
	pstMtrRunStat->ucAccGrdIdx = 0;
	pstMtrRunStat->ucDecGrdIdx = 0;

	pstMtrRunStat->ulAccPulse  = 0;
	pstMtrRunStat->ulDecPulse  = 0;
	
	// 运动距离为0,不需要运动
	if( pstMtrRunStat->MovePulse < 1 )
	{
		return DEF_TRUE;
	}   

	MaxPulse = 0;
	for( idx=0; idx<MOTOR_ACC_GRADE; idx++ )
	{
		if( pstAccStep->lFreq[idx] >= pstMtrRunStat->ulMaxFreq )
		{
			break;
		}

		// 减速与加速的台阶数相同
		MaxPulse += ((INT32U)pstAccStep->sPulse[idx] + pstDecStep->sPulse[idx]);
		if( MaxPulse > pstMtrRunStat->MovePulse )
		{
			break;
		}
	
		// 减速过程总脉冲数
		pstMtrRunStat->ulDecPulse  += pstMtrRunStat->pstDecPara->sPulse[idx];
		
		pstMtrRunStat->ucAccGrdNum += 1;
		pstMtrRunStat->ucDecGrdNum += 1;
	}

	// 台阶最大速度不能满足速度要求
	if( MOTOR_ACC_GRADE==idx )
	{
		return DEF_FALSE;
	}

	if( 0==pstMtrRunStat->ucAccGrdNum )
		pstMtrRunStat->ucMoveStat = RUN_STAT_AVG;
	else			
		pstMtrRunStat->ucMoveStat = RUN_STAT_ACC;	
	
#ifdef _DEBUG_
	sprintf(str, "[AG,%d,%d],", (INT16U)pstMtrRunStat->ucAccGrdNum, (INT16U)pstMtrRunStat->ucDecGrdNum);
	APP_TRACE_INFO( (str) );
#endif
	
	return DEF_TRUE;
}

/*******************************************************************************
 * @brief  电机加减速控制
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void MOTOR_FreqControl(BYTE MtrNo, P_RUN_STAT_PARA pstMtrRunStat)
{
	INT8U   tmp8;
	
	if( RUN_STAT_AVG==pstMtrRunStat->ucMoveStat )
	{
		if( (RUN_STAT_DONE!=pstMtrRunStat->ucMoveStat)&&(0==pstMtrRunStat->MovePulse) ) 
			pstMtrRunStat->ucMoveStat = RUN_STAT_DONE;
		return ;
	}
	
	/***********************************加速控制************************************/
	if( RUN_STAT_ACC==pstMtrRunStat->ucMoveStat )
	{ 	
		// 判断是否进入下一加速台阶
		tmp8 = pstMtrRunStat->ucAccGrdIdx;				

		if( pstMtrRunStat->PulseCount >= (pstMtrRunStat->ulAccPulse + pstMtrRunStat->pstAccPara->sPulse[tmp8]) )
		{	
			// 更新计数基准
			pstMtrRunStat->ulAccPulse += pstMtrRunStat->pstAccPara->sPulse[tmp8];
				
			// 更换频率
			tmp8 += 1;	 
			if( tmp8 >= pstMtrRunStat->ucAccGrdNum  ) //
			{
				if( pstMtrRunStat->ucAccGrdIdx>1 )
				{
					pstMtrRunStat->ucDecGrdIdx = pstMtrRunStat->ucAccGrdIdx - 1;
				}
				else
				{
					pstMtrRunStat->ucDecGrdIdx = 0;
				}

				// 进入匀速阶段
				pstMtrRunStat->ucMoveStat = RUN_STAT_MAX;
			}
			else
			{	
				pstMtrRunStat->ucAccGrdIdx += 1;

				// 更新频率
				TIM_SetMotorFreq( MtrNo, pstMtrRunStat->pstAccPara->lFreq[pstMtrRunStat->ucAccGrdIdx] );	 
			}
		}
	}
	/***********************************高速控制************************************/
	else if( RUN_STAT_MAX==pstMtrRunStat->ucMoveStat )
	{
		if( pstMtrRunStat->MovePulse <= pstMtrRunStat->ulDecPulse )
		{
			if( pstMtrRunStat->ulDecPulse >= pstMtrRunStat->pstAccPara->sPulse[pstMtrRunStat->ucDecGrdIdx] )
				pstMtrRunStat->ulDecPulse -= pstMtrRunStat->pstAccPara->sPulse[pstMtrRunStat->ucDecGrdIdx];
			else
				pstMtrRunStat->ulDecPulse  = 0;

			// 下一脉冲进入减速区间
			pstMtrRunStat->ucMoveStat = RUN_STAT_DEC;	
		}
		else
		{
			// 异常处理
		}			
	}
	/***********************************减速控制************************************/
	else if( RUN_STAT_DEC==pstMtrRunStat->ucMoveStat )
	{	
		if( pstMtrRunStat->MovePulse < pstMtrRunStat->pstDecPara->sPulse[pstMtrRunStat->ucDecGrdIdx] )
		{
			if( pstMtrRunStat->ucDecGrdIdx>0 )
			{
				pstMtrRunStat->ucDecGrdIdx -= 1;
			
				// 更换频率
				TIM_SetMotorFreq( MtrNo, pstMtrRunStat->pstDecPara->lFreq[pstMtrRunStat->ucDecGrdIdx] );
			}					
		}	
	}
}
