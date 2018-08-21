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
  
#ifndef MOTOR_DRV_H  
#define MOTOR_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.H"
#include "MotorBsp.H"

/* Macros ----------------------------------------------------------------------*/

//每秒钟最大速度转速	 
#define MOTOR_SPEED_MAX    80      //(8rps x 10 = 80)        
#define MOTOR_SPEED_MIN    2       //(0.2rps x 10 = 2)  
	 
//电机转速转成脉冲,参数分为是rps(x10), 细分和传动比	 
#define MOTOR_CYCLE_TO_PULSE(cyc, div)    ((cyc)*(div)*20ul)	 

//电机加速台阶数	 
#define MOTOR_ACC_GRADE    128	 
	 
#define SetMotorSpeed(r)   ((r)*10)	
	 
#define MAX_MTR_RUN_TIME_MS     20000ul 
#define MAX_MTR_PARA_TIME_MS    1000ul 	 
	 
/* Types    --------------------------------------------------------------------*/	 
	 
/* Structure definition---------------------------------------------------------*/
// 电机加减速台阶和步数	 
__packed
typedef struct _tagACCPARA_
{
	INT16U  sPulse[MOTOR_ACC_GRADE];		  // 台阶运动脉冲
	INT32U  lFreq[MOTOR_ACC_GRADE];			  // 台阶运动频率
}MTR_STEP_PARA, *P_MTR_STEP_PARA;

/////////////////////////
typedef  void (*FuncOnePara)(BYTE);
typedef  BYTE (*FuncReturn)(BYTE);

// 电机固定参数
__packed
typedef struct _tagCTRL_TYPE_
{
	INT8U  ucMtrCtrl;              // 
	INT32U MaxPulse;               // 最大行程脉冲数
	INT16U usDitoOrigPulse;        // 光藕到原点距离
	INT8U  Division;               // 细分
	INT16U DrvPPS;                 // 起跳速度(整步)
  	INT16U CurrErrorNo;            // 当前错误码
	
	FuncOnePara  fcMtrStop;
	FuncReturn   fcIsHomeValid;    // 光藕挡住	
	FuncReturn   fcIsLimitValid;   // 光藕挡住	
}CTRL_TYPE_STAT, *P_CTRL_TYPE_STAT;  

// 电机控制过程参数
__packed
typedef struct _tagRUN_STAT_
{
	INT8U  bOriginProc;           // 是否完成初始化操作 
	
	INT32U AxisPulse;             // 当前坐标脉冲
	INT32U MovePulse;             // 运动距离脉冲
	INT32U PulseCount;            // 运动脉冲计数
	INT32U MaxPulse;              // 允许发送的最大脉冲
	
	INT32U ulDrvFreq;             // 准备起动速度
	INT32U ulMaxFreq;             // 准备最大速度
	INT8U  bHomePrevStat;         // 光藕状态 	
		
	INT8U  ucAccGrdNum;			  // 当前距离对应的加速台阶级数
	INT8U  ucAccGrdIdx;
	INT8U  ucDecGrdNum;			  // 当前距离对应的减速台阶级数
	INT8U  ucDecGrdIdx;	
	INT32U ulAccPulse;            // 加速总台阶数据
	INT32U ulDecPulse;            // 减速总台阶数据
		
	P_MTR_STEP_PARA pstAccPara;    // 加速控制参数
	P_MTR_STEP_PARA pstDecPara;    // 加速控制参数
	INT8U  ucMoveDir;             // 运动方向: 向光藕, 离光藕
	INT8U  ucMoveStat;            // 运动状态: 匀速, 加速, 高速, 减速	
	
	INT32U lCurrTick;
}RUN_STAT_PARA, *P_RUN_STAT_PARA;   //MtrRunPara, *pstMtrRunStat;
///////////////////////////

// 电机固定参数
__packed
typedef struct _tagPOS_VALUE_
{
	u16 sDatLen;
	u16 sCRC;
	u32 PosZero;                      // 0位
	u32 PosTide;                      // 压紧位
}MTR_POS_PARA, *P_MTR_POS_PARA;  

// 电机控制类型
typedef enum 
{
	CTRL_TYPE_IDLE            = 0,       // 不运动	
	CTRL_TYPE_INIT       	     ,       // 初始化, 如果挡住光藕,先退出,再回原点
	CTRL_TYPE_TO_ZERO    	     ,       // 直接运行到0点 
	CTRL_TYPE_MOVE_ABS      	 ,       // 运动到指定位置
	CTRL_TYPE_MOVE_REL      	 ,       // 运动相对距离	
}emMtrCtrl;	

// 电机控制状态
typedef enum 
{
	RUN_STAT_WAIT             = 0,       // 空闲状态   
	RUN_STAT_AVG                 ,       // 匀速运动
	RUN_STAT_ACC	             ,       // 加速运动
	RUN_STAT_MAX	             ,       // 高速运动
	RUN_STAT_DEC	             ,       // 减速运动
	RUN_STAT_DONE                ,       // 完成		
	RUN_STAT_ERROR               ,       // 异常	
}emMtrRunStat;	


// Motor 1
#define  M1_DIVISION                    8 	         // 细分 
#define  M1_DRV_PPS                     100          // 起跳整步频率 
#define  M1_MAX_PPS                     1500         // 最大整步频率 
#define  M1_PULSE_ORIGIN                (M1_DIVISION*200)	         // 光藕到机械原点的距离 
#define  M1_MAX_RUN_PULSE               (M1_DIVISION*200*62)

// Motor 2
#define  M2_DIVISION                    8 	         // 细分 
#define  M2_DRV_PPS                     100          // 起跳整步频率 
#define  M2_MAX_PPS                     1500         // 最大整步频率 
#define  M2_PULSE_ORIGIN                (M2_DIVISION*200)	         // 光藕到机械原点的距离 
#define  M2_MAX_RUN_PULSE               (M2_DIVISION*200*62)


/* Functions -------------------------------------------------------------------*/
BYTE MOTOR_ParameterInit(BYTE nMtrNo);
void MOTOR_PulseUpdate(BYTE nMtrNo, BYTE bPulseLow);
BYTE MOTOR_GetDivision(BYTE nMtrNo);
BYTE MOTOR_GetCurrCtrlType(BYTE nMtrNo);
BYTE MOTOR_GetRunStatus(BYTE MtrNo);
u16  MOTOR_GetAxisPulse(BYTE MtrNo);
BYTE MOTOR_SetRunPara(BYTE MtrNo, INT32U MovePulse, BYTE MoveDir, BYTE nCtrlType, BYTE MaxRpsX10);
void MOTOR_SetCurrError(u8 MtrNo, u8 Stat);

u8  MOTOR_Home(u8 MtrNo, u8 MaxRpsX10);          
u8  MOTOR_MoveAbsolute(u8 MtrNo, u32 MovePulse, u8 MaxRpsX10);           
u8  MOTOR_MoveRelative(u8 MtrNo, u32 MovePulse, u8 MoveDir, u8 MaxRpsX10);
u16 MOTOR_GetCurrError(u8 MtrNo);

void MOTOR_SetCurrStat(u8 MtrNo, u8 stat);
u8   MOTOR_GetCurrStat(u8 MtrNo);

u32 MOTOR_GetPosPara(void);
u32 MOTOR_SavePosPara(u8 ParaBuff[], u16 DatLength, u8 bWriteToFlash);

#ifdef __cplusplus
}
#endif

#endif

extern MTR_POS_PARA  stMtrPosPara[MOTOR_MAX];





