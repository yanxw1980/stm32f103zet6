/*******************************************************************************
 * @file    MotorDrv.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-23
 * @brief   �������˶������ӿڶ��� 
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

//ÿ��������ٶ�ת��	 
#define MOTOR_SPEED_MAX    80      //(8rps x 10 = 80)        
#define MOTOR_SPEED_MIN    2       //(0.2rps x 10 = 2)  
	 
//���ת��ת������,������Ϊ��rps(x10), ϸ�ֺʹ�����	 
#define MOTOR_CYCLE_TO_PULSE(cyc, div)    ((cyc)*(div)*20ul)	 

//�������̨����	 
#define MOTOR_ACC_GRADE    128	 
	 
#define SetMotorSpeed(r)   ((r)*10)	
	 
#define MAX_MTR_RUN_TIME_MS     20000ul 
#define MAX_MTR_PARA_TIME_MS    1000ul 	 
	 
/* Types    --------------------------------------------------------------------*/	 
	 
/* Structure definition---------------------------------------------------------*/
// ����Ӽ���̨�׺Ͳ���	 
__packed
typedef struct _tagACCPARA_
{
	INT16U  sPulse[MOTOR_ACC_GRADE];		  // ̨���˶�����
	INT32U  lFreq[MOTOR_ACC_GRADE];			  // ̨���˶�Ƶ��
}MTR_STEP_PARA, *P_MTR_STEP_PARA;

/////////////////////////
typedef  void (*FuncOnePara)(BYTE);
typedef  BYTE (*FuncReturn)(BYTE);

// ����̶�����
__packed
typedef struct _tagCTRL_TYPE_
{
	INT8U  ucMtrCtrl;              // 
	INT32U MaxPulse;               // ����г�������
	INT16U usDitoOrigPulse;        // ��ź��ԭ�����
	INT8U  Division;               // ϸ��
	INT16U DrvPPS;                 // �����ٶ�(����)
  	INT16U CurrErrorNo;            // ��ǰ������
	
	FuncOnePara  fcMtrStop;
	FuncReturn   fcIsHomeValid;    // ��ź��ס	
	FuncReturn   fcIsLimitValid;   // ��ź��ס	
}CTRL_TYPE_STAT, *P_CTRL_TYPE_STAT;  

// ������ƹ��̲���
__packed
typedef struct _tagRUN_STAT_
{
	INT8U  bOriginProc;           // �Ƿ���ɳ�ʼ������ 
	
	INT32U AxisPulse;             // ��ǰ��������
	INT32U MovePulse;             // �˶���������
	INT32U PulseCount;            // �˶��������
	INT32U MaxPulse;              // �����͵��������
	
	INT32U ulDrvFreq;             // ׼�����ٶ�
	INT32U ulMaxFreq;             // ׼������ٶ�
	INT8U  bHomePrevStat;         // ��ź״̬ 	
		
	INT8U  ucAccGrdNum;			  // ��ǰ�����Ӧ�ļ���̨�׼���
	INT8U  ucAccGrdIdx;
	INT8U  ucDecGrdNum;			  // ��ǰ�����Ӧ�ļ���̨�׼���
	INT8U  ucDecGrdIdx;	
	INT32U ulAccPulse;            // ������̨������
	INT32U ulDecPulse;            // ������̨������
		
	P_MTR_STEP_PARA pstAccPara;    // ���ٿ��Ʋ���
	P_MTR_STEP_PARA pstDecPara;    // ���ٿ��Ʋ���
	INT8U  ucMoveDir;             // �˶�����: ���ź, ���ź
	INT8U  ucMoveStat;            // �˶�״̬: ����, ����, ����, ����	
	
	INT32U lCurrTick;
}RUN_STAT_PARA, *P_RUN_STAT_PARA;   //MtrRunPara, *pstMtrRunStat;
///////////////////////////

// ����̶�����
__packed
typedef struct _tagPOS_VALUE_
{
	u16 sDatLen;
	u16 sCRC;
	u32 PosZero;                      // 0λ
	u32 PosTide;                      // ѹ��λ
}MTR_POS_PARA, *P_MTR_POS_PARA;  

// �����������
typedef enum 
{
	CTRL_TYPE_IDLE            = 0,       // ���˶�	
	CTRL_TYPE_INIT       	     ,       // ��ʼ��, �����ס��ź,���˳�,�ٻ�ԭ��
	CTRL_TYPE_TO_ZERO    	     ,       // ֱ�����е�0�� 
	CTRL_TYPE_MOVE_ABS      	 ,       // �˶���ָ��λ��
	CTRL_TYPE_MOVE_REL      	 ,       // �˶���Ծ���	
}emMtrCtrl;	

// �������״̬
typedef enum 
{
	RUN_STAT_WAIT             = 0,       // ����״̬   
	RUN_STAT_AVG                 ,       // �����˶�
	RUN_STAT_ACC	             ,       // �����˶�
	RUN_STAT_MAX	             ,       // �����˶�
	RUN_STAT_DEC	             ,       // �����˶�
	RUN_STAT_DONE                ,       // ���		
	RUN_STAT_ERROR               ,       // �쳣	
}emMtrRunStat;	


// Motor 1
#define  M1_DIVISION                    8 	         // ϸ�� 
#define  M1_DRV_PPS                     100          // ��������Ƶ�� 
#define  M1_MAX_PPS                     1500         // �������Ƶ�� 
#define  M1_PULSE_ORIGIN                (M1_DIVISION*200)	         // ��ź����еԭ��ľ��� 
#define  M1_MAX_RUN_PULSE               (M1_DIVISION*200*62)

// Motor 2
#define  M2_DIVISION                    8 	         // ϸ�� 
#define  M2_DRV_PPS                     100          // ��������Ƶ�� 
#define  M2_MAX_PPS                     1500         // �������Ƶ�� 
#define  M2_PULSE_ORIGIN                (M2_DIVISION*200)	         // ��ź����еԭ��ľ��� 
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





