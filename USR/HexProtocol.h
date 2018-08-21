/*******************************************************************************
 * @file    HexProtocol.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-23
 * @brief   与上层控制体的通信接口定义 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef PROTOCOL_HEX_H  
#define PROTOCOL_HEX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"

/* Macros ----------------------------------------------------------------------*/


// 本机地址	 
#ifndef LOCAL_ADDR
 #define  LOCAL_ADDR          0x10	 
#endif
#ifndef TARGET_ADDR
 #define  TARGET_ADDR         0x01	 
#endif	 
	 
#define  SYSTEM_VERSION      "V1.0.0"
#define  PROTOCOL_VERSION    "V1.0"
#define  PCBA_VERSION        "P00001"

#define  GetLocalAddr()       LOCAL_ADDR
#define  IsSendToMe(x)        ((x)==LOCAL_ADDR)?1:0
#define  MsToTcks(x)          (x)

// 帧头
#define  FRM_HEADER          0xAA55

// 帧类型
#define  FRM_TYPE_1          0x01
#define  FRM_TYPE_2          0x04
#define  FRM_TYPE_ACK        0x02
#define  FRM_TYPE_RES        0x03

// Timing definition
#define TIMING_TYPE_PRIME    0          // prime timing
#define TIMING_TYPE_COVER    1          // cover and get press value timing
#define TIMING_TYPE_LOADING  2          // load timing
#define TIMING_TYPE_CLEAN    3

#define PRIME_TIMING_STEP    10
#define CLEAN_TIMING_STEP    9
#define LOAD_TIMING_STEP     11

/* Types    ---------------------------------------------------------------------*/
// 上位机通信数据格式
__packed
typedef union _tagFRM_HEADER
{
	__packed
	struct _tagProcHead_
	{
		INT16U  Header;                  // 接收字节当前编号	
		INT16U  Length;
		INT8U   Type;                    // 类型, 1-需要应答和结果帧,2-需要应答;5-应答帧,6-结果帧	
		INT16U  Index;
		INT8U   Addr;                    // 目标地址		
		INT8U   Order;                   // 命令
	}Struct;
	BYTE ByteBuf[9];
}HEADER;

// 通信指令帧格式
__packed
typedef struct _tagUpProtocol_
{
	HEADER  Header;
	BYTE    DatBuf[130];	         // 数据后面跟ccrc2, NOR(DatBuf[0]+DatBuf[1]+...+DatBuf[N])=ccrc2
}FRM_PROC, *P_FRM_PROC; 

#define FRM_SIZE   sizeof(FRM_PROC)
	
// 时序指令
__packed
//typedef struct _Timing
//{
//	u8   StepIndex;
////	u8   ReagName[10];
//	u8   RotValvePos;
//	u16  SyrAsprSpeed;	           // aspirate speed
//	u16  SyrAsprVolume;	           // aspirate volume
//	u16  DelaySecond;              // delay time (second)
//	u16  SyrDrainSpeed;	           // drain speed
//	u16  SyrDrainVolume;	       // drain volume
//}TIMING_STEP, *P_TIMING_STEP; 

typedef struct _TeeValve
{
	u8   stat;
	u16  T100ms;
}TV_CTRL; 

typedef struct _Timing
{
	u8   StepIndex;
	u8   ReagName[10];
	u8   RotValvePos;
	
	TV_CTRL stTeeValAspr[3];   
	u16  DelayForValve1;           // delay time (mil second)
	
	u16  SyrAsprSpeed;	           // aspirate speed
	u16  SyrAsprVolume;	           // aspirate volume
	u16  DelaySecond;              // delay time (second)
	
	TV_CTRL stTeeValDrain[3];   
	u16  DelayForValve2;           // delay time (mil second)
	
	u16  SyrDrainSpeed;	           // drain speed
	u16  SyrDrainVolume;	       // drain volume
}TIMING_STEP, *P_TIMING_STEP; 


/* Structure definition-------------------------------------------------------*/

//UpLayer protocol
typedef enum
{
	QUERY_SOFT_VERSION       = 0x01,     //查询版本
	QUERY_HARD_VERSION       = 0x02,     //查询板卡信息
	SET_HARD_VERSION         = 0x03,     //设置板卡信息
	UPDATE_SYSTEM                  ,
	
	SET_PARA                 = 0x05,     //设置参数
	QUERY_PARA               = 0x06,     //查询参数
	
	RO_VALVE_PASS            = 0x10,     //旋转阀透传
	RO_VALVE_CTRL            = 0x11,     //旋转阀指令
	RO_VALVE_QUERY           = 0x12,     //查询旋转阀状态
	
	SYR_PUMP_PASS            = 0x16,     //注射泵透传
	SYR_PUMP_INIT            = 0x17,     //注射泵复位
	SYR_PUMP_IN              = 0x18,     //注射泵吸液
	SYR_PUMP_OUT             = 0x19,     //注射泵排液
	
	COVER_MTR_INIT           = 0x20,     //加盖电机复位,    最高速度
	COVER_MTR_ABS_DIST       = 0x21,     //顶盖电机绝对运动,最高速度+脉冲
	COVER_MTR_REL_DIST       = 0x22,     //顶盖电机绝对运动,最高速度+指定脉冲+方向
	COVER_MTR_POSITION       = 0x23,     //顶盖电机指定位置,最高速度+位置编号
	  MTR_POS_ZERO           = 0,
	  MTR_POS_TIDE           = 1,
	TEMP_CTRL_ON             = 0x30,     //温度控制开, 目标温度(x100)+校准温度(x100)	
	TEMP_CTRL_OFF            = 0x31,     //温度控制关
	TEMP_CTRL_QUERY          = 0x32,     //温度查询
	PRESS_QUERY              = 0x33,     //压力查询
	
	PUMP_CTRL                = 0x38,     //泵控制, 泵编号, 1-打开/0-关闭 打开时间
	PUMP_STAT_QUERY          = 0x39,     //泵状态查询, 泵编号
	
	VALVE_CTRL               = 0x3C,     //阀控制, 阀编号, 1-打开/0-关闭 打开时间
	VALVE_STAT_QUERY         = 0x3D,     //阀状态查询, 阀编号
	  VALVE_NO_1             = 1,
	  VALVE_NO_2             = 2,
	  VALVE_NO_3             = 3,
	  VALVE_NO_M             = 3,
	  
	QUERY_DIODE_STAT         = 0x40,     //查询光藕电平, 光藕编号  	
	
	SYSTEM_DELAY_MS          = 0x50,     //延时  
	
	FLOW_PRIME_DOWNLOAD      = 0x60,     //下载灌注时序表
	FLOW_PRIME_QUERY         = 0x61,     //查询灌注时序表
	FLOW_PRIME_START         = 0x62,     //开始灌注流程
	FLOW_PRIME_PAUSE         = 0x63,     //暂停灌注流程
	FLOW_PRIME_STOP          = 0x64,     //停止灌注流程
	FLOW_PRIME_STAT_QUERY    = 0x65,     //查询灌注状态
	
	FLOW_CLEAN_DOWNLOAD      = 0x68,     //下载灌注时序表
	FLOW_CLEAN_QUERY         = 0x69,     //查询灌注时序表
	FLOW_CLEAN_START         = 0x6A,     //开始灌注流程
	FLOW_CLEAN_PAUSE         = 0x6B,     //暂停灌注流程
	FLOW_CLEAN_STOP          = 0x6C,     //停止灌注流程
	FLOW_CLEAN_STAT_QUERY    = 0x6D,     //查询灌注状态
	
	FLOW_LOAD_DOWNLOAD       = 0x70,     //下载Loading时序表
	FLOW_LOAD_QUERY          = 0x71,     //查询Loading时序表
	FLOW_LOAD_START          = 0x72,     //开始Loading流程
	FLOW_LOAD_PAUSE          = 0x73,     //暂停Loading流程
	FLOW_LOAD_STOP           = 0x74,     //停止Loading流程
	FLOW_LOAD_STAT_QUERY     = 0x75,     //查询Loading状态
	
	FLOW_COVER_DOWNLOAD      = 0x78,     //下载气密性检测时序表
	FLOW_COVER_QUERY         = 0x79,     //查询气密性检测时序表
	FLOW_COVER_START         = 0x7A,     //开始气密性检测流程
	FLOW_COVER_PAUSE         = 0x7B,     //暂停气密性检测流程
	FLOW_COVER_STOP          = 0x7C,     //停止气密性检测流程
	FLOW_COVER_STAT_QUERY    = 0x7D,     //查询气密性检测状态
}HEX_PROTOCOL;

// 帧返回状态位
typedef enum
{
	FRM_RECV_NONE            = 0,
	FRM_RECV_HEADER_GOOD        ,
	FRM_RECV_LEN_SIZE_GOOD      ,
	FRM_RECV_LEN_GOOD           ,
	FRM_RECV_CRC_GOOD           ,
	FRM_RECV_HEADER_ERROR       ,
	FRM_RECV_LEN_ERROR          ,
	FRM_RECV_CRC_ERROR          ,

	FRM_RECV_NON_ORDER       = 0x80,
	FRM_RECV_PARA_LEN_ERR    = 0x81,  //指令长度参数
	FRM_RECV_PARA_CON_ERR    = 0x82,	
	
	FRM_EXEC_SUCCESS         = 0x00,	
	
	FRM_EXEC_FORBID_PRIMING  = 0xF0,  //执行灌注,禁止执行
	FRM_EXEC_FORBID_CLEANING = 0xF1,  //执行清洗,禁止执行
	FRM_EXEC_FORBID_TESTING  = 0xF2,  //执行测试,禁止执行
	
	FRM_EXEC_MODULE_RUNNING  = 0xFF,  //当前模块运行中,禁止执行
	
	// 响应帧返回的状态位
	FRM_ACK_OK               = 0x00,
	FRM_ACK_ERR_LEN          = 0x01,   //长度错误
	FRM_ACK_ERR_CRC          = 0x02,   //校验码错误
	FRM_ACK_ERR_ADDR         = 0x03,   //地址错误
	FRM_ACK_REPEAT           = 0x04,   //重复帧错误
}FRM_STAT;

// 帧返回状态位
typedef enum
{
	SYS_CURR_OP_IDLE         = 0,      //
	SYS_CURR_OP_PRIMING      = 1,
	SYS_CURR_OP_CLEANING     = 2,
	SYS_CURR_OP_TESTING      = 3,
}emSysCurrOper;


/* Functions -------------------------------------------------------------------*/
u16  CRC_GetValue(BYTE *Buff, u16 Len);

void SetSysCurrOperation(u8 stat);
u8 GetSysCurrOperation(void);

void InitRecvFramePara(void);
u8   HEXPR_FrameDecode( u8 *pRecvBuff, u16 DatLen );
void HEX_RoValveTask( u8 *pRecvBuff );
void HEX_SyrPumpTask( u8 *pRecvBuff );
void HEX_MotorCtrlTask( u8 *pRecvBuff );
void ResultUpLayerComm( u16 Index, u8 Order, u8 dat[], u16 len );
u16  OrderMakeToFrame( const u8 *pSrcBuf, u16 SrcLen, u8 *pDstBuf );

u16 TimingToCommand(u8 *pDatBuf, u8 TimingType);
#ifdef __cplusplus
}
#endif

#endif


