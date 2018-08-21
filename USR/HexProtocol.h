/*******************************************************************************
 * @file    HexProtocol.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-23
 * @brief   ���ϲ�������ͨ�Žӿڶ��� 
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


// ������ַ	 
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

// ֡ͷ
#define  FRM_HEADER          0xAA55

// ֡����
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
// ��λ��ͨ�����ݸ�ʽ
__packed
typedef union _tagFRM_HEADER
{
	__packed
	struct _tagProcHead_
	{
		INT16U  Header;                  // �����ֽڵ�ǰ���	
		INT16U  Length;
		INT8U   Type;                    // ����, 1-��ҪӦ��ͽ��֡,2-��ҪӦ��;5-Ӧ��֡,6-���֡	
		INT16U  Index;
		INT8U   Addr;                    // Ŀ���ַ		
		INT8U   Order;                   // ����
	}Struct;
	BYTE ByteBuf[9];
}HEADER;

// ͨ��ָ��֡��ʽ
__packed
typedef struct _tagUpProtocol_
{
	HEADER  Header;
	BYTE    DatBuf[130];	         // ���ݺ����ccrc2, NOR(DatBuf[0]+DatBuf[1]+...+DatBuf[N])=ccrc2
}FRM_PROC, *P_FRM_PROC; 

#define FRM_SIZE   sizeof(FRM_PROC)
	
// ʱ��ָ��
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
	QUERY_SOFT_VERSION       = 0x01,     //��ѯ�汾
	QUERY_HARD_VERSION       = 0x02,     //��ѯ�忨��Ϣ
	SET_HARD_VERSION         = 0x03,     //���ð忨��Ϣ
	UPDATE_SYSTEM                  ,
	
	SET_PARA                 = 0x05,     //���ò���
	QUERY_PARA               = 0x06,     //��ѯ����
	
	RO_VALVE_PASS            = 0x10,     //��ת��͸��
	RO_VALVE_CTRL            = 0x11,     //��ת��ָ��
	RO_VALVE_QUERY           = 0x12,     //��ѯ��ת��״̬
	
	SYR_PUMP_PASS            = 0x16,     //ע���͸��
	SYR_PUMP_INIT            = 0x17,     //ע��ø�λ
	SYR_PUMP_IN              = 0x18,     //ע�����Һ
	SYR_PUMP_OUT             = 0x19,     //ע�����Һ
	
	COVER_MTR_INIT           = 0x20,     //�Ӹǵ����λ,    ����ٶ�
	COVER_MTR_ABS_DIST       = 0x21,     //���ǵ�������˶�,����ٶ�+����
	COVER_MTR_REL_DIST       = 0x22,     //���ǵ�������˶�,����ٶ�+ָ������+����
	COVER_MTR_POSITION       = 0x23,     //���ǵ��ָ��λ��,����ٶ�+λ�ñ��
	  MTR_POS_ZERO           = 0,
	  MTR_POS_TIDE           = 1,
	TEMP_CTRL_ON             = 0x30,     //�¶ȿ��ƿ�, Ŀ���¶�(x100)+У׼�¶�(x100)	
	TEMP_CTRL_OFF            = 0x31,     //�¶ȿ��ƹ�
	TEMP_CTRL_QUERY          = 0x32,     //�¶Ȳ�ѯ
	PRESS_QUERY              = 0x33,     //ѹ����ѯ
	
	PUMP_CTRL                = 0x38,     //�ÿ���, �ñ��, 1-��/0-�ر� ��ʱ��
	PUMP_STAT_QUERY          = 0x39,     //��״̬��ѯ, �ñ��
	
	VALVE_CTRL               = 0x3C,     //������, �����, 1-��/0-�ر� ��ʱ��
	VALVE_STAT_QUERY         = 0x3D,     //��״̬��ѯ, �����
	  VALVE_NO_1             = 1,
	  VALVE_NO_2             = 2,
	  VALVE_NO_3             = 3,
	  VALVE_NO_M             = 3,
	  
	QUERY_DIODE_STAT         = 0x40,     //��ѯ��ź��ƽ, ��ź���  	
	
	SYSTEM_DELAY_MS          = 0x50,     //��ʱ  
	
	FLOW_PRIME_DOWNLOAD      = 0x60,     //���ع�עʱ���
	FLOW_PRIME_QUERY         = 0x61,     //��ѯ��עʱ���
	FLOW_PRIME_START         = 0x62,     //��ʼ��ע����
	FLOW_PRIME_PAUSE         = 0x63,     //��ͣ��ע����
	FLOW_PRIME_STOP          = 0x64,     //ֹͣ��ע����
	FLOW_PRIME_STAT_QUERY    = 0x65,     //��ѯ��ע״̬
	
	FLOW_CLEAN_DOWNLOAD      = 0x68,     //���ع�עʱ���
	FLOW_CLEAN_QUERY         = 0x69,     //��ѯ��עʱ���
	FLOW_CLEAN_START         = 0x6A,     //��ʼ��ע����
	FLOW_CLEAN_PAUSE         = 0x6B,     //��ͣ��ע����
	FLOW_CLEAN_STOP          = 0x6C,     //ֹͣ��ע����
	FLOW_CLEAN_STAT_QUERY    = 0x6D,     //��ѯ��ע״̬
	
	FLOW_LOAD_DOWNLOAD       = 0x70,     //����Loadingʱ���
	FLOW_LOAD_QUERY          = 0x71,     //��ѯLoadingʱ���
	FLOW_LOAD_START          = 0x72,     //��ʼLoading����
	FLOW_LOAD_PAUSE          = 0x73,     //��ͣLoading����
	FLOW_LOAD_STOP           = 0x74,     //ֹͣLoading����
	FLOW_LOAD_STAT_QUERY     = 0x75,     //��ѯLoading״̬
	
	FLOW_COVER_DOWNLOAD      = 0x78,     //���������Լ��ʱ���
	FLOW_COVER_QUERY         = 0x79,     //��ѯ�����Լ��ʱ���
	FLOW_COVER_START         = 0x7A,     //��ʼ�����Լ������
	FLOW_COVER_PAUSE         = 0x7B,     //��ͣ�����Լ������
	FLOW_COVER_STOP          = 0x7C,     //ֹͣ�����Լ������
	FLOW_COVER_STAT_QUERY    = 0x7D,     //��ѯ�����Լ��״̬
}HEX_PROTOCOL;

// ֡����״̬λ
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
	FRM_RECV_PARA_LEN_ERR    = 0x81,  //ָ��Ȳ���
	FRM_RECV_PARA_CON_ERR    = 0x82,	
	
	FRM_EXEC_SUCCESS         = 0x00,	
	
	FRM_EXEC_FORBID_PRIMING  = 0xF0,  //ִ�й�ע,��ִֹ��
	FRM_EXEC_FORBID_CLEANING = 0xF1,  //ִ����ϴ,��ִֹ��
	FRM_EXEC_FORBID_TESTING  = 0xF2,  //ִ�в���,��ִֹ��
	
	FRM_EXEC_MODULE_RUNNING  = 0xFF,  //��ǰģ��������,��ִֹ��
	
	// ��Ӧ֡���ص�״̬λ
	FRM_ACK_OK               = 0x00,
	FRM_ACK_ERR_LEN          = 0x01,   //���ȴ���
	FRM_ACK_ERR_CRC          = 0x02,   //У�������
	FRM_ACK_ERR_ADDR         = 0x03,   //��ַ����
	FRM_ACK_REPEAT           = 0x04,   //�ظ�֡����
}FRM_STAT;

// ֡����״̬λ
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


