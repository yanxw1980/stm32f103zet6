/*******************************************************************************
 * @file    Protocol.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-08-23
 * @brief   与上层控制体的通信接口定义 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef PROTOCOL_H  
#define PROTOCOL_H

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

#define IsSendToMe(x)        ((x)==LOCAL_ADDR)?1:0


/* Types    ---------------------------------------------------------------------*/

// 上位机通信数据格式
__packed
typedef union _tagModbus
{
	__packed
	struct _tagModbusdef_
	{
		BYTE  Addr;                      // 本地地址
		BYTE  Order;                     // 功能码
		u8    Data[254];
	}Struct;
	u8    Data[256];
}MODBUS, *P_MODBUS;
#define MODBUS_SIZE   sizeof(MODBUS)
	
/* Structure definition-------------------------------------------------------*/

// Modbus
typedef enum
{
	READ_COIL_STATUS         = 0x01,     //
	INPUT_STATUS             = 0x02,     //
	READ_MORE_REGISTERS      = 0x03,     //
	READ_INPUT_REGISTER      = 0x04,
	
	WRITE_SINGLE_COIL        = 0x05,
	WRITE_SINGLE_REGISTER    = 0x06,
	WRITE_MORE_COILS         = 0x0F,
	WRITE_MORE_REGISTERS     = 0x10,
	
	REPORT_SLAVE_ID          = 0x11,
	
	MASK_WRITE_REGISTER      = 0x16,
	READ_WRITE_REGISTERS     = 0x17,
	
	READ_FILE_RECORD         = 0x14,
	WRITE_FILE_RECORD        = 0x15,
	
	READ_DEVICE_SN           = 0x2B,     // 读设备识别码
}UpLayer_PROTOCOL;

// Modbus
typedef enum
{
	MODBUS_OK                = 0x00,     // 
	MODBUS_ERR_FUNC_CODE     = 0x01,     //非法功能
	MODBUS_ERR_ADDRESS       = 0x02,     //非法数据地址
	MODBUS_ERR_VALUE         = 0x03,	 //非法数据值
	MODBUS_ERR_SLAVE_DEVICE  = 0x04,     //从站设备故障 
	MODBUS_ERR_CONFIRM       = 0x05,     //确认
	MODBUS_ERR_BUSY          = 0x06,     //从属设备忙
	MODBUS_ERR_STORE_PARITY  = 0x08,     //存储奇偶性差错
	MODBUS_ERR_NET_GATE      = 0x0A,     //不可用网关路径
	MODBUS_ERR_TARGET_ACK    = 0x0B,     //网关目标设备响应失败 
	
	R_VALVE_EXEC_TIMEOUT     = 0xF0,     // 旋转阀执行超时
	R_VALVE_COMMUNICATION    = 0xF1,     // 旋转阀返回指令不对
}MODBUS_ERROR;

#define COIL_CTRL_ON         0xFF00
#define COIL_CTRL_OFF        0x0000

// COIL address definition
#define COIL_ADDR_VALVE_1    0x0000
#define COIL_ADDR_VALVE_2    0x0001
#define COIL_ADDR_VALVE_3    0x0002

#define COIL_ADDR_OPTO_1     0x0010
#define COIL_ADDR_OPTO_2     0x0011
#define COIL_ADDR_OPTO_3     0x0012

#define COIL_ADDR_FAN        0x0020

// rorate valve address definition
#define RV_REG_STATUS        0x0140
#define RV_REG_CURR_POS      0x0141
#define RV_REG_COMMAND       0x0142
#define RV_REG_BAUDRATE      0x0143

#define RV_REG_MIN           RV_REG_STATUS
#define RV_REG_MAX           RV_REG_BAUDRATE

// Register address definition
#define REG_ADDR_RR_VALVE    0x1000      //透传旋转阀指令
#define REG_ADDR_SYRINGE     0x1100      //透传注射泵指令


/* Functions -------------------------------------------------------------------*/
BYTE ModbusProtocol( u8 *pDatBuff, u8 FrameLen );
void MODBUS_RoValveTask( u8 *pRecvBuff );
#ifdef __cplusplus
}
#endif

#endif


