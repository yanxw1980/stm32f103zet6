/*******************************************************************************
 * @file    Protocol.c    
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

/* Includes --------------------------------------------------------------------*/ 
#include <string.h>
#include <stdio.h>
#include "BoardIoDef.h"
#include "ModbusProl.h"
#include "Bsp.h"
#include "MotorBsp.h"
#include "RotValveDef.h"

#ifdef PR_MODBUS

/* Private typedef -----------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void ModbusAckNormal(u8 Addr, u8 Order, BYTE dat[], INT16U len);
static void ModbusAckError(u8 Addr, u8 Order, u8 ErrNo);

/* Private functions ---------------------------------------------------------*/ 
extern unsigned int GetCRC16(unsigned char *ptr,  unsigned char len);

/*******************************************************************************
 * @brief  异常返回
 * @input  Index,     帧编号
 *         
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void ModbusAckError(u8 Addr, u8 Order, u8 ErrNo)
{
	u8   buff[256];
	u16  crc;
	u16  DatLen;

	DatLen = 0;
	
	buff[0] = Addr;
	
	buff[1] = 0x80 | Order;	
	
	buff[2] = ErrNo;

	DatLen = 2 + 1;
	
	// ccrc
	crc = GetCRC16(buff, DatLen); 
	
	buff[DatLen+0] = crc>>8;
	buff[DatLen+1] = crc;	
	
	BSP_Ser_WrHex (UART_MAIN, (CPU_CHAR *)buff, DatLen+2);
}

/*******************************************************************************
 * @brief  正常返回 
 * @input  Index,     帧编号
 *         
 * @return  
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void ModbusAckNormal(u8 Addr, u8 Order, BYTE dat[], INT16U len)
{
	u8   buff[256];
	u16  crc;
	u16  DatLen;

	DatLen = 0;
	
	buff[0] = Addr;
	
	buff[1] = Order;
	
	memcpy(buff+2, dat, len);
	
	DatLen = 2 + len;
	
	// ccrc
	crc = GetCRC16(buff, DatLen); 
	
	buff[DatLen+0] = crc>>8;
	buff[DatLen+1] = crc;
	
	BSP_Ser_WrHex (UART_MAIN, (CPU_CHAR *)buff, DatLen+2);	
}

/*******************************************************************************
 * @brief  上位机指令 
 * @input  
 * @return 0-正常, 其它异常 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE ModbusProtocol( u8 *pDatBuff, u8 FrameLen )
{
	P_MODBUS  pModbusBuff;
	OS_ERR    err;
	u8  aSndBuff[30];
	u8  *pFrameBuff;
	u8  SndBuffIdx;
	u8  bStat;
	u16 BaseAddr;
	u16 DataLen;
	u8  ByteNum;
	
	
	if( NULL==pDatBuff )
		return 0;
	
	pModbusBuff = (P_MODBUS)pDatBuff;
	
	bStat = MODBUS_OK;
	
	aSndBuff[0] = 0x00;	
	SndBuffIdx  = 1;
	switch( pModbusBuff->Struct.Order )
	{		
		case READ_COIL_STATUS:				
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get coil number
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			// 需要考虑不是从1开始查询的情况
			if( COIL_ADDR_VALVE_1==BaseAddr )        
			{
				if( DataLen != 3 )
				{
					bStat = MODBUS_ERR_VALUE;          
				}
				else
				{
					aSndBuff[0] = 0x01;
					aSndBuff[1] = 0x00;
					SndBuffIdx  = 2;
					
					if( IsValve1On() )
						aSndBuff[1] |= (0x01<<0);
					if( IsValve2On() )
						aSndBuff[1] |= (0x01<<1);
					if( IsValve3On() )
						aSndBuff[1] |= (0x01<<2);
				}
			}
			else if( COIL_ADDR_OPTO_1==BaseAddr )   
			{
				if( DataLen != 4 )
				{
					bStat = MODBUS_ERR_VALUE;          
				}
				else
				{
					aSndBuff[0] = 0x01;
					aSndBuff[1] = 0x00;
					SndBuffIdx  = 2;
					
					if( IsMtr1HomeValid() )
						aSndBuff[1] |= (0x01<<0);
					if( IsMtr1LimitValid() )
						aSndBuff[1] |= (0x01<<1);
				}
			}
			else if( COIL_ADDR_FAN==BaseAddr )    // fan
			{
				if( DataLen != 1 )
				{
					bStat = MODBUS_ERR_VALUE;         
				}
				else
				{
					aSndBuff[0] = 0x01;
					aSndBuff[1] = 0x00;
					SndBuffIdx  = 2;
					
					if( IsFanOn() )
						aSndBuff[1] |= (0x01<<0);
				}
			}
			else                           // error
			{
				bStat = MODBUS_ERR_ADDRESS;              
			}
			
			break;
		}
		case INPUT_STATUS:
		{
			break;
		}
		case READ_MORE_REGISTERS:
		{
			break;
		}
		case READ_INPUT_REGISTER:
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get register number
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			if( (BaseAddr>=RV_REG_MIN)&&(BaseAddr<=RV_REG_MAX) )
			{
				if( (RV_REG_STATUS==BaseAddr)||(RV_REG_CURR_POS==BaseAddr) )
				{
					stUart2Msg.Type  = MSG_TYPE_PROTOCOL;
					stUart2Msg.DatLen= FrameLen;
					memcpy(stUart2Msg.DatBuf, pDatBuff, FrameLen); 
					OSQPost((OS_Q *)&RoValMsgQueue, &stUart2Msg, sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
						
					// 暂时不返回
					SndBuffIdx = 0;
				}				
				else if( RV_REG_COMMAND==BaseAddr )
				{
				}
				else if( RV_REG_BAUDRATE==BaseAddr )
				{
				}	
				
			}
			break;
		}		
		case WRITE_SINGLE_COIL:
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get command
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			if( COIL_ADDR_VALVE_1==BaseAddr )         
			{
				if( COIL_CTRL_ON==DataLen )      // on
					VALVE_1_ON();
				else if( COIL_CTRL_OFF==DataLen ) // off
					VALVE_1_OFF();
				else
					bStat = MODBUS_ERR_VALUE;             
			}
			else if( COIL_ADDR_VALVE_2==BaseAddr )    
			{
				if( COIL_CTRL_ON==DataLen )      // on
					VALVE_2_ON();
				else if( COIL_CTRL_OFF==DataLen ) // off
					VALVE_2_OFF();
				else
					bStat = MODBUS_ERR_VALUE;          
			}
			else if( COIL_ADDR_VALVE_3==BaseAddr )          
			{
				if( COIL_CTRL_ON==DataLen )       // on
					VALVE_3_ON();
				else if( COIL_CTRL_OFF==DataLen )   // off
					VALVE_3_OFF();
				else
					bStat = MODBUS_ERR_VALUE;              
			}
			else if( COIL_ADDR_FAN==BaseAddr )          
			{
				if( COIL_CTRL_ON==DataLen )       // on
					VALVE_3_ON();
				else if( COIL_CTRL_OFF==DataLen )   // off
					VALVE_3_OFF();
				else
					bStat = MODBUS_ERR_VALUE;              
			}
			else                           // error
			{
				bStat = MODBUS_ERR_ADDRESS;             
			}
			
			// make return data
			if( (MODBUS_OK==bStat)&&(SndBuffIdx>0) )
			{
				aSndBuff[0] = pModbusBuff->Data[2];
				aSndBuff[1] = pModbusBuff->Data[3];
				aSndBuff[2] = pModbusBuff->Data[4];
				aSndBuff[3] = pModbusBuff->Data[5];
				
				SndBuffIdx  = 4;
			}
			
			break;
		}
		case WRITE_SINGLE_REGISTER:
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get value
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			if( (BaseAddr>=RV_REG_MIN)&&(BaseAddr<=RV_REG_MAX) )
			{
				stUart2Msg.Type  = MSG_TYPE_PROTOCOL;
				stUart2Msg.DatLen= FrameLen;
				memcpy(stUart2Msg.DatBuf, pDatBuff, FrameLen); 
				OSQPost((OS_Q *)&RoValMsgQueue, &stUart2Msg, sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);	
				
				// 暂时不返回
				SndBuffIdx = 0;
			}
			
			// make return data
			if( (MODBUS_OK==bStat)&&(SndBuffIdx>0) )
			{
				aSndBuff[0] = pModbusBuff->Data[2];
				aSndBuff[1] = pModbusBuff->Data[3];
				aSndBuff[2] = pModbusBuff->Data[4];
				aSndBuff[3] = pModbusBuff->Data[5];
				
				SndBuffIdx  = 4;
			}

			break;
		}
		case WRITE_MORE_COILS:
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get coil number
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			ByteNum = *pFrameBuff++;
			
			// 需要考虑不是从1开始查询的情况
			if( COIL_ADDR_VALVE_1==BaseAddr )        
			{
				if( DataLen != 3 )
				{
					bStat = MODBUS_ERR_VALUE;          
				}
				else
				{
					if( ByteNum != ( (DataLen/8) + (((DataLen%8)>0)?1:0) ) )
					{
						bStat = MODBUS_ERR_SLAVE_DEVICE;
					}
					else
					{
						ByteNum = *pFrameBuff;
						
						if( ByteNum &(0x01<<0) )
						{
							if( !IsValve1On() )
								VALVE_1_ON();
						}							
						else
						{
							if( IsValve1On() )
								VALVE_1_OFF();
						}
						
						if( ByteNum &(0x01<<1) )
						{
							if( !IsValve2On() )
								VALVE_2_ON();
						}							
						else
						{
							if( IsValve2On() )
								VALVE_2_OFF();
						}
						
						if( ByteNum &(0x01<<2) )
						{
							if( !IsValve3On() )
								VALVE_3_ON();
						}							
						else
						{
							if( IsValve3On() )
								VALVE_3_OFF();
						}
					}
				}
			}			
			else if( COIL_ADDR_FAN==BaseAddr )    // fan
			{
				if( DataLen != 1 )
				{
					bStat = MODBUS_ERR_VALUE;         
				}
				else
				{
					if( ByteNum != ( (DataLen/8) + (((DataLen%8)>0)?1:0) ) )
					{
						bStat = MODBUS_ERR_SLAVE_DEVICE;
					}
					else
					{
						ByteNum = *pFrameBuff;
						
						if( ByteNum &(0x01<<0) )	
						{	
							if( !IsFanOn() )
								FAN_ON();
						}
						else
						{
							if( IsFanOn() )
								FAN_OFF();
						}
					}
				}
			}
			else                           // error
			{
				bStat = MODBUS_ERR_ADDRESS;              
			}
			
			// make return data
			if( (MODBUS_OK==bStat)&&(SndBuffIdx>0) )
			{
				aSndBuff[0] = pModbusBuff->Data[2];
				aSndBuff[1] = pModbusBuff->Data[3];
				aSndBuff[2] = pModbusBuff->Data[4];
				aSndBuff[3] = pModbusBuff->Data[5];
				
				SndBuffIdx  = 4;
			}
			
			break;
		}
		case WRITE_MORE_REGISTERS:
		{
			pFrameBuff = (BYTE *)&pModbusBuff->Struct.Data[0];
			
			// get base address
			BaseAddr   = *pFrameBuff++;
			BaseAddr <<= 8;
			BaseAddr  |= *pFrameBuff++;
			
			// get register number
			DataLen   = *pFrameBuff++;
			DataLen <<= 8;
			DataLen  |= *pFrameBuff++;
			
			ByteNum = *pFrameBuff++;			
			
			if( REG_ADDR_RR_VALVE==BaseAddr )      
			{
				// 0x0001 至 0x0078 
				if( DataLen<0x01 || DataLen>0x78 )
				{
					bStat = MODBUS_ERR_VALUE;
				}
				else
				{
#if 0               // 暂不校验寄存器与数据的对齐
					if( ByteNum != (DataLen<<1) )
					{
						bStat = MODBUS_ERR_SLAVE_DEVICE;
					}
					else
#endif
					{						
						stUart2Msg.Type  = MSG_TYPE_PROTOCOL;
						stUart2Msg.DatLen= FrameLen;
						memcpy(stUart2Msg.DatBuf, pDatBuff, FrameLen); 
						OSQPost((OS_Q *)&RoValMsgQueue, &stUart2Msg, sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
						
						// 暂时不返回
						SndBuffIdx = 0;
					}
				}
			}
			else if( REG_ADDR_SYRINGE==BaseAddr )      
			{
				// 0x0001 至 0x0078 
				if( DataLen<0x01 || DataLen>0x78 )
				{
					bStat = MODBUS_ERR_VALUE;
				}
				else
				{
#if 0               // 暂不校验寄存器与数据的对齐
					if( ByteNum != (DataLen<<1) )
					{
						bStat = MODBUS_ERR_SLAVE_DEVICE;
					}
					else
#endif
					{
	//					memcpy( aNonExecOrderBuf, pFrameBuff, ByteNum );
						
//						LNK_AddNode(stRecvFrmStat.pstFrame->Struct.Order, BaseAddr, DataLen, aNonExecOrderBuf, ByteNum);						
						
						// 暂时不返回
						SndBuffIdx = 0;
					}
				}
			}
			else                           
			{
				bStat = MODBUS_ERR_ADDRESS;              
			}
			
			break;
		}		
		case REPORT_SLAVE_ID:
		{
			break;
		}
		case MASK_WRITE_REGISTER:
		{
			break;
		}
		case READ_WRITE_REGISTERS:
		{
			break;
		}
		case READ_FILE_RECORD:
		{
			break;
		}		
		case WRITE_FILE_RECORD:
		{
			break;
		}
		case READ_DEVICE_SN:
		{
			break;
		}
		default:
		{				
			bStat = MODBUS_ERR_FUNC_CODE;        
		
			break;
		}
	}

	// respone if done
	if( (SndBuffIdx>0)||(bStat>0) )
	{
		if( bStat>0 )
			ModbusAckError( LOCAL_ADDR, pModbusBuff->Struct.Order, bStat );
		else
			ModbusAckNormal( LOCAL_ADDR, pModbusBuff->Struct.Order, aSndBuff, SndBuffIdx );
	}	

	return TRUE;
}

/*******************************************************************************
 * @brief  旋转阀的任务处理
 * @input  
 * @return 0-正常, 其它异常 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MODBUS_RoValveTask( u8 *pRecvBuff )
{
	CPU_CHAR  *pMsg;
	OS_ERR     err;
	u8   *pFrameBuff;
	u8    sndstr[30];
	u16   ModbusAddr;
	u16   ModbusLen;
	u8    SndLen;
	u8    bStat;
	P_TASK_MSG  pstUart2Msg;
		
	pstUart2Msg = (P_TASK_MSG)pRecvBuff;	
	
	pFrameBuff = (BYTE *)&pstUart2Msg->DatBuf[2];
	
	SndLen = 0;	
	
	// get base address
	ModbusAddr   = *pFrameBuff++;
	ModbusAddr <<= 8;
	ModbusAddr  |= *pFrameBuff++;
			
	// get value
	ModbusLen   = *pFrameBuff++;
	ModbusLen <<= 8;
	ModbusLen  |= *pFrameBuff++;
			
	// save receive data
	SndLen = 6;
	memcpy(sndstr, pstUart2Msg->DatBuf, SndLen);
			
	bStat = DEF_TRUE;
	switch( pstUart2Msg->DatBuf[1] )
	{
		case WRITE_MORE_REGISTERS:
		{
			BSP_Ser_WrHex (UART_RV, (CPU_CHAR *)&pstUart2Msg->DatBuf[7], pstUart2Msg->DatBuf[6]);	
			break;
		}
		case WRITE_SINGLE_REGISTER:					
		{
			if( (ModbusAddr>=RV_REG_MIN)&&(ModbusAddr<=RV_REG_MAX) )
			{	
				if( RV_REG_CURR_POS==ModbusAddr )
				{
					if( (ModbusLen>0x00) && (ModbusLen<0x0B) )
					{
						RV_MoveToPosition( ModbusLen );
					}
					else if( 'M'==ModbusLen )
					{
						RV_Home();
					}
					else
					{
						bStat = MODBUS_ERR_VALUE;      
					}
				}
				else if( RV_REG_COMMAND==ModbusAddr )
				{
				}
				else if( RV_REG_BAUDRATE==ModbusAddr )
				{
				}
						
				// 等待从机响应
				if( DEF_TRUE==bStat )
				{
					pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&RoValMsgQueue, 
												(OS_TICK       )3000,
												(OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
												(OS_MSG_SIZE  *)sizeof(TASK_MSG),
												(CPU_TS       *)0,
												(OS_ERR       *)&err);	
					if( OS_ERR_NONE == err )
					{
								
					}							
					else
					{	
						// 返回错误码
						SndLen = 0;

						ModbusAckError(pstUart2Msg->DatBuf[0], pstUart2Msg->DatBuf[1], R_VALVE_EXEC_TIMEOUT);	
					}
				}
			}
			break;
		}
		case READ_INPUT_REGISTER:
		{
			if( (ModbusAddr>=RV_REG_MIN)&&(ModbusAddr<=RV_REG_MAX) )
			{	
				if( RV_REG_STATUS==ModbusAddr )
				{
					RV_QueryStatus();
							
					pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&RoValMsgQueue, 
												(OS_TICK       )1000,
												(OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
												(OS_MSG_SIZE  *)sizeof(TASK_MSG),
												(CPU_TS       *)0,
												(OS_ERR       *)&err);	
					if( OS_ERR_NONE == err )
					{	
						pstUart2Msg = (P_TASK_MSG)pMsg;	
								
						sndstr[2] = 0x02;
						sndstr[3] = pstUart2Msg->DatBuf[0];
						sndstr[4] = pstUart2Msg->DatBuf[1];
						ModbusAckNormal(sndstr[0], sndstr[1], (u8 *)sndstr+2, 3);	
								
						SndLen = 0;
					}							
					else
					{
						// 返回错误码
						SndLen = 0;
						ModbusAckError(pstUart2Msg->DatBuf[0], pstUart2Msg->DatBuf[1], R_VALVE_EXEC_TIMEOUT);	
							
					}
				}
			}
			break;
		}
		default:
		{
			APP_TRACE_INFO(("fail to process rv valve msg\n\r"));
			break;
		}
	}

	// Return data back to uplevel	
	if( SndLen>0 )
	{
		ModbusAckNormal(sndstr[0], sndstr[1], (u8 *)sndstr+2, SndLen-2);	
				
	}		
}
#endif // #ifdef PR_MODBUS
