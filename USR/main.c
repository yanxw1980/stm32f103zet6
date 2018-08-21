/*
 ************************************************************************************************
 * @file   : BSP.c
 * @author : Yan xiangwen
 * @version: V1.0.0
 * @date   : 2017-11-21
 * @brief  :
 ************************************************************************************************
 */
 
/*
 ************************************************************************************************
 *                                             INCLUDE FILES
 ************************************************************************************************
 */ 
#include "includes.h" 
#include "string.h"

#include "ModbusProl.h"
#include "HexProtocol.h"
#include "Adc7689.h"
#include "BoardIoDef.h"
#include "MotorDrv.h"
#include "SpiFlash.h"
#include "RotValveDef.h"
#include "XLP6000.h"
#include "SysError.h"

/*
 ************************************************************************************************
 *                                               INT DEFINES
 ************************************************************************************************
 */


/*
 *************************************************************************************************
 *                                            LOCAL DEFINES
 *************************************************************************************************
 */
//extern volatile uint32_t G_ulSysEvent;	
//extern volatile RUN_PARA stRunPara;	
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE];

// 3个串口接收任务
static  OS_TCB   AppUartRecvTCB[nUART_M];
static  CPU_STK  AppUartRecvStk[nUART_M][APP_UART_RECV_STK_SIZE];

// step motor control task
static  OS_TCB   AppMotor1CtrlTCB;
static  CPU_STK  AppMotor1CtrlStk[APP_MOTOR_CTRL_STK_SIZE];

// step motor control task
static  OS_TCB   AppTeeValveCtrlTCB;
static  CPU_STK  AppTeeValveCtrlStk[APP_TEE_VALVE_CTRL_STK_SIZE];

#if defined(MOTOR_DEF_2)
static  OS_TCB   AppMotor2CtrlTCB;
static  CPU_STK  AppMotor2CtrlStk[APP_MOTOR_CTRL_STK_SIZE];
#endif

// AD7689 convert task
static  OS_TCB   AppAdConvTCB;
static  CPU_STK  AppAdConvStk[APP_ADC_CONV_STK_SIZE];

// temperature control task
static  OS_TCB   AppTempCtrlTCB;
static  CPU_STK  AppTempCtrlStk[APP_TEMP_CTRL_STK_SIZE];

#ifndef BOARD_REAG_RACK
// Rotate Valve convert task
static  OS_TCB   AppRotateValveTCB;
static  CPU_STK  AppRotateValveStk[APP_RV_VALVE_STK_SIZE];
#endif

#ifndef BOARD_REAG_RACK
// Syringe Pump task
static  OS_TCB   AppSyrPumpTCB;
static  CPU_STK  AppSyrPumpStk[APP_SYR_PUMP_STK_SIZE];
#endif

#ifndef BOARD_REAG_RACK
// Prime Flow task
static  OS_TCB   AppPrimeFlowTCB;
static  CPU_STK  AppPrimeFlowStk[APP_PRIME_FLOW_STK_SIZE];
#endif

#ifndef BOARD_REAG_RACK
// Clean Flow task
static  OS_TCB   AppCleanFlowTCB;
static  CPU_STK  AppCleanFlowStk[APP_CLEAN_FLOW_STK_SIZE];
#endif

#ifndef BOARD_REAG_RACK
// Test Flow task
static  OS_TCB   AppTestFlowTCB;
static  CPU_STK  AppTestFlowStk[APP_TEST_FLOW_STK_SIZE];
#endif
/*
 *************************************************************************************************
 *                                        LOCAL GLOBAL VARIABLES
 *************************************************************************************************
 */
OS_Q     MsgQueue[TASK_MSG_MAX];   
TASK_MSG stMsgHandle[TASK_MSG_MAX];
 
OS_SEM   semExecMtrStat[MOTOR_MAX];              // 电机运行状态
OS_SEM   semExecRoValStat;            // 旋转阀运行状态
OS_SEM   semExecSyrPumpStat;          // 注射泵运行状态

/*
 *************************************************************************************************
 *                                         FUNCTION PROTOTYPES
 *************************************************************************************************
 */
static  void  AppTaskCreate (void);
static  void  AppObjCreate  (void);
static  void  AppTaskStart  (void *p_arg);

static  void  AppUart1Recv  (void *p_arg);
static  void  AppUart2Recv  (void *p_arg);
static  void  AppUart3Recv  (void *p_arg);
static  void  AppUart4Recv  (void *p_arg);

static  void  AppAdConv     (void *p_arg);
static  void  AppTempCtrl   (void *p_arg);
static  void  AppMotor1Ctrl (void *p_arg);
static  void  AppTeeValveCtrl(void *p_arg);

#ifndef BOARD_REAG_RACK
static  void  AppRotateValve(void *p_arg);
#endif
#ifndef BOARD_REAG_RACK
static  void  AppSyrPump(void *p_arg);
#endif
#ifndef BOARD_REAG_RACK
static  void  AppPrimeFlow(void *p_arg);
#endif
#ifndef BOARD_REAG_RACK
static  void  AppCleanFlow (void *p_arg);
#endif
#ifndef BOARD_REAG_RACK
static  void  AppTestFlow (void *p_arg);
#endif

extern unsigned int GetCRC16(unsigned char *ptr,  unsigned char len);
/*
 *************************************************************************************************
 * Func. Name: SysParaInit                               
 *Description: 参数初始化接口函数
 * Arguments   
 *     Inputs: 
 *    Outputs: 
 * Note(s)   : 
 *************************************************************************************************
 */ 
//void SysParaInit(void)
//{
// 	ParameterInit( );

//	TempParaInit( );
//}

/*
 *************************************************************************************************
 * Func. Name: main                               
 *Description: 主函数
 * Arguments   
 *     Inputs: 
 *    Outputs: 
 * Note(s)   : 
 *************************************************************************************************
 */ 
//extern void TIM_SetTempCtrlTim(BYTE bEnable, INT32U DelayMs);
//int main(void)
//{ 
//	u8   bAdcConvert;
//	u8   bAdcGetTemp;
//	s16  sTempData[3];	

//	
//	// 更新系统时间
//	SystemCoreClockUpdate();
//	
//	// 初始化IO端口配置
//	BoardLowLevelInit( );

//	// 串口参数配置
//  	UART_Settings( );

//	// Initialize the SPI FLASH driver 
//	sFLASH_Init();
//	
//	// 定时器配置 
//  	TIM_ConfigTempCtrlTim( );	

//	// 参数初始化
//	SysParaInit( );	 
//		
//	// 系统中断 1毫秒
//	if( SysTick_Config(GetSystemCoreClock()/1000) )
//	{
//		while(1) ;
//	}
//	
////	G_ulSysErrorIdx &= ~SYS_ERROR_PARAM_NONINIT;

//	DebugMessage("[FOX COOL BOARD POWER ON]");		
//	

//	bAdcConvert = FALSE;
// 	while( 1 )
//	{
//		// 通信处理
//		UplevelMsgParsing();

//		// 1秒中断,温度采样和控制参数计算
//		if( G_ulSysEvent&SYSTEM_EVENT_1S )
//		{ 
//			G_ulSysEvent &= ~SYSTEM_EVENT_1S;	
//			
//			// 采样温度
//			if( bAdcConvert )
//				bAdcGetTemp = DS18S20_GetTempValue(sTempData);
//			
//			// 温度转换
//			bAdcConvert = DS18S20_Convert();
//			
//			// 温度计算与PWM输出
//			if( TRUE==bAdcGetTemp )
//			{				
//				stRunPara.sAmbientTemp   = sTempData[1]; 			
//				stRunPara.sCoolCurrTemp  = sTempData[0]+stRunPara.sCoolFactor;
//				
////				DebugMessage("C=%d, F=%d",sTempData[0], stRunPara.sCoolFactor);
//				
////				DebugMessage("<Temp: %d,%d,%d>.", sTempData[0],sTempData[1], sTempData[2]);	
//	
//				SetCoolCurrTemp(stRunPara.sCoolCurrTemp);
//				
//				if( DS18S20_IsCoolTempNorm()&&(TRUE==CalcCoolTemp(stRunPara.sCoolCurrTemp)) )
//					CtrlCoolTemp();
//			}			
//		}
//	}		

//} 


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR  err;

    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                /* Create the start task                                */
                 (CPU_CHAR   *)"App Task Start",
                 (OS_TASK_PTR ) AppTaskStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TASK_START_PRIO,
                 (CPU_STK    *)&AppTaskStartStk[0],
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
					 
    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;
//	u8    I2C_Buff[128];

   (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();

    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
    cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */

 //   Mem_Init();                                                 /* Initialize Memory Management Module                  */

#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

    CPU_IntDisMeasMaxCurReset();
	
	BSP_BoardIoInit();
	
	BSP_Ser_Init(UART_PC,    115200);
	BSP_Ser_Init(UART_RV,    19200);
	BSP_Ser_Init(UART_SYR,   9600);
	BSP_Ser_Init(UART_DEBUG, 115200);
    
//    APP_TRACE_INFO(("Creating Application Tasks...\n\r"));
    AppTaskCreate();                                            /* Create Application Tasks                             */
    
//    APP_TRACE_INFO(("Creating Application Events...\n\r"));
    AppObjCreate();                                             /* Create Application Objects                           */
	
	OSQCreate((OS_Q*)&MsgQueue[TASK_MSG_RV],(CPU_CHAR*)"RoMsgQueue", (OS_MSG_QTY)Q_MSG_NUM, (OS_ERR*)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create rv message queue [%02x]\n\r", err));
	
	OSQCreate((OS_Q*)&MsgQueue[TASK_MSG_SYR],(CPU_CHAR*)"SyrPumpQueue", (OS_MSG_QTY)Q_MSG_NUM, (OS_ERR*)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create syr message queue [%02x]\n\r", err));
	
	OSQCreate((OS_Q*)&MsgQueue[TASK_MSG_MTR1],(CPU_CHAR*)"Mtr1Queue", (OS_MSG_QTY)Q_MSG_NUM, (OS_ERR*)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create mtr 1 message queue [%02x]\n\r", err));
	
	OSQCreate((OS_Q*)&MsgQueue[TASK_MSG_TIMING],(CPU_CHAR*)"TimingQueue", (OS_MSG_QTY)Q_MSG_NUM, (OS_ERR*)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create prime message queue [%02x]\n\r", err));
	
	BSP_OS_SemCreate(&semExecMtrStat[MOTOR_1], 0, (CPU_CHAR *)"Motor1Status");
	BSP_OS_SemCreate(&semExecRoValStat,   0, (CPU_CHAR *)"RoValve");
	BSP_OS_SemCreate(&semExecSyrPumpStat, 0, (CPU_CHAR *)"Syringe");
	
	//Init flash spi parameters
	sFLASH_Init();
    
    BSP_LED_Off(0);
	
	// Get SPI Flash ID 
	APP_TRACE_INFO(("Flash ID [%02x]\n\r", sFLASH_ReadID()));	
	
	// 设置当前系统操作
	SetSysCurrOperation( SYS_CURR_OP_IDLE );
	
//	memset( I2C_Buff, 0x37, 128);
//	sFLASH_WriteBuffer(I2C_Buff, 0x2, 3);
//	
//	memset( I2C_Buff, 0x0, 128);
//	sFLASH_ReadBuffer(I2C_Buff, 0x0, 3);
//	
//	APP_TRACE_INFO(("[SPI2, R %02x,%02x]\n\r", I2C_Buff[2], I2C_Buff[1]));
	
    while (DEF_TRUE) 
	{
//		APP_TRACE_INFO(("Flash ID [%02x]\n\r", sFLASH_ReadID()));
//		sFLASH_ReadBuffer(I2C_Buff, 0x100, 128);
//		APP_TRACE_INFO(("[SPI2, R %02x,%02x]\n\r", I2C_Buff[1], I2C_Buff[2]));
		
        /* Task body, always written as an infinite loop.       */
        BSP_LED_Toggle(0);
        OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);				
    }
}
	

/*
*********************************************************************************************************
*                                          UART1 TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppUart1Recv (void *p_arg)
{
	const CPU_INT16U MAX_RECV_BUFF = 256;
	
	CPU_CHAR    RecvBuff[MAX_RECV_BUFF];
	CPU_CHAR    strBuff[MAX_RECV_BUFF];
	CPU_INT16U  BuffIdx;
	CPU_INT16U  datlen;		
	CPU_INT16U  stat;
	
#ifdef PR_MODBUS
	CPU_INT16U  crc;
	P_MODBUS    pModbusBuff;
#else
#endif
	
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating Uart1 Tasks...\n\r"));

	datlen = 0;
    while (DEF_TRUE) 
	{ 		
		BuffIdx = 0;
		stat = BSP_Ser_RdHex (nUART_1, strBuff, &BuffIdx, 10);
		if( BuffIdx>0 )
		{
			memcpy( RecvBuff+datlen, strBuff, BuffIdx );
			datlen += BuffIdx;
			
//			BSP_Ser_WrHex (nUART_1, strBuff, BuffIdx);	
		}
		
		if( DEF_OK != stat )
		{
#ifdef PR_MODBUS			
			pModbusBuff = (P_MODBUS)RecvBuff;
			
			// min length = 1 address + 1 function code + 2 crc
			if( (datlen>=4) && (IsSendToMe(pModbusBuff->Struct.Addr)) )
			{	
				// Calculate crc and compare
				crc = GetCRC16( (BYTE *)RecvBuff, datlen-2 ); //计算CRC校验值  			
		
				//判断CRC校验是否正确
				if( (RecvBuff[datlen-2] == ((crc>>8) & 0xFF)) && 
					(RecvBuff[datlen-1] ==  (crc     & 0xFF)) )
				{
					// output debug message					
//					BSP_Ser_WrHex (nUART_4, RecvBuff, datlen);	
					ModbusProtocol( (u8 *)RecvBuff, datlen );			


					// clear receive buffer
					if( datlen>0 )
					{						
						memset( RecvBuff, 0x00, MAX_RECV_BUFF );
						datlen = 0;
					}	
				}								
			}			
			
			// clear receive buffer
			if( datlen>0 )
			{
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
			
#else   // #ifdef PR_MODBUS

			// min length = 2 header + 2 length
			if( datlen>=4 )
			{
				HEXPR_FrameDecode( (u8 *)RecvBuff, datlen );
			}
			
			// clear receive buffer
			if( datlen>0 )
			{
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
#endif	// #ifdef PR_MODBUS		
		}
    }
}

/*
*********************************************************************************************************
*                                          UART2 TASK Rotate valve
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppUart2Recv (void *p_arg)
{
	const CPU_INT16U MAX_RECV_BUFF = 10;
	
	CPU_CHAR    RecvBuff[MAX_RECV_BUFF];
	CPU_CHAR    strBuff[20];	
	CPU_INT16U  datlen;
	CPU_INT16U  BuffIdx;
	CPU_INT16U  stat;
	CPU_INT32U  sum;	
	OS_ERR      err;
	(void)p_arg;
	
	APP_TRACE_INFO(("Creating Uart2 Tasks...\n\r"));

	datlen = 0;
    while (DEF_TRUE) 
	{ 	
		BuffIdx = 0;
		stat = BSP_Ser_RdHex (UART_RV, strBuff, &BuffIdx, 10);
		if( BuffIdx>0 )
		{
			memcpy( RecvBuff+datlen, strBuff, BuffIdx );
			datlen += BuffIdx;
			
//			BSP_Ser_WrHex (nUART_4, strBuff, BuffIdx);	
		}
		
		// 判断是否接收到有效数据
		if( DEF_OK != stat )
		{	
			// clear receive buffer
			if( datlen>0 )
			{
				sum = 0;
				BuffIdx = 0;
				while( BuffIdx<datlen )
				{
					sum += RecvBuff[BuffIdx];
					
					BuffIdx += 1;
				}
				if( (0!=sum)&&((0xFF*datlen)!=sum) )
				{
					// 返回数据
					stMsgHandle[TASK_MSG_RV].Type  = MSG_TYPE_PROTOCOL;
					stMsgHandle[TASK_MSG_RV].DatLen= datlen;
					memcpy(stMsgHandle[TASK_MSG_RV].DatBuf, RecvBuff, datlen); 
					OSQPost((OS_Q *)&MsgQueue[TASK_MSG_RV], &stMsgHandle[TASK_MSG_RV], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
				}
				
//				BSP_Ser_WrHex (nUART_1, strBuff, BuffIdx);	
			
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
		}
    }
}

/*
*********************************************************************************************************
*                                          UART4 TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppUart4Recv (void *p_arg)
{
	const CPU_INT16U MAX_RECV_BUFF = 256;
	
	CPU_CHAR    RecvBuff[MAX_RECV_BUFF];
	CPU_CHAR    strBuff[MAX_RECV_BUFF];
	CPU_INT16U  BuffIdx;
	CPU_INT16U  datlen;		
	CPU_INT16U  stat;
	
#ifdef PR_MODBUS
	CPU_INT16U  crc;
	P_MODBUS    pModbusBuff;
#else
#endif
	
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating Uart1 Tasks...\n\r"));

	datlen = 0;
    while (DEF_TRUE) 
	{ 		
		BuffIdx = 0;
		stat = BSP_Ser_RdHex (nUART_4, strBuff, &BuffIdx, 10);
		if( BuffIdx>0 )
		{
			memcpy( RecvBuff+datlen, strBuff, BuffIdx );
			datlen += BuffIdx;
			
//			BSP_Ser_WrHex (nUART_1, strBuff, BuffIdx);	
		}
		
		if( DEF_OK != stat )
		{
#ifdef PR_MODBUS			
			pModbusBuff = (P_MODBUS)RecvBuff;
			
			// min length = 1 address + 1 function code + 2 crc
			if( (datlen>=4) && (IsSendToMe(pModbusBuff->Struct.Addr)) )
			{	
				// Calculate crc and compare
				crc = GetCRC16( (BYTE *)RecvBuff, datlen-2 ); //计算CRC校验值  			
		
				//判断CRC校验是否正确
				if( (RecvBuff[datlen-2] == ((crc>>8) & 0xFF)) && 
					(RecvBuff[datlen-1] ==  (crc     & 0xFF)) )
				{
					// output debug message					
//					BSP_Ser_WrHex (nUART_4, RecvBuff, datlen);	
					ModbusProtocol( (u8 *)RecvBuff, datlen );			


					// clear receive buffer
					if( datlen>0 )
					{						
						memset( RecvBuff, 0x00, MAX_RECV_BUFF );
						datlen = 0;
					}	
				}								
			}			
			
			// clear receive buffer
			if( datlen>0 )
			{
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
			
#else   // #ifdef PR_MODBUS

			// min length = 2 header + 2 length
			if( datlen>=4 )
			{
				HEXPR_FrameDecode( (u8 *)RecvBuff, datlen );
			}
			
			// clear receive buffer
			if( datlen>0 )
			{
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
#endif	// #ifdef PR_MODBUS		
		}
    }
}

/*
*********************************************************************************************************
*                                          ROTATE VALVE TASK Rotate valve
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static  void  AppRotateValve (void *p_arg)
{
	CPU_CHAR   *pMsg;
	OS_ERR      err;
	P_TASK_MSG  pstRoValMsg;
    (void)p_arg;
	
	// 处于未联接状态
	RV_SetCurrStat(MODULE_STAT_DISCONNECT);

    while (DEF_TRUE) 
	{ 		
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&MsgQueue[TASK_MSG_RV], 
                          		    (OS_TICK       )0,
                                    (OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
                                    (OS_MSG_SIZE  *)sizeof(TASK_MSG),
                                    (CPU_TS       *)0,
                                    (OS_ERR       *)&err);
		
		if( OS_ERR_NONE != err )
		{
			APP_TRACE_INFO(("fail to get msg [%d]\n\r", err));
			
			continue ;
		}
		
		// 处于指令执行状态
		RV_SetCurrStat(MODULE_STAT_EXECUTING);
		
		pstRoValMsg = (P_TASK_MSG)pMsg;		
				
		if( (MSG_TYPE_PROTOCOL==pstRoValMsg->Type)||(MSG_TYPE_FUNCTION==pstRoValMsg->Type) )
		{
#ifdef PR_MODBUS			
			MODBUS_RoValveTask( (u8 *)pMsg );
#else
			HEX_RoValveTask( (u8 *)pMsg );
#endif	
		}			
		// clear msg buffer
		memset( pMsg, 0x00, sizeof(TASK_MSG) );		
    }
}
#endif

/*
*********************************************************************************************************
*                                          UART3 TASK Syringe
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppUart3Recv (void *p_arg)
{
	const CPU_INT16U MAX_RECV_BUFF = 10;
	
	CPU_CHAR    RecvBuff[MAX_RECV_BUFF];
	CPU_CHAR    strBuff[20];	
	CPU_INT16U  datlen;
	CPU_INT16U  BuffIdx;
	CPU_INT16U  stat;
	CPU_INT32U  sum;	
	OS_ERR      err;
	(void)p_arg;
	
	APP_TRACE_INFO(("Creating Uart3 Tasks...\n\r"));

	datlen = 0;
    while (DEF_TRUE) 
	{ 	
		BuffIdx = 0;
		stat = BSP_Ser_RdHex (UART_SYR, strBuff, &BuffIdx, 50);
		if( BuffIdx>0 )
		{
			memcpy( RecvBuff+datlen, strBuff, BuffIdx );
			datlen += BuffIdx;
			
//			BSP_Ser_WrHex (nUART_4, strBuff, BuffIdx);	
		}
		
		// 判断是否接收到有效数据
		if( DEF_OK != stat )
		{	
			// clear receive buffer
			if( datlen>0 )
			{
				sum = 0;
				BuffIdx = 0;
				while( BuffIdx<datlen )
				{
					sum += RecvBuff[BuffIdx];
					
					BuffIdx += 1;
				}
				if( (0!=sum)&&((0xFF*datlen)!=sum) )
				{
					// lost first byte oxFF
					if( 0x2F == RecvBuff[0] )
					{
						memcpy(strBuff, RecvBuff, datlen);
						memcpy(RecvBuff + 1, strBuff, datlen);
						
						RecvBuff[0] = 0xFF;
						datlen  += 1;
					}
					
					// 返回数据
					stMsgHandle[TASK_MSG_SYR].Type  = MSG_TYPE_PROTOCOL;
					stMsgHandle[TASK_MSG_SYR].DatLen= datlen;
					memcpy(stMsgHandle[TASK_MSG_SYR].DatBuf, RecvBuff, datlen); 
					OSQPost((OS_Q *)&MsgQueue[TASK_MSG_SYR], &stMsgHandle[TASK_MSG_SYR], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
				}
				
//				BSP_Ser_WrHex (nUART_1, strBuff, BuffIdx);	
			
				memset( RecvBuff, 0x00, MAX_RECV_BUFF );
				datlen = 0;
			}
		}
    }
}

/*
*********************************************************************************************************
*                                          ADC7689  TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppAdConv (void *p_arg)
{
//	u8  buff[50];
	OS_ERR      err;
	u16         SetValue;
	u16         AdcValue[20];
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating ADC Tasks...\n\r"));
	
	BSP_Spi1GpioInit();
	
	ADC7689_Init();		
	
	SetValue = ( CFG_OVERWRITE | INCC_UNIPOLAR_SINGLE | INN_CHN_IN1 | BW_FULL | REF_EXT | SEQ_SCAN | RB_NOT_READ_BACK );
	
	//Set parameters
//	ADC7689_ConvAndRecv(2, SetValue<<2, AdcValue);
		
    while (DEF_TRUE) 
	{ 		
		ADC7689_ConvAndRecv(2, SetValue<<2, AdcValue);	
//		sprintf((char *)buff, "[CH: %04x, %04x, %04x, %04x, %04x, %04x, %04x, %04x]", AdcValue[0], AdcValue[1], AdcValue[2], AdcValue[3],
//                                                                                      AdcValue[4], AdcValue[5], AdcValue[6], AdcValue[7]);
//		sprintf((char *)buff, "[CH: %04x, %04x]", AdcValue[0], AdcValue[1]);
//		sprintf((char *)buff, "[CH: %04x, %04x, %04x, %04x]", AdcValue[0], AdcValue[1], AdcValue[2], AdcValue[3]);
		
//		BSP_Ser_WrStr(UART_PC, (CPU_CHAR *)buff);		

		OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);		
    }
}

/*
*********************************************************************************************************
*                                          Tee Valve Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTeeValveCtrl (void *p_arg)
{
	OS_ERR  err;
	u8      idx;
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating Tee Valve Control Tasks...\n\r"));	
	
	for( idx=0; idx<MAX_TEE_VALVE_NUM; idx++ )
	{
		stTeeValveCtrl[idx].Stat       = OFF;
		stTeeValveCtrl[idx].LastN100ms = 0;
	}
	
    while (DEF_TRUE) 
	{ 	
		for( idx=0; idx<MAX_TEE_VALVE_NUM; idx++ )
		{
			if( OFF != stTeeValveCtrl[idx].Stat ) 
			{
				if( stTeeValveCtrl[idx].LastN100ms > 0 )
				{
					stTeeValveCtrl[idx].LastN100ms -= 1;
					
					if( 0 == stTeeValveCtrl[idx].LastN100ms )
					{
						ValveCtrlDirect( idx+1, OFF );
						
						stTeeValveCtrl[idx].Stat = OFF;
					}
				}
			}
		}
	
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);		
    }
}

/*
*********************************************************************************************************
*                                          Temperature control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTempCtrl (void *p_arg)
{
	OS_ERR      err;

   (void)p_arg;
 
	
	APP_TRACE_INFO(("Creating Temperature Control Tasks...\n\r"));

    while (DEF_TRUE) 
	{ 	
//		APP_TRACE_INFO(("Temp %d\n\r", ADC_GetValue( 0 )));
		
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/*
*********************************************************************************************************
*                                          Motor Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppMotor1Ctrl (void *p_arg)
{
	CPU_CHAR  *pMsg;
	OS_ERR     err;
	P_TASK_MSG pstMtrMsg;
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating Motor Control Tasks...\n\r"));
	
	MOTOR_GpioInit(MOTOR_1);  
	
	MOTOR_ParameterInit(MOTOR_1);
	
	// read parameters from flash
	MOTOR_GetPosPara();
	
	// 处于未联接状态
	MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_DISCONNECT);

    while (DEF_TRUE) 
	{ 	
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&MsgQueue[TASK_MSG_MTR1], 
                          		    (OS_TICK       )0,
                                    (OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
                                    (OS_MSG_SIZE  *)sizeof(TASK_MSG),
                                    (CPU_TS       *)0,
                                    (OS_ERR       *)&err);
		
		if( OS_ERR_NONE != err )
		{
			APP_TRACE_INFO(("fail to get mtr1 msg [%d]\n\r", err));
			
			continue ;
		}
		
		// 处于未联接状态
		MOTOR_SetCurrStat(MOTOR_1, MODULE_STAT_EXECUTING);
		
		pstMtrMsg = (P_TASK_MSG)pMsg;		
				
		if( (MSG_TYPE_PROTOCOL==pstMtrMsg->Type)||(MSG_TYPE_FUNCTION==pstMtrMsg->Type) )
		{
#ifdef PR_MODBUS			
			
#else
			HEX_MotorCtrlTask( (u8 *)pMsg );
#endif		

		}	
		
		// clear msg buffer
		memset( pMsg, 0x00, sizeof(TASK_MSG) );
    }
}

#if defined(MOTOR_DEF_2)	
/*
*********************************************************************************************************
*                                          Motor Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppMotor2Ctrl (void *p_arg)
{
	CPU_CHAR  *pMsg;
	OS_ERR     err;
	P_TASK_MSG pstMtrMsg;
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating Motor 2 Control Tasks...\n\r"));
	
	MOTOR_GpioInit(MOTOR_2);  
	
	MOTOR_ParameterInit(MOTOR_2);
	
	// 处于未联接状态
	MTR2_SetCurrStat(MODULE_STAT_DISCONNECT);

    while (DEF_TRUE) 
	{ 	
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&MsgQueue[TASK_MSG_MTR2], 
                          		    (OS_TICK       )0,
                                    (OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
                                    (OS_MSG_SIZE  *)sizeof(TASK_MSG),
                                    (CPU_TS       *)0,
                                    (OS_ERR       *)&err);
		
		if( OS_ERR_NONE != err )
		{
			APP_TRACE_INFO(("fail to get mtr1 msg [%d]\n\r", err));
			
			continue ;
		}
		
		// 处于未联接状态
		MTR2_SetCurrStat(MODULE_STAT_EXECUTING);
		
		pstMtrMsg = (P_TASK_MSG)pMsg;		
				
		if( (MSG_TYPE_PROTOCOL==pstMtrMsg->Type)||(MSG_TYPE_FUNCTION==pstMtrMsg->Type) )
		{
#ifdef PR_MODBUS			
			
#else
//			HEX_Motor2CtrlTask( (u8 *)pMsg );
#endif		

		}	
		
		// clear msg buffer
		memset( pMsg, 0x00, sizeof(TASK_MSG) );
    }
}
#endif

/*
*********************************************************************************************************
*                                          Syringe pump Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static  void  AppSyrPump (void *p_arg)
{
	CPU_CHAR   *pMsg;
	OS_ERR      err;
	P_TASK_MSG  pstSyrPumpMsg;
    (void)p_arg;
	
	APP_TRACE_INFO(("Creating Syringe Control Tasks...\n\r"));	
	
	// 处于未联接状态
	XLP_SetCurrStat(MODULE_STAT_DISCONNECT);

    while (DEF_TRUE) 
	{ 		
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&MsgQueue[TASK_MSG_SYR], 
                          		    (OS_TICK       )0,
                                    (OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
                                    (OS_MSG_SIZE  *)sizeof(TASK_MSG),
                                    (CPU_TS       *)0,
                                    (OS_ERR       *)&err);
		
		if( OS_ERR_NONE != err )
		{
			APP_TRACE_INFO(("fail to get syringe msg [%d]\n\r", err));
			
			continue ;
		}
				
		pstSyrPumpMsg = (P_TASK_MSG)pMsg;		
				
		if( (MSG_TYPE_PROTOCOL==pstSyrPumpMsg->Type)||(MSG_TYPE_FUNCTION==pstSyrPumpMsg->Type) )
		{
			XLP_SetCurrStat(MODULE_STAT_EXECUTING);
			
#ifdef PR_MODBUS			
			
#else
			HEX_SyrPumpTask( (u8 *)pMsg );
#endif		

		}	
		
		// clear msg buffer
		memset( pMsg, 0x00, sizeof(TASK_MSG) );
    }
}
#endif

/*
*********************************************************************************************************
*                                          Prime Flow Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static u32 MotionMachineReset(u16 *pErrNum)
{
	OS_ERR  err;
	u8      ModOrder[32];
	u8      ResetOrder[10];
	u8      PackLen;       // 组成数据帧后的长度

	*pErrNum = 0;
	
	// reset machine
	// syringe pump reset
	ResetOrder[0] = SYR_PUMP_INIT;
	ResetOrder[1] = SetSyringeSpeed(200);
	PackLen = OrderMakeToFrame( ResetOrder, 2, ModOrder );	
		
	stMsgHandle[TASK_MSG_SYR].Type  = MSG_TYPE_FUNCTION;
	stMsgHandle[TASK_MSG_SYR].DatLen= PackLen;
	memcpy(stMsgHandle[TASK_MSG_SYR].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_SYR].DatLen); 
	OSQPost((OS_Q *)&MsgQueue[TASK_MSG_SYR], &stMsgHandle[TASK_MSG_SYR], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
						
	// wait for finishing						
	OSSemSet(&semExecSyrPumpStat, 0, &err);	
	OSSemPend( &semExecSyrPumpStat, MsToTcks(MAX_SYR_ASPR_TIME_MS), OS_OPT_PEND_BLOCKING, NULL, &err );
	
	if( OS_ERR_NONE == err )
	{
		*pErrNum = XLP_GetErrorNumber();
		if( SYS_ERR_NONE != *pErrNum )		
			return DEF_FALSE;		
	}
	else
	{
		*pErrNum = err;
		
		return DEF_FALSE;
	}
		
	// rotate valve reset
	ResetOrder[0] = RO_VALVE_CTRL;
	ResetOrder[1] = 'H';
	PackLen = OrderMakeToFrame( ResetOrder, 2, ModOrder );
	stMsgHandle[TASK_MSG_RV].Type  = MSG_TYPE_FUNCTION;
	stMsgHandle[TASK_MSG_RV].DatLen= PackLen;
	memcpy(stMsgHandle[TASK_MSG_RV].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_RV].DatLen); 
	OSQPost((OS_Q *)&MsgQueue[TASK_MSG_RV], &stMsgHandle[TASK_MSG_RV], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);	
							
	// wait for finishing						
	OSSemSet(&semExecRoValStat, 0, &err);	
	OSSemPend( &semExecRoValStat, MsToTcks(MAX_RV_RUN_TIME_MS), OS_OPT_PEND_BLOCKING, NULL, &err );
	
	if( OS_ERR_NONE == err )
	{
		*pErrNum = RV_GetErrorNumber();
		if( SYS_ERR_NONE != *pErrNum )		
			return DEF_FALSE;		
	}
	else
	{
		*pErrNum = err;
		
		return DEF_FALSE;
	}
		
	// tide motor reset
	ResetOrder[0] = COVER_MTR_INIT;
	ResetOrder[1] = SetMotorSpeed(3);   //rotate per second
	PackLen = OrderMakeToFrame( ResetOrder, 2, ModOrder );	
		
	stMsgHandle[TASK_MSG_MTR1].Type  = MSG_TYPE_FUNCTION;
	stMsgHandle[TASK_MSG_MTR1].DatLen= PackLen;
	memcpy(stMsgHandle[TASK_MSG_MTR1].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_MTR1].DatLen); 
	OSQPost((OS_Q *)&MsgQueue[TASK_MSG_MTR1], &stMsgHandle[TASK_MSG_MTR1], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
						
	// wait for finishing						
	OSSemSet(&semExecMtrStat[MOTOR_1], 0, &err);	
	OSSemPend( &semExecMtrStat[MOTOR_1], MsToTcks(MAX_MTR_RUN_TIME_MS), OS_OPT_PEND_BLOCKING, NULL, &err );
	
	if( OS_ERR_NONE == err )
	{
		*pErrNum = MOTOR_GetCurrError(MOTOR_1);
		if( SYS_ERR_NONE != *pErrNum )		
			return DEF_FALSE;		
	}
	else
	{
		*pErrNum = err;
		
		return DEF_FALSE;
	}
		
	return DEF_TRUE;
}
#endif

/*
*********************************************************************************************************
*                                          Prime Flow Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static  void  AppPrimeFlow (void *p_arg)
{
	CPU_CHAR  *pMsg;
	P_TASK_MSG pstPrimeMsg;
	OS_ERR  err;
	u8      OrderBuff[512];
	u8      ModOrder[32];
	u16     AddrBase;
	u16     MaxSize;
	u8      PackLen;       // 组成数据帧后的长度
	u8      CommLen;
	u8      MsgSource;
	u8      ProtocolType;
	u8      ProtocolOrder;
	u16     ProtocolIndex;
	u8      UartSendBuff[32];
	u16     UartSendLen;
	u16     ErrNum;
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating prime flow Control Tasks...\n\r"));		
	
	MaxSize = AddrBase;
	
    while (DEF_TRUE) 
	{ 			
		pMsg = (CPU_CHAR *)OSQPend ((OS_Q*)&MsgQueue[TASK_MSG_TIMING], 
                          		    (OS_TICK       )0,
                                    (OS_OPT        )OS_OPT_PEND_BLOCKING,    //OS_OPT_PEND_NON_BLOCKING
                                    (OS_MSG_SIZE  *)sizeof(TASK_MSG),
                                    (CPU_TS       *)0,
                                    (OS_ERR       *)&err);
				
		if( OS_ERR_NONE != err )
			continue ;		
				
		pstPrimeMsg = (P_TASK_MSG)pMsg;
		
		// 通信类型
		MsgSource   = pstPrimeMsg->Type;
		
		// 存储通信协议信息,用于返回帧处理
		ProtocolType  = pstPrimeMsg->DatBuf[4];
		ProtocolIndex = pstPrimeMsg->DatBuf[5] + ((u16)pstPrimeMsg->DatBuf[6]<<8);
		ProtocolOrder = pstPrimeMsg->DatBuf[8];
		
		if( FLOW_PRIME_START == ProtocolOrder )
			MaxSize =  TimingToCommand(OrderBuff, TIMING_TYPE_PRIME);
		else if( FLOW_COVER_START == ProtocolOrder )
			MaxSize =  TimingToCommand(OrderBuff, TIMING_TYPE_COVER);	
		else if( FLOW_LOAD_START == ProtocolOrder )
			MaxSize =  TimingToCommand(OrderBuff, TIMING_TYPE_LOADING);		
		else if(  FLOW_CLEAN_START== ProtocolOrder )
			MaxSize =  TimingToCommand(OrderBuff, TIMING_TYPE_CLEAN);		
		
		// 设置当前操作为灌注中
		SetSysCurrOperation( SYS_CURR_OP_PRIMING );
		
		// reset machine
		if( ((FLOW_PRIME_START == ProtocolOrder) || (FLOW_CLEAN_START == ProtocolOrder) ) && (DEF_TRUE != MotionMachineReset(&ErrNum)) )
		{
			// 返回正常状态
			if( (MSG_TYPE_PROTOCOL == MsgSource) && (FRM_TYPE_1 == ProtocolType) )
			{
				UartSendBuff[0] = 0x01;
				UartSendBuff[1] = ErrNum>>0;
				UartSendBuff[2] = ErrNum>>8;
				UartSendLen = 3;
				
				ResultUpLayerComm(ProtocolIndex, ProtocolOrder,	UartSendBuff, UartSendLen);	
			}
		
			SetSysCurrOperation( SYS_CURR_OP_IDLE );
			
			continue ;
		}		
				
		// flow starting ...
		AddrBase = 0;
		while (DEF_TRUE) 
		{
			UartSendLen = 0;
			
			switch( OrderBuff[AddrBase] )
			{
				case COVER_MTR_REL_DIST:
				case COVER_MTR_ABS_DIST:	
				case COVER_MTR_POSITION:		
				{
					if( COVER_MTR_REL_DIST == OrderBuff[AddrBase] )
						CommLen = 5;
					else if( COVER_MTR_ABS_DIST == OrderBuff[AddrBase] )
						CommLen = 4;
					else if( COVER_MTR_POSITION == OrderBuff[AddrBase] )
						CommLen = 3;
					
					PackLen = OrderMakeToFrame( &OrderBuff[AddrBase], CommLen, ModOrder );	
					
					if( PackLen>sizeof(HEADER) )
					{
					
						stMsgHandle[TASK_MSG_MTR1].Type  = MSG_TYPE_FUNCTION;
						stMsgHandle[TASK_MSG_MTR1].DatLen= PackLen;
						memcpy(stMsgHandle[TASK_MSG_MTR1].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_MTR1].DatLen); 
						OSQPost((OS_Q *)&MsgQueue[TASK_MSG_MTR1], &stMsgHandle[TASK_MSG_MTR1], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);
						
						// wait for finishing						
						OSSemSet(&semExecMtrStat[MOTOR_1], 0, &err);	
						OSSemPend( &semExecMtrStat[MOTOR_1], MsToTcks(MAX_MTR_RUN_TIME_MS), OS_OPT_PEND_BLOCKING, NULL, &err );
                   
//						if( (OS_ERR_NONE==err)&&(MODULE_STAT_NORMAL==MOTOR_GetCurrStat()) )
						if( OS_ERR_NONE==err )
						{
							// point to next command
							AddrBase += CommLen;
						}
						else
						{
							// error process 
							UartSendBuff[0] = 0x01;
							UartSendLen = 1;
								
							memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
							UartSendLen += CommLen;
						}
						
						OSSemSet(&semExecMtrStat[MOTOR_1], 0, &err);
					}
					else
					{
						// 错误处理
						UartSendBuff[0] = 0xFF;
						UartSendLen = 1;
								
						memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
						UartSendLen += CommLen;
					}
					break;
				}
				
				case RO_VALVE_CTRL:
				{
					CommLen = 2;
					
					if( MODULE_STAT_EXECUTING != RV_GetCurrStat() )
					{
						PackLen = OrderMakeToFrame( &OrderBuff[AddrBase], CommLen, ModOrder );
						
						if( PackLen>sizeof(HEADER) )
						{
							stMsgHandle[TASK_MSG_RV].Type  = MSG_TYPE_FUNCTION;
							stMsgHandle[TASK_MSG_RV].DatLen= PackLen;
							memcpy(stMsgHandle[TASK_MSG_RV].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_RV].DatLen); 
							OSQPost((OS_Q *)&MsgQueue[TASK_MSG_RV], &stMsgHandle[TASK_MSG_RV], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);							
							
							// wait for finishing						
							OSSemSet(&semExecRoValStat, 0, &err);	
							OSSemPend( &semExecRoValStat, MsToTcks(MAX_RV_RUN_TIME_MS), OS_OPT_PEND_BLOCKING, NULL, &err );
					   
							if( (OS_ERR_NONE==err)&&(MODULE_STAT_NORMAL==RV_GetCurrStat()) )
							{
								// point to next command
								AddrBase += CommLen;
							}
							else
							{
								// error process
								UartSendBuff[0] = 0x01;
								UartSendLen = 1;
								
								memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
								UartSendLen += CommLen;
							}
							
							OSSemSet(&semExecRoValStat, 0, &err);							
						}
						else
						{
							// 错误处理
							UartSendBuff[0] = 0xFF;
							UartSendLen = 1;
									
							memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
							UartSendLen += CommLen;
						}
					}
					break;
				}
				
				//注射泵
				case SYR_PUMP_INIT:
				case SYR_PUMP_IN:
				case SYR_PUMP_OUT:	
				{
					if( SYR_PUMP_INIT == OrderBuff[AddrBase] )
						CommLen = 2;
					else if( SYR_PUMP_IN == OrderBuff[AddrBase] )
						CommLen = 4;
					else if( SYR_PUMP_OUT == OrderBuff[AddrBase] )
						CommLen = 4;
					
					if( MODULE_STAT_EXECUTING != XLP_GetCurrStat() )
					{
						PackLen = OrderMakeToFrame( &OrderBuff[AddrBase], CommLen, ModOrder );
						
						if( PackLen>sizeof(HEADER) )
						{
							stMsgHandle[TASK_MSG_SYR].Type  = MSG_TYPE_FUNCTION;
							stMsgHandle[TASK_MSG_SYR].DatLen= PackLen;
							memcpy(stMsgHandle[TASK_MSG_SYR].DatBuf, (u8 *)ModOrder, stMsgHandle[TASK_MSG_SYR].DatLen); 
							OSQPost((OS_Q *)&MsgQueue[TASK_MSG_SYR], &stMsgHandle[TASK_MSG_SYR], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);							
							
							// wait for finishing						
							OSSemSet(&semExecSyrPumpStat, 0, &err);	
							OSSemPend( &semExecSyrPumpStat, MsToTcks(300000), OS_OPT_PEND_BLOCKING, NULL, &err );
					   
							if( (OS_ERR_NONE==err)&&(MODULE_STAT_NORMAL==XLP_GetCurrStat()) )
							{
								// point to next command
								AddrBase += CommLen;
							}
							else
							{
								// error process 
								UartSendBuff[0] = 0x01;
								UartSendLen += 1;
								
								memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
								UartSendLen += CommLen;
							}
							
							OSSemSet(&semExecSyrPumpStat, 0, &err);							
						}
						else
						{
							// 错误处理
							UartSendBuff[0] = 0xFF;
							UartSendLen += 1;
									
							memcpy( UartSendBuff+1, &OrderBuff[AddrBase], CommLen );
							UartSendLen += CommLen;
						}
					}
					
					break;
				}
				
				case VALVE_CTRL:
				{
					ValveCtrlFromProl( &OrderBuff[AddrBase+1], 4, NULL);
					
					AddrBase += 5;
					
					break;
				}	
				
				case SYSTEM_DELAY_MS:
				{
					BSP_OS_TimeDlyMs( OrderBuff[AddrBase+1] + ((u32)OrderBuff[AddrBase+2]<<8) + ((u32)OrderBuff[AddrBase+3]<<16) + ((u32)OrderBuff[AddrBase+4]<<24) );
				
					AddrBase += 5;
					break;
				}				
				default:
				{
					// 错误处理
					
					break;
				}
			}
		
			// 返回异常状态
			if( UartSendLen>0 )
			{
				if( (MSG_TYPE_PROTOCOL == MsgSource) && (FRM_TYPE_1 == ProtocolType) )
				{
					ResultUpLayerComm(ProtocolIndex, ProtocolOrder,	UartSendBuff, UartSendLen);					
				}
				
				break;
			}
			
			// 运行完成
			if( AddrBase >= MaxSize )
				break;
		}		
	
		// 返回正常状态
		if( (MSG_TYPE_PROTOCOL == MsgSource) && (FRM_TYPE_1 == ProtocolType) && (0 == UartSendLen) )
		{
			UartSendBuff[0] = 0;
			UartSendLen += 1;
			
			ResultUpLayerComm(ProtocolIndex, ProtocolOrder,	UartSendBuff, UartSendLen);	
		}
		
		// 完成, 更新操作状态
		SetSysCurrOperation( SYS_CURR_OP_IDLE );
    }
}
#endif

/*
*********************************************************************************************************
*                                          Clean Flow Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static  void  AppCleanFlow (void *p_arg)
{
	OS_ERR  err;

   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating clean flow Control Tasks...\n\r"));	


    while (DEF_TRUE) 
	{ 			
				
		OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}
#endif

/*
*********************************************************************************************************
*                                          Test Flow Control TASK 
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/
#ifndef BOARD_REAG_RACK
static  void  AppTestFlow (void *p_arg)
{
	OS_ERR  err;
//u8 frmBuff[20];
   (void)p_arg; 
	
	APP_TRACE_INFO(("Creating test flow Control Tasks...\n\r"));	


//	frmBuff[0] = 0xAA;
//	frmBuff[1] = 0x55;
//	frmBuff[2] = 0x05;
//	frmBuff[3] = 0x00;
//	frmBuff[4] = 0x01;
//	frmBuff[5] = 0x01;
//	frmBuff[6] = 0x00;
//	frmBuff[7] = 0x10;
//	frmBuff[8] = 0x62;
//	frmBuff[9] = 0xb0;
//	frmBuff[10]= 0x15;
    while (DEF_TRUE) 
	{ 			
//		stMsgHandle[TASK_MSG_TIMING].Type  = MSG_TYPE_FUNCTION;
//		stMsgHandle[TASK_MSG_TIMING].DatLen= 11;
//		memcpy(stMsgHandle[TASK_MSG_TIMING].DatBuf, (u8 *)frmBuff, stMsgHandle[TASK_MSG_TIMING].DatLen); 
//		OSQPost((OS_Q *)&MsgQueue[TASK_MSG_TIMING], &stMsgHandle[TASK_MSG_TIMING], sizeof(TASK_MSG), OS_OPT_POST_FIFO, &err);	
		
				
		OSTimeDlyHMSM(0, 1, 30, 0, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}
#endif

/*
*********************************************************************************************************
*                                      CREATE APPLICATION TASKS
*
* Description:  This function creates the application tasks.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
	OS_ERR  err;
	
	// Create uart1 receive task                                
	OSTaskCreate((OS_TCB     *)&AppUartRecvTCB[nUART_1],                
                 (CPU_CHAR   *)"App Uart1 Recv",
                 (OS_TASK_PTR ) AppUart1Recv,
                 (void       *) 0,
                 (OS_PRIO     ) APP_UART_RECV_PRIO + nUART_1,
                 (CPU_STK    *)&AppUartRecvStk[nUART_1][0],
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create uart1 recv task [%d]\n\r", err));
				 
	
	// Create uart2 receive task                                
	OSTaskCreate((OS_TCB     *)&AppUartRecvTCB[nUART_2],                
                 (CPU_CHAR   *)"App Uart2 Recv",
                 (OS_TASK_PTR ) AppUart2Recv,
                 (void       *) 0,
                 (OS_PRIO     ) APP_UART_RECV_PRIO + nUART_2,
                 (CPU_STK    *)&AppUartRecvStk[nUART_2][0],
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create uart2 recv task [%d]\n\r", err));
	
	// Create uart3 receive task                                
	OSTaskCreate((OS_TCB     *)&AppUartRecvTCB[nUART_3],                
                 (CPU_CHAR   *)"App Uart3 Recv",
                 (OS_TASK_PTR ) AppUart3Recv,
                 (void       *) 0,
                 (OS_PRIO     ) APP_UART_RECV_PRIO + nUART_3,
                 (CPU_STK    *)&AppUartRecvStk[nUART_3][0],
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create uart3 recv task [%d]\n\r", err));

	// Create uart4 receive task                                
	OSTaskCreate((OS_TCB     *)&AppUartRecvTCB[nUART_4],                
                 (CPU_CHAR   *)"App Uart4 Recv",
                 (OS_TASK_PTR ) AppUart4Recv,
                 (void       *) 0,
                 (OS_PRIO     ) APP_UART_RECV_PRIO + nUART_4,
                 (CPU_STK    *)&AppUartRecvStk[nUART_4][0],
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_UART_RECV_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create uart4 recv task [%d]\n\r", err));
	
	// Create adc convert task                                
	OSTaskCreate((OS_TCB     *)&AppAdConvTCB,                
                 (CPU_CHAR   *)"App Adc convert",
                 (OS_TASK_PTR ) AppAdConv,
                 (void       *) 0,
                 (OS_PRIO     ) APP_ADC_CONV_PRIO,
                 (CPU_STK    *)&AppAdConvStk[0],
                 (CPU_STK_SIZE) APP_ADC_CONV_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_ADC_CONV_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create adc convert task [%d]\n\r", err));
	
	// Create temperature control task                                
	OSTaskCreate((OS_TCB     *)&AppTempCtrlTCB,                
                 (CPU_CHAR   *)"App Temp Ctrl",
                 (OS_TASK_PTR ) AppTempCtrl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TEMP_CTRL_PRIO,
                 (CPU_STK    *)&AppTempCtrlStk[0],
                 (CPU_STK_SIZE) APP_TEMP_CTRL_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TEMP_CTRL_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create temperatue control task [%d]\n\r", err));
	
	// Create motor control task                                
	OSTaskCreate((OS_TCB     *)&AppMotor1CtrlTCB,                
                 (CPU_CHAR   *)"App Motor1 Ctrl",
                 (OS_TASK_PTR ) AppMotor1Ctrl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_MOTOR_CTRL_PRIO,
                 (CPU_STK    *)&AppMotor1CtrlStk[0],
                 (CPU_STK_SIZE) APP_MOTOR_CTRL_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_MOTOR_CTRL_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create motor 1 control task [%d]\n\r", err));
	
	// Create motor control task                                
	OSTaskCreate((OS_TCB     *)&AppTeeValveCtrlTCB,                
                 (CPU_CHAR   *)"App Tee Valve Ctrl",
                 (OS_TASK_PTR ) AppTeeValveCtrl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TEE_VALVE_CTRL_PRIO,
                 (CPU_STK    *)&AppTeeValveCtrlStk[0],
                 (CPU_STK_SIZE) APP_TEE_VALVE_CTRL_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TEE_VALVE_CTRL_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create tee valve control task [%d]\n\r", err));
	
#if defined(MOTOR_DEF_2)	
	// Create motor control task                                
	OSTaskCreate((OS_TCB     *)&AppMotor2CtrlTCB,                
                 (CPU_CHAR   *)"App Motor2 Ctrl",
                 (OS_TASK_PTR ) AppMotor2Ctrl,
                 (void       *) 0,
                 (OS_PRIO     ) APP_MOTOR_CTRL_PRIO,
                 (CPU_STK    *)&AppMotor2CtrlStk[0],
                 (CPU_STK_SIZE) APP_MOTOR_CTRL_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_MOTOR_CTRL_STK_SIZE,
                 (OS_MSG_QTY  ) 5u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create motor 2 control task [%d]\n\r", err));
#endif

#ifndef BOARD_REAG_RACK	
	// Create rotate valve task                                
	OSTaskCreate((OS_TCB     *)&AppRotateValveTCB,                
                 (CPU_CHAR   *)"App Rotate Valve",
                 (OS_TASK_PTR ) AppRotateValve,
                 (void       *) 0,
                 (OS_PRIO     ) APP_RV_VALVE_PRIO,
                 (CPU_STK    *)&AppRotateValveStk[0],
                 (CPU_STK_SIZE) APP_RV_VALVE_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_RV_VALVE_STK_SIZE,
                 (OS_MSG_QTY  ) 64u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create rotate valve task [%d]\n\r", err));
#endif

#ifndef BOARD_REAG_RACK	
	// Create Syringe pump task                                
	OSTaskCreate((OS_TCB     *)&AppSyrPumpTCB,                
                 (CPU_CHAR   *)"App Syringe Pump",
                 (OS_TASK_PTR ) AppSyrPump,
                 (void       *) 0,
                 (OS_PRIO     ) APP_SYR_PUMP_PRIO,
                 (CPU_STK    *)&AppSyrPumpStk[0],
                 (CPU_STK_SIZE) APP_SYR_PUMP_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_SYR_PUMP_STK_SIZE,
                 (OS_MSG_QTY  ) 64u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create syringe pump task [%d]\n\r", err));
#endif

#ifndef BOARD_REAG_RACK	
	// Create prime flow task                                
	OSTaskCreate((OS_TCB     *)&AppPrimeFlowTCB,                
                 (CPU_CHAR   *)"App prime flow",
                 (OS_TASK_PTR ) AppPrimeFlow,
                 (void       *) 0,
                 (OS_PRIO     ) APP_PRIME_FLOW_PRIO,
                 (CPU_STK    *)&AppPrimeFlowStk[0],
                 (CPU_STK_SIZE) APP_PRIME_FLOW_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_PRIME_FLOW_STK_SIZE,
                 (OS_MSG_QTY  ) 64u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create prime flow task [%d]\n\r", err));
#endif

#ifndef BOARD_REAG_RACK	
	// Create clean flow task                                
	OSTaskCreate((OS_TCB     *)&AppCleanFlowTCB,                
                 (CPU_CHAR   *)"App clean flow",
                 (OS_TASK_PTR ) AppCleanFlow,
                 (void       *) 0,
                 (OS_PRIO     ) APP_CLEAN_FLOW_PRIO,
                 (CPU_STK    *)&AppCleanFlowStk[0],
                 (CPU_STK_SIZE) APP_CLEAN_FLOW_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_CLEAN_FLOW_STK_SIZE,
                 (OS_MSG_QTY  ) 64u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create clean flow task [%d]\n\r", err));
#endif

#ifndef BOARD_REAG_RACK	
	// Create test flow task                                
	OSTaskCreate((OS_TCB     *)&AppTestFlowTCB,                
                 (CPU_CHAR   *)"App test flow",
                 (OS_TASK_PTR ) AppTestFlow,
                 (void       *) 0,
                 (OS_PRIO     ) APP_TEST_FLOW_PRIO,
                 (CPU_STK    *)&AppTestFlowStk[0],
                 (CPU_STK_SIZE) APP_TEST_FLOW_STK_SIZE / 10,
                 (CPU_STK_SIZE) APP_TEST_FLOW_STK_SIZE,
                 (OS_MSG_QTY  ) 64u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);	
	if( OS_ERR_NONE != err )
		APP_TRACE_INFO(("Fail to create test flow task [%d]\n\r", err));
#endif
}


/*
*********************************************************************************************************
*                                      CREATE APPLICATION EVENTS
*
* Description:  This function creates the application kernel objects.
*
* Arguments  :  none
*
* Returns    :  none
*********************************************************************************************************
*/

static  void  AppObjCreate (void)
{
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
