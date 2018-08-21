/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
*                          (c) Copyright 2003-2010; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               This BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                       SERIAL (UART) INTERFACE
*
* Filename      : bsp_ser.c
* Version       : V1.00
* Programmer(s) : EHS
*                 SR
*                 AA
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_SER_MODULE
#include <TYPE.h>
#include <bsp.h>
#include <string.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/
const uint16_t UART_PIN_RXD[nUART_M] = {IO_PIN_UART1_RXD, IO_PIN_UART2_RXD, IO_PIN_UART3_RXD, IO_PIN_UART4_RXD };
const uint16_t UART_PIN_TXD[nUART_M] = {IO_PIN_UART1_TXD, IO_PIN_UART2_TXD, IO_PIN_UART3_TXD, IO_PIN_UART4_TXD };
const uint32_t UART_CLK_RXD[nUART_M] = {IO_CLK_UART1_RXD, IO_CLK_UART2_RXD, IO_CLK_UART3_RXD, IO_CLK_UART4_RXD };
const uint32_t UART_CLK_TXD[nUART_M] = {IO_CLK_UART1_TXD, IO_CLK_UART2_TXD, IO_CLK_UART3_TXD, IO_CLK_UART4_TXD };
GPIO_TypeDef*  UART_PORT_RXD[nUART_M]= {IO_PORT_UART1_RXD,IO_PORT_UART2_RXD,IO_PORT_UART3_RXD,IO_PORT_UART4_RXD};
GPIO_TypeDef*  UART_PORT_TXD[nUART_M]= {IO_PORT_UART1_TXD,IO_PORT_UART2_TXD,IO_PORT_UART3_TXD,IO_PORT_UART4_TXD};

USART_TypeDef *UART_TYPE[nUART_M] = {USART1,      USART2,     USART3,      UART4      };
const uint32_t UART_BAUD[nUART_M] = {UART1_BAUD,  UART2_BAUD, UART3_BAUD,  UART4_BAUD };
const uint32_t UART_PRIO[nUART_M] = {UART1_PRIO,  UART2_PRIO, UART3_PRIO,  UART4_PRIO };
const uint32_t UART_IRQn[nUART_M] = {USART1_IRQn, USART2_IRQn,USART3_IRQn, UART4_IRQn};
const uint32_t UART_CLKI[nUART_M] = {UART1_CLK,   UART2_CLK,  UART3_CLK,   UART4_CLK  };

// Usart definition
const UART_CONFIG stUartCfg[nUART_M] = 
{
	USART1, TRUE, 115200, NVIC_PriorityGroup_0, USART1_IRQn, RCC_APB2Periph_USART1, BSP_PERIPH_ID_USART1,
	USART2, TRUE, 19200,  NVIC_PriorityGroup_1, USART2_IRQn, RCC_APB1Periph_USART2, BSP_PERIPH_ID_USART2,
	USART3, TRUE, 9600,   NVIC_PriorityGroup_2, USART3_IRQn, RCC_APB1Periph_USART3, BSP_PERIPH_ID_USART3,
	UART4,  TRUE, 115200, NVIC_PriorityGroup_3, UART4_IRQn,  RCC_APB1Periph_UART4,  BSP_PERIPH_ID_USART4,
};


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  UART_SEM stUartSem[nUART_M];

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
static  CPU_CHAR     BSP_SerCmdHistory[BSP_CFG_SER_CMD_HISTORY_LEN];
#endif

#define MAX_UART_BUFF    128
static uint8_t  gUartRecvBuff[nUART_M][MAX_UART_BUFF];
static uint16_t gUartGetIndex[nUART_M];
static uint16_t gUartReadIndex[nUART_M];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        BSP_Ser_WrByteUnlocked  (CPU_INT08U  UartIdx, CPU_INT08U  c);
static  void        BSP_Ser_1_ISR_Handler   (void);
static  void        BSP_Ser_2_ISR_Handler   (void);
static  void        BSP_Ser_3_ISR_Handler   (void);
static  void        BSP_Ser_4_ISR_Handler   (void);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          BSP_Ser_Init()
*
* Description : Initialize a serial port for communication.
*
* Argument(s) : baud_rate           The desire RS232 baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_Init (CPU_INT08U   UartIdx, CPU_INT32U  baud_rate)
{
    FlagStatus              tc_status;
    GPIO_InitTypeDef        gpio_init;
    USART_InitTypeDef       usart_init;
    USART_ClockInitTypeDef  usart_clk_init;
	
	CPU_CHAR str[20];

    /* ------------------ INIT OS OBJECTS ----------------- */
	sprintf(str, "Ser%d Tx Wait", UartIdx+1);
	BSP_OS_SemCreate(&stUartSem[UartIdx].TxWait,   0, str);
	sprintf(str, "Ser%d Rx Wait", UartIdx+1);
    BSP_OS_SemCreate(&stUartSem[UartIdx].RxWait,   0, str);
	sprintf(str, "Ser%d Lock", UartIdx+1);
    BSP_OS_SemCreate(&stUartSem[UartIdx].Lock,     1, str);
	

#if (BSP_CFG_SER_CMD_HISTORY_LEN > 0u)
    BSP_SerCmdHistory[0] = (CPU_CHAR)'\0';
#endif

    /* ----------------- INIT USART STRUCT ---------------- */
    usart_init.USART_BaudRate            = baud_rate;
    usart_init.USART_WordLength          = USART_WordLength_8b;
    usart_init.USART_StopBits            = USART_StopBits_1;
    usart_init.USART_Parity              = USART_Parity_No ;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    usart_clk_init.USART_Clock           = USART_Clock_Disable;
    usart_clk_init.USART_CPOL            = USART_CPOL_Low;
    usart_clk_init.USART_CPHA            = USART_CPHA_2Edge;
    usart_clk_init.USART_LastBit         = USART_LastBit_Disable;


    BSP_PeriphEn(stUartCfg[UartIdx].PERIPH_ID);    
    BSP_PeriphEn(BSP_PERIPH_ID_AFIO);
 
    /* ----------------- SETUP USART GPIO ---------------- */
//	BSP_PeriphEn(BSP_PERIPH_ID_IOPA);
	RCC_APB2PeriphClockCmd(UART_CLK_RXD[UartIdx]|UART_CLK_TXD[UartIdx], ENABLE);	
	
    /* Configure GPIOA.2 as push-pull.                      */
    gpio_init.GPIO_Pin   = UART_PIN_TXD[UartIdx];
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(UART_PORT_TXD[UartIdx], &gpio_init);

    /* Configure GPIOA.3 as input floating.                 */
    gpio_init.GPIO_Pin   = UART_PIN_RXD[UartIdx];
    gpio_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(UART_PORT_RXD[UartIdx], &gpio_init);

    /* ------------------ SETUP USART -------------------- */
    USART_Init(stUartCfg[UartIdx].pMode, &usart_init);
    USART_ClockInit(stUartCfg[UartIdx].pMode, &usart_clk_init);
    USART_Cmd(stUartCfg[UartIdx].pMode, ENABLE);
    
    USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_TC, DISABLE);
    USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_TXE, DISABLE);
    tc_status  = USART_GetFlagStatus(stUartCfg[UartIdx].pMode, USART_FLAG_TC);
    
    while (tc_status == SET) 
	{
        USART_ClearITPendingBit(stUartCfg[UartIdx].pMode, USART_IT_TC);
        USART_ClearFlag(stUartCfg[UartIdx].pMode, USART_IT_TC);
        BSP_OS_TimeDlyMs(10);
        tc_status = USART_GetFlagStatus(stUartCfg[UartIdx].pMode, USART_FLAG_TC);        
    }	
	
	if( nUART_1==UartIdx )
	{
		BSP_IntVectSet(BSP_INT_ID_USART1, BSP_Ser_1_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_USART1);
	}
	else if( nUART_2==UartIdx )
	{ 
		BSP_IntVectSet(BSP_INT_ID_USART2, BSP_Ser_2_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_USART2);
	}
	else if( nUART_3==UartIdx )
	{ 
		BSP_IntVectSet(BSP_INT_ID_USART3, BSP_Ser_3_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_USART3);
	}
	else if( nUART_4==UartIdx )
	{ 
		BSP_IntVectSet(BSP_INT_ID_USART4, BSP_Ser_4_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_USART4);
	}
	
	// reset memory
	memset(	gUartRecvBuff[UartIdx], 0x00, MAX_UART_BUFF );
	memset(	gUartGetIndex,          0x00, nUART_M<<1 );
	memset(	gUartReadIndex,         0x00, nUART_M<<1 );

	USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_RXNE, ENABLE);  
}


/*
*********************************************************************************************************
*                                         BSP_Ser_Send_Recv()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_Send_Recv (CPU_INT08U UartIdx)
{
    FlagStatus tc_status;
    FlagStatus rxne_status;
	
    rxne_status = USART_GetFlagStatus(stUartCfg[UartIdx].pMode, USART_FLAG_RXNE);
    if (rxne_status == SET) 
	{
		gUartRecvBuff[UartIdx][gUartGetIndex[UartIdx]] = USART_ReceiveData(stUartCfg[UartIdx].pMode) & 0xFF;
		
		gUartGetIndex[UartIdx] += 1;
		if( gUartGetIndex[UartIdx]>=MAX_UART_BUFF )
			gUartGetIndex[UartIdx]  = 0;
		
		USART_ClearITPendingBit(stUartCfg[UartIdx].pMode, USART_IT_RXNE);                   /* Clear the USART receive interrupt.                */
  
		BSP_OS_SemPost(&stUartSem[UartIdx].RxWait);                                         /* Post to the sempahore                              */
    }

    tc_status = USART_GetFlagStatus(stUartCfg[UartIdx].pMode, USART_FLAG_TC);
    if (tc_status == SET) 
	{
        USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(stUartCfg[UartIdx].pMode, USART_IT_TC);                     /* Clear the USART receive interrupt.                */
 
		BSP_OS_SemPost(&stUartSem[UartIdx].TxWait);                                         /* Post to the semaphore                              */
    }
}

/*
*********************************************************************************************************
*                                         BSP_Ser_1_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_1_ISR_Handler (void)
{
	BSP_Ser_Send_Recv (nUART_1);   
}

/*
*********************************************************************************************************
*                                         BSP_Ser_2_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_2_ISR_Handler (void)
{
    BSP_Ser_Send_Recv (nUART_2);  
}

/*
*********************************************************************************************************
*                                         BSP_Ser_3_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_3_ISR_Handler (void)
{
    BSP_Ser_Send_Recv (nUART_3);  
}

/*
*********************************************************************************************************
*                                         BSP_Ser_4_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_4_ISR_Handler (void)
{
    BSP_Ser_Send_Recv (nUART_4);  
}


/*
*********************************************************************************************************
*                                           BSP_Ser_Printf()
*
* Description : Print formatted data to the output serial port.
*
* Argument(s) : format      String that contains the text to be written.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function output a maximum of BSP_SER_PRINTF_STR_BUF_SIZE number of bytes to the
*                   serial port.  The calling function hence has to make sure the formatted string will
*                   be able fit into this string buffer or hence the output string will be truncated.
*********************************************************************************************************
*/

void  BSP_Ser_Printf (CPU_CHAR  *format, ...)
{
    CPU_CHAR  buf_str[BSP_SER_PRINTF_STR_BUF_SIZE + 1u];
    va_list   v_args;


    va_start(v_args, format);
   (void)vsnprintf((char       *)&buf_str[0],
                   (size_t      ) sizeof(buf_str),
                   (char const *) format,
                                  v_args);
    va_end(v_args);

    BSP_Ser_WrStr(UART_DEBUG, buf_str);	
}



/*
*********************************************************************************************************
*                                                BSP_Ser_RdHex()
*
* Description : This function reads a string from a UART.
*
* Argument(s) : p_str       A pointer to a buffer at which the string can be stored.
*
*               len         The size of the string that will be read.
*
* Return(s)   : none.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT16U  BSP_Ser_RdHex (CPU_INT08U UartIdx, CPU_CHAR  *p_str, CPU_INT16U *p_len, CPU_INT32U TimeOut)
{
	CPU_CHAR     *p_char;
    CPU_BOOLEAN   err;

    p_str[0] = (CPU_CHAR)'\0';
    p_char   = p_str;
		
	*p_len = 0;
	
	while (DEF_TRUE)
    {
		while( gUartGetIndex[UartIdx] != gUartReadIndex[UartIdx] )
		{
			*p_char++ = (CPU_CHAR)gUartRecvBuff[UartIdx][gUartReadIndex[UartIdx]];
			*p_len += 1;
				
			gUartReadIndex[UartIdx] += 1;
			if( gUartReadIndex[UartIdx]>=MAX_UART_BUFF )
				gUartReadIndex[UartIdx] = 0;
		}
		
		if( DEF_OK!=(err=BSP_OS_SemWait(&stUartSem[UartIdx].RxWait, TimeOut)) )         /* Wait until data is received                        */
			break;
    }
	
	
//	err = BSP_OS_SemWait(&stUartSem[UartIdx].Lock, 0);                      /* Obtain access to the serial interface                */
//    if (err != DEF_OK ) 
//	{
//        return err;
//    }

//    while (DEF_TRUE)
//    {
//		if( DEF_OK==(err=BSP_OS_SemWait(&stUartSem[UartIdx].RxWait, TimeOut)) )         /* Wait until data is received                        */
//		{
//			while( gUartGetIndex != gUartReadIndex )
//			{
//				*p_char++ = (CPU_CHAR)gUartRecvBuff[UartIdx][gUartReadIndex];
//				*p_len += 1;
//				
//				gUartReadIndex += 1;
//				if( gUartReadIndex>=MAX_UART_BUFF )
//					gUartReadIndex = 0;
//			}			
//		}
//		else
//		{			
//			break;
//		}
//    }

//	BSP_OS_SemPost(&stUartSem[UartIdx].Lock);                                             /* Release access to the serial interface               */
	
	return err;
}

/*
*********************************************************************************************************
*                                          BSP_Ser_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c           The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Ser_WrByte()
*               BSP_Ser_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
*********************************************************************************************************
*/

void  BSP_Ser_WrByteUnlocked (CPU_INT08U UartIdx, CPU_INT08U c)
{
    USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_TC, ENABLE);
    USART_SendData(stUartCfg[UartIdx].pMode, c);	
	
	BSP_OS_SemWait(&stUartSem[UartIdx].TxWait, 0);
	
    USART_ITConfig(stUartCfg[UartIdx].pMode, USART_IT_TC, DISABLE);
}


/*
*********************************************************************************************************
*                                                BSP_Ser_WrByte()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : tx_byte     The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_WrByte(CPU_INT08U UartIdx, CPU_INT08U  c)
{
	BSP_OS_SemWait(&stUartSem[UartIdx].Lock, 0);                            /* Obtain access to the serial interface              */

    BSP_Ser_WrByteUnlocked(UartIdx, c);
	
	BSP_OS_SemPost(&stUartSem[UartIdx].Lock);                               /* Release access to the serial interface             */
}


/*
*********************************************************************************************************
*                                                BSP_Ser_WrStr()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_WrStr (CPU_INT08U UartIdx, CPU_CHAR  *p_str)
{
    CPU_BOOLEAN  err;


    if (p_str == (CPU_CHAR *)0) 
	{
        return;
    }

	err = BSP_OS_SemWait(&stUartSem[UartIdx].Lock, 0);                      /* Obtain access to the serial interface              */
		
    if (err != DEF_OK ) 
	{
        return;
    }

    while ((*p_str) != (CPU_CHAR )0) 
	{
        if (*p_str == ASCII_CHAR_LINE_FEED) 
		{
            BSP_Ser_WrByteUnlocked(UartIdx, ASCII_CHAR_CARRIAGE_RETURN);
            BSP_Ser_WrByteUnlocked(UartIdx, ASCII_CHAR_LINE_FEED);
            p_str++;
        } 
		else 
		{
            BSP_Ser_WrByteUnlocked(UartIdx, *p_str++);
        }
    }
	
	BSP_OS_SemPost(&stUartSem[UartIdx].Lock);                               /* Release access to the serial interface             */	
}


/*
*********************************************************************************************************
*                                                BSP_Ser_WrHex()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_WrHex (CPU_INT08U UartIdx, CPU_CHAR  *p_str, CPU_INT16U len)
{
    CPU_BOOLEAN  err;


    if( (p_str == (CPU_CHAR *)0)||(0==len) )
	{
        return;
    }

	err = BSP_OS_SemWait(&stUartSem[UartIdx].Lock, 0);                      /* Obtain access to the serial interface              */
		
    if (err != DEF_OK ) 
	{
        return;
    }

    while (len>0) 
	{
        BSP_Ser_WrByteUnlocked(UartIdx, *p_str++);
        
		len--;
    }
	
	BSP_OS_SemPost(&stUartSem[UartIdx].Lock);                               /* Release access to the serial interface             */	
}

/*
*********************************************************************************************************
*                                                UART_SendHexToRValve()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8_t  UART_SendHexToRValve (uint8_t  *p_str, uint8_t len)
{
    BSP_Ser_WrHex (UART_RV, (CPU_CHAR  *)p_str, len);
	
	return 0;
}

/*
*********************************************************************************************************
*                                                UART_SendHexToXLP6000()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8_t  UART_SendHexToXLP6000 (uint8_t  *p_str, uint8_t len)
{
    BSP_Ser_WrHex (UART_SYR, (CPU_CHAR  *)p_str, len);
	
	return 0;
}
