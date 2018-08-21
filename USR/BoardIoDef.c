/*******************************************************************************
 * @file    UartApp.c    
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
 
/* Includes ------------------------------------------------------------------*/ 
#include "BoardIoDef.h"
#include "MotorBsp.h"
#include "HexProtocol.h"

/* Private typedef ------------------------------------------------------------*/


/* Private define -------------------------------------------------------------*/


/* Private macro --------------------------------------------------------------*/


/* Private variables -----------------------------------------------------------*/
volatile uint32_t  gSysInputSignal  = 0x00;
//DISPLAY_CTRL   sBuzzerCtrl;

/* Private function prototypes -------------------------------------------------*/
TEE_VALVE_CTRL  stTeeValveCtrl[MAX_TEE_VALVE_NUM];

/* Private functions -----------------------------------------------------------*/ 



/**********************************************************************************
 * Func. Name: EXTI0_Config
 *Discription: Configure PE.00(SYS_EMERGENCY) in interrupt mode 
 *     Inputs:     
 *    Outputs:
 *     Remask:
 *********************************************************************************/
void EXTI0_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
  	// Enable GPIOE clock 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  
  	// Configure PA.00 pin as input floating 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);

  	// Enable AFIO clock 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  	// Connect EXTI0 Line to PE.00 pin 
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource0);

  	// Configure EXTI0 line 
  	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // EXTI_Trigger_Rising
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	// Enable and set EXTI0 Interrupt to the lowest priority 
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}

/**********************************************************************************
 * Func. Name: BSP_BoardIoInit
 *Discription: MCU端口定义和初始化
 *     Inputs:     
 *    Outputs:
 *     Remask:
 *********************************************************************************/
void BSP_BoardIoInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	// LED1
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_LED1, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_LED1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_LED1, &GPIO_InitStructure); 

	// turn off led #1
	MCU_LED1_OFF(); 	

	// LED2
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_LED2, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_LED2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_LED2, &GPIO_InitStructure); 

	// turn off led #2
	MCU_LED2_OFF(); 
	
	// VALVE1
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_VALVE1, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_VALVE1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_VALVE1, &GPIO_InitStructure); 

	// turn off valve #1
	VALVE_1_OFF();
	
	// VALVE2
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_VALVE2, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_VALVE2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_VALVE2, &GPIO_InitStructure); 

	// turn off valve #2
	VALVE_2_OFF();
	
	// VALVE3
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_VALVE3, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_VALVE3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_VALVE3, &GPIO_InitStructure); 

	// turn off valve #3
	VALVE_3_OFF();
	
	// FAN
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(IO_CLK_FAN, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = IO_PIN_FAN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(IO_PORT_FAN, &GPIO_InitStructure); 

	// turn off valve #3
	FAN_OFF();	

	// SPI init
	BSP_Spi1GpioInit();
	BSP_Spi2GpioInit();
}

/*******************************************************************************
 * @brief  init uart gpio and config parameters
 * @input  nUartIdx, uart index, 0~4
 *         Baud, uart baudrate, 9200,19200,38400,57600,115200
 * @return return 0 if success, 1 parameter is wrong, 2 uart is no define
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void BSP_Spi1GpioInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(sADC_SPI_CLK, ENABLE);	
		
	// ADC CS
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sADC_CONV_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sADC_CONV_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(sADC_CONV_PORT, &GPIO_InitStructure); 
	
	sADC_CONV_LOW();
	
	// ADC SCK
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sADC_SPI_SCK_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sADC_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(sADC_SPI_SCK_PORT, &GPIO_InitStructure); 
	
	// ADC MISO
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sADC_SPI_MISO_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sADC_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(sADC_SPI_MISO_PORT, &GPIO_InitStructure);
	
	// ADC MOSI
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sADC_SPI_MOSI_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sADC_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(sADC_SPI_MOSI_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
 * @brief  init uart gpio and config parameters
 * @input  nUartIdx, uart index, 0~4
 *         Baud, uart baudrate, 9200,19200,38400,57600,115200
 * @return return 0 if success, 1 parameter is wrong, 2 uart is no define
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void BSP_Spi2GpioInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(sFLASH_SPI_CLK, ENABLE);
		
	// SPI2 CS
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sFLASH_CS_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sFLASH_CS_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_Init(sFLASH_CS_PORT, &GPIO_InitStructure); 
	
	sFLASH_CS_LOW();
	
	// SPI2 SCK
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sFLASH_SPI_SCK_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sFLASH_SPI_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(sFLASH_SPI_SCK_PORT, &GPIO_InitStructure); 
	
	// SPI2 MISO
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sFLASH_SPI_MISO_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sFLASH_SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(sFLASH_SPI_MISO_PORT, &GPIO_InitStructure);
	
	// SPI2 MOSI
   	// Periph clock enable 
	RCC_APB2PeriphClockCmd(sFLASH_SPI_MOSI_CLK, ENABLE);	
	
	// Configure IO 
	GPIO_InitStructure.GPIO_Pin   = sFLASH_SPI_MOSI_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Init(sFLASH_SPI_MOSI_PORT, &GPIO_InitStructure);
}


/**********************************************************************************
* Func. Name: QueryPinPowLevel
*Discription: 查询指定端口的电平状态
*	  Inputs:	  
*	 Outputs:
*	  Remask:
*********************************************************************************/
BYTE QueryPinPowLevel(BYTE *pSrcDat, BYTE DatLen, BYTE *pSndBuff)
{
	BYTE len;
	BYTE port;
	BYTE pin_from;
	BYTE pin_to;
	BYTE tmp;
	INT16U prtvalue;
	GPIO_TypeDef *pGpioPort;

	port = *pSrcDat++;
	pin_from = *pSrcDat++;
	if( DatLen>2 )
		pin_to = *pSrcDat;
	else
		pin_to = pin_from;

	if( pin_from>pin_to )
	{
		tmp = pin_from;
		pin_from = pin_to;
		pin_to   = tmp;
	}

	if( ('a'==port)||('A'==port)||(0==port) )
		pGpioPort = GPIOA;
	else if( ('b'==port)||('B'==port)||(1==port) )
		pGpioPort = GPIOB;
	else if( ('c'==port)||('C'==port)||(2==port) )
		pGpioPort = GPIOC;
	else if( ('d'==port)||('D'==port)||(3==port) )
		pGpioPort = GPIOD;
	else if( ('e'==port)||('E'==port)||(4==port) )
		pGpioPort = GPIOE;
	else if( ('f'==port)||('F'==port)||(5==port) )
		pGpioPort = GPIOF;
	else if( ('g'==port)||('G'==port)||(6==port) )
		pGpioPort = GPIOG;

	prtvalue = pGpioPort->IDR;

	len = 0;
	while(pin_from<=pin_to )
	{
		if( prtvalue&(0x01ul<<pin_from) )
			*pSndBuff = 1;
		else
			*pSndBuff = 0;

		len++;
		pin_from++;
		pSndBuff++;
	}

	return len;
}

/**********************************************************************************
* Func. Name: SetPinPowLevel
*Discription: 设置指定端口的电平状态
*	  Inputs:	  
*	 Outputs:
*	  Remask:
*********************************************************************************/
void SetPinPowLevel(BYTE port, BYTE pin, BYTE level)
{
	INT16U mask;
	INT16U prtvalue;
	GPIO_TypeDef *pGpioPort;

	if( ('a'==port)||('A'==port)||(0==port) )
		pGpioPort = GPIOA;
	else if( ('b'==port)||('B'==port)||(1==port) )
		pGpioPort = GPIOB;
	else if( ('c'==port)||('C'==port)||(2==port) )
		pGpioPort = GPIOC;
	else if( ('d'==port)||('D'==port)||(3==port) )
		pGpioPort = GPIOD;
	else if( ('e'==port)||('E'==port)||(4==port) )
		pGpioPort = GPIOE;
	else if( ('f'==port)||('F'==port)||(5==port) )
		pGpioPort = GPIOF;
	else if( ('g'==port)||('G'==port)||(6==port) )
		pGpioPort = GPIOG;

	mask = 0x01ul<<pin;
	
	prtvalue = pGpioPort->ODR;

	if( level )
	{
		pGpioPort->ODR = prtvalue|mask;
	}
	else
	{
		pGpioPort->ODR = prtvalue&(~mask);
	}	
}

/*******************************************************************************
 * @brief  valve control runtine
 * @input  pDatBuf, command data
 *         DatLen, data length
 * @return return true if success, false if fail
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE ValveCtrlDirect(u8 nValve, u8 stat)
{
	if( nValve > MAX_TEE_VALVE_NUM )
		return FALSE;
	
	switch( nValve )
	{
		case VALVE_NO_1:
			if( OFF == stat )
				VALVE_1_OFF();
			else 
				VALVE_1_ON();			
			break;
		case VALVE_NO_2:
			if( 0x00 == stat )
				VALVE_2_OFF();
			else
				VALVE_2_ON();			
			break;
		case VALVE_NO_3:
			if( 0x00 == stat )
				VALVE_3_OFF();
			else
				VALVE_3_ON();	
			break;
		default:
			return FALSE;	
	}	
	
	return TRUE;
}

/*******************************************************************************
 * @brief  valve control runtine
 * @input  pDatBuf, command data
 *         DatLen, data length
 * @return return true if success, false if fail
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE ValveCtrlFromProl(u8 *pDatBuf, u8 DatLen, u16 *pWorkTimeMs)
{
	u16 TimeMs;
	
	if( DatLen<4 )
		return FALSE;
	
	if( (void *)0 != pWorkTimeMs )
		*pWorkTimeMs = 0;
	
	switch( pDatBuf[0] )
	{
		case VALVE_NO_1:
			if( 0x00 == pDatBuf[1] )
				VALVE_1_OFF();
			else if( 0x01 == pDatBuf[1] )
				VALVE_1_ON();
			else
				return FALSE;
			break;
		case VALVE_NO_2:
			if( 0x00 == pDatBuf[1] )
				VALVE_2_OFF();
			else if( 0x01 == pDatBuf[1] )
				VALVE_2_ON();
			else
				return FALSE;
			break;
		case VALVE_NO_3:
			if( 0x00 == pDatBuf[1] )
				VALVE_3_OFF();
			else if( 0x01 == pDatBuf[1] )
				VALVE_3_ON();
			else
				return FALSE;
			break;
		default:
			return FALSE;	
	}
	
	TimeMs = pDatBuf[2] + ((u16)pDatBuf[3]<<8);
	
	if( (void *)0 != pWorkTimeMs )
		*pWorkTimeMs = TimeMs;
	
	// update control parameter
	if( (0x01==pDatBuf[1]) && (TimeMs>0) )
	{
		stTeeValveCtrl[pDatBuf[0] - 1].Stat = ON;
		stTeeValveCtrl[pDatBuf[0] - 1].LastN100ms = TimeMs;
	}
	
	return TRUE;
}
