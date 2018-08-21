/*******************************************************************************
 * @file    BoardIoDef.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-09-03
 * @brief   PCB板上硬件端口、位号和编号等定义 (STM32F103ZE: V = 144 pins, C = 256 Kbytes of Flash memory, T = LQFP)
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */  

#ifndef __BOARD_IO_H_
#define __BOARD_IO_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ---------------------------------------------------------------------*/ 
#include "TYPE.h"
#include "stm32f10x.h"
//#include "MotorDrv.h"
	 
/* Macros ----------------------------------------------------------------------*/
// System Freq definition
#define FREQ_SYSCLK        72000000
#define FREQ_HCLK          72000000		
#define FREQ_PCLK1         36000000	 
#define FREQ_PCLK2         72000000

#define _DEBUG_

//#define BOARD_REAG_RACK    // code for reagent rack
#undef BOARD_REAG_RACK  
	 
// GPIO Definitions
// LED1
#define IO_PIN_LED1               GPIO_Pin_5  
#define IO_PORT_LED1              GPIOB
#define IO_CLK_LED1               RCC_APB2Periph_GPIOB 
#define MCU_LED1_ON()             IO_PORT_LED1->BRR = IO_PIN_LED1
#define MCU_LED1_OFF()            IO_PORT_LED1->BSRR= IO_PIN_LED1
#define MCU_LED1_TOG()            IO_PORT_LED1->ODR^= IO_PIN_LED1

// LED2
#define IO_PIN_LED2               GPIO_Pin_5  
#define IO_PORT_LED2              GPIOE
#define IO_CLK_LED2               RCC_APB2Periph_GPIOE 
#define MCU_LED2_ON()             IO_PORT_LED2->BRR = IO_PIN_LED2
#define MCU_LED2_OFF()            IO_PORT_LED2->BSRR= IO_PIN_LED2
#define MCU_LED2_TOG()            IO_PORT_LED2->ODR^= IO_PIN_LED2

// Valve1
#define IO_PIN_VALVE1             GPIO_Pin_1  
#define IO_PORT_VALVE1            GPIOF
#define IO_CLK_VALVE1             RCC_APB2Periph_GPIOF 
#define VALVE_1_ON()              IO_PORT_VALVE1->BSRR = IO_PIN_VALVE1
#define VALVE_1_OFF()             IO_PORT_VALVE1->BRR  = IO_PIN_VALVE1
#define IsValve1On()              (IO_PORT_VALVE1->IDR & IO_PIN_VALVE1)

// Valve2
#define IO_PIN_VALVE2             GPIO_Pin_0  
#define IO_PORT_VALVE2            GPIOF
#define IO_CLK_VALVE2             RCC_APB2Periph_GPIOF 
#define VALVE_2_ON()              IO_PORT_VALVE2->BSRR = IO_PIN_VALVE2
#define VALVE_2_OFF()             IO_PORT_VALVE2->BRR  = IO_PIN_VALVE2
#define IsValve2On()              (IO_PORT_VALVE2->IDR & IO_PIN_VALVE2)

// Valve3
#define IO_PIN_VALVE3             GPIO_Pin_2  
#define IO_PORT_VALVE3            GPIOF
#define IO_CLK_VALVE3             RCC_APB2Periph_GPIOF 
#define VALVE_3_ON()              IO_PORT_VALVE3->BSRR = IO_PIN_VALVE3
#define VALVE_3_OFF()             IO_PORT_VALVE3->BRR  = IO_PIN_VALVE3
#define IsValve3On()              (IO_PORT_VALVE3->IDR & IO_PIN_VALVE3)

// Fan
#define IO_PIN_FAN                GPIO_Pin_3  
#define IO_PORT_FAN               GPIOF
#define IO_CLK_FAN                RCC_APB2Periph_GPIOF 
#define FAN_ON()                  IO_PORT_FAN->BRR = IO_PIN_FAN
#define FAN_OFF()                 IO_PORT_FAN->BSRR= IO_PIN_FAN
#define IsFanOn()                 (IO_PORT_FAN->IDR & IO_PIN_FAN)

// SPI flash
#define sFLASH_SPI                SPI2
#define sFLASH_SPI_CLK            RCC_APB1Periph_SPI2

#define sFLASH_SPI_SCK_PIN        GPIO_Pin_13                 // PB.13 
#define sFLASH_SPI_SCK_PORT       GPIOB                       // GPIOB 
#define sFLASH_SPI_SCK_CLK        RCC_APB2Periph_GPIOB

#define sFLASH_SPI_MISO_PIN       GPIO_Pin_14                 // PB.14 
#define sFLASH_SPI_MISO_PORT      GPIOB                       // GPIOB 
#define sFLASH_SPI_MISO_CLK       RCC_APB2Periph_GPIOB

#define sFLASH_SPI_MOSI_PIN       GPIO_Pin_15                 // PB.15 
#define sFLASH_SPI_MOSI_PORT      GPIOB                       // GPIOB 
#define sFLASH_SPI_MOSI_CLK       RCC_APB2Periph_GPIOB

#define sFLASH_CS_PIN             GPIO_Pin_12                 // PB.12 
#define sFLASH_CS_PORT            GPIOB                       // GPIOB 
#define sFLASH_CS_CLK             RCC_APB2Periph_GPIOB	   

// @brief  Select sFLASH: Chip Select pin low
#define sFLASH_CS_LOW()           sFLASH_CS_PORT->BRR = sFLASH_CS_PIN

//  Deselect sFLASH: Chip Select pin high
#define sFLASH_CS_HIGH()          sFLASH_CS_PORT->BSRR= sFLASH_CS_PIN


// SPI ADC
#define sADC_SPI                  SPI1
#define sADC_SPI_CLK              RCC_APB2Periph_SPI1

#define sADC_SPI_SCK_PIN          GPIO_Pin_5                  // PA.05 
#define sADC_SPI_SCK_PORT         GPIOA                       // GPIOA 
#define sADC_SPI_SCK_CLK          RCC_APB2Periph_GPIOA

#define sADC_SPI_MISO_PIN         GPIO_Pin_6                  // PA.06 
#define sADC_SPI_MISO_PORT        GPIOA                       // GPIOA 
#define sADC_SPI_MISO_CLK         RCC_APB2Periph_GPIOA

#define IsConverting()            (sADC_SPI_MISO_PIN == (sADC_SPI_MISO_PORT->IDR & sADC_SPI_MISO_PIN))

#define sADC_SPI_MOSI_PIN         GPIO_Pin_7                  // PA.07 
#define sADC_SPI_MOSI_PORT        GPIOA                       // GPIOA 
#define sADC_SPI_MOSI_CLK         RCC_APB2Periph_GPIOA

#define sADC_CONV_PIN             GPIO_Pin_4                  // PA.04 
#define sADC_CONV_PORT            GPIOA                       // GPIOE 
#define sADC_CONV_CLK             RCC_APB2Periph_GPIOA	   

#define sADC_CONV_LOW()           sADC_CONV_PORT->BRR = sADC_CONV_PIN
#define sADC_CONV_HIGH()          sADC_CONV_PORT->BSRR= sADC_CONV_PIN
#define IsConvPowerLow()          (sADC_CONV_PIN != (sADC_CONV_PORT->IDR & sADC_CONV_PIN))

/* Types    --------------------------------------------------------------------*/	 
// status definition
typedef enum
{
	MODULE_STAT_DISCONNECT    = 0,	       // 未联接
	MODULE_STAT_NORMAL        = 1,         // 正常状态
	MODULE_STAT_EXECUTING     = 2,         // 指令执行中
	
	MODULE_STAT_ERR_COMM      = 10,        // 联接异常
	MODULE_STAT_ERR_TIMEOUT   = 11,        // 正常联接后,指令执行异常
	MODULE_STAT_ERR_PARA      = 12,        // 指令或参数问题
	MODULE_STAT_ERR_MACHINE   = 13,
}emModuleStat;

/* Structure definition---------------------------------------------------------*/
#define MAX_TEE_VALVE_NUM       3

// 三通阀控制
__packed
typedef struct
{	
	u8  Stat;           // 状态  0 - off, other - on
	u32 LastN100ms;     // off if the valve is not zero, and equal zero after minus 1 
}TEE_VALVE_CTRL, *P_TEE_VALVE_CTRL;

// 串口中断,时钟和优先级配置
//__packed
//typedef struct
//{	
//	USART_TypeDef *pMode;
//	uint8_t  bWork;
//	uint32_t Baud;
//	uint32_t Prio;
//	uint32_t IRQn;
//	uint32_t CLKI;
//}UART_CONFIG, *P_UART_CONFIG;

/* Functions -------------------------------------------------------------------*/
void BSP_BoardIoInit(void);

void BSP_Spi1GpioInit(void);
void BSP_Spi2GpioInit(void);
BYTE QueryPinPowLevel(BYTE *pSrcDat, BYTE DatLen, BYTE *pSndBuff);
void SetPinPowLevel(BYTE port, BYTE pin, BYTE level);

BYTE ValveCtrlFromProl(u8 *pDatBuf, u8 DatLen, u16 *pWorkTimeMs);
BYTE ValveCtrlDirect(u8 nValve, u8 stat);

#ifdef __cplusplus
}
#endif
  
#endif // __BSP_H_ 	 

extern TEE_VALVE_CTRL  stTeeValveCtrl[MAX_TEE_VALVE_NUM];

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/ 
