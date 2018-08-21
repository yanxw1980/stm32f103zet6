/*******************************************************************************
 * @file    MotorBsp.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-09-03
 * @brief   定义电机控制端口 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */  
  
#ifndef MOTOR_BSP_H  
#define MOTOR_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.H"
#include "stm32f10x_tim.h" 
#include "BoardIoDef.h" 	 
	 
/* Macros ----------------------------------------------------------------------*/
#ifdef BOARD_REAG_RACK	 
  #define MOTOR_DEF_2	
#else	 
  #undef MOTOR_DEF_2	
#endif
	 
// 电机定义
typedef enum 
{
	MOTOR_1                   = 0,
#if defined(MOTOR_DEF_2)	
	MOTOR_2                      ,
#endif	
	MOTOR_MAX     	             ,	
}emMtrNo;	 

// 电机运动方向定义
enum _tagMotorDirt_
{
	MOTOR_DIR_TO_ORIG     = 0,
	MOTOR_DIR_GO_MAX	      ,	 
};

// GPIO definition
// MOTOR1
#define MTR1_PIN_LPW         GPIO_Pin_13        //高电平：打开低功耗
#define MTR1_PORT_LPW        GPIOF
#define MTR1_CLK_LPW         RCC_APB2Periph_GPIOF

// TIM3_CH3
#define MTR1_PIN_PWM         GPIO_Pin_0
#define MTR1_PORT_PWM        GPIOB
#define MTR1_CLK_PWM         RCC_APB2Periph_GPIOB

#define MTR1_PIN_DIR         GPIO_Pin_11
#define MTR1_PORT_DIR        GPIOF
#define MTR1_CLK_DIR         RCC_APB2Periph_GPIOF

#define MTR1_PIN_EN          GPIO_Pin_12
#define MTR1_PORT_EN         GPIOF
#define MTR1_CLK_EN          RCC_APB2Periph_GPIOF

#define IsMtrClkHi1()        (MTR1_PIN_PWM==(MTR1_PORT_PWM->IDR&MTR1_PIN_PWM))
#define MTR1_CLK_HI()        MTR1_PORT_PWM->BSRR = MTR1_PIN_PWM
#define MTR1_CLK_LW()        MTR1_PORT_PWM->BRR  = MTR1_PIN_PWM
#define MTR1_DIR_HI()        MTR1_PORT_DIR->BSRR = MTR1_PIN_DIR
#define MTR1_DIR_LW()        MTR1_PORT_DIR->BRR  = MTR1_PIN_DIR
#define MTR1_LPW_HI()        
#define MTR1_LPW_LW()        
//#define MTR1_LPW_HI()        MTR1_PORT_LPW->BSRR = MTR1_PIN_LPW
//#define MTR1_LPW_LW()        MTR1_PORT_LPW->BRR  = MTR1_PIN_LPW
#define MTR1_EN_HI()         MTR1_PORT_EN->BSRR = MTR1_PIN_EN
#define MTR1_EN_LW()         MTR1_PORT_EN->BRR  = MTR1_PIN_EN

#define MTR1_PIN_OPTO_HOME   GPIO_Pin_0
#define MTR1_PORT_OPTO_HOME  GPIOC
#define MTR1_CLK_OPTO_HOME   RCC_APB2Periph_GPIOC
#define IsMtr1HomeValid()    (MTR1_PIN_OPTO_HOME==(MTR1_PORT_OPTO_HOME->IDR & MTR1_PIN_OPTO_HOME))

#define MTR1_PIN_OPTO_LIMIT  GPIO_Pin_1
#define MTR1_PORT_OPTO_LIMIT GPIOC
#define MTR1_CLK_OPTO_LIMIT  RCC_APB2Periph_GPIOC
#define IsMtr1LimitValid()   (MTR1_PIN_OPTO_LIMIT==(MTR1_PORT_OPTO_LIMIT->IDR & MTR1_PIN_OPTO_LIMIT))

#if defined(MOTOR_DEF_2)
// MOTOR2
#define MTR2_PIN_LPW         GPIO_Pin_15        //高电平：打开低功耗
#define MTR2_PORT_LPW        GPIOF
#define MTR2_CLK_LPW         RCC_APB2Periph_GPIOF

// TIM2_CH2
#define MTR2_PIN_PWM         GPIO_Pin_1
#define MTR2_PORT_PWM        GPIOA
#define MTR2_CLK_PWM         RCC_APB2Periph_GPIOA

#define MTR2_PIN_DIR         GPIO_Pin_13
#define MTR2_PORT_DIR        GPIOF
#define MTR2_CLK_DIR         RCC_APB2Periph_GPIOF

#define MTR2_PIN_EN          GPIO_Pin_14
#define MTR2_PORT_EN         GPIOF
#define MTR2_CLK_EN          RCC_APB2Periph_GPIOF

#define IsMtrClkHi2()        (MTR2_PIN_PWM==(MTR2_PORT_PWM->IDR&MTR2_PIN_PWM))
#define MTR2_CLK_HI()        MTR2_PORT_PWM->BSRR = MTR2_PIN_PWM
#define MTR2_CLK_LW()        MTR2_PORT_PWM->BRR  = MTR2_PIN_PWM
#define MTR2_DIR_HI()        MTR2_PORT_DIR->BSRR = MTR2_PIN_DIR
#define MTR2_DIR_LW()        MTR2_PORT_DIR->BRR  = MTR2_PIN_DIR
//#define MTR2_LPW_HI()        MTR2_PORT_LPW->BSRR = MTR2_PIN_LPW
//#define MTR2_LPW_LW()        MTR2_PORT_LPW->BRR  = MTR2_PIN_LPW
#define MTR2_LPW_HI()        
#define MTR2_LPW_LW()        
#define MTR2_EN_HI()         MTR2_PORT_EN->BSRR = MTR2_PIN_EN
#define MTR2_EN_LW()         MTR2_PORT_EN->BRR  = MTR2_PIN_EN

#define MTR2_PIN_OPTO_HOME   GPIO_Pin_2
#define MTR2_PORT_OPTO_HOME  GPIOC
#define MTR2_CLK_OPTO_HOME   RCC_APB2Periph_GPIOC
#define IsMtr2HomeValid()    (MTR2_PIN_OPTO_HOME==(MTR2_PORT_OPTO_HOME->IDR & MTR2_PIN_OPTO_HOME))

#define MTR2_PIN_OPTO_LIMIT  GPIO_Pin_3
#define MTR2_PORT_OPTO_LIMIT GPIOC
#define MTR2_CLK_OPTO_LIMIT  RCC_APB2Periph_GPIOC
#define IsMtr2LimitValid()   (MTR2_PIN_OPTO_LIMIT==(MTR2_PORT_OPTO_LIMIT->IDR & MTR2_PIN_OPTO_LIMIT))

#endif       // #if defined(MOTOR_DEF_2)

/* Types    --------------------------------------------------------------------*/	 
	 
/* Structure definition---------------------------------------------------------*/
typedef struct
{
	TIM_TypeDef *pTim;
	IRQn_Type    TimIrq;
	uint8_t      PrePrio;
	uint8_t      SubPrio;
}MOTOR_TIM;	


/* Functions -------------------------------------------------------------------*/
BYTE MOTOR_GpioInit(BYTE nMtrNo);

void MOTOR_SetMotorDirt(BYTE nMtrNo, BYTE Dirt);
void MOTOR_MotorRun(BYTE nMtrNo);
void MOTOR_MotorStop(BYTE MtrNo);

void TIM_InitMotorClock(BYTE nMtrNo, INT32U Freq);
void TIM_ControlMotor(BYTE nMtrNo, uint8_t bDCMtrState);
void TIM_SetMotorFreq(BYTE nMtrNo, INT32U MtrFreq);

BYTE IsMotorHomeValid(BYTE Mask);
BYTE IsMotorLimitValid(BYTE nMtrNo);
#ifdef __cplusplus
}
#endif

#endif


