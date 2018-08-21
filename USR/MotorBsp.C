/*******************************************************************************
 * @file    MotorBsp.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-09-03
 * @brief   电机端口定义与基础驱动
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes -----------------------------------------------------------------*/ 
#include "MotorBsp.H"
#include "MotorDrv.H"
#include "stdio.h"
#include  <bsp.h>

/* Private typedef -----------------------------------------------------------*/
typedef void (*PFUNC_RCC_APB_CLOCK)(uint32_t, FunctionalState);

/* Private define -------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
const MOTOR_TIM stMotorTim[MOTOR_MAX] =
{
	{TIM3, TIM3_IRQn,    1, 1},
#if defined(MOTOR_DEF_2)	
	{TIM2, TIM2_IRQn,    1, 2},
#endif	
};

/* Private variables -----------------------------------------------------------*/

/* Private function prototypes --------------------------------------------------*/

static void MOTOR_RunCallBack(BYTE nMtrNo);

#if defined(MOTOR_DEF_2)
static void BSP_Timer_2_ISR_Handler(void);
#endif

static void BSP_Timer_3_ISR_Handler(void);

/* Private functions -----------------------------------------------------------*/ 

/*******************************************************************************
 * @brief  电机端口初始化 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE MOTOR_GpioInit(BYTE nMtrNo)     
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	if( nMtrNo>MOTOR_MAX )
		return FALSE;
	
	if( MOTOR_1==nMtrNo )
	{
		// MOTOR 1
	//	RCC_APB2PeriphClockCmd( MTR1_CLK_LPW, ENABLE );	
	//	GPIO_InitStructure.GPIO_Pin  = MTR1_PIN_LPW;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	//	GPIO_Init( MTR1_PORT_LPW, &GPIO_InitStructure );

		RCC_APB2PeriphClockCmd( MTR1_CLK_PWM, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR1_PIN_PWM;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR1_PORT_PWM, &GPIO_InitStructure );

		RCC_APB2PeriphClockCmd( MTR1_CLK_EN, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR1_PIN_EN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR1_PORT_EN, &GPIO_InitStructure );
		
		RCC_APB2PeriphClockCmd( MTR1_CLK_DIR, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR1_PIN_DIR;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR1_PORT_DIR, &GPIO_InitStructure );
		
		MOTOR_MotorStop(MOTOR_1);
		
		BSP_IntVectSet(BSP_INT_ID_TIM3, BSP_Timer_3_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_TIM3);
		
		// Optic coupler gpio init
		// Optic coupler 1
		// Periph clock enable 
		RCC_APB2PeriphClockCmd(MTR1_CLK_OPTO_HOME, ENABLE);	
		
		// Configure IO 
		GPIO_InitStructure.GPIO_Pin   = MTR1_PIN_OPTO_HOME;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
		GPIO_Init(MTR1_PORT_OPTO_HOME, &GPIO_InitStructure); 
		
		// Optic coupler 2
		// Periph clock enable 
		RCC_APB2PeriphClockCmd(MTR1_CLK_OPTO_LIMIT, ENABLE);	
		
		// Configure IO 
		GPIO_InitStructure.GPIO_Pin   = MTR1_PIN_OPTO_LIMIT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
		GPIO_Init(MTR1_PORT_OPTO_LIMIT, &GPIO_InitStructure); 		
	}
#if defined(MOTOR_DEF_2)
	else if( MOTOR_2==nMtrNo )
	{
		// MOTOR 2
	//	RCC_APB2PeriphClockCmd( MTR2_CLK_LPW, ENABLE );	
	//	GPIO_InitStructure.GPIO_Pin  = MTR2_PIN_LPW;
	//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	//	GPIO_Init( MTR2_PORT_LPW, &GPIO_InitStructure );

		RCC_APB2PeriphClockCmd( MTR2_CLK_PWM, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR2_PIN_PWM;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR2_PORT_PWM, &GPIO_InitStructure );

		RCC_APB2PeriphClockCmd( MTR2_CLK_EN, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR2_PIN_EN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR2_PORT_EN, &GPIO_InitStructure );
		
		RCC_APB2PeriphClockCmd( MTR2_CLK_DIR, ENABLE );	
		GPIO_InitStructure.GPIO_Pin  = MTR2_PIN_DIR;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
		GPIO_Init( MTR2_PORT_DIR, &GPIO_InitStructure );
		
		MOTOR_MotorStop(MOTOR_2);
		
		BSP_IntVectSet(BSP_INT_ID_TIM2, BSP_Timer_2_ISR_Handler);
		BSP_IntEn(BSP_INT_ID_TIM2);
		
		// Optic coupler gpio init
		// Periph clock enable 		
		RCC_APB2PeriphClockCmd(MTR2_CLK_OPTO_HOME, ENABLE);	
		
		// Configure IO 
		GPIO_InitStructure.GPIO_Pin   = MTR2_PIN_OPTO_HOME;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
		GPIO_Init(MTR2_PORT_OPTO_HOME, &GPIO_InitStructure); 
		
		// Optic coupler 2
		// Periph clock enable 
		RCC_APB2PeriphClockCmd(MTR2_CLK_OPTO_LIMIT, ENABLE);	
		
		// Configure IO 
		GPIO_InitStructure.GPIO_Pin   = MTR2_PIN_OPTO_LIMIT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
		GPIO_Init(MTR2_PORT_OPTO_LIMIT, &GPIO_InitStructure);
	}
#endif			
	
	return TRUE;
}

/*******************************************************************************
 * @brief  查询光藕状态
 * @input  Mask, 
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE IsMotorHomeValid(BYTE nMtrNo)
{	
	if( MOTOR_1==nMtrNo )
		return IsMtr1HomeValid();
#if defined(MOTOR_DEF_2)
	else if( MOTOR_2==nMtrNo )
		return IsMtr2HomeValid();
#endif
	return 0;
}

/*******************************************************************************
 * @brief  查询光藕状态
 * @input  Mask, 
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE IsMotorLimitValid(BYTE nMtrNo)
{	
	if( MOTOR_1==nMtrNo )
		return IsMtr1LimitValid();
#if defined(MOTOR_DEF_2)
	else if( MOTOR_2==nMtrNo )
		return IsMtr2LimitValid();
#endif
	return 0;
}

/*******************************************************************************
 * @brief  定时器2中断入口 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
#if defined(MOTOR_DEF_2)
static void BSP_Timer_2_ISR_Handler(void)
{	
	if( TIM_GetITStatus(TIM2, TIM_IT_Update) )
		MOTOR_RunCallBack(MOTOR_2);	
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
#endif

/*******************************************************************************
 * @brief  定时器3中断入口 
 * @input  
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void BSP_Timer_3_ISR_Handler(void)
{
	if( TIM_GetITStatus(TIM3, TIM_IT_Update) )
		MOTOR_RunCallBack(MOTOR_1);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}



/*******************************************************************************
 * @brief  电机脉冲回调函数 
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
static void MOTOR_RunCallBack(BYTE nMtrNo)
{
	if( nMtrNo>=MOTOR_MAX )
		return ;
	
	MOTOR_PulseUpdate(nMtrNo, TRUE);	
}

/*******************************************************************************
 * @brief  设置电机运行方向 
 * @input  MtrNo, 电机编号
 *         Dirt,  方向
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MOTOR_SetMotorDirt(BYTE MtrNo, BYTE Dirt)          
{
	if( MOTOR_1==(MtrNo) )                                      
    {                                                            
		if( MOTOR_DIR_GO_MAX==Dirt )   
			MTR1_DIR_HI();
        else
 			MTR1_DIR_LW();
	} 
#if defined(MOTOR_DEF_2)	
	else if( MOTOR_2==(MtrNo) )                                 
	{                                                            
	    if( MOTOR_DIR_GO_MAX==Dirt )   
			MTR2_DIR_HI();
        else
 			MTR2_DIR_LW();                          
	} 
#endif
}

/*******************************************************************************
 * @brief  电机运行 
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void MOTOR_MotorRun(BYTE MtrNo)          
{
	if( MOTOR_1==(MtrNo) )                                      
    {              
	    TIM_ControlMotor((MtrNo), ON);  		
	}  
#if defined(MOTOR_DEF_2)	
	else if( MOTOR_2==(MtrNo) )                                 
	{             
	    TIM_ControlMotor((MtrNo), ON);                           
	} 
#endif	
}

/*******************************************************************************
 * @brief  电机停止
 * @input  MtrNo, 电机编号
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void  MOTOR_MotorStop(BYTE MtrNo)          
{	
	if( MOTOR_1==(MtrNo) )                                      
    {                                                            
		TIM_ControlMotor((MtrNo), OFF);     
	} 
#if defined(MOTOR_DEF_2)	
	else if( MOTOR_2==(MtrNo) )                                 
	{                                                            
	    TIM_ControlMotor((MtrNo), OFF);    
	}
#endif	
}

/*******************************************************************************
 * @brief  获取定时器对应的总线时钟
 * @input  pTim, 定时器寄存器地址
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
PFUNC_RCC_APB_CLOCK RCC_GetAPBClock(void *pTim)
{
	if( (TIM1==(TIM_TypeDef *)pTim)
      ||(TIM8==(TIM_TypeDef *)pTim))
	{
		return RCC_APB2PeriphClockCmd;
	}
	else if( (TIM2==(TIM_TypeDef *)pTim)
           ||(TIM3==(TIM_TypeDef *)pTim)
	       ||(TIM4==(TIM_TypeDef *)pTim)
	       ||(TIM5==(TIM_TypeDef *)pTim)
	       ||(TIM6==(TIM_TypeDef *)pTim)
	       ||(TIM7==(TIM_TypeDef *)pTim))
	{
		return RCC_APB1PeriphClockCmd;
	}	
	
	return 0;
}


/*******************************************************************************
 * @brief  获取定时器对应的时钟
 * @input  pTim, 定时器寄存器地址
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
uint32_t TIM_GetTimeClock(TIM_TypeDef *pTim)
{
	if( TIM1==pTim )
		return RCC_APB2Periph_TIM1;
	else if( TIM2==pTim )
		return RCC_APB1Periph_TIM2;
	else if( TIM3==pTim )
		return RCC_APB1Periph_TIM3;
	else if( TIM4==pTim )
		return RCC_APB1Periph_TIM4;
	else if( TIM5==pTim )
		return RCC_APB1Periph_TIM5;
	else if( TIM6==pTim )
		return RCC_APB1Periph_TIM6;
	else if( TIM7==pTim )
		return RCC_APB1Periph_TIM7;
	else if( TIM8==pTim )
		return RCC_APB2Periph_TIM8;
	
	return 0;	
}

/*******************************************************************************
 * @brief  初始化电机对应的定时器
 * @input  nMtrNo, 电机编号
 *         Freq,   频率
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void TIM_InitMotorClock(BYTE nMtrNo, INT32U Freq)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	PFUNC_RCC_APB_CLOCK pRccApbClock;
	
	if( nMtrNo>=MOTOR_MAX )
		return ;	
		
	pRccApbClock = RCC_GetAPBClock( stMotorTim[nMtrNo].pTim );
	
	// clocks enable 
	(*pRccApbClock)(TIM_GetTimeClock(stMotorTim[nMtrNo].pTim), ENABLE);

	// Set Frequence
	TIM_SetMotorFreq(nMtrNo, Freq);
	
	TIM_ARRPreloadConfig(stMotorTim[nMtrNo].pTim, ENABLE);
	
	// TIM IT enable 
	TIM_ITConfig(stMotorTim[nMtrNo].pTim, TIM_IT_Update, DISABLE);

	// Enable Interrupt 
	NVIC_InitStructure.NVIC_IRQChannel                   = stMotorTim[nMtrNo].TimIrq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = stMotorTim[nMtrNo].PrePrio;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = stMotorTim[nMtrNo].SubPrio;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 	

	TIM_ClearITPendingBit(stMotorTim[nMtrNo].pTim, TIM_IT_Update);
	
	// TIM IT enable 
	TIM_ITConfig(stMotorTim[nMtrNo].pTim, TIM_IT_Update, ENABLE);	
}

/*******************************************************************************
 * @brief  定时器开关控制
 * @input  nMtrNo, 电机编号
 *         bDCMtrState,   控制
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void TIM_ControlMotor(BYTE nMtrNo, uint8_t bDCMtrState)
{
	if( nMtrNo>=MOTOR_MAX )
		return ;	
		
	if( ON==bDCMtrState )
	{
		// TIM IT enable 
		TIM_ClearITPendingBit(stMotorTim[nMtrNo].pTim, TIM_IT_Update);		
		TIM_ITConfig(stMotorTim[nMtrNo].pTim, TIM_IT_Update, ENABLE);
		
		TIM_Cmd(stMotorTim[nMtrNo].pTim, ENABLE);			
	}
	else			
	{
		TIM_Cmd(stMotorTim[nMtrNo].pTim, DISABLE);	
		
		// TIM IT disable 
		TIM_ITConfig(stMotorTim[nMtrNo].pTim, TIM_IT_Update, DISABLE);
		TIM_ClearITPendingBit(stMotorTim[nMtrNo].pTim, TIM_IT_Update);
	}
}

/*******************************************************************************
 * @brief  设置定时器频率
 * @input  nMtrNo, 电机编号
 *         MtrFreq, 频率
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void TIM_SetMotorFreq(BYTE nMtrNo, INT32U MtrFreq)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef        TIM_OCInitStructure; 
	uint16_t  usDutyPeriod;
	uint16_t  usDivScaler;
	uint16_t  usPosTime;
	uint32_t  ApbFreq;
	PFUNC_RCC_APB_CLOCK pRccApbClock;	
	
	if( nMtrNo>=MOTOR_MAX )
		return ;
	
	if( 0==MtrFreq )
		return ;	
	
	// 获取时钟频率
	pRccApbClock = RCC_GetAPBClock( stMotorTim[nMtrNo].pTim );
	
	if( RCC_APB1PeriphClockCmd==pRccApbClock )
	{
		ApbFreq = FREQ_PCLK1;
		
		ApbFreq <<= 1;
	}
	else
	{
		ApbFreq = FREQ_PCLK2;	
	}	
	
	// 计算分频系数
	usDivScaler = 0;
	if( MtrFreq<10 )
		usDivScaler = 1199;
	else if( MtrFreq<20 )
		usDivScaler = 599;
	else if( MtrFreq<200 )
		usDivScaler = 59;
	else if( MtrFreq<400 )
		usDivScaler = 5;
	else if( MtrFreq<800 )
		usDivScaler = 3;
	else if( MtrFreq<1200 )
		usDivScaler = 1;
	
	// 频率设定
	usDutyPeriod = ApbFreq/(usDivScaler+1)/MtrFreq - 1;

	// TIM configuration 
	TIM_TimeBaseStructure.TIM_Prescaler     = usDivScaler;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
	TIM_TimeBaseStructure.TIM_Period        = usDutyPeriod; 		 
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  
	
	// should be setted if tim1 or tim8
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(stMotorTim[nMtrNo].pTim, &TIM_TimeBaseStructure);		

	usPosTime = usDutyPeriod>>1;
	
	// PWM Mode configuration: Channel2  
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;                           
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_OutputNState= TIM_OutputState_Disable;	 //This parameter is valid only for TIM1 and TIM8
    TIM_OCInitStructure.TIM_Pulse       = usPosTime;                                       
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;       
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_Low;	     //This parameter is valid only for TIM1 and TIM8
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;     //This parameter is valid only for TIM1 and TIM8
//	TIM_OCInitStructure.TIM_OCNIdleState= TIM_OCNIdleState_Reset; 
  
	if( MOTOR_1==nMtrNo )
	{
		TIM_OC3Init(stMotorTim[nMtrNo].pTim, &TIM_OCInitStructure);  		
		TIM_OC3PreloadConfig(stMotorTim[nMtrNo].pTim, TIM_OCPreload_Enable);  
	}
#if defined(MOTOR_DEF_2)	
	else if( MOTOR_2==nMtrNo )
	{
		TIM_OC2Init(stMotorTim[nMtrNo].pTim, &TIM_OCInitStructure);  		
		TIM_OC2PreloadConfig(stMotorTim[nMtrNo].pTim, TIM_OCPreload_Enable);  
	}
#endif	
}


