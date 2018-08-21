/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                            (c) Copyright 2007-2008; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                     Micrium uC-Eval-STM32F107
*                                        Evaluation Board
*
* Filename      : bsp_int.c
* Version       : V1.00
* Programmer(s) : EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_INT_MODULE
#include <bsp.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

//#define  BSP_INT_SRC_NBR                                 43    //STM32F103C8

#define  BSP_INT_SRC_NBR                                 60    

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

static  CPU_FNCT_VOID  BSP_IntVectTbl[BSP_INT_SRC_NBR];


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  BSP_IntHandler     (CPU_DATA  int_id);
static  void  BSP_IntHandlerDummy(void);


/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              BSP_IntClr()
*
* Description : Clear interrupt.
*
* Argument(s) : int_id      Interrupt to clear.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) An interrupt does not need to be cleared within the interrupt controller.
*********************************************************************************************************
*/

void  BSP_IntClr (CPU_DATA  int_id)
{

}


/*
*********************************************************************************************************
*                                              BSP_IntDis()
*
* Description : Disable interrupt.
*
* Argument(s) : int_id      Interrupt to disable.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntDis (CPU_DATA  int_id)
{
    if (int_id < BSP_INT_SRC_NBR) 
	{
        CPU_IntSrcDis(int_id + 16);
    }
}


/*
*********************************************************************************************************
*                                           BSP_IntDisAll()
*
* Description : Disable ALL interrupts.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntDisAll (void)
{
    CPU_IntDis();
}


/*
*********************************************************************************************************
*                                               BSP_IntEn()
*
* Description : Enable interrupt.
*
* Argument(s) : int_id      Interrupt to enable.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntEn (CPU_DATA  int_id)
{
    if (int_id < BSP_INT_SRC_NBR) 
	{
        CPU_IntSrcEn(int_id + 16);
    }
}


/*
*********************************************************************************************************
*                                            BSP_IntVectSet()
*
* Description : Assign ISR handler.
*
* Argument(s) : int_id      Interrupt for which vector will be set.
*
*               isr         Handler to assign
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntVectSet (CPU_DATA       int_id,
                      CPU_FNCT_VOID  isr)
{
    CPU_SR_ALLOC();


    if (int_id < BSP_INT_SRC_NBR) 
	{
	    CPU_CRITICAL_ENTER();
        BSP_IntVectTbl[int_id] = isr;
        CPU_CRITICAL_EXIT();
    }
}


/*
*********************************************************************************************************
*                                            BSP_IntPrioSet()
*
* Description : Assign ISR priority.
*
* Argument(s) : int_id      Interrupt for which vector will be set.
*
*               prio        Priority to assign
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntPrioSet (CPU_DATA    int_id,
                      CPU_INT08U  prio)
{
    CPU_SR_ALLOC();


    if (int_id < BSP_INT_SRC_NBR) 
	{
        CPU_CRITICAL_ENTER();
        CPU_IntSrcPrioSet(int_id + 16, prio);
        CPU_CRITICAL_EXIT();
    }
}


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           INTERNAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                              BSP_IntInit()
*
* Description : Initialize interrupts:
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_IntInit (void)
{
    CPU_DATA  int_id;


    for (int_id = 0; int_id < BSP_INT_SRC_NBR; int_id++) 
	{
        BSP_IntVectSet(int_id, BSP_IntHandlerDummy);
    }
}


/*
*********************************************************************************************************
*                                        BSP_IntHandler####()
*
* Description : Handle an interrupt.
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

void  WWDG_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_WWDG);            }
void  PVD_IRQHandler                (void)  { BSP_IntHandler(BSP_INT_ID_PVD);             }
void  TAMPER_IRQHandler             (void)  { BSP_IntHandler(BSP_INT_ID_TAMPER);          }
void  RTC_IRQHandler                (void)  { BSP_IntHandler(BSP_INT_ID_RTC);             }
void  FLASH_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_FLASH);           }
void  RCC_IRQHandler                (void)  { BSP_IntHandler(BSP_INT_ID_RCC);             }
void  EXTI0_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_EXTI0);           }
void  EXTI1_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_EXTI1);           }
void  EXTI2_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_EXTI2);           }
void  EXTI3_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_EXTI3);           }
void  EXTI4_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_EXTI4);           }
void  DMA1_Channel1_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH1);        }
void  DMA1_Channel2_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH2);        }
void  DMA1_Channel3_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH3);        }
void  DMA1_Channel4_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH4);        }
void  DMA1_Channel5_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH5);        }
void  DMA1_Channel6_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH6);        }
void  DMA1_Channel7_IRQHandler      (void)  { BSP_IntHandler(BSP_INT_ID_DMA1_CH7);        }
void  ADC1_2_IRQHandler             (void)  { BSP_IntHandler(BSP_INT_ID_ADC1_2);          }
void  USB_HP_CAN1_TX_IRQHandler     (void)  { BSP_IntHandler(BSP_INT_ID_CAN1_TX);         }
void  USB_LP_CAN1_RX0_IRQHandler    (void)  { BSP_IntHandler(BSP_INT_ID_CAN1_RX0);        }
void  CAN1_RX1_IRQHandler           (void)  { BSP_IntHandler(BSP_INT_ID_CAN1_RX1);        }
void  CAN1_SCE_IRQHandler           (void)  { BSP_IntHandler(BSP_INT_ID_CAN1_SCE);        }
void  EXTI9_5_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_EXTI9_5);         }
void  TIM1_BRK_IRQHandler           (void)  { BSP_IntHandler(BSP_INT_ID_TIM1_BRK);        }
void  TIM1_UP_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_TIM1_UP);         }
void  TIM1_TRG_COM_IRQHandler       (void)  { BSP_IntHandler(BSP_INT_ID_TIM1_TRG_COM);    }
void  TIM1_CC_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_TIM1_CC);         }
void  TIM2_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_TIM2);            }
void  TIM3_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_TIM3);            }
void  TIM4_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_TIM4);            }
void  I2C1_EV_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_I2C1_EV);         }
void  I2C1_ER_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_I2C1_ER);         }
void  I2C2_EV_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_I2C2_EV);         }
void  I2C2_ER_IRQHandler            (void)  { BSP_IntHandler(BSP_INT_ID_I2C2_ER);         }
void  SPI1_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_SPI1);            }
void  SPI2_IRQHandler               (void)  { BSP_IntHandler(BSP_INT_ID_SPI2);            }
void  USART1_IRQHandler             (void)  { BSP_IntHandler(BSP_INT_ID_USART1);          }
void  USART2_IRQHandler             (void)  { BSP_IntHandler(BSP_INT_ID_USART2);          }
void  USART3_IRQHandler             (void)  { BSP_IntHandler(BSP_INT_ID_USART3);          }
void  UART4_IRQHandler              (void)  { BSP_IntHandler(BSP_INT_ID_USART4);          }
void  EXTI15_10_IRQHandler          (void)  { BSP_IntHandler(BSP_INT_ID_EXTI15_10);       }
void  RTCAlarm_IRQHandler           (void)  { BSP_IntHandler(BSP_INT_ID_RTC_ALARM);       }
void  USBWakeUp_IRQHandler          (void)  { BSP_IntHandler(BSP_INT_ID_OTG_FS_WKUP);     }


/*
*********************************************************************************************************
*********************************************************************************************************
*                                           LOCAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          BSP_IntHandler()
*
* Description : Central interrupt handler.
*
* Argument(s) : int_id          Interrupt that will be handled.
*
* Return(s)   : none.
*
* Caller(s)   : ISR handlers.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  BSP_IntHandler (CPU_DATA  int_id)
{
    CPU_FNCT_VOID  isr;
    CPU_SR_ALLOC();

    CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */

    OSIntEnter();

    CPU_CRITICAL_EXIT();

    if (int_id < BSP_INT_SRC_NBR) 
	{
        isr = BSP_IntVectTbl[int_id];
        if (isr != (CPU_FNCT_VOID)0) 
		{
            isr();
        }
    }

    OSIntExit();                                                /* Tell the OS that we are leaving the ISR            */
}


/*
*********************************************************************************************************
*                                        BSP_IntHandlerDummy()
*
* Description : Dummy interrupt handler.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_IntHandler().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  BSP_IntHandlerDummy (void)
{

}
