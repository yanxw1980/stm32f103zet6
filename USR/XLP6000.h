/*******************************************************************************
 * @file    XLP6000.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-08
 * @brief   
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef PUMP_XLP6000_APP_H  
#define PUMP_XLP6000_APP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"

/* Macros ----------------------------------------------------------------------*/
#define SYR_VAL_CHN_NONE    0		 
#define SYR_VAL_CHN_1_SYR   1	       // port 1 connect to plunger
#define SYR_VAL_CHN_2_SYR   2          // port 2 connect to plunger
#define SYR_VAL_CHN_1_2     3          // port 1 connect to port 2, bypass  
#define SYR_VAL_CHN_MAX     3

#define SYS_TUBE_VOLUME     2500       //单管容量UL
#define SYS_TUBE_NUMBER     2          //两条管
	 
#define MTR_MODEL_NORM      0          // 整步
#define MTR_MODEL_MICRO     1          // 细分

// speed settings
#define SPEED_START_MIN     50
#define SPEED_START_MAX     1000

#define SPEED_MAX_MIN       50
#define SPEED_MAX_MAX       6000

#define SPEED_STOP_MIN      50
#define SPEED_STOP_MAX      2700

// status definition
#define XLP_COMM_BUSY       0x40
#define XLP_COMM_IDLE       0x60

#define SetSyringeSpeed(v)  (v)

#define MAX_SYR_HOME_TIME_MS         9000ul 
#define MAX_SYR_ASPR_TIME_MS         200000ul 
#define MAX_SYR_DRAIN_TIME_MS        200000ul 

// error definition	 
#define PUMP_ERR_NO_ERROR            0x40	   // No Error
#define PUMP_ERR_INITIAL             0x41	   // Initialization
#define PUMP_ERR_INVALID_COMM        0x42	   // Invalid command	 
#define PUMP_ERR_INVALID_OPER        0x43      // Invalid Operand
#define PUMP_ERR_INVALID_COMM_SQ     0x44      // Invalid command seq.
#define PUMP_ERR_EEPROM              0x46      // EEPROM failure	 
#define PUMP_ERR_NOT_INITIAL         0x47      // Device not initialized
#define PUMP_ERR_PLUNGER_OVERLOAD    0x49      // Plunger overload
#define PUMP_ERR_VALVE_OVERLOAD      0x4A      // Valve overload
#define PUMP_ERR_NOT_ALLOWED         0x4B      // Plunger move not allowed
#define PUMP_ERR_COMM_OVERFLOW       0x4F      // Command overflow
	 
/* Types    ---------------------------------------------------------------------*/

/* Structure definition-------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
u8 XLP_Home(u8 *pstrBuf, u32 VolxUl, u8 InValveNo, u8 OutValveNo);
u8 XLP_ValveOn(u8 *pstrBuf, u8 ValveNo);
u8 XLP_AbsMove(u8 *pstrBuf, u8 ValveNo, u8 Model, u16 Pulse);
u8 XLP_RelInhale(u8 *pstrBuf, u8 ValveNo, u8 Model, u8 Speed, u16 Pulse);
u8 XLP_RelDrain(u8 *pstrBuf, u8 ValveNo, u8 Model, u8 Speed, u16 Pulse);
u8 XLP_SetSpeed(u8 *pstrBuf, u16 StartSpeed, u16 MaxSpeed, u16 StopSpeed);
u8 XLP_QueryStatus(u8 *pstrBuf);


void  XLP_SetCurrStat(u8 stat);
u8    XLP_GetCurrStat(void);
void  XLP_SetErrorNumber(u16 err);
u16   XLP_GetErrorNumber(void);

#ifdef __cplusplus
}
#endif

#endif


