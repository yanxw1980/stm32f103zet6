/*******************************************************************************
 * @file    RotValveDef.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-11-23
 * @brief   
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef RR_VALVE_APP_H  
#define RR_VALVE_APP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"

/* Macros ----------------------------------------------------------------------*/
#define MAX_RV_RUN_TIME_MS     6000ul 
#define MAX_RV_PARA_TIME_MS    1000ul 
	 
#define RV_MAX_PORT            25
	 
// error definition	 
#define RV_ERR_VALVE_FAIL      0x63	   // valve can not be homed
#define RV_ERR_MEMORY          0x58	   // non-volatile memory error
#define RV_ERR_CONFIGURATION   0x4D	   // valve configuration error or command mode error	 
#define RV_ERR_POSITION        0x42    // valve positioning error
#define RV_ERR_DATA_INTEGRITY  0x37    // data integrity error
#define RV_ERR_DATA_CRC        0x2C    // data CRC error	 
	 
/* Types    ---------------------------------------------------------------------*/

/* Structure definition-------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
BYTE RV_MoveToPosition( BYTE ValvePos );
BYTE RV_QueryStatus( void );
BYTE RV_Home( void );
BYTE RV_QueryVersion( void );

void RV_SetCurrStat(u8 stat);
u8   RV_GetCurrStat(void);
void RV_SetErrorNumber(u16 err);
u16  RV_GetErrorNumber(void);

#ifdef __cplusplus
}
#endif

#endif


