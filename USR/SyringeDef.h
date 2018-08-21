/*******************************************************************************
 * @file    SyringeDef.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-11-23
 * @brief   
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef SYRINGE_APP_H  
#define SYRINGE_APP_H
#define  OS_EXT 

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ---------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"

/* Macros ----------------------------------------------------------------------*/
#define HEADER     '/'
#define CR         0x0D
#define LF         0x0A

typedef enum _tagError
{	
	SYR_OK                    = 0x40, 
	SYR_ERR_INIT              = 0x41,       // intialization
	SYR_ERR_INVALID_COMMAND   = 0x42,
	SYR_ERR_INVALID_OPERAND   = 0x43,
	SYR_ERR_INVALID_COMM_SEQ  = 0x44,
	SYR_ERR_INVALID_EEPROM    = 0x46,
	SYR_ERR_NOT_INITIALIZED   = 0x47,
	SYR_ERR_PLUNGER_OVERLOAD  = 0x49,
	SYR_ERR_VALVE_OVERLOAD    = 0x4A,
	SYR_ERR_MORE_PLUNGER      = 0x4B,
	SYR_ERR_COMMAND_OVERFLOW  = 0x4F,
}emSYR_ERROR;
	 
/* Types    ---------------------------------------------------------------------*/

/* Structure definition-------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
BYTE SYR_InitSystem( BYTE );
BYTE SYR_QueryStatus( BYTE );
	 
#ifdef __cplusplus
}
#endif

#endif


