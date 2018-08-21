/*******************************************************************************
 * @file    SysError.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-13
 * @brief   系统产生的错误码定义 
 ******************************************************************************
 * @attention  
 * 
 ******************************************************************************
 */ 
 
#ifndef SYS_ERROR_H  
#define SYS_ERROR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes --------------------------------------------------------------------*/
#include "TYPE.h"
#include "stm32f10x.h"

/* Macros ----------------------------------------------------------------------*/
#define SYS_ERR_BASE_SYR      0x20
#define SYS_ERR_BASE_RV       0x40
#define SYS_ERR_BASE_MTR1     0x80
#define SYS_ERR_BASE_MTR2     0xA0
	 
// system error definition
typedef enum
{
	SYS_ERR_NONE             = 0,
	SYS_ERR_MEMORY_LESS      = 0x01,
	
	// XLP error
	SYS_ERR_XLP_COMM         = SYS_ERR_BASE_SYR,   // 注射泵指令错误
	SYS_ERR_XLP_PARA           ,                   // 注射泵参数错误
	SYS_ERR_XLP_TIMEOUT        ,                   // 执行超时
	SYS_ERR_XLP_RETURN_UNKOW   ,                   // 返回码不能识别
	
	SYS_ERR_XLP_INTERNAL       ,                   // 内部错误码
	
	// RV error
	SYS_ERR_RV_COMM          = SYS_ERR_BASE_RV,    // 旋转阀指令错误
	SYS_ERR_RV_PARA            ,                   // 旋转阀参数错误
	SYS_ERR_RV_TIMEOUT         ,                   // 执行超时
	SYS_ERR_RV_RETURN_UNKOW    ,                   // 返回码不能识别
	
	SYS_ERR_RV_INTERNAL        ,                   // 内部错误码
	
	// motor error
	SYS_ERR_MTR1_COMM          = SYS_ERR_BASE_MTR1,  // 电机指令错误
	SYS_ERR_MTR1_PARA            ,                   // 电机阀参数错误
	SYS_ERR_MTR1_TIMEOUT         ,                   // 执行超时
	SYS_ERR_MTR1_NO_RESET        ,                   // 未复位
	SYS_ERR_MTR1_NON_ORIGIN      ,                   // 未找到原点	
	SYS_ERR_MTR1_OVER_RUN        ,                   // 距离超限	
	SYS_ERR_MTR1_LOST_STEP       ,                   // 丢步	
	SYS_ERR_MTR1_LOW_LIMIT       ,                   // 下超限	
	SYS_ERR_MTR1_RUNNING         ,                   // 电机正在运行
}emSysError;


/* Types    --------------------------------------------------------------------*/

/* Structure definition---------------------------------------------------------*/

/* Functions -------------------------------------------------------------------*/
void SetCurrSysError(u8 ErrorIdx);
BYTE GetCurrSysError(u8 *pcPrevError);

#ifdef __cplusplus
}
#endif

#endif


