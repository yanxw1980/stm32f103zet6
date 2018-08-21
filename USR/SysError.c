/*******************************************************************************
 * @file    SysError.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-09-13
 * @brief   系统错误相关的函数接口
 ******************************************************************************
 * @attention
 * 
 * 
 ******************************************************************************
 */ 
 
/* Includes -----------------------------------------------------------------*/ 
#include "SysError.h"
#include <string.h>
#include <stdio.h>


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
static u8 gPrevSysError = SYS_ERR_NONE;
static u8 gCurrSysError = SYS_ERR_NONE;


/* Private function prototypes ------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/ 


/*******************************************************************************
 * @brief  存储错误码
 * @input  ErrorIdx, 需要存储的错误码
 * @return 
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
void SetCurrSysError(u8 ErrorIdx)
{
	gPrevSysError = gCurrSysError;
	
	gCurrSysError = ErrorIdx;
}

/*******************************************************************************
 * @brief  增加节点
 * @input  pcPrevError, 如果需要查询之前的错误码,传入指针
 * @return 当前错误码
 * @History 
 * 1. Date
 *    Author 
 *    Modification
 *******************************************************************************/
BYTE GetCurrSysError(u8 *pcPrevError)
{
	if( NULL != pcPrevError )  
		*pcPrevError = gPrevSysError;
	
	return gCurrSysError;
}





