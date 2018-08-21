/*******************************************************************************
 * @file    SysError.c    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-09-13
 * @brief   ϵͳ������صĺ����ӿ�
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
 * @brief  �洢������
 * @input  ErrorIdx, ��Ҫ�洢�Ĵ�����
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
 * @brief  ���ӽڵ�
 * @input  pcPrevError, �����Ҫ��ѯ֮ǰ�Ĵ�����,����ָ��
 * @return ��ǰ������
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





