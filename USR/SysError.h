/*******************************************************************************
 * @file    SysError.h    
 * @author  Yan xw
 * @version V1.0.0
 * @date    2017-12-13
 * @brief   ϵͳ�����Ĵ����붨�� 
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
	SYS_ERR_XLP_COMM         = SYS_ERR_BASE_SYR,   // ע���ָ�����
	SYS_ERR_XLP_PARA           ,                   // ע��ò�������
	SYS_ERR_XLP_TIMEOUT        ,                   // ִ�г�ʱ
	SYS_ERR_XLP_RETURN_UNKOW   ,                   // �����벻��ʶ��
	
	SYS_ERR_XLP_INTERNAL       ,                   // �ڲ�������
	
	// RV error
	SYS_ERR_RV_COMM          = SYS_ERR_BASE_RV,    // ��ת��ָ�����
	SYS_ERR_RV_PARA            ,                   // ��ת����������
	SYS_ERR_RV_TIMEOUT         ,                   // ִ�г�ʱ
	SYS_ERR_RV_RETURN_UNKOW    ,                   // �����벻��ʶ��
	
	SYS_ERR_RV_INTERNAL        ,                   // �ڲ�������
	
	// motor error
	SYS_ERR_MTR1_COMM          = SYS_ERR_BASE_MTR1,  // ���ָ�����
	SYS_ERR_MTR1_PARA            ,                   // �������������
	SYS_ERR_MTR1_TIMEOUT         ,                   // ִ�г�ʱ
	SYS_ERR_MTR1_NO_RESET        ,                   // δ��λ
	SYS_ERR_MTR1_NON_ORIGIN      ,                   // δ�ҵ�ԭ��	
	SYS_ERR_MTR1_OVER_RUN        ,                   // ���볬��	
	SYS_ERR_MTR1_LOST_STEP       ,                   // ����	
	SYS_ERR_MTR1_LOW_LIMIT       ,                   // �³���	
	SYS_ERR_MTR1_RUNNING         ,                   // �����������
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


