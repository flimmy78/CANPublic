/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN协议的外部接口
********************************************************************************/
#ifndef __EZ_CAN_H
#define __EZ_CAN_H

#include <stm32f10x_lib.h>
#include "uCosII_CAN.h"
/*
*********************************************************************************************************
*                                   协议配置
*********************************************************************************************************
*/
#include "EzCANConfig.h"
#include "EzCANCommon.h"
#include "EzCANServo.h"
#include "EzCANTransAscii.h"	

/*
*********************************************************************************************************
*                                   类型定义
*********************************************************************************************************
*/

/* 外部接口 --------------------------------------------------------*/
void EzCANSlaveProcess(CAN_msg *msg);
void EzCANBoostProcess(void);
void EzCANStorageSampleProcess(void);
void EzCANParamsInit(void);
void EzCANAllParamsSaveToFlash(void);
void EzCANStorageSample(u32 prop, void* data, u8 typelen);
void EzCANAddErrHistory(u8 err);

/* 设置系统相关函数 --------------------------------------------------------*/
void EzCANSetSendFunction(EzCANSendFuncType func);
void EzCANSetPostToEzCANFunc(PostToEzCANFuncType func);
void EzCANSetDelayFunction(EzCANTimeDlyType func);
void EzCANSetResetFunction(EzCANResetSystemType func);


#endif
