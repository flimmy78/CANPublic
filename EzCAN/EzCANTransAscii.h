/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANTransAscii.h
* Author             : 徐成
* Date First Issued  : 2011.03.05
* Description        : EzCAN用ASCII控制的转码
********************************************************************************/
#ifndef __EZ_CAN_COMMON_H
#define __EZ_CAN_COMMON_H

#include <stm32f10x_lib.h>
#include "uCosII_CAN.h"

bool EzCANStringToMessage(u8 *str/*in*/, u16 size/*in*/, CAN_msg *msg/*out*/);
bool EzCANMessageToString(CAN_msg *msg/*in*/, u8 *str/*out*/, u16 *size/*out*/);

#endif 
