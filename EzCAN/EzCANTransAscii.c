/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANTransAscii.c
* Author             : 徐成
* Date First Issued  : 2012.03.05
* Description        : EzCAN用ASCII控制的转码
********************************************************************************/
#include "EzCAN.h"
#include "EzCANTransAscii.h"

bool EzCANStringToMessage(u8 *str/*in*/, u16 size/*in*/, CAN_msg *msg/*out*/)
{
	return TRUE;
}

bool EzCANMessageToString(CAN_msg *msg/*in*/, u8 *str/*out*/, u16 *size/*out*/)
{
	return TRUE;
}
