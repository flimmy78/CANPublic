/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANConfig.h
* Author             : ���
* Date First Issued  : 2011.08.05
* Description        : EzCAN�����ļ�
********************************************************************************/
#ifndef __EZ_CAN_CONFIG_H
#define __EZ_CAN_CONFIG_H

/*
*********************************************************************************************************
*                                         EzCAN ��Э������
*********************************************************************************************************
*/
//#define MASTER 
#define SLAVE
#define ERROR_HISTORY_START_PAGE 	0x08018400
#define SERVO_START_PAGE			0x08018C00


#define ERROR_POST_PRIO 	13
#define BOOST_MESSAGE_PRIO	14
#define STORAGE_SAMPLE_PRIO	15

#define STORAGE_DLY_TIM		20

#include "EzCANServoConfig.h"

#endif
