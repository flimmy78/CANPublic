/************************* (C) COPYRIGHT 2012 Robotell *************************
* File Name          : EzCANProcess.c 
* Author             : xc 
* Date First Issued  : 21/11/07
* Description        : EzCAN处理线程
********************************************************************************/
#include "EzCANProcess.h"
#include "SystemSupervise.h"
#include "CANApplication.h"
#include "USARTCommunication.h"
#include "EzCAN.h"
#include "string.h"
#include "GlobalParams.h"

#define EZCAN_MSG_BOX_SIZE 10

//============------EzCAN接收队列-----=============//
OS_EVENT *EzCANRxEvent;					           	
static void *EzCANRxBox[EZCAN_MSG_BOX_SIZE];		

OS_MEM *EzCANRxMemo;               	           		
static CAN_msg EzCANRxMemoPool[EZCAN_MSG_BOX_SIZE];	
//============------EzCAN接收队列-----=============//

//============------EzCAN发送队列-----=============//
OS_EVENT *EzCANTxEvent;					           	
static void *EzCANTxBox[EZCAN_MSG_BOX_SIZE];

OS_MEM *EzCANTxMemo;               	           		
static CAN_msg EzCANTxMemoPool[EZCAN_MSG_BOX_SIZE];
//============------EzCAN发送队列-----=============//

//static u8 EzCANAsciiBuffer[256];

bool EzCANEventInit(void)
{
	u8 err;

	EzCANRxEvent = OSQCreate(&EzCANRxBox[0], EZCAN_MSG_BOX_SIZE);
	EzCANRxMemo = OSMemCreate(&EzCANRxMemoPool[0], EZCAN_MSG_BOX_SIZE, sizeof(CAN_msg), &err);

	EzCANTxEvent = OSQCreate(&EzCANTxBox[0], EZCAN_MSG_BOX_SIZE);
	EzCANTxMemo = OSMemCreate(&EzCANTxMemoPool[0], EZCAN_MSG_BOX_SIZE, sizeof(CAN_msg), &err);

    if (err != OS_NO_ERR)
    {
		return FALSE;
	}

	EzCANSetSendFunction(EzCANSendBuffer);
	EzCANSetPostToEzCANFunc(PostToEzCANProcess);
	EzCANSetDelayFunction(OSTimeDly);
	EzCANSetResetFunction(ResetSystem);

	return TRUE;
}

void ResetSystem(void)
{
	NVIC_SETFAULTMASK();					 //系统复位，调试模拟无效果
	NVIC_GenerateSystemReset();
	while(1);	
}

void PostNowSystemStatusToMaster(void)
{
	static u64 last_sys_status;
	static u32 id;

	if(EzCANServoParams.Moto.RefreshUpload != 1)
		return;

	if(last_sys_status == EzCANServoParams.Status.NowSysStatus)
		return;

	id = BUILD_ID(EzCANServoParams.Common.DevPriority, EzCANDeviceID, SERVO_STATUS_NOWSYSSTATUS);
	//MODBUS不支持状态上报，因此tag部分传0即可
	if(EzCANSendBuffer(id, (void*)&EzCANServoParams.Status.NowSysStatus, sizeof(u64), 0) == TRUE)
		last_sys_status = EzCANServoParams.Status.NowSysStatus;
}

bool EzCANSendBuffer(u32 id, void *buf, u8 bufsize, u32 tag)
{
	u8 err;
	CAN_msg *tx_msg;

	tx_msg = OSMemGet(EzCANTxMemo, &err);
	
	if(err != OS_ERR_NONE)
	{
		SendErrorEvent(SW_FAULT, 1);
		return FALSE;
	}

	tx_msg->id = id;
    tx_msg->format = EXTENDED_FORMAT;
    tx_msg->type = DATA_FRAME;
    tx_msg->len = bufsize;	
	tx_msg->ch = tag;
	
	memset(tx_msg->data, 0, sizeof(u64));
	memcpy(tx_msg->data, buf, bufsize);

	OSQPost(EzCANTxEvent, tx_msg);

	return TRUE;
}

bool PostToEzCANProcess(CAN_msg *msg)
{
	u8 err;
	CAN_msg *rx_msg;

	rx_msg = OSMemGet(EzCANRxMemo, &err);
	
	if(err != OS_ERR_NONE)
	{
		SendErrorEvent(SW_FAULT, 1);
		return FALSE;
	}

	*rx_msg = *msg;
	OSQPost(EzCANRxEvent, rx_msg);
	return TRUE;
}

void EzCANTxTask(void *p_arg)
{
	CAN_msg *msg;
	u8 err;

	for(;;)
	{
		msg = OSQPend( EzCANTxEvent, 0, &err);
		if(err == OS_ERR_NONE)
		{
		#ifdef _COMM_USART
			switch(EzCANServoParams.Others.CommMode)
			{
			case 2:
			case 3:
			case 4:
			case 5:
				USARTSendPackage(msg, sizeof(CAN_msg));
			break;
			}
		#else
			CANSendBuffer(msg->id, msg->data, msg->len);
		#endif
			
			OSMemPut(EzCANTxMemo, msg);	
		}
		else
		{
			SendErrorEvent(SW_FAULT, 1);
			OSTimeDly(1);
		}
	}
}

void EzCANProcessTask(void *p_arg)
{
	CAN_msg *msg;
	u8 err;

	for(;;)
	{
		msg = OSQPend(EzCANRxEvent, 0, &err);
		if(err == OS_ERR_NONE)
		{
			EzCANSlaveProcess(msg);
			OSMemPut(EzCANRxMemo, msg);
		}
		else
		{
			SendErrorEvent(SW_FAULT, 1);
			OSTimeDly(1);
		}			
	}
}

void EzCANBoostTask(void *p_arg)
{
	for(;;)
	{
		EzCANBoostProcess();
		OSTimeDly(1);
	}
}

void EzCANStorageSampleTask(void *p_arg)
{
	for(;;)
	{
		EzCANStorageSampleProcess();
		OSTimeDly(1000);
	}
}
