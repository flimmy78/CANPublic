/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.c
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN外部接口
********************************************************************************/
#include "EzCAN.h"

typedef struct
{
	u32  	DevID;
	void	*DeviceParams; 
	void	(*DevProcessFunc)(CAN_msg *msg);
}DeviceTyp;

#ifdef MASTER
DeviceTyp Devices[256];
#endif

/*******************************************************************************
* Function Name  : EzCANMasterProcess
* Description    : 主机的EzCAN协议处理函数
* Input          : CAN数据包
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANMasterProcess(CAN_msg *msg)
{
}

/*******************************************************************************
* Function Name  : EzCANSlaveProcess
* Description    : 从机的EzCAN协议处理函数
* Input          : CAN数据包
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSlaveProcess(CAN_msg *msg)
{
	EzCANServoProcess(msg);
}

/*******************************************************************************
* Function Name  : EzCANParamsInit
* Description    : 初始化EzCAN基本数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANParamsInit(void)
{
	EzCANServoParamsInit();
}

/*******************************************************************************
* Function Name  : EzCANSetSendFunction
* Description    : 设置CAN数据发送函数
* Input          : func: EzCANSendFuncType类型函数指针
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetSendFunction(EzCANSendFuncType func)
{
	EzCANSend = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetPostToEzCANFunc
* Description    : 设置Post数据到EzCAN处理的函数
* Input          : func: PostToEzCANFuncType类型函数指针
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetPostToEzCANFunc(PostToEzCANFuncType func)
{
	PostToEzCAN = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetDelayFunction
* Description    : 设置延时函数
* Input          : func: EzCANTimeDlyType类型函数指针
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetDelayFunction(EzCANTimeDlyType func)
{
	EzCANTimeDly = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetResetFunction
* Description    : 设置系统重启函数
* Input          : func: EzCANResetSystemType类型函数指针
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetResetFunction(EzCANResetSystemType func)
{
	EzCANResetSystem = func;
} 

/*******************************************************************************
* Function Name  : EzCANAllParamsSaveToFlash
* Description    : 所有参数保存到Flash中去
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANAllParamsSaveToFlash(void)
{
	EzCANServoParamsSaveToFlash();
}

