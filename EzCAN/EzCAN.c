/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.c
* Author             : ���
* Date First Issued  : 2011.08.05
* Description        : EzCAN�ⲿ�ӿ�
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
* Description    : ������EzCANЭ�鴦����
* Input          : CAN���ݰ�
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANMasterProcess(CAN_msg *msg)
{
}

/*******************************************************************************
* Function Name  : EzCANSlaveProcess
* Description    : �ӻ���EzCANЭ�鴦����
* Input          : CAN���ݰ�
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSlaveProcess(CAN_msg *msg)
{
	EzCANServoProcess(msg);
}

/*******************************************************************************
* Function Name  : EzCANParamsInit
* Description    : ��ʼ��EzCAN��������
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
* Description    : ����CAN���ݷ��ͺ���
* Input          : func: EzCANSendFuncType���ͺ���ָ��
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetSendFunction(EzCANSendFuncType func)
{
	EzCANSend = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetPostToEzCANFunc
* Description    : ����Post���ݵ�EzCAN����ĺ���
* Input          : func: PostToEzCANFuncType���ͺ���ָ��
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetPostToEzCANFunc(PostToEzCANFuncType func)
{
	PostToEzCAN = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetDelayFunction
* Description    : ������ʱ����
* Input          : func: EzCANTimeDlyType���ͺ���ָ��
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetDelayFunction(EzCANTimeDlyType func)
{
	EzCANTimeDly = func;	
}

/*******************************************************************************
* Function Name  : EzCANSetResetFunction
* Description    : ����ϵͳ��������
* Input          : func: EzCANResetSystemType���ͺ���ָ��
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSetResetFunction(EzCANResetSystemType func)
{
	EzCANResetSystem = func;
} 

/*******************************************************************************
* Function Name  : EzCANAllParamsSaveToFlash
* Description    : ���в������浽Flash��ȥ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANAllParamsSaveToFlash(void)
{
	EzCANServoParamsSaveToFlash();
}

