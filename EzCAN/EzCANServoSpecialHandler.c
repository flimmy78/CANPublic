/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.c
* Author             : ���
* Date First Issued  : 2011.08.05
* Description        : EzCAN�ⲿ�ӿ�
********************************************************************************/
#include "EzCAN.h"
#include "EzCANServoSpecialHandler.h"
#include "PIDRegulators.h"
#include "SVPWM.h"
#include "Consts.h"
#include "ControlParams.h"
#include "GlobalParams.h"
#include "MotionControl.h"	
#include "EzCANProcess.h"

#define IIR_FILTER_S 65536*3142*2    //65536*3.14*2*1000(ʱ�䳣����λ��ms���������Ҫ����1000����������65536��Χ��)

/*******************************************************************************
* Function Name  : EzCANMotoPosPIHandler
* Description    : λ�û�PID����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoPosPIHandler(CAN_msg *msg, const struct ID_ITEM *item)
{
	u8 ret = OP_COMPLETE;

	if(msg->type == DATA_FRAME)
	{
		*((u32*)item->Data) = *((u32*)msg->data);

		PIDSetParams(&PIDPosition, 
					EzCANServoParams.Moto.PosKp,
					EzCANServoParams.Moto.PosKi, 0);

		PIDSetParams(&PIDSpeedPosition, 
					EzCANServoParams.Moto.PosKp,
					EzCANServoParams.Moto.PosKi, 0);
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(msg->id),	EzCANDeviceID, FETCH_PROP(msg->id)), item->Data, sizeof(s32), msg->ch);
		}
		else
			return OP_ILLEGAL_RW;	
	}	

	return ret;	
}

/*******************************************************************************
* Function Name  : EzCANMotoSpeedPIHandler
* Description    : �ٶȻ�PID����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoSpeedPIHandler(CAN_msg *msg, const struct ID_ITEM *item)
{
	if(msg->type == DATA_FRAME)
	{
		*((u32*)item->Data) = *((u32*)msg->data);

		PIDSetParams(&PIDSpeed, 
					EzCANServoParams.Moto.SpeedKp,
					EzCANServoParams.Moto.SpeedKi, 0);
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(msg->id),	EzCANDeviceID, FETCH_PROP(msg->id)), item->Data, sizeof(s32), msg->ch);
		}
		else
			return OP_ILLEGAL_RW;	
	}	

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANMotoTorquePIHandler
* Description    : �ٶȻ�PID����������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoTorquePIHandler(CAN_msg *msg, const struct ID_ITEM *item)
{
	if(msg->type == DATA_FRAME)
	{
		*((u32*)item->Data) = *((u32*)msg->data);

		PIDSetParams(&PIDTorque, 
					EzCANServoParams.Moto.TorqueKp,
					EzCANServoParams.Moto.TorqueKi, 0);

		PIDSetParams(&PIDFlux, 
					EzCANServoParams.Moto.TorqueKp,
					EzCANServoParams.Moto.TorqueKi, 0);
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(msg->id),	EzCANDeviceID, FETCH_PROP(msg->id)), item->Data, sizeof(s32), msg->ch);
		}
		else
			return OP_ILLEGAL_RW;	
	}	

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANMotoTargetPositionHandler
* Description    : ����Ŀ��λ�õĴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlTargetPositionHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	s64 data;

	if(pmsg->type == DATA_FRAME)
	{
		if(EzCANServoParams.Control.LoopMode == POS_TORQUE_LOOP
			|| EzCANServoParams.Control.LoopMode == POS_SPEED_TORQUE_LOOP
			|| EzCANServoParams.Control.LoopMode == POS_LOOP
			|| EzCANServoParams.Control.LoopMode == POS_SPEED_LOOP)
		{
			data = *((s64*)pmsg->data);
			
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data *= -1;
			}
	
			*((s64*)item->Data) = data;
		}
	}
	else
	{
		if(item->RW != WO)
		{
			data = *((s64*)item->Data);

			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data *= -1;
			}

			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(s64), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoTargetSpeedHandler
* Description    : ����Ŀ���ٶȵĴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlTargetSpeedHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	s32 data;

	if(pmsg->type == DATA_FRAME)
	{
		if(EzCANServoParams.Control.LoopMode == SPEED_TORQUE_LOOP
			|| EzCANServoParams.Control.LoopMode == SPEED_LOOP)
		{
			data = *((s32*)pmsg->data);
			
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data *= -1;
			}
	
			*((s32*)item->Data) = data;
		}
		else
			return OP_ILLEGAL_VAL;
	}
	else
	{
		if(item->RW != WO)
		{
			data = *((s32*)item->Data);
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data *= -1;
			}

			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(s32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoTargetVoltageHandler
* Description    : ����Ŀ���ѹ(PWM)�Ĵ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlTargetVoltageHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	S16Params data;

	if(pmsg->type == DATA_FRAME)
	{
		if(EzCANServoParams.Control.LoopMode == OPEN_LOOP)
		{
			data = *((S16Params*)pmsg->data);
			
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data.Param1 *= -1;
				data.Param2 *= -1;
			}
			
			*((S16Params*)item->Data) = data;
		}
		else
			return OP_ILLEGAL_VAL;
	}
	else
	{
		if(item->RW != WO)
		{
			data = *((S16Params*)item->Data);
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data.Param1 *= -1;
				data.Param2 *= -1;
			}

			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(S16Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoTargetCurrentHandler
* Description    : ����Ŀ��ת�صĴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlTargetCurrentHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	S16Params data;

	if(pmsg->type == DATA_FRAME)
	{
		//ֻ����ת�ػ�״̬�²�������Ŀ��ת��
		if(EzCANServoParams.Control.LoopMode == TORQUE_LOOP)
		{
			data = *((S16Params*)pmsg->data);
			
			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data.Param1 *= -1;
				data.Param2 *= -1;
			}

			*((S16Params*)item->Data) = data;
		}
		else
			return OP_ILLEGAL_VAL;
	}
	else
	{
		if(item->RW != WO)
		{
			data = *((S16Params*)item->Data);

			if(EzCANServoParams.Moto.PositiveDir != 1)
			{
				data.Param1 *= -1;
				data.Param2 *= -1;
			}

			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(S16Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoNowPositionHandler
* Description    : ��ȡ��ǰλ�ô�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANStatusNowPositionHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	s64 data;
	if(pmsg->type == REMOTE_FRAME)
	{
		data = *((s64*)item->Data);

		if(EzCANServoParams.Moto.PositiveDir != 1)
		{
			data *= -1;
		}

		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(s64), pmsg->ch);
	}
	else
		return OP_ILLEGAL_RW;

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoNowSpeedHandler
* Description    : ��ȡ��ǰ�ٶȴ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANStatusNowSpeedHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	s32 data;
	if(pmsg->type == REMOTE_FRAME)
	{
		data = *((s32*)item->Data);

		if(EzCANServoParams.Moto.PositiveDir != 1)
		{
			data *= -1;
		}

		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(s32), pmsg->ch);
	}
	else
		return OP_ILLEGAL_RW;

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANStatusNowCurrentDQHandler
* Description    : ��ȡ��ǰת�صĴ���������λ��%
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANStatusNowCurrentDQHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	S16Params data;
	if(pmsg->type == REMOTE_FRAME)
	{
		data = *((S16Params*)item->Data);

	   	if(EzCANServoParams.Moto.PositiveDir != 1)
		{
			data.Param1 *= -1;
			data.Param2 *= -1;
		}

		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(S16Params), pmsg->ch);
	}
	else
		return OP_ILLEGAL_RW;

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANStatusNowVoltageDQHandler
* Description    : ��ȡ��ǰDQ��ѹ�Ĵ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANStatusNowVoltageDQHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	S16Params data;
	if(pmsg->type == REMOTE_FRAME)
	{
		data = *((S16Params*)item->Data);

	   	if(EzCANServoParams.Moto.PositiveDir != 1)
		{
			data.Param1 *= -1;
			data.Param2 *= -1;
		}

		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), &data, sizeof(S16Params), pmsg->ch);
	}
	else
		return OP_ILLEGAL_RW;

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANStatusNowVoltageDQHandler
* Description    : ��ȡ��ǰDQ��ѹ�Ĵ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoPositiveDirHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	s8 data, pre_data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((s8*)pmsg->data);
		pre_data = *((s8*)item->Data);

		if(data != 1)
		{
			data = -1;
		}

		switch(EzCANServoParams.Control.LoopMode)
		{
		case OPEN_LOOP:
		case TORQUE_LOOP:
			//ֻ����λ���Ƶ�����²����޸�Ŀ��ֵ
			if(pre_data == -1 && EzCANServoParams.Moto.CtrlSign)
				EzCANServoParams.Control.TargetCurrent.Param1 *= -1;
		break;
		case SPEED_TORQUE_LOOP:
		case SPEED_LOOP:
			//ֻ����λ���Ƶ�����²����޸�Ŀ��ֵ
			if(pre_data == -1 && EzCANServoParams.Moto.CtrlSign)
				EzCANServoParams.Control.TargetSpeed *= -1;
		break;
		case POS_TORQUE_LOOP:
		case POS_SPEED_TORQUE_LOOP:
		case POS_LOOP:
		case POS_SPEED_LOOP:
			//λ�û�ͻȻ������Σ��
			if(pre_data == -1 && EzCANServoParams.Moto.CtrlSign)
				EzCANServoParams.Control.TargetPosition *= -1;
		break;
		}

		*((s8*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(s8), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoMaxTorqueHandler
* Description    : �������ת�صĴ����������ת�صĵ�λ����%
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoMaxTorqueHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	if(pmsg->type == DATA_FRAME)
	{		
		*((u16*)item->Data) = *((u16*)pmsg->data);
	}
	else
	{
		if(item->RW != WO)
		{		
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
} 

/*******************************************************************************
* Function Name  : EzCANControlLoopModeHandler
* Description    : ����ջ������л��ĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlLoopModeHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	U8Range *range;
	u8 data;
	u8 ret =  OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = *((u8*)pmsg->data);
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U8Range*)item->Range;
			if(data < range->Low)
			{
				data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data > range->High)
			{
				data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		/*if(data == POS_TORQUE_LOOP
			|| data == POS_SPEED_TORQUE_LOOP
			|| data == POS_LOOP
			|| data == POS_SPEED_LOOP)
		{			
			EzCANServoParams.Control.TargetPosition = 0;
			EzCANServoParams.Status.InternalTargetPosition = 0;
			ZERO_POS();
			EzCANServoParams.Status.NowPosition = 0;
		}*/

		*((u8*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u8), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : EzCANMotoSpeedPositiveAccTimeHandler
* Description    : ������ٵĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANCalcMotoSpeedPositiveAccTime(s32 data, u8 sense, u8 init)
{
	if(data == 0)
		data = 1;

	//������ٹ��̵Ĳ���
	InterParams.TargetSpeedIncDivisor = (s32)data * (s32)EzCANServoParams.Moto.SpeedStoreSampleFreq;
	InterParams.TargetSpeedIncRemainder = EzCANServoParams.Moto.MaxSpeed * 1000 / InterParams.TargetSpeedIncDivisor;
	InterParams.TargetSpeedIncModulus = EzCANServoParams.Moto.MaxSpeed * 1000 % InterParams.TargetSpeedIncDivisor;	

	if(EzCANServoParams.Control.LoopMode == POS_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_LOOP
		|| init == 1)
	{
		if( sense == 0 || (EzCANServoParams.Status.NowSysStatus & EZCAN_SERVO_STATUS_ACC_DONE) != 0 )	//NowSysStatus�ĵڶ�λ��Ϊ1ʱ�����޸�
		{
			//���ڴ���λ�û��Ӽ��ٹ���
			//���Ƶ�������ת��һһ��Ӧ����������ת�ٴ�����һ����
			InterParams.TargetPosIncDivisor = (s32)data * (s32)EzCANServoParams.Moto.PosStoreSampleFreq * (s32)EzCANServoParams.Moto.PosStoreSampleFreq / (s32)1000;
			InterParams.TargetPosIncRemainder = (EzCANServoParams.Moto.MaxSpeed) / (InterParams.TargetPosIncDivisor);
			InterParams.TargetPosIncModulus = (EzCANServoParams.Moto.MaxSpeed)  % (InterParams.TargetPosIncDivisor);	
			InterParams.AccPositiveTime = data;
		}
		else
		{
			return 0;
		}
	}
		

	return 1;
}

u8 EzCANMotoSpeedPositiveAccTimeHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u32 data;
	if(pmsg->type == DATA_FRAME)
	{
		data = *((u32*)pmsg->data);

		if(EzCANCalcMotoSpeedPositiveAccTime(data, 1, 0) == 1)
		{
			*((u32*)item->Data) = data;
		}
		else
		{
			return OP_ILLEGAL_VAL;
		}
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoSpeedNegativeAccTimeHandler
* Description    : ������ٵĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANCalcMotoSpeedNegativeAccTime(s32 data, u8 sense, u8 init)
{
	if(data == 0)
		data = 1;

	//������ٹ��̵Ĳ���
	InterParams.TargetSpeedDecDivisor = (s32)data * (s32)EzCANServoParams.Moto.SpeedStoreSampleFreq;
	InterParams.TargetSpeedDecRemainder = EzCANServoParams.Moto.MaxSpeed * 1000 / InterParams.TargetSpeedDecDivisor;
	InterParams.TargetSpeedDecModulus = EzCANServoParams.Moto.MaxSpeed * 1000 % InterParams.TargetSpeedDecDivisor;

	if(EzCANServoParams.Control.LoopMode == POS_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_LOOP
		|| init == 1)
	{
		if( sense == 0 || (EzCANServoParams.Status.NowSysStatus & EZCAN_SERVO_STATUS_ACC_DONE) != 0 )	//NowSysStatus�ĵڶ�λ��Ϊ1ʱ�����޸�
		{
			//���ڴ���λ�û��Ӽ��ٹ���
			//���Ƶ�������ת��һһ��Ӧ����������ת�ٴ�����һ����
			InterParams.TargetPosDecDivisor = (s32)data * (s32)EzCANServoParams.Moto.PosStoreSampleFreq * (s32)EzCANServoParams.Moto.PosStoreSampleFreq / (s32)1000;
			InterParams.TargetPosDecRemainder = (EzCANServoParams.Moto.MaxSpeed) / (InterParams.TargetPosDecDivisor);
			InterParams.TargetPosDecModulus = (EzCANServoParams.Moto.MaxSpeed) % (InterParams.TargetPosDecDivisor);

			InterParams.AccNegativeTime = data;
		}
		else
		{
			return 0;
		}
	}

	return 1;	
}
u8 EzCANMotoSpeedNegativeAccTimeHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u32 data;
	if(pmsg->type == DATA_FRAME)
	{
		data = *((u32*)pmsg->data);
		
		if(EzCANCalcMotoSpeedNegativeAccTime(data, 1, 0) == 1)
		{
			*((u32*)item->Data) = data;
		}
		else
		{
			return OP_ILLEGAL_VAL;
		}		
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANServoPerformanceStateHandler
* Description    : ����ջ������л��ĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPosCmdSmooth(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 6)	//��֤(0<a<65536)
		InterParams.PosCmdSmoothParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.PosStoreSampleFreq * (u32)data);
	else
		InterParams.PosCmdSmoothParam = 65536;
}
u8 EzCANMotoPosCmdSmoothHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	U16Range *range;
	u16 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		if(item->Range != NULL)
		{
			range = (U16Range*)item->Range;
			if(data < range->Low)
			{
				data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data > range->High)
			{
				data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		EzCANCalcMotoPosCmdSmooth(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;	
}

/*******************************************************************************
* Function Name  : EzCANServoPerformanceStateHandler
* Description    : ����ջ������л��ĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcPosCmdFIR(u16 data)
{
	InterParams.PosCmdFIRParam = data * EzCANServoParams.Moto.PosStoreSampleFreq / 1000;
}
u8 EzCANMotoPosCmdFIRHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	U16Range *range;
	u16 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		if(item->Range != NULL)
		{
			range = (U16Range*)item->Range;
			if(data < range->Low)
			{
				data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data > range->High)
			{
				data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

	   	EzCANCalcPosCmdFIR(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;	
}

/*******************************************************************************
* Function Name  : EzCANMotoSpeedKffHandler
* Description    : �����ٶ�ǰ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoSpeedKffHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		FeedForwardSetParams(&FFPositionToSpeed, (s32)data*(s32)EzCANServoParams.Moto.PosStoreSampleFreq);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoTorqueKffHandler
* Description    : ����ת��ǰ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoTorqueKffHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		FeedForwardSetParams(&FFSpeedToTorque, (s32)data*(s32)EzCANServoParams.Moto.SpeedStoreSampleFreq);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoSpeedKffFilterHandler
* Description    : �����ٶ�ǰ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSpeedKffFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 6)	//��֤(0<a<65536)
   		InterParams.SpeedFeedForwardFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.PosStoreSampleFreq*(u32)data);
	else
		InterParams.SpeedFeedForwardFilterParam = 65536;
}
u8 EzCANMotoSpeedKffFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoSpeedKffFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoTorqueKffFilterHandler
* Description    : ����ת��ǰ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoTorqueKffFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.TorqueFeedForwardFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.TorqueFeedForwardFilterParam = 65536;
}
u8 EzCANMotoTorqueKffFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoTorqueKffFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoSVSpeedCmdFilterHandler
* Description    : ����ת��ǰ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSVSpeedCmdFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.SVSpeedCmdFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.SVSpeedCmdFilterParam = 65536;
}

u8 EzCANMotoSVSpeedCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoSVSpeedCmdFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoSVTorqueCmdFilterHandler
* Description    : ����ת��ǰ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSVTorqueCmdFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.SVTorqueCmdFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.SVTorqueCmdFilterParam = 65536;
}

u8 EzCANMotoSVTorqueCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoSVTorqueCmdFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoPWMSpeedCmdFilterHandler
* Description    : ����PWM�ٶ�ָ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPWMSpeedCmdFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.PWMSpeedCmdFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.PWMSpeedCmdFilterParam = 65536;
}

u8 EzCANMotoPWMSpeedCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoPWMSpeedCmdFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoPWMTorqueCmdFilterHandler
* Description    : ����PWMת��ָ���˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPWMTorqueCmdFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.PWMTorqueCmdFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.PWMTorqueCmdFilterParam = 65536;
}

u8 EzCANMotoPWMTorqueCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcMotoPWMTorqueCmdFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANControlClearPositionHandler
* Description    : λ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlClearPositionHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u8 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u8*)pmsg->data);

		if(data == 1)
		{
			if(EzCANServoParams.Control.State == IDLE)
			{			
				InterParams.AccTargetPos = InterParams.AccStartTargetPos = 0;
				InterParams.AccThisTimeTargetPos = 0;			//��ǰ�Ӽ��ٹ����е�Ŀ��λ��
				InterParams.AccLastTargetPos = 0;				//�ϴ�Ŀ��λ��
				InterParams.AccStatus = 0;						//�Ӽ���״̬
				EzCANServoParams.Control.TargetPosition = 0;
				EzCANServoParams.Status.InternalTargetPosition = 0;
				ZERO_POS();
				EzCANServoParams.Status.NowPosition = 0;
			}
		}
		else
			return OP_ILLEGAL_VAL;
	}

	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoEncoderGapNumHandler
* Description    : ���ñ��������������³�ʼ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoEncoderGapNumHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u32 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u32*)pmsg->data);

		if(EzCANServoParams.Control.State == IDLE
			|| EzCANServoParams.Control.State == GUIDE
			|| EzCANServoParams.Control.Enable == 0)
		{	
			*((u32*)item->Data) = data;
			EzCANServoParams.Control.TargetPosition = 0;
			EzCANServoParams.Status.InternalTargetPosition = 0;
			InterParams.AccTargetPos = InterParams.AccStartTargetPos = 0;
			InterParams.AccThisTimeTargetPos = 0;			//��ǰ�Ӽ��ٹ����е�Ŀ��λ��
			InterParams.AccLastTargetPos = 0;				//�ϴ�Ŀ��λ��
			InterParams.AccStatus = 0;						//�Ӽ���״̬
		#ifdef ENCODER
			EncInit(0);
		#endif
			ZERO_POS();
			EzCANServoParams.Status.NowPosition = 0;
		}
		else
		{
			return OP_ILLEGAL_VAL;
		}
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANMotoMaxSpeedHandler
* Description    : �������ת��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoMaxSpeedHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u32 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u32*)pmsg->data);

		if(EzCANCalcMotoSpeedPositiveAccTime(EzCANServoParams.Moto.SpeedPositiveAccTime, 1, 0) == 1
			&& EzCANCalcMotoSpeedNegativeAccTime(EzCANServoParams.Moto.SpeedNegativeAccTime, 1, 0) == 1)
		{
			*((u32*)item->Data) = data;
		}
		else
			return OP_ILLEGAL_VAL;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}
	return OP_COMPLETE;
}

/*******************************************************************************
* Function Name  : EzCANMotoRefreshUploadHandler
* Description    : ˢ��״̬�ϱ�������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANMotoRefreshUploadHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u8 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u8*)pmsg->data);

		if(data == 1)
		{		 
			*((u8*)item->Data) = data;
			PostNowSystemStatusToMaster();
		}
		else
		{
			data = 0;
			*((u8*)item->Data) = data;
		}
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u8), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANCalcSpeedFeedbackFilter
* Description    : �����ٶȷ����˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcSpeedFeedbackFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.SpeedFeedbackFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.SpeedFeedbackFilterParam = 65536;
	
}
u8 EzCANMotoSpeedFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcSpeedFeedbackFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;	
}

/*******************************************************************************
* Function Name  : EzCANCalcTorqueFeedbackFilter
* Description    : ����ת�ط����˲�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcTorqueFeedbackFilter(u16 data)
{
	//f=a/(2*��*t) ����, f:��ֹƵ��;a:�˲�ϵ��;t:�������(ms)
	if(data > 1)	//��֤(0<a<65536)
   		InterParams.TorqueFeedbackFilterParam = (u32)IIR_FILTER_S/((u32)EzCANServoParams.Moto.SpeedStoreSampleFreq*(u32)data);
	else
		InterParams.TorqueFeedbackFilterParam = 65536;

}
u8 EzCANMotoTorqueFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
   	u16 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);

		EzCANCalcTorqueFeedbackFilter(data);

		*((u16*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(u16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return OP_COMPLETE;	
}
