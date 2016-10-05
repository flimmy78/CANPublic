/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.c
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN外部接口
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

#define IIR_FILTER_S 65536*3142*2    //65536*3.14*2*1000(时间常数单位是ms，因此这里要乘以1000；参数扩大到65536范围内)

/*******************************************************************************
* Function Name  : EzCANMotoPosPIHandler
* Description    : 位置环PID参数处理函数
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
* Description    : 速度环PID参数处理函数
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
* Description    : 速度环PID参数处理函数
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
* Description    : 设置目标位置的处理函数
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
* Description    : 设置目标速度的处理函数
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
* Description    : 设置目标电压(PWM)的处理函数
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
* Description    : 设置目标转矩的处理函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANControlTargetCurrentHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	S16Params data;

	if(pmsg->type == DATA_FRAME)
	{
		//只有在转矩环状态下才能设置目标转矩
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
* Description    : 读取当前位置处理函数
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
* Description    : 读取当前速度处理函数
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
* Description    : 读取当前转矩的处理函数，单位：%
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
* Description    : 读取当前DQ电压的处理函数
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
* Description    : 读取当前DQ电压的处理函数
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
			//只有上位控制的情况下才能修改目标值
			if(pre_data == -1 && EzCANServoParams.Moto.CtrlSign)
				EzCANServoParams.Control.TargetCurrent.Param1 *= -1;
		break;
		case SPEED_TORQUE_LOOP:
		case SPEED_LOOP:
			//只有上位控制的情况下才能修改目标值
			if(pre_data == -1 && EzCANServoParams.Moto.CtrlSign)
				EzCANServoParams.Control.TargetSpeed *= -1;
		break;
		case POS_TORQUE_LOOP:
		case POS_SPEED_TORQUE_LOOP:
		case POS_LOOP:
		case POS_SPEED_LOOP:
			//位置环突然反向会很危险
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
* Description    : 设置最大转矩的处理函数，最大转矩的单位都是%
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
* Description    : 处理闭环类型切换的函数
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
* Description    : 处理加速的函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANCalcMotoSpeedPositiveAccTime(s32 data, u8 sense, u8 init)
{
	if(data == 0)
		data = 1;

	//计算加速过程的参数
	InterParams.TargetSpeedIncDivisor = (s32)data * (s32)EzCANServoParams.Moto.SpeedStoreSampleFreq;
	InterParams.TargetSpeedIncRemainder = EzCANServoParams.Moto.MaxSpeed * 1000 / InterParams.TargetSpeedIncDivisor;
	InterParams.TargetSpeedIncModulus = EzCANServoParams.Moto.MaxSpeed * 1000 % InterParams.TargetSpeedIncDivisor;	

	if(EzCANServoParams.Control.LoopMode == POS_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_LOOP
		|| init == 1)
	{
		if( sense == 0 || (EzCANServoParams.Status.NowSysStatus & EZCAN_SERVO_STATUS_ACC_DONE) != 0 )	//NowSysStatus的第二位不为1时可以修改
		{
			//用于处理位置环加减速过程
			//最高频率与最高转速一一对应，因此用最高转速处理是一样的
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
* Description    : 处理减速的函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 EzCANCalcMotoSpeedNegativeAccTime(s32 data, u8 sense, u8 init)
{
	if(data == 0)
		data = 1;

	//计算减速过程的参数
	InterParams.TargetSpeedDecDivisor = (s32)data * (s32)EzCANServoParams.Moto.SpeedStoreSampleFreq;
	InterParams.TargetSpeedDecRemainder = EzCANServoParams.Moto.MaxSpeed * 1000 / InterParams.TargetSpeedDecDivisor;
	InterParams.TargetSpeedDecModulus = EzCANServoParams.Moto.MaxSpeed * 1000 % InterParams.TargetSpeedDecDivisor;

	if(EzCANServoParams.Control.LoopMode == POS_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_TORQUE_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_LOOP
		|| EzCANServoParams.Control.LoopMode == POS_SPEED_LOOP
		|| init == 1)
	{
		if( sense == 0 || (EzCANServoParams.Status.NowSysStatus & EZCAN_SERVO_STATUS_ACC_DONE) != 0 )	//NowSysStatus的第二位不为1时可以修改
		{
			//用于处理位置环加减速过程
			//最高频率与最高转速一一对应，因此用最高转速处理是一样的
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
* Description    : 处理闭环类型切换的函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPosCmdSmooth(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 6)	//保证(0<a<65536)
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
* Description    : 处理闭环类型切换的函数
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
* Description    : 处理速度前馈
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
* Description    : 处理转矩前馈
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
* Description    : 处理速度前馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSpeedKffFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 6)	//保证(0<a<65536)
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
* Description    : 处理转矩前馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoTorqueKffFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 处理转矩前馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSVSpeedCmdFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 处理转矩前馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoSVTorqueCmdFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 处理PWM速度指令滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPWMSpeedCmdFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 处理PWM转矩指令滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcMotoPWMTorqueCmdFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 位置清零
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
				InterParams.AccThisTimeTargetPos = 0;			//当前加减速过程中的目标位置
				InterParams.AccLastTargetPos = 0;				//上次目标位置
				InterParams.AccStatus = 0;						//加减速状态
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
* Description    : 设置编码器线数，重新初始化编码器
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
			InterParams.AccThisTimeTargetPos = 0;			//当前加减速过程中的目标位置
			InterParams.AccLastTargetPos = 0;				//上次目标位置
			InterParams.AccStatus = 0;						//加减速状态
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
* Description    : 设置最高转速
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
* Description    : 刷新状态上报处理函数
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
* Description    : 计算速度反馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcSpeedFeedbackFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
* Description    : 计算转矩反馈滤波
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANCalcTorqueFeedbackFilter(u16 data)
{
	//f=a/(2*π*t) 其中, f:截止频率;a:滤波系数;t:采样间隔(ms)
	if(data > 1)	//保证(0<a<65536)
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
