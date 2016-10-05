/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANOrgTypesHandler.c
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN通用的数据类型的处理程序
********************************************************************************/
#include "EzCANOrgTypes.h"
#include "EzCANServo.h"

EzCANSendFuncType EzCANSend;
PostToEzCANFuncType PostToEzCAN;
EzCANTimeDlyType EzCANTimeDly;
EzCANResetSystemType EzCANResetSystem;

/*******************************************************************************
* Function Name  : U64Handler
* Description    : u64数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U64Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U64Range *range;
	u64 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = *((u64*)pmsg->data);
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U64Range*)item->Range;
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

		*((u64*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), 
					item->Data, 
					sizeof(u64), 
					pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : S64Handler
* Description    : s64数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S64Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S64Range *range;
	s64 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = *((s64*)pmsg->data);
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S64Range*)item->Range;
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

		*((s64*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(s64), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : U32Handler
* Description    : u32数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U32Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U32Range *range;
	u32 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = *((u32*)pmsg->data);
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U32Range*)item->Range;
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

		*((u32*)item->Data) = data;
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

	return ret;
}

/*******************************************************************************
* Function Name  : S32Handler
* Description    : s32数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S32Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S32Range *range;
	s32 data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = *((s32*)pmsg->data);
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S32Range*)item->Range;
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

		*((s32*)item->Data) = data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(s32), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : U16Handler
* Description    : u16数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U16Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U16Range *range;
	u16 *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (u16*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U16Range*)item->Range;
			if(*data < range->Low)
			{
				*data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(*data > range->High)
			{
				*data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((u16*)item->Data) = *data;
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
* Function Name  : S16Handler
* Description    : s16数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S16Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S16Range *range;
	s16 *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (s16*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S16Range*)item->Range;
			if(*data < range->Low)
			{
				*data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(*data > range->High)
			{
				*data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((s16*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(s16), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : U8Handler
* Description    : u8数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U8Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U8Range *range;
	u8 *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (u8*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U8Range*)item->Range;
			if(*data < range->Low)
			{
				*data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(*data > range->High)
			{
				*data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((u8*)item->Data) = *data;
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
* Function Name  : S8Handler
* Description    : s8数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S8Handler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S8Range *range;
	s8 *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (s8*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S8Range*)item->Range;
			if(*data < range->Low)
			{
				*data = range->Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(*data > range->High)
			{
				*data = range->High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((s8*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(s8), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : U32PARAMSHandler
* Description    : U32Params数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U32PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U32PARAMSRange *range;
	U32Params *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (U32Params*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U32PARAMSRange*)item->Range;
			if(data->Param1 < range->Param1Low)
			{
				data->Param1 = range->Param1Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param1 > range->Param1High)
			{
				data->Param1 = range->Param1High;
				ret = OP_ILLEGAL_VAL;
			}

			if(data->Param2 < range->Param2Low)
			{
				data->Param2 = range->Param2Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param2 > range->Param2High)
			{
				data->Param2 = range->Param2High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((U32Params*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(U32Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : S32PARAMSHandler
* Description    : S32Params数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S32PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S32PARAMSRange *range;
	S32Params *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (S32Params*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S32PARAMSRange*)item->Range;
			if(data->Param1 < range->Param1Low)
			{
				data->Param1 = range->Param1Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param1 > range->Param1High)
			{
				data->Param1 = range->Param1High;
				ret = OP_ILLEGAL_VAL;
			}

			if(data->Param2 < range->Param2Low)
			{
				data->Param2 = range->Param2Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param2 > range->Param2High)
			{
				data->Param2 = range->Param2High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((S32Params*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(S32Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : U16PARAMSHandler
* Description    : U16Params数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 U16PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U16PARAMSRange *range;
	U16Params *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (U16Params*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (U16PARAMSRange*)item->Range;
			if(data->Param1 < range->Param1Low)
			{
				data->Param1 = range->Param1Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param1 > range->Param1High)
			{
				data->Param1 = range->Param1High;
				ret = OP_ILLEGAL_VAL;
			}

			if(data->Param2 < range->Param2Low)
			{
				data->Param2 = range->Param2Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param2 > range->Param2High)
			{
				data->Param2 = range->Param2High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((U16Params*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(U16Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
} 

/*******************************************************************************
* Function Name  : S16PARAMSHandler
* Description    : S16Params数据处理函数
* Input          : pmsg: CAN_msg类型CAN数据包
                   item: 待处理数据指针
* Output         : None
* Return         : None
*******************************************************************************/
u8 S16PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	S16PARAMSRange *range;
	S16Params *data;
	u8 ret = OP_COMPLETE;

	if(pmsg->type == DATA_FRAME)
	{
		if(item->RW != RO)
		{
			data = (S16Params*)pmsg->data;
		}
		else
			return OP_ILLEGAL_RW;

		if(item->Range != NULL)
		{
			range = (S16PARAMSRange*)item->Range;
			if(data->Param1 < range->Param1Low)
			{
				data->Param1 = range->Param1Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param1 > range->Param1High)
			{
				data->Param1 = range->Param1High;
				ret = OP_ILLEGAL_VAL;
			}

			if(data->Param2 < range->Param2Low)
			{
				data->Param2 = range->Param2Low;
				ret = OP_ILLEGAL_VAL;
			}
			else if(data->Param2 > range->Param2High)
			{
				data->Param2 = range->Param2High;
				ret = OP_ILLEGAL_VAL;
			}
		}

		*((S16Params*)item->Data) = *data;
	}
	else
	{
		if(item->RW != WO)
		{
			EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)),item->Data, sizeof(S16Params), pmsg->ch);
		}
		else
			return OP_ILLEGAL_RW;
	}

	return ret;
}



u64 U64Init(u64 v)
{
	return v;
}

s64 S64Init(s64 v)
{
	return v;
}

u32 U32Init(u32 v)
{
	return v;
}

s32 S32Init(s32 v)
{
	return v;
}

u16 U16Init(u16 v)
{
	return v;
}

s16 S16Init(s16 v)
{
	return v;
}

u8  U8Init(u8 v)
{
	return v;
}

s8  S8Init(s8 v)
{
	return v;
}

U32Params U32PARAMSInit(u32 v1, u32 v2)
{
	U32Params p;

	p.Param1 = v1;
	p.Param2 = v2;

	return p;
}

S32Params S32PARAMSInit(s32 v1, s32 v2)
{
	S32Params p;

	p.Param1 = v1;
	p.Param2 = v2;

	return p;
}

U16Params U16PARAMSInit(u16 v1, u16 v2)
{
	U16Params p;

	p.Param1 = v1;
	p.Param2 = v2;

	return p;
}

S16Params S16PARAMSInit(s16 v1, s16 v2)
{
	S16Params p;

	p.Param1 = v1;
	p.Param2 = v2;

	return p;
}
