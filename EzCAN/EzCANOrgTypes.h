/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANOrgTypes.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN通用的数据类型
********************************************************************************/
#ifndef __EZCAN_ORG_TYPES_H
#define __EZCAN_ORG_TYPES_H

#include <stm32f10x_lib.h>
#include "uCosII_CAN.h"

/*
*********************************************************************************************************
*                                         EzCAN 公用宏定义
*********************************************************************************************************
*/
#ifndef NULL
#define NULL                          0
#endif

#ifndef VALIST
#define VALIST(...) {__VA_ARGS__}
#endif

#ifndef PALIST
#define PALIST(...)	__VA_ARGS__
#endif

#define PRIO_MASK	0x0F000000
#define BUILD_ID(prio, deviceid, property) ((prio<<24)|(deviceid<<16)|(property))
#define FETCH_DEV_ID(id) ((id>>16)&0xFF)
#define FETCH_PRIO(id)	((id>>24)&0xFF)
#define FETCH_PROP(id)	((id)&0xFFFF)

enum{OP_SUCCESS = 0, OP_FAILURE};
enum{RO = 0, WO, RW};

/*
*********************************************************************************************************
*                                         EzCAN 基本数据类型定义
*********************************************************************************************************
*/
typedef struct
{
	u8 Switch:1;
	u8 Reserved1:7;
	u8 Reserved2;
	u16 Interval;
	u32 Id;
}Boost;

typedef struct
{
	u32 Param1;
	u32 Param2;
}U32Params;

typedef struct
{
	s32 Param1;
	s32 Param2;		
}S32Params;

typedef struct
{
	u16 Param1;
	u16 Param2;
}U16Params;

typedef struct
{
	s16 Param1;
	s16 Param2;
}S16Params;

/*
*********************************************************************************************************
*                                         EzCAN 基本数据范围
*********************************************************************************************************
*/
typedef struct
{
	s32 Low;
	s32 High;
}S64Range;

typedef struct
{
	u32 Low;
	u32 High;
}U64Range;

typedef struct
{
	s32 Low;
	s32 High;
}S32Range;

typedef struct
{
	u32 Low;
	u32 High;
}U32Range;

typedef struct
{
	s16 Low;
	s16 High;
}S16Range;

typedef struct
{
	u16 Low;
	u16 High;
}U16Range;

typedef struct
{
	u8 Low;
	u8 High;
}U8Range;

typedef struct
{
	s8 Low;
	s8 High;
}S8Range;

typedef struct
{
	u32 Param1Low;
	u32 Param1High;
	u32 Param2Low;
	u32 Param2High;
}U32PARAMSRange;

typedef struct
{
	s32 Param1Low;
	s32 Param1High;
	s32 Param2Low;
	s32 Param2High;
}S32PARAMSRange;

typedef struct
{
	u16 Param1Low;
	u16 Param1High;
	u16 Param2Low;
	u16 Param2High;
}U16PARAMSRange;

typedef struct
{
	s16 Param1Low;
	s16 Param1High;
	s16 Param2Low;
	s16 Param2High;
}S16PARAMSRange;

//===============--------数据处理类型区域-------==============//
typedef	struct ID_ITEM
{
	u32 	ID;
	void* 	Data;
	u8 		RW;
	void* 	Range;
	u8	(*Handler)(CAN_msg *pmsg, const struct ID_ITEM *pitem);
	char*	Ascii;
}IDItemTyp;

typedef bool 		(*EzCANSendFuncType)(u32 id, void *buf, u8 bufsize, u32 tag);
typedef bool 		(*PostToEzCANFuncType)(CAN_msg *msg);
typedef void 		(*EzCANTimeDlyType)(u16 ticks);
typedef void 		(*EzCANResetSystemType)(void);

extern EzCANSendFuncType 	EzCANSend;
extern PostToEzCANFuncType 	PostToEzCAN;
extern EzCANTimeDlyType 	EzCANTimeDly;
extern EzCANResetSystemType EzCANResetSystem;

/* Exported functions ------------------------------------------------------------*/
u8 BOOSTHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U64Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S64Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U32Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S32Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U16Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S16Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U8Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S8Handler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U32PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S32PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 U16PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 S16PARAMSHandler(CAN_msg *pmsg, const IDItemTyp *item);


u64 U64Init(u64 v);
s64 S64Init(s64 v);
u32 U32Init(u32 v);
s32 S32Init(s32 v);
u16 U16Init(u16 v);
s16 S16Init(s16 v);
u8  U8Init(u8 v);
s8  S8Init(s8 v);
U32Params U32PARAMSInit(u32 v1, u32 v2);
S32Params S32PARAMSInit(s32 v1, s32 v2);
U16Params U16PARAMSInit(u16 v1, u16 v2);
S16Params S16PARAMSInit(s16 v1, s16 v2);

#endif
