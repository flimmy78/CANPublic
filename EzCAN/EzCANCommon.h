/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANCommon.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN公共属性接口和定义
********************************************************************************/
#ifndef __EZ_CAN_COMMON_H
#define __EZ_CAN_COMMON_H
#include "EzCANOrgTypes.h"

/*
*********************************************************************************************************
*                                       EzCAN 公共属性枚举常量及类型定义
*********************************************************************************************************
*/
typedef enum {
	MOTO_COEUS_PM_3610,
	MOTO_COEUS_PM_2405,
	MOTO_ATLAS_4820,
	MOTO_ATLAS_3610,
	MOTO_ATLAS_2405,
	MOTO_COEUS_DC_3610,
	MOTO_COEUS_DC_2405,
	MOTO_COEUS_PM_11010,
}EzCANDevTypeTyp;

typedef enum {
	PROTOCOL_SERVO,
}EzCANDevProtocolTyp;

typedef enum {
	DT_U64,
	DT_S64,
	DT_U32,
	DT_S32,
	DT_U16,
	DT_S16,
	DT_U8,
	DT_S8,
	DT_U32PARAMS,
	DT_S32PARAMS,
	DT_U16PARAMS,
	DT_S16PARAMS,
	DT_U8PARAMS,
	DT_S8PARAMS,
	DT_FLOAT,
	DT_DOUBLE,
}EzCANDataTypeTyp;

/*
*********************************************************************************************************
*                                         EzCAN 采样部分数据类型定义
*********************************************************************************************************
*/
typedef struct
{
	u16 *StorageProps;	//同步属性列表
	u16 ArraySize;		//同步属性列表长度
	u16	Channels;		//属性所属的通道号，用位表示，最多8个通道
	u8 	Flag;			//同步标志位
}EzCANStorageSynTyp;

typedef struct
{
	u16 			Property;
	u8 				Enable;
	u8				Open;
	u8 				Ready;
	u8 				TypeLength;
	unsigned char*	Buffer;
	u32				BufferLength;
	u8 				AlreadyReset;
	EzCANStorageSynTyp	*StorageSyn;
}EzCANStorageSampleDataTyp;

/*
*********************************************************************************************************
*                                       EzCAN 公共属性数据结构
*
* Note: EZCAN_COMMON_START_TOK 定义起始元素的使用方法
		EZCAN_COMMON_TOK 定义如何使用EzCAN_Common.protocol协议里的数据表
*********************************************************************************************************
*/
typedef struct
{
//===============---------公共属性---------==============//
#define EZCAN_COMMON_COMMON_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_START_TOK

#define EZCAN_COMMON_COMMON_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_TOK
}EzCANCommonCommonTyp;

typedef struct
{
//===============---------BOOST属性---------==============//
#define EZCAN_COMMON_BOOST_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_START_TOK

#define EZCAN_COMMON_BOOST_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_TOK
}EzCANCommonBoostTyp;

typedef struct
{
//===============---------STORAGE属性---------==============//
#define EZCAN_COMMON_STORAGESAMPLE_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_START_TOK

#define EZCAN_COMMON_STORAGESAMPLE_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_TOK
}EzCANCommonStorageSampleTyp;

enum
{
#define EZCAN_COMMON_COMMON_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_START_TOK

#define EZCAN_COMMON_COMMON_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_TOK

	COMMON_END,
};
#define COMMON_NUM (COMMON_END - COMMON_COMMON_DEVTYPE)

enum
{
#define EZCAN_COMMON_BOOST_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_START_TOK

#define EZCAN_COMMON_BOOST_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_TOK

	BOOST_END,
};
#define BOOST_NUM (BOOST_END - COMMON_BOOST_BOOSTCH1)

enum
{
#define EZCAN_COMMON_STORAGESAMPLE_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_START_TOK

#define EZCAN_COMMON_STORAGESAMPLE_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_TOK

	STORAGESAMPLE_END,
};
#define STORAGESAMPLE_NUM (STORAGESAMPLE_END - COMMON_STORAGESAMPLE_STORAGESAMPLECH1)

/*
*********************************************************************************************************
*                                         EzCAN 公共属性处理函数
*********************************************************************************************************
*/
void EzCANCommonProcess(CAN_msg *msg);
void EzCANCommonParamsSaveToFlash(u32 addr);

#endif
