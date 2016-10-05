/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANServo.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN伺服驱动子协议定义
********************************************************************************/
#ifndef __EZ_CAN_SERVO_H
#define __EZ_CAN_SERVO_H
#include "EzCANOrgTypes.h"
#include "EzCANCommon.h"
#include "EzCANServoConfig.h"

/*
*********************************************************************************************************
*                                       EzCAN 伺服枚举常量及类型定义
*********************************************************************************************************
*/
typedef enum {
	OP_COMPLETE,  		//完成操作
	OP_ILLEGAL_FUNC,	//非法功能
	OP_ILLEGAL_ADDR,	//非法地址，读写地址错误
	OP_ILLEGAL_VAL,		//非法数据值，按默认范围裁剪
	OP_ILLEGAL_RW,		//读写非法，比如写只读属性，或者读只写属性时
}EzCANOpErrorTyp;

typedef enum {
	NO_ERROR,
	BUS_OVR_VOL,
	BUS_UDR_VOL,
	BUS_OVR_CUR,
	OVR_HEAT,
	HALL_FAULT,
	POS_OVR_DIFF,
	OVR_LOAD,
	ENC_BREAK,
	ENC_FAULT,
	SPD_OVR_DIFF,
	OVR_SPD,
	PLS_FRQ_FAULT,
	PLS_DIV_FAULT,
	MOTO_RANGE_FAULT,
	FLASH_PARAM_FAULT,
	FLASH_CRC_FAULT,
	ANA_IN_OVR,
	ENC_Z_FAULT,
	MOTO_DIFF,
	MOTO_ZRO_POS_FAULT,
	SW_FAULT,
    CLOSE_TO_VOL,
}EzCANErrorCodeTyp;

typedef enum {
	OPEN_LOOP,
	TORQUE_LOOP,
	SPEED_TORQUE_LOOP,
	POS_TORQUE_LOOP,
	POS_SPEED_TORQUE_LOOP,
	SPEED_LOOP,
	POS_LOOP,
	POS_SPEED_LOOP,
}EzCANLoopModeTyp;

typedef enum {
	IDLE,
	INIT,
	START,
	RUN,
	BREAK,
	FAULT,
	GUIDE,
}EzCANStatusTyp;

typedef enum {
	NO_PERF,
	STEP,
	TRIANGLE,
	SQUARE,
	SIN,
}EzCANPerformModeTyp;

typedef enum {
	GUIDE_READY,
	POWER_LINK,
	HALL_LINK,
	ENC_LINK,
	ELEC_ANGLE,
}EzCANGuideStepTyp;

/*
*********************************************************************************************************
*                                       EzCAN 当前状态定义
*
* Note: 
*********************************************************************************************************
*/
#define EZCAN_SERVO_STATUS_SYS_ERR 	(u64)(((u64)1)<<0)
#define EZCAN_SERVO_STATUS_POS_AT	(u64)(((u64)1)<<1)
#define EZCAN_SERVO_STATUS_SPD_AT	(u64)(((u64)1)<<2)
#define EZCAN_SERVO_STATUS_ACC_DONE (u64)(((u64)1)<<3)
#define EZCAN_SERVO_STATUS_TO_ZERO	(u64)(((u64)1)<<4)
#define EZCAN_SERVO_STATUS_USR		(u64)(((u64)1)<<32)

/*
*********************************************************************************************************
*                                       EzCAN 伺服属性数据结构
*
* Note: EZCAN_XX_XX_START_TOK 定义起始元素的使用方法
		EZCAN_XX_XX_TOK 定义如何使用EzCAN_Servo.protocol协议里的数据表
*********************************************************************************************************
*/
//===============---------伺服电机参数段---------==============//
typedef struct EzCANServoMotoST
{
#define EZCAN_SERVO_MOTO_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_START_TOK

#define EZCAN_SERVO_MOTO_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_TOK
}EzCANServoMotoTyp;

//===============---------伺服其他参数段---------==============//
typedef struct EzCANServoOthersST
{
#define EZCAN_SERVO_OTHERS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_START_TOK

#define EZCAN_SERVO_OTHERS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_TOK
}EzCANServoOthersTyp;

//===============---------伺服控制段---------==============//
typedef struct EzCANServoControlST
{
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK
}EzCANServoControlTyp;

//===============---------伺服状态段---------==============//
typedef struct EzCANServoStatusST
{
#define EZCAN_SERVO_STATUS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_START_TOK

#define EZCAN_SERVO_STATUS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_TOK

}EzCANServoStatusTyp;

/*
*********************************************************************************************************
*                                       EzCAN 伺服属性枚举
*
* Note: EZCAN_XX_XX_START_TOK 定义起始元素的使用方法
		EZCAN_XX_XX_TOK 定义如何使用EzCAN_Servo.protocol协议里的数据表
*********************************************************************************************************
*/
//===============---------伺服电机参数段---------==============//
enum
{
#define EZCAN_SERVO_MOTO_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_START_TOK

#define EZCAN_SERVO_MOTO_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_TOK

	SERVO_MOTO_END
};
#define SERVO_MOTO_NUM (SERVO_MOTO_END - SERVO_MOTO_POSITIVEDIR)

//===============---------伺服其他参数段---------==============//
enum
{
#define EZCAN_SERVO_OTHERS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_START_TOK

#define EZCAN_SERVO_OTHERS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_TOK

	SERVO_OTHERS_END
};
#define SERVO_OTHERS_NUM (SERVO_OTHERS_END - SERVO_OTHERS_CANBAUDATE)

//===============---------伺服控制段---------==============//
enum
{
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK

	SERVO_CONTROL_END
};
#define SERVO_CONTROL_NUM (SERVO_CONTROL_END - SERVO_CONTROL_ENABLE)

//===============---------伺服状态段---------==============//
enum
{
#define EZCAN_SERVO_STATUS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID=BEGIN,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_START_TOK

#define EZCAN_SERVO_STATUS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) ID,
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_TOK

	SERVO_STATUS_END
};
#define SERVO_STATUS_NUM (SERVO_STATUS_END - SERVO_STATUS_ELECTRICANGULAR)

/*
*********************************************************************************************************
*                                         EzCAN 伺服子协议属性变量
*********************************************************************************************************
*/
typedef struct
{
	//================--------公共属性段-------===============//	
	EzCANCommonCommonTyp 				Common;
	EzCANCommonBoostTyp 				Boost;
	EzCANCommonStorageSampleTyp 		StorageSample;

	//================--------伺服属性段-------===============//	
	volatile EzCANServoMotoTyp 			Moto;
	volatile EzCANServoOthersTyp		Others;
	volatile EzCANServoControlTyp 		Control;
	volatile EzCANServoStatusTyp		Status;
}EzCANServoTyp;

extern EzCANServoTyp EzCANServoParams;
extern u8 EzCANDeviceID;
extern u8 EzCANPerformanceBuffer[PERFORM_BUFFER_NUM];

/*
*********************************************************************************************************
*                                         EzCAN 伺服存储采样部分定义
*  存储采样只提供控制类和状态类的采样
*  这里的存储采样通道设计的是
*********************************************************************************************************
*/
typedef struct
{
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) u8 NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) u8 NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK			

#define EZCAN_SERVO_STATUS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) u8 NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_START_TOK

#define EZCAN_SERVO_STATUS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) u8 NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_TOK

}EzCANStorageSampleChannelTyp;

extern EzCANStorageSampleChannelTyp EzCANStorageSampleChannel;

/*
*********************************************************************************************************
*                                         EzCAN 伺服子协议属性处理函数
*********************************************************************************************************
*/
void EzCANServoProcess(CAN_msg *msg);
void EzCANServoParamsInit(void);
void EzCANErrHistoryInit(void);
void EzCANServoParamsSaveToFlash(void);
bool EzCANGeneratePerfoamance(EzCANPerformModeTyp mode);
void EzCANPerformanceExamination(u8 *target, EzCANDataTypeTyp data_typ);

#endif
