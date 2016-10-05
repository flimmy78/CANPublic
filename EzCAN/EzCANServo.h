/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANServo.h
* Author             : ���
* Date First Issued  : 2011.08.05
* Description        : EzCAN�ŷ�������Э�鶨��
********************************************************************************/
#ifndef __EZ_CAN_SERVO_H
#define __EZ_CAN_SERVO_H
#include "EzCANOrgTypes.h"
#include "EzCANCommon.h"
#include "EzCANServoConfig.h"

/*
*********************************************************************************************************
*                                       EzCAN �ŷ�ö�ٳ��������Ͷ���
*********************************************************************************************************
*/
typedef enum {
	OP_COMPLETE,  		//��ɲ���
	OP_ILLEGAL_FUNC,	//�Ƿ�����
	OP_ILLEGAL_ADDR,	//�Ƿ���ַ����д��ַ����
	OP_ILLEGAL_VAL,		//�Ƿ�����ֵ����Ĭ�Ϸ�Χ�ü�
	OP_ILLEGAL_RW,		//��д�Ƿ�������дֻ�����ԣ����߶�ֻд����ʱ
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
*                                       EzCAN ��ǰ״̬����
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
*                                       EzCAN �ŷ��������ݽṹ
*
* Note: EZCAN_XX_XX_START_TOK ������ʼԪ�ص�ʹ�÷���
		EZCAN_XX_XX_TOK �������ʹ��EzCAN_Servo.protocolЭ��������ݱ�
*********************************************************************************************************
*/
//===============---------�ŷ����������---------==============//
typedef struct EzCANServoMotoST
{
#define EZCAN_SERVO_MOTO_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_START_TOK

#define EZCAN_SERVO_MOTO_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_TOK
}EzCANServoMotoTyp;

//===============---------�ŷ�����������---------==============//
typedef struct EzCANServoOthersST
{
#define EZCAN_SERVO_OTHERS_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_START_TOK

#define EZCAN_SERVO_OTHERS_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_TOK
}EzCANServoOthersTyp;

//===============---------�ŷ����ƶ�---------==============//
typedef struct EzCANServoControlST
{
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYP, NAME, RW, RANGE, HANDLER, ASCII) TYP NAME;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK
}EzCANServoControlTyp;

//===============---------�ŷ�״̬��---------==============//
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
*                                       EzCAN �ŷ�����ö��
*
* Note: EZCAN_XX_XX_START_TOK ������ʼԪ�ص�ʹ�÷���
		EZCAN_XX_XX_TOK �������ʹ��EzCAN_Servo.protocolЭ��������ݱ�
*********************************************************************************************************
*/
//===============---------�ŷ����������---------==============//
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

//===============---------�ŷ�����������---------==============//
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

//===============---------�ŷ����ƶ�---------==============//
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

//===============---------�ŷ�״̬��---------==============//
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
*                                         EzCAN �ŷ���Э�����Ա���
*********************************************************************************************************
*/
typedef struct
{
	//================--------�������Զ�-------===============//	
	EzCANCommonCommonTyp 				Common;
	EzCANCommonBoostTyp 				Boost;
	EzCANCommonStorageSampleTyp 		StorageSample;

	//================--------�ŷ����Զ�-------===============//	
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
*                                         EzCAN �ŷ��洢�������ֶ���
*  �洢����ֻ�ṩ�������״̬��Ĳ���
*  ����Ĵ洢����ͨ����Ƶ���
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
*                                         EzCAN �ŷ���Э�����Դ�����
*********************************************************************************************************
*/
void EzCANServoProcess(CAN_msg *msg);
void EzCANServoParamsInit(void);
void EzCANErrHistoryInit(void);
void EzCANServoParamsSaveToFlash(void);
bool EzCANGeneratePerfoamance(EzCANPerformModeTyp mode);
void EzCANPerformanceExamination(u8 *target, EzCANDataTypeTyp data_typ);

#endif
