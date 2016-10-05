/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCAN.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN协议的外部接口
********************************************************************************/
#ifndef __EZ_CAN_SERVO_SPECIAL_HANDLER_H
#define __EZ_CAN_SERVO_SPECIAL_HANDLER_H

#include "EzCAN.h"

/*
*********************************************************************************************************
*                                         EzCAN 属性内部参数计算函数
*********************************************************************************************************
*/
u8 EzCANCalcMotoSpeedPositiveAccTime(s32 data, u8, u8);
u8 EzCANCalcMotoSpeedNegativeAccTime(s32 data, u8, u8);
void EzCANCalcMotoPosCmdSmooth(u16 data);
void EzCANCalcPosCmdFIR(u16 data);
void EzCANCalcMotoTorqueKffFilter(u16 data);
void EzCANCalcMotoSpeedKffFilter(u16 data);
void EzCANCalcMotoSVSpeedCmdFilter(u16 data);
void EzCANCalcMotoSVTorqueCmdFilter(u16 data);
void EzCANCalcMotoPWMTorqueCmdFilter(u16 data);
void EzCANCalcMotoPWMSpeedCmdFilter(u16 data);
void EzCANCalcSpeedFeedbackFilter(u16 data);
void EzCANCalcTorqueFeedbackFilter(u16 data);

/*
*********************************************************************************************************
*                                         EzCAN 属性特殊处理函数
*********************************************************************************************************
*/
u8 EzCANControlTargetPositionHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANControlTargetSpeedHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANControlTargetVoltageHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANControlTargetCurrentHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANControlLoopModeHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANControlClearPositionHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANStatusNowCurrentDQHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANStatusNowVoltageDQHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANStatusNowSpeedHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANStatusNowPositionHandler(CAN_msg *pmsg, const struct ID_ITEM *item);

u8 EzCANMotoPosPIHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANMotoSpeedPIHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANMotoPositiveDirHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANMotoMaxTorqueHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANMotoMaxSpeedHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
u8 EzCANMotoSpeedPositiveAccTimeHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoSpeedNegativeAccTimeHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoPosCmdSmoothHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoPosCmdFIRHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoSpeedKffHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoTorqueKffHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoTorqueKffFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoSpeedKffFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoSVSpeedCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoSVTorqueCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoPWMSpeedCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoPWMTorqueCmdFilterHandler(CAN_msg *pmsg, const struct ID_ITEM *item);
u8 EzCANMotoTorquePIHandler(CAN_msg *msg, const struct ID_ITEM *item);
u8 EzCANMotoEncoderGapNumHandler(CAN_msg *msg, const struct ID_ITEM *item);
u8 EzCANMotoRefreshUploadHandler(CAN_msg *msg, const struct ID_ITEM *item);	
u8 EzCANMotoSpeedFilterHandler(CAN_msg *msg, const struct ID_ITEM *item);
u8 EzCANMotoTorqueFilterHandler(CAN_msg *msg, const struct ID_ITEM *item);

/*
*********************************************************************************************************
*                                         兰州交大~屏蔽门项目定制
*********************************************************************************************************
*/
bool EzCANControlDoorStateHandler(CAN_msg *msg, const struct ID_ITEM *pitem);

#endif
