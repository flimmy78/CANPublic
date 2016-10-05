/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANServoConfig.h
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN配置文件
********************************************************************************/
#ifndef __EZ_CAN_SERVO_CONFIG_H
#define __EZ_CAN_SERVO_CONFIG_H

/*
*********************************************************************************************************
*                                         EzCAN Servo 子协议采样配置
*********************************************************************************************************
*/
#define BOOST_CH_NUM		8
#define STORAGE_CH_NUM		4
#define STORAGE_BUF_LEN		1024		//8的倍数
#define ERR_HISTORY_NUM		8
#define PERFORM_BUFFER_NUM  800

//==================----1.属性所属存储采样同步信号源----==================//
#define STORAGE_SAMPLE_SYN_SRC_NUM	3		//同步信号源最多8个

#define SERVO_STATUS_NOWPOSITION_SYN_SRC 				(&StorageSampleSyn[0])
#define SERVO_STATUS_INTERNALTARGETPOSITION_SYN_SRC 	(&StorageSampleSyn[0])
#define SERVO_CONTROL_TARGETPOSITION_SYN_SRC			(&StorageSampleSyn[0])

#define SERVO_STATUS_NOWSPEED_SYN_SRC					(&StorageSampleSyn[1])
#define SERVO_CONTROL_TARGETSPEED_SYN_SRC				(&StorageSampleSyn[1])
#define SERVO_STATUS_INTERNALTARGETSPEED_SYN_SRC		(&StorageSampleSyn[1])

#define SERVO_STATUS_NOWCURRENTDQ_SYN_SRC				(&StorageSampleSyn[2])
#define SERVO_CONTROL_TARGETCURRENT_SYN_SRC				(&StorageSampleSyn[2])
#define SERVO_STATUS_NOWCURRABQ_SYN_SRC					(&StorageSampleSyn[2])

//==================----2.同步信号源所有属性----==================//
#ifndef SYNLIST
#define SYNLIST(...) {__VA_ARGS__}
#endif

#define STORAGE_SAMPLE_SYN_PROPS_0 \
	SYNLIST(SERVO_STATUS_NOWPOSITION, SERVO_STATUS_INTERNALTARGETPOSITION, SERVO_CONTROL_TARGETPOSITION)

#define STORAGE_SAMPLE_SYN_PROPS_1 \
	SYNLIST(SERVO_STATUS_NOWSPEED, SERVO_CONTROL_TARGETSPEED, SERVO_STATUS_INTERNALTARGETSPEED)

#define STORAGE_SAMPLE_SYN_PROPS_2 \
	SYNLIST(SERVO_STATUS_NOWCURRENTDQ, SERVO_CONTROL_TARGETCURRENT, SERVO_STATUS_NOWCURRAB)

#endif
