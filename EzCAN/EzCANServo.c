/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANServo.c
* Author             : 徐成
* Date First Issued  : 2011.08.05
* Description        : EzCAN伺服子协议部分协议实现
********************************************************************************/
#include "EzCAN.h"
#include "Consts.h"
#include <string.h>
#include <math.h>

/*
*********************************************************************************************************
*                                         EzCAN 属性特殊处理函数
*********************************************************************************************************
*/
//==================-----公共属性特殊处理函数-----=================//
u8 EzCANCommonSaveToFlashHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem);
u8 EzCANCommonRestoreHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem);
u8 EzCANCommonBoostHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonStorageSampleHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonResetSystemHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonSetDevIDHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonIdentifyHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommomStorageSwitchHandler(CAN_msg *pmsg, const IDItemTyp *item);

//==================-----伺服属性特殊处理函数-----=================//
u8 EzCANCommonErrorHistoryHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANServoPerformanceStateHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
#include "EzCANServoSpecialHandler.h"

/*
*********************************************************************************************************
*                                         EzCAN 数据范围定义
*********************************************************************************************************
*/
//==================--------EzCAN公共属性范围-------================//
#define EZCAN_COMMON_COMMON_RANGE(Typ, Name, Range) Typ Name = Range;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_RANGE
										 
//==================--------EzCAN伺服属性范围-------================//
#define EZCAN_SERVO_MOTO_RANGE(Typ, Name, Range) static Typ Name = Range;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_RANGE

#define EZCAN_SERVO_OTHERS_RANGE(Typ, Name, Range) static Typ Name = Range;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_RANGE

#define EZCAN_SERVO_CONTROL_RANGE(Typ, Name, Range) static Typ Name = Range;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_RANGE

#define EZCAN_SERVO_STATUS_RANGE(Typ, Name, Range) static Typ Name = Range;
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_RANGE

/*
*********************************************************************************************************
*                                         EzCAN 公共属性变量声明
*********************************************************************************************************
*/
EzCANStorageSampleChannelTyp EzCANStorageChannel;
/*static */unsigned char StorageData[STORAGE_CH_NUM][STORAGE_BUF_LEN];
/*static */EzCANStorageSampleDataTyp EzCANStorageSampleData[STORAGE_CH_NUM];

//==================----4.同步信号源属性列表定义----==================//
#if STORAGE_SAMPLE_SYN_SRC_NUM > 0
u16 StorageSampleSynProps_0[] = STORAGE_SAMPLE_SYN_PROPS_0;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 1
u16 StorageSampleSynProps_1[] = STORAGE_SAMPLE_SYN_PROPS_1;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 2
u16 StorageSampleSynProps_2[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 3
u16 StorageSampleSynProps_3[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 4
u16 StorageSampleSynProps_4[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 5
u16 StorageSampleSynProps_5[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 6
u16 StorageSampleSynProps_6[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 7
u16 StorageSampleSynProps_7[] = STORAGE_SAMPLE_SYN_PROPS_2;
#endif

//==================----3.同步信号源定义----======================//
EzCANStorageSynTyp StorageSampleSyn[STORAGE_SAMPLE_SYN_SRC_NUM] = {
#if STORAGE_SAMPLE_SYN_SRC_NUM > 0
	{StorageSampleSynProps_0, sizeof(StorageSampleSynProps_0)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 1
	{StorageSampleSynProps_1, sizeof(StorageSampleSynProps_1)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 2
	{StorageSampleSynProps_2, sizeof(StorageSampleSynProps_2)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 3
	{StorageSampleSynProps_3, sizeof(StorageSampleSynProps_3)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 4
	{StorageSampleSynProps_4, sizeof(StorageSampleSynProps_4)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 5
	{StorageSampleSynProps_5, sizeof(StorageSampleSynProps_5)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 6
	{StorageSampleSynProps_6, sizeof(StorageSampleSynProps_6)/sizeof(u16), 0},
#endif

#if STORAGE_SAMPLE_SYN_SRC_NUM > 7
	{StorageSampleSynProps_7, sizeof(StorageSampleSynProps_7)/sizeof(u16), 0},
#endif
};

/*
*********************************************************************************************************
*                                         EzCAN 伺服属性列表
*********************************************************************************************************
*/
EzCANServoTyp EzCANServoParams;
u8 EzCANPerformanceBuffer[PERFORM_BUFFER_NUM];		//电流环是s16类型，速度环是s32类型，位置环也是s32类型
u8 EzCANDeviceID;

const IDItemTyp EzCANServoItems[] = 
{
//===============---------公共属性段---------==============//
#define EZCAN_COMMON_COMMON_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Common.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_START_TOK

#define EZCAN_COMMON_COMMON_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Common.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_TOK

//===============---------公共BOOST属性段---------==============//
#define EZCAN_COMMON_BOOST_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Boost.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_START_TOK

#define EZCAN_COMMON_BOOST_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Boost.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_TOK

//===============---------公共存储采样属性段---------==============//
#define EZCAN_COMMON_STORAGESAMPLE_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.StorageSample.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_START_TOK

#define EZCAN_COMMON_STORAGESAMPLE_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.StorageSample.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_TOK


//===============---------伺服电机参数段---------==============//
#define EZCAN_SERVO_MOTO_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Moto.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_START_TOK

#define EZCAN_SERVO_MOTO_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Moto.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_TOK

//===============---------伺服其他参数段---------==============//
#define EZCAN_SERVO_OTHERS_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Others.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_START_TOK

#define EZCAN_SERVO_OTHERS_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Others.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_TOK

//===============---------伺服控制段---------==============//
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Control.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Control.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK

//===============---------伺服状态段---------==============//
#define EZCAN_SERVO_STATUS_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Status.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_START_TOK

#define EZCAN_SERVO_STATUS_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Status.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_TOK
};

/*
*********************************************************************************************************
*                                         EzCAN 处理线程调用接口
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANMessageHandle
* Description    : 伺服系统消息操作函数
* Input          : msg: CAN数据包
                   index: 消息索引
* Output         : None
* Return         : None
*******************************************************************************/
__inline void EzCANMessageHandle(CAN_msg *msg, u16 index)
{
	U32Params err_info;
	u8 err;

	if(FETCH_PROP(msg->id) == COMMON_COMMON_ACK
		&& msg->type == DATA_FRAME)
		return ;

	if(EzCANServoItems[index].Handler != NULL)  
	{ 
		err = EzCANServoItems[index].Handler(msg, &EzCANServoItems[index]);
		if( (msg->type == DATA_FRAME && EzCANServoParams.Common.EnableACK)	//如果是数据帧，并且使能了ACK
			|| (msg->type == REMOTE_FRAME && err == OP_ILLEGAL_FUNC) )	//如果请求了只读数据
		{ 
			//返回ACK
			err_info.Param1 = msg->id; 
			err_info.Param2 = err; 		//具体定义参考EzCAN文档
			EzCANSend(BUILD_ID(FETCH_PRIO(msg->id), EzCANDeviceID, COMMON_COMMON_ACK), 
					&err_info, 
					sizeof(U32Params),
					msg->ch);
		}
	}
}

/*******************************************************************************
* Function Name  : EzCANServoProcess
* Description    : 伺服系统消息处理
* Input          : msg: CAN数据包
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoProcess(CAN_msg *msg)
{
	u16 prop = msg->id;
	u16 index = 0;
	u16 total_num = 0;
	U32Params err_info;

	//==================---------公共属性段---------=================//
	index = prop - COMMON_COMMON_DEVTYPE;
	if(prop < COMMON_NUM)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += COMMON_NUM;

	index = prop - COMMON_BOOST_BOOSTCH1 + total_num;
	if(prop >= COMMON_BOOST_BOOSTCH1 && prop < BOOST_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += BOOST_NUM;

	index = prop - COMMON_STORAGESAMPLE_STORAGESAMPLECH1 + total_num;
	if(prop >= COMMON_STORAGESAMPLE_STORAGESAMPLECH1 && prop < STORAGESAMPLE_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += STORAGESAMPLE_NUM;

	//==================---------伺服属性段---------=================//
	index = prop - SERVO_MOTO_POSITIVEDIR + total_num;
	if(prop >= SERVO_MOTO_POSITIVEDIR && prop < SERVO_MOTO_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += SERVO_MOTO_NUM;

	index = prop - SERVO_OTHERS_CANBAUDATE + total_num;
	if(prop >= SERVO_OTHERS_CANBAUDATE && prop < SERVO_OTHERS_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += SERVO_OTHERS_NUM;

	index = prop - SERVO_CONTROL_ENABLE + total_num;
	if(prop >= SERVO_CONTROL_ENABLE && prop < SERVO_CONTROL_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}
	total_num += SERVO_CONTROL_NUM;

	index = prop - SERVO_STATUS_ELECTRICANGULAR + total_num;
	if(prop >= SERVO_STATUS_ELECTRICANGULAR && prop < SERVO_STATUS_END)
	{
		EzCANMessageHandle(msg, index);
		return;
	}

	//如果不属于地址范围，则用ACK报错
	err_info.Param1 = msg->id; 
	err_info.Param2 = OP_ILLEGAL_ADDR; 		//具体定义参考EzCAN文档
	EzCANSend(BUILD_ID(FETCH_PRIO(msg->id), EzCANDeviceID, COMMON_COMMON_ACK), 
		&err_info, 
		sizeof(U32Params),
		msg->ch);
}

/*******************************************************************************
* Function Name  : EzCANBoostProcess
* Description    : Boost数据处理函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANBoostProcess(void)
{
	static u8 escapetime[BOOST_CH_NUM];
	u32 i;
	CAN_msg msg;
	Boost *boostStart = &EzCANServoParams.Boost.BoostCh1;

	if(EzCANServoParams.Common.BoostSwitch)
	{
		for(i=0; i<BOOST_CH_NUM; i++)
		{
			if(boostStart->Switch == 1)
			{
				if(escapetime[i] <= 0)
				{
					//BOOST类型消息优先级只能是0x0F，最低优先级
					msg.id = (boostStart->Id & ~PRIO_MASK) | (BOOST_MESSAGE_PRIO << 24);
					msg.type = REMOTE_FRAME;
					msg.format = EXTENDED_FORMAT;
					msg.len = 8;

					PostToEzCAN(&msg);
	
					escapetime[i] = boostStart->Interval;
				}
				else
				{
					escapetime[i]--;
				}
			}
			boostStart++;
		}
	}
}

/*******************************************************************************
* Function Name  : EzCANStorageSampleProcess
* Description    : 存储采样数据处理函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__inline void EzCANStorageSampleSendReadyData(u16 index)
{
	u16 j;
	u32 id;
	u16 cnt = STORAGE_BUF_LEN / EzCANStorageSampleData[index].TypeLength;
	
	for(j=0; j<cnt; j++)
	{
		id = BUILD_ID(STORAGE_SAMPLE_PRIO, EzCANDeviceID, EzCANStorageSampleData[index].Property);

		EzCANSend(  id, 
					EzCANStorageSampleData[index].Buffer + EzCANStorageSampleData[index].BufferLength,
					EzCANStorageSampleData[index].TypeLength,
					0);		//MODBUS不支持存储采样，因此随便传一个数即可

		EzCANStorageSampleData[index].BufferLength += EzCANStorageSampleData[index].TypeLength;

		EzCANTimeDly(STORAGE_DLY_TIM);
	}

	EzCANStorageSampleData[index].TypeLength = 4;
	EzCANStorageSampleData[index].BufferLength = 0;
	EzCANStorageSampleData[index].AlreadyReset = 0;
}

void EzCANStorageSampleProcess(void)
{
	u16 i, j;
	EzCANStorageSynTyp *pStrSyn = NULL;
	u16 ch_temp;
	u16 ch_comp_flag = 0;
	u16 syn_comp_flag = 0;

	if(EzCANServoParams.Common.StorageSwitch)
	{
		for(i=0; i<STORAGE_CH_NUM; i++)
			ch_comp_flag |= (1<<i);

		//1.如果有某个通道完成了计数，则查询有相同同步信号的其他通道，等待其他通道完成采样
		//2.所有通道全部完成采样后，开启同步信号
		for(i=0; i<STORAGE_CH_NUM; i++)
		{
			if(EzCANStorageSampleData[i].Enable
				&& EzCANStorageSampleData[i].Ready
				&& ((ch_comp_flag>>i) & 0x0001 == 1))
			{
				ch_comp_flag &= ~(1<<i);		//设置通道发送完成标志
				pStrSyn = EzCANStorageSampleData[i].StorageSyn;
			
				EzCANStorageSampleSendReadyData(i);	//发送当前已完成通道的数据
					
				if(pStrSyn != NULL)
				{
					syn_comp_flag = pStrSyn->Channels;
					syn_comp_flag &= ~(1<<i);	//清当的同步信号通道位

					//如果还有其他的同步通道
					if(syn_comp_flag > 0)
					{
						//这里j循环的次数待调整
						for(j=0; j<sizeof(u16)*8; j++)
						{
							ch_temp = syn_comp_flag>>j;

							//如果当前通道是i一起的同步通道
							if(ch_temp & 0x0001 == 1)
							{
								while(!(EzCANStorageSampleData[j].Enable
										&& EzCANStorageSampleData[j].Ready))
								{
									OSTimeDly(5);
								}
	
								//发送当前已完成通道的数据
								ch_comp_flag &= ~(1<<j);
								EzCANStorageSampleSendReadyData(j);
		
								//如果遍历了所有的同步位，则跳出同步查询								
								syn_comp_flag &= ~(1<<j);
								if(syn_comp_flag == 0)
								{
									break;
								}
							}
						}
					}

					//1.先关总同步标志
					pStrSyn->Flag = 0;
					
					//2.开每个同步通道的Ready位
					syn_comp_flag = pStrSyn->Channels;
					for(j=0; j<sizeof(u16)*8; j++)
					{
						ch_temp = syn_comp_flag>>j;
						if(ch_temp & 0x0001 == 1)
						{
							EzCANStorageSampleData[j].Ready = 0;
						}
						syn_comp_flag &= ~(1<<j);
						if(syn_comp_flag == 0)
						{
							break;
						}
					}

					//3.开总同步标志
					pStrSyn->Flag = 1;
				} 				
				else
				{
					EzCANStorageSampleData[i].Ready = 0;
				}
			}
		}
	}
}


/*
*********************************************************************************************************
*                                         EzCAN 其他调用接口
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANServoParamsSaveToFlash
* Description    : 参数保存到Flash
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoParamsSaveToFlash(void)
{
	u32 i;
	u32 *pdata;
	u32 plus_elem_size = 0;
	u32 start_page = 0;

	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	NVIC_SETPRIMASK();

	//擦除FLASH	  		
	FLASH_Unlock();	
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
	for(i = 0; i < 3; i++)
	{
		if(FLASH_ErasePage(SERVO_START_PAGE + i * 0x400) != FLASH_COMPLETE)
		{
			return;
		}
	} 

	//写入公共属性--DevID
	pdata = (u32 *)&EzCANServoParams.Common.DevID;
	start_page = SERVO_START_PAGE;
	plus_elem_size = (sizeof(EzCANServoParams.Common.DevID) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}
	
	//写入公共属性--DevPriority
	pdata = (u32 *)&EzCANServoParams.Common.DevPriority;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Common.DevPriority) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}

	//写入公共属性--MasterID
	pdata = (u32 *)&EzCANServoParams.Common.MasterID;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Common.MasterID) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}

	//写入Moto属性
	pdata = (u32 *)&EzCANServoParams.Moto;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Moto) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE) 
			return;
	}

	//写入其他属性
	pdata = (u32 *)&EzCANServoParams.Others;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Others) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return;
	}

	FLASH_Lock();
	
	NVIC_RESETPRIMASK();
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/*******************************************************************************
* Function Name  : EzCANSaveErrorHistoryToFlash
* Description    : 参数恢复出厂设置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSaveErrorHistoryToFlash(void)
{
	u32 *pdata;
	u8 i;

	NVIC_SETPRIMASK();

	//擦除FLASH	  		
	FLASH_Unlock();	
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
	FLASH_ErasePage(ERROR_HISTORY_START_PAGE);

	//写入其他属性
	pdata = (u32 *)&EzCANServoParams.Common.ErrorHistory;
	for(i = 0 ; i < sizeof(EzCANServoParams.Common.ErrorHistory)/4 ; i ++)
		FLASH_ProgramWord(ERROR_HISTORY_START_PAGE + i * 4, *pdata ++);

	FLASH_Lock();
	
	NVIC_RESETPRIMASK(); 
}

/*******************************************************************************
* Function Name  : EzCANServoParamsRestore
* Description    : 参数恢复出厂设置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoParamsRestore(u8 binit)
{
	u16 i;

//===============---------可保存段初始化为0--------============//
	memset((void*)&EzCANServoParams.Common, 0, sizeof(EzCANServoParams.Common));
	memset((void*)&EzCANServoParams.Moto, 0, sizeof(EzCANServoParams.Moto));
	memset((void*)&EzCANServoParams.Others, 0, sizeof(EzCANServoParams.Others));
	memset((void*)&EzCANServoParams.Status, 0, sizeof(EzCANServoParams.Others));
	memset((void*)&EzCANServoParams.Control, 0, sizeof(EzCANServoParams.Others));

//===============---------公共属性段配置---------==============//
#define EZCAN_COMMON_COMMON_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Common.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_INIT

	//初始化存储采样数据区
	for(i=0; i<STORAGE_CH_NUM; i++)
	{
		EzCANStorageSampleData[i].Buffer = StorageData[i];
		EzCANStorageSampleData[i].BufferLength = 0;
	}

	//设备ID不能变
	EzCANServoParams.Common.DevID = EzCANDeviceID;

//===============---------伺服电机参数段---------==============//
#define EZCAN_SERVO_MOTO_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Moto.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_INIT

//===============---------伺服其他参数段---------==============//
#define EZCAN_SERVO_OTHERS_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Others.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_INIT

//如果不是初始化，则不需要恢复这一段
if(binit)
{
//===============---------伺服控制段---------==============//
#define EZCAN_SERVO_CONTROL_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Control.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_INIT

//===============---------伺服状态段---------==============//
#define EZCAN_SERVO_STATUS_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Status.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_INIT	
}

}

/*******************************************************************************
* Function Name  : EzCANServoParamsInit
* Description    : 参数初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoParamsInit(void)
{
	u8 *param = (u8*)SERVO_START_PAGE;
	u8 *param_err_history = (u8*)ERROR_HISTORY_START_PAGE;
	u32 plus_elem_size = 0;
	
	EzCANServoParamsRestore(TRUE);
	if( *((s32*)param) != -1)
	{
		EzCANServoParams.Common.ErrorHistory = *((u64*)param_err_history);

		EzCANServoParams.Common.DevID = *((u32*)param);
		EzCANDeviceID = EzCANServoParams.Common.DevID;
		plus_elem_size = (sizeof(EzCANServoParams.Common.DevID) + (sizeof(u32) - 1)) / sizeof(u32);

		param += sizeof(u32) * plus_elem_size;
		EzCANServoParams.Common.DevPriority = *((u32*)param);
		plus_elem_size = (sizeof(EzCANServoParams.Common.DevPriority) + (sizeof(u32) - 1)) / sizeof(u32);

		param += sizeof(u32) * plus_elem_size;
		EzCANServoParams.Common.MasterID = *((u32*)param);
		plus_elem_size = (sizeof(EzCANServoParams.Common.MasterID) + (sizeof(u32) - 1)) / sizeof(u32);

		param += sizeof(u32) * plus_elem_size;
		EzCANServoParams.Moto = *((EzCANServoMotoTyp*)param);
		plus_elem_size = (sizeof(EzCANServoParams.Moto) + (sizeof(u32) - 1)) / sizeof(u32);

		param += sizeof(u32) * plus_elem_size;
		EzCANServoParams.Others = *((EzCANServoOthersTyp*)param);
	}	
	else
	{
		EzCANServoParamsSaveToFlash();
		EzCANSaveErrorHistoryToFlash();		
	}

	memset((void*)&EzCANStorageChannel, 0xFF, sizeof(EzCANStorageChannel));
}

/*******************************************************************************
* Function Name  : EzCANStorageSample
* Description    : 存储采样接口函数
* Input          : prop: 采样值属性
                   data: 采样值
				   typelen: 类型长度 
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANStorageSample(u32 prop, void* data, u8 typelen)
{
	u8 b_syn = 0;
	u8 index, i;

	if(EzCANServoParams.Common.StorageSwitch)
	{
		//找到prop在EzCANStorageChannel里面的索引号
		index = prop - SERVO_CONTROL_ENABLE;
		if(prop >= SERVO_OTHERS_END && prop < SERVO_CONTROL_END)
		{
			i = *((u8*)&EzCANStorageChannel + index);
		}
		else
		{
			index = prop - SERVO_STATUS_ELECTRICANGULAR + SERVO_CONTROL_NUM;
			if(prop >= SERVO_CONTROL_END && prop < SERVO_STATUS_END)
			{
				i = *((u8*)&EzCANStorageChannel + index);	
			}
			else
			{
				return;
			}
		}

	 	//判断同步采样是否要在性能测试的时候关闭
		if(EzCANServoParams.Control.StrSampleSyncOnPerformSwitch == 1
			&& EzCANServoParams.Control.PerformanceState == NO_PERF)
		{
			//如果长度等于0，则表明已经完成发送或者还没开始性能测试，直接返回
			if(EzCANStorageSampleData[i].BufferLength == 0)
			{
				return;
			}
		}

		//重置当前采样，将BufferLength设置为0，从头开始采
		if(EzCANServoParams.Control.PerformanceState != NO_PERF
			&& EzCANStorageSampleData[i].AlreadyReset == 0)
		{
			EzCANStorageSampleData[i].BufferLength = 0;
			EzCANStorageSampleData[i].Ready = 0;
			EzCANStorageSampleData[i].AlreadyReset = 1;
		}

		//判断存储采样同步信号
		if(EzCANStorageSampleData[i].StorageSyn == NULL)
		{
			b_syn = 1;
		}
		else
		{
			if(EzCANStorageSampleData[i].StorageSyn->Flag == 0)
			{
				b_syn = 0;	
			}
			else
			{
				b_syn = 1;
			}
		}

		//采样数据
		if(EzCANStorageSampleData[i].Enable == 1			//设备使能
			&& EzCANStorageSampleData[i].Ready == 0			//存储采样还未完成，需要继续采样
			&& b_syn == 1)
		{
			memcpy(EzCANStorageSampleData[i].Buffer + EzCANStorageSampleData[i].BufferLength, data, typelen);
			EzCANStorageSampleData[i].BufferLength += typelen;

			if(EzCANStorageSampleData[i].BufferLength + typelen > STORAGE_BUF_LEN)
			{
				EzCANStorageSampleData[i].BufferLength = 0;
				EzCANStorageSampleData[i].TypeLength = typelen;
				EzCANStorageSampleData[i].Ready = 1;
			}
		}
	}
}

void EzCANAddErrHistory(u8 err)
{
	static u8 last_err;

	if(err == last_err)
	{
		return;
	}

	last_err = err;

	EzCANServoParams.Common.ErrorHistory <<= 8;
	EzCANServoParams.Common.ErrorHistory |= err;

	EzCANSaveErrorHistoryToFlash();
}

/*
*********************************************************************************************************
*                                         EzCAN 公共属性特殊处理函数实现
*********************************************************************************************************
*/
u8 EzCANCommonSaveToFlashHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem)
{
	if(pmsg->type == DATA_FRAME)
		EzCANAllParamsSaveToFlash();

	return OP_COMPLETE;
}

u8 EzCANCommonRestoreHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem)
{
	if(pmsg->type == DATA_FRAME)
	{
		if(*((u8*)pmsg->data) == 1)
		{
			EzCANServoParamsRestore(FALSE);
		}
	}

	return OP_COMPLETE;
}

u8 EzCANCommonBoostHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	u8 i;
	Boost *p_msg_boost;
	Boost *p_boost = &EzCANServoParams.Boost.BoostCh1;
	
	if(pmsg->type == DATA_FRAME)
	{
		p_msg_boost = (Boost*)pmsg->data;

		//检查是否有重复设置的通道，如果有，则关闭该通道
		for(i=0; i<BOOST_CH_NUM; i++)
		{
			if(p_msg_boost->Switch == p_boost->Switch
				&& p_msg_boost->Id == p_boost->Id)
			{
				return OP_COMPLETE;
			}

			p_boost++;
		}
		*((Boost*)item->Data) = *((Boost*)pmsg->data);
	}
	else
	{
		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(Boost), pmsg->ch);
	}

	return OP_COMPLETE;
}

u8 EzCANCommonStorageSampleHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	U32Params *params;
	u32 index = ((u32)((U32Params*)item->Data - (U32Params*)&EzCANServoParams.StorageSample));///sizeof(U32Params);
	u16 i, j, pre_prop;
	EzCANStorageSynTyp *pStrSyn;

	if(pmsg->type == DATA_FRAME)
	{
		if(index >= STORAGE_CH_NUM)
			return OP_ILLEGAL_ADDR;

		params = (U32Params*)pmsg->data;

		//修改EzCANStorageSampleChannel原先属于index的属性
		pre_prop = EzCANStorageSampleData[index].Property;
		i = pre_prop - SERVO_CONTROL_ENABLE;
		if(pre_prop >= SERVO_OTHERS_END && pre_prop < SERVO_CONTROL_END)
		{
			*((u8*)&EzCANStorageChannel + i) = 0xFF;
		}
		else
		{
			i = pre_prop - SERVO_STATUS_ELECTRICANGULAR + SERVO_CONTROL_NUM;
			if(pre_prop >= SERVO_CONTROL_END && pre_prop < SERVO_STATUS_END)
			{
				*((u8*)&EzCANStorageChannel + i) = 0xFF;
			}
		}

		EzCANStorageSampleData[index].Property = params->Param1;
		EzCANStorageSampleData[index].Enable = params->Param2;
		EzCANStorageSampleData[index].TypeLength = 4;
		EzCANStorageSampleData[index].BufferLength = 0;
		EzCANStorageSampleData[index].AlreadyReset = 0;

		//如果该通道以前已经在使用，则先删掉该通道的同步信号内相关数据
		if(EzCANStorageSampleData[index].StorageSyn != NULL)
		{
			EzCANStorageSampleData[index].StorageSyn->Channels &= ~(1<<index);
			EzCANStorageSampleData[index].StorageSyn = NULL;
		}

		//存储到EzCANStorageSampleChannel里面去
		i = params->Param1 - SERVO_CONTROL_ENABLE;
		if(params->Param1 >= SERVO_OTHERS_END && params->Param1 < SERVO_CONTROL_END)
		{
			*((u8*)&EzCANStorageChannel + i) = index;
		}
		else
		{
		
			i = params->Param1 - SERVO_STATUS_ELECTRICANGULAR + SERVO_CONTROL_NUM;
			if(params->Param1 >= SERVO_CONTROL_END && params->Param1 < SERVO_STATUS_END)
			{
				*((u8*)&EzCANStorageChannel + i) = index;
			}
			else
			{
				return OP_ILLEGAL_FUNC;
			}
		}

		for(i=0; i<STORAGE_SAMPLE_SYN_SRC_NUM; i++)
		{
			pStrSyn = &StorageSampleSyn[i];
			for(j=0; j<pStrSyn->ArraySize; j++)
			{
				if(pStrSyn->StorageProps[j] == params->Param1)
				{
					EzCANStorageSampleData[index].StorageSyn = pStrSyn;
					//写入通道号
					pStrSyn->Channels |= (1<<index);
					goto start_sample;
				}
			}
		}

		//开通道采样开关
start_sample:
		EzCANStorageSampleData[index].Ready = 0;

		*((U32Params*)item->Data) = *params;
	}
	else
	{
		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(U32Params), pmsg->ch);		
	}

	return OP_COMPLETE;
}

u8 EzCANCommomStorageSwitchHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	u32 _switch;
	u8 i, j;
	EzCANStorageSynTyp *pStrSyn = NULL;
	u16 ch_comp_flag, syn_comp_flag, ch_temp;

	if(pmsg->type == DATA_FRAME)
	{
		_switch = *((u8*)pmsg->data);

		if(_switch == 1)
		{
			ch_comp_flag = 0;
			for(i=0; i<STORAGE_CH_NUM; i++)
				ch_comp_flag |= (1<<i);

			for(i=0; i<STORAGE_CH_NUM; i++)
			{
				pStrSyn = EzCANStorageSampleData[i].StorageSyn;

				if(pStrSyn == NULL)
				{
					EzCANStorageSampleData[i].BufferLength = 0;
					EzCANStorageSampleData[i].Ready = 0;
					EzCANStorageSampleData[i].AlreadyReset = 0;

					ch_comp_flag &= ~(1<<i);
				}
				else
				{
					//如果已经处理过该同步通道，则继续处理其他通道
					ch_temp = ch_comp_flag>>i;
					if(ch_temp & 0x0001 == 0)
						continue;

					//1.关总同步标志
					pStrSyn->Flag = 0;
					
					//2.开每个同步通道的Ready位
					syn_comp_flag = pStrSyn->Channels;
					for(j=0; j<sizeof(u16)*8; j++)
					{
						ch_temp = syn_comp_flag>>j;
						if(ch_temp & 0x0001 == 1)
						{
							EzCANStorageSampleData[j].BufferLength = 0;
							EzCANStorageSampleData[j].Ready = 0;
							EzCANStorageSampleData[j].AlreadyReset = 0;
						}
						syn_comp_flag &= ~(1<<j);
						ch_comp_flag &= ~(1<<j);
						if(syn_comp_flag == 0)
						{
							break;
						}
					}

					//3.开总同步标志
					pStrSyn->Flag = 1;
				}
			}
		}
		

		*((u8*)item->Data) = _switch;
	}
	else
	{
		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), item->Data, sizeof(u8), pmsg->ch);		
	}

	return OP_COMPLETE;
}

u8 EzCANCommonResetSystemHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	if(pmsg->type == DATA_FRAME)
	{
		if(EzCANResetSystem != NULL)
		{
			EzCANResetSystem();
		}
	}

	return OP_COMPLETE;
}

u8 EzCANCommonSetDevIDHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
	if(pmsg->type == DATA_FRAME)
	{
		EzCANServoParams.Common.DevID = *((u32*)pmsg->data);
	}

	return OP_COMPLETE;
}

u8 EzCANCommonIdentifyHandler(CAN_msg *pmsg, const IDItemTyp *item)
{
 	u32 CPUID[3];
	U32Params data;
	if(pmsg->type == REMOTE_FRAME)
	{
		//获取CPUID
		CPUID[0] = *(u32*)(0x1FFFF7E8);
		CPUID[1] = *(u32*)(0x1FFFF7EC);
		CPUID[2] = *(u32*)(0x1FFFF7F0);

		data.Param1 = EzCANDeviceID;
		data.Param2 = (CPUID[0] + CPUID[1]>>1 + CPUID[2]>>2);

		EzCANSend(BUILD_ID(FETCH_PRIO(pmsg->id), EzCANDeviceID, FETCH_PROP(pmsg->id)), 
					&data, 
					sizeof(U32Params),
					pmsg->ch);
	}

	return OP_COMPLETE;
}

/*
*********************************************************************************************************
*                                         EzCAN 伺服特殊处理函数实现
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANServoPerformanceStateHandler
* Description    : 处理闭环类型切换的函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
s16 EzCANGetSinVal(s16 angle)	//angle范围:-0x7FFF~0X7FFF
{
	u16 hindex;
	s16 hSin;

	hindex = (u16)(angle + 32768);
	hindex /= 64;

	switch (hindex & SIN_MASK)
	{
	case U0_90:
		hSin = SinCosTable[(u8)(hindex)];
	break;
	case U90_180:
		hSin = SinCosTable[(u8)(0xFF-(u8)(hindex))];
	break;
	case U180_270:
		hSin = -SinCosTable[(u8)(hindex)];
	break;
	case U270_360:
		hSin =  -SinCosTable[(u8)(0xFF-(u8)(hindex))];
	break;
	default:
	break;
	}

	return hSin;
}

u8 EzCANGeneratePerfoamance(EzCANPerformModeTyp mode)
{
	u16 i;
	EzCANDataTypeTyp data_typ;	//数据类型:0->s16; 1->s32;
	s16 p_per_period = 0;		//每个周期有多少个点
	s32 p_amp = 0;
	s16 p2p_int = 0;	
	s64 amp;

	switch(EzCANServoParams.Control.LoopMode)
	{
	case OPEN_LOOP:
	case TORQUE_LOOP:
		data_typ = DT_S16;	 
		amp = EzCANServoParams.Moto.TorquePerformAmplitude;
		if(mode != STEP)
		{
			p_per_period = EzCANServoParams.Moto.TorqueStoreSampleFreq / EzCANServoParams.Moto.TorquePerformFreq;
			if(p_per_period < 8 && p_per_period > PERFORM_BUFFER_NUM/sizeof(s16))
				return OP_ILLEGAL_VAL;
		}
	break;
	case SPEED_TORQUE_LOOP:
	case SPEED_LOOP:
		data_typ = DT_S32;
		amp = EzCANServoParams.Moto.SpeedPerformAmplitude;
		if(mode != STEP)
		{
			p_per_period = EzCANServoParams.Moto.SpeedStoreSampleFreq / EzCANServoParams.Moto.SpeedPerformFreq;
			if(p_per_period < 8 && p_per_period > PERFORM_BUFFER_NUM/sizeof(s32))
				return OP_ILLEGAL_VAL;
		}
	break;
	case POS_TORQUE_LOOP:
	case POS_SPEED_TORQUE_LOOP:
	case POS_LOOP:
	case POS_SPEED_LOOP:
		data_typ = DT_S32;
		amp = EzCANServoParams.Moto.PosPerformAmplitude;
		if(mode != STEP)
		{
			//位置环只有阶跃响应
			return OP_ILLEGAL_FUNC;
		}
	break;
	}

	switch(mode)
	{
	case STEP:
		if(data_typ == DT_S16)
		{
			for(i=0; i<PERFORM_BUFFER_NUM/sizeof(s16); i++)
			{	
				*((s16*)&EzCANPerformanceBuffer[i*sizeof(s16)]) = (s16)amp;
			}
		}
		else if(data_typ == DT_S32)
		{
			for(i=0; i<PERFORM_BUFFER_NUM/sizeof(s32); i++)
			{
				*((s32*)&EzCANPerformanceBuffer[i*sizeof(s32)]) = (s32)amp;
			}
		}
	break;
	case TRIANGLE:
		//	 
	break;
	case SQUARE:
	break;
	case SIN:
		if(data_typ == DT_S16)
		{
			//计算每个点之间的间隔
			p2p_int = 0x10000 / p_per_period;

			for(i=0; i<PERFORM_BUFFER_NUM/sizeof(s16); i++)
			{
				p_amp = amp * ( 0x10000 +  EzCANGetSinVal(p2p_int*(i%p_per_period)) )/(2*0x10000);
				*((s16*)&EzCANPerformanceBuffer[i*sizeof(s16)]) = (s16)p_amp;
			}
		}
		else if(data_typ == DT_S32)
		{
			//计算每个点之间的间隔
			p2p_int = 0x10000 / p_per_period;

			for(i=0; i<PERFORM_BUFFER_NUM/sizeof(s32); i++)
			{
				p_amp = amp * ( (s64)0x10000 +  (s32)EzCANGetSinVal(p2p_int*(i%p_per_period)) )/(2*(s32)0x10000);
				*((s32*)&EzCANPerformanceBuffer[i*sizeof(s32)]) = p_amp;
			}
		}
	break;
	default:
		memset(EzCANPerformanceBuffer, 0, sizeof(EzCANPerformanceBuffer));
	break;
	}

	EzCANServoParams.Control.PerformanceState = mode;

	return OP_COMPLETE;
}

u8 EzCANServoPerformanceStateHandler(CAN_msg *pmsg, const struct ID_ITEM *item)
{
	u32 data;

	if(pmsg->type == DATA_FRAME)
	{
		data = *((u16*)pmsg->data);
		if(data > SIN)
			return OP_ILLEGAL_VAL;

		return EzCANGeneratePerfoamance((EzCANPerformModeTyp)data);
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

void EzCANPerformanceExamination(u8 *target, EzCANDataTypeTyp data_typ)
{
	static u16 exam_cnt = 0;

	if(EzCANServoParams.Control.PerformanceState != NO_PERF)
	{
		if(exam_cnt >= PERFORM_BUFFER_NUM)
		{
			EzCANServoParams.Control.PerformanceState = NO_PERF;
			return;
		}

		switch(data_typ)
		{
		case DT_S16:
			*((s16*)target) = *((s16*)&EzCANPerformanceBuffer[exam_cnt]);
			exam_cnt += sizeof(s16);
		break;
		case DT_S32:
			*((s32*)target) = *((s32*)&EzCANPerformanceBuffer[exam_cnt]);	
			exam_cnt += sizeof(s32);
		break;
		case DT_S64:		//在这里，用不完64位数据，只用32位代替
			*((s64*)target) = *((s32*)&EzCANPerformanceBuffer[exam_cnt]);	
			exam_cnt += sizeof(s32);
		break;
		}
	}
	else
	{
		exam_cnt = 0;		
	}
}

