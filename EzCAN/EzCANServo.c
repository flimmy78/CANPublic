/******************** (C) COPYRIGHT 2011 Robotell ********************
* File Name          : EzCANServo.c
* Author             : ���
* Date First Issued  : 2011.08.05
* Description        : EzCAN�ŷ���Э�鲿��Э��ʵ��
********************************************************************************/
#include "EzCAN.h"
#include "Consts.h"
#include <string.h>
#include <math.h>

/*
*********************************************************************************************************
*                                         EzCAN �������⴦����
*********************************************************************************************************
*/
//==================-----�����������⴦����-----=================//
u8 EzCANCommonSaveToFlashHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem);
u8 EzCANCommonRestoreHandler(CAN_msg *pmsg, const struct ID_ITEM *pitem);
u8 EzCANCommonBoostHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonStorageSampleHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonResetSystemHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonSetDevIDHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommonIdentifyHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANCommomStorageSwitchHandler(CAN_msg *pmsg, const IDItemTyp *item);

//==================-----�ŷ��������⴦����-----=================//
u8 EzCANCommonErrorHistoryHandler(CAN_msg *pmsg, const IDItemTyp *item);
u8 EzCANServoPerformanceStateHandler(CAN_msg *msg, const struct ID_ITEM *pitem);
#include "EzCANServoSpecialHandler.h"

/*
*********************************************************************************************************
*                                         EzCAN ���ݷ�Χ����
*********************************************************************************************************
*/
//==================--------EzCAN�������Է�Χ-------================//
#define EZCAN_COMMON_COMMON_RANGE(Typ, Name, Range) Typ Name = Range;
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_RANGE
										 
//==================--------EzCAN�ŷ����Է�Χ-------================//
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
*                                         EzCAN �������Ա�������
*********************************************************************************************************
*/
EzCANStorageSampleChannelTyp EzCANStorageChannel;
/*static */unsigned char StorageData[STORAGE_CH_NUM][STORAGE_BUF_LEN];
/*static */EzCANStorageSampleDataTyp EzCANStorageSampleData[STORAGE_CH_NUM];

//==================----4.ͬ���ź�Դ�����б���----==================//
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

//==================----3.ͬ���ź�Դ����----======================//
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
*                                         EzCAN �ŷ������б�
*********************************************************************************************************
*/
EzCANServoTyp EzCANServoParams;
u8 EzCANPerformanceBuffer[PERFORM_BUFFER_NUM];		//��������s16���ͣ��ٶȻ���s32���ͣ�λ�û�Ҳ��s32����
u8 EzCANDeviceID;

const IDItemTyp EzCANServoItems[] = 
{
//===============---------�������Զ�---------==============//
#define EZCAN_COMMON_COMMON_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Common.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_START_TOK

#define EZCAN_COMMON_COMMON_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Common.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_TOK

//===============---------����BOOST���Զ�---------==============//
#define EZCAN_COMMON_BOOST_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Boost.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_START_TOK

#define EZCAN_COMMON_BOOST_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Boost.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_BOOST_TOK

//===============---------�����洢�������Զ�---------==============//
#define EZCAN_COMMON_STORAGESAMPLE_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.StorageSample.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_START_TOK

#define EZCAN_COMMON_STORAGESAMPLE_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.StorageSample.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_STORAGESAMPLE_TOK


//===============---------�ŷ����������---------==============//
#define EZCAN_SERVO_MOTO_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Moto.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_START_TOK

#define EZCAN_SERVO_MOTO_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Moto.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_TOK

//===============---------�ŷ�����������---------==============//
#define EZCAN_SERVO_OTHERS_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Others.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_START_TOK

#define EZCAN_SERVO_OTHERS_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Others.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_TOK

//===============---------�ŷ����ƶ�---------==============//
#define EZCAN_SERVO_CONTROL_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Control.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_START_TOK

#define EZCAN_SERVO_CONTROL_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Control.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_TOK

//===============---------�ŷ�״̬��---------==============//
#define EZCAN_SERVO_STATUS_START_TOK(ID, BEGIN, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Status.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_START_TOK

#define EZCAN_SERVO_STATUS_TOK(ID, TYPE, NAME, RW, RANGE, HANDLER, ASCII) {ID, (void*)&EzCANServoParams.Status.##NAME, RW, RANGE, HANDLER, #ASCII},
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_TOK
};

/*
*********************************************************************************************************
*                                         EzCAN �����̵߳��ýӿ�
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANMessageHandle
* Description    : �ŷ�ϵͳ��Ϣ��������
* Input          : msg: CAN���ݰ�
                   index: ��Ϣ����
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
		if( (msg->type == DATA_FRAME && EzCANServoParams.Common.EnableACK)	//���������֡������ʹ����ACK
			|| (msg->type == REMOTE_FRAME && err == OP_ILLEGAL_FUNC) )	//���������ֻ������
		{ 
			//����ACK
			err_info.Param1 = msg->id; 
			err_info.Param2 = err; 		//���嶨��ο�EzCAN�ĵ�
			EzCANSend(BUILD_ID(FETCH_PRIO(msg->id), EzCANDeviceID, COMMON_COMMON_ACK), 
					&err_info, 
					sizeof(U32Params),
					msg->ch);
		}
	}
}

/*******************************************************************************
* Function Name  : EzCANServoProcess
* Description    : �ŷ�ϵͳ��Ϣ����
* Input          : msg: CAN���ݰ�
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoProcess(CAN_msg *msg)
{
	u16 prop = msg->id;
	u16 index = 0;
	u16 total_num = 0;
	U32Params err_info;

	//==================---------�������Զ�---------=================//
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

	//==================---------�ŷ����Զ�---------=================//
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

	//��������ڵ�ַ��Χ������ACK����
	err_info.Param1 = msg->id; 
	err_info.Param2 = OP_ILLEGAL_ADDR; 		//���嶨��ο�EzCAN�ĵ�
	EzCANSend(BUILD_ID(FETCH_PRIO(msg->id), EzCANDeviceID, COMMON_COMMON_ACK), 
		&err_info, 
		sizeof(U32Params),
		msg->ch);
}

/*******************************************************************************
* Function Name  : EzCANBoostProcess
* Description    : Boost���ݴ�����
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
					//BOOST������Ϣ���ȼ�ֻ����0x0F��������ȼ�
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
* Description    : �洢�������ݴ�����
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
					0);		//MODBUS��֧�ִ洢�����������㴫һ��������

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

		//1.�����ĳ��ͨ������˼��������ѯ����ͬͬ���źŵ�����ͨ�����ȴ�����ͨ����ɲ���
		//2.����ͨ��ȫ����ɲ����󣬿���ͬ���ź�
		for(i=0; i<STORAGE_CH_NUM; i++)
		{
			if(EzCANStorageSampleData[i].Enable
				&& EzCANStorageSampleData[i].Ready
				&& ((ch_comp_flag>>i) & 0x0001 == 1))
			{
				ch_comp_flag &= ~(1<<i);		//����ͨ��������ɱ�־
				pStrSyn = EzCANStorageSampleData[i].StorageSyn;
			
				EzCANStorageSampleSendReadyData(i);	//���͵�ǰ�����ͨ��������
					
				if(pStrSyn != NULL)
				{
					syn_comp_flag = pStrSyn->Channels;
					syn_comp_flag &= ~(1<<i);	//�嵱��ͬ���ź�ͨ��λ

					//�������������ͬ��ͨ��
					if(syn_comp_flag > 0)
					{
						//����jѭ���Ĵ���������
						for(j=0; j<sizeof(u16)*8; j++)
						{
							ch_temp = syn_comp_flag>>j;

							//�����ǰͨ����iһ���ͬ��ͨ��
							if(ch_temp & 0x0001 == 1)
							{
								while(!(EzCANStorageSampleData[j].Enable
										&& EzCANStorageSampleData[j].Ready))
								{
									OSTimeDly(5);
								}
	
								//���͵�ǰ�����ͨ��������
								ch_comp_flag &= ~(1<<j);
								EzCANStorageSampleSendReadyData(j);
		
								//������������е�ͬ��λ��������ͬ����ѯ								
								syn_comp_flag &= ~(1<<j);
								if(syn_comp_flag == 0)
								{
									break;
								}
							}
						}
					}

					//1.�ȹ���ͬ����־
					pStrSyn->Flag = 0;
					
					//2.��ÿ��ͬ��ͨ����Readyλ
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

					//3.����ͬ����־
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
*                                         EzCAN �������ýӿ�
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANServoParamsSaveToFlash
* Description    : �������浽Flash
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

	//����FLASH	  		
	FLASH_Unlock();	
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
	for(i = 0; i < 3; i++)
	{
		if(FLASH_ErasePage(SERVO_START_PAGE + i * 0x400) != FLASH_COMPLETE)
		{
			return;
		}
	} 

	//д�빫������--DevID
	pdata = (u32 *)&EzCANServoParams.Common.DevID;
	start_page = SERVO_START_PAGE;
	plus_elem_size = (sizeof(EzCANServoParams.Common.DevID) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}
	
	//д�빫������--DevPriority
	pdata = (u32 *)&EzCANServoParams.Common.DevPriority;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Common.DevPriority) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}

	//д�빫������--MasterID
	pdata = (u32 *)&EzCANServoParams.Common.MasterID;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Common.MasterID) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE)
			return ;
	}

	//д��Moto����
	pdata = (u32 *)&EzCANServoParams.Moto;
	start_page += plus_elem_size * sizeof(u32);
	plus_elem_size = (sizeof(EzCANServoParams.Moto) + (sizeof(u32) - 1)) / sizeof(u32);
	for(i = 0 ; i < plus_elem_size; i ++)
	{
		if(FLASH_ProgramWord(start_page + i * sizeof(u32), *pdata ++) != FLASH_COMPLETE) 
			return;
	}

	//д����������
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
* Description    : �����ָ���������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANSaveErrorHistoryToFlash(void)
{
	u32 *pdata;
	u8 i;

	NVIC_SETPRIMASK();

	//����FLASH	  		
	FLASH_Unlock();	
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR); 
	FLASH_ErasePage(ERROR_HISTORY_START_PAGE);

	//д����������
	pdata = (u32 *)&EzCANServoParams.Common.ErrorHistory;
	for(i = 0 ; i < sizeof(EzCANServoParams.Common.ErrorHistory)/4 ; i ++)
		FLASH_ProgramWord(ERROR_HISTORY_START_PAGE + i * 4, *pdata ++);

	FLASH_Lock();
	
	NVIC_RESETPRIMASK(); 
}

/*******************************************************************************
* Function Name  : EzCANServoParamsRestore
* Description    : �����ָ���������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANServoParamsRestore(u8 binit)
{
	u16 i;

//===============---------�ɱ���γ�ʼ��Ϊ0--------============//
	memset((void*)&EzCANServoParams.Common, 0, sizeof(EzCANServoParams.Common));
	memset((void*)&EzCANServoParams.Moto, 0, sizeof(EzCANServoParams.Moto));
	memset((void*)&EzCANServoParams.Others, 0, sizeof(EzCANServoParams.Others));
	memset((void*)&EzCANServoParams.Status, 0, sizeof(EzCANServoParams.Others));
	memset((void*)&EzCANServoParams.Control, 0, sizeof(EzCANServoParams.Others));

//===============---------�������Զ�����---------==============//
#define EZCAN_COMMON_COMMON_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Common.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Common.protocol"
#undef EZCAN_COMMON_COMMON_INIT

	//��ʼ���洢����������
	for(i=0; i<STORAGE_CH_NUM; i++)
	{
		EzCANStorageSampleData[i].Buffer = StorageData[i];
		EzCANStorageSampleData[i].BufferLength = 0;
	}

	//�豸ID���ܱ�
	EzCANServoParams.Common.DevID = EzCANDeviceID;

//===============---------�ŷ����������---------==============//
#define EZCAN_SERVO_MOTO_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Moto.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_MOTO_INIT

//===============---------�ŷ�����������---------==============//
#define EZCAN_SERVO_OTHERS_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Others.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_OTHERS_INIT

//������ǳ�ʼ��������Ҫ�ָ���һ��
if(binit)
{
//===============---------�ŷ����ƶ�---------==============//
#define EZCAN_SERVO_CONTROL_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Control.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_CONTROL_INIT

//===============---------�ŷ�״̬��---------==============//
#define EZCAN_SERVO_STATUS_INIT(NAME, TYPE, INITVALUE) EzCANServoParams.Status.##NAME = TYPE##Init(INITVALUE);
#include "EzCAN_Servo.protocol"
#undef EZCAN_SERVO_STATUS_INIT	
}

}

/*******************************************************************************
* Function Name  : EzCANServoParamsInit
* Description    : ������ʼ��
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
* Description    : �洢�����ӿں���
* Input          : prop: ����ֵ����
                   data: ����ֵ
				   typelen: ���ͳ��� 
* Output         : None
* Return         : None
*******************************************************************************/
void EzCANStorageSample(u32 prop, void* data, u8 typelen)
{
	u8 b_syn = 0;
	u8 index, i;

	if(EzCANServoParams.Common.StorageSwitch)
	{
		//�ҵ�prop��EzCANStorageChannel�����������
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

	 	//�ж�ͬ�������Ƿ�Ҫ�����ܲ��Ե�ʱ��ر�
		if(EzCANServoParams.Control.StrSampleSyncOnPerformSwitch == 1
			&& EzCANServoParams.Control.PerformanceState == NO_PERF)
		{
			//������ȵ���0��������Ѿ���ɷ��ͻ��߻�û��ʼ���ܲ��ԣ�ֱ�ӷ���
			if(EzCANStorageSampleData[i].BufferLength == 0)
			{
				return;
			}
		}

		//���õ�ǰ��������BufferLength����Ϊ0����ͷ��ʼ��
		if(EzCANServoParams.Control.PerformanceState != NO_PERF
			&& EzCANStorageSampleData[i].AlreadyReset == 0)
		{
			EzCANStorageSampleData[i].BufferLength = 0;
			EzCANStorageSampleData[i].Ready = 0;
			EzCANStorageSampleData[i].AlreadyReset = 1;
		}

		//�жϴ洢����ͬ���ź�
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

		//��������
		if(EzCANStorageSampleData[i].Enable == 1			//�豸ʹ��
			&& EzCANStorageSampleData[i].Ready == 0			//�洢������δ��ɣ���Ҫ��������
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
*                                         EzCAN �����������⴦����ʵ��
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

		//����Ƿ����ظ����õ�ͨ��������У���رո�ͨ��
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

		//�޸�EzCANStorageSampleChannelԭ������index������
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

		//�����ͨ����ǰ�Ѿ���ʹ�ã�����ɾ����ͨ����ͬ���ź����������
		if(EzCANStorageSampleData[index].StorageSyn != NULL)
		{
			EzCANStorageSampleData[index].StorageSyn->Channels &= ~(1<<index);
			EzCANStorageSampleData[index].StorageSyn = NULL;
		}

		//�洢��EzCANStorageSampleChannel����ȥ
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
					//д��ͨ����
					pStrSyn->Channels |= (1<<index);
					goto start_sample;
				}
			}
		}

		//��ͨ����������
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
					//����Ѿ��������ͬ��ͨ�����������������ͨ��
					ch_temp = ch_comp_flag>>i;
					if(ch_temp & 0x0001 == 0)
						continue;

					//1.����ͬ����־
					pStrSyn->Flag = 0;
					
					//2.��ÿ��ͬ��ͨ����Readyλ
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

					//3.����ͬ����־
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
		//��ȡCPUID
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
*                                         EzCAN �ŷ����⴦����ʵ��
*********************************************************************************************************
*/
/*******************************************************************************
* Function Name  : EzCANServoPerformanceStateHandler
* Description    : ����ջ������л��ĺ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
s16 EzCANGetSinVal(s16 angle)	//angle��Χ:-0x7FFF~0X7FFF
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
	EzCANDataTypeTyp data_typ;	//��������:0->s16; 1->s32;
	s16 p_per_period = 0;		//ÿ�������ж��ٸ���
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
			//λ�û�ֻ�н�Ծ��Ӧ
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
			//����ÿ����֮��ļ��
			p2p_int = 0x10000 / p_per_period;

			for(i=0; i<PERFORM_BUFFER_NUM/sizeof(s16); i++)
			{
				p_amp = amp * ( 0x10000 +  EzCANGetSinVal(p2p_int*(i%p_per_period)) )/(2*0x10000);
				*((s16*)&EzCANPerformanceBuffer[i*sizeof(s16)]) = (s16)p_amp;
			}
		}
		else if(data_typ == DT_S32)
		{
			//����ÿ����֮��ļ��
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
		case DT_S64:		//������ò���64λ���ݣ�ֻ��32λ����
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

