#ifndef __EZCAN_PROCESS_H
#define __EZCAN_PROCESS_H

#include <stm32f10x_lib.h>
#include "uCosII_CAN.h"

bool EzCANEventInit(void);
bool PostToEzCANProcess(CAN_msg *msg);
void ResetSystem(void);
bool EzCANSendBuffer(u32 id, void *buf, u8 bufsize, u32 tag);
void PostNowSystemStatusToMaster(void);

#endif

