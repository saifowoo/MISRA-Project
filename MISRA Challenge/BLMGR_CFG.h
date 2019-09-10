﻿/*
 * BLMGR_CFG.h
 *
 * Created: 28/02/2016 06:54:32 م
 *  Author: hossam
 */ 


#ifndef BLMGR_CFG_H_
#define BLMGR_CFG_H_

#include "Basic_Types.h"
typedef struct  
{
	u8 Portname;
	u8 Port_Mask;
}BLMGR_DioPinConfig;

#define BLOUETOOTH_ON 0xffu
#define BUZEER_ON     0xffu
#define BUZEER_OFF    0x00u

 extern BLMGR_DioPinConfig BuzzerConfig;
 extern BLMGR_DioPinConfig BlueToothPwrConfig;
 extern BLMGR_DioPinConfig BluetoothKeyConfig;
 

#endif /* BLMGR_CFG_H_ */
