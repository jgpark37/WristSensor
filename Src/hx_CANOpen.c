/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems¡¯s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_API.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Message box
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"
#include "hx_CANOpen.h"
#include "CAN_OPEN_DSP402.h"
#include "hx_WristSensor.h"
#include "hx_CANDrv.h"
#include "hx_mcuFlash.h"
#include "CanOpenDef.h"


/* Private define ------------------------------------------------------------*/

//=========================================================

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern char CommonBuf[FLASH_PAGE_SIZE1*2];

/* Private function prototypes -----------------------------------------------*/
//extern static void MX_CAN_Init(void);


////////////////////////////////////////////////////////////////////////////////////////
// CAN OPEN
uint16_t CANOpen_GetIndex(uint16_t index , uint8_t subindex)
{
	return index;
}

void CANOpen_Control(uint8_t node, CanOpenCommand *pCoc)
{
	uint8_t buf[5];
	uint8_t cnt;
	float ftemp;

	cnt = 0;

	CCANDrv_SetTxStdID(&CanBuf[1].txHeader, Wrist.can.txid);
	
	switch (pCoc->m_subinx) {
	//case CAN_OPEN_CONTROL_SHUTDOWN:
	//	break;
	case CAN_OPEN_SWITCH_ON:
		if (pCoc->data[0] == 1) {
		}
		else {
		}
		break;
	case CAN_OPEN_DISABLE_OPERATION:
		CWrist_SetRunMode(RM_STANDBY);
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_DISABLE_OPERATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_ENABLE_OPERATION:
		CWrist_SetRunMode(RM_NORMAL);
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_ENABLE_OPERATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_FAULT_RESET:
		if (pCoc->data[0] == 1) {
		}
		else {
		}
		break;
	case CAN_OPEN_SET_INTERVAL_SEND_DATA:
		Wrist.data.sendTime = (pCoc->data[1]<<8) | pCoc->data[0];
		if (Wrist.data.sendTime < 1) Wrist.data.sendTime = 5;
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_INTERVAL_SEND_DATA;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_RUN_CALIBRATION:
		CWrist_RunMode_Calibration();
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_RUN_CALIBRATION;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_SET_DEVICE_NUM:
		Wrist.can.txid = CAN_TX_STD_ID + pCoc->data[0];
		Wrist.can.rxid = CAN_RX_STD_ID + pCoc->data[0];
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DEVICE_NUM;
		if (CWrist_SaveParamToFlash(CommonBuf)) buf[cnt++] = 0;
		else buf[cnt++] = 1;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_SET_DATA_OUTPUT_TYPE:
		Wrist.data.outputType = pCoc->data[0];
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DATA_OUTPUT_TYPE;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_SET_DATA_SIZE:
		//Wrist.data.size = pCoc->data[0];
		//Wrist.data.shift = pCoc->data[1];
		if (Wrist.runMode > RM_SLEEP && Wrist.runMode < RM_MAX) {
			CWrist_SetRunMode(RM_NORMAL);
		}
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DATA_SIZE;
		buf[cnt++] = 0;
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	case CAN_OPEN_GET_INFORMATION:
		buf[cnt++] = FW_VER;
		buf[cnt++] = HW_VER;
		buf[cnt++] = FW_USER;
		buf[cnt++] = WMN_SELF;
		buf[cnt++] = FW_CREATE_YEAR;
		buf[cnt++] = FW_CREATE_MONTH;
		buf[cnt++] = FW_CREATE_DAY;
		buf[cnt++] = FW_CREATE_HOUR;
		//buf[cnt++] = FW_CREATE_MIN;
		//cnt = 8;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	default:
		break;
	}
}

void CANOpen_Status(uint8_t node, CanOpenCommand *pCoc)
{
	uint8_t buf[8];
	uint8_t cnt;

	CCANDrv_SetTxStdID(&CanBuf[1].txHeader, Wrist.can.txid);
 	cnt = 0;
	switch (pCoc->m_subinx) {
	case CAN_OPEN_GET_STATUS:
		break;
	default:
		break;
	}
	
}

//void CANOpen_Ack(uint8_t node, CanOpenCommand *pCoc)
//{
	
//}

void CANOpen_Etc(uint8_t node, CanOpenCommand *pCoc)
{
	uint8_t buf[5];
	uint8_t cnt;
	float ftemp;

	cnt = 0;

	CCANDrv_SetTxStdID(&CanBuf[1].txHeader, Wrist.can.txid);
	
	switch (pCoc->m_inx) {
	case CIA_402_POSITION_OFFSET:
		buf[cnt++] = SDO_UPLOAD_RESPONSE|SDO_SIZE_BYTE;
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)&0x00FF);
		buf[cnt++] = (uint8_t)(CANOpen_GetIndex(CAN_OPEN_CONTROL)>>8);
		buf[cnt++] = (uint8_t)CAN_OPEN_SET_DATA_OUTPUT_TYPE;
		if (pCoc->m_subinx < 2) {
			if (*Wrist.Enc[pCoc->m_subinx].pValue > 1024) Wrist.Enc[pCoc->m_subinx].value = 1024;
			else Wrist.Enc[pCoc->m_subinx].value = *Wrist.Enc[pCoc->m_subinx].pValue;
			ftemp = ((pCoc->data[1]<<8)+pCoc->data[0])/100.0;
			Wrist.Enc[pCoc->m_subinx].offset = (((float)(Wrist.Enc[pCoc->m_subinx].value))*0.35+1.25) - ftemp;			
			buf[cnt++] = 0;
		}
		else {
			buf[cnt++] = 1;
		}
		//cnt = 5;
		CCANDrv_WriteFile(CBC_WRIST, buf, cnt+1);
		break;
	default:
		break;
	}
}

void CANOpen_ProcessSdo(uint8_t node, unsigned char *buf)
{
	CanOpenCommand *pCoc;
	//int i;

	if (node < CBC_WRIST) {
		if (node == CBC_FT1) {
			Wrist.FTSensor.Fx.upper = buf[1];
			Wrist.FTSensor.Fx.lower = buf[2];
			Wrist.FTSensor.Fy.upper = buf[3];
			Wrist.FTSensor.Fy.lower = buf[4];
			Wrist.FTSensor.Fz.upper = buf[5];
			Wrist.FTSensor.Fz.lower = buf[6];
			Wrist.FTSensor.Tx.upper = buf[7];
		}
		else if (node == CBC_FT2) {
			Wrist.FTSensor.Tx.lower = buf[0];
			Wrist.FTSensor.Ty.upper = buf[1];
			Wrist.FTSensor.Ty.lower = buf[2];
			Wrist.FTSensor.Tz.upper = buf[3];
			Wrist.FTSensor.Tz.lower = buf[4];
			Wrist.FTSensor.statusOfOverload = buf[5];
		}
	}
	else {
		//if (CanDrv[i].rx.head == CanDrv[i].rx.tail) continue;
		
		pCoc = (CanOpenCommand *)buf;//&CanDrv[i].rx.pbuf[CanDrv[i].rx.tail];
		if (pCoc->m_inx == CANOpen_GetIndex(CAN_OPEN_CONTROL)) {
			CANOpen_Control(node, pCoc);
		}
		else if (pCoc->m_inx == CANOpen_GetIndex(CAN_OPEN_STATUS)) {
			CANOpen_Status(node, pCoc);
		}
		else {
			//CANOpen_Ack(node, pCoc);
			CANOpen_Etc(node, pCoc);
		}
		//CanDrv[i].rx.tail++;
		//if (CanDrv[i].rx.tail >= CAN_RX_BUF_NUM) 
		//	CanDrv[i].rx.tail = 0;
	}

}


