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
* Filename		: hx_CANDrv.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

#ifndef _CAN_H_
#define _CAN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Private define ------------------------------------------------------------*/
#define CAN_PORT_CNT							3
#define CAN_RX_BUF_NUM							(10)
#define CAN_TX_BUF_NUM							(1)
#define CAN_TX_ERR_CNT							3

/* Private typedef -----------------------------------------------------------*/
typedef void (*FnCANDrv_ISR)(uint8_t node, unsigned char *buf);

typedef struct tagCAN_BUF{
	FDCAN_RxHeaderTypeDef rxHeader;
	uint8_t rxbuf[CAN_RX_BUF_NUM][8];
	uint8_t head;
	uint8_t tail;
	FDCAN_TxHeaderTypeDef txHeader;
	uint8_t txbuf[8];
	uint32_t txMailBox;
	int fifo0TxNum;
	uint32_t runtime; //pjg++190613
}CAN_BUF;

typedef struct tagCAN_DRV{
	struct {
		uint8_t head;
		uint8_t tail;
		uint8_t *pbuf[CAN_RX_BUF_NUM];
		uint32_t id;
		uint8_t status;
	}rx;
	struct {
		uint8_t buf[CAN_TX_BUF_NUM][8];	// 2 is 12bit  
		//uint16_t *ptr[CAN_TX_BUF_NUM+1];
		uint8_t cnt;
		uint32_t id;
		uint8_t status;
		uint8_t bus;
	}tx;
	uint32_t run_time;
	uint8_t commErr;
}CCAN_DRV;

typedef struct tagCAN_PORT{
	uint32_t runTime;
}CAN_PORT;

enum _eCAN_BUF_CONTANTS {
	CBC_FT1,
	CBC_FT2,
	CBC_WRIST,
	CBC_MAX
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern CCAN_DRV CanDrv[CBC_MAX];
extern CAN_BUF CanBuf[2];
//extern CAN_PORT CanPort[CAN_PORT_CNT];
extern FDCAN_TxHeaderTypeDef TxHeader[2];

/* Private function prototypes -----------------------------------------------*/
extern void CCANDrv_CANDrv(void);
extern void CCANDrv_InitHW(uint8_t flag);
extern void CCANDrv_SetFilter16bit(FDCAN_HandleTypeDef *phcan, uint32_t filter);
extern int CCANDrv_WriteFile(uint8_t node, uint8_t *buf, uint8_t cnt);
extern uint8_t *CCANDrv_ReadFile(uint8_t node, uint8_t cnt);
//extern int CCANDrv_ReadFile(unsigned int buf_num, uint8_t *buf, uint8_t cnt);
extern void CCANDrv_SetTxStdID(FDCAN_TxHeaderTypeDef *pheader, uint32_t id);
extern void CCANDrv_SetIDInsol(uint8_t node, uint16_t txBase, uint16_t rxBase);
extern void CCANDrv_SetID(uint8_t node, uint16_t txBase, uint16_t rxBase);
extern void  CCANDrv_ISR(FDCAN_HandleTypeDef *phcan);
extern void CCANDrv_SetISRFn( void (*fp)(uint8_t node, unsigned char *buf));

#ifdef __cplusplus
}
#endif

#endif //_CAN_H_
