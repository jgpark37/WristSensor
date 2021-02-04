/*
*******************************************************************************
*
*                              Hexar Systems, Inc.
*                      104, Factory Experiment Bldg, No41.55
*              Hanyangdaehak-ro, Sangnok-gu, Ansan, Gyeonggi-do, Korea
*
*                (c) Copyright 2017, Hexar Systems, Inc., Sangnok-gu, Ansan
*
* All rights reserved. Hexar Systems’s source code is an unpublished work and the
* use of a copyright notice does not imply otherwise. This source code contains
* confidential, trade secret material of Hexar Systems, Inc. Any attempt or participation
* in deciphering, decoding, reverse engineering or in any way altering the 
source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_CAN.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/05/18
* Description		: CAN Driver
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "hx_CANDrv.h"
#include "Utilities.h"
//#include "cmsis_os.h"
//#include "DLib_Product_string.h"

/* Private define ------------------------------------------------------------*/
#define CANDRV_RX_BUF_CNT					20

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//static CAN_FilterConfTypeDef myFilter[CAN_PORT_CNT]; 	//motor
//FDCAN_TxHeaderTypeDef TxHeader[2];
const uint8_t CAN_BUS_NUM[5] = {0, 0, 1, 0, 0};
CCAN_DRV CanDrv[CBC_MAX];
CAN_BUF CanBuf[2];
//long	can_sid1;	//모터1로 보내는 SID
//long	can_sid2;	//모터2로 보내는 SID
//volatile unsigned char	c1_index=0, c1_rindex=0, c1_rcv_status=0;
//CAN_PORT CanPort[CAN_PORT_CNT];
FnCANDrv_ISR fpCANDrv_ISR;

//extern osMutexId myMutex01Handle;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* Private function prototypes -----------------------------------------------*/
void CCANDrv_CAN1Init(void);
void CCANDrv_CAN2Init(void);
void CCANDrv_CAN3Init(void);
//extern void Error_Handler(void);
extern void Error_Handler(char * file, int line);
extern void CAN_Init(FDCAN_HandleTypeDef *phcan);


void CCANDrv_SetTxStdID(FDCAN_TxHeaderTypeDef *pheader, uint32_t id)
{
	pheader->Identifier = id;
}

#if 0
int CCANDrv_Status(FDCAN_HandleTypeDef *phcan)
{
	HAL_CAN_StateTypeDef status;

	//	if (cnt > 4) return;
	status = HAL_CAN_GetState(phcan);

	if (phcan->Instance == FDCAN1) {
		switch (status) {
		case HAL_CAN_STATE_READY:		// normal
			return 0;
			break;
		//case HAL_CAN_STATE_BUSY_RX:
		//case HAL_CAN_STATE_BUSY_TX:	//host no run
		//case HAL_CAN_STATE_BUSY_TX_RX: // can connector plug out
		//	return -1;
		case HAL_CAN_STATE_RESET:
			return 0;
		case HAL_CAN_STATE_ERROR:
			return -1;
		//case HAL_CAN_STATE_TIMEOUT:
			//Insol.can.timeout++;
			//if (Insol.can.timeout > 100) {
			//	Insol.can.timeout = 0;
			//	return -1;
			//}
			//return 0;
		default:
			return 0;
		}	
	}
	//else if (phcan->Instance == CAN2) {
	//}
	
	return 0;
}
#endif

void CCANDrv_ISR(FDCAN_HandleTypeDef *phcan)
{
  	int buf_pos;
	//int can_bus;

	//if (phcan->Instance == FDCAN1) can_bus = 0;
	//else if (phcan->Instance == FDCAN2) can_bus = 1;
	//else return;
	
	/* Retreive Rx messages from RX FIFO0 */
	HAL_FDCAN_GetRxMessage(phcan, FDCAN_RX_FIFO0, &CanBuf[0].rxHeader, CanBuf[0].rxbuf[0]);
		
	if (CanBuf[0].rxHeader.IdType == FDCAN_STANDARD_ID) {
		switch (CanBuf[0].rxHeader.Identifier) {
			case SID_T_WRIST:
				buf_pos = CBC_WRIST;
				break;
			case SID_T_FT1:
				buf_pos = CBC_FT1;
				break;
			case SID_T_FT2:
				buf_pos = CBC_FT2;
				break;
			default:
				buf_pos = CBC_MAX;
				break;
		}
		
		if  (buf_pos != CBC_MAX) {
			if (CanDrv[buf_pos].rx.id == CanBuf[0].rxHeader.Identifier) {
				CanDrv[buf_pos].rx.pbuf[CanDrv[buf_pos].rx.head] = CanBuf[0].rxbuf[0];
				fpCANDrv_ISR(buf_pos-CBC_FT1, CanDrv[buf_pos].rx.pbuf[CanDrv[buf_pos].rx.head]);
				CanDrv[buf_pos].rx.head++;
				if (CanDrv[buf_pos].rx.head >= CAN_RX_BUF_NUM) 
					CanDrv[buf_pos].rx.head = 0;
			}
			
		}
	}
	//CanBuf[can_bus].head++;
	//if (CanBuf[can_bus].head >= CAN_RX_BUF_NUM) CanBuf[can_bus].head = 0;
	//Wrist.led.canCnt++;
}

int CCANDrv_WriteFile(uint8_t node, uint8_t *buf, uint8_t cnt)
{
	//int i;
	FDCAN_HandleTypeDef *phcan;
	HAL_StatusTypeDef status;
	FDCAN_TxHeaderTypeDef *ptxHeader;
	//int can_bus;

	if (node < CBC_WRIST) {
		phcan = &hfdcan1;
		ptxHeader = &CanBuf[0].txHeader;
	}
	else {
		phcan = &hfdcan2;
		ptxHeader = &CanBuf[1].txHeader;
	}
	//if (CCANDrv_Status(phcan)) return 1;
	
	//osMutexWait(myMutex01Handle,1);
	
	/* 전송할 메시지의 프레임 유형을 지정합니다
      #define CAN_RTR_DATA    (0x00000000U)  Data frame
      #define CAN_RTR_REMOTE  (0x00000002U)  Remote frame */
	//CanBuf.txHeader.RTR = CAN_RTR_DATA;
	ptxHeader->DataLength =(uint32_t)(cnt<<16);
	/*##-3- Start the Transmission process ###############################*/
	//CanBuf[0].txMailBox = HAL_CAN_GetTxMailboxesFreeLevel(phcan);
	//status = HAL_CAN_AddTxMessage(phcan, &CanBuf[0].txHeader, CanBuf[0].txbuf, &CanBuf[0].txMailBox);
	status = HAL_FDCAN_AddMessageToTxFifoQ(phcan, ptxHeader, buf);//CanBuf[0].txbuf);
	if (status != HAL_OK) { // HAL_OK 가 아니면?
		CAN_Init(phcan);
		if (phcan->Instance == FDCAN1) {
			CCANDrv_CAN1Init();
		}
		else if (phcan->Instance == FDCAN2) {
			CCANDrv_CAN2Init();
		}
		//osMutexRelease(myMutex01Handle);
		return 0;
	}
	//osMutexRelease(myMutex01Handle);
	return 1;
}

uint8_t *CCANDrv_ReadFile(uint8_t node, uint8_t cnt)
{
	uint8_t *buf;
	//if (!CanDrv[CBC_MOTOR1+buf_num].rx.status) return 0;
	//CanDrv[CBC_MOTOR1+buf_num].rx.id = 0;
	if (CanDrv[CBC_FT1+node].rx.head == CanDrv[CBC_FT1+node].rx.tail) 
		return 0;
	//timeout += 500;
	//memcpy(buf, CanDrv[CBC_MOTOR1+buf_num].rx.pbuf
	//							[CanDrv[CBC_MOTOR1+buf_num].rx.tail], cnt);
	buf = CanDrv[CBC_FT1+node].rx.pbuf[CanDrv[CBC_FT1+node].rx.tail];
	CanDrv[CBC_FT1+node].rx.tail++;
	if (CanDrv[CBC_FT1+node].rx.tail >= CAN_RX_BUF_NUM)
		CanDrv[CBC_FT1+node].rx.tail = 0;
	return buf;
}

uint32_t CCANDrv_FilterID_Generator_16Bits(uint32_t ID, uint32_t RTR, uint32_t IDE)
{
	uint32_t ret;

	if (IDE == FDCAN_STANDARD_ID)
	{
		 ret = (ID & 0x7FF) << 5 | (RTR << 3) | (IDE << 1);
	}
	else // IDE == CAN_ID_EXT
	{
		 ret = ((ID & 0x1FFC0000) >> 13) | (RTR << 3) | (IDE << 1) | ((ID & 0x38000) >> 15);
	}

	return ret;
}

void CCANDrv_FilterId_Generator_32Bits( FDCAN_FilterTypeDef* sFilterConfig,
                                   uint32_t ID_1, uint32_t RTR_1,
                                   uint32_t ID_2, uint32_t RTR_2 )
{

	//sFilterConfig->FilterIdHigh = (ID_1 & 0x1FFFE000) >> 13;
	//sFilterConfig->FilterIdLow = (((ID_1 & 0x1FFF) << 3) | RTR_1 | CAN_ID_EXT) & ~(0x1U);
	//sFilterConfig->FilterMaskIdHigh = (ID_2 & 0x1FFFE000) >> 13;
	//sFilterConfig->FilterMaskIdLow = (((ID_2 & 0x1FFF) << 3) | RTR_2 | CAN_ID_EXT) & ~(0x1U);
	sFilterConfig->FilterID1 = 0;//0x111;
	sFilterConfig->FilterID2 = 0;//0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
}

void CCANDrv_SetFilter16bit(FDCAN_HandleTypeDef *phcan, uint32_t filter)
{
	FDCAN_FilterTypeDef sFilterConfig;
	
	if (phcan->Instance == FDCAN1) {
	/* Configure Rx filter */
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.FilterType = FDCAN_FILTER_MASK;
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
		sFilterConfig.FilterID1 = 0;//0x111;
		sFilterConfig.FilterID2 = 0;//0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
	}
	//else if (phcan->Instance == CAN2) {
	//	myFilter[1].FilterIdHigh = 0;//CAN_STD_ID<<5;//0x0000; 
	//	myFilter[1].FilterIdLow = 0x0000; 
	//	myFilter[1].FilterMaskIdHigh = 0;//(CAN_STD_ID<<3)>>16;//0x0000; 
	//	myFilter[1].FilterMaskIdLow = 0;//((CAN_STD_ID<<3)&0xffff)|(0x1<<2);//0x0000; 
	//	HAL_CAN_ConfigFilter(&hcan, &myFilter[1]);
	//}
	//else if (phcan->Instance == CAN3) {
	//	myFilter[2].FilterIdHigh = 0;//CAN_STD_ID<<5;//0x0000; 
	//	myFilter[2].FilterIdLow = 0x0000; 
	//	myFilter[2].FilterMaskIdHigh = 0;//(CAN_STD_ID<<3)>>16;//0x0000; 
	//	myFilter[2].FilterMaskIdLow = 0;//((CAN_STD_ID<<3)&0xffff)|(0x1<<2);//0x0000; 
	//	HAL_CAN_ConfigFilter(&hcan, &myFilter[2]);
	//}
	HAL_FDCAN_ConfigFilter(phcan, &sFilterConfig);
}

void CCANDrv_CAN1Init(void)
{
	int fifo0RxNum;
	FDCAN_FilterTypeDef sFilterConfig;

	fifo0RxNum = 1;
	CanBuf[0].fifo0TxNum = 0;
	/*                Bit time configuration:
	Bit time parameter         | Nominal      |  Data
	---------------------------|--------------|----------------
	fdcan_ker_ck               	| 80 MHz       | 80 MHz
	Time_quantum (tq)          | 125 ns        | 125 ns
	Synchronization_segment    | 1 tq         | 1 tq
	Propagation_segment        | 3 tq        | 3 tq
	Phase_segment_1            | 2 tq         | 2 tq
	Phase_segment_2            | 4 tq         | 4 tq
	Synchronization_Jump_width | 8 tq         | 4 tq
	Bit_length                 | 8 tq = 8 탎 | 8 tq = 8 탎
	Bit_rate                     | 1 MBit/s      | 1 MBit/s
	*/
	hfdcan1.State = HAL_FDCAN_STATE_RESET;
	hfdcan1.Instance = FDCAN1;
	hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;//FDCAN_FRAME_FD_BRS;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.AutoRetransmission = ENABLE;
	hfdcan1.Init.TransmitPause = DISABLE;
	hfdcan1.Init.ProtocolException = ENABLE;
	hfdcan1.Init.NominalPrescaler = 0x1; /* tq = NominalPrescaler x (1/fdcan_ker_ck) */
	hfdcan1.Init.NominalSyncJumpWidth = 1;//0x10;//0x8;
	hfdcan1.Init.NominalTimeSeg1 = 0x3F; /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
	hfdcan1.Init.NominalTimeSeg2 = 0x10;
	hfdcan1.Init.DataPrescaler = 0x1;
	hfdcan1.Init.DataSyncJumpWidth = 0x10;//0x8;
	hfdcan1.Init.DataTimeSeg1 = 0x3F; /* DataTimeSeg1 = Propagation_segment + Phase_segment_1 */
//	hfdcan1.Init.DataTimeSeg2 = 0x10;
//	hfdcan1.Init.MessageRAMOffset = 0;
	hfdcan1.Init.StdFiltersNbr = 1;
	hfdcan1.Init.ExtFiltersNbr = 0;
//	hfdcan1.Init.RxFifo0ElmtsNbr = fifo0RxNum;
//	hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
//	hfdcan1.Init.RxFifo1ElmtsNbr = 0;
//	hfdcan1.Init.RxBuffersNbr = 0;
//	hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
//	hfdcan1.Init.TxEventsNbr = 0;
//	hfdcan1.Init.TxBuffersNbr = CanBuf[0].fifo0TxNum;
//	hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
	hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;//FDCAN_TX_QUEUE_OPERATION;;
//	hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	HAL_FDCAN_Init(&hfdcan1);

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;//0x111;
	sFilterConfig.FilterID2 = 0;//0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

	/* Configure Rx FIFO 0 watermark to 2 */
	//HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, fifo0RxNum);

	/* Activate Rx FIFO 0 watermark notification */
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, CanBuf[0].fifo0TxNum);

	/* Prepare Tx Header */
	CanBuf[0].txHeader.Identifier = 0x581;
	CanBuf[0].txHeader.IdType = FDCAN_STANDARD_ID;
	CanBuf[0].txHeader.TxFrameType = FDCAN_DATA_FRAME;
	CanBuf[0].txHeader.DataLength = FDCAN_DLC_BYTES_8;
	CanBuf[0].txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;//FDCAN_ESI_PASSIVE;//;
	CanBuf[0].txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CanBuf[0].txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CanBuf[0].txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CanBuf[0].txHeader.MessageMarker = 0;
	CanBuf[0].head = CanBuf[0].tail = 0;

	/* Start the FDCAN module */
	HAL_FDCAN_Start(&hfdcan1);
}

void CCANDrv_CAN2Init(void)
{
	int fifo0Num;
	FDCAN_FilterTypeDef sFilterConfig;

	fifo0Num = 1;
	/*                Bit time configuration:
	Bit time parameter         | Nominal      |  Data
	---------------------------|--------------|----------------
	fdcan_ker_ck               | 20 MHz       | 20 MHz
	Time_quantum (tq)          | 50 ns        | 50 ns
	Synchronization_segment    | 1 tq         | 1 tq
	Propagation_segment        | 23 tq        | 1 tq
	Phase_segment_1            | 8 tq         | 4 tq
	Phase_segment_2            | 8 tq         | 4 tq
	Synchronization_Jump_width | 8 tq         | 4 tq
	Bit_length                 | 40 tq = 2 탎 | 10 tq = 0.5 탎
	Bit_rate                   | 0.5 MBit/s   | 2 MBit/s
	*/
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = ENABLE;
	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = ENABLE;
	hfdcan2.Init.NominalPrescaler = 10;//0x1; /* tq = NominalPrescaler x (1/fdcan_ker_ck) */
	hfdcan2.Init.NominalSyncJumpWidth = 1;//0x8;
	hfdcan2.Init.NominalTimeSeg1 = 14;//32;//0x3; /* NominalTimeSeg1 = Propagation_segment + Phase_segment_1 */
	hfdcan2.Init.NominalTimeSeg2 = 2;//9;//52;//0xC;
	hfdcan2.Init.DataPrescaler = 10;//2;//0x1;
	hfdcan2.Init.DataSyncJumpWidth = 1;//0x8;
	hfdcan2.Init.DataTimeSeg1 = 14;//32;//0x2; /* DataTimeSeg1 = Propagation_segment + Phase_segment_1 */
	hfdcan2.Init.DataTimeSeg2 = 2;//9;//52;//0x1;
//	hfdcan2.Init.MessageRAMOffset = 0;
	hfdcan2.Init.StdFiltersNbr = 1;
	hfdcan2.Init.ExtFiltersNbr = 0;
//	hfdcan2.Init.RxFifo0ElmtsNbr = fifo0Num;
//	hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
//	hfdcan2.Init.RxFifo1ElmtsNbr = 0;
//	hfdcan2.Init.RxBuffersNbr = 0;
//	hfdcan2.Init.TxEventsNbr = 0;
//	hfdcan2.Init.TxBuffersNbr = CanBuf[1].fifo0TxNum;
//	hfdcan2.Init.TxFifoQueueElmtsNbr = 1;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
//	hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	HAL_FDCAN_Init(&hfdcan2);

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0;//0x111;
	sFilterConfig.FilterID2 = 0;//0x7FF; /* For acceptance, MessageID and FilterID1 must match exactly */
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

	/* Configure Rx FIFO 0 watermark to 2 */
	//HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, fifo0Num);

	/* Activate Rx FIFO 0 watermark notification */
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, CanBuf[1].fifo0TxNum);

	/* Prepare Tx Header */
	CanBuf[1].txHeader.Identifier = 0x581;
	CanBuf[1].txHeader.IdType = FDCAN_STANDARD_ID;
	CanBuf[1].txHeader.TxFrameType = FDCAN_DATA_FRAME;
	CanBuf[1].txHeader.DataLength = FDCAN_DLC_BYTES_8;
	CanBuf[1].txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;//FDCAN_ESI_PASSIVE;//;
	CanBuf[1].txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CanBuf[1].txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CanBuf[1].txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CanBuf[1].txHeader.MessageMarker = 0;
	CanBuf[1].head = CanBuf[1].tail = 0;

	/* Start the FDCAN module */
	HAL_FDCAN_Start(&hfdcan2);
}

void CCANDrv_InitHW(uint8_t flag)
{
	if (flag) {
		CAN_Init(&hfdcan1);
		CAN_Init(&hfdcan1);
		//CAN_Init(&hcan3);
	}
	
	CCANDrv_CAN1Init();
	CCANDrv_CAN2Init();
}

void CCANDrv_SetID(uint8_t node, uint32_t id)
{
	CanDrv[node].rx.head = 0;
	CanDrv[node].rx.tail = 0;
	CanDrv[node].rx.status = 0;
	CanDrv[node].rx.id = id;
	CanDrv[node].tx.bus = CAN_BUS_NUM[node];
}

void CCANDrv_NullProcess(uint8_t node, unsigned char *buf)
{
}

void CCANDrv_SetISRFn(void (*fp)(uint8_t node, unsigned char *buf))
{
	fpCANDrv_ISR = fp;
}

void CCANDrv_CANDrv(void)
{
	int i;
	
	for (i = 0; i < CAN_PORT_CNT; i++) {
		//CanPort[i].runTime = 0;
	}
	CCANDrv_SetISRFn(CCANDrv_NullProcess);
	CCANDrv_InitHW(0);
}

