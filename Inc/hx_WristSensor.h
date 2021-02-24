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
* in deciphering, decoding, reverse engineering or in any way altering the source
* code is strictly prohibited, unless the prior written consent of Hexar Systems, Inc.
* is obtained.
*
* Filename		: hx_InsolSensor.h
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32F407
* Compiler		: IAR
* Created      	: 2017/02/21
* Description		: Insol Sensor
*******************************************************************************
*
*/

#ifndef _INSOL_SENSOR_H_
#define _INSOL_SENSOR_H_

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx.h"

/* Private define ------------------------------------------------------------*/
//#define SUPPORT_HW_V1_2				//180408
//#define SUPPORT_UART_PRINT
//#define SUPPORT_ALL_SENSOR

//#define CAN_RX_BUF_NUM				(20)
//#define CAN_TX_BUF_NUM				(21)
#define CAN_DATA_SEND_NUM				6
#define NumOfAdcChan 					2 // num of scanned ADC channels
#define DMABUFSIZE 						NumOfAdcChan + 2 // total elements of DMABuf-Array

#define WRIST_SENSOR_NUM				2

#define WRIST_SENSOR_MAX_VALUE			4096
#define LED_RED							GPIO_PIN_13
#define LED_GREEN						GPIO_PIN_14
//#define LED_ADC_RUN_BLINK 				HAL_GPIO_TogglePin(GPIOB, LED_ORANGE)
//#define LED_ADC_RUN_OFF 				HAL_GPIO_WritePin(GPIOB, LED_ORANGE, GPIO_PIN_SET)
#define LED_CAN_TX_BLINK 				HAL_GPIO_TogglePin(GPIOC, LED_GREEN)
#define LED_CAN_TX_OFF 					HAL_GPIO_WritePin(GPIOC, LED_GREEN, GPIO_PIN_RESET)
#define LED_CAN_TX_ON 					HAL_GPIO_WritePin(GPIOC, LED_GREEN, GPIO_PIN_SET)
#define LED_RED_TOGGLE					HAL_GPIO_TogglePin(GPIOC, LED_RED)
#define LED_RED_ON						HAL_GPIO_WritePin(GPIOC, LED_RED, GPIO_PIN_SET)
#define LED_RED_OFF						HAL_GPIO_WritePin(GPIOC, LED_RED, GPIO_PIN_RESET)
//#define LED_BLUE_TOGGLE					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14)
#define LED_GREEN_TOGGLE				HAL_GPIO_TogglePin(GPIOC, LED_GREEN)

#define COMPANY_LENGTH					15
#define MODEL_LENGTH					10
#define FW_VER							0x10	//v1.0
#define HW_VER							0x10	//v0.2(18.04.17)
#define FW_USER							'H'		//user : hexar
#define FW_CREATE_YEAR					21
#define FW_CREATE_MONTH					2
#define FW_CREATE_DAY					2
#define FW_CREATE_HOUR					9
#define FW_CREATE_MIN					34

//extern const char COMPANY_t[COMPANY_LENGTH];// = {"HEXAR SYSTEMS"};
//extern const char MODEL_t[MODEL_LENGTH];// = {"INSOL"};
//extern const uint8_t FW_VER[4];// = {'1', '0', 'K', 'G'};		//v1.0.KG
//extern const uint8_t CREATE_DATE[3];// = {17, 3, 24};		//17y, 3m, 24d
//extern const uint8_t CREATE_TIME[3];// = {16, 31, 55};	//hour, min, sec

enum WRIST_MODEL_NUM {
	WMN_SELF,
	WMN_SMART,
	WMN_MAX
};

enum eSSYTEM_STATUS_CAN {
	SSC_RX_ERR				= 0x01,
	SSC_TX_ERR				= 0x02,
};

enum eSSYTEM_STATUS_MCU {
	SSM_ADC_ERR			= 0x01,
	SSM_CAN_ERR			= 0x02,
};

/* Private typedef -----------------------------------------------------------*/
typedef struct tag_WRIST{
	//uint8_t sourcePos;
	uint8_t m_sendOrder;
	//uint16_t sensorValueBuf[WRIST_SENSOR_NUM];
	struct {
		uint8_t complete;
		uint8_t cnt;
	}m_adc;
	struct {
		uint16_t adcCnt;
		uint16_t canCnt;
	}m_led;
	uint8_t m_runMode;
	uint8_t m_oldRunMode;
	struct {
		uint16_t sendTime;
		uint16_t sendTimeBackup;
		uint8_t outputType;
	}m_data;
	struct {
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Fx;
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Fy;
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Fz;
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Tx;
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Ty;
		union {
			int16_t all;
			struct {
				unsigned short lower	:8;
				unsigned short upper 	:8;
			}b;
		}Tz;
		//uint8_t statusOfOverload; //R13 : Status of Overload	
		uint8_t tmpSaveTorqueUpperData;
		uint8_t fReceiveDataMark;
		float forceDivider;
		float torqueDivider;
	}m_RFT; //6axis
	struct {
		uint16_t *pValue;
		uint16_t value;
		int32_t offset;
		uint16_t angle; //x100
	}m_Enc[2];
	struct {
		uint32_t rxid;
		uint32_t txid;
		//uint16_t chkTime;
		int rv;
		uint8_t timeout;
		uint32_t complete;
	}m_can[2];
	struct {
		uint8_t ok;
	}m_cal;
	union {
		uint16_t all;
		struct {
			unsigned short encError		:1;
			unsigned short rftOverload	:1;
			unsigned short reserved 	:14;
		}b;
	}m_systemInfo;
}WRIST;

typedef struct tag_INSOL_Timer{
	uint32_t tick;
	uint16_t tick2;
	//uint16_t sendDataTick;
}WRIST_TIMER;

typedef union tag_SYSTEM_STATUS{
	uint64_t all;
	uint8_t can1;
	uint8_t can2;
	uint8_t sensor1;
	uint8_t sensor2;
	uint8_t mcu1;
	uint8_t mcu2;
	uint8_t etc1;
	uint8_t etc2;
}SYSTEM_STATUS;
/*
typedef struct tagCAN_INFO{
	struct {
		uint8_t head;
		uint8_t tail;
		uint8_t buf[CAN_RX_BUF_NUM][8];
	}rx;
	struct {
		uint8_t buf[WRIST_SENSOR_NUM*2+1];	// 2 is 12bit  
		uint16_t *ptr[WRIST_SENSOR_NUM+1];
		uint8_t cnt;
	}tx;
}CAN_INFO;
*/
enum _RUN_MODE {
	RM_STANDBY,
	RM_SLEEP,
	RM_CALIBRATION,
	RM_NORMAL,
	RM_MAX
};

enum _DATA_OUTPUT_TYPE {
	DOT_NONE,
	DOT_CAN				= 0x01,
	DOT_UART			= 0x02,
	DOT_CAN_UART		= 0x04
};


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint16_t AdcDmaBuf[DMABUFSIZE];
extern uint16_t AdcCalBuf[WRIST_SENSOR_NUM];
extern WRIST Wrist;
extern WRIST_TIMER WristTimer;
extern SYSTEM_STATUS SysStatus;
//extern CAN_INFO CanInfo;

/* Private function prototypes -----------------------------------------------*/
extern void (*fnADCDrv_Copy2Buf)(void);
extern void (*fnWrist_RunMode)(void);
//extern void ADCDrv_Init();
//extern void ADCDrv_SelectSensorSource(void);
extern void CWrist_Init();
extern void CWrist_RunMode_Calibration(void);
//extern void Insol_RunMode_Standby(void);
extern void CWrist_RunMode_Normal(void);
extern void CWrist_SetRunMode(uint8_t runmode);
extern void CWrist_Print2Uart_SystemInfo(void);
extern int CWrist_SaveParamToFlash(char *buf);
//extern uint8_t Insol_GetSensorONNum(uint32_t data);
//extern void CWrist_SetBufPtr(uint8_t data_size, uint32_t data);
extern void CWrist_SetEncValuePtr(uint8_t ch, uint16_t *ptr);
extern void CWrist_FT_StartDataOutput(void);
extern void CWrist_FT_SetBias(uint8_t is_on);


#endif //_INSOL_SENSOR_H_

