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
* Filename		: hx_InsolSensor.c
* Programmer(s)	: PJG
*                   	  Other name if it be
* MCU 			: STM32G473CET6
* Compiler		: IAR
* Created      	: 2021/01/27
* Description		: Wrist Sensor
*******************************************************************************
*
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "hx_WristSensor.h"
#include "hx_CANDrv.h"
#include "hx_mcuFlash.h"
#include "hx_CANOpen.h"

/* Private define ------------------------------------------------------------*/
#define LED_ADC_RUN_BLINK_TIME					3500
#define LED_CAN_TX_BLINK_TIME					5
#define CAN_ERR_RECHECK_TIME					1000
#define USE_PULLUP								1 		//reduce nois than floating


/* Private typedef -----------------------------------------------------------*/
struct _tagSETUP_INFO{	//셋업(옵션)창의 변수 저장
	uint32_t canRxID;					
	uint32_t canTxID;					
}Setup, SetupOld;

typedef struct _tagFLASH_SAVE_SYSTEM_INFO {
	char tag[2];
	char company[COMPANY_LENGTH];
	char model[MODEL_LENGTH];
	char swVer;
	char hwVer;
	char createDate[3];
	char createTime[3];
	char modifyDate[3];
	char modifyTime[3];
	uint32_t writeTime;	//flash re-write times(max 10,000)
	uint32_t nextNode;
	uint32_t replaceNode;
	struct _tagSETUP_INFO setup;
}SYSTEM_INFO;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint16_t AdcDmaBuf[DMABUFSIZE] = {0xAA, 0, 0, 0xBB};
uint16_t AdcCalBuf[WRIST_SENSOR_NUM];	//base data with no load
uint16_t ForceConstant[WRIST_SENSOR_NUM];
//CAN_INFO CanInfo;
WRIST Wrist;
WRIST_TIMER WristTimer;
SYSTEM_STATUS SysStatus;
char CommonBuf[FLASH_PAGE_SIZE1*2];	//for mcu flash page size
const char COMPANY_t[COMPANY_LENGTH] = {"HEXAR HUMANCARE"};
const char MODEL_t[MODEL_LENGTH] = {"WRIST"};
//const uint8_t FW_VER[4] = {'1', '0', 'K', 'G'};		//v1.0.KG
//const uint8_t CREATE_DATE[3] = {17, 3, 24};		//17y, 3m, 24d
//const uint8_t CREATE_TIME[3] = {16, 31, 55};	//hour, min, sec

extern ADC_HandleTypeDef hadc1;
extern IWDG_HandleTypeDef hiwdg;


/* Private function prototypes -----------------------------------------------*/
//void (*fnADCDrv_Copy2Buf)(void);
void (*fnWrist_RunMode)(void);

void CWrist_RunMode_Calibration(void)
{

}

void CWrist_RunMode_Standby(void)
{

}

void CWrist_RunMode_Normal(void)
{
	//int i;//, j;
	int16_t temp;
	float ftemp;

  	//if (Wrist.adc.complete == 0) return;

	if (WristTimer.tick < Wrist.data.sendTime) return;
#if 0	
	if (Wrist.data.outputType&DOT_UART) {
		//if (!Insol.sendOrder) 
		{
			//printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,\r\n", 
			//	Wrist.sensorValueBuf[0], Wrist.sensorValueBuf[1],
			//	Wrist.sensorValueBuf[2], Wrist.sensorValueBuf[3],
			//	);

			Wrist.led.canCnt++;
			if (Wrist.led.canCnt > LED_CAN_TX_BLINK_TIME) {
				LED_CAN_TX_BLINK;
				Wrist.led.canCnt = 0;
			}
		}
		WristTimer.tick = 0;
	}
#endif
	
	Wrist.can.rv = 0;
	if (Wrist.data.outputType&DOT_CAN) {
		//if (Insol.can.complete > 0) Insol.can.complete--;    
		//if (Insol.can.complete != 0) return;
		Wrist.sendOrder = 0;
		CCANDrv_SetTxStdID(&CanBuf[1].txHeader, Wrist.can.txid + Wrist.sendOrder);

		CanDrv[CBC_WRIST].tx.buf[0][0] = Wrist.FTSensor.Fx.upper;
		CanDrv[CBC_WRIST].tx.buf[0][1] = Wrist.FTSensor.Fx.lower;
		CanDrv[CBC_WRIST].tx.buf[0][2] = Wrist.FTSensor.Fy.upper;
		CanDrv[CBC_WRIST].tx.buf[0][3] = Wrist.FTSensor.Fy.lower;
		CanDrv[CBC_WRIST].tx.buf[0][4] = Wrist.FTSensor.Fz.upper;
		CanDrv[CBC_WRIST].tx.buf[0][5] = Wrist.FTSensor.Fz.lower;
		CanDrv[CBC_WRIST].tx.buf[0][6] = Wrist.FTSensor.Tx.upper;
		CanDrv[CBC_WRIST].tx.buf[0][7] = Wrist.FTSensor.Tx.lower;
		CanDrv[CBC_WRIST].tx.cnt = 8;
		Wrist.can.rv = CCANDrv_WriteFile(CBC_WRIST, CanDrv[CBC_WRIST].tx.buf[0], CanDrv[CBC_WRIST].tx.cnt);
		if (!Wrist.can.rv) return;
#if 1
		Wrist.sendOrder++;
		CCANDrv_SetTxStdID(&CanBuf[1].txHeader, Wrist.can.txid + Wrist.sendOrder);
		
		CanDrv[CBC_WRIST].tx.buf[0][0] = Wrist.FTSensor.Ty.upper;
		CanDrv[CBC_WRIST].tx.buf[0][1] = Wrist.FTSensor.Ty.lower;
		CanDrv[CBC_WRIST].tx.buf[0][2] = Wrist.FTSensor.Tz.upper;
		CanDrv[CBC_WRIST].tx.buf[0][3] = Wrist.FTSensor.Tz.lower;
		
		if (*Wrist.Enc[0].pValue > 1024) Wrist.Enc[0].value = 1024;
		else Wrist.Enc[0].value = *Wrist.Enc[0].pValue;
		
		if (Wrist.Enc[0].value < 2) { 
			Wrist.Enc[0].angle = 0; 
		}
		else {
			ftemp = (float)(Wrist.Enc[0].value)*0.35+1.25;
			if (ftemp >= Wrist.Enc[0].offset) {
				Wrist.Enc[0].angle = (uint16_t)((ftemp-Wrist.Enc[0].offset)*100); 
			}
			else {
				Wrist.Enc[0].angle = (uint16_t)((359.65-Wrist.Enc[0].offset+ftemp)*100); 
			}
		}
		
		if (*Wrist.Enc[1].pValue > 1024) Wrist.Enc[1].value = 1024;
		else Wrist.Enc[1].value = *Wrist.Enc[1].pValue;
		
		if (Wrist.Enc[1].value < 2) { 
			Wrist.Enc[1].angle = 0; 
		}
		else {
			ftemp = (float)(Wrist.Enc[1].value)*0.35+1.25;
			if (ftemp >= Wrist.Enc[1].offset) {
				Wrist.Enc[1].angle = (uint16_t)((ftemp-Wrist.Enc[1].offset)*100); 
			}
			else {
				Wrist.Enc[1].angle = (uint16_t)((359.65-Wrist.Enc[1].offset+ftemp)*100); 
			}
		}
		CanDrv[CBC_WRIST].tx.buf[0][4] = Wrist.Enc[0].angle&0xff;
		CanDrv[CBC_WRIST].tx.buf[0][5] = (Wrist.Enc[0].angle>>8);
		CanDrv[CBC_WRIST].tx.buf[0][6] = Wrist.Enc[1].angle&0xff;
		CanDrv[CBC_WRIST].tx.buf[0][7] = (Wrist.Enc[1].angle>>8);
		CanDrv[CBC_WRIST].tx.cnt = 8;
		Wrist.can.rv = CCANDrv_WriteFile(CBC_WRIST, CanDrv[CBC_WRIST].tx.buf[0], CanDrv[CBC_WRIST].tx.cnt);
		if (!Wrist.can.rv) return;
#endif		
		//Wrist.sendOrder++;
		//if (Wrist.sendOrder > 1) Wrist.sendOrder = 0;
		
		Wrist.led.canCnt++;
		if (Wrist.led.canCnt > LED_CAN_TX_BLINK_TIME) {
			LED_CAN_TX_BLINK;
			Wrist.led.canCnt = 0;
		}
		WristTimer.tick = 0;
	}
	
	if (Wrist.can.rv == -1) {
		if (!Wrist.data.sendTimeBackup) Wrist.data.sendTimeBackup = Wrist.data.sendTime;
		Wrist.data.sendTime = CAN_ERR_RECHECK_TIME;
	}
	else {
		if (Wrist.data.sendTimeBackup > 0) {
			Wrist.data.sendTime = Wrist.data.sendTimeBackup;
			Wrist.data.sendTimeBackup = 0;
		}
	}

	//if (Wrist.led.adcCnt > LED_ADC_RUN_BLINK_TIME) {
		//LED_ADC_RUN_BLINK;
	//	Wrist.led.adcCnt = 0;
	//}

}

void CWrist_SetRunMode(uint8_t runmode)
{
	//Wrist.sourcePos = 0;
	Wrist.adc.complete = 0;

	if (Wrist.runMode == runmode) return;

	switch (runmode) {
	case RM_STANDBY:
	case RM_SLEEP:
		Wrist.runMode = RM_STANDBY;
		//fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnWrist_RunMode = CWrist_RunMode_Standby;
		//HAL_ADC_Stop_DMA(&hadc1);
		//LED_ADC_RUN_OFF;
		LED_CAN_TX_OFF;
		break;
	case RM_NORMAL:
		Wrist.runMode = RM_NORMAL;
		//fnADCDrv_Copy2Buf = ADCDrv_Copy2Buf;
		fnWrist_RunMode = CWrist_RunMode_Normal;
		//Wrist_SetBufPtr(Insol.data.size, Insol.sensorONPos);
		//HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		//if (Wrist.sensorONNum > 3) CanInfo.tx.cnt = 8;
		//else CanInfo.tx.cnt = Wrist.sensorONNum<<1;
		break;
	case RM_CALIBRATION:
		Wrist.adc.cnt = 1;		// avg varible
		Wrist.runMode = RM_CALIBRATION;
		//fnADCDrv_Copy2Buf = ADCDrv_Copy2CalBuf;
		fnWrist_RunMode = CWrist_RunMode_Calibration;
		//HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&AdcDmaBuf[1], NumOfAdcChan );
		break;
	default:
		break;
	}
	
	Wrist.oldRunMode = runmode;
	Wrist.runMode = runmode;
	
}

uint32_t CWrist_LoadParamFromFlash(char *buf)
{
	SYSTEM_INFO *pFSSysInfo;
	uint32_t addr;
	int i;

#ifdef SUPPORT_MCU_FLASH
	addr = FLASH_SYSTEM_INFO_ADDR;
	for (i = 0; i < FLASH_USER_END_ADDR; i += FLASH_PAGE_SIZE1) {
		FlashDrv_Read(addr, buf, sizeof(SYSTEM_INFO));
		pFSSysInfo = (SYSTEM_INFO *)buf;
		if (pFSSysInfo->company[0] != 'H' ||
			pFSSysInfo->company[1] != 'E' ||
			pFSSysInfo->company[2] != 'X') {
			return 0;
		}
		else {
			if (pFSSysInfo->writeTime > FLASH_ENDURANCE) {
				if (pFSSysInfo->replaceNode == 0xffffffff || 
					pFSSysInfo->replaceNode == 0) return 0;
				else {
					addr = pFSSysInfo->replaceNode;
				}
			}
			else {
				break;
			}
		}
	}

	return addr;
#else 
	return 0;
#endif
}

int CWrist_SaveParamToFlash(char *buf)
{
	SYSTEM_INFO *pFSSysInfo;
	int i;
	uint32_t addr;

#ifdef SUPPORT_MCU_FLASH
	addr = CWrist_LoadParamFromFlash(buf);

	pFSSysInfo = (SYSTEM_INFO *)buf;
	//pFSSysInfo->setup.vol = Setup.vol;
	//pFSSysInfo->setup.bright = Setup.bright;
	//pFSSysInfo->setup.angle = Setup.angle;
	//pFSSysInfo->setup.speed = Setup.speed;
	//pFSSysInfo->setup.repeat = Setup.repeat;
	//pFSSysInfo->setup.lsrpt_en = Setup.lsrpt_en;
	//pFSSysInfo->setup.sndGuid = Setup.sndGuid;
	//pFSSysInfo->setup.hiAngle = Setup.hiAngle;

	pFSSysInfo->tag[0] = 'H';
	pFSSysInfo->tag[1] = 'X';
	for (i = 0; i < COMPANY_LENGTH; i++) pFSSysInfo->company[i] = COMPANY_t[i];
	for (i = 0; i < MODEL_LENGTH; i++) pFSSysInfo->model[i] = MODEL_t[i];
	pFSSysInfo->swVer = FW_VER;
	pFSSysInfo->hwVer = HW_VER;
	pFSSysInfo->createDate[0] = FW_CREATE_YEAR;
	pFSSysInfo->createDate[1] = FW_CREATE_MONTH;
	pFSSysInfo->createDate[2] = FW_CREATE_DAY;
	pFSSysInfo->createTime[0] = 16;
	pFSSysInfo->createTime[1] = 20;
	pFSSysInfo->createTime[2] = 40;
	pFSSysInfo->modifyTime[0] = 'a';
	pFSSysInfo->modifyTime[1] = 'b';
	pFSSysInfo->modifyTime[2] = 'c';
	
	pFSSysInfo->nextNode = 0;
	pFSSysInfo->replaceNode = 0;
	pFSSysInfo->replaceNode = 0x12345678;

	pFSSysInfo->setup.canTxID = Wrist.can.txid;
	pFSSysInfo->setup.canRxID = Wrist.can.rxid;
		
	if (addr >= FLASH_SYSTEM_INFO_ADDR) {
		pFSSysInfo->writeTime++;
		if (!FlashDrv_Write(addr, buf, sizeof(SYSTEM_INFO))) return 0;
	}
	else {
		pFSSysInfo->writeTime = 1;
		if (!FlashDrv_Write(FLASH_SYSTEM_INFO_ADDR, 
					buf, sizeof(SYSTEM_INFO))) return 0;
	}

	return 1;
#else
	return 0;
#endif
}

/*
uint8_t Insol_GetSensorONNum(uint32_t data)
{
	int i;
	uint8_t cnt;

	cnt = 0;
	for (i = 0; i < 32; i++) {
		if (data & 0x1) cnt++;
		data >>= 1;
	}
	return cnt;
}
*/

void CWrist_SetEncValuePtr(uint8_t ch, uint16_t *ptr)
{
	Wrist.Enc[ch].pValue = ptr;
}

void CWrist_Print2Uart_SystemInfo(void)
{
	SYSTEM_INFO *psi;
	
	if (CWrist_LoadParamFromFlash(CommonBuf)) {
		psi = (SYSTEM_INFO *)CommonBuf;
		if (psi->setup.canTxID < 0x1FFFFFFF) Wrist.can.txid = psi->setup.canTxID;
		if (psi->setup.canRxID < 0x1FFFFFFF) Wrist.can.rxid = psi->setup.canRxID;
	}
	
	printf("------- HEXAR SYSTEMS -------\r\n");
	printf("F/W Version : %d.%d\r\n", (FW_VER&0x0)>>4, FW_VER&0x0f);
	printf("User : %c\r\n", FW_USER);
	printf("CAN ID : Tx : %Xh, Rx : %Xh\r\n", Wrist.can.txid, Wrist.can.rxid);
	printf("Create date : %d/%d/%d %d:%d\r\n", 
		FW_CREATE_YEAR, FW_CREATE_MONTH, FW_CREATE_DAY, 
		FW_CREATE_HOUR, FW_CREATE_MIN);
	printf("------- Wrist Sensor -------\r\n");
}

void CWrist_Init(void)
{
	//int i;
	
	//LED_ADC_RUN_BLINK;
	//LED_CAN_TX_BLINK;

	Wrist.sendOrder = 0;
	Wrist.led.adcCnt = 0;
	Wrist.led.canCnt = 0;
	Wrist.data.sendTime = 21;
	Wrist.data.sendTimeBackup = 0;
	fnWrist_RunMode = CWrist_RunMode_Standby;
	CCANDrv_SetISRFn(CANOpen_ProcessSdo);
	Wrist.runMode = Wrist.oldRunMode = RM_STANDBY;
	#ifdef SUPPORT_UART_PRINT
	Wrist.data.outputType = DOT_UART;
	#else
	Wrist.data.outputType = DOT_CAN;
	#endif
	Wrist.Enc[0].offset = 0;
	Wrist.Enc[1].offset = 0;

	CCANDrv_SetID(CBC_FT1, SID_R_FT1);
	CCANDrv_SetID(CBC_FT2, SID_R_FT2);
	CCANDrv_SetID(CBC_WRIST, SID_R_WRIST);
	Wrist.can.txid = SID_T_WRIST;

	//LED_ADC_RUN_OFF;
	LED_CAN_TX_OFF;
	
#ifdef SUPPORT_MCU_FLASH
	FlashDrv_SetTempBuf(&CommonBuf[FLASH_PAGE_SIZE1]);
	FlashDrv_SetParam(FLASH_PAGE_SIZE1);
#endif
}

#if 0
#include "stdarg.h"
#include "hal_uart.h"
#include 
#include "hal_types.h"

void printf(char *format, ...);

static void sendByte(unsigned char byte)
{
    HalUARTWrite(HAL_UART_PORT_0, &byte, 1); //change port to suit your needs
}

static void putc(unsigned char c)
{
    sendByte(c);
}

static void puts(uint8 *str)
{
    HalUARTWrite(HAL_UART_PORT_0, str, strlen((const char*)str)); //change port to suit your needs
}

static const unsigned long dv[] =
{
    // 4294967296 // 32 bit unsigned max
    1000000000,// +0
    100000000, // +1
    10000000, // +2
    1000000, // +3
    100000, // +4
    // 65535 // 16 bit unsigned max
    10000, // +5
    1000, // +6
    100, // +7
    10, // +8
    1, // +9
};

static void xtoa(unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;

    if (x)
    {
        while (x < *dp)
            ++dp;
        do
        {
            d = *dp++;
            c = '0';
            while (x >= d)
                ++c, x -= d;
            putc(c);
        } while (!(d & 1));
    } else
        putc('0');
}

static void puth(unsigned n)
{
    static const char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8',
        '9', 'A', 'B', 'C', 'D', 'E', 'F' };

    putc(hex[n & 15]);
}

void printf(char *format, ...)
{
    char c;
    int i;
    long n;

    va_list a;
    va_start(a, format);
    while(c = *format++)
    {
        if(c == '%')
        {
            switch(c = *format++)
            {
                case 's': // String
                    puts(va_arg(a, char*));
                    break;

                case 'c':// Char
                    putc(va_arg(a, char));
                    break;

                case 'i':// 16 bit Integer
                case 'u':// 16 bit Unsigned
                    i = va_arg(a, int);
                    if(c == 'i' && i < 0)
                        i = -i, putc('-');
                    xtoa((unsigned)i, dv + 5);
                    break;

                case 'l':// 32 bit Long
                case 'n':// 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' && n < 0)
                        n = -n, putc('-');
                    xtoa((unsigned long)n, dv);
                    break;

                case 'x':// 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(i >> 12);
                    puth(i >> 8);
                    puth(i >> 4);
                    puth(i);
                    break;

                case 0:
                    return;

                default:
                    goto bad_fmt;
            }
        }
        else
            bad_fmt: putc(c);
    }

    va_end(a);
}
//출처: http://ingorae.tistory.com/1423 [잉고래의 잇다이어리]
#endif

