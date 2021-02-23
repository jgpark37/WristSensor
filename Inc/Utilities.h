/********************************************************************/
/*              Header for Utilities								*/
/********************************************************************/
#ifndef _UTILITIES_H
#define _UTILITIES_H

//#define	TRUE	1
//#define	FALSE	0
//#define	SET		1
#define	CLEAR	0
//#define	ENABLE	1
//#define	DISABLE	0
#define	HIGH	1
#define	LOW		0
#define	ON		1
#define	OFF		0
#define	_ON		0
#define	_OFF	1
#define	INPUT	1
#define	OUTPUT	0
#define	START	1
#define	STOP	0
#define TURNON	0
#define TURNOFF 1

#define FORWARD 0
#define BACKWARD 1

//===========================================================================================================
// 현재 Ver 1.0 (수정일자 : 2012.03.01)
// 개정이력 : 최초버전
// 작성기관 : Flexsystem(한국산업기술대학교)

// CID  = CAN ID
//***********************************************
//***********************************************
//      Base ID 정의
//***********************************************

//===========================================================================================================
#define SID_R_WRIST_BASE		0x629
#define SID_T_WRIST_BASE		0x5A9
#define WRIST1_NODE				0

#define SID_R_FT_BASE			0x01
#define SID_T_FT_BASE			0x64
#define FT1_NODE0				0
#define FT1_NODE1				1

#define SID_T_WRIST				( SID_T_WRIST_BASE ) + ( WRIST1_NODE )
#define SID_R_WRIST				( SID_R_WRIST_BASE ) + ( WRIST1_NODE )

#define SID_T_FT				( SID_T_FT_BASE ) + ( FT1_NODE0 )
#define SID_R_FT1				( SID_R_FT_BASE ) + ( FT1_NODE0 )
#define SID_R_FT2				( SID_R_FT_BASE ) + ( FT1_NODE1 )


/*
***********************************************************
*                       Type Define
***********************************************************
*/
//xc16
typedef	signed char		INT8S;			/* signed 8bits integer,	-128 ~ 127							*/
typedef	unsigned char	INT8U;			/* unsigned 8bits integer,	0 ~ 255								*/
typedef	signed int		INT16S;			/* signed 16bits integer, 	-32768 ~ 32767						*/
typedef	unsigned int	INT16U;			/* unsigned 16bits integer, 0 ~ 65535							*/
typedef	signed long		INT32S;			/* signed 32bits integer, 	-2147483648 ~ 2147483647			*/
typedef	unsigned long	INT32U;			/* unsigned 32bits integer,	0 ~ 4294967295						*/

typedef	float			FP32;			/* 32bits floating point,	-1.175494e-38 ~ 3.40282346e+38		*/
typedef	long double		FP64;			/* 64bits floating point,	2.22507385e-308 ~ 1.79769313e+308	*/

/*
***********************************************************
*	Redefines for PORTs
***********************************************************
*/
//Ver 5.0 modifed


/*
***********************************************************
*  Extern Definition of Global Variables
***********************************************************
*/
// etc.
//extern INT8U	chk1, chk2;         //unsigned char

/*
***********************************************************
*  Extern Init. Functions
***********************************************************
*/
//extern void __delay32(unsigned long);

#endif
