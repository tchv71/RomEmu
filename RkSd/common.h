// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#ifndef COMMON_H
#define COMMON_H
#include "../Core/Inc/main.h"
//#include <mega328p.h>

#define CONST
#define USE_DMA 1
//#define USE_PORT 1

#ifdef USE_DMA
#define DRQ_Pin    A2_Pin
#define DRQ_Port   A2_GPIO_Port
#define nDACK_Pin  A3_Pin
#define nDACK_Port A3_GPIO_Port
#define nIOW_Pin   A4_Pin
#define nIOW_Port  A4_GPIO_Port
#define nIOR_Pin   A5_Pin
#define nIOR_Port  A5_GPIO_Port
#define nCS_MC_Pin    A6_Pin
#define nCS_MC_Port   A6_GPIO_Port
#endif

typedef unsigned char   BYTE;
typedef unsigned short  WORD;
typedef unsigned long	DWORD;

#endif
