// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

// Based on sources CC Dharmani, Chennai (India)
// 1 May 2013 vinxru

#include "spi.h"
#include "../Core/Inc/main.h"

//#include <delay.h>

#ifndef SPI2X
#define SPI2X 0
#endif
         
#if 1
extern 	SPI_HandleTypeDef hspi1;

void spi_init(void)
{
}
                    
void spi_transmit(BYTE data)
{
	HAL_SPI_Transmit(&hspi1, &data, 1,100);
}

BYTE spi_receive()
{
	BYTE val;
	HAL_SPI_Receive(&hspi1, &val, 1, 100);
	return val;
}

void spi_highSpeed()
{
}
#endif
