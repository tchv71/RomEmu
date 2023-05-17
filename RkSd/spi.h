// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

// Based on sources CC Dharmani, Chennai (India)
// 1 May 2013 vinxru

#ifndef _SPI_ROUTINES_H_
#define _SPI_ROUTINES_H_

#include "common.h"

void spi_init();
void spi_transmit(BYTE);
BYTE spi_receive();
void spi_highSpeed();

#endif
