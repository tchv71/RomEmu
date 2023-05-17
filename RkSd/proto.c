// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "../Core/Inc/main.h"

void
RkSd_Main();

void wait()
{
  // Ждем перепад 1->0 A5
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 0)
    ;
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 1)
    ;
  //return;
  uint32_t pa = GPIOA->IDR;
  uint32_t pb = GPIOB->IDR;
  //               D7,D6                D1                D0                      D5-D2
  //               B7,B6                B0                A7                      A4-A1
  uint32_t addr = (pb & 0b11000000) | ((pb & 1) << 1) | ((pa & 0b10000000) >> 7)
      | ((pa & 0b11110) << 1);
  if ((addr & 0x3f) == 0)
    return;
  RkSd_main ();
}

void sendStart(BYTE c)
{
  //DATA_BUS_OUT();
  wait ();
  uint32_t v = 0b01010101000001010000000000010100;
  uint32_t m = 0b11111111000011110000000000111100;
  GPIOB->MODER = (GPIOB->MODER & ~m) | v;
  GPIOB->ODR = ((c & 0b11110011) << 8) | ((c & 0b1100) >> 1);
}

void recvStart()
{
  wait ();
  //DATA_BUS_IN ();
  uint32_t m = 0b11111111000011110000000000111100;
  GPIOB->MODER = (GPIOB->MODER & ~m);
  //PORTD = 0xFF;
}

BYTE wrecv()
{
  wait ();
  uint32_t idr = GPIOB->IDR;
  return ((idr >> 8) & 0b11110011) | ((idr & 0b110) << 1);
}

void send(BYTE c)
{
  wait ();
  GPIOB->ODR = ((c & 0b11110011) << 8) | ((c & 0b1100) >> 1);
}
