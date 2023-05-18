// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "../Core/Inc/main.h"

void RkSd_Main();

void wait()
{
  // Ждем перепад 1->0 A5
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 0)
    ;
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 1)
    ;
  uint32_t addr = READ_ADDR ();
  if ((addr & 0x3f) == 0)
    return;
  RkSd_main ();
}

void sendStart(BYTE c)
{
  wait ();
  DATA_BUS_OUT();
  WRITE_DATA(c);
}

void recvStart()
{
  wait ();
  DATA_BUS_IN ();
}

BYTE wrecv()
{
  wait ();
  return READ_DATA();
}

void send(BYTE c)
{
  wait ();
  WRITE_DATA(c);
}
