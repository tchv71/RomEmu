// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

#include "proto.h"
#include "../Core/Inc/main.h"

void RkSd_Main();


void wait()
{
#ifndef USE_DMA
  // Ждем перепад 1->0 A5
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 0)
    ;
  while (HAL_GPIO_ReadPin (A5_GPIO_Port, A5_Pin) == 1)
    ;
  uint32_t addr = READ_ADDR ();
  if ((addr & 0x3f) == 0)
    return;
  RkSd_main ();
#endif
}
#ifdef USE_DMA
enum DMA_MODE { DM_NONE = 0, DM_SEND, DM_RECEIVE} dm_mode;
BYTE cmd_buf[32];
BYTE* cmd_buf_ptr;
#endif

void sendStart(BYTE c)
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_OUT();
  WRITE_DATA(c);
#else
  #ifndef USE_PORT
    dm_mode = DM_SEND;
    cmd_buf_ptr = cmd_buf;
    *cmd_buf_ptr++=c;
  #else
    send(c);
  #endif
#endif
}

void recvStartNoDma()
{
  if (dm_mode == DM_SEND && cmd_buf_ptr != cmd_buf)
    sendFlush();
  dm_mode = DM_RECEIVE;
  cmd_buf_ptr = cmd_buf;
}

void recvStart()
{
#ifndef USE_DMA
  wait ();
  DATA_BUS_IN ();
#else
  #ifndef USE_PORT
    recvStartNoDma();
    BYTE len[2];
    dma_receive(len, 2);
    WORD l = (len[1] << 8) + len[0];
    dma_receive(cmd_buf, l);
  #endif
#endif
}

BYTE wrecv()
{
#ifndef USE_DMA
  wait ();
  return READ_DATA();
#else
  #ifndef USE_PORT
    return *cmd_buf_ptr++;
  #else
    while (nCS_MC_Port->IDR & nCS_MC_Pin) ;
    while (nIOW_Port->IDR & nIOW_Pin) ;
    while ((nIOR_Port->IDR & nIOR_Pin)==0) ;
    return READ_DATA();
  #endif
#endif
}

void send(BYTE c)
{
#ifndef USE_DMA
  wait ();
  WRITE_DATA(c);
#else
#ifndef USE_PORT
  *cmd_buf_ptr++=c;
#else
  while ((nCS_MC_Port->IDR & nCS_MC_Pin) != 0/* || (nIOR_Port->IDR & nIOR_Pin)!=0*/) ;
  //while (nIOR_Port->IDR & nIOR_Pin) ;
  DATA_OUT();
  WRITE_DATA(c);
  while ((nIOR_Port->IDR & nIOR_Pin)==0 || (nCS_MC_Port->IDR & nCS_MC_Pin) == 0 ) ;
  DATA_IN();

#endif
#endif
}
void sendFlush()
{
#if defined(USE_DMA) && !defined(USE_PORT)
  WORD l = cmd_buf_ptr - cmd_buf;
  BYTE len[2] = {l&255, l>>8};
  if (l == 0)
    return;
  dma_send(len, 2);
  dma_send(cmd_buf, l);
  cmd_buf_ptr = cmd_buf;
#endif
}
