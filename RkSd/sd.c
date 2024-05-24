/*
 It is an open source software to implement SD routines to
 small embedded systems. This is a free software and is opened for education,
 research and commercial developments under license policy of following trems.

 (C) 2013 vinxru (aleksey.f.morozov@gmail.com)

 It is a free software and there is NO WARRANTY.
 No restriction on use. You can use, modify and redistribute it for
 personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
 Redistributions of source code must retain the above copyright notice.

 Version 0.99 5-05-2013
 */
#include "../Core/Inc/main.h"
#include "common.h"
//#include <delay.h>
#include "sd.h"
#include "fs.h"

BYTE sd_sdhc; /* Используется SDHC карта */

/**************************************************************************
 *  Протокол SPI для ATMega8                                               *
 *  Может отличаться для разных МК.                                        *
 **************************************************************************/

/* Куда подключена линия CS карты */
#define SD_CS_ENABLE    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 0) //GPIO_RX PORTB &= ~0x04;
#define SD_CS_DISABLE   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, 1) //PORTB |= 0x04;

/* Совместимость с разными версиями CodeVisionAVR */
#ifndef SPI2X
#define SPI2X 0
#endif

#define SPI_INIT      {}// SPCR = 0x52; SPSR = 0x00; }
#define SPI_HIGHSPEED {}// SPCR = 0x50; SPSR |= (1<<SPI2X); delay_ms(1); }
#define SD_INI_SPEED	6 	/* скорость SPI при инициализации карты:
									0 - FCLK/2,
									1 - FCLK/4,
									2 - FCLK/8,
									3 - FCLK/16,
									4 - FCLK/32,
									5 - FCLK/64,
									6 - FCLK/128,
									7 - FCLK/256    */
#define SD_MAX_SPEED	1 	/* скорость SPI после инициализации карты */

extern SPI_HandleTypeDef hspi1;
#define SPI_SD hspi1.Instance

#if 0
static void spi_transmit(BYTE data) {
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
}

static BYTE spi_receive() {
	BYTE val;
	HAL_SPI_Receive(&hspi1, &val, 1, 1000);
	return val;
}
#else
static BYTE spi_rw(BYTE wval)
{
  SPI_SD->DR = wval;
  while (!(SPI_SD->SR & SPI_SR_RXNE))
    ;
  return SPI_SD->DR;
}

#define spi_transmit(dat)		 spi_rw(dat)
#define spi_receive()			 spi_rw(0xFF)
#endif

//скорость интерфейса SPI
//speed от 0 (FCLK/2) до 7 (FCLK/256)
static void set_sd_interface_speed(uint8_t speed)
{
  if (speed > 7)
    speed = 7;
  SPI_SD->CR1 &= ~SPI_CR1_SPE; //SPI отключено
  SPI_SD->CR1 &= ~(0x07UL << (3U)); //маска бит скорости
  SPI_SD->CR1 |= (uint32_t) (speed << (3U));
  SPI_SD->CR1 |= SPI_CR1_SPE; // SPI enable
}

/**************************************************************************
 *  Отправка команды                                                       *
 **************************************************************************/

/* Используемые команды SD карты */

#define GO_IDLE_STATE      (0x40 | 0 )
#define SEND_IF_COND       (0x40 | 8 )
#define READ_SINGLE_BLOCK  (0x40 | 17)
#define WRITE_SINGLE_BLOCK (0x40 | 24)
#define SD_SEND_OP_COND    (0x40 | 41)
#define APP_CMD            (0x40 | 55)
#define READ_OCR           (0x40 | 58)
enum SD_results
{

  R1_READY_STATE = 0x00, R1_IDLE_STATE = 0x01, R1_ILLEGAL_COMMAND = 0x04

};
static BYTE sd_sendCommand(BYTE cmd, DWORD arg)
{
  BYTE response, retry;

  /* Размещение этого кода тут -4 команды, хотя вроде лишние проверки */
  if (sd_sdhc == 0 && (cmd == READ_SINGLE_BLOCK || cmd == WRITE_SINGLE_BLOCK))
    arg <<= 9;

  /* Выбираем карту */
  SD_CS_ENABLE;

  /* Заголовок команды */
  spi_transmit(cmd);
  spi_transmit(((BYTE* ) &arg)[3]);
  spi_transmit(((BYTE* ) &arg)[2]);
  spi_transmit(((BYTE* ) &arg)[1]);
  spi_transmit(((BYTE* ) &arg)[0]);

  /* Пару команд требуют CRC. Остальные же команды игнорируют его, поэтому упрощаем код */
  spi_transmit(cmd == SEND_IF_COND ? 0x87 : 0x95);

  /* Ждем подтвреждение (256 тактов) */
  retry = 0;
  while ((response = spi_receive()) & 0x80)
    if (++retry == 0)
      break;

  if (cmd == SEND_IF_COND)
  {
    spi_receive();
    spi_receive();
    spi_receive();
    spi_receive();
  }
  /* Результат команды READ_OCR обрабатываем тут, так как в конце этой функции мы снимем CS и пропускаем 1 байт */
  if (response == 0 && cmd == READ_OCR)
  {
    /* 32 бита из которых нас интересует один бит */
    sd_sdhc = spi_receive() & 0x40;
    spi_receive();
    spi_receive();
    spi_receive();
  }

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE;
  spi_receive();

  return response;
}

/**************************************************************************
 *  Проверка готовности/наличия карты                                      *
 **************************************************************************/

BYTE sd_check()
{
  BYTE i = 0;
  do
  {
    sd_sendCommand (APP_CMD, 0);
    BYTE res = sd_sendCommand (SD_SEND_OP_COND, 0x40000000);
    if (res == 0)
      return 0;
  }
  while (--i);
  return 1;
}

/**************************************************************************
 *  Инициализация карты (эта функция вызывается функцией sd_init)          *
 **************************************************************************/

static BYTE sd_init_int()
{
  BYTE i;
  set_sd_interface_speed (SD_INI_SPEED); //медленное spi

  /* Сбрасываем SDHC флаг */
  sd_sdhc = 0;

  /* Минимум 80 пустых тактов */
  for (i = 10; i; --i)
    spi_transmit(0xFF);

  /* CMD0 Посылаем команду сброса */
  if (sd_sendCommand (GO_IDLE_STATE, 0) != 1)
    goto abort;

  /* CMD8 Узнаем версию карты */
  i = 0;
  BYTE res = sd_sendCommand (SEND_IF_COND, 0x000001AA);
  if (res)
    i = 1;

  /* CMD41 Ожидание окончания инициализации */
  if (sd_check ())
    goto abort;

  /* Только для второй версии карты */
  if (i)
  {
    /* CMD58 определение SDHC карты. Ответ обрабатывается в функции sd_sendCommand */
    if (sd_sendCommand (READ_OCR, 0) != 0)
      goto abort;
  }

  return 0;
abort: return 1;
}

/**************************************************************************
 *  Инициализация карты                                                    *
 **************************************************************************/

BYTE sd_init()
{
  BYTE tries;

  /* Освобождаем CS на всякий случай */
  SD_CS_DISABLE;

  /* Включаем SPI */
  SPI_INIT

  /* Делаем несколько попыток инициализации */
  tries = 10;
  while (sd_init_int ())
    if (--tries == 0)
    {
      lastError = ERR_DISK_ERR;
      return 1;
    }

  /* Включаем максимальную скорость */
  set_sd_interface_speed (SD_MAX_SPEED);

  return 0;
}

/**************************************************************************
 *  Ожидание определенного байта на шине                                   *
 **************************************************************************/

static BYTE sd_waitBus(BYTE byte)
{
  WORD retry = 0;
  do
  {
    if (spi_receive() == byte)
      return 0;
  }
  while (++retry);
  return 1;
}

/**************************************************************************
 *  Чтение произвольного участка сектора                                   *
 **************************************************************************/

BYTE sd_read(BYTE *buffer, DWORD sector, WORD offsetInSector, WORD length)
{
  BYTE b;
  WORD i;

  /* Посылаем команду */
  if (sd_sendCommand (READ_SINGLE_BLOCK, sector))
    goto abort;

  /* Сразу же возращаем CS, что бы принять ответ команды */
  SD_CS_ENABLE;

  /* Ждем стартовый байт */
  if (sd_waitBus (0xFE))
    goto abort;

  /* Принимаем 512 байт */
  for (i = 512; i; --i)
  {
    b = spi_receive();
    if (offsetInSector)
    {
      offsetInSector--;
      continue;
    }
    if (length == 0)
      continue;
    length--;
    *buffer++ = b;
  }

  /* CRC игнорируем */
  spi_receive();
  spi_receive();

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE;
  spi_receive();

  /* Ок */
  return 0;

  /* Ошибка и отпускаем CS.*/
abort:
  SD_CS_DISABLE;
  lastError = ERR_DISK_ERR;
  return 1;
}
#include <string.h>

#define WB_size 64

struct WB_Rec
{
  ;BYTE bWrite;
  DWORD dwSector;
  BYTE buf[512];
};
struct WB_Rec WB_RecBuf[WB_size];

volatile BYTE WB_start=0;
volatile BYTE WB_end=0;

void InitWB_Ring()
{
  WB_start = WB_end = 0;
}

uint8_t rb_write_rec()
{
  if (WB_start == WB_end)
    return 0; // Nothing to write
  sd_write512(WB_RecBuf[WB_start].buf, WB_RecBuf[WB_start].dwSector);
  WB_start = (WB_start+1)%WB_size;
  return 1;
}

BYTE sd_write512_Buf(BYTE *buffer, DWORD sector)
{
  if ((WB_end+1)%WB_size == WB_start)
  {
    // No more space in ring buffer - write pending record and add current to end
    rb_write_rec();
  }
  memcpy(WB_RecBuf[WB_end].buf, buffer, 512);
  WB_RecBuf[WB_end].dwSector = sector;
  WB_end = (WB_end+1)%WB_size;
  return 0;
}
/**************************************************************************
 *  Запись сектора (512 байт)                                              *
 **************************************************************************/

BYTE sd_write512(BYTE *buffer, DWORD sector)
{
  WORD n;

  /* Посылаем команду */
  if (sd_sendCommand (WRITE_SINGLE_BLOCK, sector))
    goto abort;

  /* Сразу же возращаем CS, что бы отправить блок данных */
  SD_CS_ENABLE;

  /* Посылаем стартовый байт */
  spi_transmit(0xFE);

  /* Данные */
  for (n = 512; n; --n)
    spi_transmit(*buffer++);

  /* CRC игнорируется */
  spi_transmit(0xFF);
  spi_transmit(0xFF);

  /* Ответ МК */
  if ((spi_receive() & 0x1F) != 0x05)
    goto abort;

  /* Ждем окончания записи, т.е. пока не освободится шина */
  if (sd_waitBus (0xFF))
    goto abort;

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE;
  spi_receive();

  /* Ок */
  return 0;

  /* Ошибка.*/
abort:
  SD_CS_DISABLE;
  lastError = ERR_DISK_ERR;
  return 1;
}
