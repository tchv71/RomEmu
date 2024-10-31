﻿// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

//#include <stdafx.h>

#include "common.h"
#include <string.h>
#include "sd.h"
#include "fs.h"
#include "proto.h"
#include "../Core/Inc/main.h"
#include "stm32f4xx_hal_rtc.h"

#define O_OPEN   0
#define O_CREATE 1
#define O_MKDIR  2
#define O_DELETE 100
#define O_SWAP   101

#define STA_START       0x40
#define STA_WAIT        0x41
#define STA_OK_DISK     0x42
#define STA_OK_CMD      0x43
#define STA_OK_READ     0x44
#define STA_OK_ENTRY    0x45
#define STA_OK_WRITE    0x46
#define STA_OK_RKS      0x47
#define ERR_DATETIME    0x50
#define STA_OK_BLOCK    0x4F

BYTE buf[512];
BYTE rom[128];

/*******************************************************************************
 * Для удобства                                                                 *
 *******************************************************************************/

void recvBin(BYTE *d, WORD l)
{
  for (; l; --l)
  {
    *d++ = wrecv ();
  }
}

void recvString()
{
  BYTE c;
  BYTE *p = buf;
  do
  {
    c = wrecv ();
    if (p != buf + FS_MAXFILE)
      *p++ = c;
    else
      lastError = ERR_RECV_STRING;
  }
  while (c);
}

void sendBin(BYTE *p, WORD l)
{
  for (; l; l--)
    send (*p++);
}

void sendBinf(const BYTE *d, BYTE l)
{
  for (; l; --l)
    send (*d++);
}

/*******************************************************************************
 * Отправка всех блоков файла                                                   *
 *******************************************************************************/

WORD readLength;

void readInt(char rks)
{
  WORD readedLength, lengthFromFile;
  BYTE tmp;
  BYTE *wptr;

  while (readLength)
  {
    // Расчет длины блока (выравниваем чтение на сектор)
    if (fs_tell ())
      return;
    readedLength = 512 - (fs_tmp % 512);
    if (readedLength > readLength)
      readedLength = readLength;

    // Уменьшаем счетчик
    readLength -= readedLength;

    // Читаем блок
    if (fs_read0 (buf, readedLength))
      return;

    // Заголовок RKS файла
    wptr = buf;
    if (rks)
    { // Если rks=1, перед вызовом надо проверить, что бы readLength>4 и fs_file.ptr=0, иначе может быть злостный сбой
      rks = 0;

      // У апогея числа перепутаны
      tmp = buf[0], buf[0] = buf[1];
      buf[1] = tmp;
      tmp = buf[2], buf[2] = buf[3];
      buf[3] = tmp;

      // Посылаем адрес загрузки
      send (STA_OK_RKS);
      sendBin (buf, 2);
      //send (ERR_WAIT);

      // Корректируем указатели
      wptr += 4;
      readedLength -= 4;

      // Длина из файла
      lengthFromFile = *(WORD*) (buf + 2) - *(WORD*) (buf) + 1;

      // Корректируем длину
      if (readedLength > lengthFromFile)
      {
	readedLength = lengthFromFile;
      }
      else
      {
	lengthFromFile -= readedLength;
	if (readLength > lengthFromFile)
	  readLength = lengthFromFile;
      }
    }

    // Отправляем блок
    send (STA_OK_BLOCK);
    sendBin ((BYTE*) &readedLength, 2);
#ifdef USE_DMA
    sendFlush();
    dma_send(wptr, readedLength); //
#else
    sendBin (wptr, readedLength);
#endif
    //send (ERR_WAIT);
  }

  // Если все ОК
  if (!lastError)
    lastError = STA_OK_READ;
}

/*******************************************************************************
 * Версия команд контроллера                                                    *
 *******************************************************************************/

void cmd_ver()
{
  sendStart (1);

  // Версия + Производитель
  {
    const unsigned char * ver = (const unsigned char*)"V1.0 10-05-2014 ";
    sendBinf (ver, 16);
  }
#ifdef USE_DMA
  sendFlush();
#endif
}

/*******************************************************************************
 * BOOT / EXEC                                                                  *
 *******************************************************************************/

void cmd_boot_exec()
{
  // Файл по умолчанию
#ifndef USE_DMA
    const char *bootSdbiosRk = "boot/sdbios.rk";
#else
    const char *bootSdbiosRk = "boot/sdbiosd.rkl";
#endif
    if (buf[0] == 0)
      strcpy((char*) buf, (const char*) (nCS_GPIO_Port->IDR & nCS_Pin) ?  "boota/sdbios.rk" : bootSdbiosRk);

  // Открываем файл
  if (fs_open ())
    return;

  // Максимальный размер файла
  readLength = 0xFFFF;
  if (fs_getfilesize ())
    return;
  if (readLength > fs_tmp)
    readLength = (WORD) fs_tmp;

  // Файлы RK должны быть длиной >4 байт. Мы заносим в readLength = 0 и программа
  // получает ERR_OK. Но так как она ждет ERR_OK_RKS, это будет ошибкой
  if (readLength < 4)
    readLength = 0;

  readInt (/*rks*/1);
}

void cmd_boot()
{
  sendStart (STA_WAIT);
  buf[0] = 0;
  cmd_boot_exec ();
}

void cmd_exec()
{
  // Прием имени файла
  recvString ();

  if ((nCS_GPIO_Port->IDR & nCS_Pin) && stricmp((char*)buf, "BOOT/SHELL.RK") == 0)
    strcpy((char*)buf, "BOOTA/SHELL.RK");
  // Режим передачи и подтверждение
  sendStart (STA_WAIT);
  if (lastError)
    return; // Переполнение строки

  cmd_boot_exec ();
}

/*******************************************************************************
 * Начать/продолжить поиск файлов в папке                                       *
 *******************************************************************************/

typedef struct
{
  char fname[11];    // File name
  BYTE fattrib;    // Attribute
  DWORD fsize;        // File size
  union
  {
    struct
    {
      WORD ftime;        // Last modified time
      WORD fdate;        // Last modified date
    };
    DWORD ftimedate;
  };
} FILINFO2;

void cmd_find()
{
  WORD n = 0;
  FILINFO2 info;

  // Принимаем путь
  recvString ();

  // Принимаем макс кол-во элементов
  recvBin ((BYTE*) &n, 2);

  // Режим передачи и подтверждение
  sendStart (STA_WAIT);
  if (lastError)
    return;

  // Открываем папку
  if (buf[0] != ':')
  {
    if (fs_opendir ())
      return;
  }

  for (; n; --n)
  {
    /* Читаем очереной описатель */
    if (fs_readdir ())
      return;

    /* Конец */
    if (FS_DIRENTRY[0] == 0)
    {
      lastError = STA_OK_CMD;
      return;
    }

    /* Сжимаем ответ для компьютера */
    memcpy (info.fname, FS_DIRENTRY + DIR_Name, 12);
    memcpy (&info.fsize, FS_DIRENTRY + DIR_FileSize, 4);
    memcpy (&info.ftimedate, FS_DIRENTRY + DIR_WrtTime, 4);
    //memcpy(memcpy(memcpy(info.fname, FS_DIRENTRY+DIR_Name, 12, FS_DIRENTRY+DIR_FileSize, 4), FS_DIRENTRY+DIR_WrtTime, 4);

    /* Отправляем */
    send (STA_OK_ENTRY);
    #ifndef USE_DMA
      sendBin ((BYTE*) &info, sizeof(info));
    #else
      sendFlush();
      dma_send((BYTE*) &info, sizeof(info));
    #endif
    //send (ERR_WAIT);
  }

  /* Ограничение по размеру */
  lastError = ERR_MAX_FILES; /*! Надо опеределать, что бы не было ложных ошибок */
}

/*******************************************************************************
 * Открыть/создать файл/папку                                                   *
 *******************************************************************************/

void cmd_open()
{
  BYTE mode;

  /* Принимаем режим */
  mode = wrecv ();

  // Принимаем имя файла
  recvString ();

  // Режим передачи и подтверждение
  sendStart (STA_WAIT);

  // Открываем/создаем файл/папку
  if (mode == O_SWAP)
  {
    fs_swap ();
  }
  else if (mode == O_DELETE)
  {
    fs_delete ();
  }
  else if (mode == O_OPEN)
  {
    fs_open ();
  }
  else if (mode < 3)
  {
    fs_open0 (mode);
  }
  else
  {
    lastError = ERR_INVALID_COMMAND;
  }

  // Ок
  if (!lastError)
    lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Переместить файл/папку                                                       *
 *******************************************************************************/

void cmd_move()
{
  recvString ();
  sendStart (STA_WAIT);
  fs_openany();
  sendStart (STA_OK_WRITE);
  recvStart ();
  recvString ();
  sendStart (STA_WAIT);
  if (!lastError)
    fs_move0 ();
  if (!lastError)
    lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Установить/прочитать указатель чтения                                        *
 *******************************************************************************/

void cmd_lseek()
{
  BYTE mode;
  DWORD off;

  // Принимаем режим и смещение
  mode = wrecv ();
  recvBin ((BYTE*) &off, 4);

  // Режим передачи и подтверждение
  sendStart (STA_WAIT);

  // Размер файла
  if (mode == 100)
  {
    if (fs_getfilesize ())
      return;
  }

  // Размер диска
  else if (mode == 101)
  {
    if (fs_gettotal ())
      return;
  }

  // Свободное место на диске
  else if (mode == 102)
  {
    if (fs_getfree ())
      return;
  }

  else
  {
    /* Устаналиваем смещение. fs_tmp сохраняется */
    if (fs_lseek (off, mode))
      return;
  }

  // Передаем результат
  send (STA_OK_CMD);
  sendBin ((BYTE*) &fs_tmp, 4);
  lastError = 0; // На всякий случай, результат уже передан
}

/*******************************************************************************
 * Получить дату                                                                *
 *******************************************************************************/
extern RTC_HandleTypeDef hrtc;
void cmd_get_date()
{
  sendStart(STA_WAIT);
  RTC_DateTypeDef sDate={0};
  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)!= HAL_OK)
  {
    lastError = ERR_DATETIME;
    return;
  }
  sendBin((BYTE*)&sDate, sizeof(sDate));
  send (STA_OK_CMD);
  //lastError = ERR_OK_CMD;
}

static const uint8_t list_mth[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

uint8_t Calendar_GetDayWeek (RTC_DateTypeDef thisDate){
	uint8_t ret = 0;
    if(thisDate.Month < 3){
    	thisDate.Year -= 1;
    }
    ret = (uint8_t)((thisDate.Year + (thisDate.Year/4) - (thisDate.Year/100) + (thisDate.Year/400) + list_mth[thisDate.Month-1] + thisDate.Date) % 7);
	return (ret);
}
/*******************************************************************************
 * Установить дату                                                              *
 *******************************************************************************/
void cmd_set_date()
{
  RTC_DateTypeDef sDate={0};
  //recvStart();
  sDate.WeekDay = wrecv();
  sDate.Month = wrecv();
  sDate.Date = wrecv();
  sDate.Year = wrecv();
  sDate.WeekDay = Calendar_GetDayWeek(sDate);
  // Режим передачи и подтверждение
  sendStart (STA_WAIT);

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN)!= HAL_OK)
  {
    lastError = ERR_DATETIME;
    return;
  }
  send (STA_OK_CMD);
  //lastError = ERR_OK_CMD;
}

/*******************************************************************************
 * Получить время                                                               *
 *******************************************************************************/
void cmd_get_time()
{
  sendStart(STA_WAIT);
  RTC_TimeTypeDef sTime={0};

  if (HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN)!= HAL_OK)
  {
    lastError = ERR_DATETIME;
    return;
  }
  RTC_DateTypeDef sDate={0};
  if (HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN)!= HAL_OK)
  {
    lastError = ERR_DATETIME;
    return;
  }
  sendBin((BYTE*)&sTime, 3);
  sendBin((BYTE*)&sTime.SecondFraction,1);
  sendBin((BYTE*)&sTime.SubSeconds,1);
  send (STA_OK_CMD);
  lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Установить время                                                               *
 *******************************************************************************/
void cmd_set_time()
{
  RTC_TimeTypeDef sTime={0};
  sTime.Hours = wrecv();
  sTime.Minutes = wrecv();
  sTime.Seconds = wrecv();
  sTime.SecondFraction = wrecv();
  sTime.SubSeconds = wrecv();
  if (sTime.SecondFraction != 255)
  {
    sTime.SubSeconds = sTime.SubSeconds * sTime.SecondFraction / 256;
  }
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN)!= HAL_OK)
  {
    lastError = ERR_DATETIME;
    return;
  }
  send (STA_OK_CMD);
  lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Прочитать из файла                                                           *
 *******************************************************************************/

void cmd_read()
{
  DWORD s;

  // Длина
  recvBin ((BYTE*) &readLength, 2);

  // Режим передачи и подтверждение
  sendStart (STA_WAIT);

  // Ограничиваем длину длиной файла
  if (fs_getfilesize ())
    return;
  s = fs_tmp;
  if (fs_tell ())
    return;
  s -= fs_tmp;

  if (readLength > s)
    readLength = (WORD) s;

  // Отправляем все блоки файла
  readInt (/*rks*/0);
}

/*******************************************************************************
 * Записать данные в файл                                                       *
 *******************************************************************************/

void cmd_write()
{
  // Аргументы
  recvBin ((BYTE*) &fs_wtotal, 2);

  // Ответ
  sendStart (STA_WAIT);

  // Конец файла
  if (fs_wtotal == 0)
  {
    fs_write_eof ();
    lastError = STA_OK_CMD;
    return;
  }

  // Запись данных
  do
  {
    if (fs_write_start ())
      return;

    // Принимаем от компьютера блок данных
    send (STA_OK_WRITE);
    sendBin ((BYTE*) &fs_file_wlen, 2);
    recvStartNoDma ();
    #ifndef USE_DMA
      recvBin (fs_file_wbuf, fs_file_wlen);
    #else
      dma_receive(fs_file_wbuf, fs_file_wlen);
    #endif
    sendStart (STA_WAIT);

    if (fs_write_end ())
      return;
  }
  while (fs_wtotal);

  lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Главная процедура                                                            *
 *******************************************************************************/

void error()
{
  while (1)
  {
    HAL_GPIO_TogglePin (LED13_GPIO_Port, LED13_Pin);
    HAL_Delay (100);
  }
}

void LedOff()
{
  // Гасим светодиод
  HAL_GPIO_WritePin (LED13_GPIO_Port, LED13_Pin, 1);
}
void LedOn()
{
  // Зажигаем светодиод
  HAL_GPIO_WritePin (LED13_GPIO_Port, LED13_Pin, 0);
}

BYTE RkSd_Loop()
{
  BYTE c;
  //while (1)
  {

    // Проверяем наличие карты
    sendStart (STA_START);
    //send (ERR_WAIT);
    //if (fs_check ())
    //{
    //  send (ERR_DISK_ERR);
    //}
    //else
    {
      send (STA_OK_DISK);
      recvStart ();
      c = wrecv ();
      // Зажигаем светодиод
      LedOn ();

      // Сбрасываем ошибку
      lastError = 0;

      // Принимаем аргументы
      switch (c)
	{
	case 0:
	  cmd_boot ();
	  break;
	case 1:
	  cmd_ver ();
	  break;
	case 2:
	  cmd_exec ();
	  break;
	case 3:
	  cmd_find ();
	  break;
	case 4:
	  cmd_open ();
	  break;
	case 5:
	  cmd_lseek ();
	  break;
	case 6:
	  cmd_read ();
	  break;
	case 7:
	  cmd_write ();
	  break;
	case 8:
	  cmd_move ();
	  break;
	case 9:
	  LedOff ();
	  return 0;
	case 0x2A:
	  cmd_get_date();
	  break;
	case 0x2B:
	  cmd_set_date();
	  break;
	case 0x2C:
	  cmd_get_time();
	  break;
	case 0x2D:
	  cmd_set_time();
	  break;
	default:
	  lastError = ERR_INVALID_COMMAND;
	}

      // Вывод ошибки
      if (lastError)
	sendStart (lastError);
      #ifdef USE_DMA
	sendFlush();
      #endif
      LedOff();
    }

    #ifndef USE_DMA
      // Порт работает на выход
      wait ();
      DATA_OUT();
    #endif

    // Гасим светодиод
    LedOff ();
    return 1;
  }
}

#ifdef USE_DMA


BYTE dma_send(const BYTE* ptr, WORD len)
{
  DATA_IN();
  DRQ_Port->ODR = DRQ_Port->ODR | DRQ_Pin; //DRQ = 1;
  WORD count = len;
  __disable_irq();
  do
  {
    while (nIOR_Port->IDR & (nIOR_Pin | nDACK_Pin)) ;
    DATA_OUT();
    WRITE_DATA(*ptr++); //PORTD = *ptr++;
    while ((nIOR_Port->IDR & nIOR_Pin) == 0) ;
    DATA_IN();
  } while (--count);
  DRQ_Port->ODR = DRQ_Port->ODR & ~DRQ_Pin; //DRQ = 0;
  __enable_irq();
  return 0;
}

BYTE dma_receive(BYTE* ptr, WORD len)
{
  DATA_IN();
  DRQ_Port->ODR = DRQ_Port->ODR | DRQ_Pin; //DRQ = 1;
  __disable_irq();
  do
  {
    while (nIOW_Port->IDR & (nIOW_Pin | nDACK_Pin)) ;
    while ((nIOR_Port->IDR & nIOW_Pin) == 0) ;
    *ptr++ = READ_DATA();
  } while (--len);
  DRQ_Port->ODR = DRQ_Port->ODR & ~DRQ_Pin; //DRQ = 0;
  __enable_irq();
  return 0;
}
#endif

void RkSd_main()
{

#ifndef USE_DMA
  DATA_OUT();
#endif
  // Шина данных (DDRD)
  //DDRC = 0b00000000; // Шина адреса
  //DDRB = 0b00101101; // Шина адреса, карта и светодиод
  //PORTB = 0b00010001; // Подтягивающий резистор на MISO и светодиод

  // Пауза, пока не стабилизируется питание
  HAL_Delay (500);

  // Запуск файловой системы
  if (fs_init ())
    error ();
  if(nCS_GPIO_Port->IDR & nCS_Pin)
    strcpy ((char*) buf, "boota/boot.rk");
  else
    strcpy ((char*) buf, "boot/boot.rk");
  if (fs_open ())
    error ();
  if (fs_getfilesize ())
    error ();
  if (fs_tmp < 7)
    error ();
  if (fs_tmp > 128)
    error ();
  WORD rom_size = (WORD)fs_tmp;
  if (fs_read0 (rom, rom_size))
    error ();

  HAL_Delay (100);

  // Гасим светодиод
  LedOff ();
#if 0 //def USE_DMA
  dma_send(rom, rom_size);
  dma_receive(buf, rom_size);
  int n = memcmp(buf, rom, rom_size);
  GPIOB->ODR = n;
  while(1) ;
#endif
  while (1)
  {
    // Эмуляция ПЗУ



    #ifndef USE_DMA
      BYTE lastAddr = 0;
      while (1)
      {
	static uint32_t oldAddr = -1;
	uint32_t addr = READ_ADDR();
	if (oldAddr != addr)
	{
	  uint32_t val = rom[addr & 0x7f];
	  WRITE_DATA(val);
	  oldAddr = addr;
	  if (addr == 0x44)
	  {
	    lastAddr = 0x44;
	  }
	  else if (addr == 0x40)
	  {
	    if (lastAddr == 0x44)
	      lastAddr = 0x40;
	    else
	      lastAddr = 0;
	  }
	  else if (addr == 0)
	  {
	    if (lastAddr == 0x40)
	      break;
	    else
	      lastAddr = 0;
	  }
	}

      }
    #endif
    if (!RkSd_Loop())
      return;
  }
}
