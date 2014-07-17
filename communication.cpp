#define __COMMUNICATION_LIB

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "io.h"
#include "com.h"

#include "communication.h"

static COMPort *Serial;

bool cm_init(void)
{
  if (io_Init() == false)
    return false;
  sb_Write(0xc0, sb_Read(0xc0) & 0x7fffffffL | ((unsigned long)1UL << 31));
  io_Close();
  
  //com_SetUSBPins(2, 0, 2, 1);
  if ((Serial = com_Init(COM4)) == NULL)
    return false;
  com_SetTimeOut(Serial, 0);
  com_SetBPS(Serial, COM_UARTBAUD_1000000BPS);
  com_SetFormat(Serial, BYTESIZE8 + STOPBIT1 + NOPARITY);
  com_SetFlowControl(Serial, NO_CONTROL);
  com_EnableFIFO(Serial, FIFO_032);
  return true;
}

void cm_close(void)
{
  com_Close(Serial);
}

int commandAvailable(void)
{
  return com_QueryRxQueue(Serial);
}

int readCommandChr(void)
{
  int c = com_Read(Serial);
  return (c == 0xFFFF) ? (-1) : (c);
}

int writeString(const char *str)
{
  return com_Send(Serial, (unsigned char*)str, strlen(str));
}

int writeLong(long val)
{
  char str[32];
  
  sprintf(str, "%ld", val);
  return com_Send(Serial, (unsigned char*)str, strlen(str));
}

int writeFloat(double val)
{
  char str[32];
  
  sprintf(str, "%.2f", val);
  return com_Send(Serial, (unsigned char*)str, strlen(str));
}

void FlushReceiveBuffer(void)
{
  com_ClearRFIFO(Serial);
  com_FlushRxQueue(Serial);
}
