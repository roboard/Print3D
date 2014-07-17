#define __COMMAND_LIB

#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <ctype.h>

#define USE_COMMON
#include "common.h"

#include "global_setting.h"
#include "communication.h"
#include "protocol.h"
#include "command.h"

static char *cmdBuffer = NULL;
static long  cmdHead = 0L;
static long  cmdTail = 0L;
static long  cmdTailOut = 0L;
static long  cmdSize = 0L;
static long  cmdLineNumber = 0L;
static bool  cmdCommentDetected = false;
static int   cmdWaitAck = 0;

static __inline__ bool cmdBuffferFull(void)
{
  if (((cmdTail + 1) % cmdSize) != cmdHead)
    return false;
	
  return true;
}

static __inline__ bool cmdBuffferNearFull(void)
{
  long res;
  
  if (cmdHead > cmdTail)
    res = cmdHead - (cmdTail + 1);
  else
    res = (cmdHead + cmdSize) - (cmdTail + 1);

  if (res > sys->cmd_buffer_near_full)
    return false;
	
  return true;
}

static __inline__ bool cmdBuffferEmpty(void)
{
  return (cmdTail == cmdHead) ? (true) : (false);
}

static __inline__ long cmdBufferChr(long offset, int c)
{
  const char ch = c;

  while (cmdBuffer[offset] != ch) {
  
    if (cmdBuffer[offset] == '\0')
	  return -1L;
	  
    offset = (offset + 1) % cmdSize;
  }
  
  return offset;
}

static __inline__ long cmdBufferStr(long offset, const char *str)
{
  int i;
  long tmp;
  
  if (str[0] == '\0')
    return -1L;

  while (cmdBuffer[offset] != '\0') {
  
    tmp = offset;
	i = 0;
	
	while (cmdBuffer[tmp] == str[i]) {
	
	  i++;
	  tmp = (tmp + 1) % cmdSize;
	  
	  if (str[i] == '\0')
	    return offset;
	}
	offset = (offset + 1) % cmdSize;
  }
  
  return -1L;
}

static __inline__ long parseLongValue(long offset)
{
  long result, sign;
  
  while (isspace(cmdBuffer[offset]))
  {
    offset = (offset + 1) % cmdSize;
  }
  
  if (cmdBuffer[offset] == '-')
  {
    sign = -1L;
	offset = (offset + 1) % cmdSize;
  }
  else
  {
    sign = 1L;
	if (cmdBuffer[offset] == '+')
	  offset = (offset + 1) % cmdSize;
  }
  
  result = 0L;
  while (isdigit(cmdBuffer[offset]))
  {
    result = 10 * result + (cmdBuffer[offset] - '0');
    offset = (offset + 1) % cmdSize;
  }
  
  return sign * result;
}

static __inline__ bool parseCommand(void)
{
  long pNCode, pCheck;
  
  pNCode = cmdBufferChr(cmdTailOut, 'N');
  pCheck = cmdBufferChr(cmdTailOut, '*');
  
  if (pNCode == -1L && pCheck != -1L)
  {
	report_parser_error(NO_LINENUMBER_WITH_CHECKSUM, cmdLineNumber);
    return false;
  }
  
  if (pNCode != -1L && pCheck == -1L)
  {
	report_parser_error(NO_CHECKSUM, cmdLineNumber);
    return false;
  }
  
  if (pNCode != -1L && pCheck != -1L)
  {
	long LineNumber;
    unsigned char checksum = 0;
	
	pCheck = cmdTailOut;
	while (cmdBuffer[pCheck] != '*')
	{
      checksum ^= cmdBuffer[pCheck];
      pCheck = (pCheck + 1) % cmdSize;
    }
	  
	if (checksum != parseLongValue((pCheck + 1) % cmdSize))
	{
	  report_parser_error(CHECKSUM_MISMATCH, cmdLineNumber);
	  return false;
	}
	
	LineNumber = parseLongValue((pNCode + 1) % cmdSize);
	if (LineNumber != (cmdLineNumber + 1) && cmdBufferStr(cmdTailOut, "M110") == -1L)
	{
	  report_parser_error(LINE_NO, cmdLineNumber);
	  return false;
	}
	
	cmdLineNumber = LineNumber;
  }

  return true;
}

static __inline__ long skipUnusedCode(void)
{
  char ch;
  long tail, tailOut;
  bool checksumDetected, spaceSkiped;
  
  tail = tailOut = cmdTailOut;
  checksumDetected = spaceSkiped = false;
  
  while ((ch = cmdBuffer[tail]) != '\0')
  {
	tail = (tail + 1) % cmdSize;

	if (checksumDetected)
	{
	  if (!spaceSkiped && !isspace(ch))
	    spaceSkiped = true;
		
	  if (spaceSkiped && !isdigit(ch))
	  {
	    checksumDetected = spaceSkiped = false;
	  }
	}
	
	if (checksumDetected)
	{}
	else if (ch == '*')
	{
	  checksumDetected = true;
	}
	else if (ch >= 'a' && ch <= 'z')
	{
	  cmdBuffer[tailOut] = ch - 'a' + 'A';
	  tailOut = (tailOut + 1) % cmdSize;
	}
	else if (ch > ' ')
	{
	  cmdBuffer[tailOut] = ch;
	  tailOut = (tailOut + 1) % cmdSize;
	}
	else if (ch <= ' ')
	{
	  cmdBuffer[tailOut] = ' ';
	  tailOut = (tailOut + 1) % cmdSize;
	}
  }
  cmdBuffer[tailOut] = '\0';
  tailOut = (tailOut + 1) % cmdSize;
  
  return tailOut;
}

static __inline__ void execute_M110(void)
{
	long LineNumber = 1L;
	long pNCode = cmdBufferChr(cmdTailOut, 'N');
	
	if (pNCode != -1)
	  LineNumber = parseLongValue((pNCode + 1) % cmdSize);
	  
	cmdLineNumber = LineNumber;
}

static __inline__ bool reportInfo(void)
{
  long mNumber;
  long pMCode = cmdBufferChr(cmdTailOut, 'M');
  
  if (pMCode == -1L)
    return false;

  mNumber = parseLongValue((pMCode + 1) % cmdSize);
  switch (mNumber)
  {
  case 110: execute_M110(); break;
  case 105: report_M105_info(); break;
  case 114: report_M114_info(); break;
  case 115: report_M115_info(); break;
  case 119: report_M119_info(); break;
  default: return false;
  }
  
  return true;
}

bool cmd_init()
{
  if ((cmdBuffer = (char*)malloc(sizeof(char)*sys->cmd_buffer_size)) == NULL)
    return false;
  
  cmdHead = 0L;
  cmdTail = 0L;
  cmdTailOut = 0L;
  cmdSize = sys->cmd_buffer_size;
  cmdLineNumber = 0L;
  cmdCommentDetected = false;
  
  return true;
}

void cmd_close(void)
{
  if (cmdBuffer != NULL)
    free(cmdBuffer);
}

void getCommand(unsigned long runTime)
{
  int ch;
  unsigned long preTime = getclocks();
  
  do {
    while (cmdWaitAck > 0 && !cmdBuffferNearFull()) {
		response_ack();
		cmdWaitAck--;
	}

    if (commandAvailable() <= 0)
	  break;
	ch = readCommandChr();

	if (ch == '\n' || ch == '\t' || (ch == ':' && !cmdCommentDetected))
	{
	  cmdCommentDetected = false;
	  
	  if (cmdTail == cmdTailOut)
	    continue;
	
	  cmdBuffer[cmdTail] = '\0';
	  cmdTail = (cmdTail + 1) % cmdSize;
	  
	  if (parseCommand() == false)
		request_to_send(cmdLineNumber + 1);
	  else if (reportInfo() == false)
        cmdTailOut = skipUnusedCode();
	  
	  cmdTail = cmdTailOut;
	  if (cmdBuffferNearFull())
	    cmdWaitAck++;
	  else
	    response_ack();
	}
	else if (ch == ';')
	  cmdCommentDetected = true;
	else if (cmdCommentDetected == false)
	{
	  cmdBuffer[cmdTail] = ch;
	  cmdTail = (cmdTail + 1) % cmdSize;
	}
  } while (getclocks() - preTime < runTime);
}

char popCommandChr()
{
  char ch;
  
  if (cmdHead == cmdTailOut)
    return -1;
	
  ch = cmdBuffer[cmdHead];
  cmdHead = (cmdHead + 1) % cmdSize;
  
  return ch;
}
