#ifndef __ERR_MSG_H
#define __ERR_MSG_H

#include "stdarg.h"

#ifdef __cplusplus
extern "C" {
#endif

static void print_errmsg(char *fmt, ...){
		va_list args;
		char buffer[512];

		va_start(args, fmt);
		vsprintf(buffer, fmt, args);
		va_end(args);	
		//TODO:
		printf("%s\n", buffer);
}

#ifdef __cplusplus
}
#endif

#endif

