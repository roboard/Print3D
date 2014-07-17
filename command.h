#ifndef __COMMAND_H
#define __COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

bool cmd_init(void);
void cmd_close(void);

void getCommand(unsigned long runTime);
char popCommandChr();


#ifdef __cplusplus
}
#endif

#endif
