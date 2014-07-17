#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define COMM_WRITE(v)       writeString((const char*)v)
#define COMM_WRITE_LONG(v)  writeLong(v)
#define COMM_WRITE_FLOAT(v) writeFloat(v)

#ifdef __cplusplus
extern "C" {
#endif

bool cm_init(void);
void cm_close(void);

int commandAvailable(void);
int readCommandChr(void);

int writeString(const char *str);
int writeLong(long val);
int writeFloat(double val);

void FlushReceiveBuffer(void);

#ifdef __cplusplus
}
#endif

#endif
