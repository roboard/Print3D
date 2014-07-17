#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#define NO_LINENUMBER_WITH_CHECKSUM  (0)
#define NO_CHECKSUM                  (1)
#define CHECKSUM_MISMATCH            (2)
#define LINE_NO                      (3)

#ifdef __cplusplus
extern "C" {
#endif

void report_parser_error(int error, long line);
void report_M105_info();
void report_M114_info();
void report_M115_info();
void report_M119_info();
void request_to_send(long line);
void response_ack();

#ifdef __cplusplus
}
#endif

#endif
