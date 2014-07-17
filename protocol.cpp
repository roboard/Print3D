#define __PROTOCOL_LIB

#include "protocol.h"
#include "communication.h"
#include "global_setting.h"
#include "temperature.h"
#include "stepper.h"

void report_parser_error(int error, long line)
{
  COMM_WRITE("Error:");
  switch (error)
  {
  case NO_LINENUMBER_WITH_CHECKSUM:
    COMM_WRITE("No Line Number with checksum, Last Line: ");
    break;
  case NO_CHECKSUM:
    COMM_WRITE("No Checksum with line number, Last Line: ");
    break;
  case CHECKSUM_MISMATCH:
    COMM_WRITE("checksum mismatch, Last Line: ");
    break;
  case LINE_NO:
    COMM_WRITE("Line Number is not Last Line Number+1, Last Line: ");
    break;
  default:
    COMM_WRITE("Undefined error, Last Line: ");
    break;
  }
  COMM_WRITE_LONG(line);
  COMM_WRITE("\n");
}

void report_M105_info()
{
  int e;
  
  COMM_WRITE(" T:");
  COMM_WRITE_FLOAT(getExtruderTemp(EXTRUDER0));
  COMM_WRITE(" /");
  COMM_WRITE_FLOAT(getExtruderTargetTemp(EXTRUDER0));
  COMM_WRITE(" B:");
  COMM_WRITE_FLOAT(getHotedBedTemp());
  COMM_WRITE(" /");
  COMM_WRITE_FLOAT(getHotedBedTargetTemp());
  
  for (e = 0; e < tp->extruders; e++)
  {
    COMM_WRITE(" T");
    COMM_WRITE_LONG(e);
	COMM_WRITE(":");
    COMM_WRITE_FLOAT(getExtruderTemp(e));
	COMM_WRITE(" /");
    COMM_WRITE_FLOAT(getExtruderTargetTemp(e));
  }
  COMM_WRITE(" @:");
  COMM_WRITE_FLOAT(getHotedBedTemp());
  COMM_WRITE(" B@:");
  COMM_WRITE_FLOAT(getHotedBedTargetTemp());
  
  COMM_WRITE("    ADC");
  COMM_WRITE(" B:");
  COMM_WRITE_FLOAT(getHotedBedTemp());
  COMM_WRITE("C->");
  COMM_WRITE_FLOAT(getHotedBedTempRawValue());
	
  COMM_WRITE(" ");
  COMM_WRITE(" T");
  COMM_WRITE_LONG(EXTRUDER0);
  COMM_WRITE(":");
  COMM_WRITE_FLOAT(getExtruderTemp(EXTRUDER0));
  COMM_WRITE("C->");
  COMM_WRITE_FLOAT(getExtruderTempRawValue(EXTRUDER0));
  COMM_WRITE("\n");
}

void report_M114_info()
{
	long x_count = st_get_position(X_AXIS);
	long y_count = st_get_position(Y_AXIS);
	long z_count = st_get_position(Z_AXIS);
	long e_count = st_get_position(E_AXIS);
	COMM_WRITE(" MOTOR COUNT X:");
	COMM_WRITE_LONG(x_count);
	COMM_WRITE(",   ");
	COMM_WRITE("Y:");
	COMM_WRITE_LONG(y_count);
	COMM_WRITE(",   ");
	COMM_WRITE("Z:");
	COMM_WRITE_LONG(z_count);
	COMM_WRITE(",   ");
	COMM_WRITE("E:");
	COMM_WRITE_LONG(e_count);
	double x_pos = st_get_position_mm(X_AXIS);
	double y_pos = st_get_position_mm(Y_AXIS);
	double z_pos = st_get_position_mm(Z_AXIS);
	COMM_WRITE(" \nMACHINE POSITION X:");
	COMM_WRITE_FLOAT(x_pos);
	COMM_WRITE(",   ");
	COMM_WRITE("Y:");
	COMM_WRITE_FLOAT(y_pos);
	COMM_WRITE(",   ");
	COMM_WRITE("Z:");
	COMM_WRITE_FLOAT(z_pos);
	COMM_WRITE("\n");
}

void report_M115_info()
{
  COMM_WRITE("My 3D printer firmware.\n");
}

void report_M119_info()
{
	if(Check_limit(X_AXIS) == 1){
		COMM_WRITE("X LIMIT IS TRIGGERED\n");
	}else{
		COMM_WRITE("X LIMIT IS OPEN\n");
	}
	if(Check_limit(Y_AXIS) == 1){
		COMM_WRITE("Y LIMIT IS TRIGGERED\n");
	}else{
		COMM_WRITE("Y LIMIT IS OPEN\n");
	}
	if(Check_limit(Z_AXIS) == 1){
		COMM_WRITE("Z LIMIT IS TRIGGERED\n");
	}else{
		COMM_WRITE("Z LIMIT IS OPEN\n");
	}

  // COMM_WRITE("x_min: ");
  // COMM_WRITE("open\n"); // "open" or "TRIGGERED"
  // COMM_WRITE("y_min: ");
  // COMM_WRITE("open\n"); // "open" or "TRIGGERED"
  // COMM_WRITE("z_min: ");
  // COMM_WRITE("open\n"); // "open" or "TRIGGERED"
  // TODO
}

void request_to_send(long line)
{
  COMM_WRITE("Resend: ");
  COMM_WRITE_LONG(line);
  COMM_WRITE("\n");
  FlushReceiveBuffer();
}

void response_ack()
{
  COMM_WRITE("ok\n");
}
