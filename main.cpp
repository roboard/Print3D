#include <stdio.h>
#include <stdlib.h>
#include <conio.h>

#include "global_setting.h"
#include "g_code.h"
#include "stepper.h"
#include "temperature.h"
#include "command.h"
#include "communication.h"
#include "Arduino.h"
#include "io.h"

bool init()
{
	if(io_Init() == false) return false;
	
	// set ADC Base Address
	sb_Write(0xBC, sb_Read(0xBC) & (~(1L<<28)));  // active adc
	sb1_Write16(0xDE, sb1_Read16(0xDE) | 0x02);   // not Available for 8051A Access ADC
	sb1_Write(0xE0, 0x00500000UL | ADC_BASEADDR); // baseaddr = 0xfe00, disable irq
	
	return true;
}

int main(int argc, char *argv[])
{
	init();
	if(init_setting() != 0){
		print_errmsg("ERROR: SETTING init fail. \n");
		return 1;	
	}
	if(gc_init() != 0){
		print_errmsg("ERROR: G_CODE init fail. \n");
		return 1;
	}
	if (cm_init() == false)
	{
		print_errmsg("ERROR: USB-DEV init fail. \n");
		return 1;
	}
  
	if (cmd_init() == false)
	{
		print_errmsg("ERROR: G code init fail.\n");
		cm_close();
		return 1;
	}
	
	if (tp_init() == false)
	{
		print_errmsg("ERROR: Temperature library init fail. \n");
		cmd_close();
		cm_close();
		return 1;
	}

	plan_init();
	Config_ResetDefault();
	st_init();

	while(1) {
		LCD(10UL);
		getCommand(10UL);
		process_commands(50UL);
		stepper(1000L);
		manageHeater(10UL);
	}

	st_close();
	plan_close();
	tp_close();
	cmd_close();
	cm_close();
	//debug
	//sfp_close();
	return 0;
}

