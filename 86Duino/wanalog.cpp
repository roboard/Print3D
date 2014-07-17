#include <stdio.h>
#include "io.h"
#define USE_COMMON
#include "common.h"
#include "Arduino.h"

#define TimeOut		(1000)

int analogRead(int pin) {
	unsigned long d;
	unsigned long time;
	
	// if((pin > 6 && pin < 45) || pin > 51) return 0xffff;
	
	// #if defined(__86DUINO_EDUCAKE) || defined(__86DUINO_ONE) || defined(__86DUINO_ZERO)
		// if(pin >= 45) pin -= 45;
	// #endif
	
	while((io_inpb(ADC_BASEADDR + 2) & 0x01) != 0)
		io_inpb(ADC_BASEADDR + 4);
	
	io_DisableINT();	
	io_outpb(ADC_BASEADDR + 1, 0x08); // disable ADC
	io_outpb(ADC_BASEADDR + 0, (0x01<<pin));
	io_outpb(ADC_BASEADDR + 1, 0x01); // enable ADC_ST
	
	for(time = timer_nowtime(); (io_inpb(ADC_BASEADDR + 2) & 0x01) == 0;)
	{
		if(timer_nowtime() - time > TimeOut)
			return 0xffff;
	}
	
    d = io_inpw(ADC_BASEADDR + 4);
	io_RestoreINT();
	
	d &= 0x7ffL;
	// d = mapResolution((d&0x7ffL), ADC_RESOLUTION, _readResolution);
	
	return d;
}