#ifndef stepper_h
#define stepper_h

#include "global_setting.h"

// Initialize and start the stepper motor subsystem
void st_init(void);

// Set current position in steps
void st_set_position(const long &x, const long &y, const long &z, const long &e);
void st_set_e_position(const long &e);

// Get current position in steps
long st_get_position(unsigned char axis);

void stepper(unsigned long clock); 
void st_close(void);
void LCD(unsigned long clock);
int Check_limit(unsigned char axis);
bool st_buffer_null();
//debug
//void sfp_close();

// Get current position in mm
double st_get_position_mm(unsigned char axis);

//
extern system_t *sys;
extern machine_t *machine;
extern pins_t *pins;
//

#endif
