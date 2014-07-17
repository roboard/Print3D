#ifndef g_code_h
#define g_code_h

// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include "planner.h"
#include "stepper.h"
#include "global_setting.h"

void process_commands(unsigned long clock);
void Config_ResetDefault();

int gc_init(void);

//
extern system_t *sys;
extern machine_t *machine;
extern pins_t *pins;
//

#endif
