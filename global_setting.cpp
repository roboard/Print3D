#include "global_setting.h"
#include "ini.h"

system_t *sys;
machine_t *machine;
temperature_t *tp;
pins_t *pins;
autoleveling_t *level;
//TODO:

static __inline__ void load_default(void) {
	sys->cmd_buffer_size = 20*1024*1024UL;
	sys->cmd_buffer_near_full = 64*256UL;
	sys->block_buffer_size = 256;
	sys->block_buffer_bound = 16;
	sys->max_cmd_size = 128;
	sys->stepper_buffer_size = 1024;
	sys->stepper_buffer_bound = 20;
	sys->min_step_pulse_duty = 250; //2.5us
	sys->default_pulse_period = 100000; //1ms
	sys->default_pulse_duty = 50000; //0.5ms
	sys->hit_pulse_period = 10000; //100us
	sys->hit_pulse_duty = 5000; //50uss

	machine->min_software_limit = 1;
	machine->max_software_limit = 1;
	machine->type = H_BOT;
	machine->num_axis = 4;
	machine->x_max_pos_mm = 125;
	machine->x_min_pos_mm = 0;
	machine->x_home_pos_mm = 0;
	machine->y_max_pos_mm = 75;
	machine->y_min_pos_mm = 0;
	machine->y_home_pos_mm = 75;
	machine->z_max_pos_mm = 130;
	machine->z_min_pos_mm = 0;
	machine->z_home_pos_mm = 0;
	machine->x_home_retract_mm = 5;
	machine->y_home_retract_mm = 5;
	machine->z_home_retract_mm = 1;
	machine->x_home_dir = -1;
	machine->y_home_dir = 1;
	machine->z_home_dir = -1;
	machine->qx_steps_per_mm = 40;
	machine->qy_steps_per_mm = 40;
	machine->qz_steps_per_mm = 1287;
	machine->qe_steps_per_mm = 47.99;
	machine->x_max_feedrate_mps = 500;
	machine->y_max_feedrate_mps = 500;
	machine->z_max_feedrate_mps = 50;
	machine->e_max_feedrate_mps = 25;
	machine->x_homing_feedrate_mpm = 50;
	machine->y_homing_feedrate_mpm = 50;
	machine->z_homing_feedrate_mpm = 4;
	machine->qx_max_acceleration = 9000;
	machine->qy_max_acceleration = 9000;
	machine->qz_max_acceleration = 20;
	machine->qe_max_acceleration = 10000;
	machine->default_acceleration = 3000;
	machine->default_retract_acceleration = 3000;
	machine->default_minimumfeedrate = 0.0;
	machine->default_mintravelfeedrate = 0.0;
	machine->default_xyjerk = 20.0;
	machine->default_ejerk = 5.0;	
	machine->default_zjerk = 0.4;
	machine->minimum_planner_speed = 0.5;
	machine->step_pulse_duty_ratio = 0.5;
	machine->mcm_samplecycle = 5;
	machine->mm_per_arc_segment = 1;
	machine->n_arc_correction = 25;
	machine->home_offset[0] = 0.0;
	machine->home_offset[1] = 0.0;
	machine->home_offset[2] = 0.0;
	
	pins->mc_qx = 0;
	pins->md_qx = 0;
	pins->mc_qy = 0;
	pins->md_qy = 1;
	pins->mc_qz = 0;
	pins->md_qz = 2;
	pins->mc_qe = 1;
	pins->md_qe = 0;
	pins->mc_limit = 3;
	pins->md_limit = 1;
	pins->mc_lcd = 2;
	pins->md_lcd = 2;
	pins->invert_x_dir = 1;
	pins->x_step_pin = 0;
	pins->x_dir_pin = 1;
	pins->invert_y_dir = 1;
	pins->y_step_pin = 2;
	pins->y_dir_pin = 3;	
	pins->invert_z_dir = 1;
	pins->z_step_pin = 5;
	pins->z_dir_pin = 4;
	pins->invert_e_dir = 0;
	pins->e_step_pin = 0;
	pins->e_dir_pin = 1;	
	pins->x_enable_port = 4;
	pins->x_enable_pin = 0;
	pins->invert_x_enable = 1;
	pins->y_enable_port = 4;
	pins->y_enable_pin = 1;
	pins->invert_y_enable = 1;
	pins->z_enable_port = 4;
	pins->z_enable_pin = 2;
	pins->invert_z_enable = 1;
	pins->e_enable_port = 4;
	pins->e_enable_pin = 3;
	pins->invert_e_enable =1;
	
	tp->extruders = 1;
	tp->pwm_period = 10000;
	
	tp->heater_velocity = 3.5;
	tp->K1 = 0.95;
	tp->K2 = 1 - tp->K1;
	tp->autotune_pid_enable = 0;
	tp->default_Kp = 22.3;
	tp->default_Ki = 0.0132;
	tp->default_Kd = 54.5;
	tp->default_bedKp = 10.00;
	tp->default_bedKi = 0.0028;
	tp->default_bedKd = 2500.4625;
	tp->bang_max = 255;
	tp->pid_max = 255;
	tp->max_bed_power = 255;
	tp->fan_max = 255;
	tp->pid_integral_drive_max = 255;
	tp->heater_mintemp[0] = 5;
	tp->heater_maxtemp[0] = 275;
	tp->bed_mintemp = 5;
	tp->bed_maxtemp =150;
	tp->oversampler = 16;
	tp->pid_functional_range = 30;
	
	// 86Duino Print3D v1.0
	pins->mc_pwm[0] = 3;
	pins->md_pwm[0] = 0;
	pins->mc_pwm[1] = 3;
	pins->md_pwm[1] = 1;
	pins->mc_pwm[2] = -1;
	pins->md_pwm[2] = -1;
	pins->mc_pwm[3] = 2;
	pins->md_pwm[3] = 1;
	pins->mc_pwm[4] = -1;
	pins->md_pwm[4] = -1;
	pins->extruder_ad[0] = 0;
	pins->extruder_ad[1] = 1;
	pins->hotedbed_ad = 3;
	pins->vin_ad = 6;
	/*
	// 86Duino Print3D v2.0
	pins->mc_pwm[0] = 3;
	pins->md_pwm[0] = 0;
	pins->mc_pwm[1] = 3;
	pins->md_pwm[1] = 1;
	pins->mc_pwm[2] = 2;
	pins->md_pwm[2] = 2;
	pins->mc_pwm[3] = 2;
	pins->md_pwm[3] = 0;
	pins->mc_pwm[4] = 2;
	pins->md_pwm[4] = 1;

	pins->extruder_ad[0] = 0;
	pins->extruder_ad[1] = 1;
	pins->hotedbed_ad = 2;
	pins->vin_ad = 3;*/
	
	level->enable_auto_bed_leveling = 1;
	level->auto_bed_leveling_grid = 1;
	level->abl_probe_pt_1_x = 5;
	level->abl_probe_pt_1_y = 75;
	level->abl_probe_pt_2_x = 5;
	level->abl_probe_pt_2_y = 5;
	level->abl_probe_pt_3_x = 125;
	level->abl_probe_pt_3_y = 5;
	level->xy_travel_speed = 8000;
	level->x_probe_offset_from_extruder = 0;
	level->y_probe_offset_from_extruder = 0;
	level->z_probe_offset_from_extruder = 0;
	level->auto_bed_leveling_grid_points = 3;
	level->left_probe_bed_position = 5;
	level->right_probe_bed_position = 125;
	level->back_probe_bed_position = 75;
	level->front_probe_bed_position = 5;
	level->z_raise_before_probing = 15;
	level->z_raise_between_probings = 5;
}

static __inline__ void mem_free() {
	if(sys != NULL) free(sys);
	if(machine != NULL) free(machine);
	if(pins != NULL) free(pins);
	if(tp != NULL) free(tp);
	if(level != NULL) free(level);
}

static __inline__ int mem_set() {
	sys = (system_t *)malloc(sizeof(system_t));
	machine = (machine_t *)malloc(sizeof(machine_t));
	pins = (pins_t *)malloc(sizeof(pins_t));
	tp = (temperature_t *)malloc(sizeof(temperature_t));
	level = (autoleveling_t *)malloc(sizeof(autoleveling_t));
	if(sys == NULL || machine == NULL || pins == NULL || tp == NULL || level == NULL){
		mem_free();
		return -1;
	}
	return 0;
}

int init_setting(void) {
//
	double data;
	char temp[128] = {'\0'};
	INI_HANDLE handle = ini_open("setting.ini");
	if(handle == INI_FAIL){
		print_errmsg("%s\n", ini_geterrmsg());
		return -1;
	}
//
	if(mem_set() != 0) {
		print_errmsg("memory alloc fail!\n");
		return -1;
	}

	load_default();
	
	ini_get_key_ulong(handle, "system", "CMD_BUFFER_SIZE", &sys->cmd_buffer_size);
	ini_get_key_ulong(handle, "system", "CMD_BUFFER_NEAR_FULL", &sys->cmd_buffer_near_full);
	ini_get_key_ulong(handle, "system", "BLOCK_BUFFER_SIZE", &sys->block_buffer_size);
	ini_get_key_ulong(handle, "system", "BLOCK_BUFFER_BOUND", &sys->block_buffer_bound);
	ini_get_key_ulong(handle, "system", "MAX_CMD_SIZE", &sys->max_cmd_size);
	ini_get_key_ulong(handle, "system", "STEPPER_BUFFER_SIZE", &sys->stepper_buffer_size);
	ini_get_key_ulong(handle, "system", "STEPPER_BUFFER_BOUND", &sys->stepper_buffer_bound);
	ini_get_key_ulong(handle, "system", "MIN_STEP_PULSE_DUTY", &sys->min_step_pulse_duty);
	ini_get_key_ulong(handle, "system", "DEFAULT_PULSE_PERIOD", &sys->default_pulse_period);
	ini_get_key_ulong(handle, "system", "DEFAULT_PULSE_DUTY", &sys->default_pulse_duty);
	ini_get_key_ulong(handle, "system", "HIT_PULSE_PERIOD", &sys->hit_pulse_period);
	ini_get_key_ulong(handle, "system", "HIT_PULSE_DUTY", &sys->hit_pulse_duty);

	ini_get_key_int(handle, "machine", "MIN_SOFTWARE_LIMIT", &machine->min_software_limit);
	ini_get_key_int(handle, "machine", "MAX_SOFTWARE_LIMIT", &machine->max_software_limit);
	//TODO:改不管大小寫
	ini_get_key(handle, "machine", "TYPE", &temp[0]);
	if(strcmp(temp, "H_BOT") == 0) 
		machine->type = H_BOT;
	else if(strcmp(temp, "DELTA") == 0)
		machine->type = DELTA;
	else
		machine->type = NORMAL;
	
	ini_get_key_int(handle, "machine", "NUM_AXIS", &machine->num_axis);
	ini_get_key_double(handle, "machine", "X_MAX_POS_MM", &machine->x_max_pos_mm);
	ini_get_key_double(handle, "machine", "X_MIN_POS_MM", &machine->x_min_pos_mm);
	ini_get_key_double(handle, "machine", "X_HOME_POS_MM", &machine->x_home_pos_mm);
	ini_get_key_double(handle, "machine", "Y_MAX_POS_MM", &machine->y_max_pos_mm);
	ini_get_key_double(handle, "machine", "Y_MIN_POS_MM", &machine->y_min_pos_mm);
	ini_get_key_double(handle, "machine", "Y_HOME_POS_MM", &machine->y_home_pos_mm);
	ini_get_key_double(handle, "machine", "Z_MAX_POS_MM", &machine->z_max_pos_mm);
	ini_get_key_double(handle, "machine", "Z_MIN_POS_MM", &machine->z_min_pos_mm);
	ini_get_key_double(handle, "machine", "Z_HOME_POS_MM", &machine->z_home_pos_mm);
	ini_get_key_double(handle, "machine", "X_HOME_RETRACT_MM", &machine->x_home_retract_mm);
	ini_get_key_double(handle, "machine", "Y_HOME_RETRACT_MM", &machine->y_home_retract_mm);
	ini_get_key_double(handle, "machine", "Z_HOME_RETRACT_MM", &machine->z_home_retract_mm);
	ini_get_key_int(handle, "machine", "X_HOME_DIR", &machine->x_home_dir);;
	ini_get_key_int(handle, "machine", "Y_HOME_DIR", &machine->y_home_dir);
	ini_get_key_int(handle, "machine", "Z_HOME_DIR", &machine->z_home_dir);
	ini_get_key_double(handle, "machine", "QX_STEPS_PER_MM", &machine->qx_steps_per_mm);
	ini_get_key_double(handle, "machine", "QY_STEPS_PER_MM", &machine->qy_steps_per_mm);
	ini_get_key_double(handle, "machine", "QZ_STEPS_PER_MM", &machine->qz_steps_per_mm);
	ini_get_key_double(handle, "machine", "QE_STEPS_PER_MM", &machine->qe_steps_per_mm);	
	ini_get_key_double(handle, "machine", "X_MAX_FEEDRATE", &machine->x_max_feedrate_mps);
	ini_get_key_double(handle, "machine", "Y_MAX_FEEDRATE", &machine->y_max_feedrate_mps);
	ini_get_key_double(handle, "machine", "Z_MAX_FEEDRATE", &machine->z_max_feedrate_mps);
	ini_get_key_double(handle, "machine", "E_MAX_FEEDRATE", &machine->e_max_feedrate_mps);	
	ini_get_key_double(handle, "machine", "X_HOMING_FEEDRATE", &machine->x_homing_feedrate_mpm);
	ini_get_key_double(handle, "machine", "Y_HOMING_FEEDRATE", &machine->y_homing_feedrate_mpm);
	ini_get_key_double(handle, "machine", "Z_HOMING_FEEDRATE", &machine->z_homing_feedrate_mpm);	
	ini_get_key_double(handle, "machine", "QX_MAX_ACCELERATION", &machine->qx_max_acceleration);
	ini_get_key_double(handle, "machine", "QY_MAX_ACCELERATION", &machine->qy_max_acceleration);
	ini_get_key_double(handle, "machine", "QZ_MAX_ACCELERATION", &machine->qz_max_acceleration);
	ini_get_key_double(handle, "machine", "QE_MAX_ACCELERATION", &machine->qe_max_acceleration);	
	ini_get_key_double(handle, "machine", "DEFAULT_ACCELERATION", &machine->default_acceleration);
	ini_get_key_double(handle, "machine", "DEFAULT_RETRACT_ACCELERATION", &machine->default_retract_acceleration);
	ini_get_key_double(handle, "machine", "DEFAULT_MINIMUMFEEDRATE", &machine->default_minimumfeedrate);
	ini_get_key_double(handle, "machine", "DEFAULT_MINTRAVELFEEDRATE", &machine->default_mintravelfeedrate);
	ini_get_key_double(handle, "machine", "DEFAULT_XYJERK", &machine->default_xyjerk);
	ini_get_key_double(handle, "machine", "DEFAULT_EJERK", &machine->default_ejerk);
	ini_get_key_double(handle, "machine", "DEFAULT_ZJERK", &machine->default_zjerk);
	ini_get_key_double(handle, "machine", "MINIMUM_PLANNER_SPEED", &machine->minimum_planner_speed);
	
	//這裡必須檔0.5 跟 DUTY_MIN
	ini_get_key_double(handle, "machine", "STEP_PULSE_DUTY_RATE", &data);
	if(data < 0.5)
		machine->step_pulse_duty_ratio = 0.5;
	else if(data > 1 && data < (double)sys->min_step_pulse_duty)
		machine->step_pulse_duty_ratio = (double)sys->min_step_pulse_duty;
	else
		machine->step_pulse_duty_ratio = data;
	
	//TODO:改不管大小寫
	ini_get_key(handle, "machine", "LIMIT_SPEED", &temp[0]);
	if(strcmp(temp, "250k") == 0)
		machine->mcm_samplecycle = 25;
	else if(strcmp(temp, "200k") == 0)
		machine->mcm_samplecycle = 20;
	else if(strcmp(temp, "100k") == 0)
		machine->mcm_samplecycle = 10;
	else if(strcmp(temp, "50k") == 0)
		machine->mcm_samplecycle = 5;
	else if(strcmp(temp, "40k") == 0)
		machine->mcm_samplecycle = 4;
	else
		machine->mcm_samplecycle = 1;
	
	ini_get_key_int(handle, "machine", "MM_PER_ARC_SEGMENT", &machine->mm_per_arc_segment);
	ini_get_key_int(handle, "machine", "N_ARC_CORRECTION", &machine->n_arc_correction);	
	ini_get_key_double(handle, "machine", "X_HOME_OFFSET", &machine->home_offset[0]);
	ini_get_key_double(handle, "machine", "Y_HOME_OFFSET", &machine->home_offset[1]);
	ini_get_key_double(handle, "machine", "Z_HOME_OFFSET", &machine->home_offset[2]);

	ini_get_key_int(handle, "pins", "mc_qx", &pins->mc_qx);
	ini_get_key_int(handle, "pins", "md_qx", &pins->md_qx);
	ini_get_key_int(handle, "pins", "mc_qy", &pins->mc_qy);
	ini_get_key_int(handle, "pins", "md_qy", &pins->md_qy);
	ini_get_key_int(handle, "pins", "mc_qz", &pins->mc_qz);
	ini_get_key_int(handle, "pins", "md_qz", &pins->md_qz);
	ini_get_key_int(handle, "pins", "mc_qe", &pins->mc_qe);
	ini_get_key_int(handle, "pins", "md_qe", &pins->md_qe);
	ini_get_key_int(handle, "pins", "MC_LIMIT", &pins->mc_limit);
	ini_get_key_int(handle, "pins", "MD_LIMIT", &pins->md_limit);
	ini_get_key_int(handle, "pins", "MC_LCD", &pins->mc_lcd);
	ini_get_key_int(handle, "pins", "MD_LCD", &pins->md_lcd);

	ini_get_key_int(handle, "pins", "INVERT_X_DIR", &pins->invert_x_dir);
	ini_get_key_int(handle, "pins", "X_STEP_PIN", &pins->x_step_pin);
	ini_get_key_int(handle, "pins", "X_DIR_PIN", &pins->x_dir_pin);
	ini_get_key_int(handle, "pins", "INVERT_Y_DIR", &pins->invert_y_dir);
	ini_get_key_int(handle, "pins", "Y_STEP_PIN", &pins->y_step_pin);
	ini_get_key_int(handle, "pins", "Y_DIR_PIN", &pins->y_dir_pin);	
	ini_get_key_int(handle, "pins", "INVERT_Z_DIR", &pins->invert_z_dir);
	ini_get_key_int(handle, "pins", "Z_STEP_PIN", &pins->z_step_pin);
	ini_get_key_int(handle, "pins", "Z_DIR_PIN", &pins->z_dir_pin);
	ini_get_key_int(handle, "pins", "INVERT_E_DIR", &pins->invert_e_dir);	
	ini_get_key_int(handle, "pins", "E_STEP_PIN", &pins->e_step_pin);
	ini_get_key_int(handle, "pins", "E_DIR_PIN", &pins->e_dir_pin);	
	
	ini_get_key_int(handle, "pins", "X_ENABLE_PORT", &pins->x_enable_port);
	ini_get_key_int(handle, "pins", "X_ENABLE_PIN", &pins->x_enable_pin);
	ini_get_key_int(handle, "pins", "INVERT_X_ENABLE", &pins->invert_x_enable);
	ini_get_key_int(handle, "pins", "Y_ENABLE_PORT", &pins->y_enable_port);
	ini_get_key_int(handle, "pins", "Y_ENABLE_PIN", &pins->y_enable_pin);
	ini_get_key_int(handle, "pins", "INVERT_Y_ENABLE", &pins->invert_y_enable);
	ini_get_key_int(handle, "pins", "Z_ENABLE_PORT", &pins->z_enable_port);
	ini_get_key_int(handle, "pins", "Z_ENABLE_PIN", &pins->z_enable_pin);
	ini_get_key_int(handle, "pins", "INVERT_Z_ENABLE", &pins->invert_z_enable);
	ini_get_key_int(handle, "pins", "E_ENABLE_PORT", &pins->e_enable_port);
	ini_get_key_int(handle, "pins", "E_ENABLE_PIN", &pins->e_enable_pin);
	ini_get_key_int(handle, "pins", "INVERT_E_ENABLE", &pins->invert_e_enable);
	
	ini_get_key_int(handle, "temperature", "EXTRUDERS", &tp->extruders);
	ini_get_key_ulong(handle, "temperature", "PWM_PERIOD", &tp->pwm_period);
	
	ini_get_key_double(handle, "temperature", "HEATER_VELOCITY", &tp->heater_velocity);
	ini_get_key_double(handle, "temperature", "K1", &tp->K1);
	tp->K2 = 1 - tp->K1;
	ini_get_key_int(handle, "temperature", "AUTOTUNE_PID_ENABLE", &tp->autotune_pid_enable);
	ini_get_key_double(handle, "temperature", "DEFAULT_KP", &tp->default_Kp);
	ini_get_key_double(handle, "temperature", "DEFAULT_KI", &tp->default_Ki);
	ini_get_key_double(handle, "temperature", "DEFAULT_KD", &tp->default_Kd);
	ini_get_key_double(handle, "temperature", "DEFAULT_BEDKP", &tp->default_bedKp);
	ini_get_key_double(handle, "temperature", "DEFAULT_BEDKI", &tp->default_bedKi);
	ini_get_key_double(handle, "temperature", "DEFAULT_BEDKD", &tp->default_bedKd);

	ini_get_key_int(handle, "temperature", "BANG_MAX", &tp->bang_max);
	ini_get_key_int(handle, "temperature", "PID_MAX", &tp->pid_max);
	ini_get_key_int(handle, "temperature", "MAX_BED_POWER", &tp->max_bed_power);
	ini_get_key_int(handle, "temperature", "FAN_MAX", &tp->fan_max);	
	ini_get_key_int(handle, "temperature", "PID_INTEGRAL_DRIVE_MAX", &tp->pid_integral_drive_max);
	
	ini_get_key_double(handle, "temperature", "HEATER0_MINTEMP", &tp->heater_mintemp[0]);
	ini_get_key_double(handle, "temperature", "HEATER0_MAXTEMP", &tp->heater_maxtemp[0]);
	ini_get_key_double(handle, "temperature", "BED_MINTEMP", &tp->bed_mintemp);
	ini_get_key_double(handle, "temperature", "BED_MAXTEMP", &tp->bed_maxtemp);
	ini_get_key_int(handle, "temperature", "OVERSAMPLER", &tp->oversampler);
	ini_get_key_int(handle, "temperature", "PID_FUNCTIONAL_RANGE", &tp->pid_functional_range);
	
	// 86Duino Print3D v1.0
	ini_get_key_int(handle, "pins", "MC_PWM_HEATER0", &pins->mc_pwm[0]);
	ini_get_key_int(handle, "pins", "MD_PWM_HEATER0", &pins->md_pwm[0]);
	ini_get_key_int(handle, "pins", "MC_PWM_HEATER1", &pins->mc_pwm[1]);
	ini_get_key_int(handle, "pins", "MD_PWM_HEATER1", &pins->md_pwm[1]);
	ini_get_key_int(handle, "pins", "MC_PWM_BED", &pins->mc_pwm[2]);
	ini_get_key_int(handle, "pins", "MD_PWM_BED", &pins->md_pwm[2]);
	ini_get_key_int(handle, "pins", "MC_PWM_FAN0", &pins->mc_pwm[3]);
	ini_get_key_int(handle, "pins", "MD_PWM_FAN0", &pins->md_pwm[3]);
	ini_get_key_int(handle, "pins", "MC_PWM_FAN1", &pins->mc_pwm[4]);
	ini_get_key_int(handle, "pins", "MD_PWM_FAN1", &pins->md_pwm[4]);

	ini_get_key_int(handle, "pins", "AD_HEATER0", &pins->extruder_ad[0]);
	ini_get_key_int(handle, "pins", "AD_HEATER1", &pins->extruder_ad[1]);
	ini_get_key_int(handle, "pins", "AD_BED", &pins->hotedbed_ad);
	ini_get_key_int(handle, "pins", "AD_POWER", &pins->vin_ad);
	/*
	// 86Duino Print3D v2.0
	pins->mc_pwm[0] = 3;
	pins->md_pwm[0] = 0;
	pins->mc_pwm[1] = 3;
	pins->md_pwm[1] = 1;
	pins->mc_pwm[2] = 2;
	pins->md_pwm[2] = 2;
	pins->mc_pwm[3] = 2;
	pins->md_pwm[3] = 0;
	pins->mc_pwm[4] = 2;
	pins->md_pwm[4] = 1;

	pins->extruder_ad[0] = 0;
	pins->extruder_ad[1] = 1;
	pins->hotedbed_ad = 2;
	pins->vin_ad = 3;*/
	
	ini_get_key_int(handle, "Auto Bed Leveling", "ENABLE_AUTO_BED_LEVELING", &level->enable_auto_bed_leveling);
	ini_get_key_int(handle, "Auto Bed Leveling", "AUTO_BED_LEVELING_GRID", &level->auto_bed_leveling_grid);
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_1_X", &level->abl_probe_pt_1_x);
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_1_Y", &level->abl_probe_pt_1_y);
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_2_X", &level->abl_probe_pt_2_x);
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_2_Y", &level->abl_probe_pt_2_y );
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_3_X", &level->abl_probe_pt_3_x);
	ini_get_key_double(handle, "Auto Bed Leveling", "ABL_PROBE_PT_3_Y", &level->abl_probe_pt_3_y);
	ini_get_key_double(handle, "Auto Bed Leveling", "XY_TRAVEL_SPEED", &level->xy_travel_speed);
	ini_get_key_double(handle, "Auto Bed Leveling", "X_PROBE_OFFSET_FROM_EXTRUDER", &level->x_probe_offset_from_extruder);
	ini_get_key_double(handle, "Auto Bed Leveling", "Y_PROBE_OFFSET_FROM_EXTRUDER", &level->y_probe_offset_from_extruder);
	ini_get_key_double(handle, "Auto Bed Leveling", "Z_PROBE_OFFSET_FROM_EXTRUDER", &level->z_probe_offset_from_extruder);
	ini_get_key_int(handle, "Auto Bed Leveling", "AUTO_BED_LEVELING_GRID_POINTS", &level->auto_bed_leveling_grid_points);
	ini_get_key_double(handle, "Auto Bed Leveling", "LEFT_PROBE_BED_POSITION", &level->left_probe_bed_position);
	ini_get_key_double(handle, "Auto Bed Leveling", "RIGHT_PROBE_BED_POSITION", &level->right_probe_bed_position);
	ini_get_key_double(handle, "Auto Bed Leveling", "BACK_PROBE_BED_POSITION", &level->back_probe_bed_position);
	ini_get_key_double(handle, "Auto Bed Leveling", "FRONT_PROBE_BED_POSITION", &level->front_probe_bed_position);
	ini_get_key_double(handle, "Auto Bed Leveling", "Z_RAISE_BEFORE_PROBING", &level->z_raise_before_probing);
	ini_get_key_double(handle, "Auto Bed Leveling", "Z_RAISE_BETWEEN_PROBINGS", &level->z_raise_between_probings);
	
	if(ini_close(handle) == 1){
		print_errmsg("ini save fail\n");
	}
	return 0;
}
