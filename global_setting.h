#ifndef setting_h
#define setting_h

#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "conio.h"

#define USE_COMMON
#include "dmpcfg.h"
#include "common.h"
#include "err_msg.h"

#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)
#define E_AXIS (3)

#define NORMAL	 	(0)
#define H_BOT		(1)
#define DELTA		(2)

/************g_code global variables************/











typedef struct {
	unsigned long cmd_buffer_size;
	unsigned long cmd_buffer_near_full;

	unsigned long block_buffer_size;
	unsigned long block_buffer_bound;
	unsigned long max_cmd_size;
	
	unsigned long stepper_buffer_size;
	unsigned long stepper_buffer_bound;
	
	unsigned long min_step_pulse_duty;//  	250 //2.5us
	unsigned long default_pulse_period;//	100000//1ms
	unsigned long default_pulse_duty;//	50000//0.5ms
	unsigned long hit_pulse_period;//	10000//100us
	unsigned long hit_pulse_duty;//	5000//50us
} system_t;



typedef struct {
	int min_software_limit;
	int max_software_limit;
	int type; // normal, H_BOT, DELTA
	int num_axis;
	//unsigned char x_axis;
	//unsigned char y_axis;
	//unsigned char z_axis;
	//unsigned char e_axis;
	double x_max_pos_mm;
	double x_min_pos_mm;
	double x_home_pos_mm;
	double y_max_pos_mm;
	double y_min_pos_mm;
	double y_home_pos_mm;
	double z_max_pos_mm;
	double z_min_pos_mm;
	double z_home_pos_mm;
	
	double x_home_retract_mm;
	double y_home_retract_mm;
	double z_home_retract_mm;
	
	int x_home_dir;
	int y_home_dir;
	int z_home_dir;

	double qx_steps_per_mm;
	double qy_steps_per_mm;
	double qz_steps_per_mm;
	double qe_steps_per_mm;

	double x_max_feedrate_mps;//mm/sec
	double y_max_feedrate_mps;
	double z_max_feedrate_mps;
	double e_max_feedrate_mps;
	
	double x_homing_feedrate_mpm;//mm/min
	double y_homing_feedrate_mpm;//mm/min
	double z_homing_feedrate_mpm;//mm/min
	
	double qx_max_acceleration;//mm/sec^2
	double qy_max_acceleration;
	double qz_max_acceleration;
	double qe_max_acceleration;
	
	double default_acceleration;//mm/sec^2
	double default_retract_acceleration;//mm/sec^2
	
	double default_minimumfeedrate;//mm/sec
	double default_mintravelfeedrate;//mm/sec
	
	double default_xyjerk;//steps/sec
	double default_ejerk;//steps/sec	
	double default_zjerk;//steps/sec
	
	double minimum_planner_speed;//mm/sec
	
	double step_pulse_duty_ratio;//step_pluse_duty_ratio
	int mcm_samplecycle;//mcm_samplecycle
	
	//arc
	int mm_per_arc_segment;
	int n_arc_correction;	
	//
	
	double home_offset[3];//mm
	
} machine_t;

typedef struct {

	int extruders;
	unsigned long pwm_period;
	
	double default_Kp;
	double default_Ki;
	double default_Kd;
	double default_bedKp;
	double default_bedKi;
	double default_bedKd;
	double K1;
	double K2;
	double heater_velocity;
	
	int bang_max;
	int pid_max;
	int max_bed_power;
	int fan_max;
	
	int pid_integral_drive_max;
	double heater_mintemp[2];
	double heater_maxtemp[2];
	double bed_mintemp;
	double bed_maxtemp;
	
	int oversampler;
	int pid_functional_range;
	
	int autotune_pid_enable;
	
} temperature_t;

typedef struct {

	int enable_auto_bed_leveling;
	int auto_bed_leveling_grid;
	
	double abl_probe_pt_1_x;
	double abl_probe_pt_1_y;
	double abl_probe_pt_2_x;
	double abl_probe_pt_2_y;
	double abl_probe_pt_3_x;
	double abl_probe_pt_3_y;
	
	double xy_travel_speed;
	double x_probe_offset_from_extruder;
	double y_probe_offset_from_extruder;
	double z_probe_offset_from_extruder;
	
	int auto_bed_leveling_grid_points;
	double left_probe_bed_position;
	double right_probe_bed_position;
	double back_probe_bed_position;
	double front_probe_bed_position;
	double z_raise_before_probing;
	double z_raise_between_probings;
	
} autoleveling_t;

typedef struct {
	int mc_qx;
	int md_qx;
	int mc_qy;
	int md_qy;
	int mc_qz;
	int md_qz;
	int mc_qe;
	int md_qe;
	int mc_limit;
	int md_limit;
	int mc_lcd;
	int md_lcd;
	
	int invert_x_dir;
	int x_step_pin;
	int x_dir_pin;
	int invert_y_dir;
	int y_step_pin;
	int y_dir_pin;	
	int invert_z_dir;
	int z_step_pin;
	int z_dir_pin;	
	int invert_e_dir;
	int e_step_pin;
	int e_dir_pin;

	int x_enable_port;
	int x_enable_pin;
	int invert_x_enable;
	int y_enable_port;
	int y_enable_pin;
	int invert_y_enable;
	int z_enable_port;
	int z_enable_pin;
	int invert_z_enable;
	int e_enable_port;
	int e_enable_pin;
	int invert_e_enable;

	
	int mc_pwm[5];
	int md_pwm[5];
	int extruder_ad[2];
	int hotedbed_ad;
	int vin_ad;
	
} pins_t;



int init_setting(void);

extern system_t *sys;
extern machine_t *machine;
extern temperature_t *tp;
extern pins_t *pins;
extern autoleveling_t *level;


#endif
