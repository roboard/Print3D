#include "g_code.h"
#include "planner.h"
#include "command.h"
#include "protocol.h"
#include "communication.h"
#include "temperature.h"
#include "vector_3.h"
#include "qr_solve.h"
#include "global_setting.h"
#include <Arduino.h>

/************g_code private variables************/

/*****NFA*****/
#define SYS_STATE_NORMAL            (0x00)
#define SYS_STATE_HOMING            (0x01)
#define SYS_STATE_Z_PROBE           (0x02)
#define SYS_STATE_SINGLE_Z_PROBE    (0x03)
#define SYS_STATE_ALARM             (0x04)
#define SYS_STATE_SET_TEMP          (0x05)
#define SYS_STATE_SET_FAN           (0x06)
#define SYS_STATE_PID_AUTOTUNE      (0x07)
#define SYS_STATE_DISABLE_MOTOR     (0x08)
#define SYS_STATE_ARC  			    (0x09)
#define SYS_STATE_CREATE_TEMP_TABLE (0x0A)
#define SYS_STATE_SELECT_TEMP_TABLE (0x0B)
static int sys_state = SYS_STATE_NORMAL; //紀錄目前NFA的狀態

/*****end NFA*****/

/*****CMD*****/
static char *cmdbuffer; //存放一條G code的buffer
static char *strchr_pointer; //讀取數字所需記錄的起始位置 
/*****end CMD*****/

/*****水平校正*****/	
// "A" matrix of the linear system of equations
static double *eqnAMatrix; //水平校正使用的變數
// "B" vector of Z points
static double *eqnBVector; //水平校正使用的變數
/*****end 水平校正*****/

static bool relative_mode = false;  //Determines Absolute or Relative Coordinates
const char *axis_codes = "XYZE";
static double *destination;
static double offset[3] = {0.0, 0.0, 0.0};
double *current_position;
double add_homeing[3]={0,0,0};
bool *axis_relative_modes;
static double feedrate = 1500.0, next_feedrate, saved_feedrate;
unsigned char active_extruder = 0;
int saved_feedmultiply;
int feedmultiply = 100; //100->1 200->2
static bool home_all_axis = true;
double *homing_feedrate; // set the homing speeds (mm/min)
bool axis_known_position[3] = {false, false, false};


double min_pos[3];// = { machine->x_min_pos, machine->y_min_pos, machine->z_min_pos };
double max_pos[3];// = { machine->x_max_pos, machine->y_max_pos, machine->z_max_pos };

double zprobe_zoffset;

int get_command() {
	int serial_count = 0;
	char serial_char;
	while((serial_char = popCommandChr()) != '\0') {
		if(serial_char == -1 || serial_count >= sys->max_cmd_size) {
			// error mag
			return -1;
		}
		cmdbuffer[serial_count++] = serial_char;		
	}
	cmdbuffer[serial_count++] = '\0';	
	return 0;
}

double code_value() {
	return (strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL));
}

long code_value_long() {
	return (strtol(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL, 10));
}

bool code_seen(char code) {
	strchr_pointer = strchr(cmdbuffer, code);
	return (strchr_pointer != NULL);  //Return True if a character was found
}

void get_coordinates() {
	bool seen[4]={false,false,false,false};
	for(int i=0; i < machine->num_axis; i++) {
		if(code_seen(axis_codes[i])) {
		destination[i] = (double)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
		seen[i]=true;
		} else {
			destination[i] = current_position[i]; //Are these else lines really needed?
		}
	}
	if(code_seen('F')) {
		next_feedrate = code_value();
		if(next_feedrate > 0.0) feedrate = next_feedrate;
	}
}

void get_arc_coordinates() {
	get_coordinates();
	if(code_seen('I')) {
		offset[0] = code_value();
	} else {
		offset[0] = 0.0;
	}
	
	if(code_seen('J')) {
		offset[1] = code_value();
	} else {
		offset[1] = 0.0;
	}
}

void clamp_to_software_endstops(double target[]) {
	if (machine->min_software_limit == 1) {
		if (target[X_AXIS] < min_pos[X_AXIS]) target[X_AXIS] = min_pos[X_AXIS];
		if (target[Y_AXIS] < min_pos[Y_AXIS]) target[Y_AXIS] = min_pos[Y_AXIS];
		if (target[Z_AXIS] < min_pos[Z_AXIS]) target[Z_AXIS] = min_pos[Z_AXIS];
	}

	if (machine->max_software_limit == 1) {
		if (target[X_AXIS] > max_pos[X_AXIS]) target[X_AXIS] = max_pos[X_AXIS];
		if (target[Y_AXIS] > max_pos[Y_AXIS]) target[Y_AXIS] = max_pos[Y_AXIS];
		if (target[Z_AXIS] > max_pos[Z_AXIS]) target[Z_AXIS] = max_pos[Z_AXIS];
	}
	
}

void prepare_move() {
	clamp_to_software_endstops(destination);
	
	// Do not use feedmultiply for E or Z only moves
	if( (current_position[X_AXIS] == destination[X_AXIS]) && (current_position[Y_AXIS] == destination[Y_AXIS])) {
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);
	} else {
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
	}

	for(int i=0; i < machine->num_axis; i++) {
		current_position[i] = destination[i];
	}
}

#define ARC_CW (0)
#define ARC_CCW (1)

static char clockflag;
void prepare_arc_move() {
	static int arc_state = 0;
	double r;
	static double feed_rate;
	static double center_axis0;
	static double center_axis1;
	double linear_travel;
	double extruder_travel;
	static double r_axis0;
	static double r_axis1;
	double rt_axis0;
	double rt_axis1;
	double angular_travel;
	double millimeters_of_travel;
	static unsigned int segments;
	static double theta_per_segment;
	static double linear_per_segment;
	static double extruder_per_segment;
	static double cos_T;
	static double sin_T;
	static double arc_target[4];
	static double sin_Ti = 0.0;
	static double cos_Ti = 0.0;
	static double r_axisi = 0.0;
	static unsigned int i = 1;
	static char count = 0;
	unsigned int bound;
	switch (arc_state)
	{
	case 0:
		r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
		feed_rate = feedrate*feedmultiply/60/100.0;
		// Trace the arc
		//   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
		//   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
		center_axis0 = current_position[X_AXIS] + offset[X_AXIS];
		center_axis1 = current_position[Y_AXIS] + offset[Y_AXIS];
		linear_travel = destination[Z_AXIS] - current_position[Z_AXIS];
		extruder_travel = destination[E_AXIS] - current_position[E_AXIS];
		r_axis0 = -offset[X_AXIS];  // Radius vector from center to current location
		r_axis1 = -offset[Y_AXIS];
		rt_axis0 = destination[X_AXIS] - center_axis0;
		rt_axis1 = destination[Y_AXIS] - center_axis1;

		// CCW angle between position and target from circle center. Only one atan2() trig computation required.
		angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);

		//if (angular_travel < 0) { angular_travel += 2*M_PI; }
		//if (clockflag == ARC_CCW) { angular_travel -= 2*M_PI; }
		if (clockflag == ARC_CW) { // Correct atan2 output per direction
			if (angular_travel >= 0) { angular_travel -= 2*M_PI; }
		} else {
			if (angular_travel <= 0) { angular_travel += 2*M_PI; }
		}

		millimeters_of_travel = hypot(angular_travel*r, fabs(linear_travel));
		if (millimeters_of_travel < 0.001) { 
			arc_state = 2;
			return;
		}
		segments = floor(millimeters_of_travel/machine->mm_per_arc_segment);
		if(segments == 0) segments = 1;

		/*  
		// Multiply inverse feed_rate to compensate for the fact that this movement is approximated
		// by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
		// all segments.
		if (invert_feed_rate) { feed_rate *= segments; }
		*/
		theta_per_segment = angular_travel/segments;
		linear_per_segment = linear_travel/segments;
		extruder_per_segment = extruder_travel/segments;

		/* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
		 and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
			 r_T = [cos(phi) -sin(phi);
					sin(phi)  cos(phi] * r ;
		 
		 For arc generation, the center of the circle is the axis of rotation and the radius vector is 
		 defined from the circle center to the initial position. Each line segment is formed by successive
		 vector rotations. This requires only two cos() and sin() computations to form the rotation
		 matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
		 all double numbers are single precision on the Arduino. (True double precision will not have
		 round off issues for CNC applications.) Single precision error can accumulate to be greater than
		 tool precision in some cases. Therefore, arc path correction is implemented. 

		 Small angle approximation may be used to reduce computation overhead further. This approximation
		 holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
		 theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
		 to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
		 numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
		 issue for CNC machines with the single precision Arduino calculations.
		 
		 This approximation also allows mc_arc to immediately insert a line segment into the planner 
		 without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
		 a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
		 This is important when there are successive arc motions. 
		*/
		// Vector rotation matrix values
		cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
		sin_T = theta_per_segment;

		// Initialize the linear axis
		arc_target[Z_AXIS] = current_position[Z_AXIS];

		// Initialize the extruder axis
		arc_target[E_AXIS] = current_position[E_AXIS];
		arc_state = 1;
		i = 1;
		break;
	case 1:
		bound = i + 10;
		for (; i < bound && i < segments; i++) { // Increment (segments-1)
			if (count < machine->n_arc_correction) {
			  // Apply vector rotation matrix 
			  r_axisi = r_axis0*sin_T + r_axis1*cos_T;
			  r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
			  r_axis1 = r_axisi;
			  count++;
			} else {
			  // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
			  // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
			  cos_Ti = cos(i*theta_per_segment);
			  sin_Ti = sin(i*theta_per_segment);
			  r_axis0 = -offset[X_AXIS]*cos_Ti + offset[Y_AXIS]*sin_Ti;
			  r_axis1 = -offset[X_AXIS]*sin_Ti - offset[Y_AXIS]*cos_Ti;
			  count = 0;
			}

			// Update arc_target location
			arc_target[X_AXIS] = center_axis0 + r_axis0;
			arc_target[Y_AXIS] = center_axis1 + r_axis1;
			arc_target[Z_AXIS] += linear_per_segment;
			arc_target[E_AXIS] += extruder_per_segment;

			clamp_to_software_endstops(arc_target);
			plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, active_extruder);
		}
		if(i >= segments) arc_state = 2;
		break;
	case 2:
		// Ensure last segment arrives at target location.
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feed_rate, active_extruder);

		//   plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
		// As far as the parser is concerned, the position is now == target. In reality the
		// motion control system might still be processing the action and the real tool position
		// in any intermediate location.

		for(int i=0; i < machine->num_axis; i++) {
			current_position[i] = destination[i];
		}
		arc_state = 0;
		sys_state = SYS_STATE_NORMAL;
		break;
	}
}

double base_min_pos(int axis) {
	if(axis == X_AXIS) return machine->x_min_pos_mm;
	if(axis == Y_AXIS) return machine->y_min_pos_mm;
	if(axis == Z_AXIS) return machine->z_min_pos_mm;
}

double base_max_pos(int axis) {
	if(axis == X_AXIS) return machine->x_max_pos_mm;
	if(axis == Y_AXIS) return machine->y_max_pos_mm;
	if(axis == Z_AXIS) return machine->z_max_pos_mm;
}

double base_home_pos(int axis) {
	if(axis == X_AXIS) return machine->x_home_pos_mm;
	if(axis == Y_AXIS) return machine->y_home_pos_mm;
	if(axis == Z_AXIS) return machine->z_home_pos_mm;
}

double max_length(int axis) {
	if(axis == X_AXIS) return machine->x_max_pos_mm - machine->x_min_pos_mm;
	if(axis == Y_AXIS) return machine->y_max_pos_mm - machine->y_min_pos_mm;
	if(axis == Z_AXIS) return machine->z_max_pos_mm - machine->z_min_pos_mm;
}

double home_retract_mm(int axis) {
	if(axis == X_AXIS) return machine->x_home_retract_mm;
	if(axis == Y_AXIS) return machine->y_home_retract_mm;
	if(axis == Z_AXIS) return machine->z_home_retract_mm;
}

int home_dir(int axis) {
	if(axis == X_AXIS) return machine->x_home_dir;
	if(axis == Y_AXIS) return machine->y_home_dir;
	if(axis == Z_AXIS) return machine->z_home_dir;
}

static void axis_is_at_home(int axis) {
	current_position[axis] = base_home_pos(axis) + add_homeing[axis];
	min_pos[axis] =          base_min_pos(axis) + add_homeing[axis];
	max_pos[axis] =          base_max_pos(axis) + add_homeing[axis];
}

static bool homeaxis(int axis) {
	static int homeaxis_state = 0;
	int axis_home_dir = home_dir(axis);

	if (!plan_buffer_null() || !st_buffer_null()) return false;
	
	switch (homeaxis_state)
	{
	case 0:
		current_position[axis] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		destination[axis] = 1.5 * max_length(axis) * axis_home_dir;
		feedrate = homing_feedrate[axis];
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);

		homeaxis_state = 1;
		break;
	case 1:
		current_position[axis] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = -home_retract_mm(axis) * axis_home_dir;
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);

		homeaxis_state = 2;
		break;
	case 2:
		destination[axis] = 2*home_retract_mm(axis) * axis_home_dir;

		feedrate = homing_feedrate[axis]/2 ;

		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);

		homeaxis_state = 3;
		break;
	case 3:
		current_position[axis] = 0;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = machine->home_offset[axis] * (-axis_home_dir);
		plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate/60, active_extruder);

		homeaxis_state = 4;
		break;
	case 4:
		axis_is_at_home(axis);
		destination[axis] = current_position[axis];
		feedrate = 0.0;
		//endstops_hit_on_purpose(); //stepper中check的設定 
		axis_known_position[axis] = true;

		homeaxis_state = 0;
		return true;
	}

	return false;
}
//#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)



static void homing()
{
	static int home_state = 0;

	switch (home_state)
	{
	case 0:
		if (level->enable_auto_bed_leveling != 0)
			plan_bed_level_matrix.set_to_identity();  //Reset the plane ("erase" all leveling data)

		saved_feedrate = feedrate;
		saved_feedmultiply = feedmultiply;
		feedmultiply = 100;

		for(int i=0; i < machine->num_axis; i++) {
			destination[i] = current_position[i];
		}
		feedrate = 0.0;
		
		home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS])));
		home_state = 1;
		break;
	case 1:
		if (!(home_all_axis) && !(code_seen(axis_codes[Y_AXIS])))
			home_state = 2;
		else if (homeaxis(Y_AXIS) == true)
			home_state = 2;
		else
			;
		break;
	case 2:
		if (!(home_all_axis) && !(code_seen(axis_codes[X_AXIS])))
			home_state = 3;
		else if (homeaxis(X_AXIS) == true)
			home_state = 3;
		else
			;
		break;
	case 3:
		if (!(home_all_axis) && !(code_seen(axis_codes[Z_AXIS])))
			home_state = 4;
		else if (homeaxis(Z_AXIS) == true)
			home_state = 4;
		else
			;
		break;
	case 4:
		if(code_seen(axis_codes[X_AXIS])) {
			if(code_value_long() != 0) {
				current_position[X_AXIS]=code_value()+add_homeing[0];
			}
		}
		if(code_seen(axis_codes[Y_AXIS])) {
			if(code_value_long() != 0) {
				current_position[Y_AXIS]=code_value()+add_homeing[1];
			}
		}           
		if(code_seen(axis_codes[Z_AXIS])) {
			if(code_value_long() != 0) {
				current_position[Z_AXIS]=code_value()+add_homeing[2];
			}
		}
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

		feedrate = saved_feedrate;
		feedmultiply = saved_feedmultiply;
		
		COMM_WRITE("Bed");
		COMM_WRITE(" X: ");
		COMM_WRITE_FLOAT(current_position[X_AXIS]);
		COMM_WRITE(" Y: ");
		COMM_WRITE_FLOAT(current_position[Y_AXIS]);
		COMM_WRITE(" Z: ");
		COMM_WRITE_FLOAT(current_position[Z_AXIS]);
		COMM_WRITE("\n");
		
		home_state = 0;
		sys_state = SYS_STATE_NORMAL;
		break;
	}
}

static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
    planeNormal.debug("planeNormal");
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    //bedLevel.debug("bedLevel");

    //plan_bed_level_matrix.debug("bed level before");
    //vector_3 uncorrected_position = plan_get_position_mm();
    //uncorrected_position.debug("position before");

    vector_3 corrected_position = plan_get_position();
//    corrected_position.debug("position after");
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    // but the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset; // in the lsq we reach here after raising the extruder due to the loop structure

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}

static void set_bed_level_equation_3pts(double z_at_pt_1, double z_at_pt_2, double z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(level->abl_probe_pt_1_x, level->abl_probe_pt_1_y, z_at_pt_1);
    vector_3 pt2 = vector_3(level->abl_probe_pt_2_x, level->abl_probe_pt_2_y, z_at_pt_2);
    vector_3 pt3 = vector_3(level->abl_probe_pt_3_x, level->abl_probe_pt_3_y, z_at_pt_3);
	
    vector_3 from_2_to_1 = (pt1 - pt2).get_normal();
    vector_3 from_2_to_3 = (pt3 - pt2).get_normal();
	vector_3 planeNormal = vector_3::cross(from_2_to_1, from_2_to_3).get_normal();
    planeNormal = vector_3(planeNormal.x, planeNormal.y, fabs(planeNormal.z));
	
    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
    plan_bed_level_matrix = matrix_3x3::transpose(plan_bed_level_matrix);
	
    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;
	
    // put the bed at 0 so we don't go below it.
    current_position[Z_AXIS] = zprobe_zoffset;

    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

}

static bool run_z_probe() {
	static int run_z_probe_state = 0;
	static double zPosition;
	
	if (!plan_buffer_null() || !st_buffer_null()) return false;
	
	switch (run_z_probe_state)
	{
	case 0:
		plan_bed_level_matrix.set_to_identity();
		feedrate = homing_feedrate[Z_AXIS];
		
		// move down until you find the bed
		zPosition = -10;
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
		run_z_probe_state = 1;
		break;
	case 1:
		// we have to let the planner know where we are right now as it is not where we said to go.
		zPosition = st_get_position_mm(Z_AXIS);
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS]);
        
		// move up the retract distance
		zPosition += home_retract_mm(Z_AXIS);
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
		run_z_probe_state = 2;
		break;
	case 2:
		// move back down slowly to find bed
		feedrate = homing_feedrate[Z_AXIS]/4;
		zPosition -= home_retract_mm(Z_AXIS) * 2;
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], zPosition, current_position[E_AXIS], feedrate/60, active_extruder);
		run_z_probe_state = 3;
		break;
	case 3:
		current_position[Z_AXIS] = st_get_position_mm(Z_AXIS);
		// make sure the planner knows where we are as it may be a bit different than we last said to move to
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		run_z_probe_state = 0;
		return true;
	}
	
	return false;
}

static bool do_blocking_move_to(double x, double y, double z) {
	static int do_blocking_move_to_state = 0;
    static double oldFeedRate;
	
	switch (do_blocking_move_to_state)
	{
	case 0:
		oldFeedRate = feedrate;
		feedrate = level->xy_travel_speed;
		
		current_position[X_AXIS] = x;
		current_position[Y_AXIS] = y;
		current_position[Z_AXIS] = z;
		plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate/60, active_extruder);
		do_blocking_move_to_state = 1;
		break;
	case 1:
		if (!plan_buffer_null() || !st_buffer_null()) break;
		feedrate = oldFeedRate;
		
		do_blocking_move_to_state = 0;
		return true;
	}
	
	return false;
}

static bool do_blocking_move_relative(double offset_x, double offset_y, double offset_z) {
    return do_blocking_move_to(current_position[X_AXIS] + offset_x, current_position[Y_AXIS] + offset_y, current_position[Z_AXIS] + offset_z);
}

static void setup_for_endstop_move() {
    saved_feedrate = feedrate;
    saved_feedmultiply = feedmultiply;
    feedmultiply = 100;
    // previous_millis_cmd = millis();

    // enable_endstops(true);
}

static void clean_up_after_endstop_move() {
// #ifdef ENDSTOPS_ONLY_FOR_HOMING
    // enable_endstops(false);
// #endif

    feedrate = saved_feedrate;
    feedmultiply = saved_feedmultiply;
    // previous_millis_cmd = millis();
}

static void engage_z_probe() {

}

static void retract_z_probe() {

}

/// Probe bed height at position (x,y), returns the measured z value
static bool probe_pt(double x, double y, double z_before, double *pz_probe) {
	static int probe_pt_state = 0;
	
	switch (probe_pt_state)
	{
	case 0:
		// move to right place
		if (do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z_before) == true)
			probe_pt_state = 1;
		break;
	case 1:
		if (do_blocking_move_to(x - level->x_probe_offset_from_extruder, y - level->y_probe_offset_from_extruder, current_position[Z_AXIS]) == true)
			probe_pt_state = 2;
		break;
	case 2:
		engage_z_probe();   // Engage Z Servo endstop if available
		probe_pt_state = 3;
		break;
	case 3:
		if (run_z_probe() == true) {
			*pz_probe = current_position[Z_AXIS];
			retract_z_probe();
			/*SERIAL_PROTOCOLPGM(MSG_BED);
			SERIAL_PROTOCOLPGM(" x: ");
			SERIAL_PROTOCOL(x);
			SERIAL_PROTOCOLPGM(" y: ");
			SERIAL_PROTOCOL(y);
			SERIAL_PROTOCOLPGM(" z: ");
			SERIAL_PROTOCOL(measured_z);
			SERIAL_PROTOCOLPGM("\n");*/
			COMM_WRITE("Bed");
			COMM_WRITE(" x: ");
			COMM_WRITE_FLOAT(x);
			COMM_WRITE(" y: ");
			COMM_WRITE_FLOAT(y);
			COMM_WRITE(" z: ");
			COMM_WRITE_FLOAT(*pz_probe);
			COMM_WRITE("\n");
			probe_pt_state = 0;
			return true;
		}
		break;
	}
	
	return false;
}

static bool probe_grid()
{
	static int probe_grid_state = 0;
	static int xGridSpacing, yGridSpacing;
	static int probePointCounter, yProbe, xProbe, xInc, xCount;
	static bool zig;
	static double z_before;
	double measured_z;
	
	switch (probe_grid_state)
	{
	case 0:
		// probe at the points of a lattice grid

		xGridSpacing = (level->right_probe_bed_position - level->left_probe_bed_position) / (level->auto_bed_leveling_grid_points-1);
		yGridSpacing = (level->back_probe_bed_position - level->front_probe_bed_position) / (level->auto_bed_leveling_grid_points-1);
	
		// solve the plane equation ax + by + d = z
		// A is the matrix with rows [x y 1] for all the probed points
		// B is the vector of the Z positions
		// the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
		// so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

		probePointCounter = 0;
		zig = true;
		yProbe = level->front_probe_bed_position;
		
		probe_grid_state = 1;
		break;
	case 1:
		if (yProbe <= level->back_probe_bed_position) {
			if (zig)
			{
				xProbe = level->left_probe_bed_position;
				//xEnd = level->right_probe_bed_position;
				xInc = xGridSpacing;
				zig = false;
			} else // zag
			{
				xProbe = level->right_probe_bed_position;
				//xEnd = level->left_probe_bed_position;
				xInc = -xGridSpacing;
				zig = true;
			}
			xCount = 0;
			probe_grid_state = 2;
		} else {
			probe_grid_state = 4;
		}
		break;
	case 2:
		if (xCount < level->auto_bed_leveling_grid_points) {
			if (probePointCounter == 0)
			{
				// raise before probing
				z_before = level->z_raise_before_probing;
			} else
			{
				// raise extruder
				z_before = current_position[Z_AXIS] + level->z_raise_between_probings;
			}
			probe_grid_state = 3;
		} else {
			yProbe += yGridSpacing;
			probe_grid_state = 1;
		}
		break;
	case 3:
		if (probe_pt(xProbe, yProbe, z_before, &measured_z) == false) break;
		eqnBVector[probePointCounter] = measured_z;

		eqnAMatrix[probePointCounter + 0*level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points] = xProbe;
		eqnAMatrix[probePointCounter + 1*level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points] = yProbe;
		eqnAMatrix[probePointCounter + 2*level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points] = 1;
		probePointCounter++;
		xProbe += xInc;
		xCount++;
		probe_grid_state = 2;
		break;
	case 4:
		clean_up_after_endstop_move();

		// solve lsq problem
		double *plane_equation_coefficients = qr_solve(level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points, 3, eqnAMatrix, eqnBVector);

		/*SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
		SERIAL_PROTOCOL(plane_equation_coefficients[0]);
		SERIAL_PROTOCOLPGM(" b: ");
		SERIAL_PROTOCOL(plane_equation_coefficients[1]);
		SERIAL_PROTOCOLPGM(" d: ");
		SERIAL_PROTOCOLLN(plane_equation_coefficients[2]);*/
		COMM_WRITE("Eqn coefficients: a: ");
		COMM_WRITE_FLOAT(plane_equation_coefficients[0]);
		COMM_WRITE(" b: ");
		COMM_WRITE_FLOAT(plane_equation_coefficients[1]);
		COMM_WRITE(" d: ");
		COMM_WRITE_FLOAT(plane_equation_coefficients[2]);


		set_bed_level_equation_lsq(plane_equation_coefficients);

		free(plane_equation_coefficients);
		
		probe_grid_state = 0;
		return true;
	}
	
	return false;
}

static bool probe_at_3_points()
{
	static int probe_at_3_points_state = 0;
	static double z_at_pt_1, z_at_pt_2, z_at_pt_3;
	
	switch (probe_at_3_points_state)
	{
	case 0:
		// Probe at 3 arbitrary points
		// probe 1
		if (probe_pt(level->abl_probe_pt_1_x, level->abl_probe_pt_1_y, level->z_raise_before_probing, &z_at_pt_1) == true)
			probe_at_3_points_state = 1;
		break;
	case 1:
		// probe 2
		if (probe_pt(level->abl_probe_pt_2_x, level->abl_probe_pt_2_y, current_position[Z_AXIS] + level->z_raise_between_probings, &z_at_pt_2) == true)
			probe_at_3_points_state = 2;
		break;
	case 2:
		// probe 3
		if (probe_pt(level->abl_probe_pt_3_x, level->abl_probe_pt_3_y, current_position[Z_AXIS] + level->z_raise_between_probings, &z_at_pt_3) == true)
			probe_at_3_points_state = 3;
		break;
	case 3:
		clean_up_after_endstop_move();
		
		set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
		
		probe_at_3_points_state = 0;
		return true;
	}

	return false;
}


static vector_3 uncorrected_position;
static void Auto_Z_Probe()
{
	static int auto_z_probe_state = 0;
	double x_tmp, y_tmp, z_tmp, real_z;

	//#if Z_MIN_PIN == -1
	//#error "You must have a Z_MIN endstop in order to enable Auto Bed Leveling feature!!! Z_MIN_PIN must point to a valid hardware pin."
	//#endif
	
	switch (auto_z_probe_state)
	{
	case 0:
		// Prevent user from running a G29 without first homing in X and Y
		if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )
		{
			// LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
			// SERIAL_ECHO_START;
			// SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
			COMM_WRITE("echo:");
			COMM_WRITE("Home X/Y before Z\n");
			sys_state = SYS_STATE_NORMAL;
			break; // abort G29, since we don't know where we are
		}
		auto_z_probe_state = 1;
		break;
	case 1:
		if (!plan_buffer_null() || !st_buffer_null()) break;
			
		// make sure the bed_level_rotation_matrix is identity or the planner will get it incorectly
		//vector_3 corrected_position = plan_get_position_mm();
		//corrected_position.debug("position before G29");
		plan_bed_level_matrix.set_to_identity();
		uncorrected_position = plan_get_position();
		//uncorrected_position.debug("position durring G29");
		current_position[X_AXIS] = uncorrected_position.x;
		current_position[Y_AXIS] = uncorrected_position.y;
		current_position[Z_AXIS] = uncorrected_position.z;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		setup_for_endstop_move();

		feedrate = homing_feedrate[Z_AXIS];
		
		auto_z_probe_state = 2;
		break;
	case 2:
		if ((level->auto_bed_leveling_grid != 0 && probe_grid() == true)
		 || (level->auto_bed_leveling_grid == 0 && probe_at_3_points() == true))
			auto_z_probe_state = 3;
		break;
	case 3:
		if (!plan_buffer_null() || !st_buffer_null()) break;
		// The following code correct the Z height difference from z-probe position and hotend tip position.
		// The Z height on homing is measured by Z-Probe, but the probe is quite far from the hotend.
		// When the bed is uneven, this height must be corrected.
		real_z = double(st_get_position(Z_AXIS))/axis_steps_per_unit[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
		x_tmp = current_position[X_AXIS] + level->x_probe_offset_from_extruder;
		y_tmp = current_position[Y_AXIS] + level->y_probe_offset_from_extruder;
		z_tmp = current_position[Z_AXIS];
		
		apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);         //Apply the correction sending the probe offset
		current_position[Z_AXIS] = z_tmp - real_z + current_position[Z_AXIS];   //The difference is added to current position and sent to planner.
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		
		COMM_WRITE("Bed");
		COMM_WRITE(" X: ");
		COMM_WRITE_FLOAT(current_position[X_AXIS]);
		COMM_WRITE(" Y: ");
		COMM_WRITE_FLOAT(current_position[Y_AXIS]);
		COMM_WRITE(" Z: ");
		COMM_WRITE_FLOAT(current_position[Z_AXIS]);
		COMM_WRITE("\n");
		
		auto_z_probe_state = 0;
		sys_state = SYS_STATE_NORMAL;
		break;
	}
}

static void Single_Z_Probe()
{
	static int single_z_probe_state = 0;
	
	switch (single_z_probe_state)
	{
	case 0:
		engage_z_probe(); // Engage Z Servo endstop if available
		single_z_probe_state = 1;
		break;
	case 1:
		setup_for_endstop_move();

		feedrate = homing_feedrate[Z_AXIS];
		single_z_probe_state = 2;
		break;
	case 2:
		if (run_z_probe() == true)
			single_z_probe_state = 3;
		break;
	case 3:
		COMM_WRITE("Bed");
		COMM_WRITE(" X: ");
		COMM_WRITE_FLOAT(current_position[X_AXIS]);
		COMM_WRITE(" Y: ");
		COMM_WRITE_FLOAT(current_position[Y_AXIS]);
		COMM_WRITE(" Z: ");
		COMM_WRITE_FLOAT(current_position[Z_AXIS]);
		COMM_WRITE("\n");

		clean_up_after_endstop_move();

		retract_z_probe(); // Retract Z Servo endstop if available
		single_z_probe_state = 0;
		sys_state = SYS_STATE_NORMAL;
		break;
	}
}

static double reg_temp;
static bool ChangeTempWait = false;
static bool CooldownNoWait = false;
static bool ChangeTempFirst = false;
static bool target_direction;
static void set_temperature()
{
	if (!plan_buffer_null() || !st_buffer_null()) return;
	
	if (ChangeTempWait) {
		if  (ChangeTempFirst) {
			setTargetHotend(reg_temp, EXTRUDER0);
			target_direction = isHeatingHotend(EXTRUDER0);
			ChangeTempFirst = false;
		}
		if (target_direction ? (!isHeatingHotend(EXTRUDER0)) : (!isCoolingHotend(EXTRUDER0)||(CooldownNoWait==true)))
		{
			ChangeTempWait = false;
			CooldownNoWait = false;
			sys_state = SYS_STATE_NORMAL;
		}
	} else {
		setTargetHotend(reg_temp, EXTRUDER0);
		sys_state = SYS_STATE_NORMAL;
	}
}

static double fanSpeed = 0.0;
static void set_fan()
{
	if (!plan_buffer_null() || !st_buffer_null()) return;
	
	setFanPeriod(COLDEDFAN0, fanSpeed);
	sys_state = SYS_STATE_NORMAL;
}

static double pid_autotune_temp = 150.0;
static int pid_autotune_extruder = 0;
static int pid_autotune_cycle = 5;
static void pid_autotune()
{
	if (PID_autotune(pid_autotune_temp, pid_autotune_extruder, pid_autotune_cycle) == true)
		sys_state = SYS_STATE_NORMAL;
}

static int disable_axis = 0;
static void disable_motor()
{
	if (!plan_buffer_null() || !st_buffer_null()) return;
	
	if (disable_axis & (1 << X_AXIS)) disable_x();
	if (disable_axis & (1 << Y_AXIS)) disable_y();
	if (disable_axis & (1 << Z_AXIS)) disable_z();
	if (disable_axis & (1 << E_AXIS)) {
		disable_e();
		//disable_e0();
		//disable_e1();
	}
	disable_axis = 0;
	sys_state = SYS_STATE_NORMAL;
}

static int select_table_e;
static int select_table_d;
static void select_temp_talbe()
{
	if (!plan_buffer_null() || !st_buffer_null()) return;
	
	SelectThermistorTable(select_table_e, select_table_d);
	sys_state = SYS_STATE_NORMAL;
}

static int create_table_d, create_table_n;
static double create_table_m, create_table_r;
static double create_table_i, create_table_j, create_table_k;
static double create_table_u, create_table_v, create_table_w;
static void create_temp_talbe()
{
	if (!plan_buffer_null() || !st_buffer_null()) return;

	CreateThermistorTable(create_table_d, create_table_n, 0, create_table_m, create_table_r,
                          create_table_i, create_table_j, create_table_k,
                          create_table_u, create_table_v, create_table_w);
	
	sys_state = SYS_STATE_NORMAL;
}

extern volatile unsigned long block_buffer_head;
extern volatile unsigned long block_buffer_tail;
void process_commands(unsigned long clock) {
	unsigned long codenum; //throw away variable
	char *starpos = NULL;
	//if(sys.state == STATE_ALARM) return;
	unsigned long next_buffer_head;

	if (sys_state == SYS_STATE_ARC) {
		prepare_arc_move();
		return;
	}
	
	if (sys_state == SYS_STATE_HOMING) {
		homing();
		return;
	}
	if (sys_state == SYS_STATE_SET_TEMP) {
		set_temperature();
		return;
	}
	if (sys_state == SYS_STATE_SET_FAN) {
		set_fan();
		return;
	}
	if (sys_state == SYS_STATE_PID_AUTOTUNE) {
		pid_autotune();
		return;
	}
	if (sys_state == SYS_STATE_Z_PROBE) {
		Auto_Z_Probe();
		return;
	}
	if (sys_state == SYS_STATE_SINGLE_Z_PROBE) {
		Single_Z_Probe();
		return;
	}
	if (sys_state == SYS_STATE_DISABLE_MOTOR) {
		disable_motor();
		return;
	}
	if (sys_state == SYS_STATE_SELECT_TEMP_TABLE) {
		select_temp_talbe();
		return;
	}
	if (sys_state == SYS_STATE_CREATE_TEMP_TABLE) {
		create_temp_talbe();
		return;
	}
	
	while(clock != 0) {
	next_buffer_head = next_block_index(block_buffer_head);
	if(block_buffer_tail == next_buffer_head) {
		return;
	}

	if(get_command() != 0)
		return;

	if(code_seen('G')) {
		switch((int)code_value()) {
		case 0: // G0 -> G1
		case 1: // G1
			get_coordinates(); // For X Y Z E F
			prepare_move();
			// // response_ack();
			break;
		case 2: // G2  - CW ARC
			get_arc_coordinates();
			clockflag = ARC_CW;
			sys_state = SYS_STATE_ARC;
			// // response_ack();
			break;
		case 3: // G3  - CCW ARC
			get_arc_coordinates();
			clockflag = ARC_CCW;
			sys_state = SYS_STATE_ARC;
			// // response_ack();
			break;
		case 4: // G4 dwell
			codenum = 0;
			if(code_seen('P')) codenum = code_value(); // milliseconds to wait
			if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
			// response_ack();
			//st_synchronize();
			//delay_ms(codenum);
			//delay時也要做溫度的控制
			break;
		case 10://回抽
			// response_ack();
			break;
		case 11://釋放回抽
			// response_ack();
			break;
		case 28: //G28 Home all Axis one at a time
			sys_state = SYS_STATE_HOMING;
			// response_ack();
			return;
		case 29:
			if (level->enable_auto_bed_leveling == 0) break;
			sys_state = SYS_STATE_Z_PROBE;
			// response_ack();
			return;
		case 30: // G30 Single Z Probe
			if (level->enable_auto_bed_leveling == 0) break;
			sys_state = SYS_STATE_SINGLE_Z_PROBE;
			// response_ack();
			return;
		case 90: // G90
			relative_mode = false;
			// response_ack();
			break;
		case 91: // G91
			relative_mode = true;
			// response_ack();
			break;
		case 92: // G92
			// if(!code_seen(axis_codes[E_AXIS]))
				//st_synchronize();
			for(int i=0; i < machine->num_axis; i++) {
				if(code_seen(axis_codes[i])) {
					if(i == E_AXIS) {
						current_position[i] = code_value();
						plan_set_e_position(current_position[E_AXIS]);//in planner
					}
					else {
						current_position[i] = code_value()+add_homeing[i];
						plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
					}
				}
			}
			// response_ack();
			break;
		default:
			// response_ack();
			break;
		}
	} else 
	if(code_seen('M')) {
		switch( (int)code_value() ) {
		case 106:
			if (code_seen('S')){
			   fanSpeed=constrain(code_value(),0, tp->fan_max);
			}
			else {
			  fanSpeed=tp->fan_max;
			}
			sys_state = SYS_STATE_SET_FAN;
			// response_ack();
			return;
		case 107:
			fanSpeed=0;
			sys_state = SYS_STATE_SET_FAN;
			// response_ack();
			return;
		case 109:
			if (code_seen('S')) {
				reg_temp = code_value();
				sys_state = SYS_STATE_SET_TEMP;
				CooldownNoWait = true;
			} else if (code_seen('R')) {
				reg_temp = code_value();
				sys_state = SYS_STATE_SET_TEMP;
				CooldownNoWait = false;
			}
			ChangeTempWait = true;
			ChangeTempFirst = true;
			// response_ack();
			return;
		case 104: // M104 Set extruder target temp
			//if(setTargetedHotend(104)){
			//  break;
			//}
			if (code_seen('S')) {
				reg_temp = code_value();
				sys_state = SYS_STATE_SET_TEMP;
			}			
			// response_ack();
			//setWatch();
			return;
		case 140: // M140 set bed temp
			if (code_seen('S')) setTargetBed(code_value());
			// response_ack();
			break;
		case 303: // M303 PID autotune
			pid_autotune_temp = 150.0;
			pid_autotune_extruder = 0;
			pid_autotune_cycle = 5;
			if (code_seen('E')) pid_autotune_extruder = code_value();
			if (pid_autotune_extruder < 0)
				pid_autotune_temp = 70.0;
			if (code_seen('S')) pid_autotune_temp = code_value();
			if (code_seen('C')) pid_autotune_cycle = code_value();
			sys_state = SYS_STATE_PID_AUTOTUNE;
			return;
		case 301:
		{
			double Kp, Ki, Kd;
			if(code_seen('P')) {
				Kp = code_value();
				setExtruderPGain(Kp);
			}
			if(code_seen('I')) {
				Ki = scalePID_i(code_value());
				setExtruderIGain(Ki);
			}
			if(code_seen('D')) {
				Kd = scalePID_d(code_value());
				setExtruderDGain(Kd);
			}
			
			updatePID();
			COMM_WRITE("ok");
			COMM_WRITE(" p:");
			COMM_WRITE_FLOAT(Kp);
			COMM_WRITE(" i:");
			COMM_WRITE_FLOAT(unscalePID_i(Ki));
			COMM_WRITE(" d:");
			COMM_WRITE_FLOAT(unscalePID_d(Kd));
			COMM_WRITE("\n");
		}
		break;
		case 304:
		{
			double bedKp, bedKi, bedKd;
			if(code_seen('P')) {
				bedKp = code_value();
				setBedPGain(bedKp);
			}
			if(code_seen('I')) {
				bedKi = scalePID_i(code_value());
				setBedIGain(bedKi);
			}
			if(code_seen('D')) {
				bedKd = scalePID_d(code_value());
				setBedDGain(bedKd);
			}

			updatePID();
			COMM_WRITE("ok");
			COMM_WRITE(" p:");
			COMM_WRITE_FLOAT(bedKp);
			COMM_WRITE(" i:");
			COMM_WRITE_FLOAT(unscalePID_i(bedKi));
			COMM_WRITE(" d:");
			COMM_WRITE_FLOAT(unscalePID_d(bedKd));
			COMM_WRITE("\n");
		}
		break;
		case 105 : // M105 Read current temp
		
			break;
		case 117:
			// response_ack();
			break;
		case 82:
			axis_relative_modes[3] = false;
			// response_ack();
			break;
		case 83:
			axis_relative_modes[3] = true;
			// response_ack();
			break;
		case 80: // M80 - Turn on Power Supply
			// response_ack();
			break;
		case 81: // M81 - Turn off Power Supply
			// response_ack();
			break;
		case 17:
			enable_x();
			enable_y();
			enable_z();
			enable_e();
			// response_ack();
			break;
		case 18:
		case 84: // M84 Disable steppers until next move
			disable_axis = 0;
			if (code_seen(axis_codes[X_AXIS])) disable_axis |= (1 << X_AXIS);
			if (code_seen(axis_codes[Y_AXIS])) disable_axis |= (1 << Y_AXIS);
			if (code_seen(axis_codes[Z_AXIS])) disable_axis |= (1 << Z_AXIS);
			if (code_seen(axis_codes[E_AXIS])) disable_axis |= (1 << E_AXIS);
			if (disable_axis == 0)
				disable_axis = (1 << X_AXIS) | (1 << Y_AXIS) | (1 << Z_AXIS) | (1 << E_AXIS);
			sys_state = SYS_STATE_DISABLE_MOTOR;
			/*if(code_seen('S')){
				//stepper_inactive_time = code_value() * 1000;
			} else {
				bool all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])) || (code_seen(axis_codes[Z_AXIS]))
				               || (code_seen(axis_codes[E_AXIS])));
				if(all_axis) {
					//st_synchronize();
					//disable_e0();
					//disable_e1();
					//disable_e2();
					//finishAndDisableSteppers();
				} else {
					//st_synchronize();
					//if(code_seen('X')) disable_x();
					//if(code_seen('Y')) disable_y();
					//if(code_seen('Z')) disable_z();
					//if(code_seen('E')) {
						//disable_e0();
						//disable_e1();
						//disable_e2();
					//}
				}
			}*/
			// response_ack();
			return;
		case 201: // M201
			for(int i=0; i < machine->num_axis; i++)
			{
				if(code_seen(axis_codes[i]))
				{
					max_acceleration_units_per_sq_second[i] = code_value();
				}
			}
			// steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
			reset_acceleration_rates();
		
			break;
		case 203: // M203 max feedrate mm/sec
			for(int i=0; i < machine->num_axis; i++) {
				if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
			}
			break;
		case 204: // M204 acclereration S normal moves T filmanent only moves
			if(code_seen('S')) acceleration = code_value();
			//if(code_seen('T')) retract_acceleration = code_value();
			break;
		case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
			if(code_seen('S')) minimumfeedrate = code_value();
			if(code_seen('T')) mintravelfeedrate = code_value();
			//if(code_seen('B')) minsegmenttime = code_value() ;
			if(code_seen('X')) max_xy_jerk = code_value() ;
			if(code_seen('Z')) max_z_jerk = code_value() ;
			if(code_seen('E')) max_e_jerk = code_value() ;
			break;
		case 206: // M206 additional homeing offset
			for(int i=0; i < 3; i++)
			{
				if(code_seen(axis_codes[i])) add_homeing[i] = code_value();
			}
			break;
		case 220: // M220 S<factor in percent>- set speed factor override percentage
			if(code_seen('S')) feedmultiply = code_value() ;
		break;
		case 999: // M999: Restart after being stopped

			break;
		case 8600:
			if (code_seen('E')) select_table_e = code_value();
			if (code_seen('D')) select_table_d = code_value();
			sys_state = SYS_STATE_SELECT_TEMP_TABLE;
			return;
		case 8601:
			create_table_d = 0;
			create_table_n = 36;
			create_table_m = 350.0;
			create_table_r = 10000.0;
			
			// default 104GT thermistor
			create_table_i = 20.0;
			create_table_j = 150.0;
			create_table_k = 250.0;
			create_table_u = 126800.0;
			create_table_v = 1360.0;
			create_table_w = 174.0;
			
			if (code_seen('D')) create_table_d = code_value();
			if (code_seen('S')) create_table_n = code_value();
			if (code_seen('T')) create_table_m = code_value();
			if (code_seen('R')) create_table_r = code_value();
			if (code_seen('I')) create_table_i = code_value();
			if (code_seen('J')) create_table_j = code_value();
			if (code_seen('K')) create_table_k = code_value();
			if (code_seen('U')) create_table_u = code_value();
			if (code_seen('V')) create_table_v = code_value();
			if (code_seen('W')) create_table_w = code_value();
			sys_state = SYS_STATE_CREATE_TEMP_TABLE;
			return;
		default:
			// response_ack();
			break;
		}
	} else 
	if(code_seen('T')) {
		// response_ack();
	/*
		tmp_extruder = code_value();
		if(tmp_extruder >= EXTRUDERS) {
			SERIAL_ECHO_START;
			SERIAL_ECHO("T");
			SERIAL_ECHO(tmp_extruder);
			SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
		} else {
			boolean make_move = false;
			if(code_seen('F')) {
				make_move = true;
				next_feedrate = code_value();
				if(next_feedrate > 0.0) {
					feedrate = next_feedrate;
				}
			}
			if(tmp_extruder != active_extruder) {
				// Save current position to return to after applying extruder offset
				memcpy(destination, current_position, sizeof(destination));
				// Offset extruder (only by XY)
				int i;
				for(i = 0; i < 2; i++) {
					current_position[i] = current_position[i] -
                                 extruder_offset[i][active_extruder] +
                                 extruder_offset[i][tmp_extruder];
				}
				// Set the new active extruder and position
				active_extruder = tmp_extruder;
				plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
				// Move to the old position if 'F' was in the parameters
				if(make_move && Stopped == false) {
					prepare_move();
				}
			}

			SERIAL_ECHO_START;
			SERIAL_ECHO(MSG_ACTIVE_EXTRUDER);
			SERIAL_PROTOCOLLN((int)active_extruder);
		}*/
	} else {/*
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
		SERIAL_ECHO(cmdbuffer[bufindr]);
		SERIAL_ECHOLNPGM("\"");*/
	}
	clock--;
	}
}

//init function

void Config_ResetDefault() {
    double tmp1[4]={machine->qx_steps_per_mm, machine->qy_steps_per_mm, machine->qz_steps_per_mm, machine->qe_steps_per_mm};
    double tmp2[4]={machine->x_max_feedrate_mps, machine->y_max_feedrate_mps, machine->z_max_feedrate_mps, machine->e_max_feedrate_mps};
    double tmp3[4]={machine->qx_max_acceleration, machine->qy_max_acceleration, machine->qz_max_acceleration, machine->qe_max_acceleration};
    for (short i=0;i<4;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration = machine->default_acceleration;
    retract_acceleration = machine->default_retract_acceleration;
    minimumfeedrate = machine->default_minimumfeedrate;
    //minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate = machine->default_mintravelfeedrate;
    max_xy_jerk = machine->default_xyjerk;
    max_z_jerk = machine->default_zjerk;
    max_e_jerk = machine->default_ejerk;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = 0;
	
	if (tp->autotune_pid_enable != 0)
		sys_state = SYS_STATE_PID_AUTOTUNE;
}

int gc_init(void){
	if((cmdbuffer = (char *)malloc(sizeof(char) * sys->max_cmd_size)) == NULL){
		return -1;
	}
	//axis_codes = (const char *)malloc(sizeof(const char) * machine->num_axis);
	if((destination = (double *)malloc(sizeof(double) * machine->num_axis)) == NULL){
		free(cmdbuffer);
		return -1;
	}
	if((current_position = (double *)malloc(sizeof(double) * machine->num_axis)) == NULL){
		free(destination);
		free(cmdbuffer);
		return -1;
	}
	if((axis_relative_modes = (bool *)malloc(sizeof(bool) * machine->num_axis)) == NULL){
		free(current_position);
		free(destination);
		free(cmdbuffer);		
		return -1;
	}
	if((homing_feedrate = (double *)malloc(sizeof(double) * machine->num_axis)) == NULL){
		free(axis_relative_modes);
		free(current_position);
		free(destination);
		free(cmdbuffer);		
		return -1;
	}
	if((eqnAMatrix = (double *)malloc(sizeof(double) * level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points*3)) == NULL){
		free(homing_feedrate);
		free(axis_relative_modes);
		free(current_position);
		free(destination);
		free(cmdbuffer);		
		return -1;
	}
	if((eqnBVector = (double *)malloc(sizeof(double) * level->auto_bed_leveling_grid_points*level->auto_bed_leveling_grid_points)) == NULL){
		free(eqnAMatrix);
		free(homing_feedrate);
		free(axis_relative_modes);
		free(current_position);
		free(destination);
		free(cmdbuffer);		
		return -1;
	}
	
	//axis_codes[X_AXIS] = (char)'X';
	//axis_codes[Y_AXIS] = (char)'Y';
	//axis_codes[Z_AXIS] = (char)'Z';
	//axis_codes[E_AXIS] = (char)'E';
	destination[X_AXIS] = destination[Y_AXIS] = destination[Z_AXIS] = destination[E_AXIS] = 0.0;
	current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = current_position[E_AXIS] = 0.0;
	axis_relative_modes[X_AXIS] = axis_relative_modes[1] = axis_relative_modes[Z_AXIS] = axis_relative_modes[E_AXIS] = false;
	homing_feedrate[X_AXIS] = machine->x_homing_feedrate_mpm*60;
	homing_feedrate[Y_AXIS] = machine->y_homing_feedrate_mpm*60;
	homing_feedrate[Z_AXIS] = machine->z_homing_feedrate_mpm*60;
	homing_feedrate[E_AXIS] = 0;

	min_pos[0] = machine->x_min_pos_mm;
	min_pos[1] = machine->y_min_pos_mm;
	min_pos[2] = machine->z_min_pos_mm;
	max_pos[0] = machine->x_max_pos_mm;
	max_pos[1] = machine->y_max_pos_mm;
	max_pos[2] = machine->z_max_pos_mm;
	return 0;
}




