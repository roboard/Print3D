#ifndef planner_h
#define planner_h

#include "config.h"
#include "vector_3.h"
#include "global_setting.h"

// void MotorEnable(int axis);
// void MotorDisable(int axis);
// #define enable_x()  MotorEnable(X_AXIS)
// #define enable_y()  MotorEnable(Y_AXIS)
// #define enable_z()  MotorEnable(Z_AXIS)
// #define enable_e0()  MotorEnable(E_AXIS)
// #define enable_e1()  MotorEnable(E_AXIS+1)
// #define disable_x() MotorDisable(X_AXIS)
// #define disable_y() MotorDisable(Y_AXIS)
// #define disable_z() MotorDisable(Z_AXIS)
// #define disable_e0() MotorDisable(E_AXIS)
// #define disable_e1() MotorDisable(E_AXIS+1)

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))

void MotorEnable(int axis);
void MotorDisable(int axis);

#define enable_x()  MotorEnable(X_AXIS)
#define enable_y()  MotorEnable(Y_AXIS)
#define enable_z()  MotorEnable(Z_AXIS)
#define enable_e()  MotorEnable(E_AXIS)
#define disable_x() MotorDisable(X_AXIS)
#define disable_y() MotorDisable(Y_AXIS)
#define disable_z() MotorDisable(Z_AXIS)
#define disable_e() MotorDisable(E_AXIS)

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
	// Fields used by the bresenham algorithm for tracing the line
	long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
	unsigned long step_event_count;           // The number of step events required to complete this block
	long accelerate_until;                    // The index of the step event on which to stop acceleration
	long decelerate_after;                    // The index of the step event on which to start decelerating
	//long acceleration_rate;                   // The acceleration rate used for acceleration calculation
	unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	unsigned char active_extruder;            // Selects the active extruder


	// Fields used by the motion planner to manage acceleration
	double nominal_speed;                               // The nominal speed for this block in mm/sec 
	double entry_speed;                                 // Entry speed at previous-current junction in mm/sec
	double max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
	double millimeters;                                 // The total travel of this block in mm
	double acceleration;                                // acceleration mm/sec^2
	unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
	unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached

	// Settings for the trapezoid generator
	unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec 
	unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block  
	unsigned long final_rate;                          // The minimal rate at exit
	unsigned long acceleration_st;                     // acceleration steps/sec^2
	unsigned long fan_speed;
	volatile char busy;
} block_t;

// Initialize the motion plan subsystem      
void plan_init();
void plan_close();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion.

void plan_buffer_line(double x, double y, double z, const double &e, double feed_rate, const char &extruder);

// Get the position applying the bed level matrix if enabled
vector_3 plan_get_position();

// Set position. Used for G92 instructions.
void plan_set_position(double x, double y, double z, const double &e);

void plan_set_e_position(const double &e);

void check_axes_activity();

extern machine_t *machine;
extern double *max_feedrate; // set the max speeds
extern double *axis_steps_per_unit;
extern double *max_acceleration_units_per_sq_second; // Use M201 to override by software
extern double minimumfeedrate;
extern double acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern double retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern double max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
extern double max_z_jerk;
extern double max_e_jerk;
extern double mintravelfeedrate;
extern unsigned long *axis_steps_per_sqr_second;


extern block_t *block_buffer;            // A ring buffer for motion instfructions
extern volatile unsigned long block_buffer_head;           // Index of the next block to be pushed
extern volatile unsigned long block_buffer_tail; 
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    

unsigned long get_block_num();

void plan_discard_current_block();

block_t *plan_get_current_block();

//bool blocks_queued();

void reset_acceleration_rates();

unsigned long next_block_index(unsigned long block_index);

bool plan_buffer_null();

// this holds the required transform to compensate for bed level
extern matrix_3x3 plan_bed_level_matrix;


#endif
