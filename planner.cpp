/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */
#include "dmpcfg.h"
#include "planner.h"
#include "io.h"
#include "stepper.h"

#define GPCFG_ADDR       (0xF100)
#define GPBAS_ADDR       (0xF200)

static unsigned long old_gpio_enable;

static bool GPIO_Inited = false;
static unsigned short GPCfgAD;

static unsigned short EnableDataAD[4];
static unsigned short EnableDirAD[4];
static int EnablePin[4];// = {pins->x_enable_pin, pins->y_enable_pin, pins->z_enable_pin, pins->e_enable_pin};

//static unsigned short EnableDataAD[5];
//static unsigned short EnableDirAD[5];
//static int EnablePin[5] = {X_ENABLE_PIN, Y_ENABLE_PIN, Z_ENABLE_PIN, E0_ENABLE_PIN, E1_ENABLE_PIN};


static __inline__ bool GPIOInit(void)
{
  int i, port;
  unsigned short ports_enable;
  unsigned short temp;
  
  /* Configuration south-bridge GPIO config address */
  if (io_Init() == false || GPIO_Inited == true)
    return false;
  
  if ((GPCfgAD = sb_Read16(0x62) & 0xFFFE) == 0x0000)
    sb_Write16(0x62, sb_Read16(0x62) | (GPCfgAD = GPCFG_ADDR));
  sb_Write16(0x62, sb_Read16(0x62) | 0x0001);
	
  io_Close();

  /* Enable GPIO port */
  ports_enable = (1 << pins->x_enable_port)
               | (1 << pins->y_enable_port)
			   | (1 << pins->z_enable_port)
			   | (1 << pins->e_enable_port);
//  ports_enable = (1 << X_ENABLE_PORT)
//               | (1 << Y_ENABLE_PORT)
//			   | (1 << Z_ENABLE_PORT)
//			   | (1 << E0_ENABLE_PORT)
//			   | (1 << E1_ENABLE_PORT);
  
  old_gpio_enable = io_inpdw(GPCfgAD + 0x00);
  io_outpdw(GPCfgAD + 0x00, old_gpio_enable | ports_enable);

  /* Configuration GPIO DATA & DIR address */
  port = 0;
  while (ports_enable != 0)
  {
    if (ports_enable & 0x01)
	{
	  /* Set DATA address */
	  temp = io_inpw(GPCfgAD + 4 + 4*port);
	  if (temp == 0x0000 || temp == 0xFFFF)
	    io_outpw(GPCfgAD + 4 + 4*port, GPBAS_ADDR + port);
	  
	  /* Set DIR address */
	  temp = io_inpw(GPCfgAD + 6 + 4*port);
	  if (temp == 0x0000 || temp == 0xFFFF)
	    io_outpw(GPCfgAD + 6 + 4*port, GPBAS_ADDR + port + 10);
	}
	
	port++;
	ports_enable = ports_enable >> 1;
  }
  
  EnableDataAD[0] = io_inpw(GPCfgAD + 4 + 4*pins->x_enable_port);
  EnableDirAD[0]  = io_inpw(GPCfgAD + 6 + 4*pins->x_enable_port);
  EnableDataAD[1] = io_inpw(GPCfgAD + 4 + 4*pins->y_enable_port);
  EnableDirAD[1]  = io_inpw(GPCfgAD + 6 + 4*pins->y_enable_port);
  EnableDataAD[2] = io_inpw(GPCfgAD + 4 + 4*pins->z_enable_port);
  EnableDirAD[2]  = io_inpw(GPCfgAD + 6 + 4*pins->z_enable_port);
  EnableDataAD[3] = io_inpw(GPCfgAD + 4 + 4*pins->e_enable_port);
  EnableDirAD[3]  = io_inpw(GPCfgAD + 6 + 4*pins->e_enable_port);
  
  io_outpb(EnableDirAD[0], io_inpb(EnableDirAD[0]) | (1 << pins->x_enable_pin));
  io_outpb(EnableDirAD[1], io_inpb(EnableDirAD[1]) | (1 << pins->y_enable_pin));
  io_outpb(EnableDirAD[2], io_inpb(EnableDirAD[2]) | (1 << pins->z_enable_pin));
  io_outpb(EnableDirAD[3], io_inpb(EnableDirAD[3]) | (1 << pins->e_enable_pin));

  io_outpb(EnableDataAD[0], io_inpb(EnableDataAD[0]) | (1 << pins->x_enable_pin));
  io_outpb(EnableDataAD[1], io_inpb(EnableDataAD[1]) | (1 << pins->y_enable_pin));
  io_outpb(EnableDataAD[2], io_inpb(EnableDataAD[2]) | (1 << pins->z_enable_pin));
  io_outpb(EnableDataAD[3], io_inpb(EnableDataAD[3]) | (1 << pins->e_enable_pin));
  
  // EnableDataAD[0] = io_inpw(GPCfgAD + 4 + 4*X_ENABLE_PORT);
  // EnableDirAD[0]  = io_inpw(GPCfgAD + 6 + 4*X_ENABLE_PORT);
  // EnableDataAD[1] = io_inpw(GPCfgAD + 4 + 4*Y_ENABLE_PORT);
  // EnableDirAD[1]  = io_inpw(GPCfgAD + 6 + 4*Y_ENABLE_PORT);
  // EnableDataAD[2] = io_inpw(GPCfgAD + 4 + 4*Z_ENABLE_PORT);
  // EnableDirAD[2]  = io_inpw(GPCfgAD + 6 + 4*Z_ENABLE_PORT);
  // EnableDataAD[3] = io_inpw(GPCfgAD + 4 + 4*E0_ENABLE_PORT);
  // EnableDirAD[3]  = io_inpw(GPCfgAD + 6 + 4*E0_ENABLE_PORT);
  // EnableDataAD[4] = io_inpw(GPCfgAD + 4 + 4*E1_ENABLE_PORT);
  // EnableDirAD[4]  = io_inpw(GPCfgAD + 6 + 4*E1_ENABLE_PORT);
  
  // io_outpb(EnableDirAD[0], io_inpb(EnableDirAD[0]) | (1 << X_ENABLE_PIN));
  // io_outpb(EnableDirAD[1], io_inpb(EnableDirAD[1]) | (1 << Y_ENABLE_PIN));
  // io_outpb(EnableDirAD[2], io_inpb(EnableDirAD[2]) | (1 << Z_ENABLE_PIN));
  // io_outpb(EnableDirAD[3], io_inpb(EnableDirAD[3]) | (1 << E0_ENABLE_PIN));
  // io_outpb(EnableDirAD[4], io_inpb(EnableDirAD[4]) | (1 << E1_ENABLE_PIN));

  // io_outpb(EnableDataAD[0], io_inpb(EnableDataAD[0]) | (1 << X_ENABLE_PIN));
  // io_outpb(EnableDataAD[1], io_inpb(EnableDataAD[1]) | (1 << Y_ENABLE_PIN));
  // io_outpb(EnableDataAD[2], io_inpb(EnableDataAD[2]) | (1 << Z_ENABLE_PIN));
  // io_outpb(EnableDataAD[3], io_inpb(EnableDataAD[3]) | (1 << E0_ENABLE_PIN));
  // io_outpb(EnableDataAD[4], io_inpb(EnableDataAD[4]) | (1 << E1_ENABLE_PIN));

//LCD init
  old_gpio_enable = io_inpdw(GPCfgAD + 0x00);
  io_outpdw(GPCfgAD + 0x00, old_gpio_enable | 0x80);//port7 enable
  io_outpdw(0xf100 + 8*4,((0xf202 + 7*4)<<16) + 0xf200 + 7*4);//set data, dir addr
  io_outpb(0xF21E, io_inpb(0xF21E) | 0x70);//set dir
//end LCD init
  
  GPIO_Inited = true;
  
  return true;
}

static __inline__ void GPIOClose(void)
{
  if (GPIO_Inited == false) return;
  
  io_outpdw(GPCfgAD + 0x00, old_gpio_enable);
  
  GPIO_Inited = false;
}

void MotorEnable(int axis)
{
  io_outpb(EnableDataAD[axis], io_inpb(EnableDataAD[axis]) & ~(1 << EnablePin[axis]));
}

void MotorDisable(int axis)
{
  io_outpb(EnableDataAD[axis], io_inpb(EnableDataAD[axis]) | (1 << EnablePin[axis]));
}

static __inline__ long lround(double num) {
	long data;
	if(num >= 0) {
		data = (long)(num + 0.5);
	} else {
		data = (long)(num - 0.5);
	}
	return data;
}


//===========================================================================
//=============================public variables ============================
//===========================================================================
//TODO:
double *max_feedrate; // set the max speeds
double *axis_steps_per_unit;
double *max_acceleration_units_per_sq_second; // Use M201 to override by software
double minimumfeedrate;
double acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
double retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
double max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
double max_z_jerk;
double max_e_jerk;
double mintravelfeedrate;
unsigned long *axis_steps_per_sqr_second;

// The current position of the tool in absolute steps
long position[4];   //rescaled from extern when axis_steps_per_unit are changed by gcode
static double previous_speed[4]; // Speed of previous path line segment
static double previous_nominal_speed; // Nominal speed of previous path line segment

// this holds the required transform to compensate for bed level
matrix_3x3 plan_bed_level_matrix = {
	1.0, 0.0, 0.0,
	0.0, 1.0, 0.0,
	0.0, 0.0, 1.0,
};

//===========================================================================
//=================semi-private variables, used in inline  functions    =====
//===========================================================================
block_t *block_buffer;            					// A ring buffer for motion instfructions
volatile unsigned long block_buffer_head;           // Index of the next block to be pushed
volatile unsigned long block_buffer_tail;           // Index of the block to process now
bool plan_buffer_null()
{
	return (block_buffer_head == block_buffer_tail) ? true : false;
}

//===========================================================================
//=============================private variables ============================
//===========================================================================

//TODO:
//эΘdefine
const unsigned int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement
int extrudemultiply=100;
//挡эΘdefine

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.

static __inline__ int init_block_buffer() {
	block_buffer = (block_t *)malloc(sizeof(block_t) * sys->block_buffer_size);
	if(block_buffer == NULL) {
		//error
		return -1;
	}
	return 0;	
}

static __inline__ void free_block_buffer() {
	if (block_buffer) free(block_buffer);
	block_buffer = NULL;
}

unsigned long next_block_index(unsigned long block_index) {
// static unsigned long next_block_index(unsigned long block_index) {
	block_index++;
	if (block_index == sys->block_buffer_size) { 
		block_index = 0; 
	}
	return(block_index);
}

// Returns the index of the previous block in the ring buffer
static __inline__ unsigned long prev_block_index(unsigned long block_index) {
	if (block_index == 0) { 
		block_index = sys->block_buffer_size; 
	}
	block_index--;
	return(block_index);
}

unsigned long get_block_num() {
	unsigned long temp;
	if(block_buffer_head >= block_buffer_tail) {
		temp = block_buffer_head - block_buffer_tail;
	} else {
		// temp = block_buffer_head + sys->block_buffer_size - block_buffer_tail;
		temp = block_buffer_head + sys->block_buffer_size - block_buffer_tail + 1;
	}
	return(temp);
}

void plan_discard_current_block() {
	if (block_buffer_head != block_buffer_tail) {
		block_buffer_tail = (block_buffer_tail + 1) & (sys->block_buffer_size - 1);  
	}
}

// Gets the current block. Returns NULL if buffer empty
block_t *plan_get_current_block() {
	if (block_buffer_head == block_buffer_tail) { 
		return(NULL); 
	}
	block_t *block = &block_buffer[block_buffer_tail];
	block->busy = true;
	return(block);
}



// Gets the current block. Returns NULL if buffer empty
//bool blocks_queued() {
//	if (block_buffer_head == block_buffer_tail) { 
//		return false; 
//	} else {
//		return true;
//	}
//}

//===========================================================================
//=============================functions         ============================
//===========================================================================

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
static __inline__ double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
	if (acceleration!=0) {
		return((target_rate*target_rate-initial_rate*initial_rate)/(2.0*acceleration));
	} else {
		return 0.0;  // acceleration was 0, set acceleration distance to 0
	}
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

static __inline__ double intersection_distance(double initial_rate, double final_rate, double acceleration, double distance) {
	if (acceleration!=0) {
		return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/(4.0*acceleration) );
	} else {
		return 0.0;  // acceleration was 0, set intersection distance to 0
	}
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

static __inline__ void calculate_trapezoid_for_block(block_t *block, double entry_factor, double exit_factor) {
	unsigned long initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
	unsigned long final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)
	
	// Limit minimal step rate (Otherwise the timer will overflow.)
	if(initial_rate <120) {
		initial_rate=120; 
	}
	if(final_rate < 120) {
		final_rate=120;  
	}

	long acceleration = block->acceleration_st;
	int accelerate_steps = ceil(estimate_acceleration_distance(initial_rate, block->nominal_rate, acceleration));
	int decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, final_rate, -acceleration));

	// Calculate the size of Plateau of Nominal Rate.
	int plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

	// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
	// have to use intersection_distance() to calculate when to abort acceleration and start braking
	// in order to reach the final_rate exactly at the end of this block.
	if (plateau_steps < 0) {
		accelerate_steps = ceil(intersection_distance(initial_rate, final_rate, acceleration, block->step_event_count));
		accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
		accelerate_steps = min((unsigned int)accelerate_steps,block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
		plateau_steps = 0;
	}

	// block->accelerate_until = accelerate_steps;
	// block->decelerate_after = accelerate_steps+plateau_steps;
	//CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
	if(block->busy == false) { // Don't update variables if block is busy.
		block->accelerate_until = accelerate_steps;
		block->decelerate_after = accelerate_steps+plateau_steps;
		block->initial_rate = initial_rate;
		block->final_rate = final_rate;
	}

  //CRITICAL_SECTION_END;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
//TODO:
static __inline__ double max_allowable_speed(double acceleration, double target_velocity, double distance) {
	return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}


// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
static __inline__ void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
	if(!current) { 
		return; 
	}
	if (next) {
		// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
		// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
		// check for maximum allowable speed reductions to ensure maximum possible planned speed.
		if (current->entry_speed != current->max_entry_speed) {

		// If nominal length true, max junction speed is guaranteed to be reached. Only compute
		// for max allowable speed if block is decelerating and nominal length is false.
			if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
				current->entry_speed = min( current->max_entry_speed,
				max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
			} else {
				current->entry_speed = current->max_entry_speed;
			}
			current->recalculate_flag = true;
		}
	} // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
static __inline__ void planner_reverse_pass() {
	unsigned long block_index = block_buffer_head;
	unsigned long tail = block_buffer_tail;
  
	if(((block_buffer_head-tail + sys->block_buffer_size) & (sys->block_buffer_size - 1)) > 3) {
		block_index = (block_buffer_head - 3) & (sys->block_buffer_size - 1);
		block_t *block[3] = { NULL, NULL, NULL};
		while(block_index != tail) { 
			block_index = prev_block_index(block_index); 
			block[2]= block[1];
			block[1]= block[0];
			block[0] = &block_buffer[block_index];
			planner_reverse_pass_kernel(block[0], block[1], block[2]);
		}
	}
}

// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
static __inline__ void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
	if(!previous) { 
		return; 
	}

	// If the previous block is an acceleration block, but it is not long enough to complete the
	// full speed change within the block, we need to adjust the entry speed accordingly. Entry
	// speeds have already been reset, maximized, and reverse planned by reverse planner.
	// If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
	if (!previous->nominal_length_flag) {
		if (previous->entry_speed < current->entry_speed) {
			double entry_speed = min( current->entry_speed,
			max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

			// Check for junction speed change
			if (current->entry_speed != entry_speed) {
				current->entry_speed = entry_speed;
				current->recalculate_flag = true;
			}
		}
	}
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
static __inline__ void planner_forward_pass() {
	unsigned long block_index = block_buffer_tail;
	block_t *block[3] = {NULL, NULL, NULL};

	while(block_index != block_buffer_head) {
		block[0] = block[1];
		block[1] = block[2];
		block[2] = &block_buffer[block_index];
		planner_forward_pass_kernel(block[0],block[1],block[2]);
		block_index = next_block_index(block_index);
	}
	planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
static __inline__ void planner_recalculate_trapezoids() {
	unsigned long block_index = block_buffer_tail;
	block_t *current;
	block_t *next = NULL;

	while(block_index != block_buffer_head) {
		current = next;
		next = &block_buffer[block_index];
		if (current) {
			// Recalculate if current block entry or exit junction speed has changed.
			if (current->recalculate_flag || next->recalculate_flag) {
				// NOTE: Entry and exit factors always > 0 by all previous logic operations.
				calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed, next->entry_speed/current->nominal_speed);
				current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
			}	
		}	
		block_index = next_block_index( block_index );	
	}
	// Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
	if(next != NULL) {
		calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed, machine->minimum_planner_speed/next->nominal_speed);
		next->recalculate_flag = false;
	}
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

static __inline__ void planner_recalculate() {   
	planner_reverse_pass();
	planner_forward_pass();
	planner_recalculate_trapezoids();
}

void plan_init() {
//
	max_feedrate = (double *)malloc(sizeof(double) * machine->num_axis);
	axis_steps_per_unit = (double *)malloc(sizeof(double) * machine->num_axis);
	max_acceleration_units_per_sq_second = (double *)malloc(sizeof(double) * machine->num_axis);
	axis_steps_per_sqr_second = (unsigned long *)malloc(sizeof(unsigned long) * machine->num_axis);
//

	init_block_buffer();
	block_buffer_head = 0;
	block_buffer_tail = 0;
	memset(position, 0, sizeof(position)); // clear position
	previous_speed[0] = 0.0;
	previous_speed[1] = 0.0;
	previous_speed[2] = 0.0;
	previous_speed[3] = 0.0;
	previous_nominal_speed = 0.0;
	
	EnablePin[0] = pins->x_enable_pin;
	EnablePin[1] = pins->y_enable_pin;
	EnablePin[2] = pins->z_enable_pin;
	EnablePin[3] = pins->e_enable_pin;
	GPIOInit();
}

void plan_close() {
	disable_x();
	disable_y();
	disable_z();
	disable_e();
	//disable_e0();
	//disable_e1();
	GPIOClose();
	free_block_buffer();
}
//TODO: clear parameter
// double junction_deviation = 0.1;

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.

void plan_buffer_line(double x, double y, double z, const double &e, double feed_rate, const char &extruder)
  {
	// Calculate the buffer head after we push this byte
	unsigned long next_buffer_head = next_block_index(block_buffer_head);

	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Rest here until there is room in the buffer.
	if(block_buffer_tail == next_buffer_head) {
		return;
	}
	
	if (level->enable_auto_bed_leveling != 0)
		apply_rotation_xyz(plan_bed_level_matrix, x, y, z);

	// The target position of the tool in absolute steps
	// Calculate target position in absolute steps
	//this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
	long target[4];
	target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
	target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
	target[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
	target[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);
	// Prepare to set up new block
	block_t *block = &block_buffer[block_buffer_head];

	// Mark block as not busy (Not executed by the stepper interrupt)
	block->busy = false;

	// Number of steps for each axis
	if(machine->type == H_BOT) {
		block->steps_x = labs((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]));
		block->steps_y = labs((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]));	
	} else if(machine->type == DELTA) {
		//干
	} else {
		block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
		block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);	
	}

	block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
	block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
	block->steps_e *= extrudemultiply;
	block->steps_e /= 100;
	block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

	// Bail if this is a zero-length block
	if (block->step_event_count <= dropsegments) { 
		return; 
	}

	// Compute direction bits for this block 
	block->direction_bits = 0;
	
	if(machine->type == H_BOT) {
		if ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]) < 0) {
		// if ((-(target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS])) < 0) {
			block->direction_bits |= (1<<X_AXIS); 
		}
		if ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]) < 0) {
		// if ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]) > 0) {
			block->direction_bits |= (1<<Y_AXIS); 
		}
	} else if(machine->type == DELTA) {
		//干
	} else {
		if (target[X_AXIS] < position[X_AXIS]) {
			block->direction_bits |= (1<<X_AXIS); 
		}
		if (target[Y_AXIS] < position[Y_AXIS]) {
			block->direction_bits |= (1<<Y_AXIS); 
		}	
	}	
	
	if (target[Z_AXIS] < position[Z_AXIS]) {
		block->direction_bits |= (1<<Z_AXIS); 
	}
	if (target[E_AXIS] < position[E_AXIS]) {
		block->direction_bits |= (1<<E_AXIS); 
	}

	block->active_extruder = extruder;

	//enable active axes
	if(machine->type == H_BOT) {
		if((block->steps_x != 0) || (block->steps_y != 0)) {
			enable_x();
			enable_y();
		}
	} else if(machine->type == DELTA) {
		//干
	} else {
		if(block->steps_x != 0) enable_x();
		if(block->steps_y != 0) enable_y();
	}
	if(block->steps_z != 0) enable_z();
	// Enable all
	if(block->steps_e != 0) {
		enable_e();
		//enable_e0();
		//enable_e1();
		//enable_e2(); 
	}

	if (block->steps_e == 0) {
		if(feed_rate<mintravelfeedrate) feed_rate=mintravelfeedrate;
	} else {
		if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;
	} 

	double delta_mm[4];
	if(machine->type == H_BOT) {
		delta_mm[X_AXIS] = ((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[X_AXIS];
		delta_mm[Y_AXIS] = ((target[X_AXIS]-position[X_AXIS]) - (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[Y_AXIS];
		// delta_mm[X_AXIS] = ((-(target[X_AXIS]-position[X_AXIS])) + (target[Y_AXIS]-position[Y_AXIS]))/axis_steps_per_unit[X_AXIS];
		// delta_mm[Y_AXIS] = (-((target[X_AXIS]-position[X_AXIS]) + (target[Y_AXIS]-position[Y_AXIS])))/axis_steps_per_unit[Y_AXIS];
	} else if(machine->type == DELTA) {
		//干
	} else {
		delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
		delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
	}	
	delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/axis_steps_per_unit[Z_AXIS];
	delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0;
	if ( block->steps_x <=dropsegments && block->steps_y <=dropsegments && block->steps_z <=dropsegments ) {
		block->millimeters = fabs(delta_mm[E_AXIS]);
	} else {
		block->millimeters = sqrt(pow(delta_mm[X_AXIS], 2) + pow(delta_mm[Y_AXIS], 2) + pow(delta_mm[Z_AXIS], 2));
	}
	double inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 

    // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
	double inverse_second = feed_rate * inverse_millimeters;

	int moves_queued=(block_buffer_head-block_buffer_tail + sys->block_buffer_size) & (sys->block_buffer_size - 1);

	block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
	block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

	// Calculate and limit speed in mm/sec for each axis
	double current_speed[4];
	double speed_factor = 1.0; //factor <=1 do decrease speed
	for(int i=0; i < 4; i++) {
		current_speed[i] = delta_mm[i] * inverse_second;
		if(fabs(current_speed[i]) > max_feedrate[i]) {
			speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
		}
	}

	// Correct the speed  
	if( speed_factor < 1.0) {
		for(unsigned char i=0; i < 4; i++) {
			current_speed[i] *= speed_factor;
		}
		block->nominal_speed *= speed_factor;
		block->nominal_rate *= speed_factor;
	}

	// Compute and limit the acceleration rate for the trapezoid generator.  
	double steps_per_mm = block->step_event_count/block->millimeters;
	if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0) {
		block->acceleration_st = ceil(retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
	} else {
		block->acceleration_st = ceil(acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
		// Limit acceleration per axis
		if(((double)block->acceleration_st * (double)block->steps_x / (double)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
		if(((double)block->acceleration_st * (double)block->steps_y / (double)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
		if(((double)block->acceleration_st * (double)block->steps_e / (double)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
		if(((double)block->acceleration_st * (double)block->steps_z / (double)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
	}
	block->acceleration = block->acceleration_st / steps_per_mm;
	//block->acceleration_rate = (long)((double)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

	// Start with a safe speed
	double vmax_junction = max_xy_jerk/2; 
	double vmax_junction_factor = 1.0; 
	if(fabs(current_speed[Z_AXIS]) > max_z_jerk/2) 
		vmax_junction = min(vmax_junction, max_z_jerk/2);
	if(fabs(current_speed[E_AXIS]) > max_e_jerk/2) 
		vmax_junction = min(vmax_junction, max_e_jerk/2);
	vmax_junction = min(vmax_junction, block->nominal_speed);
	double safe_speed = vmax_junction;

	if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
		double jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
		//    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
		vmax_junction = block->nominal_speed;
		//    }
		if (jerk > max_xy_jerk) {
			vmax_junction_factor = (max_xy_jerk/jerk);
		} 
		if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > max_z_jerk) {
			vmax_junction_factor= min(vmax_junction_factor, (max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
		} 
		if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > max_e_jerk) {
			vmax_junction_factor = min(vmax_junction_factor, (max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
		} 
		vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
	}
	block->max_entry_speed = vmax_junction;

	// Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
	double v_allowable = max_allowable_speed(-block->acceleration,machine->minimum_planner_speed,block->millimeters);
	block->entry_speed = min(vmax_junction, v_allowable);

	// Initialize planner efficiency flags
	// Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
	// If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
	// the current block and next block junction speeds are guaranteed to always be at their maximum
	// junction speeds in deceleration and acceleration, respectively. This is due to how the current
	// block nominal speed limits both the current and next maximum junction speeds. Hence, in both
	// the reverse and forward planners, the corresponding block junction speed will always be at the
	// the maximum junction speed and may always be ignored for any speed reduction checks.
	if (block->nominal_speed <= v_allowable) { 
		block->nominal_length_flag = true; 
	} else { 
		block->nominal_length_flag = false; 
	}
	block->recalculate_flag = true; // Always calculate trapezoid for new block

	// Update previous path unit_vector and nominal speed
	memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
	previous_nominal_speed = block->nominal_speed;

	calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed, safe_speed/block->nominal_speed);

	// Move buffer head
	block_buffer_head = next_buffer_head;

	// Update position
	memcpy(position, target, sizeof(target)); // position[] = target[]

	planner_recalculate();
}

void plan_set_position(double x, double y, double z, const double &e)
{
	if (level->enable_auto_bed_leveling != 0)
		apply_rotation_xyz(plan_bed_level_matrix, x, y, z);

	position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
	position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
	position[Z_AXIS] = lround(z*axis_steps_per_unit[Z_AXIS]);     
	position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
	st_set_position(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], position[E_AXIS]);
	previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
	previous_speed[0] = 0.0;
	previous_speed[1] = 0.0;
	previous_speed[2] = 0.0;
	previous_speed[3] = 0.0;
}

void plan_set_e_position(const double &e) {
	position[E_AXIS] = lround(e*axis_steps_per_unit[E_AXIS]);  
	st_set_e_position(position[E_AXIS]);
}

// Calculate the steps/s^2 acceleration rates, based on the mm/s^s
void reset_acceleration_rates() {
	for(int i=0; i < machine->num_axis; i++) {
        axis_steps_per_sqr_second[i] = (unsigned long)(max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i]);
    }
}

vector_3 plan_get_position() {
	vector_3 position = vector_3(st_get_position_mm(X_AXIS), st_get_position_mm(Y_AXIS), st_get_position_mm(Z_AXIS));
	
	//position.debug("in plan_get position");
	//plan_bed_level_matrix.debug("in plan_get bed_level");
	matrix_3x3 inverse = matrix_3x3::transpose(plan_bed_level_matrix);
	//inverse.debug("in plan_get inverse");
	position.apply_rotation(inverse);
	//position.debug("after rotation");

	return position;
}
