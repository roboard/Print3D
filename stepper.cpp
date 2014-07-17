#include "dmpcfg.h"
#include "global_setting.h"
#include "stepper.h"
#include "planner.h"
#include "config.h"
#include "mcex.h"
#include "io.h"
#include "irq.h"
//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static unsigned char out_mdata;
static unsigned char out_edata;
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y,
            counter_z,
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block

static unsigned long acceleration_time, deceleration_time;
static unsigned long acc_step_rate; // needed for deccelaration start point
static unsigned long step_nominal;

//volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
//volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

volatile long *count_position;
volatile signed char *count_direction;

//86duino add
typedef struct {
	unsigned char is_header;
	unsigned char motor_val;
	unsigned char extruder_val;
	//for 不同的機器時用的
	unsigned char x_dir;
	unsigned char y_dir;
	unsigned char z_dir;
	//end
	unsigned long period;
	unsigned long duty;
	unsigned long sc;
	unsigned long dda_x;
	unsigned long dda_y;
	unsigned long dda_z;
	unsigned long dda_e;
} stepper_t;

stepper_t *current_stepper; //// A pointer to the stepper currently being traced
unsigned long period;


//===========================================================================
//=============================functions=====================================
//===========================================================================

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

static __inline__ unsigned long calc_vel(unsigned long acc_time, unsigned long acc_st) {
	return ((unsigned long)((double)(acc_time / 10000) * (double)(acc_st / 10000)));
}

static __inline__ unsigned long calc_timer(unsigned long step_rate) {
	unsigned long timer;
	timer = (unsigned long)((double)100000000/step_rate);
	return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
static __inline__ void trapezoid_generator_reset(void) {
	deceleration_time = 0;
	// step_rate to timer interval
	step_nominal = calc_timer(current_block->nominal_rate);
	// make a note of the number of step loops required at nominal speed
	//acc_step_rate = current_block->initial_rate;
	acceleration_time = 0;//calc_timer(acc_step_rate);
	//period = acceleration_time;
}
//queue function
stepper_t *stepper_buffer;        // A ring buffer for stepper instfructions
volatile unsigned long stepper_buffer_head;           // Index of the next stepper to be pushed
volatile unsigned long stepper_buffer_tail;           // Index of the stepper to process now

bool st_buffer_null()
{
	unsigned long head, tail;
	
	io_DisableINT();
	head = stepper_buffer_head;
	tail = stepper_buffer_tail;
	io_RestoreINT();
	
	return (head == tail) ? true : false;
}

static __inline__ int init_stepper_buffer(void) {
	stepper_buffer = (stepper_t *)malloc(sizeof(stepper_t) * sys->stepper_buffer_size);
	if(stepper_buffer == NULL) {
		//error
		return -1;
	}
	return 0;
}
volatile bool in_discard_block_state = false;
volatile int discard_flag = 0;
static __inline__ void st_discard_current_block(void) {
	stepper_t *stepper;
	while (stepper_buffer_head != stepper_buffer_tail ) {
		stepper = &stepper_buffer[stepper_buffer_tail];
		if (stepper->is_header == 1) {
			in_discard_block_state = false;
			break;
		}
		stepper_buffer_tail = (stepper_buffer_tail + 1) & (sys->stepper_buffer_size - 1);
	}
	if(in_discard_block_state == true)
		discard_flag = 1;
}

static __inline__ void plan_discard_current_stepper(void) {
	if (stepper_buffer_head != stepper_buffer_tail) {
		stepper_buffer_tail = (stepper_buffer_tail + 1) & (sys->stepper_buffer_size - 1);  
	}
}

static __inline__ stepper_t *plan_get_current_stepper(void) {
	if (stepper_buffer_head == stepper_buffer_tail) { 
		return(NULL); 
	}
	stepper_t *stepper = &stepper_buffer[stepper_buffer_tail];
	return(stepper);
}

static __inline__ unsigned long next_stepper_index(unsigned long stepper_index) {
	stepper_index++;
	if (stepper_index == sys->stepper_buffer_size) { 
		stepper_index = 0; 
	}
	return(stepper_index);
}

static __inline__ unsigned long prev_stepper_index(unsigned long stepper_index) {
	if (stepper_index == 0) { 
		stepper_index = sys->stepper_buffer_size; 
	}
	stepper_index--;
	return(stepper_index);
}

static __inline__ unsigned long get_stepper_num() {
	unsigned long temp;
	if(stepper_buffer_head >= stepper_buffer_tail) {
		temp = stepper_buffer_head - stepper_buffer_tail;
	} else {
		// temp = stepper_buffer_head + sys->stepper_buffer_size - stepper_buffer_tail;
		temp = stepper_buffer_head + sys->stepper_buffer_size - stepper_buffer_tail + 1;
	}
	return(temp);
}

/*debug 
FILE *fp3;

void sfp_init() {
	fp3 = fopen("stepper.txt", "w");
}

void sfp_close() {
	fclose(fp3);
};
//end debug*/

void stepper(unsigned long clock) {
	while(clock != 0) {
		if(discard_flag == 1) { //碰到limit時，如果還沒解完block的話，直接丟掉整個block
			current_block = NULL;
			plan_discard_current_block();
			io_DisableINT();
			in_discard_block_state = false;
			discard_flag = 0;
			io_RestoreINT();
			return;
		}
		//如果stepper buf的數量足夠且block buf的數量不夠，則跳出。
		//換一個說法，如果stepper buf的量不夠，或stepper buf 量夠且block buf的量也夠，則要做。
		//block buf算是look ahead的量。
		if((get_stepper_num() > sys->stepper_buffer_bound) && (get_block_num() < sys->block_buffer_bound)) {
			return;
		}
		
		//check stepper buf is full?
		unsigned long next_buffer_head = next_stepper_index(stepper_buffer_head);
		if(stepper_buffer_tail == next_buffer_head){ //stepper buf full.
			return;
		}
		stepper_t *stepper = &stepper_buffer[stepper_buffer_head];
		
		// If there is no current block, attempt to pop one from the buffer
		if (current_block == NULL) {
			// Anything in the buffer?
			current_block = plan_get_current_block();
			if (current_block != NULL) {
				current_block->busy = true;
				trapezoid_generator_reset();
				counter_x = -(current_block->step_event_count >> 1);
				counter_y = counter_x;
				counter_z = counter_x;
				counter_e = counter_x;
				step_events_completed = 0;
				stepper->is_header = 1;
				stepper->dda_x = 0;
				stepper->dda_y = 0;
				stepper->dda_z = 0;
				stepper->dda_e = 0;
				
				/*debug 
				fprintf(fp3, "new_block,  initial_rate = %ld, final_rate = %ld, nominal_rate = %ld, acceleration_st = %ld, num = %ld\n", current_block->initial_rate, current_block->final_rate, current_block->nominal_rate, current_block->acceleration_st, current_block->step_event_count);
				fprintf(fp3, "accelerate_until = %ld, decelerate_after = %ld\n\n", current_block->accelerate_until, current_block->decelerate_after);
				//end debug*/
			}
		} else {
			stepper->is_header = 0;
			stepper->dda_x = 0;
			stepper->dda_y = 0;
			stepper->dda_z = 0;
			stepper->dda_e = 0;
		}

		if (current_block != NULL) {
			// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
			out_bits = current_block->direction_bits;
			out_mdata = 0;
			out_edata = 0;
			// Set the direction bits (X_AXIS=A_AXIS and Y_AXIS=B_AXIS for COREXY)
			if((out_bits & (1<<X_AXIS))!=0){  
				if(pins->invert_x_dir) out_mdata &= (unsigned char)(~(1 << pins->x_dir_pin));
				else out_mdata |= 1 << pins->x_dir_pin;
			} else {  
				if(pins->invert_x_dir) out_mdata |= 1 << pins->x_dir_pin;
				else out_mdata &= (unsigned char)(~(1 << pins->x_dir_pin));
			}
			
			if((out_bits & (1<<Y_AXIS))!=0){
				if(pins->invert_y_dir) out_mdata &= (unsigned char)(~(1 << pins->y_dir_pin));
				else out_mdata |= 1 << pins->y_dir_pin;
			} else {
				if(pins->invert_y_dir) out_mdata |= 1 << pins->y_dir_pin;
				else out_mdata &= (unsigned char)(~(1 << pins->y_dir_pin));
			}

			if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
				if(pins->invert_z_dir) out_mdata &= (unsigned char)(~(1 << pins->z_dir_pin));
				else out_mdata |= 1 << pins->z_dir_pin;
			} else { // +direction
				if(pins->invert_z_dir) out_mdata |= 1 << pins->z_dir_pin;
				else out_mdata &= (unsigned char)(~(1 << pins->z_dir_pin));
			}
			
			if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
				if(pins->invert_e_dir) out_edata &= (unsigned char)(~(1 << pins->e_dir_pin));
				else out_edata |= 1 << pins->e_dir_pin;
			} else { // +direction
				if(pins->invert_e_dir) out_edata |= 1 << pins->e_dir_pin;
				else out_edata &= (unsigned char)(~(1 << pins->e_dir_pin));
			}

			//for 取得機器三軸的行進方向，不同機器不一樣，給判斷limit使用
			if(machine->type == H_BOT) {
				if((((out_mdata >> pins->x_dir_pin) & 0x01L)) == 0 
				&& (((out_mdata >> pins->y_dir_pin) & 0x01L)) == 0) {
					stepper->x_dir = 1;
				} else if((((out_mdata >> pins->x_dir_pin) & 0x01L)) == 1 
				       && (((out_mdata >> pins->y_dir_pin) & 0x01L)) == 1) {
					stepper->x_dir = 0;
				}				
				if((((out_mdata >> pins->x_dir_pin) & 0x01L)) == 1 
				&& (((out_mdata >> pins->y_dir_pin) & 0x01L)) == 0) {
					stepper->y_dir = 1;
				} else if((((out_mdata >> pins->x_dir_pin) & 0x01L)) == 0 
				       && (((out_mdata >> pins->y_dir_pin) & 0x01L)) == 1) {
					stepper->y_dir = 0;
				}	
				if((((out_mdata >> pins->z_dir_pin) & 0x01L)) == 1) {
					stepper->z_dir = 1;
				} else {
					stepper->z_dir = 0;
				}				
			} else if(machine->type == DELTA) {
				//待補
			} else {
				if((((out_mdata >> pins->x_dir_pin) & 0x01L)) == 1) {
					stepper->x_dir = 1;
				} else {
					stepper->x_dir = 0;
				}
				if((((out_mdata >> pins->y_dir_pin) & 0x01L)) == 1) {
					stepper->y_dir = 1;
				} else {
					stepper->y_dir = 0;
				}
				if((((out_mdata >> pins->z_dir_pin) & 0x01L)) == 1) {
					stepper->z_dir = 1;
				} else {
					stepper->z_dir = 0;
				}
			}

			// Calculare timer value
			unsigned long timer = 0;
			unsigned long step_rate = 0;
			unsigned long step_loop = 0;
			if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {
				acc_step_rate = calc_vel(acceleration_time, current_block->acceleration_st);//利用經過的時間來算出增加的多少速度 need add(加速段)
				acc_step_rate += current_block->initial_rate;
				
				// upper limit
				if(acc_step_rate > current_block->nominal_rate){
					acc_step_rate = current_block->nominal_rate;
				}	
				// step_rate to timer interval
				timer = calc_timer(acc_step_rate);
				if(timer < 10000L) {
					period = 10000L / machine->mcm_samplecycle;
					step_loop = 10000L / timer;
					stepper->sc = (unsigned long)(machine->mcm_samplecycle - 1);
					acceleration_time += (step_loop * timer);
				} else {
					period = timer;
					step_loop = 1;
					stepper->sc = 0;
					acceleration_time += timer;
				}
			} else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
				step_rate = calc_vel(deceleration_time, current_block->acceleration_st);//利用經過的時間來算出增加的多少速度 need add(減速段)

				if(step_rate > acc_step_rate) { // Check step_rate stays positive
					step_rate = current_block->final_rate;
				} else {
					step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
				}

				// lower limit
				if(step_rate < current_block->final_rate)
					step_rate = current_block->final_rate;

				// step_rate to timer interval
				timer = calc_timer(step_rate);

				if(timer < 10000L) {
					period = 10000L / machine->mcm_samplecycle;
					step_loop = 10000L / timer;
					stepper->sc = (unsigned long)(machine->mcm_samplecycle - 1);
					deceleration_time += (step_loop * timer);
				} else {
					period = timer;
					step_loop = 1;
					stepper->sc = 0;
					deceleration_time += timer;
				}
			} else {
				if(step_nominal < 10000L) {
					period = 10000L / machine->mcm_samplecycle;
					step_loop = 10000L / step_nominal;
					stepper->sc = (unsigned long)(machine->mcm_samplecycle - 1);
				} else {
					period = step_nominal;
					step_loop = 1;
					stepper->sc = 0;
				}
			}			
			int i;
			for(i = 0; i < step_loop; i++){
				counter_x += current_block->steps_x;
				if (counter_x > 0) {
					//out_mdata |= 1 << pins->x_step_pin;//要打x pluse，改output的值
					counter_x -= current_block->step_event_count;
					stepper->dda_x += 1;   
				}

				counter_y += current_block->steps_y;
				if (counter_y > 0) {
					//out_mdata |= 1 << pins->y_step_pin;//要打y pluse，改output的值
					counter_y -= current_block->step_event_count;
					stepper->dda_y += 1;
				}

				counter_z += current_block->steps_z;
				if (counter_z > 0) {
					//out_mdata |= 1 << pins->z_step_pin;//要打z pluse，改output的值
					counter_z -= current_block->step_event_count;
					stepper->dda_z += 1;
				}

				counter_e += current_block->steps_e;
				if (counter_e > 0) {
					//out_edata |= 1 << pins->e_step_pin;//要打e pluse，改output的值
					counter_e -= current_block->step_event_count;
					stepper->dda_e += 1;
				}
				step_events_completed += 1;			
			}
			
			//把period & out_mdata & out_edata放進queue中
			stepper->motor_val = out_mdata;
			stepper->extruder_val = out_edata;
			stepper->period = period;
			if(machine->step_pulse_duty_ratio < 1) {
				stepper->duty = (unsigned long)(period * machine->step_pulse_duty_ratio);
			} else {
				stepper->duty = (unsigned long)machine->step_pulse_duty_ratio;
			}
			stepper_buffer_head = next_buffer_head;
			/* debug 
			fprintf(fp3, "header = %d, sc = %x, dda_x = %x, dda_y = %x, dda_z = %x, dda_e = %x, count = %ld, period = %ld\n", 
			stepper->is_header, stepper->sc, stepper->dda_x, stepper->dda_y, stepper->dda_z, stepper->dda_e, 
			step_events_completed, stepper->period);
			//end debug*/
			// If current block is finished, reset pointer
			if (step_events_completed >= current_block->step_event_count) {
				current_block = NULL;
				plan_discard_current_block();
			}
			
		}
		clock--;
	}
}

void st_set_position(const long &x, const long &y, const long &z, const long &e) {
	io_DisableINT();
	count_position[X_AXIS] = x;
	count_position[Y_AXIS] = y;
	count_position[Z_AXIS] = z;
	count_position[E_AXIS] = e;
	io_RestoreINT();
}

void st_set_e_position(const long &e) {
	io_DisableINT();
	count_position[E_AXIS] = e;
	io_RestoreINT();
}

long st_get_position(unsigned char axis) {
	long count_pos;
	io_DisableINT();
	count_pos = count_position[axis];
	io_RestoreINT();
	return count_pos;
}

int Check_limit(unsigned char axis) {
	if(axis == X_AXIS) {
		if((mcpfau_ReadCapStatREG(pins->mc_limit, pins->md_limit) & 0x80L) == 0)
			return 1;
		else return 0;
	} else
	if(axis == Y_AXIS) {
		if((mcpfau_ReadCapStatREG(pins->mc_limit, pins->md_limit) & 0x40L) == 0)
			return 1;
		else return 0;
	} else
	if(axis == Z_AXIS) {
		if((mcpfau_ReadCapStatREG(pins->mc_limit, pins->md_limit) & 0x20L) == 0)
			return 1;
		else return 0;
	} else {
		return -1;
	}
}


#define SC_END_INT       (0x02L)
int mcint_offset[3] = {0, 8, 16};
static __inline__ void clear_INTSTATUS(void) {
    mc_outp(pins->mc_qx, 0x04, SC_END_INT << mcint_offset[pins->md_qx]); //for EX
}

static __inline__ unsigned long read_INTSTATUS(void) {
    return (mc_inp(pins->mc_qx, 0x04) >> mcint_offset[pins->md_qx]);
}

static __inline__ void disable_MCINT(void) {
    mc_outp(pins->mc_qx, 0x00, 0x00L);  // disable mc interrupt
    mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) | (1L << pins->mc_qx));
}

static __inline__ void enable_MCINT(void) {
	mc_outp(MC_GENERAL, 0x38, mc_inp(MC_GENERAL, 0x38) & ~(1L << pins->mc_qx));
	mc_outp(pins->mc_qx, 0x00, SC_END_INT<<mcint_offset[pins->md_qx]);
}


//isr test 
//volatile unsigned long intdata = 2000000L;
//volatile unsigned long counter = 0L;
//volatile unsigned long average = 0L;
//volatile unsigned long total = 0L;
//volatile unsigned long pre_period = 100000L;
//volatile unsigned long temp_period = 100000L;
//volatile unsigned long min = 2000000L;
//volatile unsigned long max = 0L;
//volatile unsigned long dif = 0L;

//volatile unsigned long average1 = 0L;
//volatile unsigned long total1 = 0L;
//volatile unsigned long min1 = 2000000L;
//volatile unsigned long max1 = 0L;

//end test isr

volatile unsigned long isr_num = 0; 
volatile unsigned int x_hit_limit = 0; 
volatile unsigned int y_hit_limit = 0; 
volatile unsigned int z_hit_limit = 0; 

static int isr_handler(int irq, void* data) {
	if((read_INTSTATUS() & SC_END_INT) != 0) {//check int status
		//isr test	
		//if(in_discard_block_state != true) {
		//	intdata = mcsv_ReadSTATREG2(pins->mc_qx, pins->md_qx);
		//	dif = pre_period - intdata;
		//	if (dif > max1) max1 = dif;
		//	if (dif < min1) min1 = dif; 
		//	counter++;
		//	total1 += dif;
		//	average1 = total1/counter;
		//}
		//end isr test
		clear_INTSTATUS();//clear status
		
		if (in_discard_block_state) st_discard_current_block();
		if (in_discard_block_state) return ISR_HANDLED;
		
		current_stepper = plan_get_current_stepper();
		if(current_stepper == NULL) {//stepper buf is empty, 設定period = 500us(200k)? dda = 0
			mc_LockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);
			mcsv_SetSamplCycle(pins->mc_qx, pins->md_qx, 0x00L);
			mcsv_SetDDA(pins->mc_qx, pins->md_qx, MCSV_DDADIR_NONE, 0x00L);
			mcsv_SetWidth(pins->mc_qx, pins->md_qx, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
			mcsv_ReloadSV(pins->mc_qx, pins->md_qx, MCSV_PDIR_RELOAD_SCEND);
			mcsv_SetSamplCycle(pins->mc_qy, pins->md_qy, 0x00L);
			mcsv_SetDDA(pins->mc_qy, pins->md_qy, MCSV_DDADIR_NONE, 0x00L);
			mcsv_SetWidth(pins->mc_qy, pins->md_qy, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
			mcsv_ReloadSV(pins->mc_qy, pins->md_qy, MCSV_PDIR_RELOAD_SCEND);
			mcsv_SetSamplCycle(pins->mc_qz, pins->md_qz, 0x00L);
			mcsv_SetDDA(pins->mc_qz, pins->md_qz, MCSV_DDADIR_NONE, 0x00L);
			mcsv_SetWidth(pins->mc_qz, pins->md_qz, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
			mcsv_ReloadSV(pins->mc_qz, pins->md_qz, MCSV_PDIR_RELOAD_SCEND);
			mcsv_SetSamplCycle(pins->mc_qe, pins->md_qe, 0x00L);
			mcsv_SetDDA(pins->mc_qe, pins->md_qe, MCSV_DDADIR_NONE, 0x00L);
			mcsv_SetWidth(pins->mc_qe, pins->md_qe, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
			mcsv_ReloadSV(pins->mc_qe, pins->md_qe, MCSV_PDIR_RELOAD_SCEND);
			mc_UnlockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);
			discard_flag = 0;
			//temp_period = 100000L;//isr_test
		} else {
			if(current_stepper->is_header == 1) {//block開頭，設定方向, 每個block會是同方向的
				
				if(((current_stepper->motor_val >> pins->x_dir_pin) & 0x01L) == 1) {
					mcsv_SetDIR(pins->mc_qx, pins->md_qx, MCSV_DIR_CCW);
					count_direction[X_AXIS] = -1;
				} else {
					mcsv_SetDIR(pins->mc_qx, pins->md_qx, MCSV_DIR_CW);
					count_direction[X_AXIS] = 1;
				}
				mcsv_ReloadOUT(pins->mc_qx, pins->md_qx, MCSV_PDIR_RELOAD_SCEND);
				//mcsv_SetDIR(pins->mc_qy, pins->md_qy, (((current_stepper->motor_val >> pins->y_dir_pin) & 0x01L) /*^ pins->invert_y_dir*/) << 30);
				if(((current_stepper->motor_val >> pins->y_dir_pin) & 0x01L) == 1) {
					mcsv_SetDIR(pins->mc_qy, pins->md_qy, MCSV_DIR_CCW);
					count_direction[Y_AXIS] = -1;
				} else {
					mcsv_SetDIR(pins->mc_qy, pins->md_qy, MCSV_DIR_CW);
					count_direction[Y_AXIS] = 1;
				}
				mcsv_ReloadOUT(pins->mc_qy, pins->md_qy, MCSV_PDIR_RELOAD_SCEND);
				//mcsv_SetDIR(pins->mc_qz, pins->md_qz, (((current_stepper->motor_val >> pins->z_dir_pin) & 0x01L) /*^ pins->invert_z_dir*/) << 30);
				if(((current_stepper->motor_val >> pins->z_dir_pin) & 0x01L) == 1) {
					mcsv_SetDIR(pins->mc_qz, pins->md_qz, MCSV_DIR_CCW);
					count_direction[Z_AXIS] = -1;
				} else {
					mcsv_SetDIR(pins->mc_qz, pins->md_qz, MCSV_DIR_CW);
					count_direction[Z_AXIS] = 1;
				}
				mcsv_ReloadOUT(pins->mc_qz, pins->md_qz, MCSV_PDIR_RELOAD_SCEND);
				//mcsv_SetDIR(pins->mc_qe, pins->md_qe, (((current_stepper->extruder_val >> pins->e_dir_pin) & 0x01L) /*^ pins->invert_e_dir*/) << 30);
				if(((current_stepper->extruder_val >> pins->e_dir_pin) & 0x01L) == 1) {
					mcsv_SetDIR(pins->mc_qe, pins->md_qe, MCSV_DIR_CCW);
					count_direction[E_AXIS] = -1;
				} else {
					mcsv_SetDIR(pins->mc_qe, pins->md_qe, MCSV_DIR_CW);
					count_direction[E_AXIS] = 1;
				}
				mcsv_ReloadOUT(pins->mc_qe, pins->md_qe, MCSV_PDIR_RELOAD_SCEND);
				x_hit_limit = 0;
				y_hit_limit = 0;
				z_hit_limit = 0;
			}
			//check limit
			if(current_stepper->x_dir == 1) {
				if(Check_limit(X_AXIS) == 1) {//x limit 
					if((long)(count_direction[X_AXIS] * current_stepper->dda_x) + (long)(count_direction[Y_AXIS] * current_stepper->dda_y) > 0)
						x_hit_limit = 1;
					else 
						x_hit_limit = 0;
				}
			}
			if(current_stepper->y_dir == 1) {
				if(Check_limit(Y_AXIS) == 1) {//y limit
					if((long)(count_direction[X_AXIS] * current_stepper->dda_x) - (long)(count_direction[Y_AXIS] * current_stepper->dda_y) < 0)
						y_hit_limit = 1;
					else
						y_hit_limit = 0;
				}
			}
			if(current_stepper->z_dir == 0) {
				if(Check_limit(Z_AXIS) == 1) {//z limit
					if(current_stepper->dda_z != 0)
						z_hit_limit = 1;
					else
						z_hit_limit = 0;
				}
			}
			//if limit hit, discard block
			if(x_hit_limit == 1 || y_hit_limit == 1 || z_hit_limit == 1) {
				mc_LockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);
				mcsv_SetSamplCycle(pins->mc_qx, pins->md_qx, 0x00L);
				mcsv_SetDDA(pins->mc_qx, pins->md_qx, MCSV_DDADIR_NONE, 0x00L);
				mcsv_SetWidth(pins->mc_qx, pins->md_qx, sys->hit_pulse_period - 1L, sys->hit_pulse_duty - 1L);
				mcsv_ReloadSV(pins->mc_qx, pins->md_qx, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qy, pins->md_qy, 0x00L);
				mcsv_SetDDA(pins->mc_qy, pins->md_qy, MCSV_DDADIR_NONE, 0x00L);
				mcsv_SetWidth(pins->mc_qy, pins->md_qy, sys->hit_pulse_period - 1L, sys->hit_pulse_duty - 1L);
				mcsv_ReloadSV(pins->mc_qy, pins->md_qy, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qz, pins->md_qz, 0x00L);
				mcsv_SetDDA(pins->mc_qz, pins->md_qz, MCSV_DDADIR_NONE, 0x00L);
				mcsv_SetWidth(pins->mc_qz, pins->md_qz, sys->hit_pulse_period - 1L, sys->hit_pulse_duty - 1L);
				mcsv_ReloadSV(pins->mc_qz, pins->md_qz, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qe, pins->md_qe, 0x00L);
				mcsv_SetDDA(pins->mc_qe, pins->md_qe, MCSV_DDADIR_NONE, 0x00L);
				mcsv_SetWidth(pins->mc_qe, pins->md_qe, sys->hit_pulse_period - 1L, sys->hit_pulse_duty - 1L);
				mcsv_ReloadSV(pins->mc_qe, pins->md_qe, MCSV_PDIR_RELOAD_SCEND);
				mc_UnlockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);						
				// plan_discard_current_stepper();
				in_discard_block_state = true;
				plan_discard_current_stepper();
				//st_discard_current_block();
			} else {
				//設定period & duty & dda & sc
				mc_LockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);
				mcsv_SetSamplCycle(pins->mc_qx, pins->md_qx, current_stepper->sc);
				mcsv_SetDDA(pins->mc_qx, pins->md_qx, MCSV_DDADIR_NONE, current_stepper->dda_x);
				mcsv_SetWidth(pins->mc_qx, pins->md_qx, current_stepper->period - 1, current_stepper->duty - 1);
				mcsv_ReloadSV(pins->mc_qx, pins->md_qx, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qy, pins->md_qy, current_stepper->sc);
				mcsv_SetDDA(pins->mc_qy, pins->md_qy, MCSV_DDADIR_NONE, current_stepper->dda_y);
				mcsv_SetWidth(pins->mc_qy, pins->md_qy, current_stepper->period - 1, current_stepper->duty - 1);
				mcsv_ReloadSV(pins->mc_qy, pins->md_qy, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qz, pins->md_qz, current_stepper->sc);
				mcsv_SetDDA(pins->mc_qz, pins->md_qz, MCSV_DDADIR_NONE, current_stepper->dda_z);
				mcsv_SetWidth(pins->mc_qz, pins->md_qz, current_stepper->period - 1, current_stepper->duty - 1);
				mcsv_ReloadSV(pins->mc_qz, pins->md_qz, MCSV_PDIR_RELOAD_SCEND);
				mcsv_SetSamplCycle(pins->mc_qe, pins->md_qe, current_stepper->sc);
				mcsv_SetDDA(pins->mc_qe, pins->md_qe, MCSV_DDADIR_NONE, current_stepper->dda_e);
				mcsv_SetWidth(pins->mc_qe, pins->md_qe, current_stepper->period - 1, current_stepper->duty - 1);
				mcsv_ReloadSV(pins->mc_qe, pins->md_qe, MCSV_PDIR_RELOAD_SCEND);
				mc_UnlockLDRDYs(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, 0x00L, 0x00L);	
				count_position[X_AXIS]+=(count_direction[X_AXIS] * current_stepper->dda_x);				
				count_position[Y_AXIS]+=(count_direction[Y_AXIS] * current_stepper->dda_y);
				count_position[Z_AXIS]+=(count_direction[Z_AXIS] * current_stepper->dda_z);
				count_position[E_AXIS]+=(count_direction[E_AXIS] * current_stepper->dda_e);				
				//isr_num++;
			}
			//temp_period = current_stepper->period;//isr test

		}
		//isr test
		//if(in_discard_block_state != true) {
		//	intdata = mcsv_ReadSTATREG2(pins->mc_qx, pins->md_qx);
		//	dif = pre_period - intdata;
		//	if (dif > max) max = dif;
		//	if (dif < min) min = dif; 
			//counter++;
		//	total += dif;
		//	average = total/counter;
		//}
		//pre_period = temp_period;
		//end isr test
		plan_discard_current_stepper();
		return ISR_HANDLED;
	}
	return ISR_NONE;
}

static __inline__  void write_mc_pcireg(unsigned idx, unsigned long val) {   
    unsigned long cf8 = (0x01L << 31)  // Type-1 PCI configuration
                      + (0x00L << 16)  // Bus 0x00
                      + (0x10L << 11) // Device 0x01
                      + (0x00L << 8)   // Fun 0x00;
                      + idx;

    io_DisableINT();
	io_outpdw(0x0cf8, cf8 & 0xfffffffcL);
	io_outpdw(0x0cfc, val);
    io_RestoreINT();
}

static __inline__  unsigned long read_mc_pcireg(unsigned idx) {
    unsigned long tmp;
    unsigned long cf8 = (0x01L << 31)  // Type-1 PCI configuration
                      + (0x00L << 16)  // Bus 0x00
                      + (0x10L << 11)  // Device 0x01
                      + (0x00L << 8)   // Fun 0x00;
                      + idx;

    io_DisableINT();
	io_outpdw(0x0cf8, cf8 & 0xfffffffcL);
    tmp = io_inpdw(0x0cfc);
    io_RestoreINT();

    return tmp;
}

static __inline__ unsigned char GetMCIRQ(void) {  
    return (unsigned char)(read_mc_pcireg(0x3c) & 0xffL);
}

static __inline__  bool init_mc_irq(void) {
    if(irq_Init() == false) {
        print_errmsg("irq_init fail\n"); return false;
    }
    
    if(irq_Setting(GetMCIRQ(), IRQ_LEVEL_TRIGGER + IRQ_DISABLE_INTR) == false) {
        print_errmsg("%s\n", __FUNCTION__); return false;
    }
    
    if(irq_InstallISR(GetMCIRQ(), isr_handler, (void *)NULL) == false) {
        print_errmsg("irq_install fail\n"); return false;
    }
    return true;
}

static __inline__  bool close_mc_irq(void) {
    if(irq_UninstallISR(GetMCIRQ(), (void *)NULL) == false)
    {
        print_errmsg("irq_uninstall fail\n"); return false;
    }
	
    if(irq_Close() == false) 
    {
        print_errmsg("irq_close fail\n"); return false;
    }
    return true;
}

static __inline__ int mcm_init(void) {
	set_MMIO();
	mc_setbaseaddr();
	//init mcm mode
	if (mc_SetMode(pins->mc_qx, MCMODE_SERVO) == false) {
		print_errmsg("MC%d init error\n", pins->mc_qx);
		return -1;
	}
	if(pins->mc_qx != pins->mc_qy) {
		if (mc_SetMode(pins->mc_qy, MCMODE_SERVO) == false) {
			print_errmsg("MC%d init error\n", pins->mc_qy);
			return -1;
		}	
	}
	if(pins->mc_qx != pins->mc_qz && pins->mc_qy != pins->mc_qz) {
		if (mc_SetMode(pins->mc_qz, MCMODE_SERVO) == false) {
			print_errmsg("MC%d init error\n", pins->mc_qz);
			return -1;
		}	
	}
	if(pins->mc_qx != pins->mc_qe && pins->mc_qy != pins->mc_qe && pins->mc_qz != pins->mc_qe) {
		if (mc_SetMode(pins->mc_qe, MCMODE_SERVO) == false) {
			print_errmsg("MC%d init error\n", pins->mc_qe);
			return -1;
		}
	}
	//
	// 溫度那邊有設定了
    // if (mc_SetMode(pins->mc_limit, MCMODE_SIF) == false) {
        // printf("MC%d init error\n", pins->mc_limit);
        // return -1;
    // }
	
	//servo init(X, Y, Z, E)
	mcsv_SetOutMask(pins->mc_qx, pins->md_qx, MCSV_P1MASK_NONE + MCSV_P2MASK_NONE);
	mcsv_SetOutPolarity(pins->mc_qx, pins->md_qx, MCSV_POL_NORMAL);
	mcsv_SetDIR(pins->mc_qx, pins->md_qx, MCSV_DIR_CW);
	mcsv_ReloadOUT(pins->mc_qx, pins->md_qx, MCSV_RELOAD_NOW);
	mcsv_ClearDirOwner(pins->mc_qx, pins->md_qx);
	mcsv_SetDDAMode(pins->mc_qx, pins->md_qx, MCSV_DDA_COUNT_POPZERO);
	mcsv_SetDDA(pins->mc_qx, pins->md_qx, MCSV_DDADIR_NONE, 0x00L);
	mcsv_ReloadSV(pins->mc_qx, pins->md_qx, MCSV_RELOAD_NOW_NODDA);
	mcsv_SetMode(pins->mc_qx, pins->md_qx, MCSV_PDIR_I0A1);
	mcsv_SetWidth(pins->mc_qx, pins->md_qx, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
	mcsv_SetSamplCycle(pins->mc_qx, pins->md_qx, 0x00L);
	mcsv_ReloadSV(pins->mc_qx, pins->md_qx, MCSV_RELOAD_NOW_NODDA);
	while(mcsv_ReadDDASTAT(pins->mc_qx, pins->md_qx) != MCSV_DDAFIFO_EMPTY)
		mcsv_ReadDDA(pins->mc_qx, pins->md_qx);
		
	mcsv_SetOutMask(pins->mc_qy, pins->md_qy, MCSV_P1MASK_NONE + MCSV_P2MASK_NONE);
	mcsv_SetOutPolarity(pins->mc_qy, pins->md_qy, MCSV_POL_NORMAL);
	mcsv_SetDIR(pins->mc_qy, pins->md_qy, MCSV_DIR_CW);
	mcsv_ReloadOUT(pins->mc_qy, pins->md_qy, MCSV_RELOAD_NOW);
	mcsv_ClearDirOwner(pins->mc_qy, pins->md_qy);
	mcsv_SetDDAMode(pins->mc_qy, pins->md_qy, MCSV_DDA_COUNT_POPZERO);
	mcsv_SetDDA(pins->mc_qy, pins->md_qy, MCSV_DDADIR_NONE, 0x00L);
	mcsv_ReloadSV(pins->mc_qy, pins->md_qy, MCSV_RELOAD_NOW_NODDA);
	mcsv_SetMode(pins->mc_qy, pins->md_qy, MCSV_PDIR_I0A1);
	mcsv_SetWidth(pins->mc_qy, pins->md_qy, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
	mcsv_SetSamplCycle(pins->mc_qy, pins->md_qy, 0x00L);
	mcsv_ReloadSV(pins->mc_qy, pins->md_qy, MCSV_RELOAD_NOW_NODDA);
	while(mcsv_ReadDDASTAT(pins->mc_qy, pins->md_qy) != MCSV_DDAFIFO_EMPTY)
		mcsv_ReadDDA(pins->mc_qy, pins->md_qy);
		
	mcsv_SetOutMask(pins->mc_qz, pins->md_qz, MCSV_P1MASK_NONE + MCSV_P2MASK_NONE);
	mcsv_SetOutPolarity(pins->mc_qz, pins->md_qz, MCSV_POL_NORMAL);
	mcsv_SetDIR(pins->mc_qz, pins->md_qz, MCSV_DIR_CW);
	mcsv_ReloadOUT(pins->mc_qz, pins->md_qz, MCSV_RELOAD_NOW);
	mcsv_ClearDirOwner(pins->mc_qz, pins->md_qz);
	mcsv_SetDDAMode(pins->mc_qz, pins->md_qz, MCSV_DDA_COUNT_POPZERO);
	mcsv_SetDDA(pins->mc_qz, pins->md_qz, MCSV_DDADIR_NONE, 0x00L);
	mcsv_ReloadSV(pins->mc_qz, pins->md_qz, MCSV_RELOAD_NOW_NODDA);
	mcsv_SetMode(pins->mc_qz, pins->md_qz, MCSV_PDIR_I0A1);
	mcsv_SetWidth(pins->mc_qz, pins->md_qz, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
	mcsv_SetSamplCycle(pins->mc_qz, pins->md_qz, 0x00L);
	mcsv_ReloadSV(pins->mc_qz, pins->md_qz, MCSV_RELOAD_NOW_NODDA);
	while(mcsv_ReadDDASTAT(pins->mc_qz, pins->md_qz) != MCSV_DDAFIFO_EMPTY)
		mcsv_ReadDDA(pins->mc_qz, pins->md_qz);
		
	mcsv_SetOutMask(pins->mc_qe, pins->md_qe, MCSV_P1MASK_NONE + MCSV_P2MASK_NONE);
	mcsv_SetOutPolarity(pins->mc_qe, pins->md_qe, MCSV_POL_NORMAL);
	mcsv_SetDIR(pins->mc_qe, pins->md_qe, MCSV_DIR_CW);
	mcsv_ReloadOUT(pins->mc_qe, pins->md_qe, MCSV_RELOAD_NOW);
	mcsv_ClearDirOwner(pins->mc_qe, pins->md_qe);
	mcsv_SetDDAMode(pins->mc_qe, pins->md_qe, MCSV_DDA_COUNT_POPZERO);
	mcsv_SetDDA(pins->mc_qe, pins->md_qe, MCSV_DDADIR_NONE, 0x00L);
	mcsv_ReloadSV(pins->mc_qe, pins->md_qe, MCSV_RELOAD_NOW_NODDA);
	mcsv_SetMode(pins->mc_qe, pins->md_qe, MCSV_PDIR_I0A1);
	mcsv_SetWidth(pins->mc_qe, pins->md_qe, sys->default_pulse_period - 1L, sys->default_pulse_duty - 1L);
	mcsv_SetSamplCycle(pins->mc_qe, pins->md_qe, 0x00L);
	mcsv_ReloadSV(pins->mc_qe, pins->md_qe, MCSV_RELOAD_NOW_NODDA);
	while(mcsv_ReadDDASTAT(pins->mc_qe, pins->md_qe) != MCSV_DDAFIFO_EMPTY)
		mcsv_ReadDDA(pins->mc_qe, pins->md_qe);	

	//sif init
    mcsif_SetInputFilter(pins->mc_limit, pins->md_limit, 500L);
    mcsif_SetSWDeadband(pins->mc_limit, pins->md_limit, 0L);
    mcsif_SetSWPOL(pins->mc_limit, pins->md_limit, MCSIF_SWPOL_REMAIN);
    mcsif_SetSamplWin(pins->mc_limit, pins->md_limit, MCSIF_SWSTART_NOW + MCSIF_SWEND_DISABLE);

	mcsif_SetMode(pins->mc_limit, pins->md_limit, MCSIF_PFAU);	

	//LCD
	mcpwm_SetOutMask(pins->mc_lcd, pins->md_lcd, MCPWM_HMASK_NONE + MCPWM_LMASK_NONE);
    mcpwm_SetOutPolarity(pins->mc_lcd, pins->md_lcd, MCPWM_HPOL_NORMAL + MCPWM_LPOL_NORMAL);
    mcpwm_SetDeadband(pins->mc_lcd, pins->md_lcd, 0L);
    mcpwm_ReloadOUT_Unsafe(pins->mc_lcd, pins->md_lcd, MCPWM_RELOAD_NOW);
    mcpwm_SetWaveform(pins->mc_lcd, pins->md_lcd, MCPWM_EDGE_A0I1);
    mcpwm_SetWidth(pins->mc_lcd, pins->md_lcd, 100000L-1L, 0L);
    mcpwm_SetSamplCycle(pins->mc_lcd, pins->md_lcd, 100L);
    mcpwm_Enable(pins->mc_lcd, pins->md_lcd);
	
	return 0;
}


void st_init(void) {
	//
	count_position = (volatile long *)malloc(sizeof(volatile long) * machine->num_axis);
	count_direction = (volatile signed char *)malloc(sizeof(volatile signed char) * machine->num_axis);
	//
	init_stepper_buffer();
	/* debug
	sfp_init();
	//end debug */
	io_Init();
	if(init_mc_irq() == false) {
		print_errmsg("mc_irq fail\n");
	}
	if(mcm_init() != 0) {
		print_errmsg("mcm_init fail\n");//error msg & close 
	}
	enable_MCINT();
	mc_EnableMulti(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, MCMULTI_NONE, MCMULTI_NONE);

}

void st_close(void) {
	mc_DisableMulti(MCMULTI_MC0SERVOA+MCMULTI_MC0SERVOB+MCMULTI_MC0SERVOC+MCMULTI_MC1SERVOA, MCMULTI_NONE, MCMULTI_NONE);
	disable_MCINT();
	close_mc_irq();
	free(stepper_buffer);
	//printf("before : min=%ldus, max=%ldus, avg = %ldus,  counter=%ld\n", min1/100, max1/100, average1/100, counter);
	//printf("after : min=%ldus, max=%ldus, avg = %ldus,  counter=%ld\n", min/100, max/100, average/100, counter);
}

unsigned long lcd_duty = 0;
char sign = 1;
void LCD(unsigned long clock) {
	while(clock != 0) {
		if(mcpwm_ReadReloadPWM(pins->mc_lcd, pins->md_lcd) == 0) {
			if(lcd_duty >= 100000L) {
				sign = -1;
			} else if(lcd_duty <= 0L) {
				sign = 1;
			}
			lcd_duty = (lcd_duty + sign *100L);
			mcpwm_SetWidth(pins->mc_lcd, pins->md_lcd, 100000L-1L, lcd_duty % 100001L);
			mcpwm_ReloadPWM(pins->mc_lcd, pins->md_lcd, MCPWM_RELOAD_SCEND);
		}
		if(Check_limit(Z_AXIS) != 1) {//z
			io_outpb(0xF21C, io_inpb(0xF21C) | 0x40);
		} else {
			io_outpb(0xF21C, io_inpb(0xF21C) & (~0x40));
		}
		if(Check_limit(Y_AXIS) != 1) {//y
			io_outpb(0xF21C, io_inpb(0xF21C) | 0x20);
		} else {
			io_outpb(0xF21C, io_inpb(0xF21C) & (~0x20));
		}
		if(Check_limit(X_AXIS) != 1) {//x
			io_outpb(0xF21C, io_inpb(0xF21C) | 0x10);
		} else {
			io_outpb(0xF21C, io_inpb(0xF21C) & (~0x10));
		}
		clock--;
	}
}

double st_get_position_mm(unsigned char axis)
{
	double steper_position_in_steps;
	unsigned long temp_x, temp_y;
	char invert_dir[4];
	if(pins->invert_x_dir == 1) invert_dir[X_AXIS] = -1;
	else invert_dir[X_AXIS] = 1;
	if(pins->invert_y_dir == 1) invert_dir[Y_AXIS] = -1;
	else invert_dir[Y_AXIS] = 1;
	if(pins->invert_z_dir == 1) invert_dir[Z_AXIS] = -1;
	else invert_dir[Z_AXIS] = 1;
	if(pins->invert_e_dir == 1) invert_dir[E_AXIS] = -1;
	else invert_dir[E_AXIS] = 1;
	
	if((machine->type == H_BOT) && (axis == X_AXIS)) {
		temp_x = st_get_position(X_AXIS) * invert_dir[X_AXIS];
		temp_y = st_get_position(Y_AXIS) * invert_dir[Y_AXIS];
		steper_position_in_steps = (double)(temp_x + temp_y) / axis_steps_per_unit[X_AXIS];
	} else if((machine->type == H_BOT) && (axis == Y_AXIS)) {
		temp_x = st_get_position(X_AXIS) * invert_dir[X_AXIS];
		temp_y = st_get_position(Y_AXIS) * invert_dir[Y_AXIS];
		steper_position_in_steps = (double)(temp_x - temp_y) / axis_steps_per_unit[X_AXIS];
	} else {
		steper_position_in_steps = ((double)st_get_position(axis) / axis_steps_per_unit[axis]) * invert_dir[axis];
	}
	//TODO:DELTA
	return steper_position_in_steps;
}


