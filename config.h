//#include "math.h"
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"
//#include "conio.h"

//#define USE_COMMON
//#include "dmpcfg.h"
//#include "common.h"


//#define NUM_AXIS 4 //ok
//#define BLOCK_BUFFER_SIZE 256 //ok
//#define BLOCK_BUFFER_BOUND 16 //ok
//#define MAX_CMD_SIZE 128 //ok

//#define STEPPER_BUFFER_SIZE 1024 //ok
//#define STEPPER_BUFFER_BOUND 20 //ok

//#define COREXY

//#define min_software_endstops true // If true, axis won't move to coordinates less than HOME_POS.
//#define max_software_endstops true // If true, axis won't move to coordinates greater than the defined lengths below.
// Travel limits after homing
//#define X_MAX_POS 125 //ok
//#define X_MIN_POS 0 //ok
//#define X_HOME_POS 0//ok
//#define Y_MAX_POS 75 //ok
//#define Y_MIN_POS 0 //ok
//#define Y_HOME_POS Y_MAX_POS//ok
//#define Z_MAX_POS 130 //ok
//#define Z_MIN_POS 0 //ok
//#define Z_HOME_POS 0//ok

//#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS) //不要了
//#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)//不要了
//#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)//不要了

//#define X_HOME_RETRACT_MM 5 //ok 
//#define Y_HOME_RETRACT_MM 5  //ok
//#define Z_HOME_RETRACT_MM 1  //ok

//#define X_HOME_DIR -1//ok
//#define Y_HOME_DIR  1 //ok
//#define Z_HOME_DIR -1 //ok

//#define DEFAULT_AXIS_STEPS_PER_UNIT   {40,40, 1287,47.99} //ok
//<<<<<<< HEAD
//=======
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {40,40, 1287,45.88}
//>>>>>>> b4b1159fc7cf7f4f5a9c0e110ae1832c2e9adaad
// #define DEFAULT_MAX_FEEDRATE          {10000, 10000, 3, 25}    // (mm/sec)
// #define DEFAULT_MAX_ACCELERATION      {4000,4000,50,4000} 
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {78.7402,78.7402,200.0*8/3,760*1.1}  // default steps per unit for Ultimaker
// #define DEFAULT_AXIS_STEPS_PER_UNIT   {40,40,40,40}
// #define DEFAULT_MAX_FEEDRATE          {500, 500, 5, 25}    // (mm/sec)
// #define DEFAULT_MAX_ACCELERATION      {9000,9000,100,10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
//#define DEFAULT_MAX_FEEDRATE          {500, 500, 200, 25}    // (mm/sec)
//#define DEFAULT_MAX_ACCELERATION      {9000,9000,20,10000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

//#define DEFAULT_ACCELERATION          3000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
//#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

//#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
//#define DEFAULT_MINTRAVELFEEDRATE     0.0

//#define DEFAULT_XYJERK                20.0    // (mm/sec)
//#define DEFAULT_ZJERK                 0.4     // (mm/sec)
//#define DEFAULT_EJERK                 5.0    // (mm/sec)

//#define MINIMUM_PLANNER_SPEED 0.05// (mm/sec)

//#define F_CPU 16000000

// Arc interpretation settings:

//#define MM_PER_ARC_SEGMENT 1 //ok
//#define N_ARC_CORRECTION 25 //ok

//#define X_AXIS 0 //ok
//#define Y_AXIS 1 //ok
//#define Z_AXIS 2 //ok
//#define E_AXIS 3 //ok

//#define INVERT_X_DIR 	1 //ok
//#define X_STEP_PIN 		0 //ok
//#define X_DIR_PIN 		1 //ok
//#define INVERT_Y_DIR 	1 //ok
//#define Y_STEP_PIN 		2 //ok
//#define Y_DIR_PIN		3 //ok
//#define INVERT_Z_DIR 	1 //ok
//#define Z_STEP_PIN 		5 //ok
//#define Z_DIR_PIN		4 //ok
//#define INVERT_E_DIR 	0 //ok
//#define E_STEP_PIN 		0 //ok
//#define E_DIR_PIN		1 //ok


//#define X_ENABLE_PORT   4
//#define X_ENABLE_PIN    0
//#define X_ENABLE_XON    1
//#define Y_ENABLE_PORT   4
//#define Y_ENABLE_PIN    1
//#define Y_ENABLE_XON    1
//#define Z_ENABLE_PORT   4
//#define Z_ENABLE_PIN    2
//#define Z_ENABLE_XON    1
//#define E0_ENABLE_PORT  4
//#define E0_ENABLE_PIN   3
//#define E0_ENABLE_XON   1
//#define E1_ENABLE_PORT  4
//#define E1_ENABLE_PIN   4
//#define E1_ENABLE_XON   1

//#define MC_X 0  //ok
//#define MD_X 0 //ok
//#define MC_Y 0 //ok
//#define MD_Y 1 //ok
//#define MC_Z 0 //ok
//#define MD_Z 2 //ok
//#define MC_E 1 //ok
//#define MD_E 0 //ok
//#define MC_LIMIT 3 //ok
//#define MD_LIMIT 1 //ok
//#define MC_LCD 2 //ok
//#define MD_LCD 2 //ok

//#define DUTY_RATE 	0.5  //ok
//new
//#define DUTY_MIN  	250 //2.5us
//#define DEF_PERIOD	100000//1ms
//#define DEF_DUTY	50000//1ms
//#define HIT_PERIOD	10000//1ms
//#define HIT_DUTY	5000//1ms

//高速的選擇

//machine type
//#define NORMAL	 	(0)
//#define H_BOT		(1)
//#define DELTA		(2)
//end machine type

//enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};
// #define EXTRUDERS      1

// #define BANG_MAX       255 // limits current to nozzle while in bang-bang mode; 255=full current
// #define PID_MAX        255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
// #define MAX_BED_POWER  255 // limits duty cycle to bed; 255=full current
// #define FAN_MAX        255

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
// #define HEATER_0_MINTEMP 5
// #define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
// #define HEATER_0_MAXTEMP 275
// #define BED_MAXTEMP 150

// #define PID_FUNCTIONAL_RANGE 30

// #define K1  0.95
// #define K2 (1-K1)
// #define DEFAULT_Kp 3.71
// #define DEFAULT_Ki 0.40
// #define DEFAULT_Kd 8.64
// #define DEFAULT_Kp 22.3
// #define DEFAULT_Ki 0.0132
// #define DEFAULT_Kd 54.5
// #define DEFAULT_Kp 22.2
// #define DEFAULT_Ki 1.08
// #define DEFAULT_Kd 114
// #define DEFAULT_bedKp 10.00
// #define DEFAULT_bedKi .0028
// #define DEFAULT_bedKd 2500.4625
// #define bedKp 10.00
// #define bedKi .023
// #define bedKd 305.4

// #define PID_INTEGRAL_DRIVE_MAX 255

// #define STD_POWER_ADC (620.303)

// #define ENABLE_AUTO_BED_LEVELING
// #ifdef ENABLE_AUTO_BED_LEVELING

// There are 2 different ways to pick the X and Y locations to probe:

//  - "grid" mode
//    Probe every point in a rectangular grid
//    You must specify the rectangle, and the density of sample points
//    This mode is preferred because there are more measurements.
//    It used to be called ACCURATE_BED_LEVELING but "grid" is more descriptive

//  - "3-point" mode
//    Probe 3 arbitrary points on the bed (that aren't colinear)
//    You must specify the X & Y coordinates of all 3 points

  // #define AUTO_BED_LEVELING_GRID
  // with AUTO_BED_LEVELING_GRID, the bed is sampled in a
  // AUTO_BED_LEVELING_GRID_POINTSxAUTO_BED_LEVELING_GRID_POINTS grid
  // and least squares solution is calculated
  // Note: this feature occupies 10'206 byte
  // #ifdef AUTO_BED_LEVELING_GRID

    // set the rectangle in which to probe
    // #define LEFT_PROBE_BED_POSITION 5
    // #define RIGHT_PROBE_BED_POSITION 125
    // #define BACK_PROBE_BED_POSITION 95
    // #define FRONT_PROBE_BED_POSITION 5

     // set the number of grid points per dimension
     // I wouldn't see a reason to go above 3 (=9 probing points on the bed)
    // #define AUTO_BED_LEVELING_GRID_POINTS 3


  // #else  // not AUTO_BED_LEVELING_GRID
    // with no grid, just probe 3 arbitrary points.  A simple cross-product
    // is used to esimate the plane of the print bed

      // #define ABL_PROBE_PT_1_X 5
      // #define ABL_PROBE_PT_1_Y 95
      // #define ABL_PROBE_PT_2_X 5
      // #define ABL_PROBE_PT_2_Y 5
      // #define ABL_PROBE_PT_3_X 125
      // #define ABL_PROBE_PT_3_Y 5

  // #endif // AUTO_BED_LEVELING_GRID


  // these are the offsets to the probe relative to the extruder tip (Hotend - Probe)
  // #define X_PROBE_OFFSET_FROM_EXTRUDER 0.0
  // #define Y_PROBE_OFFSET_FROM_EXTRUDER 0.0
  // #define Z_PROBE_OFFSET_FROM_EXTRUDER 0.0

  // #define Z_RAISE_BEFORE_HOMING 4       // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                        // Be sure you have this distance over your Z_MAX_POS in case

  // #define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min

  // #define Z_RAISE_BEFORE_PROBING 15    //How much the extruder will be raised before traveling to the first probing point.
  // #define Z_RAISE_BETWEEN_PROBINGS 5  //How much the extruder will be raised when traveling from between next probing points


  //If defined, the Probe servo will be turned on only during movement and then turned off to avoid jerk
  //The value is the delay to turn the servo off after powered on - depends on the servo speed; 300ms is good value, but you can try lower it.
  // You MUST HAVE the SERVO_ENDSTOPS defined to use here a value higher than zero otherwise your code will not compile.

//  #define PROBE_SERVO_DEACTIVATION_DELAY 300


//If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
//it is highly recommended you let this Z_SAFE_HOMING enabled!!!

  // #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with probe outside the bed area.
                          // When defined, it will:
                          // - Allow Z homing only after X and Y homing AND stepper drivers still enabled
                          // - If stepper drivers timeout, it will need X and Y homing again before Z homing
                          // - Position the probe in a defined XY point before Z Homing when homing all axis (G28)
                          // - Block Z homing only when the probe is outside bed area.

  // #ifdef Z_SAFE_HOMING

    // #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28)
    // #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28)

  // #endif

// #endif // ENABLE_AUTO_BED_LEVELING
