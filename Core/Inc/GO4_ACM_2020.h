/*
 * GO4_ACM_2020.h
 *
 *  Created on: Sep 3, 2020
 *      Author: bents
 */

#ifndef INC_GO4_ACM_2020_H_
#define INC_GO4_ACM_2020_H_

#include "base_types.h"

//*************DEFINES*************//
#define BUFFER_SIZE 				8		// Ring buffer size [bytes]
#define NUM_PARAMETERS 				6		// Number of sensors used by ACM
#define BUFFER_OVERFLOW_MODULO 		7		// Prevent ring buffer pointer from going out of bounds
#define ERROR_THRESHOLD 			10		// Maximum number of errors a sensor can produce before being rendered "inoperative" [errors]
#define BRAKE_PRESSURE_COEFFICIENT 	1.0		// Coefficient in acceleration equation
#define THROTTLE_COEFFICIENT 		1.0		// Coefficient in acceleration equation
#define LINEAR_POSITION_TO_TICKS 	34.72	// Used to convert servo position value to timer ticks
#define MINIMUM_ANGLE_SERVO_TICKS 	1700	// Smallest angle the servo can go to, used to calculate timer ticks of servo PWM
#define TIMER_PERIOD 				60000	// Period of the PWM timer

//STEERING INDEX BOUNDS [degrees]
#define STEERING_INDEX_1_THRESHOLD -90
#define STEERING_INDEX_2_THRESHOLD -72
#define STEERING_INDEX_3_THRESHOLD -54
#define STEERING_INDEX_4_THRESHOLD 54
#define STEERING_INDEX_5_THRESHOLD 72

// Make sure that none of the STEERING_INDEX values overlap
#if STEERING_INDEX_1_THRESHOLD > STEERING_INDEX_2_THRESHOLD
#error *** STEERING_INDEX_1_THRESHOLD > STEERING_INDEX_2_THRESHOLD, values overlap ***
#endif

#if STEERING_INDEX_2_THRESHOLD > STEERING_INDEX_3_THRESHOLD
#error *** STEERING_INDEX_2_THRESHOLD > STEERING_INDEX_3_THRESHOLD, values overlap ***
#endif

#if STEERING_INDEX_3_THRESHOLD > STEERING_INDEX_4_THRESHOLD
#error *** STEERING_INDEX_3_THRESHOLD > STEERING_INDEX_4_THRESHOLD, values overlap ***
#endif

#if STEERING_INDEX_4_THRESHOLD > STEERING_INDEX_5_THRESHOLD
#error *** STEERING_INDEX_4_THRESHOLD > STEERING_INDEX_5_THRESHOLD, values overlap ***
#endif

//ACCELERATION INDEX BOUNDS [m/(s^2)]
#define ACCELERATION_INDEX_1_THRESHOLD 	-127
#define ACCELERATION_INDEX_2_THRESHOLD 	-112
#define ACCELERATION_INDEX_3_THRESHOLD 	-97
#define ACCELERATION_INDEX_4_THRESHOLD 	-82
#define ACCELERATION_INDEX_5_THRESHOLD 	-67
#define ACCELERATION_INDEX_6_THRESHOLD 	-52
#define ACCELERATION_INDEX_7_THRESHOLD 	-37
#define ACCELERATION_INDEX_8_THRESHOLD 	-22
#define ACCELERATION_INDEX_9_THRESHOLD 	-7
#define ACCELERATION_INDEX_10_THRESHOLD 8
#define ACCELERATION_INDEX_11_THRESHOLD 23
#define ACCELERATION_INDEX_12_THRESHOLD 38
#define ACCELERATION_INDEX_13_THRESHOLD 53
#define ACCELERATION_INDEX_14_THRESHOLD 68
#define ACCELERATION_INDEX_15_THRESHOLD 83
#define ACCELERATION_INDEX_16_THRESHOLD 98
#define ACCELERATION_INDEX_17_THRESHOLD 113

// Make sure that none of the acceleration indices overlap in value
#if ACCELERATION_INDEX_1_THRESHOLD > ACCELERATION_INDEX_2_THRESHOLD
#error *** ACCELERATION_INDEX_1_THRESHOLD > ACCELERATION_INDEX_2_THRESHOLD, VALUES OVERLAP ***
#endif

#if ACCELERATION_INDEX_2_THRESHOLD > ACCELERATION_INDEX_3_THRESHOLD
#error *** ACCELERATION_INDEX_2_THRESHOLD > ACCELERATION_INDEX_3_THRESHOLD, VALUES OVERLAP ***
#endif

#if ACCELERATION_INDEX_3_THRESHOLD > ACCELERATION_INDEX_4_THRESHOLD
#error *** ACCELERATION_INDEX_3_THRESHOLD > ACCELERATION_INDEX_4_THRESHOLD, VALUES OVERLAP ***
#endif

#if ACCELERATION_INDEX_4_THRESHOLD > ACCELERATION_INDEX_5_THRESHOLD
#error *** ACCELERATION_INDEX_4_THRESHOLD > ACCELERATION_INDEX_5_THRESHOLD
#endif

#if ACCELERATION_INDEX_5_THRESHOLD > ACCELERATION_INDEX_6_THRESHOLD
#error *** ACCELERATION_INDEX_5_THRESHOLD > ACCELERATION_INDEX_6_THRESHOLD
#endif

#if ACCELERATION_INDEX_6_THRESHOLD > ACCELERATION_INDEX_7_THRESHOLD
#error *** ACCELERATION_INDEX_6_THRESHOLD > ACCELERATION_INDEX_7_THRESHOLD
#endif

#if ACCELERATION_INDEX_7_THRESHOLD > ACCELERATION_INDEX_8_THRESHOLD
#error *** ACCELERATION_INDEX_7_THRESHOLD > ACCELERATION_INDEX_8_THRESHOLD
#endif

#if ACCELERATION_INDEX_8_THRESHOLD > ACCELERATION_INDEX_9_THRESHOLD
#error *** ACCELERATION_INDEX_8_THRESHOLD > ACCELERATION_INDEX_9_THRESHOLD
#endif

#if ACCELERATION_INDEX_9_THRESHOLD > ACCELERATION_INDEX_10_THRESHOLD
#error *** ACCELERATION_INDEX_9_THRESHOLD > ACCELERATION_INDEX_10_THRESHOLD
#endif

#if ACCELERATION_INDEX_10_THRESHOLD > ACCELERATION_INDEX_11_THRESHOLD
#error *** ACCELERATION_INDEX_10_THRESHOLD > ACCELERATION_INDEX_11_THRESHOLD
#endif

#if ACCELERATION_INDEX_11_THRESHOLD > ACCELERATION_INDEX_12_THRESHOLD
#error *** ACCELERATION_INDEX_11_THRESHOLD > ACCELERATION_INDEX_12_THRESHOLD
#endif

#if ACCELERATION_INDEX_12_THRESHOLD > ACCELERATION_INDEX_13_THRESHOLD
#error *** ACCELERATION_INDEX_12_THRESHOLD > ACCELERATION_INDEX_13_THRESHOLD
#endif

#if ACCELERATION_INDEX_13_THRESHOLD > ACCELERATION_INDEX_14_THRESHOLD
#error *** ACCELERATION_INDEX_13_THRESHOLD > ACCELERATION_INDEX_14_THRESHOLD
#endif

#if ACCELERATION_INDEX_14_THRESHOLD > ACCELERATION_INDEX_15_THRESHOLD
#error *** ACCELERATION_INDEX_14_THRESHOLD > ACCELERATION_INDEX_15_THRESHOLD
#endif

#if ACCELERATION_INDEX_15_THRESHOLD > ACCELERATION_INDEX_16_THRESHOLD
#error *** ACCELERATION_INDEX_15_THRESHOLD > ACCELERATION_INDEX_16_THRESHOLD
#endif

#if ACCELERATION_INDEX_16_THRESHOLD > ACCELERATION_INDEX_17_THRESHOLD
#error *** ACCELERATION_INDEX_16_THRESHOLD > ACCELERATION_INDEX_17_THRESHOLD
#endif

//SPEED INDEX BOUNDS [m/s]
#define SPEED_INDEX_1_THRESHOLD 	0
#define SPEED_INDEX_2_THRESHOLD 	5
#define SPEED_INDEX_3_THRESHOLD 	10
#define SPEED_INDEX_4_THRESHOLD 	15
#define SPEED_INDEX_5_THRESHOLD 	20
#define SPEED_INDEX_6_THRESHOLD 	25
#define SPEED_INDEX_7_THRESHOLD 	30
#define SPEED_INDEX_8_THRESHOLD 	35
#define SPEED_INDEX_9_THRESHOLD 	40
#define SPEED_INDEX_10_THRESHOLD 	45
#define SPEED_INDEX_11_THRESHOLD 	50
#define SPEED_INDEX_12_THRESHOLD 	55
#define SPEED_INDEX_13_THRESHOLD 	60
#define SPEED_INDEX_14_THRESHOLD 	65
#define SPEED_INDEX_15_THRESHOLD 	70
#define SPEED_INDEX_16_THRESHOLD 	75
#define SPEED_INDEX_17_THRESHOLD 	80

// Make sure that none of the speed indices overlap in value
#if SPEED_INDEX_1_THRESHOLD > SPEED_INDEX_2_THRESHOLD
#error *** SPEED_INDEX_1_THRESHOLD > SPEED_INDEX_2_THRESHOLD, VALUES OVERLAP ***
#endif

#if SPEED_INDEX_2_THRESHOLD > SPEED_INDEX_3_THRESHOLD
#error *** SPEED_INDEX_2_THRESHOLD > SPEED_INDEX_3_THRESHOLD, VALUES OVERLAP ***
#endif

#if SPEED_INDEX_3_THRESHOLD > SPEED_INDEX_4_THRESHOLD
#error *** SPEED_INDEX_3_THRESHOLD > SPEED_INDEX_4_THRESHOLD, VALUES OVERLAP ***
#endif

#if SPEED_INDEX_4_THRESHOLD > SPEED_INDEX_5_THRESHOLD
#error *** SPEED_INDEX_4_THRESHOLD > SPEED_INDEX_5_THRESHOLD
#endif

#if SPEED_INDEX_5_THRESHOLD > SPEED_INDEX_6_THRESHOLD
#error *** SPEED_INDEX_5_THRESHOLD > SPEED_INDEX_6_THRESHOLD
#endif

#if SPEED_INDEX_6_THRESHOLD > SPEED_INDEX_7_THRESHOLD
#error *** SPEED_INDEX_6_THRESHOLD > SPEED_INDEX_7_THRESHOLD
#endif

#if SPEED_INDEX_7_THRESHOLD > SPEED_INDEX_8_THRESHOLD
#error *** SPEED_INDEX_7_THRESHOLD > SPEED_INDEX_8_THRESHOLD
#endif

#if SPEED_INDEX_8_THRESHOLD > SPEED_INDEX_9_THRESHOLD
#error *** SPEED_INDEX_8_THRESHOLD > SPEED_INDEX_9_THRESHOLD
#endif

#if SPEED_INDEX_9_THRESHOLD > SPEED_INDEX_10_THRESHOLD
#error *** SPEED_INDEX_9_THRESHOLD > SPEED_INDEX_10_THRESHOLD
#endif

#if SPEED_INDEX_10_THRESHOLD > SPEED_INDEX_11_THRESHOLD
#error *** SPEED_INDEX_10_THRESHOLD > SPEED_INDEX_11_THRESHOLD
#endif

#if SPEED_INDEX_11_THRESHOLD > SPEED_INDEX_12_THRESHOLD
#error *** SPEED_INDEX_11_THRESHOLD > SPEED_INDEX_12_THRESHOLD
#endif

#if SPEED_INDEX_12_THRESHOLD > SPEED_INDEX_13_THRESHOLD
#error *** SPEED_INDEX_12_THRESHOLD > SPEED_INDEX_13_THRESHOLD
#endif

#if SPEED_INDEX_13_THRESHOLD > SPEED_INDEX_14_THRESHOLD
#error *** SPEED_INDEX_13_THRESHOLD > SPEED_INDEX_14_THRESHOLD
#endif

#if SPEED_INDEX_14_THRESHOLD > SPEED_INDEX_15_THRESHOLD
#error *** SPEED_INDEX_14_THRESHOLD > SPEED_INDEX_15_THRESHOLD
#endif

#if SPEED_INDEX_15_THRESHOLD > SPEED_INDEX_16_THRESHOLD
#error *** SPEED_INDEX_15_THRESHOLD > SPEED_INDEX_16_THRESHOLD
#endif

#if SPEED_INDEX_16_THRESHOLD > SPEED_INDEX_17_THRESHOLD
#error *** SPEED_INDEX_16_THRESHOLD > SPEED_INDEX_17_THRESHOLD
#endif


// Wing positions [respective servo degrees]
#define FRONT_LEFT_WING_MANUAL_POSITION 	100
#define FRONT_RIGHT_WING_MANUAL_POSITION 	100
#define REAR_WING_MANUAL_POSITION 			100

// Wing positions [respective servo degrees]
#define FRONT_LEFT_WING_DRS_POSITION 		100
#define FRONT_RIGHT_WING_DRS_POSITION 		100
#define REAR_WING_DRS_POSITION 				100

// Wheel speed bounds [mph?]
#define WHEEL_SPEED_UPPER_BOUND 			100
#define WHEEL_SPEED_LOWER_BOUND 			0

// Air speed bounds [mph?]
#define AIR_SPEED_UPPER_BOUND 				100
#define AIR_SPEED_LOWER_BOUND 				0

// Throttle position bounds [percent]
#define THROTTLE_POSITION_UPPER_BOUND 		100
#define THROTTLE_POSITION_LOWER_BOUND 		0

// Steering angle bounds [degrees]
#define STEERING_ANGLE_UPPER_BOUND 			100
#define STEERING_ANGLE_LOWER_BOUND 			0

// Brake pressure bounds [psi]
#define BRAKE_PRESSURE_UPPER_BOUND	 		100
#define BRAKE_PRESSURE_LOWER_BOUND 			0

// Acceleration bounds [m/(s^2)]
#define ACCELERATION_UPPER_BOUND 			100
#define ACCELERATION_LOWER_BOUND 			0

// Create the 3D maps
S8 front_left_wing_map[17][5][17];		//speed, steering, acceleration (order of dimensions)
S8 front_right_wing_map[17][5][17];		//speed, steering, acceleration (order of dimensions)
S8 rear_wing_map[17][5][17];			//speed, steering, acceleration (order of dimensions)
S8 front_left_wing_map_position;		//Position in the map for the front left wing
S8 front_right_wing_map_position;		//Position in the map for the front right wing
S8 rear_wing_map_position;				//Position in the map for the rear wing

typedef enum {
    OUT_OF_BOUNDS = 0, // Current data has not been used in a calculation yet
    IN_BOUNDS = 1,   // Current data HAS been used in a calculation - dont double count
} IN_BOUNDS_STATE;

typedef enum {
	INOPERATIVE = 0,	// There are too many errors within the specific parameters error checking time frame
	OPERATIVE = 1,		// There are not enough errors within the specific parameters error checking time frame to cause problems
} PARAMETER_STATE;

typedef enum {
	MANUAL = 1,			// ACM is controlled by the driver
	AUTO = 0,			// ACM don't take orders from no one
} ACM_CONTROL_STATE;

// struct for parameters that can be negative
typedef struct {
	S16 				current_value;				// received value (for negative parameters)
	IN_BOUNDS_STATE 	in_bounds_flag;				// SET if value IN BOUNDS
	S16 				buffer[BUFFER_SIZE];		// Ring buffer for parameter values
	U8 					buffer_index;				// location in ring buffer
	double 				buffer_average;				// average buffer value
	S16					upper_bound;				// Max value
	S16					lower_bound;				// Min value
	U8 					error_count;				// Count how many times a value has been received out-of-bounds
	PARAMETER_STATE		parameter_state;			// If there are too many errors
}ACM_parameter;

/***************************** Function Prototypes *****************************/
void ACM_Init(void);											// Run at startup
void fetch_data(void);											// Gets the data and does pre-calculations
void calculate_wing_angle(void);								// Uses the filtered and "pre-calculated" to lookup position in 3d map
void arbitrate_speed(void);										// Calculate what the speed value input to the map is
void arbitrate_acceleration(void);								// Calculate what the acceleration value input to the map is
void arbitrate_steering_angle(void);							// Calculate what the steering angle value input to the map is
void output_angles(void);										// Change PWM to servos corresponding to position in 3d map
void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);	// Change PWM
void get_btn_state(void);										// Read DRS button

#endif /* INC_GO4_ACM_2020_H_ */
