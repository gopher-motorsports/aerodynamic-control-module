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
#define BUFFER_SIZE 8
#define NUM_UNSIGNED_PARAMETERS 4
#define NUM_SIGNED_PARAMETERS 2
#define BUFFER_OVERFLOW_MODULO 7
#define ERROR_THRESHOLD 10
#define SIMILAR_THRESHOLD 5
#define BRAKE_PRESSURE_COEFFICIENT 1.0
#define THROTTLE_COEFFICIENT 1.0
#define LINEAR_POSITION_TO_TICKS 34.72
#define MINIMUM_ANGLE_IN_TICKS 1700
#define TIMER_PERIOD 60000

#define STEERING_INDEX_1_LOWER_BOUND -90
#define STEERING_INDEX_1_UPPER_BOUND -72
#define STEERING_INDEX_2_LOWER_BOUND -72
#define STEERING_INDEX_2_UPPER_BOUND -54
#define STEERING_INDEX_3_LOWER_BOUND -54
#define STEERING_INDEX_3_UPPER_BOUND 54
#define STEERING_INDEX_4_LOWER_BOUND 54
#define STEERING_INDEX_4_UPPER_BOUND 72
#define STEERING_INDEX_5_LOWER_BOUND 72
#define STEERING_INDEX_5_UPPER_BOUND 90

//ACCELERATION INDEX BOUNDS
#define ACCELERATION_INDEX_1_LOWER_BOUND -127
#define ACCELERATION_INDEX_1_UPPER_BOUND -112
#define ACCELERATION_INDEX_2_LOWER_BOUND -112
#define ACCELERATION_INDEX_2_UPPER_BOUND -97
#define ACCELERATION_INDEX_3_LOWER_BOUND -97
#define ACCELERATION_INDEX_3_UPPER_BOUND -82
#define ACCELERATION_INDEX_4_LOWER_BOUND -82
#define ACCELERATION_INDEX_4_UPPER_BOUND -67
#define ACCELERATION_INDEX_5_LOWER_BOUND -67
#define ACCELERATION_INDEX_5_UPPER_BOUND -52
#define ACCELERATION_INDEX_6_LOWER_BOUND -52
#define ACCELERATION_INDEX_6_UPPER_BOUND -37
#define ACCELERATION_INDEX_7_LOWER_BOUND -37
#define ACCELERATION_INDEX_7_UPPER_BOUND -22
#define ACCELERATION_INDEX_8_LOWER_BOUND -22
#define ACCELERATION_INDEX_8_UPPER_BOUND -7
#define ACCELERATION_INDEX_9_LOWER_BOUND -7
#define ACCELERATION_INDEX_9_UPPER_BOUND 8
#define ACCELERATION_INDEX_10_LOWER_BOUND 8
#define ACCELERATION_INDEX_10_UPPER_BOUND 23
#define ACCELERATION_INDEX_11_LOWER_BOUND 23
#define ACCELERATION_INDEX_11_UPPER_BOUND 38
#define ACCELERATION_INDEX_12_LOWER_BOUND 38
#define ACCELERATION_INDEX_12_UPPER_BOUND 53
#define ACCELERATION_INDEX_13_LOWER_BOUND 53
#define ACCELERATION_INDEX_13_UPPER_BOUND 68
#define ACCELERATION_INDEX_14_LOWER_BOUND 68
#define ACCELERATION_INDEX_14_UPPER_BOUND 83
#define ACCELERATION_INDEX_15_LOWER_BOUND 83
#define ACCELERATION_INDEX_15_UPPER_BOUND 98
#define ACCELERATION_INDEX_16_LOWER_BOUND 98
#define ACCELERATION_INDEX_16_UPPER_BOUND 113
#define ACCELERATION_INDEX_17_LOWER_BOUND 113
#define ACCELERATION_INDEX_17_UPPER_BOUND 127

//SPEED INDEX BOUNDS
#define SPEED_INDEX_1_LOWER_BOUND 0
#define SPEED_INDEX_1_UPPER_BOUND 5
#define SPEED_INDEX_2_LOWER_BOUND 5
#define SPEED_INDEX_2_UPPER_BOUND 10
#define SPEED_INDEX_3_LOWER_BOUND 10
#define SPEED_INDEX_3_UPPER_BOUND 15
#define SPEED_INDEX_4_LOWER_BOUND 15
#define SPEED_INDEX_4_UPPER_BOUND 20
#define SPEED_INDEX_5_LOWER_BOUND 20
#define SPEED_INDEX_5_UPPER_BOUND 25
#define SPEED_INDEX_6_LOWER_BOUND 25
#define SPEED_INDEX_6_UPPER_BOUND 30
#define SPEED_INDEX_7_LOWER_BOUND 30
#define SPEED_INDEX_7_UPPER_BOUND 35
#define SPEED_INDEX_8_LOWER_BOUND 35
#define SPEED_INDEX_8_UPPER_BOUND 40
#define SPEED_INDEX_9_LOWER_BOUND 40
#define SPEED_INDEX_9_UPPER_BOUND 45
#define SPEED_INDEX_10_LOWER_BOUND 45
#define SPEED_INDEX_10_UPPER_BOUND 50
#define SPEED_INDEX_11_LOWER_BOUND 50
#define SPEED_INDEX_11_UPPER_BOUND 55
#define SPEED_INDEX_12_LOWER_BOUND 55
#define SPEED_INDEX_12_UPPER_BOUND 60
#define SPEED_INDEX_13_LOWER_BOUND 60
#define SPEED_INDEX_13_UPPER_BOUND 65
#define SPEED_INDEX_14_LOWER_BOUND 65
#define SPEED_INDEX_14_UPPER_BOUND 70
#define SPEED_INDEX_15_LOWER_BOUND 70
#define SPEED_INDEX_15_UPPER_BOUND 75
#define SPEED_INDEX_16_LOWER_BOUND 75
#define SPEED_INDEX_16_UPPER_BOUND 80
#define SPEED_INDEX_17_LOWER_BOUND 80
#define SPEED_INDEX_17_UPPER_BOUND 85


#define FRONT_LEFT_WING_MANUAL_POSITION 100
#define FRONT_RIGHT_WING_MANUAL_POSITION 100
#define REAR_WING_MANUAL_POSITION 100

#define FRONT_LEFT_WING_DRS_POSITION 100
#define FRONT_RIGHT_WING_DRS_POSITION 100
#define REAR_WING_DRS_POSITION 100

#define WHEEL_SPEED_UPPER_BOUND 100
#define WHEEL_SPEED_LOWER_BOUND 0

#define AIR_SPEED_UPPER_BOUND 100
#define AIR_SPEED_LOWER_BOUND 0

#define THROTTLE_POSITION_UPPER_BOUND 100
#define THROTTLE_POSITION_LOWER_BOUND 0

#define STEERING_ANGLE_UPPER_BOUND 100
#define STEERING_ANGLE_LOWER_BOUND 0

#define BRAKE_PRESSURE_UPPER_BOUND 100
#define BRAKE_PRESSURE_LOWER_BOUND 0

#define ACCELERATION_UPPER_BOUND 100
#define ACCELERATION_LOWER_BOUND 0

// Create the maps
S8 front_left_wing_map[17][5][17];		//speed, steering, acceleration (order of dimensions)
S8 front_right_wing_map[17][5][17];		//speed, steering, acceleration (order of dimensions)
S8 rear_wing_map[17][5][17];			//speed, steering, acceleration (order of dimensions)
S8 front_left_wing_map_position;
S8 front_right_wing_map_position;
S8 rear_wing_map_position;

typedef enum {
    OUT_OF_BOUNDS = 0, // Current data has not been used in a calculation yet
    IN_BOUNDS = 1,   // Current data HAS been used in a calculation - dont double count
} IN_BOUNDS_STATE;

typedef enum {
	INOPERATIVE = 0,	// There are too many errors within the specific parameters error checking time frame
	OPERATIVE = 1,		// There are not enough errors within the specific parameters error checking time frame to cause problems
} PARAMETER_STATE;

typedef enum {
	MANUAL = 1,			// ACM only takes input from the driver
	AUTO = 0,			// ACM only takes orders from itself
} ACM_CONTROL_STATE;

typedef struct {
	U8 					current_value;			// received value
	IN_BOUNDS_STATE 	in_bounds_flag;			// SET if value IN BOUNDS
	U8 					buffer[BUFFER_SIZE];	// Ring buffer for parameter values
	U8 					buffer_index;			// location in ring buffer
	double 				buffer_average;			// average buffer value
	U8					upper_bound;			// Max value
	U8					lower_bound;			// Min value
	U8 					error_count;			// Count how many times a value has been received out-of-bounds
	PARAMETER_STATE		parameter_state;		// If there are too many errors

}ACM_parameter_unsigned;

// struct for parameters that can be negative
typedef struct {
	S8 					current_value;			// received value (for negative parameters)
	IN_BOUNDS_STATE 	in_bounds_flag;			// SET if value IN BOUNDS
	S8 					buffer[BUFFER_SIZE];	// Ring buffer for parameter values
	U8 					buffer_index;			// location in ring buffer
	double 				buffer_average;			// average buffer value
	S8					upper_bound;			// Max value
	S8					lower_bound;			// Min value
	U8 					error_count;			// Count how many times a value has been received out-of-bounds
	PARAMETER_STATE		parameter_state;		// If there are too many errors
}ACM_parameter_signed;

void ACM_Init(void);
void fetch_data(void);
void calculate_wing_angle(void);
void arbitrate_speed(void);
void arbitrate_acceleration(void);
void arbitrate_steering_angle(void);
void output_angles(void);
void setPWM(TIM_HandleTypeDef, uint32_t, uint16_t, uint16_t);
void get_btn_state(void);

#endif /* INC_GO4_ACM_2020_H_ */
