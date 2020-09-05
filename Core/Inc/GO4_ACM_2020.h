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
#define NUM_PARAMETERS 6
#define BUFFER_OVERFLOW_MODULO 7
#define ERROR_THRESHOLD 10
#define SIMILAR_THRESHOLD 5

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

typedef enum {
    OUT_OF_BOUNDS = 0, // Current data has not been used in a calculation yet
    IN_BOUNDS = 1,   // Current data HAS been used in a calculation - dont double count
} IN_BOUNDS_STATE;

typedef enum {
	INOPERATIVE = 0,	// There are too many errors within the specific parameters error checking time frame
	OPERATIVE = 1,		// There are not enough errors within the specific parameters error checking time frame to cause problems
}PARAMETER_STATE;

typedef enum {
	MANUAL = 1,
	AUTO = 0,
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

}ACM_parameter;

void ACM_Init(void);
void fetch_data(void);
void calculate_wing_angle(void);
void arbitrate_speed(void);

#endif /* INC_GO4_ACM_2020_H_ */
