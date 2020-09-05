/*
 * GO4_ACM_2020.c
 *
 *  Created on: Sep 3, 2020
 *      Author: bents
 */


//SYSTEM INCLUDES
#include "cmsis_os.h"
#include "main.h"

// Project Includes
#include "base_types.h"
#include "GO4_ACM_2020.h"

extern DMA_HandleTypeDef hdma_adc;

/*
void ACM_Init(void);
void fetch_data(void);
void calculate_wing_angle(void);
void arbitrate_speed(void);
*/

static ACM_parameter wheel_speed =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= WHEEL_SPEED_UPPER_BOUND,
		.lower_bound		= WHEEL_SPEED_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE
};

static ACM_parameter air_speed =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= AIR_SPEED_UPPER_BOUND,
		.lower_bound		= AIR_SPEED_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE
};

static ACM_parameter throttle_position =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= THROTTLE_POSITION_UPPER_BOUND,
		.lower_bound		= THROTTLE_POSITION_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE
};

static ACM_parameter steering_angle =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= STEERING_ANGLE_UPPER_BOUND,
		.lower_bound		= STEERING_ANGLE_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE

};

static ACM_parameter brake_pressure =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= BRAKE_PRESSURE_UPPER_BOUND,
		.lower_bound		= BRAKE_PRESSURE_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE
};

static ACM_parameter acceleration =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= ACCELERATION_UPPER_BOUND,
		.lower_bound		= ACCELERATION_LOWER_BOUND,
		.error_count		= 0,
		.parameter_state	= OPERATIVE
};

//List of parameters for filtering and calculation
ACM_parameter parameters[NUM_PARAMETERS];

void ACM_Init(void) {

	parameters[0] = wheel_speed;
	parameters[1] = air_speed;
	parameters[2] = throttle_position;
	parameters[3] = steering_angle;
	parameters[4] = brake_pressure;
	parameters[5] = acceleration;


	// Start Timers for CAN?
	// Do the Wave
}

void fetch_data(void) {
	ACM_parameter* global_parameter_list;	// List of all of the parameters
	U8 i;
	U8 average_index;
	U16 average_aggregate;

	for(i = 0, global_parameter_list = parameters;i < NUM_PARAMETERS;i++) {
		// Try to Receive Data over CAN
		// global_parameter_list->current_value = fetch_parameter_function(specific parameter);

		// Skip this parameter if it is INOPERATIVE (too many errors)
		if(global_parameter_list->parameter_state != OPERATIVE) {
			continue;

			// Possibly implement a restart feature??
		}

		if ((global_parameter_list->current_value >= global_parameter_list->upper_bound) ||
				(global_parameter_list->current_value <= global_parameter_list->lower_bound)) {

			// Add current value to buffer
			global_parameter_list->buffer[global_parameter_list->buffer_index] = global_parameter_list->current_value;	// Add current value to the buffer index of the specific variables buffer
		} else {
			//Add 1 to error count because current_value is out of bounds
			global_parameter_list->error_count += 1;

			// Render parameter INOPERATIVE if parameter has an error count that is too high
			if (global_parameter_list->error_count >= ERROR_THRESHOLD) {
				global_parameter_list->parameter_state = INOPERATIVE;
			}
		}

		// add 1 to buffer and prevent buffer_index from going out of bounds
		global_parameter_list->buffer_index += 1;
		global_parameter_list->buffer_index = global_parameter_list->buffer_index % BUFFER_OVERFLOW_MODULO;

		// Aggregate all buffer values to calculate average
		for(average_index = 0;average_index < BUFFER_SIZE; average_index++) {
			average_aggregate += global_parameter_list->buffer[average_index];
		}

		// Assign buffer average to specific "object"
		global_parameter_list->buffer_average = average_aggregate / BUFFER_SIZE;
	}
	average_aggregate = 0;		// Reset aggregate variable to correctly calculate average for all parameters
}

void calculate_wing_angle(void) {

}

void arbitrate_speed(void) {
	double average_speed;
	double previous_average_speed;
	double acceleration;
	ACM_CONTROL_STATE state;

	if (air_speed.parameter_state == INOPERATIVE && wheel_speed.parameter_state == INOPERATIVE)
	{
		// Completely skunked
		state = MANUAL;
	} else {
		state = AUTO;

		// Determine which speed to use
		if (air_speed.parameter_state == INOPERATIVE) {
			average_speed = wheel_speed.buffer_average;
		} else if (wheel_speed.parameter_state == INOPERATIVE) {
			average_speed = air_speed.buffer_average;
		} else {
			average_speed = wheel_speed.buffer_average;
		}

		// Calculate acceleration in some way (derivative/difference of velocity and timer)
	}

}
