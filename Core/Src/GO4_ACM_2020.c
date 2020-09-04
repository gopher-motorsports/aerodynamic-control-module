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
		.lower_bound		= WHEEL_SPEED_LOWER_BOUND
};

static ACM_parameter air_speed =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= AIR_SPEED_UPPER_BOUND,
		.lower_bound		= AIR_SPEED_LOWER_BOUND
};

static ACM_parameter throttle_position =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= THROTTLE_POSITION_UPPER_BOUND,
		.lower_bound		= THROTTLE_POSITION_LOWER_BOUND
};

static ACM_parameter steering_angle =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= STEERING_ANGLE_UPPER_BOUND,
		.lower_bound		= STEERING_ANGLE_LOWER_BOUND

};

static ACM_parameter brake_pressure =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= BRAKE_PRESSURE_UPPER_BOUND,
		.lower_bound		= BRAKE_PRESSURE_LOWER_BOUND
};

static ACM_parameter acceleration =
{
		.current_value 		= 0,
		.in_bounds_flag 	= IN_BOUNDS,
		.buffer 			= {0},
		.buffer_index		= 0,
		.buffer_average		= 0,
		.upper_bound		= ACCELERATION_UPPER_BOUND,
		.lower_bound		= ACCELERATION_LOWER_BOUND
};

static ACM_parameter parameters[] = {wheel_speed, air_speed, throttle_position, steering_angle,
		brake_pressure, acceleration
};

void ACM_Init(void) {
	// Start Timers for CAN?
	// Do the Wave
}

void fetch_data(void) {
	ACM_parameter* current_parameter;



	// Try to Receive Data over CAN
	// Assign all _val variables here

	if(1)
	{
		wheel_speed.in_bounds_flag = OUT_OF_BOUNDS;
	} else {
		wheel_speed.in_bounds_flag = IN_BOUNDS;


	}
}

void calculate_wing_angle(void) {

}

void arbitrate_speed(void) {

}
