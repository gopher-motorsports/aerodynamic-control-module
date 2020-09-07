/*
 * GO4_ACM_2020.c
 *
 *  Created on: Sep 3, 2020
 *      Author: bents
 */

/* TODO:
 * 1. Acceleration sign
 * 2. CAN
 */

//SYSTEM INCLUDES
#include "cmsis_os.h"
#include "main.h"

// Project Includes
#include "base_types.h"
#include "GO4_ACM_2020.h"

extern TIM_HandleTypeDef htim3;

static ACM_parameter_unsigned wheel_speed =
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

static ACM_parameter_unsigned air_speed =
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

static ACM_parameter_unsigned throttle_position =
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

static ACM_parameter_signed steering_angle =
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

static ACM_parameter_unsigned brake_pressure =
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

static ACM_parameter_signed acceleration =
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

ACM_parameter_unsigned unsigned_parameters[NUM_UNSIGNED_PARAMETERS];	// unsigned parameters that are read over CAN
ACM_parameter_signed signed_parameters[NUM_SIGNED_PARAMETERS];			// signed parameters that are read over CAN
U8 speed_index;						// speed variable used in the big map (0 is 0 mph)
U8 steering_angle_index;			// steering angle variable used in the big map (127 is 0 degrees)
U8 acceleration_index;				// acceleration variable used in the big map (127 is 0 g)
U16 front_left_servo_ticks;
U16 front_right_servo_ticks;
U16 rear_servo_ticks;
U8 drs_button_state;
ACM_CONTROL_STATE control_state;	// the state of control (Either AUTO - controlled by the ACM or MANUAL - Driver controlled)

void ACM_Init(void) {

	// all unsigned parameters
	unsigned_parameters[0] = wheel_speed;
	unsigned_parameters[1] = air_speed;
	unsigned_parameters[2] = throttle_position;
	unsigned_parameters[3] = brake_pressure;

	// all signed parameters
	signed_parameters[0] = acceleration;
	signed_parameters[1] = steering_angle;


	// Start Timers for CAN?
	// Do the Wave
}

void fetch_data(void) {
	ACM_parameter_unsigned* global_unsigned_parameter_list;	// List of all of the unsigned parameters
	ACM_parameter_signed* global_signed_parameter_list;		// List of all of the signed parameters
	U8 average_index;										// index of the next element added to the sum to find average
	S16 average_aggregate;									// sum of the elements before the average_index

	for(global_unsigned_parameter_list = unsigned_parameters;global_unsigned_parameter_list < global_unsigned_parameter_list + NUM_UNSIGNED_PARAMETERS;global_unsigned_parameter_list++) {
		/******************************   CAN   ******************************/
		// Try to Receive specific chunk of data over CAN
		// global_parameter_list->current_value = fetch_parameter_function(specific parameter);

	}

	for(global_signed_parameter_list = signed_parameters;global_signed_parameter_list < global_signed_parameter_list + NUM_SIGNED_PARAMETERS;global_signed_parameter_list++) {
		/******************************   CAN   ******************************/
		// Try to Receive specific chunk of data over CAN
		// global_parameter_list->current_value = fetch_parameter_function(specific parameter);
	}

	//***************************  update unsigned data  ***************************//
	for(global_unsigned_parameter_list = unsigned_parameters;global_unsigned_parameter_list < global_unsigned_parameter_list + NUM_UNSIGNED_PARAMETERS;global_unsigned_parameter_list++) {

		// Skip this parameter if it is INOPERATIVE (too many errors)
		if(global_unsigned_parameter_list->parameter_state != OPERATIVE) {
			continue;
		}

		if ((global_unsigned_parameter_list->current_value >= global_unsigned_parameter_list->upper_bound) ||
				(global_unsigned_parameter_list->current_value <= global_unsigned_parameter_list->lower_bound)) {

			// Add current value to buffer
			global_unsigned_parameter_list->buffer[global_unsigned_parameter_list->buffer_index] = global_unsigned_parameter_list->current_value;	// Add current value to the buffer index of the specific variables buffer
		} else {
			//Add 1 to error count because current_value is out of bounds
			global_unsigned_parameter_list->error_count += 1;

			// Render parameter INOPERATIVE if parameter has an error count that is too high
			if (global_unsigned_parameter_list->error_count >= ERROR_THRESHOLD) {
				global_unsigned_parameter_list->parameter_state = INOPERATIVE;
			}
		}

		// add 1 to buffer and prevent buffer_index from going out of bounds
		global_unsigned_parameter_list->buffer_index += 1;
		global_unsigned_parameter_list->buffer_index = global_unsigned_parameter_list->buffer_index % BUFFER_OVERFLOW_MODULO;

		// Aggregate all buffer values to calculate average
		for(average_index = 0;average_index < BUFFER_SIZE; average_index++) {
			average_aggregate += global_unsigned_parameter_list->buffer[average_index];
		}

		// Assign buffer average to specific "object"
		global_unsigned_parameter_list->buffer_average = average_aggregate / BUFFER_SIZE;
		average_aggregate = 0;		// Reset aggregate variable to correctly calculate average for all parameters
	}


	//***************************  update signed data  ***************************//
	for(global_signed_parameter_list = signed_parameters;global_unsigned_parameter_list < global_unsigned_parameter_list + NUM_SIGNED_PARAMETERS;global_signed_parameter_list++) {

		// Skip this parameter if it is INOPERATIVE (too many errors)
		if(global_signed_parameter_list->parameter_state != OPERATIVE) {
			continue;
		}

		if ((global_signed_parameter_list->current_value >= global_signed_parameter_list->upper_bound) ||
				(global_signed_parameter_list->current_value <= global_signed_parameter_list->lower_bound)) {

			// Add current value to buffer
			global_signed_parameter_list->buffer[global_signed_parameter_list->buffer_index] = global_signed_parameter_list->current_value;	// Add current value to the buffer index of the specific variables buffer
		} else {
			//Add 1 to error count because current_value is out of bounds
			global_signed_parameter_list->error_count += 1;

			// Render parameter INOPERATIVE if parameter has an error count that is too high
			if (global_signed_parameter_list->error_count >= ERROR_THRESHOLD) {
				global_signed_parameter_list->parameter_state = INOPERATIVE;
			}
		}

		// add 1 to buffer and prevent buffer_index from going out of bounds
		global_signed_parameter_list->buffer_index += 1;
		global_signed_parameter_list->buffer_index = global_signed_parameter_list->buffer_index % BUFFER_OVERFLOW_MODULO;

		// Aggregate all buffer values to calculate average
		for(average_index = 0;average_index < BUFFER_SIZE; average_index++) {
			average_aggregate += global_signed_parameter_list->buffer[average_index];
		}

		// Assign buffer average to specific "object"
		global_signed_parameter_list->buffer_average = average_aggregate / BUFFER_SIZE;
		average_aggregate = 0;		// Reset aggregate variable to correctly calculate average for all parameters
	}

}

void calculate_wing_angle(void) {
	double temp_front_left_servo_ticks;
	double temp_front_right_servo_ticks;
	double temp_rear_servo_ticks;

	if (control_state == AUTO && drs_button_state == 0) {
		//super cool kickflip pointer math for locating the piece of data in the map
		front_left_wing_map_position 	= *(*(*(front_left_wing_map + speed_index) + steering_angle_index) + acceleration_index);
		front_right_wing_map_position 	= *(*(*(front_right_wing_map + speed_index) + steering_angle_index) + acceleration_index);
		rear_wing_map_position 			= *(*(*(rear_wing_map + speed_index) + steering_angle_index) + acceleration_index);

		// convert angle to timer count
		temp_front_left_servo_ticks = LINEAR_POSITION_TO_TICKS * front_left_wing_map_position;
		temp_front_right_servo_ticks = LINEAR_POSITION_TO_TICKS * front_right_wing_map_position;
		temp_rear_servo_ticks = LINEAR_POSITION_TO_TICKS * rear_wing_map_position;

		//convert angle to timer count
		front_left_servo_ticks = (U16)temp_front_left_servo_ticks + 1700;
		front_right_servo_ticks = (U16)temp_front_right_servo_ticks + 1700;
		rear_servo_ticks = (U16)temp_rear_servo_ticks + 1700;
	} else if(control_state == MANUAL && drs_button_state == 0) {
		//Manual Mode
		front_left_wing_map_position = FRONT_LEFT_WING_MANUAL_POSITION;
		front_right_wing_map_position = FRONT_RIGHT_WING_MANUAL_POSITION;
		rear_wing_map_position = REAR_WING_MANUAL_POSITION;

		temp_front_left_servo_ticks = LINEAR_POSITION_TO_TICKS * front_left_wing_map_position;
		temp_front_right_servo_ticks = LINEAR_POSITION_TO_TICKS * front_right_wing_map_position;
		temp_rear_servo_ticks = LINEAR_POSITION_TO_TICKS * rear_wing_map_position;

		//convert angle to timer count
		front_left_servo_ticks = (U16)temp_front_left_servo_ticks + 1700;
		front_right_servo_ticks = (U16)temp_front_right_servo_ticks + 1700;
		rear_servo_ticks = (U16)temp_rear_servo_ticks + 1700;
	} else {
		// DRS ACTIVATED
		front_left_wing_map_position = FRONT_LEFT_WING_DRS_POSITION;
		front_right_wing_map_position = FRONT_RIGHT_WING_DRS_POSITION;
		rear_wing_map_position = REAR_WING_DRS_POSITION;

		temp_front_left_servo_ticks = LINEAR_POSITION_TO_TICKS * front_left_wing_map_position;
		temp_front_right_servo_ticks = LINEAR_POSITION_TO_TICKS * front_right_wing_map_position;
		temp_rear_servo_ticks = LINEAR_POSITION_TO_TICKS * rear_wing_map_position;

		//convert angle to timer count
		front_left_servo_ticks = (U16)temp_front_left_servo_ticks + 1700;
		front_right_servo_ticks = (U16)temp_front_right_servo_ticks + 1700;
		rear_servo_ticks = (U16)temp_rear_servo_ticks + 1700;
	}
}

void arbitrate_acceleration(void) {
	double average_acceleration;

	// if (throttle position or brake pressure) and acceleration are inoperative then go into manual mode
	if ((throttle_position.parameter_state == INOPERATIVE || brake_pressure.parameter_state == INOPERATIVE) && (acceleration.parameter_state == INOPERATIVE)) {
		control_state = MANUAL;
	}

	// arbitrate acceleration only if the control mode is set to AUTO
	if (control_state == AUTO) {

		// If either the throttle position or brake pressure is inoperative then use the accelerometer
		if (throttle_position.parameter_state == INOPERATIVE || brake_pressure.parameter_state == INOPERATIVE) {

			// assign acceleration index for map. 127 is effectively 0 acceleration
			average_acceleration = acceleration.buffer_average + 127;

			// assign acceleration index
			if (average_acceleration >= ACCELERATION_INDEX_1_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_1_UPPER_BOUND) {
				acceleration_index = 0;
			} else if (average_acceleration >= ACCELERATION_INDEX_2_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_2_UPPER_BOUND) {
				acceleration_index = 1;
			} else if (average_acceleration >= ACCELERATION_INDEX_3_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_3_UPPER_BOUND) {
				acceleration_index = 2;
			} else if (average_acceleration >= ACCELERATION_INDEX_4_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_4_LOWER_BOUND) {
				acceleration_index = 3;
			} else if (average_acceleration >= ACCELERATION_INDEX_5_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_5_UPPER_BOUND) {
				acceleration_index = 4;
			} else if (average_acceleration >= ACCELERATION_INDEX_6_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_6_UPPER_BOUND) {
				acceleration_index = 5;
			} else if (average_acceleration >= ACCELERATION_INDEX_7_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_7_UPPER_BOUND) {
				acceleration_index = 6;
			} else if (average_acceleration >= ACCELERATION_INDEX_8_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_8_UPPER_BOUND) {
				acceleration_index = 7;
			} else if (average_acceleration >= ACCELERATION_INDEX_9_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_9_UPPER_BOUND) {
				acceleration_index = 8;
			} else if (average_acceleration >= ACCELERATION_INDEX_10_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_10_UPPER_BOUND) {
				acceleration_index = 9;
			} else if (average_acceleration >= ACCELERATION_INDEX_11_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_11_UPPER_BOUND) {
				acceleration_index = 10;
			} else if (average_acceleration >= ACCELERATION_INDEX_12_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_12_UPPER_BOUND) {
				acceleration_index = 11;
			} else if (average_acceleration >= ACCELERATION_INDEX_13_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_13_UPPER_BOUND) {
				acceleration_index = 12;
			} else if (average_acceleration >= ACCELERATION_INDEX_14_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_14_UPPER_BOUND) {
				acceleration_index = 13;
			} else if (average_acceleration >= ACCELERATION_INDEX_15_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_15_UPPER_BOUND) {
				acceleration_index = 14;
			} else if (average_acceleration >= ACCELERATION_INDEX_16_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_16_UPPER_BOUND) {
				acceleration_index = 15;
			} else if (average_acceleration >= ACCELERATION_INDEX_17_LOWER_BOUND && average_acceleration < ACCELERATION_INDEX_17_UPPER_BOUND) {
				acceleration_index = 16;
			}
		} else {
			// otherwise just use this equation to map the acceleration parameter
			average_acceleration = (THROTTLE_COEFFICIENT * throttle_position.buffer_average) - (BRAKE_PRESSURE_COEFFICIENT * brake_pressure.buffer_average);
			acceleration_index = (S8)average_acceleration;
		}
	}
}

void arbitrate_speed(void) {

	// if both airspeed and wheelspeed are inoperative then go into manual mode
	if (air_speed.parameter_state == INOPERATIVE && wheel_speed.parameter_state == INOPERATIVE)
	{
		// Completely skunked
		control_state = MANUAL;
	}

	// if control state is not AUTO don't do anything to speed_index
	if (control_state == AUTO) {

		// if the airspeed is inoperative use the wheelspeed, and vice-versa, if both are good then use wheelspeed
		if (air_speed.parameter_state == INOPERATIVE) {
			speed_index = (U8)wheel_speed.buffer_average;
		} else if (wheel_speed.parameter_state == INOPERATIVE) {
			speed_index = (U8)air_speed.buffer_average;
		} else {
			speed_index = (U8)wheel_speed.buffer_average;	// use wheelspeed as "default"
		}

		// ASSIGN SPEED_INDEX
		if (speed_index >= SPEED_INDEX_1_LOWER_BOUND && speed_index < SPEED_INDEX_1_UPPER_BOUND) {
			speed_index = 0;
		} else if (speed_index >= SPEED_INDEX_2_LOWER_BOUND && speed_index < SPEED_INDEX_2_UPPER_BOUND) {
			speed_index = 1;
		} else if (speed_index >= SPEED_INDEX_3_LOWER_BOUND && speed_index < SPEED_INDEX_3_UPPER_BOUND) {
			speed_index = 2;
		} else if (speed_index >= SPEED_INDEX_4_LOWER_BOUND && speed_index < SPEED_INDEX_4_LOWER_BOUND) {
			speed_index = 3;
		} else if (speed_index >= SPEED_INDEX_5_LOWER_BOUND && speed_index < SPEED_INDEX_5_UPPER_BOUND) {
			speed_index = 4;
		} else if (speed_index >= SPEED_INDEX_6_LOWER_BOUND && speed_index < SPEED_INDEX_6_UPPER_BOUND) {
			speed_index = 5;
		} else if (speed_index >= SPEED_INDEX_7_LOWER_BOUND && speed_index < SPEED_INDEX_7_UPPER_BOUND) {
			speed_index = 6;
		} else if (speed_index >= SPEED_INDEX_8_LOWER_BOUND && speed_index < SPEED_INDEX_8_UPPER_BOUND) {
			speed_index = 7;
		} else if (speed_index >= SPEED_INDEX_9_LOWER_BOUND && speed_index < SPEED_INDEX_9_UPPER_BOUND) {
			speed_index = 8;
		} else if (speed_index >= SPEED_INDEX_10_LOWER_BOUND && speed_index < SPEED_INDEX_10_UPPER_BOUND) {
			speed_index = 9;
		} else if (speed_index >= SPEED_INDEX_11_LOWER_BOUND && speed_index < SPEED_INDEX_11_UPPER_BOUND) {
			speed_index = 10;
		} else if (speed_index >= SPEED_INDEX_12_LOWER_BOUND && speed_index < SPEED_INDEX_12_UPPER_BOUND) {
			speed_index = 11;
		} else if (speed_index >= SPEED_INDEX_13_LOWER_BOUND && speed_index < SPEED_INDEX_13_UPPER_BOUND) {
			speed_index = 12;
		} else if (speed_index >= SPEED_INDEX_14_LOWER_BOUND && speed_index < SPEED_INDEX_14_UPPER_BOUND) {
			speed_index = 13;
		} else if (speed_index >= SPEED_INDEX_15_LOWER_BOUND && speed_index < SPEED_INDEX_15_UPPER_BOUND) {
			speed_index = 14;
		} else if (speed_index >= SPEED_INDEX_16_LOWER_BOUND && speed_index < SPEED_INDEX_16_UPPER_BOUND) {
			speed_index = 15;
		} else if (speed_index >= SPEED_INDEX_17_LOWER_BOUND && speed_index < SPEED_INDEX_17_UPPER_BOUND) {
			speed_index = 16;
		}
	}
}

void arbitrate_steering_angle(void) {
	// if steering angle is inoperative assign control_state to manual and do not update steering angle
	if (steering_angle.parameter_state == INOPERATIVE) {
		control_state = MANUAL;
	}

	// if control state is auto assign steering angle
	if (control_state == AUTO) {
		// 0 is 0 degrees steering angle (-90 to 90)

		if (steering_angle.buffer_average >= STEERING_INDEX_1_LOWER_BOUND && steering_angle.buffer_average < STEERING_INDEX_1_UPPER_BOUND) {
			steering_angle_index = 0;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_2_LOWER_BOUND && steering_angle.buffer_average < STEERING_INDEX_2_UPPER_BOUND) {
			steering_angle_index = 1;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_3_LOWER_BOUND && steering_angle.buffer_average < STEERING_INDEX_3_UPPER_BOUND) {
			steering_angle_index = 2;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_4_LOWER_BOUND && steering_angle.buffer_average < STEERING_INDEX_4_UPPER_BOUND) {
			steering_angle_index = 3;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_5_LOWER_BOUND && steering_angle.buffer_average < STEERING_INDEX_5_UPPER_BOUND) {
			steering_angle_index = 4;
		}

	}
}

void output_angles(void) {
	setPWM(htim3, TIM_CHANNEL_1, 60000, front_left_servo_ticks);	// 3000 == 1ms, 20ms == 60000ms
	setPWM(htim3, TIM_CHANNEL_2, 60000, front_right_servo_ticks);	// 3000 == 1ms, 20ms == 60000ms
	setPWM(htim3, TIM_CHANNEL_3, 60000, rear_servo_ticks);			// 3000 == 1ms, 20ms == 60000ms
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
uint16_t pulse)
{
	HAL_TIM_PWM_Stop(&timer, channel); 						// stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; 							// set the period duration
	HAL_TIM_PWM_Init(&timer); 								// reinititialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse; 								// set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); 					// start pwm generation
}

void get_btn_state(void) {
	if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) 	//read the onboard DRS button
	{
		drs_button_state = 1;
	} else {
		drs_button_state = 0;
	}
}
