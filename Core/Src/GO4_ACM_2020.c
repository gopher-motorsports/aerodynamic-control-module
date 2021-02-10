/*
 * GO4_ACM_2020.c
 *
 *  Created on: Sep 3, 2020
 *      Author: bents
 */

/* TODO:
 * 1. Acceleration sign
 * 2. CAN
 * 3. Time based Wake-Up
 */

//SYSTEM INCLUDES
#include "cmsis_os.h"
#include "main.h"
#include <math.h>

// Project Includes
#include "base_types.h"
#include "GO4_ACM_2020.h"
#include "GopherCAN.h"

extern TIM_HandleTypeDef htim3;

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

ACM_parameter parameters[NUM_PARAMETERS];			// signed parameters that are read over CAN
U8 speed_index;						// speed variable used in the big map (0 is 0 mph)
U8 steering_angle_index;			// steering angle variable used in the big map (127 is 0 degrees)
U8 acceleration_index;				// acceleration variable used in the big map (127 is 0 g)
U16 front_left_servo_ticks;
U16 front_right_servo_ticks;
U16 rear_servo_ticks;
U8 drs_button_state;

U32 last_wheel_speed_req = 0;
U32 last_air_speed_req = 0;
U32 last_throttle_position_req = 0;
U32 last_steering_angle_req = 0;
U32 last_brake_pressure_req = 0;
U32 last_acceleration_req = 0;

ACM_CONTROL_STATE control_state;	// the state of control (Either AUTO - controlled by the ACM or MANUAL - Driver controlled)
U8 this_module = ACM_ID;

/* CAN */

extern CAN_COMMAND_STRUCT can_command;
extern S16_CAN_STRUCT received_wheel_speed;
extern S16_CAN_STRUCT received_air_speed;
extern S16_CAN_STRUCT received_throttle_position;
extern S16_CAN_STRUCT received_steering_angle;
extern S16_CAN_STRUCT received_brake_pressure;
extern S16_CAN_STRUCT received_acceleration;

// the HAL_CAN struct
CAN_HandleTypeDef* hcan;

void init(CAN_HandleTypeDef* hcan_ptr)
{
	hcan = hcan_ptr;

	// initialize CAN
	// NOTE: CAN will also need to be added in CubeMX and code must be generated
	// Check the STM_CAN repo for the file "F0xx CAN Config Settings.pptx" for the correct settings
	if (init_can(hcan, this_module))
	{
		// an error has occurred, just... no
		while(1);
	}

	// enable updating the RPM and fan_current. Parameters that are not added to this list
	// will not be updated over CAN, even if they are requested
	received_wheel_speed.update_enabled = TRUE;
	received_air_speed.update_enabled = TRUE;

	// enable the other variables
	received_throttle_position.update_enabled = TRUE;
	received_steering_angle.update_enabled = TRUE;
	received_brake_pressure.update_enabled = TRUE;
	received_acceleration.update_enabled = TRUE;
}

void ACM_Init(void) {

	// all unsigned parameters
	parameters[0] = wheel_speed;
	parameters[1] = air_speed;
	parameters[2] = throttle_position;
	parameters[3] = brake_pressure;
	parameters[4] = acceleration;
	parameters[5] = steering_angle;

	// Do the Wave
}

void fetch_data(void) {

	/******************** REQUEST WHEEL_SPEED ********************/
	if((HAL_GetTick() - received_wheel_speed.last_rx) >= RECEIVED_WHEEL_SPEED_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_wheel_speed.pending_response == FALSE ||
			HAL_GetTick() - last_wheel_speed_req >= RECEIVED_WHEEL_SPEED_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_wheel_speed_req = received_wheel_speed.last_rx;
	}

	/******************** REQUEST AIR_SPEED ********************/
	if((HAL_GetTick() - received_air_speed.last_rx) >= RECEIVED_AIR_SPEED_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_air_speed.pending_response == FALSE ||
			HAL_GetTick() - last_air_speed_req >= RECEIVED_AIR_SPEED_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_air_speed_req = received_air_speed.last_rx;
	}

	/******************** REQUEST THROTTLE_POSITION ********************/
	if((HAL_GetTick() - received_throttle_position.last_rx) >= RECEIVED_THROTTLE_POSITION_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_throttle_position.pending_response == FALSE ||
			HAL_GetTick() - last_throttle_position_req >= RECEIVED_THROTTLE_POSITION_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_throttle_position_req = received_throttle_position.last_rx;
	}

	/******************** REQUEST BRAKE_PRESSURE ********************/
	if((HAL_GetTick() - received_brake_pressure.last_rx) >= RECEIVED_BRAKE_PRESSURE_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_brake_pressure.pending_response == FALSE ||
			HAL_GetTick() - last_brake_pressure_req >= RECEIVED_BRAKE_PRESSURE_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_brake_pressure_req = received_brake_pressure.last_rx;
	}

	/******************** REQUEST STEERING_ANGLE ********************/
	if((HAL_GetTick() - received_steering_angle.last_rx) >= RECEIVED_STEERING_ANGLE_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_brake_pressure.pending_response == FALSE ||
			HAL_GetTick() - last_steering_angle_req >= RECEIVED_STEERING_ANGLE_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_steering_angle_req = received_steering_angle.last_rx;
	}

	/******************** REQUEST STEERING_ANGLE ********************/
	if((HAL_GetTick() - received_acceleration.last_rx) >= RECEIVED_ACCELERATION_UPDATE_TIME)
	{
		// if a request has already been sent don't send another one.
		if(received_acceleration.pending_response == FALSE ||
			HAL_GetTick() - last_acceleration_req >= RECEIVED_ACCELERATION_TIMEOUT_TIME)
		{
			// ERROR
		}

		// update the last time that a message was received
		last_steering_angle_req = received_acceleration.last_rx;
	}

}

void update_data(void) {
	ACM_parameter* global_parameter_list;					// List of all of the unsigned parameters
	U8 average_index;										// index of the next element added to the sum to find average
	S16 average_aggregate;									// sum of the elements before the average_index

	// Put all new received CAN data into parameters.
	// Absolutely awful, find a better way to do this.
	global_parameter_list = parameters;
	global_parameter_list->current_value = received_wheel_speed.data;
	global_parameter_list++;
	global_parameter_list->current_value = received_air_speed.data;
	global_parameter_list++;
	global_parameter_list->current_value = received_throttle_position.data;
	global_parameter_list++;
	global_parameter_list->current_value = received_steering_angle.data;
	global_parameter_list++;
	global_parameter_list->current_value = received_brake_pressure.data;
	global_parameter_list++;
	global_parameter_list->current_value = received_acceleration.data;

	//***************************  update data  ***************************//
	for(global_parameter_list = parameters;global_parameter_list < global_parameter_list + NUM_PARAMETERS;global_parameter_list++) {

		// Skip this parameter if it is INOPERATIVE (too many errors)
		if(global_parameter_list->parameter_state != OPERATIVE) {
			continue;
		}

		if ((global_parameter_list->current_value <= global_parameter_list->upper_bound) ||
				(global_parameter_list->current_value >= global_parameter_list->lower_bound)) {

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

	} else if(control_state == MANUAL && drs_button_state == 0) {

		//Manual Mode
		front_left_wing_map_position = FRONT_LEFT_WING_MANUAL_POSITION;
		front_right_wing_map_position = FRONT_RIGHT_WING_MANUAL_POSITION;
		rear_wing_map_position = REAR_WING_MANUAL_POSITION;

	} else {
		// DRS ACTIVATED
		front_left_wing_map_position = FRONT_LEFT_WING_DRS_POSITION;
		front_right_wing_map_position = FRONT_RIGHT_WING_DRS_POSITION;
		rear_wing_map_position = REAR_WING_DRS_POSITION;
	}

	// start to converting angle to timer count
	temp_front_left_servo_ticks = LINEAR_POSITION_TO_TICKS * (front_left_wing_map_position + FRONT_RIGHT_TRIM);
	temp_front_right_servo_ticks = LINEAR_POSITION_TO_TICKS * (front_right_wing_map_position + FRONT_LEFT_TRIM);
	temp_rear_servo_ticks = LINEAR_POSITION_TO_TICKS * (rear_wing_map_position + REAR_TRIM);

	// Finish converting angle to timer count
	front_left_servo_ticks = (U16)temp_front_left_servo_ticks + 1700;
	front_right_servo_ticks = (U16)temp_front_right_servo_ticks + 1700;
	rear_servo_ticks = (U16)temp_rear_servo_ticks + 1700;
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
			if (average_acceleration >= ACCELERATION_INDEX_17_THRESHOLD) {
				acceleration_index = 17;
			} else if (average_acceleration >= ACCELERATION_INDEX_16_THRESHOLD) {
				acceleration_index = 16;
			} else if (average_acceleration >= ACCELERATION_INDEX_15_THRESHOLD) {
				acceleration_index = 15;
			} else if (average_acceleration >= ACCELERATION_INDEX_14_THRESHOLD) {
				acceleration_index = 14;
			} else if (average_acceleration >= ACCELERATION_INDEX_13_THRESHOLD) {
				acceleration_index = 13;
			} else if (average_acceleration >= ACCELERATION_INDEX_12_THRESHOLD) {
				acceleration_index = 12;
			} else if (average_acceleration >= ACCELERATION_INDEX_11_THRESHOLD) {
				acceleration_index = 11;
			} else if (average_acceleration >= ACCELERATION_INDEX_10_THRESHOLD) {
				acceleration_index = 10;
			} else if (average_acceleration >= ACCELERATION_INDEX_9_THRESHOLD) {
				acceleration_index = 9;
			} else if (average_acceleration >= ACCELERATION_INDEX_8_THRESHOLD) {
				acceleration_index = 8;
			} else if (average_acceleration >= ACCELERATION_INDEX_7_THRESHOLD) {
				acceleration_index = 7;
			} else if (average_acceleration >= ACCELERATION_INDEX_6_THRESHOLD) {
				acceleration_index = 6;
			} else if (average_acceleration >= ACCELERATION_INDEX_5_THRESHOLD) {
				acceleration_index = 5;
			} else if (average_acceleration >= ACCELERATION_INDEX_4_THRESHOLD) {
				acceleration_index = 4;
			} else if (average_acceleration >= ACCELERATION_INDEX_3_THRESHOLD) {
				acceleration_index = 3;
			} else if (average_acceleration >= ACCELERATION_INDEX_2_THRESHOLD) {
				acceleration_index = 2;
			} else if (average_acceleration >= ACCELERATION_INDEX_1_THRESHOLD) {
				acceleration_index = 1;
			} else {
				acceleration_index = 0;
			}
		} else {
			// otherwise just use this equation to map the acceleration parameter
			average_acceleration = (THROTTLE_COEFFICIENT * throttle_position.buffer_average) - (BRAKE_PRESSURE_COEFFICIENT * brake_pressure.buffer_average);
			acceleration_index = (S16)average_acceleration;
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
			speed_index = (S16)wheel_speed.buffer_average;
		} else if (wheel_speed.parameter_state == INOPERATIVE) {
			speed_index = (S16)air_speed.buffer_average;
		} else {
			speed_index = (S16)wheel_speed.buffer_average;	// use wheelspeed as "default"
		}

		// ASSIGN SPEED_INDEX
		if (speed_index >= SPEED_INDEX_17_THRESHOLD) {
			speed_index = 17;
		} else if (speed_index >= SPEED_INDEX_16_THRESHOLD) {
			speed_index = 16;
		} else if (speed_index >= SPEED_INDEX_15_THRESHOLD) {
			speed_index = 15;
		} else if (speed_index >= SPEED_INDEX_14_THRESHOLD) {
			speed_index = 14;
		} else if (speed_index >= SPEED_INDEX_13_THRESHOLD) {
			speed_index = 13;
		} else if (speed_index >= SPEED_INDEX_12_THRESHOLD) {
			speed_index = 12;
		} else if (speed_index >= SPEED_INDEX_11_THRESHOLD) {
			speed_index = 11;
		} else if (speed_index >= SPEED_INDEX_10_THRESHOLD) {
			speed_index = 10;
		} else if (speed_index >= SPEED_INDEX_9_THRESHOLD) {
			speed_index = 9;
		} else if (speed_index >= SPEED_INDEX_8_THRESHOLD) {
			speed_index = 8;
		} else if (speed_index >= SPEED_INDEX_7_THRESHOLD) {
			speed_index = 7;
		} else if (speed_index >= SPEED_INDEX_6_THRESHOLD) {
			speed_index = 6;
		} else if (speed_index >= SPEED_INDEX_5_THRESHOLD) {
			speed_index = 5;
		} else if (speed_index >= SPEED_INDEX_4_THRESHOLD) {
			speed_index = 4;
		} else if (speed_index >= SPEED_INDEX_3_THRESHOLD) {
			speed_index = 3;
		} else if (speed_index >= SPEED_INDEX_2_THRESHOLD) {
			speed_index = 2;
		} else if (speed_index >= SPEED_INDEX_1_THRESHOLD) {
			speed_index = 1;
		} else {
			speed_index = 0;
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

		if (steering_angle.buffer_average >= STEERING_INDEX_5_THRESHOLD) {
			steering_angle_index = 4;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_4_THRESHOLD) {
			steering_angle_index = 3;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_3_THRESHOLD) {
			steering_angle_index = 2;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_2_THRESHOLD) {
			steering_angle_index = 1;
		} else if (steering_angle.buffer_average >= STEERING_INDEX_1_THRESHOLD) {
			steering_angle_index = 0;
		} else {
			steering_angle_index = 0;
		}
	}
}

void output_angles(void) {
	setPWM(htim3, TIM_CHANNEL_1, TIMER_PERIOD, front_left_servo_ticks);		// 3000 == 1ms, 20ms == 60000ms
	setPWM(htim3, TIM_CHANNEL_2, TIMER_PERIOD, front_right_servo_ticks);	// 3000 == 1ms, 20ms == 60000ms
	setPWM(htim3, TIM_CHANNEL_3, TIMER_PERIOD, rear_servo_ticks);			// 3000 == 1ms, 20ms == 60000ms
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
