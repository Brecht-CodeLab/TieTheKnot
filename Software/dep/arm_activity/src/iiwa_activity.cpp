/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_activity.c
 * @date September 15, 2022
 **/

#include "string.h"
#include <time.h>

#include "iiwa_activity.hpp"
#include <iostream>

// FILE *fpt;
/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the hokuyo activity
*/
void iiwa_activity_config(activity_t *activity){
	// Remove config() from the eventloop schedule in the next iteration
	remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
	// Deciding which schedule to add
	switch (activity->lcsm.state){
		case CREATION:
			printf("In creation state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "creation");
			break;
		case RESOURCE_CONFIGURATION:
			printf("In resource configuration state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
			break;
		case CAPABILITY_CONFIGURATION:
			printf("In capability configuration state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "capability_configuration");
            break;
		case PAUSING:
			printf("In pausing state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "pausing");
			break;
		case RUNNING:
			printf("In running state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "running");
			break;
		case CLEANING:
			printf("In cleaning state \n");
			add_schedule_to_eventloop(&activity->schedule_table, "cleaning");
			break;
		case DONE:
			break;
	}
};

// Creation
void iiwa_activity_creation_coordinate(activity_t *activity){
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		activity->lcsm.state = RESOURCE_CONFIGURATION;
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_activity_creation_configure(activity_t *activity){
	if (activity->lcsm.state != CREATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "creation");
	}
}

// Allocating memory here
void iiwa_activity_creation_compute(activity_t *activity){
	iiwa_activity_continuous_state_t *continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_activity_discrete_state_t *discrete_state = (iiwa_activity_discrete_state_t *) activity->state.computational_state.discrete;
	iiwa_init(&continuous_state->iiwa_state, &discrete_state->iiwa_discrete_state);
	activity->state.lcsm_flags.creation_complete = true;
}

void iiwa_activity_creation(activity_t *activity){
	iiwa_activity_creation_compute(activity);
	iiwa_activity_creation_coordinate(activity);
	iiwa_activity_creation_configure(activity);
}

// Cleaning
void iiwa_activity_cleaning_coordinate(activity_t *activity){
    iiwa_activity_coordination_state_t * coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating own activity
    if (activity->state.lcsm_flags.deletion_complete)
        activity->lcsm.state = DONE;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_activity_cleaning_configure(activity_t *activity){
    if (activity->lcsm.state != CLEANING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "cleaning");
    }
}

void iiwa_activity_cleaning_compute(activity_t *activity){
    iiwa_activity_continuous_state_t* continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
    iiwa_state_cleanup(&continuous_state->iiwa_state);
    activity->state.lcsm_flags.deletion_complete = true;
}

void iiwa_activity_cleaning(activity_t *activity){
    iiwa_activity_cleaning_compute(activity);
    iiwa_activity_cleaning_coordinate(activity);
    iiwa_activity_cleaning_configure(activity);
}

// Resource configuration
void iiwa_activity_resource_configuration_coordinate(activity_t *activity){
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	
	// Coordinating own activity
	if (activity->state.lcsm_flags.creation_complete)
		switch (activity->state.lcsm_protocol){ 
			case INITIALISATION:
				activity->lcsm.state = CAPABILITY_CONFIGURATION;
				break;
			case EXECUTION:
				activity->lcsm.state = CAPABILITY_CONFIGURATION;
				break;
			case DEINITIALISATION:
				activity->lcsm.state = CLEANING;
				break;
		}
		update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_activity_resource_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != RESOURCE_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
		// Update flags for next visit to the resource configuration LCS 
		activity->state.lcsm_flags.resource_configuration_complete = false;
	}
}

void iiwa_activity_resource_configuration_compute(activity_t *activity){
	// Connect to iiwa robot
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) activity->conf.params;
	iiwa_activity_continuous_state_t* continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_activity_discrete_state_t *discrete_state = (iiwa_activity_discrete_state_t *) activity->state.computational_state.discrete;
	switch (activity->state.lcsm_protocol){
		case INITIALISATION:
			if (iiwa_connect(&params->iiwa_params, &continuous_state->iiwa_state)){
				printf("[iiwa activity] Succesfully connected iiwa robot\n");
				activity->state.lcsm_flags.resource_configuration_complete = true;
			}
			break;
		case DEINITIALISATION:
			iiwa_disconnect(&continuous_state->iiwa_state);
			activity->state.lcsm_flags.resource_configuration_complete = true;
			break;
	}
}

void iiwa_activity_resource_configuration(activity_t *activity){
	iiwa_activity_resource_configuration_compute(activity);
	iiwa_activity_resource_configuration_coordinate(activity);
	iiwa_activity_resource_configuration_configure(activity);
}

// capability configuration
void iiwa_activity_capability_configuration_coordinate(activity_t *activity){
	iiwa_activity_coordination_state_t * coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	// if (coord_state->execution_request)
	// 	activity->state.lcsm_protocol = EXECUTION;
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;

	if (activity->state.lcsm_flags.capability_configuration_complete){
		switch (activity->state.lcsm_protocol){ 
			case INITIALISATION:
				activity->lcsm.state = PAUSING;
				break;
			case EXECUTION:
				activity->lcsm.state = RUNNING;
				break;
			case DEINITIALISATION:
				activity->lcsm.state = RESOURCE_CONFIGURATION;
				break;
		}
		update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
	}
}

void iiwa_activity_capability_configuration_configure(activity_t *activity){
	if (activity->lcsm.state != CAPABILITY_CONFIGURATION){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "capability_configuration");
		// Update flags for next visit to the capability configuration LCS 
		activity->state.lcsm_flags.capability_configuration_complete = false;
	}
}

void iiwa_activity_capability_configuration_compute(activity_t *activity){
	// Connect to iiwa robot
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) activity->conf.params;
	iiwa_activity_continuous_state_t* continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;

	switch (activity->state.lcsm_protocol){
		case INITIALISATION:
			if (iiwa_check_commanding_mode(&continuous_state->iiwa_state, params->iiwa_params.cmd_mode)){
				activity->state.lcsm_flags.capability_configuration_complete = true;
			}
			else if (continuous_state->iiwa_state.client->commanding_mode != NO_COMMAND_MODE){
				printf("[ERROR] Commanding mode different, deinitializing \n");
				activity->state.lcsm_protocol = DEINITIALISATION;
			}
			break;
		case DEINITIALISATION:
			activity->state.lcsm_flags.capability_configuration_complete = true;
			break;
	}
}

void iiwa_activity_capability_configuration(activity_t *activity){
	iiwa_activity_capability_configuration_compute(activity);
	iiwa_activity_capability_configuration_coordinate(activity);
	iiwa_activity_capability_configuration_configure(activity);
}

// Pausing
void iiwa_activity_pausing_compute(activity_t *activity){
	iiwa_activity_continuous_state_t *continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_step(&continuous_state->iiwa_state);
}

void iiwa_activity_pausing_coordinate(activity_t *activity){
	iiwa_activity_coordination_state_t * coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	iiwa_activity_continuous_state_t *continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	// Coordinating with other activities
	if (coord_state->execution_request)
		activity->state.lcsm_protocol = EXECUTION;
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	// Coordinating own activity
	switch (activity->state.lcsm_protocol){ 
		case EXECUTION:
			if (continuous_state->iiwa_state.client->current_session_state == COMMANDING_ACTIVE){
				activity->lcsm.state = RUNNING;
			}
			break;
		case DEINITIALISATION:
			activity->lcsm.state = CAPABILITY_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_activity_pausing_configure(activity_t *activity){
	if (activity->lcsm.state != PAUSING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
	}
}

void iiwa_activity_pausing(activity_t *activity){
	iiwa_activity_pausing_compute(activity);
	iiwa_activity_pausing_coordinate(activity);
	iiwa_activity_pausing_configure(activity);
}

// Running
void iiwa_activity_running_communicate(activity_t *activity){
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) activity->conf.params;
	iiwa_activity_continuous_state_t *continuous_state = (iiwa_activity_continuous_state_t *) activity->state.computational_state.continuous;
	iiwa_activity_discrete_state_t *discrete_state = (iiwa_activity_discrete_state_t *) activity->state.computational_state.discrete;
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	iiwa_state_t *iiwa_state = (iiwa_state_t *) &continuous_state->iiwa_state;

	if ((continuous_state->iiwa_state.client->current_session_state == MONITORING_READY) || 
										(continuous_state->iiwa_state.client->current_session_state == IDLE))
	{
		coord_state->commanding_not_active = true;   
		return;
	}

	pthread_mutex_lock(&coord_state->actuation_lock);
	switch (discrete_state->iiwa_discrete_state.iiwa_commanding_mode)
	{
		case POSITION:
			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
			{
				iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i] = params->iiwa_params.cmd_jnt_vel[i];
			}
			break;
		
		case TORQUE:
			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
			{
				iiwa_state->iiwa_actuation_input.cmd_torques[i] = params->iiwa_params.cmd_torques[i];
				iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i] = params->iiwa_params.cmd_jnt_vel[i];
			}
			break;

		case WRENCH:
			for (unsigned int i=0;i<CART_VECTOR_DIM;i++)
			{
				iiwa_state->iiwa_actuation_input.cmd_wrench[i] = params->iiwa_params.cmd_wrench[i];
			}
			for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
			{
				iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i] = params->iiwa_params.cmd_jnt_vel[i];
			}
			break;
	}
	pthread_mutex_unlock(&coord_state->actuation_lock);
	iiwa_step(&continuous_state->iiwa_state);
	
	iiwa_communicate(&continuous_state->iiwa_state);
	
	pthread_mutex_lock(&coord_state->sensor_lock);
	for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++){
		iiwa_state->iiwa_sensors.meas_torques[i] = iiwa_state->client->meas_torques[i];
		iiwa_state->iiwa_sensors.meas_ext_torques[i] = iiwa_state->client->meas_ext_torques[i];
		iiwa_state->iiwa_sensors.meas_jnt_pos[i] = iiwa_state->client->meas_jnt_pos[i];
	}
	discrete_state->iiwa_discrete_state.iiwa_commanding_mode = iiwa_state->client->commanding_mode;
	discrete_state->iiwa_discrete_state.iiwa_current_session_state = iiwa_state->client->current_session_state;
	discrete_state->iiwa_discrete_state.iiwa_connection_quality = iiwa_state->client->connection_quality;
	pthread_mutex_unlock(&coord_state->sensor_lock);

}

void iiwa_activity_running_coordinate(activity_t *activity){
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	// Coordinating with other activities
	if (coord_state->deinitialisation_request)
		activity->state.lcsm_protocol = DEINITIALISATION;
	
	// Go to pausing when not commanding active and wait there until active
	if (coord_state->commanding_not_active)
		activity->lcsm.state = PAUSING;

	switch (activity->state.lcsm_protocol){ 
		case DEINITIALISATION:
			activity->lcsm.state = RESOURCE_CONFIGURATION;
			break;
	}
	update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void iiwa_activity_running_configure(activity_t *activity){
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) activity->state.coordination_state;
	if (activity->lcsm.state != RUNNING){
		// Update schedule
		add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
		remove_schedule_from_eventloop(&activity->schedule_table, "running");
		// Update flags
		coord_state->commanding_not_active = false;
	}
}

void iiwa_activity_running(activity_t *activity){
	iiwa_activity_running_communicate(activity);
	iiwa_activity_running_coordinate(activity);
	iiwa_activity_running_configure(activity);
}

// SCHEDULER 
void iiwa_activity_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) iiwa_activity_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");
    
    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) iiwa_activity_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_config = {.number_of_functions = 0};
    register_function(&schedule_resource_config, (function_ptr_t) iiwa_activity_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_config, "resource_configuration");

	schedule_t schedule_capability_config = {.number_of_functions = 0};
    register_function(&schedule_capability_config, (function_ptr_t) iiwa_activity_capability_configuration, 
        activity, "capability_configuration");
    register_schedule(&activity->schedule_table, schedule_capability_config, "capability_configuration");
    
    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) iiwa_activity_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) iiwa_activity_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, "running");

	schedule_t schedule_cleaning = {.number_of_functions = 0};
    register_function(&schedule_cleaning, (function_ptr_t) iiwa_activity_cleaning, 
        activity, "cleaning");
    register_schedule(&activity->schedule_table, schedule_cleaning, 
        "cleaning");
}

void iiwa_activity_create_lcsm(activity_t* activity, const char* name_activity){
    activity->conf.params = malloc(sizeof(iiwa_activity_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(iiwa_activity_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(iiwa_activity_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(iiwa_activity_coordination_state_t));
}

void iiwa_activity_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    iiwa_activity_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void iiwa_activity_destroy_lcsm(activity_t* activity){
    destroy_activity(activity);
}

const iiwa_activity_t ec_iiwa_activity ={
    .create_lcsm = iiwa_activity_create_lcsm,
    .resource_configure_lcsm = iiwa_activity_resource_configure_lcsm,
    .destroy_lcsm = iiwa_activity_destroy_lcsm,
};