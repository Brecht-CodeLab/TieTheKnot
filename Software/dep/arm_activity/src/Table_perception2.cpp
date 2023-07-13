/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
	* @file iiwa_demo.cpp
	* @date September 16, 2022
 **/

#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <math.h>

// five_c
#include <five_c/thread/thread.h>
#include <five_c/activity/activity.h>

#include <iiwa_activity.hpp>

#include <pthread.h>
#include <unistd.h>


bool *deinitialisation_request;
double jnt_pos_save[7];
double jnt_ext_torque_save[7];
FILE *fpt;
double t = 0;
bool table_found = false;

static void sigint_handler(int sig){
	if (deinitialisation_request==NULL){
		printf("Ops.. deinitalisation request is a NULL pointer.\n");
	}else{
		printf("\nDeinitialising iiwa activity\n");
		*deinitialisation_request = true;
	}
}

void* set_actuation(void* activity){
	activity_t *iiwa_activity = (activity_t*) activity; 
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity->conf.params;
	iiwa_activity_continuous_state_t *continuous_state =
	(iiwa_activity_continuous_state_t *) iiwa_activity->state.computational_state.continuous;
	iiwa_activity_coordination_state_t *coord_state =
	(iiwa_activity_coordination_state_t *) iiwa_activity->state.coordination_state;  

	int dt = 10; // ms
	double move_distance = 0.1; // m

	while(!(*deinitialisation_request)){
		usleep(1000*dt);  // time in microseconds
		if (iiwa_activity->lcsm.state == RUNNING){
			// Copying data
			if (table_found == false) {
				pthread_mutex_lock(&coord_state->actuation_lock);
				params->iiwa_params.cmd_wrench[0] = params->iiwa_params.cmd_wrench[0] - move_distance;

				// params->iiwa_params.cmd_wrench[1] = params->iiwa_params.cmd_wrench[1] - move_distance;
				pthread_mutex_unlock(&coord_state->actuation_lock);

				// printf("%f \t",params->iiwa_params.cmd_wrench[2]);
			}
			else {
				printf("table found \n");
			}
			t += (double)dt/1000;
			// printf("%f\n", t);
		}
	}
}

void* save_sensor_data(void* activity){
	activity_t *iiwa_activity = (activity_t*) activity; 
	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity->conf.params;
	iiwa_activity_continuous_state_t *continuous_state =
	(iiwa_activity_continuous_state_t *) iiwa_activity->state.computational_state.continuous;
	iiwa_activity_coordination_state_t *coord_state =
	(iiwa_activity_coordination_state_t *) iiwa_activity->state.coordination_state;  

	double threshold = 8;

	while(!(*deinitialisation_request)){
		if (iiwa_activity->lcsm.state == RUNNING){
			usleep(10*1000);
			pthread_mutex_lock(&coord_state->sensor_lock);
			memcpy(jnt_pos_save, continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			memcpy(jnt_ext_torque_save, continuous_state->iiwa_state.iiwa_sensors.meas_ext_torques, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			pthread_mutex_unlock(&coord_state->sensor_lock);
			// Saving to a file
			fpt = fopen("jnt_pos.csv", "a+");
			fprintf(fpt, "%f, %f, %f, %f, %f, %f, %f \n", jnt_pos_save[0], jnt_pos_save[1], jnt_pos_save[2],
															jnt_pos_save[3], jnt_pos_save[4], jnt_pos_save[5], jnt_pos_save[6]);
			fclose(fpt);
			// printf("%f, %f \n", jnt_ext_torque_save[0], jnt_ext_torque_save[1]);
			if (abs(jnt_ext_torque_save[0]) > threshold || abs(jnt_ext_torque_save[1]) > threshold) {
				table_found = true;
			}
		}
	}
}

int main(int argc, char**argv){
	signal(SIGINT, sigint_handler);
	
	// ### ACTIVITIES ### //   
	activity_t iiwa_activity;    
	// iiwa Lidar
	ec_iiwa_activity.create_lcsm(&iiwa_activity, "iiwa_activity");   
	ec_iiwa_activity.resource_configure_lcsm(&iiwa_activity);

	// Share memory
	iiwa_activity_coordination_state_t *iiwa_activity_coord_state =
			(iiwa_activity_coordination_state_t *) iiwa_activity.state.coordination_state;

	deinitialisation_request = &iiwa_activity_coord_state->deinitialisation_request;

	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity.conf.params;

	strcpy(params->iiwa_params.fri_ip,"192.168.1.50");
	params->iiwa_params.fri_port = 30100;
	params->iiwa_params.cmd_mode = WRENCH;

	// Manually 
	iiwa_activity_coord_state->execution_request = true;

	// ### THREADS ### //
	thread_t thread_iiwa;

	// Create thread: data structure, thread name, cycle time in milliseconds

	create_thread(&thread_iiwa, "thread_iiwa", 4); // 4 ms This should be lower than sunrise setup

	// Register activities in threads
	register_activity(&thread_iiwa, &iiwa_activity, "iiwa_activity");

	// Create POSIX threads   
	pthread_t pthread_iiwa, pthread_actuation, phtread_saving;

	pthread_create( &pthread_iiwa, NULL, do_thread_loop, ((void*) &thread_iiwa)); // Thread with iiwa activity register
	pthread_create( &pthread_actuation, NULL, set_actuation, (void*) &iiwa_activity); // Thread with a function registered
	pthread_create( &phtread_saving, NULL, save_sensor_data, (void*) &iiwa_activity); // 

	// Wait for threads to finish, which means all activities must properly finish and reach the dead LCSM state
	pthread_join(pthread_iiwa, NULL);
	pthread_join(pthread_actuation, NULL);
	pthread_join(phtread_saving, NULL);
	
	// Freeing memory
	// ec_iiwa_activity.destroy_lcsm(&iiwa_activity);
	return 0;
}
