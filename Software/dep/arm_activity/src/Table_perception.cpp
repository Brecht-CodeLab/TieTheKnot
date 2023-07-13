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
#include <vector.h>
#include <iostream.h> //Check with @Brecht!

// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/src/frames.hpp"
// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/src/chainiksolvervel_pinv_givens.hpp"
// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/src/chain.hpp"
// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/src/frames_io.hpp"
// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/src/jntarrayvel.hpp"
// #include "/home/arcs/Desktop/arcs2022/Projects/PalletStacking/Software/kdl_sample/external/orocos_kdl/models/models.hpp"

#include <chain.hpp>
#include <chainiksolvervel_pinv_givens.hpp>
#include <jntarray.hpp>                    //needed for jont quantity arrays
#include <jntarrayvel.hpp>                 //needed for input of fkvelsolver
#include <framevel.hpp>                    //needed for vectorvel and framevel types
#include <frames.hpp>                      //needed for vector
#include <frames_io.hpp>
#include <chainfksolver.hpp>               //needed for forward velocity solver
#include <chainfksolvervel_recursive.hpp>  //needed for forward velocity solver
#include <chainiksolvervel_pinv_nso.hpp> 
#include <chainiksolvervel_pinv.hpp> 
#include <models.hpp>                      //needed for iiwa chain




// five_c
#include <five_c/thread/thread.h>
#include <five_c/activity/activity.h>

#include <iiwa_activity.hpp>

#include <pthread.h>
#include <unistd.h>

// KDL::Chain iiwa_robot_kdl = KDL::KukaIIWA14();
// Vector const_cmd_vel = Vector(0.0,0.0,0.05);
// Vector cmd_rot = Vector(0.0,0.0,0.0);
// Twist cmd_tot_vel = Twist(const_cmd_vel, cmd_rot);
bool *deinitialisation_request;
double jnt_pos_save[7];
double jnt_ext_torque_save[7];
FILE *fpt;
double t = 0;
bool table_found = false;
// KDL::ChainIkSolverVel_pinv_givens ivelksolver(iiwa_robot_kdl);

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
	// double move_distance = 0.5; // m
	float force_increment[3][1]; // 
	KDL::Chain iiwa_robot_kdl = KDL::KukaIIWA14();
	KDL::Vector const_cmd_vel = KDL::Vector(-0.1,0.0,0);
	KDL::Vector cmd_rot = KDL::Vector(0.0,0.0,0.0);
	KDL::Twist cmd_tot_vel = KDL::Twist(const_cmd_vel, cmd_rot);
	// KDL::ChainIkSolverVel_pinv_givens ivelksolver(iiwa_robot_kdl);
	// KDL::ChainIkSolverVel_pinv_nso ivelksolver(iiwa_robot_kdl);
	KDL::ChainIkSolverVel_pinv ivelksolver(iiwa_robot_kdl);
	KDL::JntArray q(iiwa_robot_kdl.getNrOfJoints());
	KDL::JntArray dq(iiwa_robot_kdl.getNrOfJoints());
	double v_max = 0.5


	// cout << "Speed: " << dq(0) << "," << dq(1) << "," << dq(2) << "," << dq(3) << "," << dq(4) << "," << dq(5) << "," <<  dq(6) << endl;	

	while(!(*deinitialisation_request)){
		usleep(100*dt);  // time in microseconds
	cout << "Speed: " << dq(0) << "," << dq(1) << "," << dq(2) << "," << dq(3) << "," << dq(4) << "," << dq(5) << "," <<  dq(6) << endl;	

		if (table_found == false) {
			if (iiwa_activity->lcsm.state == RUNNING){
					// q(0) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[0];
					// q(1) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[1];
					// q(2) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[2];
					// q(3) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[3];
					// q(4) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[4];
					// q(5) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[5];
					// q(6) = continuous_state->iiwa_state.iiwa_sensors.meas_jnt_pos[6];
					// cout << "Position: " << q(0) << "," << q(1) << "," << q(2) << "," << q(3) << "," << q(4) << "," << q(5) << "," <<  q(6) << endl;	


					// cout << "before" << endl;
					// ivelksolver.CartToJnt(q, cmd_tot_vel, dq);
					// cout << "after" << endl;

					pthread_mutex_lock(&coord_state->actuation_lock);
					
					// params->iiwa_params.cmd_jnt_vel[0] = dq(0);
					// params->iiwa_params.cmd_jnt_vel[1] = dq(1);
					// params->iiwa_params.cmd_jnt_vel[2] = dq(2);
					// params->iiwa_params.cmd_jnt_vel[3] = dq(3);
					// params->iiwa_params.cmd_jnt_vel[4] = dq(4);
					// params->iiwa_params.cmd_jnt_vel[5] = dq(5);
					// params->iiwa_params.cmd_jnt_vel[6] = dq(6);

					// cout << "Speed: " << dq(0) << "," << dq(1) << "," << dq(2) << "," << dq(3) << "," << dq(4) << "," << dq(5) << "," <<  dq(6) << endl;	
	
					params->iiwa_params.cmd_wrench[0] = params->iiwa_params.cmd_wrench[0] + force_increment[0][0];
					params->iiwa_params.cmd_wrench[1] = params->iiwa_params.cmd_wrench[1] + force_increment[1][0];
					params->iiwa_params.cmd_wrench[2] = params->iiwa_params.cmd_wrench[2] + force_increment[2][0];

					pthread_mutex_unlock(&coord_state->actuation_lock);

					// printf("%f \t",params->iiwa_params.cmd_wrench[2]);
					}
		// else {
		// 			params->iiwa_params.cmd_jnt_vel[0] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[1] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[2] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[3] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[4] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[5] = 0;
		// 			params->iiwa_params.cmd_jnt_vel[6] = 0;
		// 		}
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

	vector<double> jnt_ext_torque_saves[7]; // stores values to calculate s.d. at each iterate
	<double> standard_deviation[7]; // stores standard_deviation
	<double> average[7]; // stores average
	vector<double> averages; // stores list with i times the value of the average
	<double> sum;
	vector<double> sum_of_deviations[7];
	vector<double> deviations; 

	// double threshold = 2;
	int i = 0;

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

			
			i = i+1;
			if (table_found != true) {
				for (int j = 0; j < 7; j++) {
					
					// Start with 30 values before checking outliers of standard deviation 
					if (i > 30) {
						
						// threshold is mean + 6 standard deviations --> 99.9996% within
						if (jnt_ext_torque_save[j] > average[j] + 6*standard_deviation[j] || jnt_ext_torque_save[j] < average[j] - 6*standard_deviation[j] ) {
							table_found = true;
						}
					}
					else {

						// Add new external torque value to the end and delete first element if vector is full
						if (size(jnt_ext_torque_saves[j]) >= 100) {
							jnt_ext_torque_saves[j].erase(jnt_ext_torque_saves[j].begin()); // not very efficient but ok
						}
						jnt_ext_torque_saves[j].push_back(jnt_ext_torque_save[j]);
						
						
						// Calculate average
						sum = accumulate(jnt_ext_torque_saves[j].begin(), jnt_ext_torque_saves[j].end(), 0.0);

						if (i > 29) {
							average[j] =  sum/i;
							averages(i, average[j]);

							// Calculate standard deviation
							deviations = (jnt_ext_torque_saves[j] - averages)^2;
							sum_of_deviations = accumulate(deviations.begin(), deviations.end(), 0.0);

							if (i >= 100) {
								standard_deviation[j] = sqrt(sum_of_deviations/100);
							}
							else {
								standard_deviation[j] = sqrt(sum_of_deviations/i);
							}
						}
					}
				}
			} 

			jnt_ext_torque_save[0]
			jtn_e
			// if (abs(jnt_ext_torque_save[0]) > threshold || abs(jnt_ext_torque_save[1]) > threshold) {
			// if (abs(jnt_ext_torque_save[5]) > threshold ) {
			// 	table_found = true;
			// }
			// cout << "Torque ext: " << jnt_ext_torque_save[0] <<endl;



		}
	}
}

int main(int argc, char**argv){
	signal(SIGINT, sigint_handler);
	
	// ### ACTIVITIES ### //   
	activity_t iiwa_activity;    
	// iiwa 
	ec_iiwa_activity.create_lcsm(&iiwa_activity, "iiwa_activity");   
	ec_iiwa_activity.resource_configure_lcsm(&iiwa_activity);

	// Share memory
	iiwa_activity_coordination_state_t *iiwa_activity_coord_state =
			(iiwa_activity_coordination_state_t *) iiwa_activity.state.coordination_state;

	deinitialisation_request = &iiwa_activity_coord_state->deinitialisation_request;

	iiwa_activity_params_t* params = (iiwa_activity_params_t *) iiwa_activity.conf.params;

	strcpy(params->iiwa_params.fri_ip,"192.168.1.50");
	params->iiwa_params.fri_port = 30100;
	params->iiwa_params.cmd_mode = POSITION;
	// params->iiwa_params.cmd_mode = WRENCH;
	

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
