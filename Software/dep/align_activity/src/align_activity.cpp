/* ----------------------------------------------------------------------------
 * Tie the Knot,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Olivier Deforche, Brecht Vandekerkhove, Timon Huysse
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file align_activity.cpp
 * @date January 18, 2023
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

// #include <pthread.h>
#include <unistd.h>

// perception hpp
// #include <vision_activity.hpp>

// Orocos KDL
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/src/frames.hpp>
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/src/chain.hpp>
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/src/chainiksolvervel_pinv_givens.hpp>
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/models/models.hpp>
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/src/frames_io.hpp>
#include </home/olivierdeforche/arcs2022/Projects/TieTheKnot/Software/dep/align_activity/external/orocos_kdl/src/jntarrayvel.hpp>

// iiwa controller
// #include "iiwa_controller.hpp"

KDL::Chain iiwa_robot_kdl = KDL::KukaIIWA14();
KDL::ChainIkSolverVel_pInv_givens iksolver(iiwa_robot_kdl);

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
	iiwa_activity_continuous_state_t *continuous_state = (iiwa_activity_continuous_state_t *) iiwa_activity->state.computational_state.continuous;
	iiwa_activity_coordination_state_t *coord_state = (iiwa_activity_coordination_state_t *) iiwa_activity->state.coordination_state;  
	perception_rope_t perception_rope = params->perception_rope;
	robot_actuators_t robot_actuators = params->robot_actuators;
	float gain_camera = params->gain_camera; // TBD zie brecht hoe hij data structuur noemt

	// Get goal positions
	vector rope_end_1 = perception_rope.rope_endpoints.rope_end1.point_coordinates;
	vector rope_end_2 = perception_rope.rope_endpoints.rope_end2.point_coordinates;

	// Get inital position
	vector position_gripper_1 = robot_actuators.actuator1.base.point_coordinates;
	vector position_gripper_2 = robot_actuators.actuator2.base.point_coordinates;

	// pair grippers with goals
	double distance_11 = sqrt((goal_1[0]-position_gripper_1[0])^2.0+(goal_1[1]-position_gripper_1[1])^2.0);
	double distance_12 = sqrt((goal_1[0]-position_gripper_2[0])^2.0+(goal_1[1]-position_gripper_2[1])^2.0);
	double distance_21 = sqrt((goal_2[0]-position_gripper_1[0])^2.0+(goal_2[1]-position_gripper_1[1])^2.0);
	double distance_22 = sqrt((goal_2[0]-position_gripper_2[0])^2.0+(goal_2[1]-position_gripper_2[1])^2.0);

	int distance_1 = distance_11-distance_21;
	int distance_2 = distance_12-distance_22;

	vector goal_1, goal_2;

	if (distance_1 == min(distance_1,distance_2)) {
		goal_1 = rope_end_1;
		goal_2 = rope_end_2;
	}
	else {
		goal_1 = rope_end_2;
		goal_2 = rope_end_1;
	}

	while(!(*deinitialisation_request)){
		if (iiwa_activity->lcsm.state == RUNNING){	
			// Read the sensors from iiwa
			pthread_mutex_lock(coord_state->sensor_lock);
			memcpy(state->local_meas_jnt_pos,state->meas_jnt_pos, sizeof(state->local_meas_jnt_pos));
			pthread_mutex_unlock(coord_state->sensor_lock);

			JntArray q(iiwa_robot_kdl.getNrOfJoints());
			JntArray dq(iiwa_robot_kdl.getNrOfJoints());

			q(0) = state->local_meas_jnt_pos[0];
			q(1) = state->local_meas_jnt_pos[1];
			q(2) = state->local_meas_jnt_pos[2];
			q(3) = state->local_meas_jnt_pos[3];
			q(4) = state->local_meas_jnt_pos[4];
			q(5) = state->local_meas_jnt_pos[5];
			q(6) = state->local_meas_jnt_pos[6];
			
			// Pseudo code @BRechts part
			vector position_gripper_1 = robot_actuators.actuator1.base.point_coordinates;
			vector position_gripper_2 = robot_actuators.actuator2.base.point_coordinates;

			float base_velocity = 0.1;  // m/s
			double gain = base_velocity*gain_camera;
			double velocity_gripper_1_x = gain*(goal_1[0]-position_gripper_1[0]);
			double velocity_gripper_1_y = gain*(goal_1[1]-position_gripper_1[1]);


			KDL::Vector cmd_trans_vel = KDL::Vector(vel_gripper_1_x,vel_gripper_1_y,0.0);
			KDL::Vector cmd_rot_vel = KDL::Vector(0.0,0.0,0.0);
			KDL::Twist cmd_tot_vel = KDL::Twist(1,2);
			iksolver.CartToJnt(q, cmd_tot_vel, dq);
			pthread_mutex_lock(&coord_state->actuation_lock);
			params->iiwa_params.cmd_jnt_vel[0] = dq[0];
			params->iiwa_params.cmd_jnt_vel[1] = dq[1];
			params->iiwa_params.cmd_jnt_vel[2] = dq[2];
			params->iiwa_params.cmd_jnt_vel[3] = dq[3];
			params->iiwa_params.cmd_jnt_vel[4] = dq[4];
			params->iiwa_params.cmd_jnt_vel[5] = dq[5];
			params->iiwa_params.cmd_jnt_vel[6] = dq[6];
			pthread_mutex_unlock(&coord_state->actuation_lock);
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
