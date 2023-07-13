/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_interface.cpp
 * @date September 15, 2022
 **/

#include "iiwa_interface.hpp"
#include <friUdpConnection.h>
#include <friClientApplication.h>
#include <iostream>

// Create IIWA Client Application
void iiwa_init(iiwa_state_t *iiwa_state, iiwa_discrete_state_t *iiwa_discrete_state)
{
	UdpConnection *connection = new UdpConnection;
	iiwaClient *client = new iiwaClient(&iiwa_discrete_state->iiwa_current_session_state);
	ClientApplication *app = new ClientApplication(*connection, *client);

	iiwa_state->connection = connection;
	iiwa_state->client = client;
	iiwa_state->app = app;
}

// Clean created memory
void iiwa_state_cleanup(iiwa_state_t *iiwa_state)
{
	delete iiwa_state->app;
	delete iiwa_state->client;
	delete iiwa_state->connection;
}

// Create connection with the robot
bool iiwa_connect(iiwa_params_t *iiwa_params, iiwa_state_t *iiwa_state)
{
	bool command_success;
	printf("Connecting to iiwa robot \n");
	printf("IP: %s , PORT: %d \n", iiwa_params->fri_ip, iiwa_params->fri_port);
	bool connected = iiwa_state->app->connect(iiwa_params->fri_port, iiwa_params->fri_ip);
	// TODO: Add timeout to while statement.
	while(iiwa_state->client->current_session_state != MONITORING_READY){
		command_success = iiwa_state->app->step();
	}
	// Initialize the joint positions to the intial 
	// position of the robot (So you can start it from any configuration)
	iiwa_state->client->getContinousState();

	printf("%f\n", iiwa_state->client->robotState().getSampleTime());
	return true;
}

void iiwa_step(iiwa_state_t *iiwa_state)
{
	// To avoid code lock
	if(iiwa_state->client->current_session_state != IDLE){
		iiwa_state->app->step();
	}
}

bool iiwa_communicate(iiwa_state_t *iiwa_state)
{
	// bool command_success = iiwa_state->app->step();
	if (iiwa_state->client->current_session_state == COMMANDING_ACTIVE) {
		switch(iiwa_state->client->robotState().getClientCommandMode()) {
			case POSITION: { // Position MODE
				for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
				{
					iiwa_state->client->cmd_jnt_pos[i] += iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i]
																* iiwa_state->client->robotState().getSampleTime();
				} 
				break;
			}
			case TORQUE: {   // Torque MODE
				for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
				{
					iiwa_state->client->cmd_torques[i] = iiwa_state->iiwa_actuation_input.cmd_torques[i];
					iiwa_state->client->cmd_jnt_pos[i] += iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i]
																* iiwa_state->client->robotState().getSampleTime();
				}
				break;
			}
			case WRENCH: {   // Wrench MODE
				for (unsigned int i=0;i<CART_VECTOR_DIM;i++)
				{
					iiwa_state->client->cmd_wrench[i] = iiwa_state->iiwa_actuation_input.cmd_wrench[i];
				}
				for (unsigned int i=0;i<LBRState::NUMBER_OF_JOINTS;i++)
				{
					iiwa_state->client->cmd_jnt_pos[i] += iiwa_state->iiwa_actuation_input.cmd_jnt_vel[i]
																* iiwa_state->client->robotState().getSampleTime();
				} 
				break;
			}
			default: {
			}
		}
	}
	iiwa_state->client->getContinousState();
	iiwa_state->client->getDiscreteState();
	
	return true;
}

void iiwa_disconnect(iiwa_state_t *iiwa_state)
{
	if(iiwa_state->client->current_session_state != IDLE){
		iiwa_state->app->disconnect();
	}
}

bool iiwa_check_commanding_mode(iiwa_state_t *iiwa_state, EClientCommandMode desired_command_mode)
{
	if(iiwa_state->client->current_session_state != IDLE){
		iiwa_state->app->step();
		iiwa_state->client->getDiscreteState();
	}

	if(iiwa_state->client->commanding_mode == desired_command_mode){
		return true;
	}
	else{
		return false;
	}
}