/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_client.cpp
 * @date September 15, 2022
 **/
#include <cstdio>
#include "iiwa_client.hpp"

#include <cstring>
//******************************************************************************
iiwaClient::iiwaClient(ESessionState *iiwa_sessionS)
{	
	// current_session_state = iiwa_sessionS;
	// memcpy(cmd_jnt_pos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	for(unsigned i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
		cmd_jnt_pos[i] = 0.0;
	
	for(unsigned i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
		cmd_torques[i] = 0.0;
	
	for(unsigned i=0; i<CART_VECTOR_DIM; i++)
		cmd_wrench[i] = 0.0;
}

//******************************************************************************
iiwaClient::~iiwaClient()
{
}
      
//******************************************************************************
void iiwaClient::onStateChange(ESessionState oldState, ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	// react on state change events
	switch (newState)
	{
		case MONITORING_WAIT:
		{
			current_session_state = newState;
			break;
		}       
		case MONITORING_READY:
		{
			this->getContinousState();
			memcpy(cmd_jnt_pos, meas_jnt_pos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			current_session_state = newState;
			break;
		}
		case COMMANDING_WAIT:
		{
			this->getContinousState();
			memcpy(cmd_jnt_pos, meas_jnt_pos, LBRState::NUMBER_OF_JOINTS * sizeof(double));
			current_session_state = newState;
			break;
		}   
		case COMMANDING_ACTIVE:
		{
			current_session_state = newState;
			break;
		}
		default:
		{
			current_session_state = newState;
			break;
		}
	}
}

//******************************************************************************
void iiwaClient::monitor()
{
	LBRClient::monitor();

	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/

}

//******************************************************************************
void iiwaClient::waitForCommand()
{
	// In waitForCommand(), the joint values have to be mirrored. Which is done, 
	// by calling the base method.

	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/ 

	// I don't think this is needed
	if (robotState().getClientCommandMode() == POSITION) {
		robotCommand().setJointPosition(cmd_jnt_pos);
	} 
	else if (robotState().getClientCommandMode() == TORQUE) {
		// memcpy(cmd_jnt_pos, this->robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
		// robotCommand().setJointPosition(cmd_jnt_pos);
		// LBRClient::waitForCommand();
		robotCommand().setJointPosition(cmd_jnt_pos);
		robotCommand().setTorque(cmd_torques);
	} 
	else if (robotState().getClientCommandMode() == WRENCH) {
		// LBRClient::waitForCommand();
		robotCommand().setJointPosition(cmd_jnt_pos);
		robotCommand().setWrench(cmd_wrench);
	}
}

//******************************************************************************
void iiwaClient::command()
{
	/***************************************************************************/
	/*                                                                         */
	/*   Place user Client Code here                                           */
	/*                                                                         */
	/***************************************************************************/   

	// In command(), the joint angle values have to be set. 
	//robotCommand().setJointPosition( newJointValues );
	// LBRClient::command();

	if (robotState().getClientCommandMode() == POSITION) {
		robotCommand().setJointPosition(cmd_jnt_pos);
	} 
	else if (robotState().getClientCommandMode() == TORQUE) {
		// printf("Commanding torque %f, %f, %f, %f, %f, %f, %f \n", 
		// 					cmd_torques[0], cmd_torques[1], cmd_torques[2], 
		// 					cmd_torques[3], cmd_torques[4], cmd_torques[5], cmd_torques[6]);
		// memcpy(cmd_jnt_pos, this->robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
		// LBRClient::command();
		robotCommand().setJointPosition(cmd_jnt_pos);
		robotCommand().setTorque(cmd_torques);
	}
	else if (robotState().getClientCommandMode() == WRENCH) {
		// printf("Commanding wrench %f, %f, %f, %f, %f, %f, %f \n", 
		// 					cmd_jnt_pos[0], cmd_jnt_pos[1], cmd_jnt_pos[2], 
		// 					cmd_jnt_pos[3], cmd_jnt_pos[4], cmd_jnt_pos[5], cmd_jnt_pos[6]);
		// LBRClient::command();
		robotCommand().setJointPosition(cmd_jnt_pos);
		robotCommand().setWrench(cmd_wrench);
	}
}

void iiwaClient::getContinousState() {
	memcpy(meas_jnt_pos,this->robotState().getMeasuredJointPosition(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
	memcpy(meas_torques,this->robotState().getMeasuredTorque(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
	memcpy(meas_ext_torques,this->robotState().getExternalTorque(),LBRState::NUMBER_OF_JOINTS * sizeof(double));
}

void iiwaClient::getDiscreteState(){
	commanding_mode = this->robotState().getClientCommandMode();
	current_session_state = this->robotState().getSessionState();
	connection_quality = this->robotState().getConnectionQuality();
}