/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_interface.hpp
 * @date September 15, 2022
 **/

#ifndef IIWA_INTERFACE_HPP
#define IIWA_INTERFACE_HPP

#include "iiwa_client.hpp"
#include <friUdpConnection.h>
#include <friClientApplication.h>

#include <vector>
#include <string>
#include <tuple>
using namespace std;

using namespace KUKA::FRI;

static const int CART_VECTOR_DIM = 6;

typedef struct iiwa_state_s{
	struct iiwa_actuation_input_s{
		double 			cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];
		double 			cmd_torques[LBRState::NUMBER_OF_JOINTS];
		double 			cmd_wrench[CART_VECTOR_DIM];
	}iiwa_actuation_input;

	struct iiwa_sensors_s{
		double 			meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
		double 			meas_torques[LBRState::NUMBER_OF_JOINTS];
		double 			meas_ext_torques[LBRState::NUMBER_OF_JOINTS];
	}iiwa_sensors;

	iiwaClient         	*client;
	UdpConnection     	*connection;
	ClientApplication	*app;

}iiwa_state_t;

typedef struct iiwa_discrete_state_s{
	ESessionState			iiwa_current_session_state;
	EClientCommandMode		iiwa_commanding_mode;
	EConnectionQuality		iiwa_connection_quality;
}iiwa_discrete_state_t;

typedef struct iiwa_params_s{
	char					fri_ip[20];
	unsigned int			fri_port;
	EClientCommandMode		cmd_mode;
	double 					cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];
	double 					cmd_torques[LBRState::NUMBER_OF_JOINTS];
	double 					cmd_wrench[CART_VECTOR_DIM];
}iiwa_params_t;

void iiwa_init(iiwa_state_t *iiwa_state, iiwa_discrete_state_t *iiwa_discrete_state);
void iiwa_state_cleanup(iiwa_state_t *iiwa_state);
bool iiwa_connect(iiwa_params_t *iiwa_params, iiwa_state_t *iiwa_state);
void iiwa_disconnect(iiwa_state_t *iiwa_state);
bool iiwa_communicate(iiwa_state_t *iiwa_state);
void iiwa_step(iiwa_state_t *iiwa_state);
bool iiwa_check_commanding_mode(iiwa_state_t *iiwa_state, EClientCommandMode desired_command_mode);


#endif //IIWA_INTERFACE_HPP