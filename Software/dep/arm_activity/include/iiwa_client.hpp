/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_client.hpp
 * @date September 15, 2022
 **/

#ifndef _IIWA_CLIENT_H
#define _IIWA_CLIENT_H

#include <friLBRClient.h>

/**
 * \brief Template client implementation.
 */
using namespace KUKA::FRI;

class iiwaClient : public LBRClient
{
	
public:
		
	/**
	 * \brief Constructor.
	 */
	iiwaClient(ESessionState *iiwa_sessionS);
	
	/** 
	 * \brief Destructor.
	 */
	~iiwaClient();
	
	/**
	 * \brief Callback for FRI state changes.
	 * 
	 * @param oldState
	 * @param newState
	 */
	virtual void onStateChange(ESessionState oldState, ESessionState newState);
	
	/**
	 * \brief Callback for the FRI session states 'Monitoring Wait' and 'Monitoring Ready'.
	 * 
	 * If you do not want to change the default-behavior, you do not have to implement this method.
	 */
	virtual void monitor();
	
	/**
	 * \brief Callback for the FRI session state 'Commanding Wait'.
	 * 
	 * If you do not want to change the default-behavior, you do not have to implement this method.
	 */
	virtual void waitForCommand();
	
	/**
	 * \brief Callback for the FRI state 'Commanding Active'.
	 * 
	 * If you do not want to change the default-behavior, you do not have to implement this method.
	 */
	virtual void command();

	// 
	// ESessionState *_current_state;
	static const int CART_VECTOR_DIM = 6;     //!< number of elements in a Cartesian vector
    double meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double meas_torques[LBRState::NUMBER_OF_JOINTS];
    double meas_ext_torques[LBRState::NUMBER_OF_JOINTS];
    double cmd_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double cmd_torques[LBRState::NUMBER_OF_JOINTS];
    double cmd_wrench[CART_VECTOR_DIM];
	ESessionState			current_session_state;
	EClientCommandMode		commanding_mode;
	EConnectionQuality		connection_quality;

		// Sensor reading
	void getContinousState();
	// void getJointPosition();
    // void getJointEffort();
    // void getJointExtEffort();

	// Discrete state
	void getDiscreteState();
	
};

#endif // _IIWA_CLIENT_H
