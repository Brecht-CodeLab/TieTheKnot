/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: Louis Hanut and Brendan Pousett
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_controller.hpp
 * @date September 15, 2022
 **/

#ifndef iiwa_controller_HPP
#define iiwa_controller_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include <five_c/activity/activity.h>

#include "iiwa_client.hpp"
#include "iiwa_interface.hpp"

// KDL
#include <chain.hpp>
#include <frames_io.hpp>
#include <jntarrayvel.hpp>

// SPDLOG
#include "spdlog/spdlog.h"

using namespace std;

typedef struct iiwa_controller_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_controller_t; 

// Implement ABAG controller
typedef struct abag_params_s{
    double alpha; // "memory" parameter [0,1], the larger, the slower the exp. avg. responds.

    // bias params
    double delta_bias;
    double bias_thresh;

    // gain params
    double delta_gain;
    double gain_thresh;

    double sat_low;
    double sat_high;
}abag_params_t;

typedef struct abag_state_s{
    double ek_bar;
    // Note bias and gain are states which can be initialized as params to some value
    double bias; //bk
    double gain; //gk
    double control; //uk
}abag_state_t;

// Parameters that configure the behaviour of the controller, like gains, motion spec, etc
typedef struct iiwa_controller_params_s{
    EClientCommandMode cmd_mode;
    
    double	goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double	goal_wrench[CART_VECTOR_DIM];

    double  local_goal_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double  local_goal_wrench[CART_VECTOR_DIM];

    // parameters which affect the motion specification
    // double  max_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    // double  slow_jnt_vel[LBRState::NUMBER_OF_JOINTS];
    // double  jnt_accel[LBRState::NUMBER_OF_JOINTS];
    // double  jnt_jerk[LBRState::NUMBER_OF_JOINTS];
    // double  max_jnt_accel[LBRState::NUMBER_OF_JOINTS];
    // double  approach_buffer[LBRState::NUMBER_OF_JOINTS];
    // double  slow_buffer[LBRState::NUMBER_OF_JOINTS];
    // double  goal_buffer[LBRState::NUMBER_OF_JOINTS];

    double max_wrench_step;

    // parameter for bounding the capability of the controller
    double max_torque;
    double max_wrench;

    abag_params_t abag_params;

    // logger implemented by spdlog
    std::shared_ptr<spdlog::logger> logger;
}iiwa_controller_params_t;

// Continuous state which is the state of the controller system, including input and output signals
typedef struct iiwa_controller_continuous_state_s{
    // Input signals from arm sensors (pointers to first elements in array)
    double 	*meas_jnt_pos;
    double 	*meas_torques;
	double 	*meas_ext_torques;

    // Output signals to iiwa 
	double	*cmd_jnt_vel;
	double	*cmd_torques;
	double	*cmd_wrench;

    // Local copies of inputs and outputs will be deep copied with memcpy()
    double  local_meas_jnt_pos[LBRState::NUMBER_OF_JOINTS];
    double  local_meas_torques[LBRState::NUMBER_OF_JOINTS];
	double  local_meas_ext_torques[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_jnt_vel[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_torques[LBRState::NUMBER_OF_JOINTS];
	double	local_cmd_wrench[CART_VECTOR_DIM];

    KDL::JntArray local_q;
    KDL::JntArrayVel local_qd;
    KDL::FrameVel local_cartvel;

    // "State" Parameters which are computed in the activity
    double jnt_pos_prev[LBRState::NUMBER_OF_JOINTS];
    double meas_jnt_vel[LBRState::NUMBER_OF_JOINTS];

    // Data structures for time
    struct timespec prev_timespec;
    struct timespec current_timespec;
    long cycle_time_us; //cycle time in microseconds

    // Data structures for control algorithms
    abag_state_t abag_state;
}iiwa_controller_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_controller_discrete_state_s{
    // flags
    bool in_contact;
}iiwa_controller_discrete_state_t;

//! Coordination state
typedef struct iiwa_controller_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;

    // Mutex
    pthread_mutex_t *sensor_lock, *actuation_lock, goal_lock;

    // First run compute cycle
    bool first_run_compute_cycle;
} iiwa_controller_coordination_state_t;

extern const iiwa_controller_t ec_iiwa_controller;

// sgn function {+1, -1}
template <typename T> int sgn(T val);

// heaviside function
template <typename T> int hside(T val);

// saturation function [sat_low, sat+high]
template <typename T> T saturate(T val, T sat_low, T sat_high);
    
/**
 * Compute the difference in microseconds between two timespecs.
 * 
 * @param *current_timespec from timespec_get()
 * @param *previous_timespec from timespec_get()
 * @return the time difference in microseconds from current-previous. 
*/   
long difftimespec_us(struct timespec *current_timespec, struct timespec *prev_timespec);

/**
 * ABAG Controller, implementation similar to:
 * 
 * "Adaptive Closed-loop Speed Control of BLDC Motors with Applications to Multi-rotor Aerial Vehicles"
 * by Franchi and Mallet. 
 * 
 * Note: the sign of the error has been reversed. Instead of val-setpoint, I use setpoint-val. 
 * This gives a positive control signal uk when the value is less than the setpoint.
*/
void abag(abag_params_t *params, abag_state_t *state, double val, double setpoint);

double compute_velocity(double meas_jnt_pos, double prev_jnt_pos, double cycle_time);

#endif //iiwa_controller_HPP