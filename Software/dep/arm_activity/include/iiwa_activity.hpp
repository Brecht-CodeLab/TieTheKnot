/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file iiwa_activity.hpp
 * @date September 15, 2022
 **/

#ifndef IIWA_ACTIVITY_HPP
#define IIWA_ACTIVITY_HPP

#include <stdio.h>
#include <pthread.h>

// ACCAL
#include <five_c/activity/activity.h>

// Client
#include "iiwa_interface.hpp"

using namespace std;

typedef struct iiwa_activity_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}iiwa_activity_t; 

// Parameters
typedef struct iiwa_activity_params_s{
    iiwa_params_t               iiwa_params;
}iiwa_activity_params_t;

// Continuous state
typedef struct iiwa_activity_continuous_state_s{
    iiwa_state_t              iiwa_state;
}iiwa_activity_continuous_state_t;

//! (computational) discrete state
typedef struct iiwa_activity_discrete_state_s{
    iiwa_discrete_state_t   iiwa_discrete_state;
}iiwa_activity_discrete_state_t;

//! Coordination state
typedef struct iiwa_activity_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool deinitialisation_request;
    bool commanding_not_active;
    // Mutex
    pthread_mutex_t sensor_lock, actuation_lock;
} iiwa_activity_coordination_state_t;

extern const iiwa_activity_t ec_iiwa_activity;
#endif //IIWA_ACTIVITY_HPP