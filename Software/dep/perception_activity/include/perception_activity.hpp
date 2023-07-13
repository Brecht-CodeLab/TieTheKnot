/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file perception_activity.hpp
 * @date September 15, 2022
 **/

#ifndef PERCEPTION_ACTIVITY
#define PERCEPTION_ACTIVITY

#include <stdio.h>
#include <iostream>
#include <pthread.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <five_c/activity/activity.h>

using namespace cv;
using namespace std;

enum perception_lcsm_states_s {
  PERCEPTION_,
}perception_lcsm_states_t;

typedef struct perception_activity_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}perception_activity_t;

// Magic numbers
EROSION_TYPE = 0;
EROSION_SIZE = 10;

DILATION_TYPE = 0;
DILATION_SIZE = 10;


// Parameters
typedef struct perception_activity_params_s{
    Mat cameraFrame;
    // pthread_mutex_t store_image_lock;
}perception_activity_params_t;

// Continuous state
typedef struct perception_activity_continuous_state_s{
    Mat cpyFrame;
    long timestamp;
    // perception_lcsm_states_t perception_state;
}perception_activity_continuous_state_t;

//! (computational) discrete state
typedef struct perception_activity_discrete_state_s{
    // bool has_sensor_reading;
}perception_activity_discrete_state_t;

//! Coordination state
typedef struct perception_activity_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool initialization_request;
    bool deinitialisation_request;
    bool waiting_request;
    // Mutex
    pthread_mutex_t store_image_lock;
} perception_activity_coordination_state_t;

extern const perception_activity_t ec_perception_activity;

#endif //PERCEPTION_ACTIVITY
