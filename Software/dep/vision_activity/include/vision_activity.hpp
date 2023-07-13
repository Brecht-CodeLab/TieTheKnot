/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file vision_activity.hpp
 * @date September 15, 2022
 **/

#ifndef VISION_ACTIVITY
#define VISION_ACTIVITY

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

enum vision_lcsm_states_s {
  VISION_INIT, 
  VISION_WAIT, 
  VISION_DEINIT, 
  VISION_RUN
}vision_lcsm_states_t;

typedef struct vision_activity_s{
	void (*create_lcsm)(activity_t*, const char* name_activity);
	void (*resource_configure_lcsm)(activity_t*);
	void (*destroy_lcsm)(activity_t*);
}vision_activity_t;


// Parameters
typedef struct vision_activity_params_s{
  bool capture_frequency; //in milliseconds
  // char configuration_file[120];
  int device_port;
  // int connect_baudrate;
  // int max_number_of_connection_attempts;
  // bool has_values;
  // int camera_address = 0;
}vision_activity_params_t;

// Continuous state
typedef struct vision_activity_continuous_state_s{
    VideoCapture cap;
    Mat camera_frame;
    long timestamp;
    // vision_lcsm_states_t vision_state;
}vision_activity_continuous_state_t;

//! (computational) discrete state
typedef struct vision_activity_discrete_state_s{
    bool has_sensor_reading;
}vision_activity_discrete_state_t;

//! Coordination state
typedef struct vision_activity_coordination_state_s {
    // Activity LCS
    bool execution_request;
    bool initialization_request;
    bool deinitialisation_request;
    bool waiting_request;
    // Mutex
    pthread_mutex_t store_image_lock;
} vision_activity_coordination_state_t;

extern const vision_activity_t ec_vision_activity;

#endif //VISION_ACTIVITY
