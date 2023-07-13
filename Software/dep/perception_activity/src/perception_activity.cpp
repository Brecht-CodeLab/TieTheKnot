

/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file perception_activity.c
 * @date Ocotber 12, 2021
 **/

#include "string.h"
#include <time.h>

#include <perception_activity.hpp>

/** 
 * The config() has to be scheduled everytime a change in the LCSM occurs, 
 * so it properly configures the schedule for the next iteration according
 * to the LCSM state, resources, task, ..  
 * @param[in] activity data structure for the perception activity
*/
void perception_activity_config(activity_t* activity){
    // Remove config() from the eventloop schedule in the next iteration
    remove_schedule_from_eventloop(&activity->schedule_table, "activity_config");
    // Deciding which schedule to add
    switch (activity->lcsm.state){
        case CREATION:
            add_schedule_to_eventloop(&activity->schedule_table, "creation");
            break;
        case RESOURCE_CONFIGURATION:
            add_schedule_to_eventloop(&activity->schedule_table, "resource_configuration");
            break;
        case PAUSING:
            add_schedule_to_eventloop(&activity->schedule_table, "pausing");
            break;
        case RUNNING:
            add_schedule_to_eventloop(&activity->schedule_table, "running");
            break;
        case CLEANING:
            break;
        case DONE:
            break;
    }
};

// Creation
void perception_activity_creation_coordinate(activity_t *activity){
    perception_activity_coordination_state_t * coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
    if (coord_state->deinitialisation_request)
        activity->state.lcsm_protocol = DEINITIALISATION;

    // Coordinating own activity
    if (activity->state.lcsm_flags.creation_complete)
        activity->lcsm.state = RESOURCE_CONFIGURATION;
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void perception_activity_creation_configure(activity_t *activity){
    if (activity->lcsm.state != CREATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "creation");
    }
}

void perception_activity_creation_compute(activity_t *activity){
    perception_activity_continuous_state_t * continuous_state = (perception_activity_continuous_state_t *) activity->state.computational_state.continuous;
    // continuous_state->perception_state.camera_frame = 0;
    activity->state.lcsm_flags.creation_complete = true;
}

void perception_activity_creation(activity_t *activity){
    perception_activity_creation_compute(activity);
    perception_activity_creation_coordinate(activity);
    perception_activity_creation_configure(activity);
}


// Resource configuration
void perception_activity_resource_configuration_coordinate(activity_t *activity){
    perception_activity_params_t* params = (perception_activity_params_t *) activity->conf.params;
    perception_activity_continuous_state_t* continuous_state = (perception_activity_continuous_state_t *) activity->state.computational_state.continuous;
    perception_activity_coordination_state_t *coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;

    // if (++continuous_state->perception_state.number_of_connection_attempts >= 
    //     params->perception_params.max_number_of_connection_attempts){
    //     activity->state.lcsm_flags.resource_configuration_complete = true;  
    //     activity->state.lcsm_protocol = DEINITIALISATION;
    // }

    // Internal coordination
    if (activity->state.lcsm_flags.resource_configuration_complete){
        switch (activity->state.lcsm_protocol){ 
            case INITIALISATION:
                activity->lcsm.state = PAUSING;
                break;
            case EXECUTION:
                activity->lcsm.state = RUNNING;
                break;
            case DEINITIALISATION:
                activity->lcsm.state = DONE;
                activity->state.lcsm_flags.deletion_complete = true;
                break;
        }
        update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
    }
}

void perception_activity_resource_configuration_configure(activity_t *activity){
    if (activity->lcsm.state != RESOURCE_CONFIGURATION){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "resource_configuration");
        // Update flags for next visit to the resource configuration LCS 
        activity->state.lcsm_flags.resource_configuration_complete = false;
    }
}

void perception_activity_resource_configuration_compute(activity_t *activity){
    perception_activity_params_t* params = (perception_activity_params_t *) activity->conf.params;
    perception_activity_continuous_state_t* continuous_state = (perception_activity_continuous_state_t *) activity->state.computational_state.continuous;
    perception_activity_coordination_state_t* coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;
    Mat *original_image = &params->camera_frame;
    Mat *original_cpy = &continuous_state->original_cpy_camera_frame;
  
    switch (activity->state.lcsm_protocol){
        case INITIALISATION:
            // Copying to a buffer to be shared
            pthread_mutex_lock(&coord_state->store_image_lock);
            original_image->copyTo(* original_cpy);
            pthread_mutex_unlock(&coord_state->store_image_lock);
            break;
        case DEINITIALISATION:
            activity->state.lcsm_flags.resource_configuration_complete = true;
            break;
    }
}

void perception_activity_resource_configuration(activity_t *activity){
    perception_activity_resource_configuration_compute(activity);
    perception_activity_resource_configuration_coordinate(activity);
    perception_activity_resource_configuration_configure(activity);
}

// Pausing
void perception_activity_pausing_coordinate(activity_t *activity){
    perception_activity_coordination_state_t * coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
    if (coord_state->execution_request)
        activity->state.lcsm_protocol = EXECUTION;
    if (coord_state->deinitialisation_request)
        activity->state.lcsm_protocol = DEINITIALISATION;

    // Coordinating own activity
    switch (activity->state.lcsm_protocol){ 
        case EXECUTION:
            activity->lcsm.state = RUNNING;
            break;
        case DEINITIALISATION:
            activity->lcsm.state = RESOURCE_CONFIGURATION;
            break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void perception_activity_pausing_configure(activity_t *activity){
    if (activity->lcsm.state != PAUSING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "pausing");
    }
}

void perception_activity_pausing(activity_t *activity){
    perception_activity_pausing_coordinate(activity);
    perception_activity_pausing_configure(activity);
}

// Running
void perception_activity_running_communicate(activity_t *activity){
    perception_activity_continuous_state_t *continuous_state = (perception_activity_continuous_state_t *) activity->state.computational_state.continuous;
    perception_activity_coordination_state_t *coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;
    perception_activity_params_t *params = (perception_activity_params_t *) activity->conf.params;
    
    //==================================//
    // COPY FRAME
    //==================================// 

    Mat frameOriginal = params->cameraFrame.clone();
    if (frameOriginal.empty())
    {
        std::cout << "ERROR: frame is empty\n" << std::endl;
    }

    //==================================//
    // ROPE DETECTION
    //==================================// 
    
    // Gaussian blur
    Mat frameGaussian;
    int gaussSize = 3;
    GaussianBlur(frameOriginal,frameGaussian, Size(gaussSize,gaussSize),0);

    // BGR to HSV
    Mat frameHSV;
    cvtColor(frameGaussian,frameHSV, COLOR_BGR2HSV);

    // Color detection
    // Red (rope)
    Mat redFiltered1,redFiltered2,frameRedFiltered;
    inRange(frameGaussian, Scalar(0,40,45), Scalar(10, 255, 255), redFiltered1);
    inRange(frameGaussian, Scalar(160,40,45), Scalar(180, 255, 255), redFiltered2);
    bitwise_or(redFiltered1, redFiltered2, frameRedFiltered);
    // imshow("Red Filtered", frameRedFilteredResult);

    // Blue (endpoints of rope)
    Mat frameBlueFiltered;
    inRange(frameGaussian, Scalar(80,7,10), Scalar(140, 255, 255), frameBlueFiltered);

    // Make image Grayscale
    Mat frameGray;
    cvtColor(frameGaussian,frameGray, COLOR_BGR2GRAY);

    // Otsu
    Mat frameBinary;
    threshold(frameGray, frameBinary, 127, 255, THRESH_BINARY);

    // Erosion
    Mat frameEroded;
    Mat elementErosion = getStructuringElement(EROSION_TYPE,
                                        Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
                                        Point(EROSION_SIZE, EROSION_SIZE));
    erode(frameBinary, frameEroded, elementErosion);

    // Dilation
    Mat frameDilated;
    Mat elementDilation = getStructuringElement(DILATION_TYPE,
                                        Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1),
                                        Point(DILATION_SIZE, DILATION_SIZE));
    dilate(frameErorded, frameDilated, elementDilation);

    // Bitwise operations
    // --> Otsu AND ColorDetection() AND NOT(Dilated(Eroded(Foreground(IMG))))
    Mat detectedRopeFrame;
    

    activity->state.lcsm_flags.resource_configuration_complete = true;
}

void perception_activity_running_coordinate(activity_t *activity){
    perception_activity_coordination_state_t *coord_state = (perception_activity_coordination_state_t *) activity->state.coordination_state;
    // Coordinating with other activities
    if (coord_state->deinitialisation_request)
        activity->state.lcsm_protocol = DEINITIALISATION;

    switch (activity->state.lcsm_protocol){ 
        case DEINITIALISATION:
            activity->lcsm.state = RESOURCE_CONFIGURATION;
            break;
    }
    update_super_state_lcsm_flags(&activity->state.lcsm_flags, activity->lcsm.state);
}

void perception_activity_running_configure(activity_t *activity){
    if (activity->lcsm.state != RUNNING){
        // Update schedule
        add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
        remove_schedule_from_eventloop(&activity->schedule_table, "running");
    }
}

void perception_activity_running(activity_t *activity){
    perception_activity_running_communicate(activity);
    perception_activity_running_coordinate(activity);
    perception_activity_running_configure(activity);
}

// SCHEDULER 
void perception_activity_register_schedules(activity_t *activity){
    schedule_t schedule_config = {.number_of_functions = 0};
    register_function(&schedule_config, (function_ptr_t) perception_activity_config, 
        activity, "activity_config");
    register_schedule(&activity->schedule_table, schedule_config, "activity_config");

    schedule_t schedule_creation = {.number_of_functions = 0};
    register_function(&schedule_creation, (function_ptr_t) perception_activity_creation, 
        activity, "creation");
    register_schedule(&activity->schedule_table, schedule_creation, "creation");

    schedule_t schedule_resource_configuration = {.number_of_functions = 0};
    register_function(&schedule_resource_configuration, (function_ptr_t) perception_activity_resource_configuration, 
        activity, "resource_configuration");
    register_schedule(&activity->schedule_table, schedule_resource_configuration, 
        "resource_configuration");

    schedule_t schedule_pausing = {.number_of_functions = 0};
    register_function(&schedule_pausing, (function_ptr_t) perception_activity_pausing, 
        activity, "pausing");
    register_schedule(&activity->schedule_table, schedule_pausing, 
        "pausing");

    schedule_t schedule_running = {.number_of_functions = 0};
    register_function(&schedule_running, (function_ptr_t) perception_activity_running, 
        activity, "running");
    register_schedule(&activity->schedule_table, schedule_running, 
        "running");

}

void perception_activity_create_lcsm(activity_t *activity, const char* name_algorithm){
    activity->conf.params = malloc(sizeof(perception_activity_params_t));
    activity->state.computational_state.continuous = malloc(sizeof(perception_activity_continuous_state_t));
    activity->state.computational_state.discrete = malloc(sizeof(perception_activity_discrete_state_t));
    activity->state.coordination_state = malloc(sizeof(perception_activity_coordination_state_t));
}

void perception_activity_resource_configure_lcsm(activity_t *activity){
    resource_configure_lcsm_activity(activity);
    // Select the inital state of LCSM for this activity
    activity->lcsm.state = CREATION;
    activity->state.lcsm_protocol = INITIALISATION;

    // Schedule table (adding config() for the first eventloop iteration)
    perception_activity_register_schedules(activity);
    add_schedule_to_eventloop(&activity->schedule_table, "activity_config");
}

void perception_activity_destroy_lcsm(activity_t *activity){
    destroy_activity(activity);
}

const perception_activity_t ec_perception_activity ={
    .create_lcsm = perception_activity_create_lcsm,
    .resource_configure_lcsm = perception_activity_resource_configure_lcsm,
    .destroy_lcsm = perception_activity_destroy_lcsm,
};

int main() {
    return 0;
}