/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file petrinet_activity.hpp
 * @date December 11, 2022
 **/

#ifndef PETRINET_ACTIVITY_HPP
#define PETRINET_ACTIVITY_HPP

/// Check with team
#include <coordination_libraries/petrinet/petrinet.h> 
#include <five_c/scheduler/eventloop_composition_and_execution/petrinet_scheduler.h>  

// Tracking sources
#define NUMBER_OF_TRACKING_SOURCES 6
#define INITIATE_PERCEPTION 0
#define ROPE_DETECTED_AND_SLIGHTLY_ABOVE_TABLE 1
#define GRIPPERS_ALIGNED 2
#define TOUCHING_TABLE 3
#define ROPE_KNOTTED 4
#define ROPE_TIED 5

// Tracking sinks
#define NUMBER_OF_TRACKING_SINKS 6
#define DETECT_ROPE_AND_GRIPPERS_AND_GO_DOWN 0
#define MOVE_TO_ROPE 1
#define GO_FULLY_DOWN 2
#define KNOT_ROPE 3
#define TIE_ROPE 4
#define PUT_DOWN_ROPE 5

//const char
petrinet_t tieTheKnot_controller_create_petrinet(char *name); 
void tieTheKnot_controller_reset_petrinet(petrinet_t *p);

extern flag_token_conversion_map_t tieTheKnot_controller_petrinet_flag_map;

#endif // IIWA_CONTROLLER_PETRINET_HPP

void tieTheKnot_controller_reset_petrinet(petrinet_t *p) {
        ;
}