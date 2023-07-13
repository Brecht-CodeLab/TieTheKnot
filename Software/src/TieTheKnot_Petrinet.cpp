/* ----------------------------------------------------------------------------
 * Project Title,
 * ROB @ KU Leuven, Leuven, Belgium
 * Authors: 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file iiwa_controller_petrinet.cpp
 * @date November 24, 2022
 **/

// Q: Not in directory? --> check with team
#include "iiwa/iiwa_controller_petrinet.hpp"

// Tracking sources
char* tieTheKnot_controller_tracking_source_names[] = {//const
    "rope_detected", 
    "grippers_aligned", 
    "holding_rope", 
    "rope_knotted",
    "rope_tied" 

};
bool* controller_tracking_source_flags[NUMBER_OF_TRACKING_SOURCES];

// Tracking sinks
char* tieTheKnot_controller_tracking_sink_names[] = {//const
    "move_to_rope", // image based virtual servoing
    "grab_rope", // impedance control
    "knot_rope", // position control
    "tie_rope" // impedance control
    "put_down_rope" // position control
};

bool* controller_tracking_sink_flags[NUMBER_OF_TRACKING_SINKS];

flag_token_conversion_map_t tieTheKnot_controller_petrinet_flag_map = {
        .converting_sources = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sources = {
                .names = tieTheKnot_controller_tracking_source_names,
                .flags = tieTheKnot_controller_tracking_source_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SOURCES
        },
        .converting_sinks = {
                .names = NULL,
                .flags = NULL,
                .number_of_flags = 0
        },
        .tracking_sinks = {
                .names = tieTheKnot_controller_tracking_sink_names,
                .flags = tieTheKnot_controller_tracking_sink_flags,
                .number_of_flags = NUMBER_OF_TRACKING_SINKS
        }
};

petrinet_t iiwa_controller_create_petrinet(char *name) {
    petrinet_t *p = init_petrinet(name);

    place_t *p_rope_detected = create_place(p, controller_tracking_source_names[ROPE_DETECTED]);
    place_t *p_rope_aligned_with_grippers = create_place(p, controller_tracking_source_names[grippers_aligned]);
    place_t *p_holding_rope = create_place(p, controller_tracking_source_names[HOLDING_ROPE]);
    place_t *p_rope_knotted = create_place(p, controller_tracking_source_names[ROPE_KNOTTED]);
    place_t *p_rope_tied = create_place(p, controller_tracking_source_names[ROPE_TIED]);

    place_t *p_move_to_rope = create_place(p, controller_tracking_sink_names[MOVE_TO_ROPE]);
    place_t *p_grab_rope = create_place(p, controller_tracking_sink_names[GRAB_ROPE]);
    place_t *p_knot_rope = create_place(p, controller_tracking_sink_names[KNOT_ROPE]);
    place_t *p_tie_rope = create_place(p, controller_tracking_sink_names[TIE_ROPE]);
    place_t *p_put_down_rope = create_place(p, controller_tracking_sink_names[PUT_DOWN_ROPE]);

    // Q: What is this?
    transition_behaviour_t b1 = {
            .condition = cond_Black1,
            .consumption_behaviour=consume_Black1,
            .production_behaviour=produce_Black1
    }; 

    // TRANSITIONS
    transition_t *t1 = create_transition(p, "t1");
    add_behaviour(t1, &b1);

    agedge(p, p_rope_detected, t1, "1", TRUE);
    agedge(p, t1, p_move_to_rope, "1", TRUE);

    transition_t *t2 = create_transition(p, "t2");
    add_behaviour(t2, &b1);

    agedge(p, p_move_to_rope, t2, "1", TRUE);
    agedge(p, p_rope_aligned_with_grippers, t2, "1", TRUE);
    agedge(p, t2, p_enter_blend_model, "1", TRUE);

    transition_t *t3 = create_transition(p, "t3");
    add_behaviour(t3, &b1);
    
    agedge(p, p_enter_blend_model, t3, "1", TRUE);
    agedge(p, p_end_vel_transition, t3, "1", TRUE);
    agedge(p, t3, p_enter_slow_motion, "1", TRUE);

    transition_t *t4 = create_transition(p, "t4");
    add_behaviour(t4, &b1);
    
    agedge(p, p_enter_slow_motion, t4, "1", TRUE);
    agedge(p, p_contact_detected, t4, "1", TRUE);
    agedge(p, t4, p_terminate_net, "1", TRUE);
 
    return *p;
}

void iiwa_controller_reset_petrinet(petrinet_t *p) {
        ;
}
