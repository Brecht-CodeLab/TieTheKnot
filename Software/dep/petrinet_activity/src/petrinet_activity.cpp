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

#include  <petrinet_activity.hpp>

// Tracking sources
const char* tieTheKnot_controller_tracking_source_names[] = {//const
    "initiate_perception",
    "rope_detected_and_sligthly_above_table", 
    "grippers_aligned", 
    "touching_table",
    "rope_knotted",
    "rope_tied"
};
bool* controller_tracking_source_flags[NUMBER_OF_TRACKING_SOURCES];

// Tracking sinks
const char* tieTheKnot_controller_tracking_sink_names[] = {//const
    "detect_rope_and_grippers_and_go_down" // 
    "move_to_rope", // image based visual servoing (speed control)
    "go_fully_down", // force control
    "knot_rope", // TBD
    "tie_rope" // force control
    "put_down_rope" // position control
};
bool* tieTheKnot_controller_tracking_sink_flags[NUMBER_OF_TRACKING_SINKS];

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

petrinet_t tieTheKnot_controller_create_petrinet(char *name) {
    petrinet_t *p = init_petrinet(name);

    place_t *p_initiate_perception = create_place(p, controller_tracking_source_names[INITIATE_PERCEPTION]);
    place_t *p_rope_detected_and_sligthly_above_table = create_place(p, controller_tracking_source_names[ROPE_DETECTED_AND_SLIGHTLY_ABOVE_TABLE]);
    place_t *p_grippers_aligned = create_place(p, controller_tracking_source_names[GRIPPERS_ALIGNED]);
    place_t *p_touching_table = create_place(p, controller_tracking_source_names[TOUCHING_TABLE]);
    place_t *p_rope_knotted = create_place(p, controller_tracking_source_names[ROPE_KNOTTED]);
    place_t *p_rope_tied = create_place(p, controller_tracking_source_names[ROPE_TIED]);

    place_t *p_detect_rope_and_grippers_and_go_down = create_place(p, controller_tracking_sink_names[DETECT_ROPE_AND_GRIPPERS_AND_GO_DOWN]);
    place_t *p_move_to_rope = create_place(p, controller_tracking_sink_names[MOVE_TO_ROPE]);
    place_t *p_go_fully_down = create_place(p, controller_tracking_sink_names[GO_FULLY_DOWN]);
    place_t *p_knot_rope = create_place(p, controller_tracking_sink_names[KNOT_ROPE]);
    place_t *p_tie_rope = create_place(p, controller_tracking_sink_names[TIE_ROPE]);
    place_t *p_put_down_rope = create_place(p, controller_tracking_sink_names[PUT_DOWN_ROPE]);


    transition_behaviour_t b1 = {
            .condition = cond_Black1,
            .consumption_behaviour=consume_Black1,
            .production_behaviour=produce_Black1
    }; 

    // TRANSITIONS
    transition_t *t1 = create_transition(p, "t1");
    add_behaviour(t1, &b1);

    agedge(p, p_initiate_perception, t1, "1", TRUE);
    agedge(p, t1, p_detect_rope_and_grippers_and_go_down, "1", TRUE);


    transition_t *t2 = create_transition(p, "t2");
    add_behaviour(t2, &b1);

    agedge(p, p_detect_rope_and_grippers_and_go_down, t2, "1", TRUE);
    agedge(p, p_rope_detected_and_sligthly_above_table, t2, "1", TRUE);
    agedge(p, t2, p_move_to_rope, "1", TRUE);

    transition_t *t3 = create_transition(p, "t3");
    add_behaviour(t3, &b1);

    agedge(p, p_move_to_rope, t3, "1", TRUE);
    agedge(p, p_grippers_aligned, t3, "1", TRUE);
    agedge(p, t3, p_go_fully_down, "1", TRUE);

    transition_t *t4 = create_transition(p, "t4");
    add_behaviour(t4, &b1);
    
    agedge(p, p_go_fully_down, t4, "1", TRUE);
    agedge(p, p_touching_table, t4, "1", TRUE);
    agedge(p, t4, p_knot_rope, "1", TRUE);

    transition_t *t5 = create_transition(p, "t5");
    add_behaviour(t5, &b1);
    
    agedge(p, p_knot_rope, t5, "1", TRUE);
    agedge(p, p_rope_knotted, t5, "1", TRUE);
    agedge(p, t5, p_tie_rope, "1", TRUE);


    transition_t *t6 = create_transition(p, "t6");
    add_behaviour(t6, &b1);
    
    agedge(p, p_tie_rope, t6, "1", TRUE);
    agedge(p, p_rope_tied, t6, "1", TRUE);
    agedge(p, t6, p_put_down_rope, "1", TRUE);

    return *p;
}

void tieTheKnot_controller_reset_petrinet(petrinet_t *p) {
        ;
}

int main() {

        return 0;
}


