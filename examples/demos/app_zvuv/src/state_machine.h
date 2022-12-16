
#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

typedef enum {
    idle, //default State = 0
    hand_held_arm_prep, // 1, Was: hand_held_cap_on
    unlocked, // 2
    stopping, // 3
    hand_held_cap_off, //4
    hand_held_armed, // 5, Was: prep_remove_cap
    manual, // 6
    prop_test, // 7
    armed, // 8
    arm_prep, //9
    falling,
    landing,
    hovering
} State;

typedef enum {
    starting, //default Phase1_SubState = 0
    turn_around, // 1
    go_backwards, // 2
    go_straight,  // 3
    paused, // 4
    return_home, // 5
    prep_turn, // 6
} Phase1_SubState;

typedef enum {
    EXP_go, //default RandomExp_SubState = 0
    EXP_turn, // 1
    EXP_straight, //2
} RandomExp_SubState;

typedef enum {
    SM_takeoff_N_hover, // 0
    SM_hover_N_avoid, // 1
    SM_cruise, // 2
    SM_slowing_down, // 3
    SM_turning_around, // 4
    SM_phase_1, // 5
    SM_random_exp, // 6
    SM_guided, // 7
    SM_takeoff_N_stab, // 8 // default SM_State
    SM_stab_N_avoid, // 9
    SM_prep_4_catch, // 10
    SM_catching, // 11
    SM_loco_rtn_home, // 12
    SM_loco_goto_wp, // 13
} SM_State;


#endif   // __STATE_MACHINE_H__
